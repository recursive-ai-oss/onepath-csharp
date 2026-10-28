using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Threading;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// A built mesh together with what it takes to change it at run time: for every tile the compact heightfield after the geometry-only stages (filters and
/// compaction), the temporary obstacles and area volumes currently in force, and a queue of tiles waiting to be rebuilt.
/// Adding or removing an obstacle or volume only queues the tiles whose build grid it touches; <see cref="Update"/> rebuilds them from the cached data
/// (not from the geometry), a fixed number of tiles at a time, and swaps the new tiles into <see cref="Mesh"/>, so queries and corridors see
/// them at once. The queue is in a fixed order (first queued, first rebuilt; one operation queues its tiles by tile row, then column), and a tile is always
/// rebuilt from the obstacles and volumes in force when it is processed, so the mesh after all queued work is done does not depend on how the work was sliced.
/// The rules are in <c>docs/build3d-runtime.md</c>.
/// </summary>
public sealed class TileCache
{
    private const uint Magic = 0x4B54_4E4F; // "ONTK" little-endian
    private const ushort StateVersion = 1;
    private const int MaxObstacles = 1 << 20;

    private readonly Mesh3DPipeline _pipeline;
    private readonly SortedDictionary<long, Entry> _entries = new();
    private readonly List<(int Id, TileObstacle Obstacle)> _obstacles = new();
    private readonly List<(int Id, ConvexVolume Volume)> _volumes = new();
    private readonly List<long> _queue = new();
    private readonly HashSet<long> _queued = new();
    private readonly TileRef[] _refs = new TileRef[NavMeshFormat.MaxLayer + 1];
    private readonly List<TileObstacle> _obstacleList = new();
    private readonly List<ConvexVolume> _volumeList = new();
    private int _nextObstacle = 1;
    private int _nextVolume = 1;
    private TiledNavMesh? _mesh;

    private sealed class Entry
    {
        public Entry(int tx, int tz, CompactHeightfield chf, int[]? layers)
        {
            Layers = layers;
            TileX = tx;
            TileZ = tz;
            Chf = chf;
        }

        public Entry(LeanTile lean)
        {
            Lean = lean;
            TileX = lean.TileX;
            TileZ = lean.TileZ;
            Chf = lean.Chf;
        }

        /// <summary>The tile split into layers, each rebuilt on its own (see <c>docs/build3d-runtime.md</c>), or null for the general path.</summary>
        public LeanTile? Lean { get; }

        public int TileX { get; }

        /// <summary>The layer of every span (set with <see cref="PartitionType.Layers"/>), decided at the first build; null for the other partitions.</summary>
        public int[]? Layers { get; }

        public int TileZ { get; }

        /// <summary>The eroded compact heightfield with the pipeline's own volumes applied: what every rebuild starts from.</summary>
        public CompactHeightfield Chf { get; }

        /// <summary>
        /// Area ids of the spans the tile in the mesh was last built from (null after a restore, when the mesh may hold anything). A rebuild that
        /// marks the same areas builds the same tile, so it is skipped.
        /// </summary>
        public byte[]? LastAreas { get; set; }

        /// <summary>Layers the last build produced, or -1 when the tile is still the initial build and the count was not kept.</summary>
        public int LastLayers { get; set; } = -1;

        /// <summary>Area ids of the initial build, and the tile data it produced: a rebuild that ends with these areas again (every obstacle over the tile gone) reuses the data.</summary>
        public byte[]? BaseAreas { get; set; }

        /// <summary>See <see cref="BaseAreas"/>.</summary>
        public List<NavTileData>? BaseTiles { get; set; }
    }

    internal TileCache(Mesh3DPipeline pipeline) => _pipeline = pipeline;

    /// <summary>The mesh. It changes as <see cref="Update"/> rebuilds tiles; keep using this object (query objects holding it see the changes).</summary>
    public TiledNavMesh Mesh => _mesh!;

    /// <summary>The pipeline the tiles were built with and are rebuilt with.</summary>
    public Mesh3DPipeline Pipeline => _pipeline;

    /// <summary>Number of tiles waiting to be rebuilt. <see cref="Update"/> rebuilds them in the order the changes queued them (first queued, first rebuilt).</summary>
    public int PendingCount => _queue.Count;

    /// <summary>The number of obstacles in force.</summary>
    public int ObstacleCount => _obstacles.Count;

    /// <summary>The number of run-time area volumes in force.</summary>
    public int VolumeCount => _volumes.Count;

    /// <summary>Tile positions with cached data (tiles that were built, whether or not they hold polygons now).</summary>
    public int CachedTileCount => _entries.Count;

    /// <summary>Whether a rebuilt layer repairs the first build's contours where obstacles changed it; off, every rebuilt layer is built from scratch (the results are equal; tests compare the two).</summary>
    internal bool RepairContours { get; set; } = true;

    /// <summary>Layer builds that repaired contours, and layer builds that could have but started from scratch.</summary>
    internal (int Repaired, int Scratch) RepairCounts => (Work.Repaired, Work.Scratch);

    internal void SetMesh(TiledNavMesh mesh) => _mesh = mesh;

    /// <summary>Buffers of the layer builder, reused by every rebuild.</summary>
    internal LayerWork Work { get; } = new();

    internal void StoreLean(LeanTile tile) => _entries[Key(tile.TileX, tile.TileZ)] = new Entry(tile);

    internal void Store(int tx, int tz, CompactHeightfield chf, int[]? layers)
    {
        // Every rebuild of the tile starts from copies of this heightfield; its links never change, so the span graph is built once here.
        chf.ShareGraph();
        _entries[Key(tx, tz)] = new Entry(tx, tz, chf, layers) { LastAreas = Areas(chf), BaseAreas = Areas(chf) };
    }

    internal void StoreBase(int tx, int tz, List<NavTileData> output, int from) => _entries[Key(tx, tz)].BaseTiles = output.GetRange(from, output.Count - from);

    private static byte[] Areas(CompactHeightfield chf)
    {
        ReadOnlySpan<CompactSpan> spans = chf.Spans;
        var a = new byte[spans.Length];
        for (int i = 0; i < a.Length; i++)
        {
            a[i] = spans[i].Area;
        }
        return a;
    }

    private static bool SameAreas(CompactHeightfield chf, byte[]? last)
    {
        ReadOnlySpan<CompactSpan> spans = chf.Spans;
        if (last is null || last.Length != spans.Length)
        {
            return false;
        }
        for (int i = 0; i < last.Length; i++)
        {
            if (spans[i].Area != last[i])
            {
                return false;
            }
        }
        return true;
    }

    private static long Key(int tx, int tz) => ((long)tz << 32) + tx;

    // ------------------------------------------------------------ obstacles and volumes

    /// <summary>Adds a temporary obstacle and queues the tiles it touches.</summary>
    /// <returns>The obstacle's id, 1 or more, never reused within this cache.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="obstacle"/> is null.</exception>
    /// <exception cref="InvalidOperationException">There are already 2^20 obstacles.</exception>
    public int AddObstacle(TileObstacle obstacle)
    {
        ArgumentNullException.ThrowIfNull(obstacle);
        if (_obstacles.Count >= MaxObstacles)
        {
            throw new InvalidOperationException($"At most {MaxObstacles} obstacles are supported.");
        }
        int id = _nextObstacle++;
        _obstacles.Add((id, obstacle));
        Queue(obstacle.MinX, obstacle.MaxX, obstacle.MinZ, obstacle.MaxZ);
        return id;
    }

    /// <summary>Removes an obstacle and queues the tiles it touched.</summary>
    /// <returns>False when no obstacle has this id (nothing changed).</returns>
    public bool RemoveObstacle(int id)
    {
        int i = _obstacles.FindIndex(o => o.Id == id);
        if (i < 0)
        {
            return false;
        }
        TileObstacle o = _obstacles[i].Obstacle;
        _obstacles.RemoveAt(i);
        Queue(o.MinX, o.MaxX, o.MinZ, o.MaxZ);
        return true;
    }

    /// <summary>The obstacle with this id (the value returned when it was added), or null if there is none (never added, or removed).</summary>
    public TileObstacle? GetObstacle(int id)
    {
        int i = _obstacles.FindIndex(o => o.Id == id);
        return i < 0 ? null : _obstacles[i].Obstacle;
    }

    /// <summary>
    /// Adds an area volume and queues the tiles it touches. Run-time volumes apply after the pipeline's own volumes, in the order added.
    /// </summary>
    /// <returns>The volume's id, 1 or more, never reused within this cache.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="volume"/> is null.</exception>
    /// <exception cref="InvalidOperationException">There are already 2^20 volumes.</exception>
    public int AddVolume(ConvexVolume volume)
    {
        ArgumentNullException.ThrowIfNull(volume);
        if (_volumes.Count >= MaxObstacles)
        {
            throw new InvalidOperationException($"At most {MaxObstacles} volumes are supported.");
        }
        int id = _nextVolume++;
        _volumes.Add((id, volume));
        Queue(volume.MinX, volume.MaxX, volume.MinZ, volume.MaxZ);
        return id;
    }

    /// <summary>Removes an area volume and queues the tiles it touched.</summary>
    /// <returns>False when no volume has this id (nothing changed).</returns>
    public bool RemoveVolume(int id)
    {
        int i = _volumes.FindIndex(v => v.Id == id);
        if (i < 0)
        {
            return false;
        }
        ConvexVolume v = _volumes[i].Volume;
        _volumes.RemoveAt(i);
        Queue(v.MinX, v.MaxX, v.MinZ, v.MaxZ);
        return true;
    }

    // Queues every cached tile whose build grid (tile plus border) overlaps the box grown by the agent radius (obstacles are marked grown by it),
    // in tile row, column order. The box is widened by one more cell:
    // a tile rebuilt without a change gives the same bytes, so over-queueing is harmless and rounding at an edge cannot lose a tile.
    private void Queue(double minX, double maxX, double minZ, double maxZ)
    {
        double pad = _pipeline.CellSize * (_pipeline.Border + _pipeline.SpanOptions.WalkableRadius + 1);
        double size = _pipeline.Params.TileSize;
        foreach (KeyValuePair<long, Entry> kv in _entries)
        {
            Entry e = kv.Value;
            double x0 = _pipeline.Params.TileOriginX(e.TileX) - pad, x1 = _pipeline.Params.TileOriginX(e.TileX) + size + pad;
            double z0 = _pipeline.Params.TileOriginZ(e.TileZ) - pad, z1 = _pipeline.Params.TileOriginZ(e.TileZ) + size + pad;
            if (maxX >= x0 && minX <= x1 && maxZ >= z0 && minZ <= z1 && _queued.Add(kv.Key))
            {
                _queue.Add(kv.Key);
            }
        }
    }

    // ------------------------------------------------------------ rebuilding

    /// <summary>
    /// Rebuilds queued tiles, oldest first, at most <paramref name="maxTiles"/> of them. One tile is one iteration, so the same call does the same work
    /// on every machine. The cancellation token is checked between tiles; a tile in progress finishes.
    /// </summary>
    /// <param name="maxTiles">The budget in tiles; 0 does nothing.</param>
    /// <param name="cancellationToken">Stops between tiles.</param>
    /// <returns>The number of tiles rebuilt.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxTiles"/> is negative.</exception>
    /// <exception cref="OperationCanceledException">Cancelled; the tiles rebuilt so far stay rebuilt and the rest stay queued.</exception>
    public int Update(int maxTiles, CancellationToken cancellationToken = default)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(maxTiles);
        int done = 0;
        while (done < maxTiles && _queue.Count > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            long key = _queue[0];
            RebuildTile(_entries[key]);
            _queue.RemoveAt(0);
            _queued.Remove(key);
            done++;
        }
        return done;
    }

    /// <summary>Rebuilds every queued tile (<see cref="Update"/> with no budget).</summary>
    /// <param name="cancellationToken">Stops between tiles; see <see cref="Update"/>.</param>
    /// <returns>The number of tiles rebuilt.</returns>
    /// <exception cref="OperationCanceledException">Cancelled; the tiles rebuilt so far stay rebuilt and the rest stay queued.</exception>
    public int UpdateAll(CancellationToken cancellationToken = default) => Update(int.MaxValue, cancellationToken);

    private void RebuildTile(Entry e)
    {
        if (e.Lean is not null)
        {
            RebuildLean(e.Lean);
            return;
        }

        var obstacles = new List<TileObstacle>(_obstacles.Count);
        foreach ((int _, TileObstacle o) in _obstacles)
        {
            obstacles.Add(o);
        }
        var volumes = new List<ConvexVolume>(_volumes.Count);
        foreach ((int _, ConvexVolume v) in _volumes)
        {
            volumes.Add(v);
        }
        CompactHeightfield chf = e.Chf.CloneScratch();
        _pipeline.MarkObstacles(chf, obstacles, volumes);
        // Tiles are queued generously (a neighbour's obstacle may only graze the border), so most rebuilds find the areas unchanged.
        if (SameAreas(chf, e.LastAreas) && (e.LastLayers < 0 || Mesh.GetTilesAt(e.TileX, e.TileZ, _refs) == e.LastLayers))
        {
            chf.ReleaseScratch();
            return;
        }
        e.LastAreas = Areas(chf);
        var built = new List<NavTileData>();
        if (e.BaseTiles is not null && SameAreas(chf, e.BaseAreas))
        {
            built.AddRange(e.BaseTiles);
        }
        else
        {
            _pipeline.FinishMarked(chf, e.TileX, e.TileZ, e.Layers, built);
        }
        chf.ReleaseScratch();
        e.LastLayers = built.Count;

        TiledNavMesh mesh = Mesh;
        int existing = Math.Min(mesh.GetTilesAt(e.TileX, e.TileZ, _refs), _refs.Length);
        var old = new Dictionary<int, TileRef>();
        for (int i = 0; i < existing; i++)
        {
            NavTile? t = mesh.GetTile(_refs[i]);
            if (t is not null)
            {
                old[t.Layer] = _refs[i];
            }
        }
        foreach (NavTileData d in built)
        {
            NavTile tile = NavTile.Create(d, mesh.Params);
            // A tile whose as-built content did not change stays as it is: polygon references into it stay valid and run-time polygon edits stay.
            if (old.TryGetValue(d.Layer, out TileRef r))
            {
                old.Remove(d.Layer);
                if (mesh.GetTile(r)!.ContentId.Equals(tile.ContentId))
                {
                    continue;
                }
            }
            mesh.ReplaceTile(tile);
        }
        // Layers that no longer exist, lowest layer first.
        var gone = new List<int>(old.Keys);
        gone.Sort();
        foreach (int layer in gone)
        {
            mesh.RemoveTile(old[layer]);
        }
    }

    // Layer-first tile: every layer is marked and compared with what the mesh holds for it; only the layers whose areas changed are built again.
    // A layer whose areas are those of the first build gets the first build's tile back.
    private void RebuildLean(LeanTile t)
    {
        Mesh3DPipeline p = _pipeline;
        TiledNavMesh mesh = Mesh;
        int nl = t.Layers.Length;
        int existing = Math.Min(mesh.GetTilesAt(t.TileX, t.TileZ, _refs), _refs.Length);
        var present = new TileRef?[nl];
        for (int i = 0; i < existing; i++)
        {
            NavTile? nt = mesh.GetTile(_refs[i]);
            if (nt is not null && nt.Layer < nl)
            {
                present[nt.Layer] = _refs[i];
            }
        }

        // Obstacles and volumes in force, in the order they were added.
        var obstacles = _obstacleList;
        var volumes = _volumeList;
        obstacles.Clear();
        foreach ((int _, TileObstacle o) in _obstacles)
        {
            obstacles.Add(o);
        }

        volumes.Clear();
        foreach ((int _, ConvexVolume v) in _volumes)
        {
            volumes.Add(v);
        }

        // Off-mesh links go to the layer that fits them best, so with links the whole tile is built again whenever any layer changed.
        bool links = p.Settings.OffMeshLinks is { Count: > 0 };
        LayerWork work = Work;
        var fresh = new NavTileData?[nl];
        var changed = new bool[nl];
        var newArea = new byte[nl][];
        bool any = false;
        for (int l = 0; l < nl; l++)
        {
            LayerSpans ls = t.Layers[l];
            bool marked = t.Mark(p, l, obstacles, volumes, work);
            byte[]? last = t.LastArea[l];
            // Nothing extra reaches the layer: its areas are the first build's.
            bool same = last is not null && (marked ? work.Area.AsSpan(0, ls.Count).SequenceEqual(last) : ReferenceEquals(last, t.BaseArea[l]) || last.AsSpan().SequenceEqual(t.BaseArea[l]));
            // The mesh must hold what the last build produced for the layer; a tile removed or replaced from outside is rebuilt.
            if (same && (t.LastData[l] is null) == (present[l] is null))
            {
                continue;
            }

            changed[l] = true;
            any = true;
            if (!marked)
            {
                newArea[l] = t.BaseArea[l];
                fresh[l] = t.BaseData[l];
                continue;
            }

            newArea[l] = work.Area.AsSpan(0, ls.Count).ToArray();
            if (!links)
            {
                fresh[l] = newArea[l].AsSpan().SequenceEqual(t.BaseArea[l])
                    ? t.BaseData[l]
                    : LayerTileBuilder.Build(p, ls, work.Area, work, t.Chf, t.TileX, t.TileZ, null, RepairContours ? t.Bases[l] : null);
            }
        }

        if (!any)
        {
            return;
        }

        if (links)
        {
            // Every layer fresh, so that the links choose among all of them as in the first build.
            var built = new List<NavTileData>();
            for (int l = 0; l < nl; l++)
            {
                LayerSpans ls = t.Layers[l];
                if (!t.Mark(p, l, obstacles, volumes, work))
                {
                    Array.Copy(t.BaseArea[l], work.Area, ls.Count);
                }

                newArea[l] = work.Area.AsSpan(0, ls.Count).ToArray();
                changed[l] = true;
                fresh[l] = LayerTileBuilder.Build(p, ls, work.Area, work, t.Chf, t.TileX, t.TileZ, null, RepairContours ? t.Bases[l] : null);
                if (fresh[l] is not null)
                {
                    built.Add(fresh[l]!);
                }
            }

            OffMeshLinkBaker.Attach(p.Params, p.Settings.OffMeshLinks, built, 0);
        }

        // Swap in layer order from the lowest: a layer that builds to the content the mesh holds keeps its tile (polygon references into it stay valid and its
        // run-time polygon edits stay), a changed one replaces its tile, a new one is added; layers that no longer have a tile are removed last, lowest first.
        for (int l = 0; l < nl; l++)
        {
            if (!changed[l])
            {
                continue;
            }

            NavTileData? prev = t.LastData[l];
            NavTileData? d = fresh[l];
            t.LastArea[l] = newArea[l];
            t.LastData[l] = d;
            if (d is null)
            {
                continue;
            }

            if (present[l] is not null && prev is not null && LeanTile.SameContent(prev, d))
            {
                continue;
            }

            NavTile tile = NavTile.Create(d, mesh.Params);
            if (present[l] is not null && prev is null && mesh.GetTile(present[l]!.Value)!.ContentId.Equals(tile.ContentId))
            {
                continue;
            }

            mesh.ReplaceTile(tile);
        }

        for (int l = 0; l < nl; l++)
        {
            if (changed[l] && fresh[l] is null && present[l] is TileRef r)
            {
                mesh.RemoveTile(r);
            }
        }
    }

    // ------------------------------------------------------------ state

    /// <summary>
    /// Captures the run-time state: the obstacles and volumes in force (with their ids), the rebuild queue, and the mesh's own snapshot
    /// (<see cref="TiledNavMesh.Snapshot"/>). The cached tile data is not state: it is rebuilt from the geometry, so restoring needs a cache built from the
    /// same geometry and settings. The format is in <c>docs/navmesh-snapshot.md</c>; the same state gives the same bytes on every platform.
    /// </summary>
    public byte[] Snapshot(SnapshotMode mode = SnapshotMode.Full)
    {
        byte[] meshBytes = Mesh.Snapshot(mode);
        var w = new NavMeshSerializer.Writer(256 + meshBytes.Length);
        w.U32(Magic);
        w.U16(StateVersion);
        w.U16(NavMeshSerializer.HeaderFlags);
        w.U32((uint)_nextObstacle);
        w.U32((uint)_nextVolume);
        w.U32((uint)_obstacles.Count);
        foreach ((int id, TileObstacle o) in _obstacles)
        {
            w.U32((uint)id);
            w.U8((byte)o.Shape);
            foreach (double d in o.Parameters)
            {
                w.F64(d);
            }
        }
        w.U32((uint)_volumes.Count);
        foreach ((int id, ConvexVolume v) in _volumes)
        {
            w.U32((uint)id);
            w.U8(v.Area);
            w.F64(v.MinY);
            w.F64(v.MaxY);
            ReadOnlySpan<double> c = v.Corners;
            w.U32((uint)(c.Length / 2));
            foreach (double d in c)
            {
                w.F64(d);
            }
        }
        w.U32((uint)_queue.Count);
        foreach (long key in _queue)
        {
            Entry e = _entries[key];
            w.I32(e.TileX);
            w.I32(e.TileZ);
        }
        w.U32((uint)meshBytes.Length);
        w.Bytes(meshBytes);
        return w.ToArray();
    }

    /// <summary>
    /// Replaces the run-time state with a snapshot, keeping this object and <see cref="Mesh"/>. All or nothing: on any error nothing changes.
    /// </summary>
    /// <param name="snapshot">Bytes written by <see cref="Snapshot"/> of a cache built from the same geometry and settings.</param>
    /// <param name="library">Tiles for a snapshot taken in <see cref="SnapshotMode.Referenced"/> mode.</param>
    /// <exception cref="NavMeshFormatException">The bytes are malformed, name a tile this cache has no data for, or the mesh part does not fit the mesh.</exception>
    public void Restore(ReadOnlySpan<byte> snapshot, TileLibrary? library = null)
    {
        var r = new NavMeshSerializer.Reader(snapshot);
        if (r.U32("magic") != Magic)
        {
            throw new NavMeshFormatException("Not a tile cache snapshot (bad magic).");
        }
        ushort version = r.U16("version");
        if (version != StateVersion)
        {
            throw new NavMeshFormatException($"Tile cache snapshot version {version} is not supported (this build reads {StateVersion}).");
        }
        r.U16("flags");
        uint nextObstacle = r.U32("next obstacle id");
        uint nextVolume = r.U32("next volume id");
        if (nextObstacle < 1 || nextObstacle > int.MaxValue || nextVolume < 1 || nextVolume > int.MaxValue)
        {
            throw new NavMeshFormatException("A next id is out of range.");
        }
        uint nObstacles = r.U32("obstacle count");
        if (nObstacles > MaxObstacles || nObstacles > r.Remaining / 13)
        {
            throw new NavMeshFormatException($"Obstacle count {nObstacles} is not possible for {r.Remaining} remaining bytes.");
        }
        var obstacles = new List<(int, TileObstacle)>((int)nObstacles);
        long prev = 0;
        double[] p = new double[7];
        for (uint i = 0; i < nObstacles; i++)
        {
            uint id = r.U32("obstacle id");
            if (id <= prev || id >= nextObstacle)
            {
                throw new NavMeshFormatException($"Obstacle {i} has id {id}, which is not ascending or not below the next id {nextObstacle}.");
            }
            prev = id;
            byte shape = r.U8("obstacle shape");
            int n = TileObstacle.ParameterCount((ObstacleShape)shape);
            if (n < 0)
            {
                throw new NavMeshFormatException($"Obstacle {i} has unknown shape {shape}.");
            }
            for (int k = 0; k < n; k++)
            {
                p[k] = r.F64("obstacle parameter");
            }
            try
            {
                obstacles.Add(((int)id, TileObstacle.FromParameters((ObstacleShape)shape, p.AsSpan(0, n))));
            }
            catch (ArgumentException e)
            {
                throw new NavMeshFormatException($"Obstacle {i}: {e.Message}", e);
            }
        }
        uint nVolumes = r.U32("volume count");
        if (nVolumes > MaxObstacles || nVolumes > r.Remaining / 41)
        {
            throw new NavMeshFormatException($"Volume count {nVolumes} is not possible for {r.Remaining} remaining bytes.");
        }
        var volumes = new List<(int, ConvexVolume)>((int)nVolumes);
        prev = 0;
        for (uint i = 0; i < nVolumes; i++)
        {
            uint id = r.U32("volume id");
            if (id <= prev || id >= nextVolume)
            {
                throw new NavMeshFormatException($"Volume {i} has id {id}, which is not ascending or not below the next id {nextVolume}.");
            }
            prev = id;
            byte area = r.U8("volume area");
            double minY = r.F64("volume min y");
            double maxY = r.F64("volume max y");
            uint corners = r.U32("volume corner count");
            if (corners < 3 || corners > 256 || (ulong)corners * 16 > (ulong)r.Remaining)
            {
                throw new NavMeshFormatException($"Volume {i} has {corners} corners; 3 to 256 are possible for {r.Remaining} remaining bytes.");
            }
            var xz = new double[corners * 2];
            for (int k = 0; k < xz.Length; k++)
            {
                xz[k] = r.F64("volume corner");
            }
            if (area > NavMeshFormat.MaxArea)
            {
                throw new NavMeshFormatException($"Volume {i} has area {area}; the largest area id is {NavMeshFormat.MaxArea}.");
            }
            try
            {
                volumes.Add(((int)id, new ConvexVolume(xz, minY, maxY, area)));
            }
            catch (ArgumentException e)
            {
                throw new NavMeshFormatException($"Volume {i}: {e.Message}", e);
            }
        }
        uint nQueue = r.U32("queue length");
        if (nQueue > _entries.Count || (ulong)nQueue * 8 > (ulong)r.Remaining)
        {
            throw new NavMeshFormatException($"Queue length {nQueue} is not possible for {_entries.Count} cached tiles and {r.Remaining} remaining bytes.");
        }
        var queue = new List<long>((int)nQueue);
        var seen = new HashSet<long>();
        for (uint i = 0; i < nQueue; i++)
        {
            int tx = r.I32("queued tile x");
            int tz = r.I32("queued tile z");
            long key = Key(tx, tz);
            if (!_entries.ContainsKey(key))
            {
                throw new NavMeshFormatException($"Queued tile ({tx}, {tz}) has no cached data in this cache.");
            }
            if (!seen.Add(key))
            {
                throw new NavMeshFormatException($"Tile ({tx}, {tz}) is queued twice.");
            }
            queue.Add(key);
        }
        uint meshLen = r.U32("mesh snapshot length");
        ReadOnlySpan<byte> meshBytes = r.Take((int)Math.Min(meshLen, int.MaxValue), "mesh snapshot");
        r.ExpectEnd();

        // The mesh restore is all or nothing, and everything above is parsed, so from here on the cache cannot end up half restored.
        Mesh.Restore(meshBytes, library);
        _obstacles.Clear();
        _obstacles.AddRange(obstacles);
        _volumes.Clear();
        _volumes.AddRange(volumes);
        _queue.Clear();
        _queue.AddRange(queue);
        _queued.Clear();
        _queued.UnionWith(queue);
        foreach (Entry e in _entries.Values)
        {
            e.LastAreas = null;
            if (e.Lean is not null)
            {
                // The mesh holds anything now: forget both what each layer was built from and the tile data it was built to.
                Array.Clear(e.Lean.LastArea);
                Array.Clear(e.Lean.LastData);
            }
        }
        _nextObstacle = (int)nextObstacle;
        _nextVolume = (int)nextVolume;
    }
}
