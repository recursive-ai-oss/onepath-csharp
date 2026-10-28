using System;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for run-time changes of a 3D-built mesh. Odd first byte: the rest is a script of operations (add or remove obstacles and volumes with
/// hostile numbers mixed in, sliced updates, snapshot and restore) and the invariants are checked: documented argument errors only, a valid mesh after every
/// update, snapshot → restore → snapshot giving the same bytes, and a mesh rebuilt from scratch with the surviving obstacles equal to the cache's.
/// Even first byte: the rest is a tile cache snapshot read as is, or (second byte odd) patched into a valid one, which must be rejected with the typed
/// <see cref="NavMeshFormatException"/> leaving the cache untouched, or accepted and stable.
/// </summary>
internal static class TileCacheFuzz
{
    private static readonly NavSettings s_settings = new() { CellSize = 0.5, CellHeight = 0.25, TileSize = 8, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f };
    private static readonly double[] s_soup = { 0, 0, 0, 0, 0, 16, 16, 0, 16, 16, 0, 0, 4, 1, 4, 4, 1, 12, 12, 1, 12, 12, 1, 4 };
    private static readonly int[] s_index = { 0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7 };
    private static readonly TileCache s_cache = new Mesh3DPipeline(s_settings).BuildSoupCached(s_soup, s_index);
    private static readonly byte[] s_base = s_cache.Snapshot();

    private static readonly double[] Special =
    {
        double.NaN, double.PositiveInfinity, double.NegativeInfinity, 3e6, -3e6, 1e300, -0.0, 4.9e-324, 2097152.0, -2097152.0, 0.0,
    };

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _p;

        public Reader(byte[] d, int start) { _d = d; _p = start; }

        public bool More => _p < _d.Length;

        public int Byte() => _p < _d.Length ? _d[_p++] : 0;

        // Mostly a multiple of 1/4 within the level (so shapes fall on and near cell boundaries), sometimes hostile.
        public double Number()
        {
            int v = Byte() | (Byte() << 8);
            if ((v & 0xF000) == 0xF000)
            {
                return Special[((v >> 4) & 0xFF) % Special.Length];
            }
            return ((v & 0x3FF) / 4.0) - 32.0;
        }
    }

    public static void Run(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }
        s_cache.Restore(s_base);
        if ((data[0] & 1) == 0)
        {
            ReadState(data);
        }
        else
        {
            Script(data);
        }
    }

    private static void Script(byte[] data)
    {
        TileCache c = s_cache;
        var r = new Reader(data, 1);
        var obstacleIds = new System.Collections.Generic.List<int>();
        var volumeIds = new System.Collections.Generic.List<int>();
        var live = new System.Collections.Generic.List<TileObstacle>();
        var liveVolumes = new System.Collections.Generic.List<ConvexVolume>();
        byte[]? snap = null;
        int ops = 0;
        while (r.More && ops++ < 24)
        {
            switch (r.Byte() % 8)
            {
                case 0:
                    try
                    {
                        TileObstacle o = TileObstacle.Cylinder(r.Number(), r.Number() / 8, r.Number(), Math.Abs(r.Number()) / 8, Math.Abs(r.Number()) / 8);
                        obstacleIds.Add(c.AddObstacle(o));
                        live.Add(o);
                    }
                    catch (ArgumentException)
                    {
                    }
                    break;
                case 1:
                    try
                    {
                        TileObstacle o = TileObstacle.Box(r.Number(), r.Number() / 8, r.Number(), r.Number(), r.Number() / 8, r.Number());
                        obstacleIds.Add(c.AddObstacle(o));
                        live.Add(o);
                    }
                    catch (ArgumentException)
                    {
                    }
                    break;
                case 2:
                    try
                    {
                        TileObstacle o = TileObstacle.OrientedBox(r.Number(), r.Number() / 8, r.Number(), Math.Abs(r.Number()) / 4, Math.Abs(r.Number()) / 8, Math.Abs(r.Number()) / 4, r.Number());
                        obstacleIds.Add(c.AddObstacle(o));
                        live.Add(o);
                    }
                    catch (ArgumentException)
                    {
                    }
                    break;
                case 3:
                    if (obstacleIds.Count > 0)
                    {
                        int i = r.Byte() % obstacleIds.Count;
                        if (!c.RemoveObstacle(obstacleIds[i]))
                        {
                            throw new InvalidOperationException("a live obstacle could not be removed");
                        }
                        obstacleIds.RemoveAt(i);
                        live.RemoveAt(i);
                    }
                    break;
                case 4:
                    try
                    {
                        double x = r.Number(), z = r.Number(), w = Math.Abs(r.Number()) + 0.5, h = Math.Abs(r.Number()) + 0.5;
                        var vol = new ConvexVolume(new[] { x, z, x + w, z, x + w, z + h, x, z + h }, r.Number() / 8, r.Number() / 8, (byte)(r.Byte() & 63));
                        volumeIds.Add(c.AddVolume(vol));
                        liveVolumes.Add(vol);
                    }
                    catch (ArgumentException)
                    {
                    }
                    break;
                case 5:
                    if (volumeIds.Count > 0)
                    {
                        int i = r.Byte() % volumeIds.Count;
                        c.RemoveVolume(volumeIds[i]);
                        volumeIds.RemoveAt(i);
                        liveVolumes.RemoveAt(i);
                    }
                    break;
                case 6:
                    c.Update(r.Byte() % 3, default);
                    break;
                default:
                    if (snap is null)
                    {
                        snap = c.Snapshot();
                    }
                    else
                    {
                        c.Restore(snap);
                        if (!c.Snapshot().AsSpan().SequenceEqual(snap))
                        {
                            throw new InvalidOperationException("restore did not reproduce the snapshot");
                        }
                        snap = null;
                        // Ids and lists now follow the snapshot; the script's own bookkeeping no longer applies.
                        obstacleIds.Clear();
                        volumeIds.Clear();
                        live.Clear();
                        liveVolumes.Clear();
                        c.UpdateAll(default);
                        return;
                    }
                    break;
            }
        }
        c.UpdateAll(default);
        if (NavMeshValidator.Validate(c.Mesh).Count != 0)
        {
            throw new InvalidOperationException("the mesh is inconsistent after an update");
        }
        // From scratch with the surviving obstacles and volumes baked into the build: the same tiles.
        var baked = new Mesh3DPipeline(s_settings);
        foreach (TileObstacle o in live)
        {
            baked.Obstacles.Add(o);
        }
        foreach (ConvexVolume v in liveVolumes)
        {
            baked.Regions.Spans.Volumes.Add(v);
        }
        TiledNavMesh fresh = baked.BuildSoup(s_soup, s_index);
        var refsA = new TileRef[8];
        var refsB = new TileRef[8];
        for (int z = -1; z < 4; z++)
        {
            for (int x = -1; x < 4; x++)
            {
                int na = c.Mesh.GetTilesAt(x, z, refsA), nb = fresh.GetTilesAt(x, z, refsB);
                if (na != nb)
                {
                    throw new InvalidOperationException($"tile ({x}, {z}) has {na} layers after the update but {nb} when built from scratch");
                }
                for (int i = 0; i < na && i < 8; i++)
                {
                    if (!NavMeshSerializer.WriteTile(c.Mesh.GetTile(refsA[i])!).AsSpan().SequenceEqual(NavMeshSerializer.WriteTile(fresh.GetTile(refsB[i])!)))
                    {
                        throw new InvalidOperationException($"tile ({x}, {z}) differs between rebuilding from the cache and building from scratch");
                    }
                }
            }
        }
        byte[] state = c.Snapshot();
        c.Restore(state);
        if (!c.Snapshot().AsSpan().SequenceEqual(state))
        {
            throw new InvalidOperationException("snapshot is not stable under restore");
        }
    }

    private static void ReadState(byte[] data)
    {
        byte[] bytes;
        if ((data[1] & 1) == 0)
        {
            bytes = data[2..];
        }
        else
        {
            // Start from a state with obstacles in it so the patching reaches the obstacle, volume and queue sections.
            s_cache.AddObstacle(TileObstacle.Cylinder(5, 0, 5, 2, 2));
            s_cache.AddVolume(new ConvexVolume(new double[] { 1, 1, 6, 1, 6, 6, 1, 6 }, -1, 1, 4));
            bytes = s_cache.Snapshot();
            s_cache.Restore(s_base);
            for (int i = 2; i + 2 < data.Length; i += 3)
            {
                bytes[((data[i] << 8) | data[i + 1]) % bytes.Length] = data[i + 2];
            }
        }
        byte[] before = s_cache.Snapshot();
        try
        {
            s_cache.Restore(bytes);
        }
        catch (NavMeshFormatException)
        {
            if (!s_cache.Snapshot().AsSpan().SequenceEqual(before))
            {
                throw new InvalidOperationException("a failed restore changed the cache");
            }
            return;
        }
        byte[] again = s_cache.Snapshot();
        s_cache.Restore(again);
        if (!s_cache.Snapshot().AsSpan().SequenceEqual(again))
        {
            throw new InvalidOperationException("restored state is not stable");
        }
        s_cache.UpdateAll(default);
        if (NavMeshValidator.Validate(s_cache.Mesh).Count != 0)
        {
            throw new InvalidOperationException("restored mesh is inconsistent after the queue was rebuilt");
        }
    }
}
