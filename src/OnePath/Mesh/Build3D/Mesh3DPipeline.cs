using System;
using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;
using RegionStage = RecursiveAI.OnePath.Mesh.Build3D.Regions;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// The 3D builder from geometry to a tiled mesh, with every stage replaceable: per tile, voxelize (the input form) → span pipeline → regions →
/// tile border → contours → <see cref="Polygonizer"/> → <see cref="DetailMeshBuilder"/> → <see cref="NavTileData"/>; then all tiles are assembled and stitched.
/// Each tile is built on a grid of its cells plus a border (agent radius plus 3 cells) so that erosion and regions near a tile edge see the neighbouring
/// floor; everything the border covers is cut away before the contours, so polygons end exactly on the tile boundary. Tiles are built one after
/// the other in row-major order (tile row, then column), which fixes the slot numbering. The rules are in docs/build3d-polys.md.
/// </summary>
public sealed class Mesh3DPipeline
{
    private const int MaxTiles = 1 << 20;

    /// <summary>Creates a pipeline for the settings.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="settings"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range; the message names it.</exception>
    public Mesh3DPipeline(NavSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        Settings = settings;
        Params = settings.ToParams();
        CheckSettings(settings);
        OffMeshLinkBaker.Validate(Params, settings.OffMeshLinks);
        double ratio = (Params.TileSize / Params.CellSize) - 1e-4;
        TileCells = Math.Max(1, (int)Math.Ceiling(ratio));
        CellSize = Params.TileSize / TileCells;
        float cs = (float)settings.CellSize, ch = (float)settings.CellHeight;
        SpanOptions = new SpanSettings
        {
            // Agent quantities are rounded to float32 once, as they always were, so the cell counts below do not depend on double rounding.
            WalkableSlopeDegrees = (float)settings.AgentMaxSlopeDegrees,
            WalkableHeight = Math.Max(1, (int)Math.Ceiling((float)settings.AgentHeight / ch)),
            WalkableClimb = (int)Math.Floor((float)settings.WalkableClimb / ch),
            WalkableRadius = (int)Math.Ceiling((float)settings.AgentRadius / cs),
        };
        Border = SpanOptions.WalkableRadius + 3;
        Regions = new RegionPipeline(
            SpanOptions,
            new RegionSettings { Partition = settings.Partition, MinRegionArea = settings.MinRegionArea, MergeRegionArea = settings.MergeRegionArea },
            new ContourSettings { MaxError = (float)settings.ContourMaxError, MaxEdgeLength = (int)((settings.MaxEdgeLength / settings.CellSize) + 0.5) });
        Detail = new DetailSettings { SampleDistance = settings.DetailSampleDistance, MaxError = settings.DetailMaxError };
    }

    /// <summary>The navigation settings (cell sizes, agent size, partition, detail) the pipeline was created from; every stage setting below derives from them.</summary>
    public NavSettings Settings { get; }

    /// <summary>The mesh parameters the tiles are built for.</summary>
    public NavMeshParams Params { get; }

    /// <summary>The span-stage settings derived from the agent (rows and cells).</summary>
    public SpanSettings SpanOptions { get; }

    /// <summary>Cells along one tile side: the tile size over the cell size, rounded up so that cells divide the tile exactly.</summary>
    public int TileCells { get; }

    /// <summary>The cell size actually used: the tile size divided by <see cref="TileCells"/> (at most <see cref="NavSettings.CellSize"/> plus rounding).</summary>
    public double CellSize { get; }

    /// <summary>Border cells around each tile's build grid.</summary>
    public int Border { get; }

    /// <summary>The span, region and contour stages with their replaceable steps. Replace steps on it before building.</summary>
    public RegionPipeline Regions { get; }

    /// <summary>Parameters of the height detail stage.</summary>
    public DetailSettings Detail { get; set; }

    /// <summary>The polygon step: contours, vertex limit, region to skip, coordinate map → polygons. Default <see cref="Polygonizer.Build"/>.</summary>
    public Func<ContourSet, int, int, TileCoordinates, PolyMesh> Polygonize { get; set; } = Polygonizer.Build;

    /// <summary>The detail step. Default <see cref="DetailMeshBuilder.Build"/>.</summary>
    public Func<PolyMesh, CompactHeightfield, TileCoordinates, DetailSettings, DetailMeshSet> BuildDetail { get; set; } = DetailMeshBuilder.Build;

    private static void CheckSettings(NavSettings s)
    {
        if (!(s.CellHeight >= HeightfieldFrame.MinCell && s.CellHeight <= HeightfieldFrame.MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(s), s.CellHeight, "CellHeight: the cell height must be positive and finite.");
        }
        if (!(s.CellSize >= HeightfieldFrame.MinCell && s.CellSize <= HeightfieldFrame.MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(s), s.CellSize, "CellSize: the cell size must be positive and finite.");
        }
        if (!(s.MaxEdgeLength >= 0) || double.IsInfinity(s.MaxEdgeLength))
        {
            throw new ArgumentOutOfRangeException(nameof(s), s.MaxEdgeLength, "MaxEdgeLength: the maximum edge length must be finite and not negative.");
        }
        if (s.MinRegionArea < 0 || s.MergeRegionArea < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(s), "MinRegionArea and MergeRegionArea must not be negative.");
        }
        new ContourSettings { MaxError = (float)s.ContourMaxError }.Validate();
        new DetailSettings { SampleDistance = s.DetailSampleDistance, MaxError = s.DetailMaxError }.Validate();
        if (!Enum.IsDefined(s.Partition))
        {
            throw new ArgumentOutOfRangeException(nameof(s), s.Partition, "Partition: unknown partition type.");
        }
    }

    private static void CheckWorldValue(double v, char axis, int index)
    {
        if (double.IsNaN(v) || double.IsInfinity(v))
        {
            throw new ArgumentException($"vertex {index} has a non-finite {axis} coordinate ({v}); coordinates must be finite.", nameof(v));
        }
        if (!(Math.Abs(v) < NavMeshParams.MaxWorld))
        {
            throw new ArgumentOutOfRangeException(nameof(v), v, $"vertex {index} has {axis} = {v:R}, outside the supported coordinate range of ±{NavMeshParams.MaxWorld:R} (2^21).");
        }
    }

    // ---------------------------------------------------------------- inputs

    /// <summary>Builds the mesh of a triangle soup.</summary>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle, counter-clockwise seen from the side they face (a floor seen from above).</param>
    /// <exception cref="ArgumentException">A length is not a multiple of three, or a referenced coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range, a referenced coordinate is beyond ±2^21, or the geometry needs more than 2^20 tiles.</exception>
    public TiledNavMesh BuildSoup(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices) => BuildSoupCore(vertices, indices, null);

    /// <summary>
    /// Builds the mesh of a triangle soup like <see cref="BuildSoup"/> and keeps each tile's intermediate data in a <see cref="TileCache"/>, so that
    /// temporary obstacles and area volumes can be added and removed later and only the tiles they touch rebuilt. The cache's mesh is the same
    /// mesh <see cref="BuildSoup"/> returns (tile for tile, byte for byte).
    /// </summary>
    /// <exception cref="ArgumentException">As for <see cref="BuildSoup"/>.</exception>
    /// <exception cref="ArgumentOutOfRangeException">As for <see cref="BuildSoup"/>.</exception>
    public TileCache BuildSoupCached(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices)
    {
        var cache = new TileCache(this);
        cache.SetMesh(BuildSoupCore(vertices, indices, cache));
        return cache;
    }

    private TiledNavMesh BuildSoupCore(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, TileCache? cache)
    {
        if (vertices.Length % 3 != 0 || indices.Length % 3 != 0)
        {
            throw new ArgumentException("vertices and indices must hold whole vertices and triangles (multiples of three).");
        }
        int nv = vertices.Length / 3;
        for (int i = 0; i < indices.Length; i++)
        {
            if ((uint)indices[i] >= (uint)nv)
            {
                throw new ArgumentOutOfRangeException(nameof(indices), indices[i], $"indices[{i}] = {indices[i]} is not a vertex (there are {nv}).");
            }
        }
        int nt = indices.Length / 3;
        var tb = new double[nt * 6];
        var walkable = new bool[nt];
        double minX = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxZ = double.MinValue;
        for (int t = 0; t < nt; t++)
        {
            double x0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, z1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
            for (int k = 0; k < 3; k++)
            {
                int v = indices[(3 * t) + k];
                double x = vertices[3 * v], y = vertices[(3 * v) + 1], z = vertices[(3 * v) + 2];
                CheckWorldValue(x, 'x', v);
                CheckWorldValue(y, 'y', v);
                CheckWorldValue(z, 'z', v);
                x0 = Math.Min(x0, x);
                x1 = Math.Max(x1, x);
                y0 = Math.Min(y0, y);
                y1 = Math.Max(y1, y);
                z0 = Math.Min(z0, z);
                z1 = Math.Max(z1, z);
            }
            tb[6 * t] = x0;
            tb[(6 * t) + 1] = z0;
            tb[(6 * t) + 2] = x1;
            tb[(6 * t) + 3] = z1;
            tb[(6 * t) + 4] = y0;
            tb[(6 * t) + 5] = y1;
            walkable[t] = MayBeWalkable(vertices, indices, t);
            minX = Math.Min(minX, x0);
            maxX = Math.Max(maxX, x1);
            minZ = Math.Min(minZ, z0);
            maxZ = Math.Max(maxZ, z1);
        }
        var tiles = new List<NavTileData>();
        if (nt > 0)
        {
            double margin = Border * CellSize;
            Params.WorldToTile(minX, minZ, out int tx0, out int tz0);
            Params.WorldToTile(maxX, maxZ, out int tx1, out int tz1);
            long tw = (long)tx1 - tx0 + 1, th = (long)tz1 - tz0 + 1;
            if (tw * th > MaxTiles)
            {
                throw new ArgumentOutOfRangeException(nameof(vertices), $"The geometry spans {tw} x {th} tiles of {Params.TileSize:R}; at most {MaxTiles} are supported. Use a larger tile size.");
            }
            var buckets = new List<int>?[tw * th];
            for (int t = 0; t < nt; t++)
            {
                // Tiles whose grid (tile plus border) the triangle's box touches.
                int a0 = TileIndex(tb[6 * t] - margin, Params.OriginX, tx0, tx1);
                int a1 = TileIndex(tb[(6 * t) + 2] + margin, Params.OriginX, tx0, tx1);
                int b0 = TileIndex(tb[(6 * t) + 1] - margin, Params.OriginZ, tz0, tz1);
                int b1 = TileIndex(tb[(6 * t) + 3] + margin, Params.OriginZ, tz0, tz1);
                for (int tz = b0; tz <= b1; tz++)
                {
                    for (int tx = a0; tx <= a1; tx++)
                    {
                        long slot = ((tz - tz0) * tw) + (tx - tx0);
                        (buckets[slot] ??= new List<int>()).Add(t);
                    }
                }
            }
            for (int tz = tz0; tz <= tz1; tz++)
            {
                for (int tx = tx0; tx <= tx1; tx++)
                {
                    List<int>? list = buckets[((tz - tz0) * tw) + (tx - tx0)];
                    if (list is null)
                    {
                        continue;
                    }
                    // A tile without a single triangle that could be walkable cannot hold walkable spans: skip it unbuilt (the result is the same).
                    bool any = false;
                    foreach (int t in list)
                    {
                        any |= walkable[t];
                    }
                    if (!any)
                    {
                        continue;
                    }
                    double y0 = double.MaxValue, y1 = double.MinValue;
                    var sub = new int[list.Count * 3];
                    for (int i = 0; i < list.Count; i++)
                    {
                        int t = list[i];
                        y0 = Math.Min(y0, tb[(6 * t) + 4]);
                        y1 = Math.Max(y1, tb[(6 * t) + 5]);
                        sub[3 * i] = indices[3 * t];
                        sub[(3 * i) + 1] = indices[(3 * t) + 1];
                        sub[(3 * i) + 2] = indices[(3 * t) + 2];
                    }
                    double originY = Math.Max(y0 - Settings.CellHeight, -NavMeshParams.MaxWorld + 1);
                    int rows = RowsFor(y1 - originY);
                    var frame = TileFrame(tx, tz, originY, rows);
                    var solid = new SolidHeightfield(frame);
                    var areas = new byte[list.Count];
                    Voxelizer.ClassifyTriangles(frame, vertices, sub, SpanOptions.WalkableSlopeCosine, areas);
                    Voxelizer.Rasterize(solid, vertices, sub, areas, SpanOptions.EffectiveMergeThreshold);
                    BuildTile(solid, tx, tz, tiles, cache);
                }
            }
        }
        return Assemble(tiles);
    }

    // True unless the triangle is certainly too steep (or degenerate): a margin of 1e-3 over the slope test the voxelizer makes in float32, so the
    // answer "no" is never wrong.
    private bool MayBeWalkable(ReadOnlySpan<double> v, ReadOnlySpan<int> idx, int t)
    {
        int a = 3 * idx[3 * t], b = 3 * idx[(3 * t) + 1], c = 3 * idx[(3 * t) + 2];
        double ux = v[b] - v[a], uy = v[b + 1] - v[a + 1], uz = v[b + 2] - v[a + 2];
        double wx = v[c] - v[a], wy = v[c + 1] - v[a + 1], wz = v[c + 2] - v[a + 2];
        double nx = (uy * wz) - (uz * wy), ny = (uz * wx) - (ux * wz), nz = (ux * wy) - (uy * wx);
        double len = Math.Sqrt((nx * nx) + (ny * ny) + (nz * nz));
        return len > 0 && ny >= (SpanOptions.WalkableSlopeCosine - 1e-3) * len;
    }

    /// <summary>
    /// Builds the mesh of a voxel chunk. Voxel (i, j, k) is cell i along X, row j, cell k along Z, with the chunk's corner at
    /// (<see cref="NavSettings.OriginX"/>, 0, <see cref="NavSettings.OriginZ"/>); one voxel is one cell wide (<see cref="CellSize"/>) and one row high
    /// (<see cref="NavSettings.CellHeight"/>), so choose the tile size to be a whole multiple of the cell size.
    /// </summary>
    /// <param name="chunk">The voxels.</param>
    /// <param name="materialAreas">Area id per material id (256 entries), or empty for "every material is walkable".</param>
    /// <exception cref="ArgumentNullException"><paramref name="chunk"/> is null.</exception>
    /// <exception cref="ArgumentException">materialAreas has a length other than 0 or 256.</exception>
    public TiledNavMesh BuildChunk(VoxelChunk chunk, ReadOnlySpan<byte> materialAreas = default) => BuildChunkCore(chunk, materialAreas, null);

    /// <summary>Builds the mesh of a voxel chunk like <see cref="BuildChunk"/> and keeps the per-tile intermediate data in a <see cref="TileCache"/>.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="chunk"/> is null.</exception>
    /// <exception cref="ArgumentException">materialAreas has a length other than 0 or 256.</exception>
    public TileCache BuildChunkCached(VoxelChunk chunk, ReadOnlySpan<byte> materialAreas = default)
    {
        var cache = new TileCache(this);
        cache.SetMesh(BuildChunkCore(chunk, materialAreas, cache));
        return cache;
    }

    private TiledNavMesh BuildChunkCore(VoxelChunk chunk, ReadOnlySpan<byte> materialAreas, TileCache? cache)
    {
        ArgumentNullException.ThrowIfNull(chunk);
        if (materialAreas.Length != 0 && materialAreas.Length != 256)
        {
            throw new ArgumentException("materialAreas must be empty or have 256 entries.", nameof(materialAreas));
        }
        var tiles = new List<NavTileData>();
        CheckTileCount(chunk.Width, chunk.Depth, out int tw, out int th);
        for (int tz = 0; tz < th; tz++)
        {
            for (int tx = 0; tx < tw; tx++)
            {
                var frame = TileFrame(tx, tz, 0, chunk.Height + SpanOptions.WalkableHeight + 2);
                var solid = new SolidHeightfield(frame);
                Voxelizer.AddChunk(solid, chunk, Border - (tx * TileCells), 0, Border - (tz * TileCells), materialAreas);
                BuildTile(solid, tx, tz, tiles, cache);
            }
        }
        return Assemble(tiles);
    }

    /// <summary>
    /// Builds the mesh of a solid heightfield: its cell (i, k) is cell i along X and k along Z from (<see cref="NavSettings.OriginX"/>, <see cref="NavSettings.OriginZ"/>),
    /// one cell is <see cref="CellSize"/> wide and its rows are <see cref="NavSettings.CellHeight"/> high above the heightfield frame's Y origin; the frame's own cell sizes are ignored.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="heightfield"/> is null.</exception>
    public TiledNavMesh BuildHeightfield(SolidHeightfield heightfield) => BuildHeightfieldCore(heightfield, null);

    /// <summary>Builds the mesh of a solid heightfield like <see cref="BuildHeightfield"/> and keeps the per-tile intermediate data in a <see cref="TileCache"/>.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="heightfield"/> is null.</exception>
    public TileCache BuildHeightfieldCached(SolidHeightfield heightfield)
    {
        var cache = new TileCache(this);
        cache.SetMesh(BuildHeightfieldCore(heightfield, cache));
        return cache;
    }

    private TiledNavMesh BuildHeightfieldCore(SolidHeightfield heightfield, TileCache? cache)
    {
        ArgumentNullException.ThrowIfNull(heightfield);
        HeightfieldFrame src = heightfield.Frame;
        var tiles = new List<NavTileData>();
        CheckTileCount(src.Width, src.Depth, out int tw, out int th);
        for (int tz = 0; tz < th; tz++)
        {
            for (int tx = 0; tx < tw; tx++)
            {
                var frame = TileFrame(tx, tz, src.OriginY, src.Height + SpanOptions.WalkableHeight + 2);
                var solid = new SolidHeightfield(frame);
                for (int dz = 0; dz < frame.Depth; dz++)
                {
                    int sz = (tz * TileCells) + dz - Border;
                    if (sz < 0 || sz >= src.Depth)
                    {
                        continue;
                    }
                    for (int dx = 0; dx < frame.Width; dx++)
                    {
                        int sx = (tx * TileCells) + dx - Border;
                        if (sx < 0 || sx >= src.Width)
                        {
                            continue;
                        }
                        int dst = frame.Column(dx, dz);
                        for (int s = heightfield.Heads[src.Column(sx, sz)]; s != -1; s = heightfield.Span(s).Next)
                        {
                            ref SolidSpan sp = ref heightfield.Span(s);
                            solid.AddSpan(dst, sp.Min, sp.Max, sp.Area, 0);
                        }
                    }
                }
                BuildTile(solid, tx, tz, tiles, cache);
            }
        }
        return Assemble(tiles);
    }

    private void CheckTileCount(int width, int depth, out int tw, out int th)
    {
        tw = (width + TileCells - 1) / TileCells;
        th = (depth + TileCells - 1) / TileCells;
        if ((long)tw * th > MaxTiles)
        {
            throw new ArgumentOutOfRangeException(nameof(width), $"The input needs {tw} x {th} tiles; at most {MaxTiles} are supported. Use a larger tile size.");
        }
    }

    private int TileIndex(double world, double origin, int lo, int hi)
    {
        double f = Math.Floor((world - origin) / Params.TileSize);
        return (int)Math.Clamp(f, lo, hi);
    }

    private int RowsFor(double span)
    {
        double rows = Math.Ceiling(span / Settings.CellHeight) + SpanOptions.WalkableHeight + 2;
        if (!(rows <= HeightfieldFrame.MaxHeight))
        {
            throw new ArgumentOutOfRangeException(nameof(span), $"The geometry of one tile is {span:R} high, which is {rows:R} rows of {Settings.CellHeight:R}; at most {HeightfieldFrame.MaxHeight} are supported. Raise the cell height.");
        }
        return (int)rows;
    }

    private HeightfieldFrame TileFrame(int tx, int tz, double originY, int rows)
    {
        double ox = Params.TileOriginX(tx) - (Border * CellSize);
        double oz = Params.TileOriginZ(tz) - (Border * CellSize);
        int side = TileCells + (2 * Border);
        return new HeightfieldFrame(ox, originY, oz, (float)CellSize, (float)Settings.CellHeight, side, side, rows);
    }

    private TiledNavMesh Assemble(List<NavTileData> tiles) => Build2D.Mesh2DPipeline.Assemble(Params, tiles);

    // ---------------------------------------------------------------- one tile

    /// <summary>
    /// Temporary obstacles baked into every tile this pipeline builds (see <see cref="TileObstacle"/>); applied after erosion with their footprint
    /// grown by the agent radius, in list order. Set in an object initializer: the list is copied and fixed, because a built
    /// <see cref="TileCache"/> never looks at it again. To add or remove obstacles at run time use <see cref="TileCache.AddObstacle"/>.
    /// </summary>
    public IReadOnlyList<TileObstacle> Obstacles { get; init => field = FrozenList<TileObstacle>.Of(value); } = FrozenList<TileObstacle>.Empty;

    /// <summary>
    /// Runs the stages on one tile's rasterized heightfield (built on the tile's grid with border; see <see cref="TileFrame"/>) and appends the tile data of each layer.
    /// The geometry-only part (filters, compaction) is kept in the cache when there is one; the rest is <see cref="FinishTile"/>.
    /// </summary>
    private void BuildTile(SolidHeightfield solid, int tx, int tz, List<NavTileData> output, TileCache? cache)
    {
        // Erosion and the pipeline's own volumes come first and only depend on the geometry, so the cache keeps the result; obstacles are
        // marked on a copy afterwards, with their footprint grown by the agent radius (see TileObstacle).
        CompactHeightfield chf = Regions.Spans.Finish(Regions.Spans.Prepare(solid));
        if (UsesLeanLayers)
        {
            chf.ShareGraph();
        }

        int[]? layers = UsesLayerFirst ? ComputeSpanLayers(chf) : null;
        if (layers is not null && UsesLeanLayers)
        {
            var lean = LeanTile.Create(chf, layers, this, tx, tz);
            lean.BuildFirst(this, cache?.Work ?? new LayerWork(), output);
            cache?.StoreLean(lean);
            return;
        }

        if (cache is not null)
        {
            cache.Store(tx, tz, chf, layers);
            chf = chf.Clone();
        }
        int from = output.Count;
        FinishTile(chf, tx, tz, layers, null, null, output);
        cache?.StoreBase(tx, tz, output, from);
    }

    /// <summary>
    /// The obstacle-dependent half of a tile build, from the cached eroded compact heightfield (which it modifies, so pass a clone): obstacles (the
    /// pipeline's, then <paramref name="extraObstacles"/>) → <paramref name="extraVolumes"/> → regions → contours → polygons → detail → tile data.
    /// </summary>
    internal void FinishTile(CompactHeightfield chf, int tx, int tz, int[]? layers, IReadOnlyList<TileObstacle>? extraObstacles, IReadOnlyList<ConvexVolume>? extraVolumes, List<NavTileData> output)
    {
        MarkObstacles(chf, extraObstacles, extraVolumes);
        FinishMarked(chf, tx, tz, layers, output);
    }

    /// <summary>Marks the pipeline's obstacles, then the extra ones, then applies the extra volumes, on an eroded compact heightfield.</summary>
    internal void MarkObstacles(CompactHeightfield chf, IReadOnlyList<TileObstacle>? extraObstacles, IReadOnlyList<ConvexVolume>? extraVolumes)
    {
        double grow = SpanOptions.WalkableRadius * chf.Frame.CellSize;
        foreach (TileObstacle o in Obstacles)
        {
            o.Mark(chf, grow);
        }
        if (extraObstacles is not null)
        {
            foreach (TileObstacle o in extraObstacles)
            {
                o.Mark(chf, grow);
            }
        }
        if (extraVolumes is not null)
        {
            foreach (ConvexVolume v in extraVolumes)
            {
                v.Apply(chf);
            }
        }
    }

    /// <summary>Regions to tile data of a compact heightfield whose areas are final.</summary>
    internal void FinishMarked(CompactHeightfield chf, int tx, int tz, int[]? spanLayers, List<NavTileData> output)
    {
        int first = output.Count;
        FinishMarkedCore(chf, tx, tz, spanLayers, output);
        // Authored off-mesh links belong to the tile that holds their start, so every build and every rebuild of that tile bakes them in.
        OffMeshLinkBaker.Attach(Params, Settings.OffMeshLinks, output, first);
    }

    private void FinishMarkedCore(CompactHeightfield chf, int tx, int tz, int[]? spanLayers, List<NavTileData> output)
    {
        int[]? regionLayer = null;
        if (spanLayers is not null)
        {
            // Layer-first: regions are the connected same-area spans of one layer; a region's layer is the layer of its spans.
            RegionStage.BuildLayerComponents(chf, spanLayers, Regions.RegionOptions);
            regionLayer = new int[chf.RegionCount + 2];
            ReadOnlySpan<CompactSpan> rs = chf.Spans;
            for (int i = 0; i < rs.Length; i++)
            {
                int r = rs[i].Region;
                if (r > 0)
                {
                    regionLayer[r] = spanLayers[i];
                }
            }
        }
        else
        {
            Regions.Partition(chf, Regions.RegionOptions);
        }
        // Cut the border away: its spans become one extra region whose contours are ignored. The edges between it and the tile's
        // regions are then region-to-region edges, which contour simplification keeps straight between their end vertices, so they
        // stay exactly on the tile boundary.
        Span<CompactSpan> spans = chf.Spans;
        int maxRegion = 0;
        if (regionLayer is not null)
        {
            // The layer-first partition numbers regions 1..RegionCount.
            maxRegion = chf.RegionCount;
        }
        else
        {
            for (int i = 0; i < spans.Length; i++)
            {
                maxRegion = Math.Max(maxRegion, spans[i].Region);
            }
        }
        int borderId = maxRegion + 1;
        HeightfieldFrame f = chf.Frame;
        bool any = false;
        // Spans are stored column by column, so a run of columns is a run of spans: the rows above and below the tile and the strips left and
        // right of it are cut in one go each, and only the tile's own columns are scanned for a region.
        ReadOnlySpan<int> cellFirst = chf.CellFirst;
        int cols = f.ColumnCount;
        int total = spans.Length;
        int inLo = Border, inHi = Border + TileCells;
        for (int z = 0; z < f.Depth; z++)
        {
            int row = z * f.Width;
            if (z < inLo || z >= inHi)
            {
                CutRange(spans, borderId, cellFirst[row], row + f.Width == cols ? total : cellFirst[row + f.Width]);
                continue;
            }
            CutRange(spans, borderId, cellFirst[row], cellFirst[row + inLo]);
            int a0 = cellFirst[row + inLo], a1 = cellFirst[row + inHi];
            for (int i = a0; i < a1; i++)
            {
                if (spans[i].Region > 0)
                {
                    any = true;
                    break;
                }
            }
            CutRange(spans, borderId, a1, row + f.Width == cols ? total : cellFirst[row + f.Width]);
        }
        if (!any)
        {
            return;
        }
        chf.RegionCount = borderId;
        ContourSet contours = Regions.Trace(chf, new ContourSettings { MaxError = Regions.ContourOptions.MaxError, MaxEdgeLength = Regions.ContourOptions.MaxEdgeLength, KeepRawVertices = Regions.ContourOptions.KeepRawVertices, StraightRegionBorders = Regions.ContourOptions.StraightRegionBorders, BorderRegion = borderId });
        PolyMesh mesh = Polygonize(contours, Settings.MaxVertsPerPoly, borderId, new TileCoordinates(Border, TileCells, CellSize, Params.TileSize, Settings.CellHeight, 0));
        if (mesh.PolygonCount == 0)
        {
            return;
        }

        ReadOnlySpan<int> mv = mesh.Vertices;
        int minRow = int.MaxValue, maxRow = int.MinValue;
        for (int i = 0; i < mv.Length; i += 3)
        {
            minRow = Math.Min(minRow, mv[i + 1]);
            maxRow = Math.Max(maxRow, mv[i + 1]);
        }
        double extent = (maxRow - minRow + (2 * (double)(SpanOptions.WalkableClimb + 1))) * Settings.CellHeight;
        if (extent > Params.MaxLocalExtent)
        {
            throw new InvalidOperationException($"Tile ({tx}, {tz}) holds walkable surfaces spanning {extent:R} vertically; the format supports {Params.MaxLocalExtent:R} (16384 cells). Use a larger cell size.");
        }
        var map = new TileCoordinates(Border, TileCells, CellSize, Params.TileSize, Settings.CellHeight, minRow);
        DetailMeshSet detail = BuildDetail(mesh, chf, map, Detail);
        double originY = f.OriginY + (minRow * Settings.CellHeight);

        int nv = mesh.VertexCount;
        var local = new float[nv * 3];
        for (int i = 0; i < nv; i++)
        {
            local[3 * i] = map.Local(mv[3 * i]);
            local[(3 * i) + 1] = map.LocalY(mv[(3 * i) + 1]);
            local[(3 * i) + 2] = map.Local(mv[(3 * i) + 2]);
        }

        int[] layerOfRegion = regionLayer ?? (Settings.Partition == PartitionType.Layers ? AssignLayers(chf, borderId) : Array.Empty<int>());
        int np = mesh.PolygonCount;
        var layerOfPoly = new int[np];
        int layers = 1;
        for (int p = 0; p < np; p++)
        {
            int l = layerOfRegion.Length == 0 ? 0 : layerOfRegion[mesh.Regions[p]];
            layerOfPoly[p] = l;
            layers = Math.Max(layers, l + 1);
        }
        if (layers > NavMeshFormat.MaxLayer + 1)
        {
            throw new InvalidOperationException($"Tile ({tx}, {tz}) needs {layers} layers; at most {NavMeshFormat.MaxLayer + 1} are supported.");
        }
        ReadOnlySpan<int> counts = mesh.PolygonVertexCounts;
        ReadOnlySpan<int> idx = mesh.PolygonIndices;
        var start = new int[np + 1];
        for (int p = 0; p < np; p++)
        {
            start[p + 1] = start[p] + counts[p];
        }
        for (int layer = 0; layer < layers; layer++)
        {
            var remap = new int[nv];
            Array.Fill(remap, -1);
            var lc = new List<int>();
            var li = new List<int>();
            var la = new List<byte>();
            var lf = new List<ushort>();
            var polys = new List<int>();
            for (int p = 0; p < np; p++)
            {
                if (layerOfPoly[p] != layer)
                {
                    continue;
                }
                polys.Add(p);
                lc.Add(counts[p]);
                for (int k = 0; k < counts[p]; k++)
                {
                    remap[idx[start[p] + k]] = 0;
                }
                byte area = mesh.Areas[p];
                la.Add(area == SpanSettings.WalkableArea ? Settings.Area : area);
                lf.Add(Settings.Flags);
            }
            if (polys.Count == 0)
            {
                continue;
            }
            var lv = new List<float>();
            int next = 0;
            for (int v = 0; v < nv; v++)
            {
                if (remap[v] == 0)
                {
                    remap[v] = next++;
                    lv.Add(local[3 * v]);
                    lv.Add(local[(3 * v) + 1]);
                    lv.Add(local[(3 * v) + 2]);
                }
            }
            foreach (int p in polys)
            {
                for (int k = 0; k < counts[p]; k++)
                {
                    li.Add(remap[idx[start[p] + k]]);
                }
            }
            if (next > ushort.MaxValue + 1)
            {
                throw new InvalidOperationException($"Tile ({tx}, {tz}) layer {layer} has {next} vertices; at most 65536 are supported. Use a smaller tile size.");
            }
            NavTileData data = NavTileAssembler.FromLocalPolygons(Params, tx, tz, layer, originY, lv.ToArray(), lc.ToArray(), li.ToArray(), la.ToArray(), lf.ToArray());
            // Detail: copy each polygon's slice, rebased.
            var dm = new DetailMesh[polys.Count];
            var dvs = new List<float>();
            var dts = new List<byte>();
            for (int i = 0; i < polys.Count; i++)
            {
                DetailMesh d = detail.Meshes[polys[i]];
                dm[i] = new DetailMesh { VertexBase = (uint)(dvs.Count / 3), TriangleBase = (uint)(dts.Count / 4), VertexCount = d.VertexCount, TriangleCount = d.TriangleCount };
                for (uint k = 0; k < d.VertexCount * 3u; k++)
                {
                    dvs.Add(detail.Vertices[(int)((d.VertexBase * 3) + k)]);
                }
                for (uint k = 0; k < d.TriangleCount * 4u; k++)
                {
                    dts.Add(detail.Triangles[(int)((d.TriangleBase * 4) + k)]);
                }
            }
            data.DetailMeshes = dm;
            data.DetailVertices = dvs.ToArray();
            data.DetailTriangles = dts.ToArray();
            output.Add(data);
        }
    }

    // Spans [from, to) are outside the tile: those in a region join the border region.
    private static void CutRange(Span<CompactSpan> spans, int borderId, int from, int to)
    {
        for (int i = from; i < to; i++)
        {
            if (spans[i].Region > 0)
            {
                spans[i].Region = borderId;
            }
        }
    }

    private static readonly Action<CompactHeightfield, RegionSettings> s_defaultPartition = RegionStage.Build;
    private static readonly Func<CompactHeightfield, ContourSettings, ContourSet> s_defaultTrace = Contours.Build;
    private static readonly Func<ContourSet, int, int, TileCoordinates, PolyMesh> s_defaultPolygonize = Polygonizer.Build;
    private static readonly Func<PolyMesh, CompactHeightfield, TileCoordinates, DetailSettings, DetailMeshSet> s_defaultDetail = DetailMeshBuilder.Build;

    // The layer-first path needs the default partition step: a replaced step decides regions itself.
    private bool UsesLayerFirst => Settings.Partition == PartitionType.Layers && Regions.RegionOptions.Partition == PartitionType.Layers && Regions.Partition == s_defaultPartition;

    // The layer-first build runs on each layer's own arrays (LayerTileBuilder) when every step it replaces is the default one and the detail stage does not
    // sample heights (it would read the tile's region ids, which the layer builder does not write).
    internal bool UsesLeanLayers =>
        UsesLayerFirst && Regions.Trace == s_defaultTrace && Polygonize == s_defaultPolygonize && BuildDetail == s_defaultDetail && !(Detail.SampleDistance / CellSize > 0.9);

    /// <summary>
    /// Polygons to the tile data of one layer: the vertical origin is the layer's lowest vertex, vertices are the ones its polygons use (in order of first appearance),
    /// areas and flags come from the settings. Null when the polygons need no tile.
    /// </summary>
    internal NavTileData EmitLayerTile(PolyMesh mesh, CompactHeightfield chf, int tx, int tz, int layer, HeightfieldFrame f)
    {
        ReadOnlySpan<int> mv = mesh.Vertices;
        int minRow = int.MaxValue, maxRow = int.MinValue;
        for (int i = 0; i < mv.Length; i += 3)
        {
            minRow = Math.Min(minRow, mv[i + 1]);
            maxRow = Math.Max(maxRow, mv[i + 1]);
        }

        double extent = (maxRow - minRow + (2 * (double)(SpanOptions.WalkableClimb + 1))) * Settings.CellHeight;
        if (extent > Params.MaxLocalExtent)
        {
            throw new InvalidOperationException($"Tile ({tx}, {tz}) holds walkable surfaces spanning {extent:R} vertically; the format supports {Params.MaxLocalExtent:R} (16384 cells). Use a larger cell size.");
        }

        var map = new TileCoordinates(Border, TileCells, CellSize, Params.TileSize, Settings.CellHeight, minRow);
        DetailMeshSet detail = BuildDetail(mesh, chf, map, Detail);
        double originY = f.OriginY + (minRow * Settings.CellHeight);
        int nv = mesh.VertexCount;
        int np = mesh.PolygonCount;
        ReadOnlySpan<int> counts = mesh.PolygonVertexCounts;
        ReadOnlySpan<int> idx = mesh.PolygonIndices;
        var remap = new int[nv];
        Array.Fill(remap, -1);
        for (int k = 0; k < idx.Length; k++)
        {
            remap[idx[k]] = 0;
        }

        int next = 0;
        for (int v = 0; v < nv; v++)
        {
            if (remap[v] == 0)
            {
                remap[v] = next++;
            }
        }

        if (next > ushort.MaxValue + 1)
        {
            throw new InvalidOperationException($"Tile ({tx}, {tz}) layer {layer} has {next} vertices; at most 65536 are supported. Use a smaller tile size.");
        }

        var lv = new float[next * 3];
        for (int v = 0; v < nv; v++)
        {
            int r = remap[v];
            if (r >= 0)
            {
                lv[3 * r] = map.Local(mv[3 * v]);
                lv[(3 * r) + 1] = map.LocalY(mv[(3 * v) + 1]);
                lv[(3 * r) + 2] = map.Local(mv[(3 * v) + 2]);
            }
        }

        var li = new int[idx.Length];
        for (int k = 0; k < idx.Length; k++)
        {
            li[k] = remap[idx[k]];
        }

        var la = new byte[np];
        var lf = new ushort[np];
        for (int p = 0; p < np; p++)
        {
            byte area = mesh.Areas[p];
            la[p] = area == SpanSettings.WalkableArea ? Settings.Area : area;
            lf[p] = Settings.Flags;
        }

        NavTileData data = NavTileAssembler.FromLocalPolygons(Params, tx, tz, layer, originY, lv, counts.ToArray(), li, la, lf);
        data.DetailMeshes = detail.Meshes;
        data.DetailVertices = detail.Vertices;
        data.DetailTriangles = detail.Triangles;
        return data;
    }

    // The layer of every span of the tile, decided once from the geometry-only heightfield (before obstacles): the layer partition with every island kept,
    // then overlapping regions are put on different layers (AssignLayers). Spans that are not walkable get layer 0.
    private static int[] ComputeSpanLayers(CompactHeightfield chf)
    {
        CompactHeightfield work = chf.Clone();
        RegionStage.BuildLayers(work, new RegionSettings { Partition = PartitionType.Layers, MinRegionArea = 0, MergeRegionArea = 0 });
        int[] layerOfRegion = AssignLayers(work, work.RegionCount + 1);
        ReadOnlySpan<CompactSpan> ws = work.Spans;
        var layers = new int[ws.Length];
        for (int i = 0; i < layers.Length; i++)
        {
            int r = ws[i].Region;
            layers[i] = r > 0 ? layerOfRegion[r] : 0;
        }

        // An eroded span (not walkable) goes with a walkable neighbour, so a layer keeps the spans around its floor.
        SpanGraph g = SpanGraph.For(chf);
        for (int i = 0; i < layers.Length; i++)
        {
            if (ws[i].Region > 0)
            {
                continue;
            }

            for (int d = 0; d < 4; d++)
            {
                int t = g.Nbr[(4 * i) + d];
                if (t >= 0 && ws[t].Region > 0)
                {
                    layers[i] = layers[t];
                    break;
                }
            }
        }

        g.Release();
        return layers;
    }

    // Overlapping regions (sharing a column) go to different layers; otherwise a region joins the lowest layer holding a neighbour of it that is free,
    // else the lowest free one.
    private static int[] AssignLayers(CompactHeightfield chf, int borderId)
    {
        HeightfieldFrame f = chf.Frame;
        Span<CompactSpan> spans = chf.Spans;
        ReadOnlySpan<int> cellFirst = chf.CellFirst, cellCount = chf.CellCount;
        var layer = new int[borderId + 1];
        Array.Fill(layer, -1);
        // Spans grouped by region (counting sort, so each group is in span order), and each span's column.
        var start = new int[borderId + 2];
        var spanColumn = new int[spans.Length];
        for (int c = 0; c < f.ColumnCount; c++)
        {
            for (int s = cellFirst[c]; s < cellFirst[c] + cellCount[c]; s++)
            {
                spanColumn[s] = c;
                int r = spans[s].Region;
                if (r > 0 && r != borderId)
                {
                    start[r + 1]++;
                }
            }
        }
        for (int r = 0; r <= borderId; r++)
        {
            start[r + 1] += start[r];
        }
        var fill = new int[borderId + 1];
        var bySpan = new int[start[borderId + 1]];
        for (int s = 0; s < spans.Length; s++)
        {
            int r = spans[s].Region;
            if (r > 0 && r != borderId)
            {
                bySpan[start[r] + fill[r]++] = s;
            }
        }
        // used[l] == r: layer l holds a region sharing a column with region r.
        var used = new int[borderId + 2];
        for (int r = 1; r < borderId; r++)
        {
            int lo = start[r], hi = start[r + 1];
            if (lo == hi)
            {
                continue;
            }
            for (int k = lo; k < hi; k++)
            {
                int c = spanColumn[bySpan[k]];
                for (int s = cellFirst[c]; s < cellFirst[c] + cellCount[c]; s++)
                {
                    int o = spans[s].Region;
                    if (o > 0 && o != borderId && o != r && layer[o] >= 0)
                    {
                        used[layer[o]] = r;
                    }
                }
            }
            int pick = -1;
            for (int k = lo; k < hi; k++)
            {
                int s = bySpan[k];
                int c = spanColumn[s];
                for (int d = 0; d < 4; d++)
                {
                    int link = CompactHeightfield.GetConnection(spans[s], d);
                    if (link == CompactHeightfield.NotConnected)
                    {
                        continue;
                    }
                    int nc = c + CompactHeightfield.DirX[d] + (CompactHeightfield.DirZ[d] * f.Width);
                    int nr = spans[cellFirst[nc] + link].Region;
                    if (nr > 0 && nr != borderId && nr != r && layer[nr] >= 0 && used[layer[nr]] != r && (pick < 0 || layer[nr] < pick))
                    {
                        pick = layer[nr];
                    }
                }
            }
            if (pick < 0)
            {
                pick = 0;
                while (used[pick] == r)
                {
                    pick++;
                }
            }
            layer[r] = pick;
        }
        for (int r = 0; r < layer.Length; r++)
        {
            if (layer[r] < 0)
            {
                layer[r] = 0;
            }
        }
        return layer;
    }
}
