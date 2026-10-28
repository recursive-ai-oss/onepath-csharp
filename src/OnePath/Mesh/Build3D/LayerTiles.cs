using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// One layer of one tile for the layer-first build: the spans of a tile whose layer number is this layer, in the order of the tile's compact
/// heightfield (column by column, so row by row of the grid), with the links between them. Spans of another layer are not linked: each layer is
/// built on its own (docs/build3d-runtime.md, "Layer-first tiles"). The data never changes after it is made; a rebuild only changes areas.
/// </summary>
internal sealed class LayerSpans
{
    public LayerSpans(HeightfieldFrame frame, int layer, int border, int tileCells, CompactSpan[] spans, ReadOnlySpan<int> globalIndex, int[] globalToLocal, int[] globalNbr, int[] spanX, int[] spanZ)
    {
        Frame = frame;
        Layer = layer;
        int n = globalIndex.Length;
        Count = n;
        BaseArea = new byte[n];
        Y = new int[n];
        X = new int[n];
        Z = new int[n];
        Core = new byte[n];
        Nbr = new int[4 * n];
        RowStart = new int[frame.Depth + 1];
        int hi = border + tileCells;
        for (int k = 0; k < n; k++)
        {
            int g = globalIndex[k];
            BaseArea[k] = spans[g].Area;
            Y[k] = spans[g].Y;
            X[k] = spanX[g];
            Z[k] = spanZ[g];
            Core[k] = (byte)(X[k] >= border && X[k] < hi && Z[k] >= border && Z[k] < hi ? 1 : 0);
            RowStart[Z[k] + 1]++;
            for (int d = 0; d < 4; d++)
            {
                int t = globalNbr[(4 * g) + d];
                Nbr[(4 * k) + d] = t >= 0 ? globalToLocal[t] : -1;
            }
        }

        for (int z = 0; z < frame.Depth; z++)
        {
            RowStart[z + 1] += RowStart[z];
        }

        // World box of the walkable spans' cells: what no obstacle or volume outside it can change.
        int minX = int.MaxValue, minZ = int.MaxValue, maxX = -1, maxZ = -1;
        for (int k = 0; k < n; k++)
        {
            if (BaseArea[k] != 0)
            {
                minX = Math.Min(minX, X[k]);
                maxX = Math.Max(maxX, X[k]);
                minZ = Math.Min(minZ, Z[k]);
                maxZ = Math.Max(maxZ, Z[k]);
            }
        }

        if (maxX >= 0)
        {
            WalkableMinX = frame.OriginX + (minX * (double)frame.CellSize);
            WalkableMaxX = frame.OriginX + ((maxX + 1) * (double)frame.CellSize);
            WalkableMinZ = frame.OriginZ + (minZ * (double)frame.CellSize);
            WalkableMaxZ = frame.OriginZ + ((maxZ + 1) * (double)frame.CellSize);
        }
        else
        {
            WalkableMinX = WalkableMinZ = double.PositiveInfinity;
            WalkableMaxX = WalkableMaxZ = double.NegativeInfinity;
        }
    }

    /// <summary>World box (x and z) of the cells that hold a walkable span in the eroded heightfield; empty (min above max) when there is none.</summary>
    public double WalkableMinX { get; }

    /// <summary>See <see cref="WalkableMinX"/>.</summary>
    public double WalkableMaxX { get; }

    /// <summary>See <see cref="WalkableMinX"/>.</summary>
    public double WalkableMinZ { get; }

    /// <summary>See <see cref="WalkableMinX"/>.</summary>
    public double WalkableMaxZ { get; }

    /// <summary>True when the box of an obstacle (grown by <paramref name="reach"/>) or volume can touch a walkable cell of this layer.</summary>
    public bool Overlaps(double minX, double maxX, double minZ, double maxZ, double reach) =>
        maxX + reach >= WalkableMinX && minX - reach <= WalkableMaxX && maxZ + reach >= WalkableMinZ && minZ - reach <= WalkableMaxZ;

    public HeightfieldFrame Frame { get; }

    public int Layer { get; }

    public int Count { get; }

    /// <summary>Areas of the eroded heightfield (0: not walkable).</summary>
    public byte[] BaseArea { get; }

    /// <summary>Floor row of each span.</summary>
    public int[] Y { get; }

    /// <summary>Column x of each span in the build grid.</summary>
    public int[] X { get; }

    /// <summary>Column z of each span in the build grid.</summary>
    public int[] Z { get; }

    /// <summary>1 for a span inside the tile's own cells, 0 for one in the border.</summary>
    public byte[] Core { get; }

    /// <summary>Per span and direction (0 = −x, 1 = +z, 2 = +x, 3 = −z), the linked span of this layer or −1.</summary>
    public int[] Nbr { get; }

    /// <summary>Index of the first span at grid row z (and at <c>Depth</c>, the span count).</summary>
    public int[] RowStart { get; }

    /// <summary>
    /// Splits a tile's eroded compact heightfield into its layers. Links between spans of different layers are dropped.
    /// </summary>
    public static LayerSpans[] Split(CompactHeightfield chf, ReadOnlySpan<int> layerOfSpan, int border, int tileCells)
    {
        SpanGraph g = SpanGraph.For(chf);
        int n = chf.SpanCount;
        int layers = 1;
        for (int i = 0; i < n; i++)
        {
            layers = Math.Max(layers, layerOfSpan[i] + 1);
        }

        var count = new int[layers];
        for (int i = 0; i < n; i++)
        {
            count[layerOfSpan[i]]++;
        }

        HeightfieldFrame f = chf.Frame;
        var spanX = new int[n];
        var spanZ = new int[n];
        ReadOnlySpan<int> first = chf.CellFirst, cnt = chf.CellCount;
        for (int c = 0; c < first.Length; c++)
        {
            int x = c % f.Width, z = c / f.Width;
            for (int s = first[c]; s < first[c] + cnt[c]; s++)
            {
                spanX[s] = x;
                spanZ[s] = z;
            }
        }

        var local = new int[n];
        var fill = new int[layers];
        var members = new int[layers][];
        for (int l = 0; l < layers; l++)
        {
            members[l] = new int[count[l]];
        }

        for (int i = 0; i < n; i++)
        {
            int l = layerOfSpan[i];
            local[i] = fill[l];
            members[l][fill[l]++] = i;
        }

        // A link joins two spans of one layer only.
        int[] nbr = g.Nbr;
        var kept = new int[4 * n];
        for (int i = 0; i < n; i++)
        {
            for (int d = 0; d < 4; d++)
            {
                int t = nbr[(4 * i) + d];
                kept[(4 * i) + d] = t >= 0 && layerOfSpan[t] == layerOfSpan[i] ? t : -1;
            }
        }

        var result = new LayerSpans[layers];
        CompactSpan[] spans = chf.SpansArray;
        for (int l = 0; l < layers; l++)
        {
            result[l] = new LayerSpans(f, l, border, tileCells, spans, members[l], local, kept, spanX, spanZ);
        }

        g.Release();
        return result;
    }

    /// <summary>Sets area 0 on every walkable span this obstacle covers (see <see cref="TileObstacle.Mark"/>).</summary>
    public void Mark(TileObstacle o, double grow, byte[] area)
    {
        HeightfieldFrame f = Frame;
        double reach = grow + f.CellSize;
        if (!RowRange(f.OriginZ, o.MinZ - reach, o.MaxZ + reach, f.Depth, f.CellSize, out int z0, out int z1))
        {
            return;
        }

        double xLo = o.MinX - reach, xHi = o.MaxX + reach;
        for (int z = z0; z <= z1; z++)
        {
            double cz = f.CenterZ(z);
            if (cz < o.MinZ - reach || cz > o.MaxZ + reach)
            {
                continue;
            }

            for (int i = RowStart[z], end = RowStart[z + 1]; i < end; i++)
            {
                if (area[i] == 0)
                {
                    continue;
                }

                double cx = f.CenterX(X[i]);
                if (cx < xLo || cx > xHi || !o.ContainsGrown(cx, cz, grow))
                {
                    continue;
                }

                double wy = f.WorldY(Y[i]);
                if (wy >= o.MinY && wy <= o.MaxY)
                {
                    area[i] = 0;
                }
            }
        }
    }

    /// <summary>Sets the volume's area on every walkable span it covers (see <see cref="ConvexVolume.Apply"/>).</summary>
    public void Apply(ConvexVolume v, byte[] area)
    {
        HeightfieldFrame f = Frame;
        if (!RowRange(f.OriginZ, v.MinZ - f.CellSize, v.MaxZ + f.CellSize, f.Depth, f.CellSize, out int z0, out int z1))
        {
            return;
        }

        for (int z = z0; z <= z1; z++)
        {
            double cz = f.CenterZ(z);
            if (cz < v.MinZ || cz > v.MaxZ)
            {
                continue;
            }

            for (int i = RowStart[z], end = RowStart[z + 1]; i < end; i++)
            {
                if (area[i] == 0)
                {
                    continue;
                }

                double cx = f.CenterX(X[i]);
                if (cx < v.MinX || cx > v.MaxX || !v.Contains(cx, cz))
                {
                    continue;
                }

                double wy = f.WorldY(Y[i]);
                if (wy >= v.MinY && wy <= v.MaxY)
                {
                    area[i] = v.Area;
                }
            }
        }
    }

    // The grid rows whose centres can lie in [lo, hi] (a row beyond either end by one is included; callers test the centre).
    private static bool RowRange(double origin, double lo, double hi, int depth, double cell, out int z0, out int z1)
    {
        double a = Math.Floor((lo - origin) / cell) - 1, b = Math.Ceiling((hi - origin) / cell) + 1;
        z0 = (int)Math.Max(0, a);
        z1 = (int)Math.Min(depth - 1, b);
        return a <= depth && b >= 0 && z0 <= z1;
    }
}

/// <summary>Reusable buffers of the layer builder; one per thread of work, not shared.</summary>
internal sealed class LayerWork
{
    public int[] Lab = Array.Empty<int>();
    public int[] Region = Array.Empty<int>();
    public byte[] Visited = Array.Empty<byte>();
    public int[] Parent = Array.Empty<int>();
    public int[] Size = Array.Empty<int>();
    public int[] Map = Array.Empty<int>();
    public byte[] HasNeighbor = Array.Empty<byte>();
    public byte[] Area = Array.Empty<byte>();
    public ContourVertex[] Loop = new ContourVertex[1024];

    public void Ensure(int n)
    {
        if (Lab.Length < n)
        {
            int m = Math.Max(n, Lab.Length * 2);
            Lab = new int[m];
            Region = new int[m];
            Visited = new byte[m];
            Parent = new int[m + 1];
            Size = new int[m + 1];
            Map = new int[m + 1];
            HasNeighbor = new byte[m + 1];
            Area = new byte[m];
        }
    }
}

/// <summary>
/// The layer-first tile build for one layer: connected same-area spans become regions (numbered in order of their first span), small isolated
/// regions are dropped, the border is cut away, contours are traced and simplified, then polygons, detail and the tile are made.
/// The stages are those of <see cref="Mesh3DPipeline"/> with the layer partition; this class does them on the layer's own arrays.
/// </summary>
internal static class LayerTileBuilder
{
    /// <summary>
    /// Builds the tile of one layer from the areas in <paramref name="area"/>. Returns null when the layer has no walkable region inside the tile.
    /// </summary>
    public static NavTileData? Build(Mesh3DPipeline p, LayerSpans ls, byte[] area, LayerWork w, CompactHeightfield chf, int tx, int tz)
    {
        int n = ls.Count;
        if (n == 0)
        {
            return null;
        }

        w.Ensure(n);
        int[] nbr = ls.Nbr;
        int[] lab = w.Lab;
        int[] parent = w.Parent;
        int[] size = w.Size;

        // Regions: union-find over the links to the previous column (−x) and the previous row (−z) between spans of one area. Provisional labels are
        // numbered in order of first span and joined towards the smaller label, so the smallest label of a region belongs to its first span.
        int next = 0;
        for (int i = 0; i < n; i++)
        {
            byte a = area[i];
            if (a == 0)
            {
                lab[i] = 0;
                continue;
            }

            int r = 0;
            int l = nbr[4 * i];
            if (l >= 0 && area[l] == a)
            {
                r = lab[l];
            }

            int u = nbr[(4 * i) + 3];
            if (u >= 0 && area[u] == a)
            {
                int ru = lab[u];
                if (r == 0)
                {
                    r = ru;
                }
                else if (r != ru)
                {
                    r = Union(parent, r, ru);
                }
            }

            if (r == 0)
            {
                r = ++next;
                parent[r] = r;
                size[r] = 0;
            }

            lab[i] = r;
            size[r]++;
        }

        if (next == 0)
        {
            return null;
        }

        // Region sizes by root, roots in ascending label order.
        for (int l = 1; l <= next; l++)
        {
            int root = Find(parent, l);
            if (root != l)
            {
                size[root] += size[l];
            }
        }

        int minArea = p.Regions.RegionOptions.MinRegionArea;
        int[] map = w.Map;
        byte[] hasNeighbor = w.HasNeighbor;
        bool anySmall = false;
        if (minArea > 0)
        {
            for (int l = 1; l <= next; l++)
            {
                hasNeighbor[l] = 0;
                if (parent[l] == l && size[l] < minArea)
                {
                    anySmall = true;
                }
            }
        }

        if (anySmall)
        {
            // A small region goes only if no walkable span of another area touches it.
            for (int i = 0; i < n; i++)
            {
                if (area[i] == 0)
                {
                    continue;
                }

                int root = Find(parent, lab[i]);
                if (size[root] >= minArea || hasNeighbor[root] != 0)
                {
                    continue;
                }

                for (int d = 0; d < 4; d++)
                {
                    int t = nbr[(4 * i) + d];
                    if (t >= 0 && area[t] != 0 && area[t] != area[i])
                    {
                        hasNeighbor[root] = 1;
                        break;
                    }
                }
            }
        }

        int count = 0;
        for (int l = 1; l <= next; l++)
        {
            if (parent[l] == l)
            {
                map[l] = minArea > 0 && size[l] < minArea && hasNeighbor[l] == 0 ? 0 : ++count;
            }
        }

        for (int l = 1; l <= next; l++)
        {
            if (parent[l] != l)
            {
                map[l] = map[Find(parent, l)];
            }
        }

        if (count == 0)
        {
            return null;
        }

        int borderId = count + 1;
        int[] region = w.Region;
        byte[] core = ls.Core;
        bool any = false;
        var regionArea = new byte[borderId + 1];
        for (int i = 0; i < n; i++)
        {
            int r = lab[i] == 0 ? 0 : map[lab[i]];
            if (r > 0)
            {
                if (core[i] != 0)
                {
                    any = true;
                    regionArea[r] = area[i];
                }
                else
                {
                    r = borderId;
                }
            }

            region[i] = r;
        }

        if (!any)
        {
            return null;
        }

        // Contours: loops of the unit boundary edges of every region (a boundary edge has the region on its left).
        var outers = new List<(ContourVertex[] Raw, long Area)>?[borderId + 1];
        var holes = new List<ContourVertex[]>?[borderId + 1];
        byte[] visited = w.Visited;
        Array.Clear(visited, 0, n);
        int guard = (4 * n) + 8;
        int[] sx = ls.X, sz = ls.Z, sy = ls.Y;
        int start = ls.RowStart[p.Border], stop = ls.RowStart[p.Border + p.TileCells];
        for (int i = start; i < stop; i++)
        {
            int r = region[i];
            if (r <= 0 || r == borderId)
            {
                continue;
            }

            int b = 4 * i;
            int t0 = nbr[b], t1 = nbr[b + 1], t2 = nbr[b + 2], t3 = nbr[b + 3];
            if (t0 >= 0 && t1 >= 0 && t2 >= 0 && t3 >= 0 && region[t0] == r && region[t1] == r && region[t2] == r && region[t3] == r)
            {
                continue;
            }

            for (int d = 0; d < 4; d++)
            {
                if ((visited[i] & (1 << d)) != 0)
                {
                    continue;
                }

                int tn = nbr[b + d];
                if (tn >= 0 && region[tn] == r)
                {
                    continue;
                }

                ContourVertex[] raw = Trace(nbr, region, sx, sz, sy, visited, i, d, guard, w);
                long a2 = Contours.SignedArea2(raw);
                if (a2 > 0)
                {
                    (outers[r] ??= new List<(ContourVertex[], long)>()).Add((raw, a2));
                }
                else
                {
                    (holes[r] ??= new List<ContourVertex[]>()).Add(raw);
                }
            }
        }

        ContourSet contours = Contours.Assemble(ls.Frame, borderId, regionArea, outers, holes, new ContourSettings { MaxError = p.Regions.ContourOptions.MaxError, MaxEdgeLength = p.Regions.ContourOptions.MaxEdgeLength, KeepRawVertices = p.Regions.ContourOptions.KeepRawVertices, StraightRegionBorders = p.Regions.ContourOptions.StraightRegionBorders, BorderRegion = borderId });
        PolyMesh mesh = p.Polygonize(contours, p.Settings.MaxVertsPerPoly, borderId, new TileCoordinates(p.Border, p.TileCells, p.CellSize, p.Params.TileSize, p.Settings.CellHeight, 0));
        if (mesh.PolygonCount == 0)
        {
            return null;
        }

        return p.EmitLayerTile(mesh, chf, tx, tz, ls.Layer, ls.Frame);
    }

    // Walks one boundary loop with the region on the left. State (span, dir) is "at the start of edge dir of span".
    private static ContourVertex[] Trace(int[] nbr, int[] region, int[] sx, int[] sz, int[] sy, byte[] visited, int start, int startDir, int guard, LayerWork w)
    {
        ContourVertex[] loop = w.Loop;
        int count = 0;
        int s = start;
        int dir = startDir;
        int r = region[start];
        int steps = 0;
        do
        {
            if (++steps > guard)
            {
                throw new InvalidOperationException("Contour tracing did not close; the layer's links are inconsistent.");
            }

            int t = nbr[(4 * s) + dir];
            int nv = t >= 0 ? region[t] : 0;
            if (nv != r)
            {
                visited[s] |= (byte)(1 << dir);
                int x = sx[s];
                int z = sz[s];
                // Start corner of the edge: bottom (3) at (x,z), right (2) at (x+1,z), top (1) at (x+1,z+1), left (0) at (x,z+1).
                int cx = dir == 2 || dir == 1 ? x + 1 : x;
                int cz = dir == 1 || dir == 0 ? z + 1 : z;
                if (count == loop.Length)
                {
                    Array.Resize(ref loop, count * 2);
                    w.Loop = loop;
                }

                loop[count++] = new ContourVertex(cx, CornerHeight(nbr, sy, s, dir), cz, nv);
                dir = (dir + 3) & 3;
            }
            else
            {
                s = t;
                dir = (dir + 1) & 3;
            }
        }
        while (s != start || dir != startDir);
        var result = new ContourVertex[count];
        Array.Copy(loop, result, count);
        return result;
    }

    // Highest floor among the spans around the start corner of edge dir: this span, the two linked across the corner's edges, and the diagonal one.
    private static int CornerHeight(int[] nbr, int[] sy, int s, int dir)
    {
        int h = sy[s];
        int d2 = (dir + 1) & 3;
        int a = nbr[(4 * s) + dir];
        int b = nbr[(4 * s) + d2];
        if (a >= 0)
        {
            h = Math.Max(h, sy[a]);
            int diag = nbr[(4 * a) + d2];
            if (diag >= 0)
            {
                h = Math.Max(h, sy[diag]);
            }
        }

        if (b >= 0)
        {
            h = Math.Max(h, sy[b]);
            int diag = nbr[(4 * b) + dir];
            if (diag >= 0)
            {
                h = Math.Max(h, sy[diag]);
            }
        }

        return h;
    }

    // The smaller label is the root, so the root of a region is the label of its first span.
    private static int Find(int[] parent, int i)
    {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }

        return i;
    }

    private static int Union(int[] parent, int a, int b)
    {
        a = Find(parent, a);
        b = Find(parent, b);
        if (a < b)
        {
            parent[b] = a;
            return a;
        }

        if (b < a)
        {
            parent[a] = b;
        }

        return b;
    }
}

/// <summary>
/// A tile built layer by layer (docs/build3d-runtime.md, "Layer-first tiles"): its layers, and what a tile cache remembers per layer — the areas and tile
/// the layer was last built with, and the first build's areas and tile.
/// </summary>
internal sealed class LeanTile
{
    private LeanTile(CompactHeightfield chf, LayerSpans[] layers, int tx, int tz)
    {
        Chf = chf;
        Layers = layers;
        TileX = tx;
        TileZ = tz;
        LastArea = new byte[layers.Length][];
        LastData = new NavTileData?[layers.Length];
        BaseArea = new byte[layers.Length][];
        BaseData = new NavTileData?[layers.Length];
    }

    public CompactHeightfield Chf { get; }

    public LayerSpans[] Layers { get; }

    public int TileX { get; }

    public int TileZ { get; }

    /// <summary>Per layer, the areas of the tile in the mesh (null: unknown, e.g. after a restore).</summary>
    public byte[]?[] LastArea { get; }

    /// <summary>Per layer, the tile data the mesh holds for it (null: no tile).</summary>
    public NavTileData?[] LastData { get; }

    /// <summary>Per layer, the areas of the first build.</summary>
    public byte[][] BaseArea { get; }

    /// <summary>Per layer, the tile data of the first build (null: no tile).</summary>
    public NavTileData?[] BaseData { get; }

    /// <summary>Splits a tile's eroded heightfield into layers.</summary>
    public static LeanTile Create(CompactHeightfield chf, int[] spanLayers, Mesh3DPipeline p, int tx, int tz)
    {
        LayerSpans[] layers = LayerSpans.Split(chf, spanLayers, p.Border, p.TileCells);
        if (layers.Length > NavMeshFormat.MaxLayer + 1)
        {
            throw new InvalidOperationException($"Tile ({tx}, {tz}) needs {layers.Length} layers; at most {NavMeshFormat.MaxLayer + 1} are supported.");
        }

        return new LeanTile(chf, layers, tx, tz);
    }

    /// <summary>
    /// Fills <c>w.Area</c> with the areas of a layer: the first build's areas (the pipeline's own obstacles are in them), then the extra obstacles, then the volumes.
    /// Returns false, leaving <c>w.Area</c> alone, when nothing extra can touch the layer (its areas are the first build's).
    /// </summary>
    public bool Mark(Mesh3DPipeline p, int layer, IReadOnlyList<TileObstacle>? extraObstacles, IReadOnlyList<ConvexVolume>? extraVolumes, LayerWork w)
    {
        LayerSpans ls = Layers[layer];
        double grow = p.SpanOptions.WalkableRadius * ls.Frame.CellSize;
        double reach = grow + ls.Frame.CellSize;
        bool touched = false;
        if (extraObstacles is not null)
        {
            foreach (TileObstacle o in extraObstacles)
            {
                touched |= ls.Overlaps(o.MinX, o.MaxX, o.MinZ, o.MaxZ, reach);
            }
        }

        if (extraVolumes is not null)
        {
            foreach (ConvexVolume v in extraVolumes)
            {
                touched |= ls.Overlaps(v.MinX, v.MaxX, v.MinZ, v.MaxZ, 0);
            }
        }

        if (!touched)
        {
            return false;
        }

        w.Ensure(ls.Count);
        byte[] area = w.Area;
        Array.Copy(BaseArea[layer], area, ls.Count);
        if (extraObstacles is not null)
        {
            foreach (TileObstacle o in extraObstacles)
            {
                if (ls.Overlaps(o.MinX, o.MaxX, o.MinZ, o.MaxZ, reach))
                {
                    ls.Mark(o, grow, area);
                }
            }
        }

        if (extraVolumes is not null)
        {
            foreach (ConvexVolume v in extraVolumes)
            {
                if (ls.Overlaps(v.MinX, v.MaxX, v.MinZ, v.MaxZ, 0))
                {
                    ls.Apply(v, area);
                }
            }
        }

        return true;
    }

    /// <summary>The pipeline's own obstacles marked on a layer, into <c>w.Area</c>: the areas of the first build.</summary>
    private void MarkFirst(Mesh3DPipeline p, int layer, LayerWork w)
    {
        LayerSpans ls = Layers[layer];
        w.Ensure(ls.Count);
        Array.Copy(ls.BaseArea, w.Area, ls.Count);
        double grow = p.SpanOptions.WalkableRadius * ls.Frame.CellSize;
        foreach (TileObstacle o in p.Obstacles)
        {
            ls.Mark(o, grow, w.Area);
        }
    }

    /// <summary>True when the two tiles hold the same polygons, detail and links.</summary>
    public static bool SameContent(NavTileData a, NavTileData b) =>
        a.TileX == b.TileX && a.TileZ == b.TileZ && a.Layer == b.Layer && a.OriginY == b.OriginY
        && (a.OffMeshLinks is null or { Length: 0 }) && (b.OffMeshLinks is null or { Length: 0 })
        && Same(a.Vertices, b.Vertices) && Same(a.PolyVertexCounts, b.PolyVertexCounts) && Same(a.PolyIndices, b.PolyIndices)
        && Same(a.EdgeNeighbors, b.EdgeNeighbors) && Same(a.EdgePortalSides, b.EdgePortalSides) && Same(a.PolyAreas, b.PolyAreas) && Same(a.PolyFlags, b.PolyFlags)
        && Same(a.DetailMeshes, b.DetailMeshes) && Same(a.DetailVertices, b.DetailVertices) && Same(a.DetailTriangles, b.DetailTriangles);

    private static bool Same<T>(T[]? a, T[]? b) where T : IEquatable<T> =>
        (a is null or { Length: 0 } && b is null or { Length: 0 }) || (a is not null && b is not null && a.AsSpan().SequenceEqual(b));

    /// <summary>The first build: every layer with the pipeline's own obstacles; the result is also the tile's base.</summary>
    public void BuildFirst(Mesh3DPipeline p, LayerWork w, List<NavTileData> output)
    {
        int from = output.Count;
        for (int l = 0; l < Layers.Length; l++)
        {
            MarkFirst(p, l, w);
            NavTileData? d = LayerTileBuilder.Build(p, Layers[l], w.Area, w, Chf, TileX, TileZ);
            BaseArea[l] = LastArea[l] = w.Area.AsSpan(0, Layers[l].Count).ToArray();
            BaseData[l] = LastData[l] = d;
            if (d is not null)
            {
                output.Add(d);
            }
        }

        // Authored off-mesh links belong to the tile that holds their start, so every build and every rebuild of that tile bakes them in.
        OffMeshLinkBaker.Attach(p.Params, p.Settings.OffMeshLinks, output, from);
    }
}
