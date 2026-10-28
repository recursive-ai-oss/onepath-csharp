using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// The span adjacency of a compact heightfield as flat arrays: neighbour span index per direction, or -1. A neighbour is recorded only when the
/// link is mutual (the target links back to the span): links choose the lowest qualifying span of a cell, so with several floors in a cell a link can
/// be one-way, and boundary tracing needs a symmetric relation to close its loops.
/// </summary>
internal sealed class SpanGraph
{
    private readonly CompactHeightfield _chf;
    private int[]? _cx;
    private int[]? _cz;
    private bool _shared;

    /// <summary>
    /// The graph of <paramref name="chf"/>: the one kept with it by <see cref="CompactHeightfield.ShareGraph"/> (the links do not depend on areas or regions, so
    /// every copy of a cached heightfield uses it and nothing is rebuilt), else a new one that the caller releases.
    /// </summary>
    public static SpanGraph For(CompactHeightfield chf) => chf.SharedGraph ?? new SpanGraph(chf);

    /// <summary>Builds the graph once for a heightfield whose links stay as they are, with the span-to-column table, and never returns its arrays to the pool.</summary>
    public static SpanGraph CreateShared(CompactHeightfield chf)
    {
        var g = new SpanGraph(chf) { _shared = true };
        g.EnsureCells();
        _ = g.MultiSpanColumns;
        return g;
    }

    public SpanGraph(CompactHeightfield chf)
    {
        _chf = chf;
        Init(chf);
        Nbr = SpanLinks.Rent(chf, out List<int>? oneWay);
        Drop(oneWay);
    }

    /// <summary>
    /// Builds the graph from the raw links of <paramref name="chf"/> (see <see cref="SpanLinks.Rent(CompactHeightfield, out List{int})"/>) and the one-way entries found
    /// with them. The graph takes ownership of the (pooled, at least <c>4 * SpanCount</c> long) array and rewrites it in place; <see cref="Release"/> gives it back.
    /// </summary>
    public SpanGraph(CompactHeightfield chf, int[] rawLinks, List<int>? oneWay)
    {
        _chf = chf;
        Init(chf);
        Nbr = rawLinks;
        Drop(oneWay);
    }

    private void Init(CompactHeightfield chf)
    {
        HeightfieldFrame f = chf.Frame;
        Width = f.Width;
        Depth = f.Depth;
        Count = chf.SpanCount;
    }

    // A link is kept only when the target links back: boundary tracing needs a symmetric relation to close its loops.
    private void Drop(List<int>? oneWay)
    {
        if (oneWay is null)
        {
            return;
        }
        foreach (int k in oneWay)
        {
            Nbr[k] = -1;
        }
    }

    /// <summary>Returns the pooled arrays; the graph must not be used afterwards.</summary>
    public void Release()
    {
        if (_shared)
        {
            return;
        }
        Scratch.Return(Nbr);
        if (_cx != null)
        {
            Scratch.Return(_cx);
            Scratch.Return(_cz!);
        }
    }

    public int Width { get; private set; }

    public int Depth { get; private set; }

    public int Count { get; private set; }

    public int[] Nbr { get; }

    private int[]? _multi;

    /// <summary>The columns holding more than one span (walkable or not), in column order.</summary>
    public int[] MultiSpanColumns
    {
        get
        {
            if (_multi == null)
            {
                var list = new List<int>();
                ReadOnlySpan<int> count = _chf.CellCount;
                for (int c = 0; c < count.Length; c++)
                {
                    if (count[c] > 1)
                    {
                        list.Add(c);
                    }
                }
                _multi = list.ToArray();
            }
            return _multi;
        }
    }

    /// <summary>Column x of a span.</summary>
    public int CellX(int span)
    {
        EnsureCells();
        return _cx![span];
    }

    /// <summary>Column z of a span.</summary>
    public int CellZ(int span)
    {
        EnsureCells();
        return _cz![span];
    }

    private void EnsureCells()
    {
        if (_cx != null)
        {
            return;
        }
        int[] cx = Scratch.Rent<int>(Count);
        int[] cz = Scratch.Rent<int>(Count);
        ReadOnlySpan<int> first = _chf.CellFirst;
        ReadOnlySpan<int> count = _chf.CellCount;
        int w = Width;
        for (int c = 0, x = 0, z = 0; c < count.Length; c++)
        {
            int end = first[c] + count[c];
            for (int i = first[c]; i < end; i++)
            {
                cx[i] = x;
                cz[i] = z;
            }
            if (++x == w)
            {
                x = 0;
                z++;
            }
        }
        _cz = cz;
        _cx = cx;
    }

    public bool OutsideGrid(int span, int dir)
    {
        int x = CellX(span) + CompactHeightfield.DirX[dir];
        int z = CellZ(span) + CompactHeightfield.DirZ[dir];
        return x < 0 || z < 0 || x >= Width || z >= Depth;
    }
}

/// <summary>
/// Partitions the walkable spans of a compact heightfield into regions: watershed on the distance field (Vincent &amp; Soille 1991
/// style flooding from the farthest points), a monotone sweep (de Berg et al., monotone partitioning), or sweep regions merged into
/// layers. Region ids start at 1 and are written to <see cref="CompactSpan.Region"/>; 0 means "no region" (non-walkable spans and removed islands).
/// Definitions are in docs/build3d-regions.md.
/// </summary>
public static class Regions
{
    /// <summary>
    /// The distance field: for every span the exact shortest-path distance to a boundary span (see <see cref="Erosion"/>), in units where an
    /// orthogonal step costs 2 and a diagonal 3. Unbounded, so large open areas get large values.
    /// </summary>
    /// <param name="chf">The compact heightfield.</param>
    /// <param name="maxDistance">Receives the largest distance.</param>
    public static int[] BuildDistanceField(CompactHeightfield chf, out int maxDistance)
    {
        ArgumentNullException.ThrowIfNull(chf);
        int[] dist = Erosion.ComputeBoundaryDistance(chf, 1 << 28);
        maxDistance = 0;
        foreach (int d in dist)
        {
            maxDistance = Math.Max(maxDistance, d);
        }
        return dist;
    }

    /// <summary>Partitions with the method and thresholds in <paramref name="settings"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">Invalid settings.</exception>
    public static void Build(CompactHeightfield chf, RegionSettings settings)
    {
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        switch (settings.Partition)
        {
            case PartitionType.Watershed:
                BuildWatershed(chf, settings);
                break;
            case PartitionType.Monotone:
                BuildMonotone(chf, settings);
                break;
            default:
                BuildLayers(chf, settings);
                break;
        }
    }

    /// <summary>
    /// Watershed partition. Spans are visited from the largest distance down. At each level, regions already formed grow into
    /// the spans of that level that touch them (in synchronous rounds, taking the smallest neighbouring region id, same area only),
    /// then each remaining unassigned span starts a new region that floods its connected unassigned spans of the level. Then small regions are filtered.
    /// </summary>
    public static void BuildWatershed(CompactHeightfield chf, RegionSettings settings)
    {
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        int n = chf.SpanCount;
        int[] links = SpanLinks.Rent(chf, out List<int>? oneWay);
        int[] dist = Scratch.Rent<int>(n);
        Erosion.DistanceInto(chf, links, dist, 1 << 28);
        var g = new SpanGraph(chf, links, oneWay);
        Span<CompactSpan> spans = chf.Spans;
        int maxDist = 0;
        for (int i = 0; i < n; i++)
        {
            maxDist = Math.Max(maxDist, dist[i]);
        }
        int[] region = Scratch.RentZeroed<int>(n);
        byte[] areaOf = Scratch.Rent<byte>(n);
        byte firstArea = 0;
        bool singleArea = true;
        for (int i = 0; i < n; i++)
        {
            byte a = spans[i].Area;
            areaOf[i] = a;
            if (a != 0)
            {
                if (firstArea == 0)
                {
                    firstArea = a;
                }
                else if (a != firstArea)
                {
                    singleArea = false;
                }
            }
        }

        // Counting sort of walkable spans by distance, ascending index inside a level.
        var start = new int[maxDist + 2];
        for (int i = 0; i < n; i++)
        {
            if (areaOf[i] != 0)
            {
                start[dist[i] + 1]++;
            }
        }
        for (int l = 0; l <= maxDist; l++)
        {
            start[l + 1] += start[l];
        }
        var fill = new int[maxDist + 1];
        int[] order = Scratch.Rent<int>(start[maxDist + 1]);
        for (int i = 0; i < n; i++)
        {
            if (areaOf[i] != 0)
            {
                order[start[dist[i]] + fill[dist[i]]++] = i;
            }
        }

        int next = 0;
        int[] mark = Scratch.RentZeroed<int>(n);
        int stamp = 0;
        int[] nbr = g.Nbr;
        // The frontier and the round's assignments are plain arrays sized to the largest level; a queue of the same size serves the basin floods.
        int widest = 0;
        for (int l = 0; l <= maxDist; l++)
        {
            widest = Math.Max(widest, start[l + 1] - start[l]);
        }
        var frontier = new int[widest];
        var assignSpan = new int[widest];
        var assignRegion = new int[widest];
        var queue = new int[widest];
        // Raw accesses below: every span index comes from order or from a graph entry that is -1 or below n, and the arrays hold n entries; the
        // frontier, assignment and queue arrays hold the widest level, which bounds every count written to them.
        for (int level = maxDist; level >= 0; level--)
        {
            int lo = start[level];
            int hi = start[level + 1];
            if (lo == hi)
            {
                continue;
            }
            // Round 1: every span of this level that touches a region. Later rounds: the level's spans next to one assigned in the round before.
            ReadOnlySpan<int> current = order.AsSpan(lo, hi - lo);
            int remaining = hi - lo;
            while (current.Length > 0)
            {
                // Rounds are synchronous: every choice is made before any is applied, so the order inside the frontier does not matter.
                int ac = 0;
                for (int k = 0; k < current.Length; k++)
                {
                    int s = current[k];
                    int b = 4 * s;
                    int best = 0;
                    if (singleArea)
                    {
                        // One area id: a neighbour with a region has it too.
                        int t = Raw.At(nbr, b);
                        if (t >= 0)
                        {
                            best = Raw.At(region, t);
                        }
                        t = Raw.At(nbr, b + 1);
                        if (t >= 0)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                        t = Raw.At(nbr, b + 2);
                        if (t >= 0)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                        t = Raw.At(nbr, b + 3);
                        if (t >= 0)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                    }
                    else
                    {
                        byte a = Raw.At(areaOf, s);
                        int t = Raw.At(nbr, b);
                        if (t >= 0 && Raw.At(areaOf, t) == a)
                        {
                            best = Raw.At(region, t);
                        }
                        t = Raw.At(nbr, b + 1);
                        if (t >= 0 && Raw.At(areaOf, t) == a)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                        t = Raw.At(nbr, b + 2);
                        if (t >= 0 && Raw.At(areaOf, t) == a)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                        t = Raw.At(nbr, b + 3);
                        if (t >= 0 && Raw.At(areaOf, t) == a)
                        {
                            int rt = Raw.At(region, t);
                            if (rt > 0 && (best == 0 || rt < best))
                            {
                                best = rt;
                            }
                        }
                    }
                    if (best > 0)
                    {
                        Raw.At(assignSpan, ac) = s;
                        Raw.At(assignRegion, ac) = best;
                        ac++;
                    }
                }
                for (int k = 0; k < ac; k++)
                {
                    Raw.At(region, Raw.At(assignSpan, k)) = Raw.At(assignRegion, k);
                }
                remaining -= ac;
                if (remaining == 0)
                {
                    // Every span of the level has a region: nothing to grow into and no basin to start.
                    break;
                }
                // Next round: unassigned spans of this level next to a span assigned just now.
                stamp++;
                int nc = 0;
                for (int k = 0; k < ac; k++)
                {
                    int s = Raw.At(assignSpan, k);
                    for (int d = 0; d < 4; d++)
                    {
                        int t = Raw.At(nbr, (4 * s) + d);
                        if (t >= 0 && Raw.At(region, t) == 0 && Raw.At(areaOf, t) != 0 && Raw.At(dist, t) == level && Raw.At(mark, t) != stamp)
                        {
                            Raw.At(mark, t) = stamp;
                            Raw.At(frontier, nc++) = t;
                        }
                    }
                }
                current = frontier.AsSpan(0, nc);
            }
            // New basins: what is left of this level.
            for (int k = lo; remaining > 0 && k < hi; k++)
            {
                int s0 = order[k];
                if (Raw.At(region, s0) != 0)
                {
                    continue;
                }
                int id = ++next;
                Raw.At(region, s0) = id;
                int qh = 0, qt = 0;
                Raw.At(queue, qt++) = s0;
                byte area0 = Raw.At(areaOf, s0);
                while (qh < qt)
                {
                    int s = Raw.At(queue, qh++);
                    for (int d = 0; d < 4; d++)
                    {
                        int t = Raw.At(nbr, (4 * s) + d);
                        if (t >= 0 && Raw.At(region, t) == 0 && Raw.At(areaOf, t) == area0 && Raw.At(dist, t) == level)
                        {
                            Raw.At(region, t) = id;
                            Raw.At(queue, qt++) = t;
                        }
                    }
                }
            }
        }
        Finish(chf, g, region, next, settings.MinRegionArea, settings.MergeRegionArea);
        g.Release();
        Scratch.Return(dist);
        Scratch.Return(region);
        Scratch.Return(areaOf);
        Scratch.Return(order);
        Scratch.Return(mark);
    }

    /// <summary>
    /// Monotone partition: row by row (increasing z), spans chained along x with a same-area link form a sweep; a sweep continues the one region of
    /// the previous row it touches when it touches only that region and no other sweep of the row touches it, otherwise it starts a new region.
    /// With no filtering, a region has at most one sweep per row, so every row of it is one run along x.
    /// </summary>
    public static void BuildMonotone(CompactHeightfield chf, RegionSettings settings)
    {
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        var g = SpanGraph.For(chf);
        int count = Sweep(chf, g, out int[] region);
        Finish(chf, g, region, count, settings.MinRegionArea, settings.MergeRegionArea);
        g.Release();
        Scratch.Return(region);
    }

    /// <summary>
    /// Layer partition: monotone regions, then adjacent same-area regions are merged, lowest id first, whenever no column holds spans of both,
    /// so a layer never has two spans in one column. Islands smaller than the minimum area are removed; the merge threshold is not used.
    /// </summary>
    public static void BuildLayers(CompactHeightfield chf, RegionSettings settings)
    {
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        var g = SpanGraph.For(chf);
        if (LayersWithoutOverlap(chf, g, settings))
        {
            g.Release();
            return;
        }
        g.Release();
        BuildLayersGeneral(chf, settings);
    }

    // The sweep and merge of BuildLayers with no shortcut (BuildLayers itself uses it whenever the shortcut does not apply; tests compare the two).
    internal static void BuildLayersGeneral(CompactHeightfield chf, RegionSettings settings)
    {
        var g = SpanGraph.For(chf);
        Span<CompactSpan> spans = chf.Spans;
        int count = Sweep(chf, g, out int[] region);
        int n = g.Count;
        if (count <= MaxMatrixRegions)
        {
            MergeLayersDense(chf, g, region, count);
            Finish(chf, g, region, count, settings.MinRegionArea, 0);
            g.Release();
            Scratch.Return(region);
            return;
        }

        // overlap[r]: regions that share a column with r. adjacent[r]: regions linked to r.
        var overlap = new SortedSet<int>[count + 1];
        var adjacent = new SortedSet<int>[count + 1];
        var area = new byte[count + 1];
        for (int r = 1; r <= count; r++)
        {
            overlap[r] = new SortedSet<int>();
            adjacent[r] = new SortedSet<int>();
        }
        HeightfieldFrame f = chf.Frame;
        for (int c = 0; c < f.ColumnCount; c++)
        {
            int a = chf.CellFirst[c];
            int b = a + chf.CellCount[c];
            for (int i = a; i < b; i++)
            {
                for (int j = i + 1; j < b; j++)
                {
                    if (region[i] > 0 && region[j] > 0 && region[i] != region[j])
                    {
                        overlap[region[i]].Add(region[j]);
                        overlap[region[j]].Add(region[i]);
                    }
                }
            }
        }
        for (int i = 0; i < n; i++)
        {
            if (region[i] == 0)
            {
                continue;
            }
            area[region[i]] = spans[i].Area;
            for (int d = 0; d < 4; d++)
            {
                int t = g.Nbr[(4 * i) + d];
                if (t >= 0 && region[t] > 0 && region[t] != region[i])
                {
                    adjacent[region[i]].Add(region[t]);
                }
            }
        }
        var alias = new int[count + 1];
        for (int r = 0; r <= count; r++)
        {
            alias[r] = r;
        }
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (int a = 1; a <= count; a++)
            {
                if (alias[a] != a)
                {
                    continue;
                }
                foreach (int b in adjacent[a])
                {
                    if (b == a || alias[b] != b || area[b] != area[a] || overlap[a].Contains(b))
                    {
                        continue;
                    }
                    // Merge the higher id into the lower one.
                    int keep = Math.Min(a, b);
                    int gone = Math.Max(a, b);
                    alias[gone] = keep;
                    foreach (int o in overlap[gone])
                    {
                        overlap[keep].Add(o);
                        overlap[o].Remove(gone);
                        overlap[o].Add(keep);
                    }
                    foreach (int o in adjacent[gone])
                    {
                        if (o != keep)
                        {
                            adjacent[keep].Add(o);
                            adjacent[o].Remove(gone);
                            adjacent[o].Add(keep);
                        }
                    }
                    adjacent[keep].Remove(gone);
                    adjacent[keep].Remove(keep);
                    changed = true;
                    break;
                }
            }
        }
        for (int i = 0; i < n; i++)
        {
            if (region[i] > 0)
            {
                region[i] = Resolve(alias, region[i]);
            }
        }
        Finish(chf, g, region, count, settings.MinRegionArea, 0);
        g.Release();
        Scratch.Return(region);
    }


    // The layer partition of a heightfield in which no column holds two walkable spans. Then no two regions share a column, so the merge of BuildLayers
    // joins every pair of linked same-area regions and ends with the connected components of "linked and same area": the partition is computed directly by
    // union-find (each component is rooted at its first span, and ids are numbered in order of first span, as Finish numbers them), which gives the
    // same regions as the sweep and the merge, only without building them. Returns false when the heightfield has such a column, or a region would be
    // removed as too small (the general code then decides, with the same result).
    private static bool LayersWithoutOverlap(CompactHeightfield chf, SpanGraph g, RegionSettings settings)
    {
        Span<CompactSpan> spans = chf.Spans;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> cellCount = chf.CellCount;
        foreach (int c in g.MultiSpanColumns)
        {
            int walkable = 0;
            for (int i = first[c]; i < first[c] + cellCount[c]; i++)
            {
                if (spans[i].Area != 0)
                {
                    walkable++;
                }
            }
            if (walkable > 1)
            {
                return false;
            }
        }
        int n = g.Count;
        int[] nbr = g.Nbr;
        int[] parent = Scratch.Rent<int>(n);
        for (int i = 0; i < n; i++)
        {
            parent[i] = i;
        }
        // Links to the -x and -z neighbours point to earlier spans; the other two directions are the same links seen from the other end.
        for (int i = 0; i < n; i++)
        {
            byte area = spans[i].Area;
            if (area == 0)
            {
                continue;
            }
            int t = nbr[4 * i];
            if (t >= 0 && spans[t].Area == area)
            {
                Union(parent, i, t);
            }
            t = nbr[(4 * i) + 3];
            if (t >= 0 && spans[t].Area == area)
            {
                Union(parent, i, t);
            }
        }
        int[] region = Scratch.Rent<int>(n);
        int count = 0;
        for (int i = 0; i < n; i++)
        {
            if (spans[i].Area == 0)
            {
                region[i] = 0;
                continue;
            }
            int root = Find(parent, i);
            region[i] = root == i ? ++count : region[root];
        }
        Scratch.Return(parent);
        int[] size = Scratch.RentZeroed<int>(count + 1);
        for (int i = 0; i < n; i++)
        {
            size[region[i]]++;
        }
        bool small = false;
        for (int r = 1; r <= count && !small; r++)
        {
            small = size[r] < settings.MinRegionArea;
        }
        Scratch.Return(size);
        if (small)
        {
            Scratch.Return(region);
            return false;
        }
        for (int i = 0; i < n; i++)
        {
            spans[i].Region = region[i];
        }
        chf.RegionCount = count;
        Scratch.Return(region);
        return true;
    }

    /// <summary>
    /// Layer-first partition: a region is a connected set of linked spans that share a layer (<paramref name="layerOfSpan"/>, one entry per span, from the
    /// tile's first build) and an area. Layers hold at most one span per column, so no sweep or merge is needed: connected components by union-find, numbered
    /// in order of first span, then islands smaller than the minimum area are removed. Unlike <see cref="BuildLayers"/> the layers are decided once per tile
    /// and kept, so a rebuild after an area change repartitions in one pass.
    /// </summary>
    internal static void BuildLayerComponents(CompactHeightfield chf, ReadOnlySpan<int> layerOfSpan, RegionSettings settings)
    {
        var g = SpanGraph.For(chf);
        Span<CompactSpan> spans = chf.Spans;
        int n = g.Count;
        int[] nbr = g.Nbr;
        int[] parent = Scratch.Rent<int>(n);
        for (int i = 0; i < n; i++)
        {
            parent[i] = i;
        }
        for (int i = 0; i < n; i++)
        {
            byte area = spans[i].Area;
            if (area == 0)
            {
                continue;
            }
            int layer = layerOfSpan[i];
            int t = nbr[4 * i];
            if (t >= 0 && spans[t].Area == area && layerOfSpan[t] == layer)
            {
                Union(parent, i, t);
            }
            t = nbr[(4 * i) + 3];
            if (t >= 0 && spans[t].Area == area && layerOfSpan[t] == layer)
            {
                Union(parent, i, t);
            }
        }
        int[] region = Scratch.Rent<int>(n);
        int count = 0;
        for (int i = 0; i < n; i++)
        {
            if (spans[i].Area == 0)
            {
                region[i] = 0;
                continue;
            }
            int root = Find(parent, i);
            region[i] = root == i ? ++count : region[root];
        }
        Scratch.Return(parent);
        Finish(chf, g, region, count, settings.MinRegionArea, 0);
        g.Release();
        Scratch.Return(region);
    }

    // The root is always the smallest index of its set, so the first span of a component is its root.
    private static int Find(int[] parent, int i)
    {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    }

    private static void Union(int[] parent, int a, int b)
    {
        a = Find(parent, a);
        b = Find(parent, b);
        if (a < b)
        {
            parent[b] = a;
        }
        else if (b < a)
        {
            parent[a] = b;
        }
    }

    // Up to this many regions the layer merge keeps its two relations as bit matrices (count² bits each); beyond, as sorted sets.
    private const int MaxMatrixRegions = 2048;

    // The layer merge of BuildLayers on bit matrices: the same relations, the same visiting order (ascending ids; ascending neighbours inside), so the same merges.
    private static void MergeLayersDense(CompactHeightfield chf, SpanGraph g, int[] region, int count)
    {
        Span<CompactSpan> spans = chf.Spans;
        int n = g.Count;
        int words = (count + 64) >> 6;
        ulong[] overlap = Scratch.RentZeroed<ulong>((count + 1) * words);
        ulong[] adjacent = Scratch.RentZeroed<ulong>((count + 1) * words);
        var area = new byte[count + 1];
        var alias = new int[count + 1];
        for (int r = 0; r <= count; r++)
        {
            alias[r] = r;
        }
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> cellCount = chf.CellCount;
        for (int c = 0; c < cellCount.Length; c++)
        {
            int b = first[c] + cellCount[c];
            for (int i = first[c]; i + 1 < b; i++)
            {
                int ri = region[i];
                if (ri <= 0)
                {
                    continue;
                }
                for (int j = i + 1; j < b; j++)
                {
                    int rj = region[j];
                    if (rj > 0 && ri != rj)
                    {
                        overlap[(ri * words) + (rj >> 6)] |= 1UL << (rj & 63);
                        overlap[(rj * words) + (ri >> 6)] |= 1UL << (ri & 63);
                    }
                }
            }
        }
        int[] nbr = g.Nbr;
        for (int i = 0; i < n; i++)
        {
            int ri = region[i];
            if (ri == 0)
            {
                continue;
            }
            area[ri] = spans[i].Area;
            for (int d = 0; d < 4; d++)
            {
                int t = nbr[(4 * i) + d];
                if (t >= 0)
                {
                    int rt = region[t];
                    if (rt > 0 && rt != ri)
                    {
                        adjacent[(ri * words) + (rt >> 6)] |= 1UL << (rt & 63);
                    }
                }
            }
        }
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (int a = 1; a <= count; a++)
            {
                if (alias[a] != a)
                {
                    continue;
                }
                int b = FindMergeTarget(adjacent, overlap, words, alias, area, a);
                if (b == 0)
                {
                    continue;
                }
                // Merge the higher id into the lower one.
                int keep = Math.Min(a, b);
                int gone = Math.Max(a, b);
                alias[gone] = keep;
                for (int w = 0; w < words; w++)
                {
                    ulong bits = overlap[(gone * words) + w];
                    overlap[(keep * words) + w] |= bits;
                    while (bits != 0)
                    {
                        int o = (w << 6) + System.Numerics.BitOperations.TrailingZeroCount(bits);
                        bits &= bits - 1;
                        overlap[(o * words) + (gone >> 6)] &= ~(1UL << (gone & 63));
                        overlap[(o * words) + (keep >> 6)] |= 1UL << (keep & 63);
                    }
                }
                for (int w = 0; w < words; w++)
                {
                    ulong bits = adjacent[(gone * words) + w];
                    while (bits != 0)
                    {
                        int o = (w << 6) + System.Numerics.BitOperations.TrailingZeroCount(bits);
                        bits &= bits - 1;
                        if (o != keep)
                        {
                            adjacent[(keep * words) + (o >> 6)] |= 1UL << (o & 63);
                            adjacent[(o * words) + (gone >> 6)] &= ~(1UL << (gone & 63));
                            adjacent[(o * words) + (keep >> 6)] |= 1UL << (keep & 63);
                        }
                    }
                }
                adjacent[(keep * words) + (gone >> 6)] &= ~(1UL << (gone & 63));
                adjacent[(keep * words) + (keep >> 6)] &= ~(1UL << (keep & 63));
                changed = true;
            }
        }
        for (int i = 0; i < n; i++)
        {
            if (region[i] > 0)
            {
                region[i] = Resolve(alias, region[i]);
            }
        }
        Scratch.Return(overlap);
        Scratch.Return(adjacent);
    }

    // The lowest neighbour b of a (in ascending order) that is live, has a's area and shares no column with a; 0 when there is none.
    private static int FindMergeTarget(ulong[] adjacent, ulong[] overlap, int words, int[] alias, byte[] area, int a)
    {
        for (int w = 0; w < words; w++)
        {
            ulong bits = adjacent[(a * words) + w];
            while (bits != 0)
            {
                int b = (w << 6) + System.Numerics.BitOperations.TrailingZeroCount(bits);
                bits &= bits - 1;
                if (b == a || alias[b] != b || area[b] != area[a] || (overlap[(a * words) + (b >> 6)] & (1UL << (b & 63))) != 0)
                {
                    continue;
                }
                return b;
            }
        }
        return 0;
    }

    /// <summary>
    /// Removes and merges small regions in place, then renumbers regions 1..k in order of their first span. A region smaller than
    /// <paramref name="minArea"/> with no neighbouring region is removed; a region smaller than <paramref name="mergeArea"/> is merged into the
    /// same-area neighbour with the longest shared boundary (lowest id on ties), repeating until no such merge is left.
    /// </summary>
    public static void FilterRegions(CompactHeightfield chf, int minArea, int mergeArea)
    {
        ArgumentNullException.ThrowIfNull(chf);
        if (minArea < 0 || mergeArea < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(minArea), "Areas must not be negative.");
        }
        var g = SpanGraph.For(chf);
        Span<CompactSpan> spans = chf.Spans;
        var region = new int[g.Count];
        int max = 0;
        for (int i = 0; i < region.Length; i++)
        {
            region[i] = spans[i].Region;
            max = Math.Max(max, region[i]);
        }
        Finish(chf, g, region, max, minArea, mergeArea);
        g.Release();
    }

    private static int Resolve(int[] alias, int r)
    {
        while (alias[r] != r)
        {
            r = alias[r];
        }
        return r;
    }

    // The monotone sweep. region is a pooled array (zeroed up to the span count) that the caller returns.
    private static int Sweep(CompactHeightfield chf, SpanGraph g, out int[] region)
    {
        Span<CompactSpan> spans = chf.Spans;
        HeightfieldFrame f = chf.Frame;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> cellCount = chf.CellCount;
        int n = g.Count;
        int[] nbr = g.Nbr;
        region = Scratch.RentZeroed<int>(n);
        int[] arena = Scratch.Rent<int>(6 * (n + 2));
        int stride = n + 2;
        Span<int> sweepOf = arena.AsSpan(0, stride);
        Span<int> sweepNei = arena.AsSpan(stride, stride);
        Span<int> sweepRegion = arena.AsSpan(2 * stride, stride);
        Span<int> hitStamp = arena.AsSpan(3 * stride, stride);   // per region: row in which it was last counted
        Span<int> hitSweep = arena.AsSpan(4 * stride, stride);   // per region: last sweep that hit it
        Span<int> hitCount = arena.AsSpan(5 * stride, stride);   // per region: sweeps of the current row that hit it
        int nextRegion = 0;
        hitStamp[0] = -1;
        hitSweep[0] = -1;
        hitCount[0] = 0;
        int width = f.Width;
        for (int z = 0; z < f.Depth; z++)
        {
            int sweeps = 0;
            int rowStart = z * width;
            for (int x = 0; x < width; x++)
            {
                int c = rowStart + x;
                int end = first[c] + cellCount[c];
                for (int i = first[c]; i < end; i++)
                {
                    byte area = spans[i].Area;
                    if (area == 0)
                    {
                        continue;
                    }
                    int left = nbr[4 * i];
                    int sweep;
                    if (left >= 0 && spans[left].Area == area)
                    {
                        sweep = sweepOf[left];
                    }
                    else
                    {
                        sweep = sweeps++;
                        sweepNei[sweep] = 0;
                    }
                    sweepOf[i] = sweep;
                    int down = nbr[(4 * i) + 3];
                    if (down >= 0 && region[down] > 0 && spans[down].Area == area)
                    {
                        int r = region[down];
                        if (sweepNei[sweep] == 0)
                        {
                            sweepNei[sweep] = r;
                        }
                        else if (sweepNei[sweep] != r)
                        {
                            sweepNei[sweep] = -1;
                        }
                        if (hitStamp[r] != z)
                        {
                            hitStamp[r] = z;
                            hitCount[r] = 0;
                            hitSweep[r] = -1;
                        }
                        if (hitSweep[r] != sweep)
                        {
                            hitSweep[r] = sweep;
                            hitCount[r]++;
                        }
                    }
                }
            }
            if (sweeps == 0)
            {
                continue;
            }
            for (int s = 0; s < sweeps; s++)
            {
                int r = sweepNei[s];
                if (r > 0 && hitStamp[r] == z && hitCount[r] == 1)
                {
                    sweepRegion[s] = r;
                }
                else
                {
                    sweepRegion[s] = ++nextRegion;
                    hitStamp[nextRegion] = -1;
                    hitSweep[nextRegion] = -1;
                    hitCount[nextRegion] = 0;
                }
            }
            for (int x = 0; x < width; x++)
            {
                int c = rowStart + x;
                int end = first[c] + cellCount[c];
                for (int i = first[c]; i < end; i++)
                {
                    if (spans[i].Area != 0)
                    {
                        region[i] = sweepRegion[sweepOf[i]];
                    }
                }
            }
        }
        Scratch.Return(arena);
        return nextRegion;
    }

    private static void Finish(CompactHeightfield chf, SpanGraph g, int[] region, int count, int minArea, int mergeArea)
    {
        int n = g.Count;
        Span<CompactSpan> spans = chf.Spans;
        var size = new int[count + 1];
        var area = new byte[count + 1];
        for (int i = 0; i < n; i++)
        {
            int r = region[i];
            if (r != 0)
            {
                size[r]++;
                area[r] = spans[i].Area;
            }
        }
        // Only a region smaller than the larger threshold is ever removed or merged away, so only its neighbour list is read: those lists are
        // gathered (boundary length = number of touching span edges), first flat in neighbour-id order and turned into sorted maps on first use.
        // A bigger region's list is not kept up to date.
        int limit = Math.Max(minArea, mergeArea);
        var small = new bool[count + 1];
        for (int r = 1; r <= count; r++)
        {
            small[r] = size[r] < limit;
        }
        int[] nbr = g.Nbr;
        var off = new int[count + 2];
        int[] edgeFrom = Array.Empty<int>();
        int[] edgeTo = Array.Empty<int>();
        int edges = 0;
        if (limit > 0)
        {
            for (int i = 0; i < n; i++)
            {
                int r = region[i];
                if (r == 0 || !small[r])
                {
                    continue;
                }
                for (int d = 0; d < 4; d++)
                {
                    int t = nbr[(4 * i) + d];
                    if (t >= 0)
                    {
                        int rt = region[t];
                        if (rt > 0 && rt != r)
                        {
                            if (edges == edgeFrom.Length)
                            {
                                Array.Resize(ref edgeFrom, Math.Max(1024, edges * 2));
                                Array.Resize(ref edgeTo, edgeFrom.Length);
                            }
                            edgeFrom[edges] = r;
                            edgeTo[edges++] = rt;
                        }
                    }
                }
            }
        }
        for (int e = 0; e < edges; e++)
        {
            off[edgeFrom[e] + 1]++;
        }
        for (int r = 0; r <= count; r++)
        {
            off[r + 1] += off[r];
        }
        var raw = new int[edges];
        var fillAt = new int[count + 1];
        for (int e = 0; e < edges; e++)
        {
            int r = edgeFrom[e];
            raw[off[r] + fillAt[r]++] = edgeTo[e];
        }
        var nei = new RegionNeighbors(off, raw, count);
        var alias = new int[count + 1];
        for (int r = 0; r <= count; r++)
        {
            alias[r] = r;
        }
        var removed = new bool[count + 1];
        if (minArea > 0)
        {
            for (int r = 1; r <= count; r++)
            {
                if (size[r] > 0 && size[r] < minArea && off[r + 1] == off[r])
                {
                    removed[r] = true;
                }
            }
        }
        if (mergeArea > 0)
        {
            bool changed = true;
            while (changed)
            {
                changed = false;
                for (int r = 1; r <= count; r++)
                {
                    if (alias[r] != r || removed[r] || size[r] == 0 || size[r] >= mergeArea)
                    {
                        continue;
                    }
                    int best = 0;
                    int bestLen = 0;
                    foreach (var (m, len) in nei[r])
                    {
                        if (area[m] == area[r] && len > bestLen)
                        {
                            best = m;
                            bestLen = len;
                        }
                    }
                    if (best == 0)
                    {
                        continue;
                    }
                    // Merge r into best.
                    alias[r] = best;
                    size[best] += size[r];
                    size[r] = 0;
                    foreach (var (m, len) in nei[r])
                    {
                        if (m == best)
                        {
                            continue;
                        }
                        if (small[best])
                        {
                            nei[best][m] = nei[best].GetValueOrDefault(m) + len;
                        }
                        if (small[m])
                        {
                            nei[m][best] = nei[m].GetValueOrDefault(best) + len;
                            nei[m].Remove(r);
                        }
                    }
                    if (small[best])
                    {
                        nei[best].Remove(r);
                    }
                    nei[r].Clear();
                    changed = true;
                }
            }
        }
        // Renumber in order of first span.
        var newId = new int[count + 1];
        int next = 0;
        for (int i = 0; i < n; i++)
        {
            int r = region[i];
            if (r == 0)
            {
                spans[i].Region = 0;
                continue;
            }
            r = Resolve(alias, r);
            if (removed[r])
            {
                spans[i].Region = 0;
                continue;
            }
            if (newId[r] == 0)
            {
                newId[r] = ++next;
            }
            spans[i].Region = newId[r];
        }
        chf.RegionCount = next;
    }
}

/// <summary>
/// Region adjacency with boundary lengths. Built from a flat list of neighbour ids per region (one entry per touching span edge); a region's
/// sorted map of neighbour id → edge count is made on first use, so the merge pays only for the regions it visits.
/// </summary>
internal sealed class RegionNeighbors
{
    private readonly int[] _off;
    private readonly int[] _raw;
    private readonly SortedDictionary<int, int>?[] _maps;

    public RegionNeighbors(int[] off, int[] raw, int count)
    {
        _off = off;
        _raw = raw;
        _maps = new SortedDictionary<int, int>?[count + 1];
    }

    public SortedDictionary<int, int> this[int r]
    {
        get
        {
            SortedDictionary<int, int>? m = _maps[r];
            if (m == null)
            {
                m = new SortedDictionary<int, int>();
                for (int k = _off[r]; k < _off[r + 1]; k++)
                {
                    m[_raw[k]] = m.GetValueOrDefault(_raw[k]) + 1;
                }
                _maps[r] = m;
            }
            return m;
        }
    }
}

