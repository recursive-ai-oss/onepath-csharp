using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.NavMesh.Build3D;

/// <summary>
/// The span adjacency of a compact heightfield as flat arrays: neighbour span index per direction, or -1. A neighbour is recorded only when the
/// link is mutual (the target links back to the span): links choose the lowest qualifying span of a cell, so with several floors in a cell a link can
/// be one-way, and boundary tracing needs a symmetric relation to close its loops.
/// </summary>
internal sealed class SpanGraph
{
    public SpanGraph(CompactHeightfield chf)
    {
        HeightfieldFrame f = chf.Frame;
        Width = f.Width;
        Depth = f.Depth;
        Count = chf.SpanCount;
        Nbr = new int[4 * Count];
        CellX = new int[Count];
        CellZ = new int[Count];
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> count = chf.CellCount;
        Span<CompactSpan> spans = chf.Spans;
        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                for (int i = first[c]; i < first[c] + count[c]; i++)
                {
                    CellX[i] = x;
                    CellZ[i] = z;
                    for (int d = 0; d < 4; d++)
                    {
                        int con = CompactHeightfield.GetConnection(spans[i], d);
                        Nbr[(4 * i) + d] = con == CompactHeightfield.NotConnected
                            ? -1
                            : first[f.Column(x + CompactHeightfield.DirX[d], z + CompactHeightfield.DirZ[d])] + con;
                    }
                }
            }
        }
        for (int i = 0; i < Count; i++)
        {
            for (int d = 0; d < 4; d++)
            {
                int t = Nbr[(4 * i) + d];
                if (t >= 0 && Nbr[(4 * t) + ((d + 2) & 3)] != i)
                {
                    Nbr[(4 * i) + d] = -2; // one-way: dropped after all pairs are examined
                }
            }
        }
        for (int k = 0; k < Nbr.Length; k++)
        {
            if (Nbr[k] == -2)
            {
                Nbr[k] = -1;
            }
        }
    }

    public int Width { get; }

    public int Depth { get; }

    public int Count { get; }

    public int[] Nbr { get; }

    public int[] CellX { get; }

    public int[] CellZ { get; }

    public bool OutsideGrid(int span, int dir)
    {
        int x = CellX[span] + CompactHeightfield.DirX[dir];
        int z = CellZ[span] + CompactHeightfield.DirZ[dir];
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
        var g = new SpanGraph(chf);
        Span<CompactSpan> spans = chf.Spans;
        int n = g.Count;
        int[] dist = BuildDistanceField(chf, out int maxDist);
        var region = new int[n];

        // Counting sort of walkable spans by distance, ascending index inside a level.
        var start = new int[maxDist + 2];
        for (int i = 0; i < n; i++)
        {
            if (spans[i].Area != 0)
            {
                start[dist[i] + 1]++;
            }
        }
        for (int l = 0; l <= maxDist; l++)
        {
            start[l + 1] += start[l];
        }
        var fill = new int[maxDist + 1];
        var order = new int[start[maxDist + 1]];
        for (int i = 0; i < n; i++)
        {
            if (spans[i].Area != 0)
            {
                order[start[dist[i]] + fill[dist[i]]++] = i;
            }
        }

        int next = 0;
        var mark = new int[n];
        int stamp = 0;
        var frontier = new List<int>();
        var assign = new List<(int Span, int Region)>();
        var queue = new Queue<int>();
        for (int level = maxDist; level >= 0; level--)
        {
            int lo = start[level];
            int hi = start[level + 1];
            if (lo == hi)
            {
                continue;
            }
            // Round 1: every span of this level that touches a region.
            frontier.Clear();
            for (int k = lo; k < hi; k++)
            {
                frontier.Add(order[k]);
            }
            while (frontier.Count > 0)
            {
                assign.Clear();
                foreach (int s in frontier)
                {
                    int best = 0;
                    for (int d = 0; d < 4; d++)
                    {
                        int t = g.Nbr[(4 * s) + d];
                        if (t >= 0 && region[t] > 0 && spans[t].Area == spans[s].Area && (best == 0 || region[t] < best))
                        {
                            best = region[t];
                        }
                    }
                    if (best > 0)
                    {
                        assign.Add((s, best));
                    }
                }
                foreach (var (s, r) in assign)
                {
                    region[s] = r;
                }
                // Next round: unassigned spans of this level next to a span assigned just now.
                stamp++;
                frontier.Clear();
                foreach (var (s, _) in assign)
                {
                    for (int d = 0; d < 4; d++)
                    {
                        int t = g.Nbr[(4 * s) + d];
                        if (t >= 0 && region[t] == 0 && spans[t].Area != 0 && dist[t] == level && mark[t] != stamp)
                        {
                            mark[t] = stamp;
                            frontier.Add(t);
                        }
                    }
                }
                frontier.Sort();
            }
            // New basins: what is left of this level.
            for (int k = lo; k < hi; k++)
            {
                int s0 = order[k];
                if (region[s0] != 0)
                {
                    continue;
                }
                int id = ++next;
                region[s0] = id;
                queue.Enqueue(s0);
                while (queue.Count > 0)
                {
                    int s = queue.Dequeue();
                    for (int d = 0; d < 4; d++)
                    {
                        int t = g.Nbr[(4 * s) + d];
                        if (t >= 0 && region[t] == 0 && spans[t].Area == spans[s0].Area && dist[t] == level)
                        {
                            region[t] = id;
                            queue.Enqueue(t);
                        }
                    }
                }
            }
        }
        Finish(chf, g, region, next, settings.MinRegionArea, settings.MergeRegionArea);
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
        var g = new SpanGraph(chf);
        int count = Sweep(chf, g, out int[] region);
        Finish(chf, g, region, count, settings.MinRegionArea, settings.MergeRegionArea);
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
        var g = new SpanGraph(chf);
        Span<CompactSpan> spans = chf.Spans;
        int count = Sweep(chf, g, out int[] region);
        int n = g.Count;

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
        var g = new SpanGraph(chf);
        Span<CompactSpan> spans = chf.Spans;
        var region = new int[g.Count];
        int max = 0;
        for (int i = 0; i < region.Length; i++)
        {
            region[i] = spans[i].Region;
            max = Math.Max(max, region[i]);
        }
        Finish(chf, g, region, max, minArea, mergeArea);
    }

    private static int Resolve(int[] alias, int r)
    {
        while (alias[r] != r)
        {
            r = alias[r];
        }
        return r;
    }

    private static int Sweep(CompactHeightfield chf, SpanGraph g, out int[] region)
    {
        Span<CompactSpan> spans = chf.Spans;
        HeightfieldFrame f = chf.Frame;
        region = new int[g.Count];
        var sweepOf = new int[g.Count];
        int nextRegion = 0;
        var sweepNei = new List<int>();
        var sweepRegion = new List<int>();
        var hitStamp = new List<int>();   // per region: row in which it was last counted
        var hitSweep = new List<int>();   // per region: last sweep that hit it
        var hitCount = new List<int>();   // per region: sweeps of the current row that hit it
        hitStamp.Add(-1);
        hitSweep.Add(-1);
        hitCount.Add(0);
        for (int z = 0; z < f.Depth; z++)
        {
            sweepNei.Clear();
            sweepRegion.Clear();
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                for (int i = chf.CellFirst[c]; i < chf.CellFirst[c] + chf.CellCount[c]; i++)
                {
                    if (spans[i].Area == 0)
                    {
                        continue;
                    }
                    int left = g.Nbr[4 * i];
                    int sweep;
                    if (left >= 0 && spans[left].Area == spans[i].Area)
                    {
                        sweep = sweepOf[left];
                    }
                    else
                    {
                        sweep = sweepNei.Count;
                        sweepNei.Add(0);
                        sweepRegion.Add(0);
                    }
                    sweepOf[i] = sweep;
                    int down = g.Nbr[(4 * i) + 3];
                    if (down >= 0 && region[down] > 0 && spans[down].Area == spans[i].Area)
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
            for (int s = 0; s < sweepNei.Count; s++)
            {
                int r = sweepNei[s];
                if (r > 0 && hitStamp[r] == z && hitCount[r] == 1)
                {
                    sweepRegion[s] = r;
                }
                else
                {
                    sweepRegion[s] = ++nextRegion;
                    hitStamp.Add(-1);
                    hitSweep.Add(-1);
                    hitCount.Add(0);
                }
            }
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                for (int i = chf.CellFirst[c]; i < chf.CellFirst[c] + chf.CellCount[c]; i++)
                {
                    if (spans[i].Area != 0)
                    {
                        region[i] = sweepRegion[sweepOf[i]];
                    }
                }
            }
        }
        return nextRegion;
    }

    private static void Finish(CompactHeightfield chf, SpanGraph g, int[] region, int count, int minArea, int mergeArea)
    {
        int n = g.Count;
        Span<CompactSpan> spans = chf.Spans;
        var size = new int[count + 1];
        var area = new byte[count + 1];
        var nei = new SortedDictionary<int, int>[count + 1];
        for (int r = 1; r <= count; r++)
        {
            nei[r] = new SortedDictionary<int, int>();
        }
        for (int i = 0; i < n; i++)
        {
            int r = region[i];
            if (r == 0)
            {
                continue;
            }
            size[r]++;
            area[r] = spans[i].Area;
            for (int d = 0; d < 4; d++)
            {
                int t = g.Nbr[(4 * i) + d];
                if (t >= 0 && region[t] > 0 && region[t] != r)
                {
                    nei[r][region[t]] = nei[r].GetValueOrDefault(region[t]) + 1;
                }
            }
        }
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
                if (size[r] > 0 && size[r] < minArea && nei[r].Count == 0)
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
                        nei[best][m] = nei[best].GetValueOrDefault(m) + len;
                        nei[m][best] = nei[m].GetValueOrDefault(best) + len;
                        nei[m].Remove(r);
                    }
                    nei[best].Remove(r);
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
