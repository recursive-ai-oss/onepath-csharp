using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OneMath.Predicates;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

internal static class Fields
{
    // A random walkable field: mostly one floor with bumps, some second floors, a few holes and a second area id.
    public static CompactHeightfield Random(TestRng rng, int minSide = 6, int maxSide = 18, bool secondFloors = true)
    {
        int w = minSide + rng.Int(maxSide - minSide + 1);
        int d = minSide + rng.Int(maxSide - minSide + 1);
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, w, d, 20));
        int holes = rng.Int(4);
        for (int c = 0; c < w * d; c++)
        {
            if (rng.Int(100) < 8 + (holes * 4))
            {
                continue;
            }
            int top = 1 + (rng.Int(10) < 8 ? 0 : rng.Int(2));
            hf.AddSpan(c, 0, top, rng.Int(10) < 8 ? (byte)63 : (byte)10, 0);
            if (secondFloors && rng.Int(10) == 0)
            {
                hf.AddSpan(c, 8, 9 + rng.Int(2), 63, 0);
            }
        }
        return CompactHeightfield.Build(hf, 3, 1);
    }

    public static int[] Region(CompactHeightfield c)
    {
        var r = new int[c.SpanCount];
        for (int i = 0; i < r.Length; i++)
        {
            r[i] = c.Spans[i].Region;
        }
        return r;
    }

    public static int[,] Neighbors(CompactHeightfield c)
    {
        var g = new int[c.SpanCount, 4];
        HeightfieldFrame f = c.Frame;
        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                int cell = f.Column(x, z);
                for (int i = c.CellFirst[cell]; i < c.CellFirst[cell] + c.CellCount[cell]; i++)
                {
                    for (int d = 0; d < 4; d++)
                    {
                        int con = CompactHeightfield.GetConnection(c.Spans[i], d);
                        g[i, d] = con == CompactHeightfield.NotConnected ? -1
                            : c.CellFirst[f.Column(x + CompactHeightfield.DirX[d], z + CompactHeightfield.DirZ[d])] + con;
                    }
                }
            }
        }
        // Only mutual links count as adjacency (see the region stage's span graph).
        var mutual = (int[,])g.Clone();
        for (int i = 0; i < c.SpanCount; i++)
        {
            for (int d = 0; d < 4; d++)
            {
                int t = g[i, d];
                if (t >= 0 && g[t, (d + 2) & 3] != i)
                {
                    mutual[i, d] = -1;
                }
            }
        }
        return mutual;
    }
}

public class RegionTests
{
    private static void CheckPartition(CompactHeightfield c, bool expectAllRegioned, int minArea)
    {
        int[] reg = Fields.Region(c);
        int[,] nb = Fields.Neighbors(c);
        int n = c.SpanCount;
        // Ids are 1..RegionCount, numbered by first span.
        int seen = 0;
        for (int i = 0; i < n; i++)
        {
            if (reg[i] > seen)
            {
                Assert.Equal(seen + 1, reg[i]);
                seen = reg[i];
            }
        }
        Assert.Equal(seen, c.RegionCount);
        // One area and one connected piece per region.
        var comp = new int[n];
        for (int i = 0; i < n; i++)
        {
            if (reg[i] == 0)
            {
                Assert.True(c.Spans[i].Area == 0 || !expectAllRegioned);
                continue;
            }
            Assert.NotEqual(0, c.Spans[i].Area);
        }
        var visited = new bool[n];
        var counted = new HashSet<int>();
        for (int s = 0; s < n; s++)
        {
            if (reg[s] == 0 || visited[s])
            {
                continue;
            }
            Assert.True(counted.Add(reg[s]), $"region {reg[s]} is not connected");
            var q = new Queue<int>();
            q.Enqueue(s);
            visited[s] = true;
            while (q.Count > 0)
            {
                int u = q.Dequeue();
                Assert.Equal(c.Spans[s].Area, c.Spans[u].Area);
                for (int d = 0; d < 4; d++)
                {
                    int t = nb[u, d];
                    if (t >= 0 && !visited[t] && reg[t] == reg[s])
                    {
                        visited[t] = true;
                        q.Enqueue(t);
                    }
                }
            }
        }
        // Unregioned walkable spans only come from removed islands: no region next to them, and fewer than minArea of them together.
        var seenZero = new bool[n];
        for (int s = 0; s < n; s++)
        {
            if (reg[s] != 0 || c.Spans[s].Area == 0 || seenZero[s])
            {
                continue;
            }
            int size = 0;
            var q = new Queue<int>();
            q.Enqueue(s);
            seenZero[s] = true;
            while (q.Count > 0)
            {
                int u = q.Dequeue();
                size++;
                for (int d = 0; d < 4; d++)
                {
                    int t = nb[u, d];
                    Assert.True(t < 0 || reg[t] == 0 || c.Spans[t].Area == 0 && false, "a removed island touches a region");
                    if (t >= 0 && !seenZero[t] && c.Spans[t].Area != 0)
                    {
                        seenZero[t] = true;
                        q.Enqueue(t);
                    }
                }
            }
            Assert.True(size < minArea, $"unregioned piece of {size} spans, minimum {minArea}");
        }
    }

    private static void CheckMergeFixpoint(CompactHeightfield c, int mergeArea)
    {
        int[] reg = Fields.Region(c);
        int[,] nb = Fields.Neighbors(c);
        var size = new int[c.RegionCount + 1];
        foreach (int r in reg)
        {
            size[r]++;
        }
        for (int i = 0; i < reg.Length; i++)
        {
            if (reg[i] == 0 || size[reg[i]] >= mergeArea)
            {
                continue;
            }
            for (int d = 0; d < 4; d++)
            {
                int t = nb[i, d];
                bool canMerge = t >= 0 && reg[t] > 0 && reg[t] != reg[i] && c.Spans[t].Area == c.Spans[i].Area;
                Assert.False(canMerge, canMerge ? $"region {reg[i]} ({size[reg[i]]} spans) could still merge into {reg[t]}" : "");
            }
        }
    }

    [Fact]
    public void WatershedPartitionProperties()
    {
        var rng = new TestRng(101);
        for (int n = 0; n < 300; n++)
        {
            var c = Fields.Random(rng);
            int min = rng.Int(3) * 6;
            int merge = rng.Int(3) * 20;
            Regions.BuildWatershed(c, new RegionSettings { MinRegionArea = min, MergeRegionArea = merge });
            CheckPartition(c, min == 0, min);
            if (merge > 0)
            {
                CheckMergeFixpoint(c, merge);
            }
        }
    }

    [Fact]
    public void MonotonePartitionProperties()
    {
        var rng = new TestRng(102);
        int multiRegion = 0;
        for (int n = 0; n < 300; n++)
        {
            var c = Fields.Random(rng);
            Regions.BuildMonotone(c, new RegionSettings { Partition = PartitionType.Monotone, MinRegionArea = 0, MergeRegionArea = 0 });
            CheckPartition(c, true, 0);
            multiRegion += c.RegionCount > 1 ? 1 : 0;
            // Monotone: in every row, a region is one run along x.
            int[] reg = Fields.Region(c);
            int[,] nb = Fields.Neighbors(c);
            var g = c.Frame;
            var runStarts = new Dictionary<(int, int), int>();
            for (int z = 0; z < g.Depth; z++)
            {
                for (int x = 0; x < g.Width; x++)
                {
                    int cell = g.Column(x, z);
                    for (int i = c.CellFirst[cell]; i < c.CellFirst[cell] + c.CellCount[cell]; i++)
                    {
                        int left = nb[i, 0];
                        if (reg[i] > 0 && !(left >= 0 && reg[left] == reg[i]))
                        {
                            runStarts[(reg[i], z)] = runStarts.GetValueOrDefault((reg[i], z)) + 1;
                        }
                    }
                }
            }
            Assert.All(runStarts, kv => Assert.Equal(1, kv.Value));
            // With filtering on, the partition is still valid.
            Regions.Build(c, new RegionSettings { Partition = PartitionType.Monotone, MinRegionArea = 6, MergeRegionArea = 30 });
            CheckPartition(c, false, 6);
            CheckMergeFixpoint(c, 30);
        }
        Assert.True(multiRegion > 100);
    }

    [Fact]
    public void LayerPartitionProperties()
    {
        var rng = new TestRng(103);
        int overlapping = 0;
        for (int n = 0; n < 400; n++)
        {
            var c = Fields.Random(rng);
            Regions.Build(c, new RegionSettings { Partition = PartitionType.Layers, MinRegionArea = 0 });
            CheckPartition(c, true, 0);
            int[] reg = Fields.Region(c);
            int[,] nb = Fields.Neighbors(c);
            var f = c.Frame;
            var share = new HashSet<(int, int)>();
            for (int cell = 0; cell < f.ColumnCount; cell++)
            {
                int a = c.CellFirst[cell];
                int b = a + c.CellCount[cell];
                for (int i = a; i < b; i++)
                {
                    for (int j = i + 1; j < b; j++)
                    {
                        Assert.NotEqual(reg[i], reg[j]);
                        overlapping++;
                        share.Add((Math.Min(reg[i], reg[j]), Math.Max(reg[i], reg[j])));
                    }
                }
            }
            // Maximal: two adjacent same-area layers that could merge (no shared column) would have been merged.
            // (Merging is greedy by id, so only pairs that share a column or differ in area may stay apart *unless* a merge elsewhere changed their overlap.)
            for (int i = 0; i < reg.Length; i++)
            {
                for (int d = 0; d < 4; d++)
                {
                    int t = nb[i, d];
                    if (t >= 0 && reg[t] != reg[i] && c.Spans[t].Area == c.Spans[i].Area)
                    {
                        Assert.Contains((Math.Min(reg[i], reg[t]), Math.Max(reg[i], reg[t])), share);
                    }
                }
            }
        }
        Assert.True(overlapping > 20, "the random fields never had overlapping floors");
    }

    [Fact]
    public void EmptyAndTinyFieldsWork()
    {
        var empty = CompactHeightfield.Build(new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 3, 3, 3)), 2, 1);
        foreach (PartitionType t in Enum.GetValues<PartitionType>())
        {
            Regions.Build(empty, new RegionSettings { Partition = t });
            Assert.Equal(0, empty.RegionCount);
            Assert.Empty(Contours.Build(empty, new ContourSettings()).Contours);
        }
        Assert.Throws<ArgumentOutOfRangeException>(() => Regions.Build(empty, new RegionSettings { MinRegionArea = -1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => Regions.Build(empty, new RegionSettings { Partition = (PartitionType)9 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => Contours.Build(empty, new ContourSettings { MaxError = float.NaN }));
    }

    [Fact]
    public void DistanceFieldIsTheErosionDistance()
    {
        var rng = new TestRng(104);
        for (int n = 0; n < 50; n++)
        {
            var c = Fields.Random(rng);
            int[] d = Regions.BuildDistanceField(c, out int max);
            int[] capped = Erosion.ComputeBoundaryDistance(c, 6);
            for (int i = 0; i < d.Length; i++)
            {
                Assert.Equal(Math.Min(d[i], 6), capped[i]);
                Assert.True(d[i] <= max);
            }
        }
    }
}

public class ContourTests
{
    private static long Cross2(ContourVertex a, ContourVertex b) => ((long)a.X * b.Z) - ((long)b.X * a.Z);

    private static bool ProperlyCross(ContourVertex a, ContourVertex b, ContourVertex c, ContourVertex d)
    {
        int o1 = ExactPredicates.Orient2D(a.X, a.Z, b.X, b.Z, c.X, c.Z);
        int o2 = ExactPredicates.Orient2D(a.X, a.Z, b.X, b.Z, d.X, d.Z);
        int o3 = ExactPredicates.Orient2D(c.X, c.Z, d.X, d.Z, a.X, a.Z);
        int o4 = ExactPredicates.Orient2D(c.X, c.Z, d.X, d.Z, b.X, b.Z);
        return o1 * o2 < 0 && o3 * o4 < 0;
    }

    private static double DistToSegment(ContourVertex a, ContourVertex b, ContourVertex p)
    {
        double dx = b.X - a.X, dz = b.Z - a.Z;
        double len2 = (dx * dx) + (dz * dz);
        double t = len2 == 0 ? 0 : Math.Clamp((((p.X - a.X) * dx) + ((p.Z - a.Z) * dz)) / len2, 0, 1);
        double ex = a.X + (t * dx) - p.X, ez = a.Z + (t * dz) - p.Z;
        return Math.Sqrt((ex * ex) + (ez * ez));
    }

    private static int BoundaryEdges(CompactHeightfield c, int region)
    {
        int[] reg = Fields.Region(c);
        int[,] nb = Fields.Neighbors(c);
        int e = 0;
        for (int i = 0; i < reg.Length; i++)
        {
            if (reg[i] != region)
            {
                continue;
            }
            for (int d = 0; d < 4; d++)
            {
                int t = nb[i, d];
                e += t >= 0 && reg[t] == region ? 0 : 1;
            }
        }
        return e;
    }

    private static void CheckContours(CompactHeightfield c, ContourSet set, float maxError, ref int checkedContours, ref int holesSeen, ref int reversedPairs, ref int pairs)
    {
        int[] reg = Fields.Region(c);
        var spansPer = new int[c.RegionCount + 1];
        foreach (int r in reg)
        {
            spansPer[r]++;
        }
        var rawEdges = new int[c.RegionCount + 1];
        var area2 = new long[c.RegionCount + 1];
        var intact = Enumerable.Repeat(true, c.RegionCount + 1).ToArray();
        foreach (Contour k in set.Contours)
        {
            Assert.NotNull(k.RawVertices);
            rawEdges[k.Region] += k.RawVertices!.Length + k.RawHoles!.Sum(h => h.Length);
            area2[k.Region] += Contours_Area2(k.RawVertices) + k.RawHoles!.Sum(h => Contours_Area2(h));
            holesSeen += k.RawHoles!.Length;
            ContourVertex[] v = k.Vertices;
            Assert.True(v.Length >= 3);
            // Closed, counter-clockwise (positive area), and no two edges cross: exact predicates.
            Assert.True(Contours_Area2(v) > 0);
            for (int i = 0; i < v.Length; i++)
            {
                for (int j = i + 2; j < v.Length; j++)
                {
                    if (i == 0 && j == v.Length - 1)
                    {
                        continue;
                    }
                    Assert.False(ProperlyCross(v[i], v[(i + 1) % v.Length], v[j], v[(j + 1) % v.Length]), $"contour of region {k.Region} crosses itself: {string.Join(' ', v)} raw {string.Join(' ', k.RawVertices!)} holes {k.RawHoles!.Length}");
                }
            }
            // Simplification error: every raw vertex is within maxError of the simplified outline.
            foreach (ContourVertex p in k.RawVertices)
            {
                double best = double.MaxValue;
                for (int i = 0; i < v.Length; i++)
                {
                    best = Math.Min(best, DistToSegment(v[i], v[(i + 1) % v.Length], p));
                }
                Assert.True(best <= maxError + 1e-9, $"raw vertex {p} is {best} from the simplified contour (max error {maxError}); simplified {string.Join(' ', v)}; raw {string.Join(' ', k.RawVertices!)}");
            }
            // Portals: every raw vertex where the neighbour changes survives.
            ContourVertex[] raw = k.RawVertices;
            for (int i = 0; i < raw.Length; i++)
            {
                if (raw[i].Neighbor != raw[(i + raw.Length - 1) % raw.Length].Neighbor)
                {
                    Assert.Contains(v, q => q.X == raw[i].X && q.Z == raw[i].Z);
                }
            }
            checkedContours++;
        }
        if (set.DroppedContours == 0 && set.DroppedHoles == 0)
        {
            for (int r = 1; r <= c.RegionCount; r++)
            {
                Assert.Equal(BoundaryEdges(c, r), rawEdges[r]);
                Assert.Equal(2L * spansPer[r], area2[r]);
            }
        }
        // Shared boundaries match: an edge A -> B (neighbour B > 0) has the reversed edge B -> A on the other side.
        var edges = new HashSet<(int, int, int, int, int, int)>();
        foreach (Contour k in set.Contours)
        {
            for (int i = 0; i < k.Vertices.Length; i++)
            {
                ContourVertex a = k.Vertices[i], b = k.Vertices[(i + 1) % k.Vertices.Length];
                if (a.Neighbor > 0 && a.Neighbor != k.Region)
                {
                    edges.Add((a.X, a.Z, b.X, b.Z, k.Region, a.Neighbor));
                }
            }
        }
        foreach (var (x0, z0, x1, z1, a, b) in edges)
        {
            pairs++;
            reversedPairs += edges.Contains((x1, z1, x0, z0, b, a)) ? 1 : 0;
        }
    }

    private static long Contours_Area2(ContourVertex[] p) => Build3D_SignedArea2(p);

    private static long Build3D_SignedArea2(ContourVertex[] p)
    {
        long a = 0;
        for (int i = 0, j = p.Length - 1; i < p.Length; j = i++)
        {
            a += Cross2(p[j], p[i]);
        }
        return a;
    }

    [Fact]
    public void ContoursOnRandomFields()
    {
        var rng = new TestRng(201);
        int checkedContours = 0, holes = 0, fallbacks = 0, dropped = 0;
        var rev = new int[4];
        var pairs = new int[4];
        PartitionType[] types = Enum.GetValues<PartitionType>();
        for (int n = 0; n < 600; n++)
        {
            bool single = n % 2 == 1;
            var c = Fields.Random(rng, 5, 16, !single);
            float maxErr = rng.Int(3) == 0 ? 0f : (rng.Int(2) + 1) * 0.65f;
            var rs = new RegionSettings { Partition = types[n % 3], MinRegionArea = rng.Int(2) * 4, MergeRegionArea = (types[n % 3] == PartitionType.Layers) ? 0 : rng.Int(3) * 25 };
            Regions.Build(c, rs);
            var set = Contours.Build(c, new ContourSettings { MaxError = maxErr, MaxEdgeLength = rng.Int(3) * 4, KeepRawVertices = true });
            fallbacks += set.RawFallbacks;
            dropped += set.DroppedContours;
            int cat = (single ? 2 : 0) + (maxErr == 0f ? 1 : 0);
            CheckContours(c, set, maxErr, ref checkedContours, ref holes, ref rev[cat], ref pairs[cat]);
        }
        Assert.True(checkedContours > 1000, $"{checkedContours} contours checked");
        Assert.True(holes > 20, $"{holes} holes");
        Assert.Equal(0, dropped);
        string stats = $"matching shared edges [multi-floor, err>0 / multi-floor, err=0 / single-floor, err>0 / single-floor, err=0]: {rev[0]}/{pairs[0]}, {rev[1]}/{pairs[1]}, {rev[2]}/{pairs[2]}, {rev[3]}/{pairs[3]}; {fallbacks} fallbacks";
        Assert.True(pairs.All(p => p > 300), stats);
        // With no error allowance both sides of a shared boundary keep exactly the same vertices. With an allowance they do too, except around
        // regions so small that simplification would erase them: those are re-simplified more strictly, so their neighbours' edges no longer line up.
        Assert.True(rev[1] == pairs[1] && rev[3] == pairs[3], stats);
        Assert.True(rev[0] >= pairs[0] * 0.8 && rev[2] >= pairs[2] * 0.8, stats);
    }

    // The layer partition takes a shortcut (union-find over linked same-area spans) when no column holds two walkable spans; it must give exactly the
    // regions the sweep and merge give, on fields with and without second floors, one or two area ids, holes and small regions.
    [Fact]
    public void LayerShortcutMatchesSweepAndMerge()
    {
        var rng = new TestRng(733);
        int shortcut = 0;
        for (int n = 0; n < 1500; n++)
        {
            bool single = n % 4 != 0;
            var a = Fields.Random(rng, 4, 24, !single);
            var b = a.Clone();
            var rs = new RegionSettings { Partition = PartitionType.Layers, MinRegionArea = rng.Int(3) * 4, MergeRegionArea = 0 };
            Regions.BuildLayers(a, rs);
            Regions.BuildLayersGeneral(b, rs);
            Assert.Equal(Fields.Region(b), Fields.Region(a));
            Assert.Equal(b.RegionCount, a.RegionCount);
            if (single && Fields.Region(a).Any(r => r > 0))
            {
                shortcut++;
            }
        }
        Assert.True(shortcut > 500, $"{shortcut} fields used the shortcut");
    }

    // Regression: a link from one span to a neighbour can exist without the link back (several close floors in a cell). Following such
    // links made contour tracing run off its loop; region and contour work only use mutual links. Here one-way links are made by
    // severing single directions. Holes simplified on their own can also cross the simplified outline; the raw loops are joined then.
    [Fact]
    public void OneWayLinksDoNotBreakContours()
    {
        var rng = new TestRng(301);
        int oneWay = 0;
        for (int n = 0; n < 400; n++)
        {
            var c = Fields.Random(rng, 6, 14);
            for (int i = 0; i < c.SpanCount; i++)
            {
                for (int dir = 0; dir < 4; dir++)
                {
                    if (CompactHeightfield.GetConnection(c.Spans[i], dir) != CompactHeightfield.NotConnected && rng.Int(8) == 0)
                    {
                        c.Spans[i].Connections |= 0x3f << (dir * 6);
                        oneWay++;
                    }
                }
            }
            Regions.Build(c, new RegionSettings { Partition = (PartitionType)(n % 3), MinRegionArea = 0, MergeRegionArea = 0 });
            var set = Contours.Build(c, new ContourSettings { MaxError = 1.3f, MaxEdgeLength = 0, KeepRawVertices = true });
            Assert.Equal(0, set.DroppedContours);
            int a = 0, b = 0, e = 0, g = 0;
            CheckContours(c, set, 1.3f, ref a, ref b, ref e, ref g);
        }
        Assert.True(oneWay > 100);
    }

    [Fact]
    public void PillarMakesAHoleThatIsBridged()
    {
        const int W = 14;
        var dense = new byte[W * W * 14];
        for (int z = 0; z < W; z++)
        {
            for (int x = 0; x < W; x++)
            {
                int cell = ((z * W) + x) * 14;
                dense[cell] = 1;
                if (x >= 5 && x < 9 && z >= 5 && z < 9)
                {
                    for (int y = 1; y < 13; y++)
                    {
                        dense[cell + y] = 1;
                    }
                }
            }
        }
        var frame = new HeightfieldFrame(0, 0, 0, 1f, 1f, W, W, 14);
        var span = new SpanSettings { WalkableHeight = 2, WalkableClimb = 1, WalkableRadius = 0 };
        var pipe = new RegionPipeline(span, new RegionSettings { MinRegionArea = 30, MergeRegionArea = 400 }, new ContourSettings { MaxError = 1.3f, MaxEdgeLength = 0, KeepRawVertices = true });
        pipe.Spans.LedgeFilter = null;
        var set = pipe.BuildFromChunk(frame, VoxelChunk.FromDense(W, W, 14, dense));
        Assert.Single(set.Contours);
        Contour k = set.Contours[0];
        Assert.Single(k.RawHoles!);
        Assert.Equal(16, k.RawHoles![0].Length);
        Assert.Equal(2L * ((W * W) - 16), Contours_Area2(k.Vertices));
        Assert.Equal(0, set.DroppedHoles);
        // The bridge edges carry the region's own id and appear twice, once in each direction.
        int bridges = k.Vertices.Count(v => v.Neighbor == k.Region);
        Assert.Equal(2, bridges);
        // The hole's own edges are walls.
        Assert.True(k.Vertices.Count(v => v.Neighbor == 0) >= 4);
        for (int i = 0; i < k.Vertices.Length; i++)
        {
            for (int j = i + 2; j < k.Vertices.Length; j++)
            {
                if (i == 0 && j == k.Vertices.Length - 1)
                {
                    continue;
                }
                Assert.False(ProperlyCross(k.Vertices[i], k.Vertices[(i + 1) % k.Vertices.Length], k.Vertices[j], k.Vertices[(j + 1) % k.Vertices.Length]));
            }
        }
    }

    [Fact]
    public void RectangleContourIsFourCorners()
    {
        var dense = new byte[10 * 6 * 4];
        for (int c = 0; c < 60; c++)
        {
            dense[c * 4] = 1;
        }
        var frame = new HeightfieldFrame(0, 0, 0, 1f, 1f, 10, 6, 4);
        var pipe = new RegionPipeline(new SpanSettings { WalkableHeight = 2, WalkableClimb = 1, WalkableRadius = 0 }, new RegionSettings { Partition = PartitionType.Monotone, MinRegionArea = 0 }, new ContourSettings { MaxEdgeLength = 0 });
        pipe.Spans.LedgeFilter = null;
        var set = pipe.BuildFromChunk(frame, VoxelChunk.FromDense(10, 6, 4, dense));
        Assert.Single(set.Contours);
        var v = set.Contours[0].Vertices;
        Assert.Equal(4, v.Length);
        Assert.All(v, p => Assert.Equal(Contours.TileBorder, p.Neighbor));
        Assert.Equal(120L, Contours_Area2(v));
    }

    [Fact]
    public void BadRegionIdsAreRejected()
    {
        var c = CompactHeightfield.Build(new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 2, 2, 3)), 2, 1);
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 2, 2, 3));
        hf.AddSpan(0, 0, 1, 63, 0);
        c = CompactHeightfield.Build(hf, 2, 1);
        c.Spans[0].Region = -3;
        Assert.Throws<ArgumentException>(() => Contours.Build(c, new ContourSettings()));
    }
}
