using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OnePath.NavMesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

public class SpanStageTests
{
    private static readonly byte[] AreaChoices = { 0, 63, 10, 63 };

    private static HeightfieldFrame Frame(int w, int d, int h) => new(0, 0, 0, 1f, 1f, w, d, h);

    // ---- merging ----

    private static List<(int Min, int Max, int Area)> NaiveAdd(List<(int Min, int Max, int Area)> list, int min, int max, int area, int thr)
    {
        var rest = new List<(int Min, int Max, int Area)>();
        foreach (var s in list)
        {
            if (s.Min > max || s.Max < min)
            {
                rest.Add(s);
                continue;
            }
            if (s.Max > max)
            {
                area = s.Max - max <= thr ? Math.Max(area, s.Area) : s.Area;
                max = s.Max;
            }
            else if (max - s.Max <= thr)
            {
                area = Math.Max(area, s.Area);
            }
            min = Math.Min(min, s.Min);
        }
        rest.Add((min, max, area));
        rest.Sort((a, b) => a.Min.CompareTo(b.Min));
        return rest;
    }

    [Fact]
    public void AddSpanMergesLikeTheRule()
    {
        var rng = new TestRng(5);
        for (int n = 0; n < 3000; n++)
        {
            var hf = new SolidHeightfield(Frame(1, 1, 24));
            var naive = new List<(int, int, int)>();
            int thr = rng.Int(4);
            for (int k = 0, adds = 1 + rng.Int(8); k < adds; k++)
            {
                int a = rng.Int(23);
                int b = a + 1 + rng.Int(24 - a - 1 + 1 - 1 + 1 > 0 ? Math.Min(6, 24 - a) : 1);
                b = Math.Min(b, 24);
                byte area = AreaChoices[rng.Int(4)];
                hf.AddSpan(0, a, b, area, thr);
                naive = NaiveAdd(naive, a, b, area, thr);
                var got = Dump.Columns(hf)[0];
                Assert.Equal(naive.Count, got.Count);
                for (int i = 0; i < got.Count; i++)
                {
                    Assert.Equal(naive[i], (got[i].Min, got[i].Max, (int)got[i].Area));
                }
                for (int i = 1; i < got.Count; i++)
                {
                    Assert.True(got[i].Min > got[i - 1].Max, "spans must be disjoint and non-touching");
                }
            }
        }
    }

    [Fact]
    public void AddSpanRejectsBadRows()
    {
        var hf = new SolidHeightfield(Frame(2, 2, 5));
        Assert.Throws<ArgumentOutOfRangeException>(() => hf.AddSpan(0, 3, 3, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => hf.AddSpan(0, -1, 3, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => hf.AddSpan(0, 0, 6, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => hf.AddSpan(4, 0, 1, 1, 0));
    }

    // ---- filters against direct definitions ----

    private static SolidHeightfield RandomField(TestRng rng, out int w, out int d, out int h)
    {
        w = 2 + rng.Int(4);
        d = 2 + rng.Int(4);
        h = 10 + rng.Int(14);
        var hf = new SolidHeightfield(Frame(w, d, h));
        for (int c = 0; c < w * d; c++)
        {
            int y = rng.Int(5);
            for (int s = rng.Int(4); s > 0 && y < h - 1; s--)
            {
                int top = Math.Min(h, y + 1 + rng.Int(5));
                hf.AddSpan(c, y, top, AreaChoices[rng.Int(4)], 0);
                y = top + 1 + rng.Int(6);
            }
        }
        return hf;
    }

    private static List<(int Min, int Max, int Area)>[] Snapshot(SolidHeightfield hf)
    {
        var cols = Dump.Columns(hf);
        var r = new List<(int, int, int)>[cols.Length];
        for (int c = 0; c < cols.Length; c++)
        {
            r[c] = new List<(int, int, int)>();
            foreach (var s in cols[c])
            {
                r[c].Add((s.Min, s.Max, s.Area));
            }
        }
        return r;
    }

    private static void AssertAreas(SolidHeightfield hf, List<(int Min, int Max, int Area)>[] expect)
    {
        var got = Snapshot(hf);
        for (int c = 0; c < got.Length; c++)
        {
            Assert.Equal(expect[c], got[c]);
        }
    }

    [Fact]
    public void LowHangingFilterMatchesDefinition()
    {
        var rng = new TestRng(21);
        for (int n = 0; n < 1500; n++)
        {
            var hf = RandomField(rng, out _, out _, out _);
            int climb = rng.Int(5);
            var expect = Snapshot(hf);
            foreach (var col in expect)
            {
                var orig = new List<(int Min, int Max, int Area)>(col);
                for (int i = 1; i < col.Count; i++)
                {
                    if (orig[i].Area == 0 && orig[i - 1].Area != 0 && orig[i].Max - orig[i - 1].Max <= climb)
                    {
                        col[i] = (col[i].Min, col[i].Max, orig[i - 1].Area);
                    }
                }
            }
            SpanFilters.FilterLowHangingWalkableObstacles(hf, climb);
            AssertAreas(hf, expect);
        }
    }

    [Fact]
    public void LowHeightFilterMatchesDefinition()
    {
        var rng = new TestRng(22);
        for (int n = 0; n < 1500; n++)
        {
            var hf = RandomField(rng, out _, out _, out _);
            int height = 1 + rng.Int(6);
            var expect = Snapshot(hf);
            foreach (var col in expect)
            {
                for (int i = 0; i + 1 < col.Count; i++)
                {
                    if (col[i].Area != 0 && col[i + 1].Min - col[i].Max < height)
                    {
                        col[i] = (col[i].Min, col[i].Max, 0);
                    }
                }
            }
            SpanFilters.FilterWalkableLowHeightSpans(hf, height);
            AssertAreas(hf, expect);
        }
    }

    // Ledge definition written over plain lists: the floors an agent could step to next to a span.
    private static bool LedgeOracle(List<(int Min, int Max, int Area)>[] cols, int w, int d, int x, int z, int idx, int height, int climb)
    {
        var col = cols[x + (z * w)];
        int floor = col[idx].Max;
        int ceil = idx + 1 < col.Count ? col[idx + 1].Min : int.MaxValue / 2;
        int[] dx = { -1, 0, 1, 0 };
        int[] dz = { 0, 1, 0, -1 };
        var accessible = new List<int>();
        for (int k = 0; k < 4; k++)
        {
            int nx = x + dx[k], nz = z + dz[k];
            if (nx < 0 || nz < 0 || nx >= w || nz >= d)
            {
                return true; // nothing beyond the grid: a drop
            }
            var floors = new List<(int Floor, int Ceil)>();
            var nc = cols[nx + (nz * w)];
            floors.Add((-climb, nc.Count > 0 ? nc[0].Min : int.MaxValue / 2));
            for (int s = 0; s < nc.Count; s++)
            {
                floors.Add((nc[s].Max, s + 1 < nc.Count ? nc[s + 1].Min : int.MaxValue / 2));
            }
            foreach (var (nf, ncl) in floors)
            {
                bool passable = Math.Min(ceil, ncl) - Math.Max(floor, nf) >= height;
                if (!passable)
                {
                    continue;
                }
                if (nf - floor < -climb)
                {
                    return true;
                }
                if (Math.Abs(nf - floor) <= climb)
                {
                    accessible.Add(nf);
                }
            }
        }
        return accessible.Count > 0 && accessible.Max() - accessible.Min() > climb;
    }

    [Fact]
    public void LedgeFilterMatchesDefinition()
    {
        var rng = new TestRng(23);
        int ledges = 0, kept = 0;
        for (int n = 0; n < 2000; n++)
        {
            var hf = RandomField(rng, out int w, out int d, out _);
            int climb = rng.Int(4);
            int height = 1 + rng.Int(4);
            var before = Snapshot(hf);
            var expect = Snapshot(hf);
            for (int z = 0; z < d; z++)
            {
                for (int x = 0; x < w; x++)
                {
                    var col = expect[x + (z * w)];
                    for (int i = 0; i < col.Count; i++)
                    {
                        if (col[i].Area != 0)
                        {
                            if (LedgeOracle(before, w, d, x, z, i, height, climb))
                            {
                                col[i] = (col[i].Min, col[i].Max, 0);
                                ledges++;
                            }
                            else
                            {
                                kept++;
                            }
                        }
                    }
                }
            }
            SpanFilters.FilterLedgeSpans(hf, height, climb);
            AssertAreas(hf, expect);
        }
        Assert.True(ledges > 500 && kept > 500, $"weak coverage: {ledges} ledges, {kept} kept");
    }

    [Fact]
    public void FiltersRejectBadArguments()
    {
        var hf = new SolidHeightfield(Frame(2, 2, 5));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpanFilters.FilterLedgeSpans(hf, 0, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpanFilters.FilterLedgeSpans(hf, 1, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpanFilters.FilterLowHangingWalkableObstacles(hf, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => SpanFilters.FilterWalkableLowHeightSpans(hf, 0));
    }

    // ---- compact heightfield, erosion, volumes ----

    private static CompactHeightfield RandomCompact(TestRng rng, out int w, out int d, out int height, out int climb)
    {
        var hf = RandomField(rng, out w, out d, out int h);
        height = 1 + rng.Int(4);
        climb = rng.Int(4);
        return CompactHeightfield.Build(hf, height, climb);
    }

    [Fact]
    public void CompactLinksMatchDefinition()
    {
        var rng = new TestRng(31);
        int links = 0;
        for (int n = 0; n < 1500; n++)
        {
            var solid = RandomField(rng, out int w, out int d, out _);
            int height = 1 + rng.Int(4);
            int climb = rng.Int(4);
            var snap = Snapshot(solid);
            var chf = CompactHeightfield.Build(solid, height, climb);
            // Direct per-column lists of (floor, free height).
            var cells = new List<(int Y, int H, int Area)>[w * d];
            for (int c = 0; c < cells.Length; c++)
            {
                cells[c] = new List<(int, int, int)>();
                for (int i = 0; i < snap[c].Count; i++)
                {
                    if (snap[c][i].Area != 0)
                    {
                        int hh = i + 1 < snap[c].Count ? snap[c][i + 1].Min - snap[c][i].Max : 65535;
                        cells[c].Add((snap[c][i].Max, hh, snap[c][i].Area));
                    }
                }
                Assert.Equal(cells[c].Count, chf.CellCount[c]);
            }
            int[] dx = { -1, 0, 1, 0 };
            int[] dz = { 0, 1, 0, -1 };
            for (int z = 0; z < d; z++)
            {
                for (int x = 0; x < w; x++)
                {
                    int c = x + (z * w);
                    for (int i = 0; i < cells[c].Count; i++)
                    {
                        CompactSpan s = chf.Spans[chf.CellFirst[c] + i];
                        Assert.Equal(cells[c][i], (s.Y, s.Height, (int)s.Area));
                        for (int k = 0; k < 4; k++)
                        {
                            int expect = CompactHeightfield.NotConnected;
                            int nx = x + dx[k], nz = z + dz[k];
                            if (nx >= 0 && nz >= 0 && nx < w && nz < d)
                            {
                                var nc = cells[nx + (nz * w)];
                                for (int m = 0; m < nc.Count; m++)
                                {
                                    int top = Math.Min(cells[c][i].Y + cells[c][i].H, nc[m].Y + nc[m].H);
                                    int bot = Math.Max(cells[c][i].Y, nc[m].Y);
                                    if (top - bot >= height && Math.Abs(nc[m].Y - cells[c][i].Y) <= climb)
                                    {
                                        expect = m;
                                        break;
                                    }
                                }
                            }
                            Assert.Equal(expect, CompactHeightfield.GetConnection(s, k));
                            if (expect != CompactHeightfield.NotConnected)
                            {
                                links++;
                            }
                        }
                    }
                }
            }
        }
        Assert.True(links > 5000, $"weak coverage: {links} links");
    }

    [Fact]
    public void ErosionDistanceMatchesShortestPaths()
    {
        var rng = new TestRng(41);
        int eroded = 0, kept = 0;
        for (int n = 0; n < 400; n++)
        {
            var hf = new SolidHeightfield(Frame(10, 10, 6));
            for (int c = 0; c < 100; c++)
            {
                if (rng.Int(10) < 9)
                {
                    hf.AddSpan(c, 0, 1 + rng.Int(2), AreaChoices[1 + rng.Int(3)], 0);
                }
            }
            var chf = CompactHeightfield.Build(hf, 1, 1);
            int radius = 1 + rng.Int(3);
            int cap = 2 * radius;
            int[] dist = Erosion.ComputeBoundaryDistance(chf, cap);
            // Oracle: Dijkstra from every span over its outgoing links.
            int count = chf.SpanCount;
            var first = chf.CellFirst.ToArray();
            int Target(int cell, int idx, int dir, out bool ok)
            {
                int con = CompactHeightfield.GetConnection(chf.Spans[first[cell] + idx], dir);
                ok = con != CompactHeightfield.NotConnected;
                if (!ok)
                {
                    return -1;
                }
                int nx = (cell % 10) + CompactHeightfield.DirX[dir];
                int nz = (cell / 10) + CompactHeightfield.DirZ[dir];
                return first[nx + (nz * 10)] + con;
            }
            var cellOf = new int[count];
            var idxOf = new int[count];
            for (int c = 0; c < 100; c++)
            {
                for (int i = 0; i < chf.CellCount[c]; i++)
                {
                    cellOf[first[c] + i] = c;
                    idxOf[first[c] + i] = i;
                }
            }
            bool Boundary(int s)
            {
                if (chf.Spans[s].Area == 0)
                {
                    return true;
                }
                for (int k = 0; k < 4; k++)
                {
                    int t = Target(cellOf[s], idxOf[s], k, out bool ok);
                    if (!ok || chf.Spans[t].Area == 0)
                    {
                        return true;
                    }
                }
                return false;
            }
            for (int s = 0; s < count; s++)
            {
                var best = new int[count];
                Array.Fill(best, int.MaxValue);
                var done = new bool[count];
                best[s] = 0;
                int answer = cap;
                for (int step = 0; step < count; step++)
                {
                    int u = -1;
                    for (int q = 0; q < count; q++)
                    {
                        if (!done[q] && best[q] != int.MaxValue && (u == -1 || best[q] < best[u]))
                        {
                            u = q;
                        }
                    }
                    if (u == -1 || best[u] >= cap)
                    {
                        break;
                    }
                    if (Boundary(u))
                    {
                        answer = best[u];
                        break;
                    }
                    done[u] = true;
                    for (int k = 0; k < 4; k++)
                    {
                        int t = Target(cellOf[u], idxOf[u], k, out bool ok);
                        if (!ok)
                        {
                            continue;
                        }
                        best[t] = Math.Min(best[t], best[u] + 2);
                        for (int e = 1; e < 4; e += 2)
                        {
                            int m = Target(cellOf[t], idxOf[t], (k + e) & 3, out bool ok2);
                            if (ok2)
                            {
                                best[m] = Math.Min(best[m], best[u] + 3);
                            }
                        }
                    }
                }
                Assert.Equal(answer, dist[s]);
            }
            var areasBefore = new byte[count];
            for (int s = 0; s < count; s++)
            {
                areasBefore[s] = chf.Spans[s].Area;
            }
            Erosion.ErodeWalkableArea(chf, radius);
            for (int s = 0; s < count; s++)
            {
                Assert.Equal(dist[s] < cap ? 0 : areasBefore[s], chf.Spans[s].Area);
                if (dist[s] < cap)
                {
                    eroded++;
                }
                else
                {
                    kept++;
                }
            }
        }
        Assert.True(eroded > 1000 && kept > 1000, $"weak coverage: {eroded} eroded, {kept} kept");
    }

    [Fact]
    public void ErosionOfAFlatSquareLeavesTheCentre()
    {
        var hf = new SolidHeightfield(Frame(9, 9, 4));
        for (int c = 0; c < 81; c++)
        {
            hf.AddSpan(c, 0, 1, 63, 0);
        }
        var chf = CompactHeightfield.Build(hf, 2, 1);
        Erosion.ErodeWalkableArea(chf, 2);
        int walkable = 0;
        for (int i = 0; i < chf.SpanCount; i++)
        {
            walkable += chf.Spans[i].Area != 0 ? 1 : 0;
        }
        // Boundary spans are at distance 0; distance 2*k from the edge ring needs k cells in: radius 2 removes the outer 2 rings.
        Assert.Equal(25, walkable);
        Assert.Throws<ArgumentOutOfRangeException>(() => Erosion.ErodeWalkableArea(chf, -1));
    }

    [Fact]
    public void VolumeMarksSpansByCellCentreAndHeight()
    {
        var rng = new TestRng(51);
        for (int n = 0; n < 200; n++)
        {
            var hf = new SolidHeightfield(new HeightfieldFrame(1.5, -2, 0.25, 0.5f, 0.25f, 12, 12, 16));
            for (int c = 0; c < 144; c++)
            {
                hf.AddSpan(c, rng.Int(6), 6 + rng.Int(6), 63, 0);
            }
            var chf = CompactHeightfield.Build(hf, 1, 8);
            // A convex quadrilateral (rotated square) with random placement, either winding.
            double cx = rng.Range(2, 6), cz = rng.Range(1, 5), r = rng.Range(0.5, 2.5);
            double[] xz = { cx - r, cz, cx, cz - r, cx + r, cz, cx, cz + r };
            if (rng.Int(2) == 0)
            {
                Array.Reverse(xz);
                for (int i = 0; i < 8; i += 2)
                {
                    (xz[i], xz[i + 1]) = (xz[i + 1], xz[i]);
                }
            }
            double y0 = rng.Range(-2, 0), y1 = y0 + rng.Range(0.2, 3);
            var vol = new ConvexVolume(xz, y0, y1, 7);
            var before = new byte[chf.SpanCount];
            for (int i = 0; i < before.Length; i++)
            {
                before[i] = chf.Spans[i].Area;
            }
            int marked = vol.Apply(chf);
            var f = chf.Frame;
            int expectMarked = 0;
            for (int z = 0; z < 12; z++)
            {
                for (int x = 0; x < 12; x++)
                {
                    double px = 1.5 + ((x + 0.5) * 0.5), pz = 0.25 + ((z + 0.5) * 0.5);
                    bool inside = Math.Abs(px - cx) + Math.Abs(pz - cz) <= r;
                    for (int i = chf.CellFirst[x + (z * 12)]; i < chf.CellFirst[x + (z * 12)] + chf.CellCount[x + (z * 12)]; i++)
                    {
                        double wy = -2 + (chf.Spans[i].Y * 0.25);
                        bool expect = inside && wy >= y0 && wy <= y1;
                        Assert.Equal(expect ? 7 : before[i], chf.Spans[i].Area);
                        expectMarked += expect ? 1 : 0;
                    }
                }
            }
            Assert.Equal(expectMarked, marked);
        }
    }

    [Fact]
    public void VolumeRejectsBadPolygons()
    {
        Assert.Throws<ArgumentException>(() => new ConvexVolume(new double[] { 0, 0, 1, 1 }, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => new ConvexVolume(new double[] { 0, 0, 1, 1, 2, 2 }, 0, 1, 1)); // zero area
        Assert.Throws<ArgumentException>(() => new ConvexVolume(new double[] { 0, 0, 4, 0, 1, 1, 0, 4 }, 0, 1, 1)); // concave
        Assert.Throws<ArgumentException>(() => new ConvexVolume(new double[] { 0, 0, 1, 0, double.NaN, 1 }, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => new ConvexVolume(new double[] { 0, 0, 1, 0, 0, 1 }, 2, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new ConvexVolume(new double[] { 0, 0, 1, 0, 3e6, 1 }, 0, 1, 1));
    }
}
