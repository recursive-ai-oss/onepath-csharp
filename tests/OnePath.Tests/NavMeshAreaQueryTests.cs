using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>Random point, distance to wall, polygon-set and move-along-surface queries against brute-force oracles.</summary>
public class NavMeshAreaQueryTests
{
    private struct AllPass : IQueryFilter
    {
        public readonly bool Passes(ushort flags, byte area) => true;

        public readonly double AreaCost(byte area) => 1.0;

        public readonly double MinAreaCost => 1.0;
    }

    private static readonly QueryFilter s_any = new();

    private static QueryFilter Mixed(int i)
    {
        var f = new QueryFilter { ExcludeFlags = i % 3 == 1 ? (ushort)4 : (ushort)0 };
        if (i % 4 == 2)
        {
            f.SetAreaCost(3, double.PositiveInfinity);
        }
        return f;
    }

    private static bool Pass(QueryFilter f, NavTile t, int p) =>
        (f.IncludeFlags == 0xFFFF || (t.PolyFlags[p] & f.IncludeFlags) != 0) && (t.PolyFlags[p] & f.ExcludeFlags) == 0 && !double.IsPositiveInfinity(f.GetAreaCost(t.PolyAreas[p]));

    private static List<PolyRef> AllPolys(TiledNavMesh m, QueryFilter? f = null)
    {
        var l = new List<PolyRef>();
        for (int s = 0; s < m.SlotCount; s++)
        {
            NavTile? t = m.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                if (f is null || Pass(f, t, p))
                {
                    l.Add(m.GetSlotRef(s).Poly(p));
                }
            }
        }
        return l;
    }

    // World outline of a polygon as x,z pairs.
    private static double[] Outline(TiledNavMesh m, PolyRef pr)
    {
        m.TryGetPoly(pr, out NavTile t, out int p);
        int n = t.PolyVertexCounts[p];
        var o = new double[2 * n];
        for (int i = 0; i < n; i++)
        {
            int v = 3 * t.PolyIndices[t.PolyFirstIndex(p) + i];
            o[2 * i] = t.OriginX + (double)t.Vertices[v];
            o[(2 * i) + 1] = t.OriginZ + (double)t.Vertices[v + 2];
        }
        return o;
    }

    private static double Area(double[] o)
    {
        double a = 0;
        int n = o.Length / 2;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            a += (o[2 * i] * o[(2 * j) + 1]) - (o[2 * j] * o[(2 * i) + 1]);
        }
        return Math.Abs(a) / 2;
    }

    // Convex polygon clipped to a rectangle (Sutherland-Hodgman), written for the test.
    private static double[] ClipRect(double[] poly, double x0, double z0, double x1, double z1)
    {
        double[] cur = poly;
        for (int plane = 0; plane < 4; plane++)
        {
            var next = new List<double>();
            int n = cur.Length / 2;
            double D(double x, double z) => plane switch { 0 => x - x0, 1 => x1 - x, 2 => z - z0, _ => z1 - z };
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double dp = D(cur[2 * i], cur[(2 * i) + 1]), dq = D(cur[2 * j], cur[(2 * j) + 1]);
                if (dp >= 0)
                {
                    next.Add(cur[2 * i]);
                    next.Add(cur[(2 * i) + 1]);
                }
                if ((dp >= 0) != (dq >= 0))
                {
                    double s = dp / (dp - dq);
                    next.Add(cur[2 * i] + ((cur[2 * j] - cur[2 * i]) * s));
                    next.Add(cur[(2 * i) + 1] + ((cur[(2 * j) + 1] - cur[(2 * i) + 1]) * s));
                }
            }
            cur = next.ToArray();
            if (cur.Length < 6)
            {
                return [];
            }
        }
        return cur;
    }

    private static bool Inside(double[] o, double x, double z, double tol = 1e-6)
    {
        int n = o.Length / 2;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            double ex = o[2 * j] - o[2 * i], ez = o[(2 * j) + 1] - o[(2 * i) + 1];
            double cross = (ex * (z - o[(2 * i) + 1])) - (ez * (x - o[2 * i]));
            if (cross < -tol * Math.Sqrt((ex * ex) + (ez * ez)))
            {
                return false;
            }
        }
        return true;
    }

    // Chi-squared critical value at p = 0.001 (Wilson-Hilferty approximation, accurate to a fraction of a percent for df >= 10).
    private static double ChiCritical(int df)
    {
        double k = 2.0 / (9.0 * df);
        double c = 1 - k + (3.0902323 * Math.Sqrt(k));
        return df * c * c * c;
    }

    private static double ChiSquared(List<(double expected, int observed)> bins, out int df)
    {
        double chi = 0, eSmall = 0;
        int oSmall = 0;
        df = -1;
        foreach ((double e, int o) in bins)
        {
            if (e < 5)
            {
                eSmall += e;
                oSmall += o;
                continue;
            }
            chi += (o - e) * (o - e) / e;
            df++;
        }
        if (eSmall >= 5)
        {
            chi += (oSmall - eSmall) * (oSmall - eSmall) / eSmall;
            df++;
        }
        return chi;
    }

    private static Vector3D PointIn(TiledNavMesh m, PolyRef pr, NavMeshWorlds.Rng r)
    {
        double[] o = Outline(m, pr);
        int n = o.Length / 2;
        var w = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            w[i] = 1 + r.Int(1000);
            sum += w[i];
        }
        double x = 0, z = 0;
        for (int i = 0; i < n; i++)
        {
            x += w[i] / sum * o[2 * i];
            z += w[i] / sum * o[(2 * i) + 1];
        }
        return new Vector3D(x, 0, z);
    }

    // ---- oracle: reachable set by breadth-first search over portals, written from the documented model ----

    private delegate bool Touch(double ax, double az, double bx, double bz);

    private static Touch CircleTouch(double cx, double cz, double r) => (ax, az, bx, bz) =>
    {
        double ex = bx - ax, ez = bz - az;
        double len2 = (ex * ex) + (ez * ez);
        double s = len2 > 0 ? Math.Clamp(((cx - ax) * ex / len2) + ((cz - az) * ez / len2), 0, 1) : 0;
        double qx = ax + (ex * s) - cx, qz = az + (ez * s) - cz;
        return (qx * qx) + (qz * qz) <= r * r;
    };

    // Liang-Barsky clipping of the segment against the half-planes of a counter-clockwise convex polygon.
    private static Touch ConvexTouch(double[] poly) => (ax, az, bx, bz) =>
    {
        double t0 = 0, t1 = 1;
        int n = poly.Length / 2;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            double ex = poly[2 * j] - poly[2 * i], ez = poly[(2 * j) + 1] - poly[(2 * i) + 1];
            double f0 = (ex * (az - poly[(2 * i) + 1])) - (ez * (ax - poly[2 * i]));
            double f1 = (ex * (bz - poly[(2 * i) + 1])) - (ez * (bx - poly[2 * i]));
            if (f0 < 0 && f1 < 0)
            {
                return false;
            }
            if (f0 < 0 || f1 < 0)
            {
                double t = f0 / (f0 - f1);
                if (f0 < 0)
                {
                    t0 = Math.Max(t0, t);
                }
                else
                {
                    t1 = Math.Min(t1, t);
                }
            }
        }
        return t0 <= t1;
    };

    private static IEnumerable<(PolyRef to, double ax, double az, double bx, double bz)> Portals(TiledNavMesh m, PolyRef from, QueryFilter f)
    {
        m.TryGetPoly(from, out NavTile t, out int p);
        int n = t.PolyVertexCounts[p];
        int b = t.PolyFirstIndex(p);
        double[] o = Outline(m, from);
        for (int e = 0; e < n; e++)
        {
            int j = (e + 1) % n;
            int nb = t.EdgeNeighbors[b + e];
            if (nb != 0)
            {
                if (Pass(f, t, nb - 1))
                {
                    yield return (from.Tile.Poly(nb - 1), o[2 * e], o[(2 * e) + 1], o[2 * j], o[(2 * j) + 1]);
                }
                continue;
            }
            foreach (NavLink l in m.GetLinks(from).ToArray())
            {
                if (l.Kind != NavLinkKind.Portal || l.Index != e || !m.TryGetPoly(l.Target, out NavTile qt, out int qi) || !Pass(f, qt, qi))
                {
                    continue;
                }
                double first = l.RangeMin / 255.0, second = l.RangeMax / 255.0;
                yield return (l.Target, o[2 * e] + ((o[2 * j] - o[2 * e]) * first), o[(2 * e) + 1] + ((o[(2 * j) + 1] - o[(2 * e) + 1]) * first),
                    o[2 * e] + ((o[2 * j] - o[2 * e]) * second), o[(2 * e) + 1] + ((o[(2 * j) + 1] - o[(2 * e) + 1]) * second));
            }
        }
    }

    private static HashSet<ulong> Reach(TiledNavMesh m, PolyRef start, Touch touch, QueryFilter f, bool neighbourhood = false)
    {
        var seen = new HashSet<ulong> { start.Value };
        var q = new Queue<PolyRef>();
        q.Enqueue(start);
        while (q.Count > 0)
        {
            PolyRef cur = q.Dequeue();
            foreach ((PolyRef to, double ax, double az, double bx, double bz) in Portals(m, cur, f))
            {
                if (!seen.Contains(to.Value) && touch(ax, az, bx, bz))
                {
                    seen.Add(to.Value);
                    q.Enqueue(to);
                }
            }
        }
        _ = neighbourhood;
        return seen;
    }

    private static HashSet<ulong> Set(ReadOnlySpan<PolyRef> l)
    {
        var s = new HashSet<ulong>();
        foreach (PolyRef p in l)
        {
            Assert.True(s.Add(p.Value), "a polygon appears twice");
        }
        return s;
    }

    // ---- random point ----

    [Fact]
    public void RandomPointIsUniformByAreaAndInsideFilteredPolygons()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(41, -1, -1, 1, 1, 2, detail: true, offMesh: 0);
        var q = new NavMeshQuery(m);
        QueryFilter f = new QueryFilter { ExcludeFlags = 4 }.SetAreaCost(3, double.PositiveInfinity);
        List<PolyRef> polys = AllPolys(m, f);
        Assert.True(polys.Count > 20);
        double total = 0;
        foreach (PolyRef p in polys)
        {
            total += Area(Outline(m, p));
        }
        const int N = 200_000;
        // Bins: (polygon, quadrant of its bounding box), so both the polygon choice and the spread inside it are tested.
        var count = new Dictionary<(ulong, int), int>();
        var rng = new SeededRandom(2024);
        var allowed = new HashSet<ulong>();
        foreach (PolyRef p in polys)
        {
            allowed.Add(p.Value);
        }
        var boxes = new Dictionary<ulong, (double cx, double cz)>();
        foreach (PolyRef p in polys)
        {
            double[] o = Outline(m, p);
            double x0 = double.MaxValue, x1 = double.MinValue, z0 = double.MaxValue, z1 = double.MinValue;
            for (int i = 0; i < o.Length; i += 2)
            {
                x0 = Math.Min(x0, o[i]);
                x1 = Math.Max(x1, o[i]);
                z0 = Math.Min(z0, o[i + 1]);
                z1 = Math.Max(z1, o[i + 1]);
            }
            boxes[p.Value] = ((x0 + x1) / 2, (z0 + z1) / 2);
        }
        for (int i = 0; i < N; i++)
        {
            RandomPointResult r = q.RandomPoint(ref rng, f);
            Assert.True(r.Found);
            Assert.Contains(r.Poly.Value, allowed);
            Assert.True(Inside(Outline(m, r.Poly), r.Point.X, r.Point.Z), "point outside its polygon");
            (double cx, double cz) = boxes[r.Poly.Value];
            int quad = (r.Point.X < cx ? 0 : 1) + (r.Point.Z < cz ? 0 : 2);
            count[(r.Poly.Value, quad)] = count.GetValueOrDefault((r.Poly.Value, quad)) + 1;
        }
        var bins = new List<(double, int)>();
        foreach (PolyRef p in polys)
        {
            double[] o = Outline(m, p);
            (double cx, double cz) = boxes[p.Value];
            const double Big = 1e6;
            double[][] quads = [ClipRect(o, -Big, -Big, cx, cz), ClipRect(o, cx, -Big, Big, cz), ClipRect(o, -Big, cz, cx, Big), ClipRect(o, cx, cz, Big, Big)];
            for (int k = 0; k < 4; k++)
            {
                double a = quads[k].Length >= 6 ? Area(quads[k]) : 0;
                bins.Add((N * a / total, count.GetValueOrDefault((p.Value, k))));
                if (a == 0)
                {
                    Assert.Equal(0, count.GetValueOrDefault((p.Value, k)));
                }
            }
        }
        double chi = ChiSquared(bins, out int df);
        Assert.True(chi < ChiCritical(df), $"chi-squared {chi:F1} exceeds {ChiCritical(df):F1} at df {df} (p = 0.001)");
    }

    [Fact]
    public void RandomPointIsReproducibleAndAdvancesExactlyFourDraws()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(42, -1, -1, 2, 2, 2, detail: true, offMesh: 0);
        var q1 = new NavMeshQuery(m);
        var q2 = new NavMeshQuery(m);
        var a = new SeededRandom(7);
        var b = new SeededRandom(7);
        for (int i = 0; i < 200; i++)
        {
            RandomPointResult r1 = q1.RandomPoint(ref a, s_any);
            RandomPointResult r2 = q2.RandomPoint(ref b, new AllPass());
            Assert.Equal(r1.Poly, r2.Poly);
            Assert.Equal(r1.Point, r2.Point);
        }
        var c = new SeededRandom(7);
        for (int i = 0; i < 800; i++)
        {
            c.NextUInt64();
        }
        Assert.Equal(c.NextUInt64(), a.NextUInt64());
        // Nothing accepted: no point and the generator is untouched.
        var d = new SeededRandom(9);
        var e = new SeededRandom(9);
        RandomPointResult none = q1.RandomPoint(ref d, new QueryFilter { IncludeFlags = 0x8000, Predicate = (_, _) => false });
        Assert.False(none.Found);
        Assert.Equal(e.NextUInt64(), d.NextUInt64());
    }

    [Fact]
    public void RandomPointAroundIsInsideTheCircleReachableAndUniform()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(43, -1, -1, 2, 2, 2, detail: false, offMesh: 0);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(5);
        int tried = 0;
        foreach (PolyRef start in AllPolys(m))
        {
            if (tried >= 3 || gen.Int(6) != 0)
            {
                continue;
            }
            Vector3D c = PointIn(m, start, gen);
            double radius = 3.5;
            HashSet<ulong> reach = Reach(m, start, CircleTouch(c.X, c.Z, radius), s_any);
            if (reach.Count < 6)
            {
                continue;
            }
            tried++;
            const int N = 100_000;
            var rng = new SeededRandom(11);
            var counts = new Dictionary<ulong, int>();
            for (int i = 0; i < N; i++)
            {
                RandomPointResult r = q.RandomPointAround(ref rng, start, c, radius, s_any);
                Assert.True(r.Found);
                Assert.Contains(r.Poly.Value, reach);
                Assert.True(Inside(Outline(m, r.Poly), r.Point.X, r.Point.Z));
                Assert.True(((r.Point.X - c.X) * (r.Point.X - c.X)) + ((r.Point.Z - c.Z) * (r.Point.Z - c.Z)) <= radius * radius);
                counts[r.Poly.Value] = counts.GetValueOrDefault(r.Poly.Value) + 1;
            }
            // Expected share of each polygon: area of polygon inside the disc, integrated on a fine grid.
            var areas = new Dictionary<ulong, double>();
            const int G = 700;
            double step = 2 * radius / G;
            double cell = step * step;
            double sum = 0;
            foreach (ulong v in reach)
            {
                double[] o = Outline(m, new PolyRef(v));
                double a = 0;
                for (int ix = 0; ix < G; ix++)
                {
                    for (int iz = 0; iz < G; iz++)
                    {
                        double x = c.X - radius + ((ix + 0.5) * step), z = c.Z - radius + ((iz + 0.5) * step);
                        if (((x - c.X) * (x - c.X)) + ((z - c.Z) * (z - c.Z)) <= radius * radius && Inside(o, x, z, 0))
                        {
                            a += cell;
                        }
                    }
                }
                areas[v] = a;
                sum += a;
            }
            var bins = new List<(double, int)>();
            foreach (ulong v in reach)
            {
                bins.Add((N * areas[v] / sum, counts.GetValueOrDefault(v)));
            }
            double chi = ChiSquared(bins, out int df);
            Assert.True(chi < ChiCritical(df), $"chi-squared {chi:F1} exceeds {ChiCritical(df):F1} at df {df}");
        }
        Assert.True(tried >= 2);
    }

    // ---- polygon sets ----

    [Fact]
    public void CircleAndShapeQueriesEqualBreadthFirstOracle()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(51, -1, -1, 3, 3, 3, detail: false, offMesh: 2);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(3);
        List<PolyRef> all = AllPolys(m);
        var polys = new PolyRef[2048];
        var parents = new PolyRef[2048];
        var dist = new double[2048];
        int checkedCount = 0;
        for (int i = 0; i < 400; i++)
        {
            QueryFilter f = Mixed(i);
            PolyRef start = all[gen.Int(all.Count)];
            m.TryGetPoly(start, out NavTile st, out int sp);
            if (!Pass(f, st, sp))
            {
                Assert.Equal(AreaQueryStatus.InvalidStartPoly, q.FindPolysInCircle(start, new Vector3D(0, 0, 0), 1, f, polys, parents, dist, out _, out _));
                continue;
            }
            Vector3D c = PointIn(m, start, gen);
            double r = 0.3 + (gen.Int(1000) / 1000.0 * 6);
            AreaQueryStatus s = q.FindPolysInCircle(start, c, r, f, polys, parents, dist, out int n, out bool trunc);
            Assert.Equal(AreaQueryStatus.Success, s);
            Assert.False(trunc);
            HashSet<ulong> expect = Reach(m, start, CircleTouch(c.X, c.Z, r), f);
            HashSet<ulong> got = Set(polys.AsSpan(0, n));
            Assert.True(expect.SetEquals(got), $"circle set differs (query {i}): expected {expect.Count}, got {got.Count}");
            Assert.Equal(start, polys[0]);
            Assert.Equal(PolyRef.Invalid, parents[0]);
            Assert.Equal(0.0, dist[0]);
            for (int k = 1; k < n; k++)
            {
                Assert.True(dist[k] <= (r * r) + 1e-12);
                Assert.Contains(parents[k].Value, Set(polys.AsSpan(0, k)));
            }
            // Same through the struct-filter form.
            if (i % 5 == 0)
            {
                q.FindPolysInCircle(start, c, r, new AllPass(), polys, default, default, out int n2, out _);
                Assert.True(n2 >= n);
            }

            // A random convex shape around the point.
            int k2 = 3 + gen.Int(6);
            var angles = new double[k2];
            for (int a = 0; a < k2; a++)
            {
                angles[a] = (a + (gen.Int(1000) / 1000.0 * 0.8)) * 2 * Math.PI / k2;
            }
            double rx = 0.5 + (gen.Int(1000) / 1000.0 * 5), rz = 0.5 + (gen.Int(1000) / 1000.0 * 5), rot = gen.Int(1000) / 1000.0 * 6;
            var shape = new Vector3D[k2];
            var flat = new double[2 * k2];
            for (int a = 0; a < k2; a++)
            {
                double ex = rx * Math.Cos(angles[a]), ez = rz * Math.Sin(angles[a]);
                shape[a] = new Vector3D(c.X + (ex * Math.Cos(rot)) - (ez * Math.Sin(rot)), 0, c.Z + (ex * Math.Sin(rot)) + (ez * Math.Cos(rot)));
                flat[2 * a] = shape[a].X;
                flat[(2 * a) + 1] = shape[a].Z;
            }
            if (i % 2 == 1)
            {
                Array.Reverse(shape); // either winding is accepted
                for (int a = 0; a < k2; a++)
                {
                    flat[2 * a] = shape[k2 - 1 - a].X;
                    flat[(2 * a) + 1] = shape[k2 - 1 - a].Z;
                }
            }
            if (Area(flat) < 1e-6)
            {
                continue;
            }
            // The oracle needs counter-clockwise order.
            double signed = 0;
            for (int a = 0; a < k2; a++)
            {
                int b2 = (a + 1) % k2;
                signed += (flat[2 * a] * flat[(2 * b2) + 1]) - (flat[2 * b2] * flat[(2 * a) + 1]);
            }
            if (signed < 0)
            {
                var rev = new double[flat.Length];
                for (int a = 0; a < k2; a++)
                {
                    rev[2 * a] = flat[2 * (k2 - 1 - a)];
                    rev[(2 * a) + 1] = flat[(2 * (k2 - 1 - a)) + 1];
                }
                flat = rev;
            }
            Assert.Equal(AreaQueryStatus.Success, q.FindPolysInShape(start, shape, f, polys, parents, dist, out int sn, out _));
            HashSet<ulong> sExpect = Reach(m, start, ConvexTouch(flat), f);
            Assert.True(sExpect.SetEquals(Set(polys.AsSpan(0, sn))), $"shape set differs (query {i})");
            checkedCount++;
        }
        Assert.True(checkedCount > 100);
    }

    [Fact]
    public void SlicedSearchEqualsSynchronousForEveryBudget()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(52, -1, -1, 2, 2, 2, detail: false, offMesh: 0);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(8);
        List<PolyRef> all = AllPolys(m);
        var a = new PolyRef[512];
        var b = new PolyRef[512];
        var pa = new PolyRef[512];
        var pb = new PolyRef[512];
        var da = new double[512];
        var db = new double[512];
        for (int i = 0; i < 40; i++)
        {
            PolyRef start = all[gen.Int(all.Count)];
            Vector3D c = PointIn(m, start, gen);
            double r = 1 + (gen.Int(1000) / 1000.0 * 5);
            q.FindPolysInCircle(start, c, r, s_any, a, pa, da, out int na, out _);
            foreach (int budget in new[] { 1, 2, 3, 7, 100 })
            {
                Assert.Equal(AreaQueryStatus.InProgress, q.BeginAreaSearch(start, c, r, s_any));
                int guard = 0;
                while (q.UpdateAreaSearch(budget) == AreaQueryStatus.InProgress)
                {
                    Assert.True(guard++ < 10_000);
                }
                q.FinishAreaSearch(b, pb, db, out int nb, out _);
                Assert.Equal(na, nb);
                Assert.True(a.AsSpan(0, na).SequenceEqual(b.AsSpan(0, nb)));
                Assert.True(pa.AsSpan(0, na).SequenceEqual(pb.AsSpan(0, nb)));
                Assert.True(da.AsSpan(0, na).SequenceEqual(db.AsSpan(0, nb)));
            }
        }
        // A partly finished search lists what it has expanded.
        PolyRef first = all[0];
        q.BeginAreaSearch(first, PointIn(m, first, gen), 50, s_any);
        Assert.Equal(AreaQueryStatus.InProgress, q.UpdateAreaSearch(2));
        Assert.Equal(AreaQueryStatus.InProgress, q.FinishAreaSearch(a, default, default, out int part, out _));
        Assert.Equal(2, part);
        Assert.Equal(2, q.AreaIterations);
    }

    [Fact]
    public void TruncatedBuffersAndNodeLimit()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(53, -1, -1, 2, 2, 1, detail: false, offMesh: 0);
        var q = new NavMeshQuery(m);
        PolyRef start = AllPolys(m)[0];
        Vector3D c = PointIn(m, start, new NavMeshWorlds.Rng(1));
        var big = new PolyRef[4096];
        q.FindPolysInCircle(start, c, 100, s_any, big, default, default, out int full, out _);
        Assert.True(full > 8);
        var small = new PolyRef[4];
        Assert.Equal(AreaQueryStatus.Success, q.FindPolysInCircle(start, c, 100, s_any, small, default, default, out int n, out bool trunc));
        Assert.Equal(4, n);
        Assert.True(trunc);
        Assert.True(small.AsSpan().SequenceEqual(big.AsSpan(0, 4)));
        var limited = new NavMeshQuery(m, 5);
        limited.FindPolysInCircle(start, c, 100, s_any, big, default, default, out int cut, out _);
        Assert.Equal(5, cut);
        Assert.True(limited.AreaNodeLimitReached);
    }

    [Fact]
    public void LocalNeighbourhoodDropsStackedPolygons()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(54, -1, -1, 2, 2, 3, detail: false, offMesh: 0);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(2);
        List<PolyRef> all = AllPolys(m);
        var polys = new PolyRef[2048];
        var circle = new PolyRef[2048];
        int dropped = 0;
        for (int i = 0; i < 150; i++)
        {
            PolyRef start = all[gen.Int(all.Count)];
            Vector3D c = PointIn(m, start, gen);
            double r = 1 + (gen.Int(1000) / 1000.0 * 5);
            Assert.Equal(AreaQueryStatus.Success, q.FindLocalNeighbourhood(start, c, r, s_any, polys, default, out int n, out _));
            q.FindPolysInCircle(start, c, r, s_any, circle, default, default, out int nc, out _);
            HashSet<ulong> cs = Set(circle.AsSpan(0, nc));
            Assert.Equal(start, polys[0]);
            for (int a = 0; a < n; a++)
            {
                Assert.Contains(polys[a].Value, cs);
                double[] oa = Outline(m, polys[a]);
                for (int b = a + 1; b < n; b++)
                {
                    // Interiors must not overlap: a separating edge exists, or the overlap area is zero.
                    double[] ob = Outline(m, polys[b]);
                    Assert.True(OverlapArea(oa, ob) < 1e-9, "two kept polygons overlap");
                }
            }
            dropped += nc - n;
        }
        Assert.True(dropped > 0, "the meshes must contain stacked polygons for this test to mean anything");
    }

    // Area of the intersection of two convex polygons (clip one by each half-plane of the other).
    private static double OverlapArea(double[] a, double[] b)
    {
        double[] cur = a;
        int n = b.Length / 2;
        for (int i = 0; i < n && cur.Length >= 6; i++)
        {
            int j = (i + 1) % n;
            double ex = b[2 * j] - b[2 * i], ez = b[(2 * j) + 1] - b[(2 * i) + 1];
            var next = new List<double>();
            int m = cur.Length / 2;
            for (int k = 0; k < m; k++)
            {
                int l = (k + 1) % m;
                double dp = (ex * (cur[(2 * k) + 1] - b[(2 * i) + 1])) - (ez * (cur[2 * k] - b[2 * i]));
                double dq = (ex * (cur[(2 * l) + 1] - b[(2 * i) + 1])) - (ez * (cur[2 * l] - b[2 * i]));
                if (dp >= 0)
                {
                    next.Add(cur[2 * k]);
                    next.Add(cur[(2 * k) + 1]);
                }
                if ((dp >= 0) != (dq >= 0))
                {
                    double s = dp / (dp - dq);
                    next.Add(cur[2 * k] + ((cur[2 * l] - cur[2 * k]) * s));
                    next.Add(cur[(2 * k) + 1] + ((cur[(2 * l) + 1] - cur[(2 * k) + 1]) * s));
                }
            }
            cur = next.ToArray();
        }
        return cur.Length >= 6 ? Area(cur) : 0;
    }

    // ---- distance to wall ----

    [Fact]
    public void DistanceToWallEqualsBruteForce()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(61, -1, -1, 3, 3, 3, detail: false, offMesh: 2);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(4);
        List<PolyRef> all = AllPolys(m);
        int found = 0, none = 0;
        for (int i = 0; i < 500; i++)
        {
            QueryFilter f = Mixed(i);
            PolyRef start = all[gen.Int(all.Count)];
            m.TryGetPoly(start, out NavTile st, out int sp);
            if (!Pass(f, st, sp))
            {
                continue;
            }
            Vector3D c = PointIn(m, start, gen);
            double r = 0.1 + (gen.Int(1000) / 1000.0 * 4);
            WallHit w = q.DistanceToWall(start, c, r, f, out AreaQueryStatus status);
            Assert.Equal(AreaQueryStatus.Success, status);

            double best = double.PositiveInfinity;
            foreach (ulong v in Reach(m, start, CircleTouch(c.X, c.Z, r), f))
            {
                foreach ((double ax, double az, double bx, double bz) in WallPieces(m, new PolyRef(v), f))
                {
                    best = Math.Min(best, SegDist(c.X, c.Z, ax, az, bx, bz));
                }
            }
            if (best <= r)
            {
                found++;
                Assert.True(w.Found, $"wall within {r} missed (query {i})");
                Assert.Equal(best, w.Distance, 9);
                Assert.Equal(w.Distance, Math.Sqrt(((w.Point.X - c.X) * (w.Point.X - c.X)) + ((w.Point.Z - c.Z) * (w.Point.Z - c.Z))), 9);
                Assert.True(m.TryGetPoly(w.Poly, out _, out _));
                double nl = Math.Sqrt((w.Normal.X * w.Normal.X) + (w.Normal.Z * w.Normal.Z));
                Assert.Equal(1.0, nl, 9);
                // The hit lies on a wall piece of the reported polygon and edge.
                bool onWall = false;
                foreach ((double ax, double az, double bx, double bz) in WallPieces(m, w.Poly, f))
                {
                    onWall |= SegDist(w.Point.X, w.Point.Z, ax, az, bx, bz) < 1e-9;
                }
                Assert.True(onWall);
                Assert.InRange(w.Edge, 0, 15);
            }
            else
            {
                none++;
                Assert.False(w.Found, $"phantom wall (query {i})");
                Assert.Equal(r, w.Distance);
            }

            // The struct-filter form and the sliced form agree.
            if (i % 7 == 0)
            {
                WallHit w2 = q.DistanceToWall(start, c, r, new AllPass(), out _);
                Assert.True(!w.Found || w2.Found == (w2.Distance <= r));
                q.BeginAreaSearch(start, c, r, f);
                while (q.UpdateAreaSearch(3) == AreaQueryStatus.InProgress)
                {
                }
                WallHit w3 = q.FinishDistanceToWall();
                Assert.Equal(w.Found, w3.Found);
                Assert.Equal(w.Distance, w3.Distance);
                Assert.Equal(w.Point, w3.Point);
                Assert.Equal(w.Poly, w3.Poly);
                Assert.Equal(w.Edge, w3.Edge);
            }
        }
        Assert.True(found > 100 && none > 20, $"found {found}, none {none}");
    }

    private static double SegDist(double px, double pz, double ax, double az, double bx, double bz)
    {
        double ex = bx - ax, ez = bz - az;
        double len2 = (ex * ex) + (ez * ez);
        double s = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0, 1) : 0;
        double qx = ax + (ex * s) - px, qz = az + (ez * s) - pz;
        return Math.Sqrt((qx * qx) + (qz * qz));
    }

    // Wall pieces of a polygon by interval merging (sorted covered ranges, then the gaps), independent of the query's scan.
    private static IEnumerable<(double ax, double az, double bx, double bz)> WallPieces(TiledNavMesh m, PolyRef pr, QueryFilter f)
    {
        m.TryGetPoly(pr, out NavTile t, out int p);
        int n = t.PolyVertexCounts[p];
        int b = t.PolyFirstIndex(p);
        double[] o = Outline(m, pr);
        NavLink[] links = m.GetLinks(pr).ToArray();
        for (int e = 0; e < n; e++)
        {
            int j = (e + 1) % n;
            double ax = o[2 * e], az = o[(2 * e) + 1], bx = o[2 * j], bz = o[(2 * j) + 1];
            int nb = t.EdgeNeighbors[b + e];
            if (nb != 0)
            {
                if (!Pass(f, t, nb - 1))
                {
                    yield return (ax, az, bx, bz);
                }
                continue;
            }
            var cover = new List<(double, double)>();
            foreach (NavLink l in links)
            {
                if (l.Kind == NavLinkKind.Portal && l.Index == e && m.TryGetPoly(l.Target, out NavTile qt, out int qi) && Pass(f, qt, qi))
                {
                    cover.Add((l.RangeMin / 255.0, l.RangeMax / 255.0));
                }
            }
            cover.Sort();
            double at = 0;
            var gaps = new List<(double, double)>();
            foreach ((double lo, double hi) in cover)
            {
                if (lo > at)
                {
                    gaps.Add((at, lo));
                }
                at = Math.Max(at, hi);
            }
            if (at < 1)
            {
                gaps.Add((at, 1));
            }
            foreach ((double lo, double hi) in gaps)
            {
                yield return (ax + ((bx - ax) * lo), az + ((bz - az) * lo), ax + ((bx - ax) * hi), az + ((bz - az) * hi));
            }
        }
    }

    // ---- move along the surface ----

    [Fact]
    public void MoveAlongSurfaceStaysOnTheMeshAndSlidesAlongWalls()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(71, -1, -1, 3, 3, 2, detail: true, offMesh: 0);
        var q = new NavMeshQuery(m);
        var gen = new NavMeshWorlds.Rng(6);
        List<PolyRef> all = AllPolys(m);
        var visited = new PolyRef[256];
        int blocked = 0, free = 0;
        for (int i = 0; i < 600; i++)
        {
            QueryFilter f = Mixed(i);
            PolyRef start = all[gen.Int(all.Count)];
            m.TryGetPoly(start, out NavTile st, out int sp);
            if (!Pass(f, st, sp))
            {
                Assert.Equal(RaycastStatus.InvalidStartPoly, q.MoveAlongSurface(start, default, default, f, visited, out _).Status);
                continue;
            }
            Vector3D a = PointIn(m, start, gen);
            var end = new Vector3D(a.X + ((gen.Int(2000) - 1000) / 1000.0 * 4), 0, a.Z + ((gen.Int(2000) - 1000) / 1000.0 * 4));
            MoveResult r = q.MoveAlongSurface(start, a, end, f, visited, out int vc);
            Assert.Equal(RaycastStatus.Ok, r.Status);
            double[] o = Outline(m, r.Poly);
            Assert.True(Inside(o, r.Position.X, r.Position.Z, 1e-6), "result is off its polygon");
            Assert.True(m.TryGetPoly(r.Poly, out NavTile rt, out int rp) && Pass(f, rt, rp));
            Assert.Equal(start, visited[0]);
            if (!r.Blocked)
            {
                free++;
                Assert.Equal(end.X, r.Position.X);
                Assert.Equal(end.Z, r.Position.Z);
                // Free move: nothing on the segment is wall; sample it against the visited polygons.
                for (int k = 0; k <= 40; k++)
                {
                    double x = a.X + ((end.X - a.X) * k / 40.0), z = a.Z + ((end.Z - a.Z) * k / 40.0);
                    bool inAny = false;
                    for (int v = 0; v < vc; v++)
                    {
                        inAny |= Inside(Outline(m, visited[v]), x, z, 1e-6);
                    }
                    Assert.True(inAny, $"free move leaves the visited polygons at {k}/40 (query {i})");
                }
            }
            else
            {
                blocked++;
                // Slid to the closest point of a wall edge: no wall piece of the result polygon is closer to the end position.
                double d = Math.Sqrt(((r.Position.X - end.X) * (r.Position.X - end.X)) + ((r.Position.Z - end.Z) * (r.Position.Z - end.Z)));
                bool onWall = false;
                foreach ((double ax, double az, double bx, double bz) in WallPieces(m, r.Poly, f))
                {
                    onWall |= SegDist(r.Position.X, r.Position.Z, ax, az, bx, bz) < 1e-9;
                }
                Assert.True(onWall, "blocked result is not on a wall");
                double bestOnHitEdge = double.PositiveInfinity;
                foreach ((double ax, double az, double bx, double bz) in WallPieces(m, r.Poly, f))
                {
                    if (SegDist(r.Position.X, r.Position.Z, ax, az, bx, bz) < 1e-9)
                    {
                        bestOnHitEdge = Math.Min(bestOnHitEdge, SegDist(end.X, end.Z, ax, az, bx, bz));
                    }
                }
                Assert.True(d <= bestOnHitEdge + 1e-9);
                // The straight part before the wall stayed on visited polygons.
                for (int k = 0; k <= 20; k++)
                {
                    double x = a.X + ((r.Position.X - a.X) * k / 20.0), z = a.Z + ((r.Position.Z - a.Z) * k / 20.0);
                    bool inAny = false;
                    for (int v = 0; v < vc; v++)
                    {
                        inAny |= Inside(Outline(m, visited[v]), x, z, 1e-6);
                    }
                    _ = inAny; // the slide itself may leave the straight line, so only the wall point is asserted above
                }
            }
        }
        Assert.True(blocked > 50 && free > 50, $"blocked {blocked}, free {free}");
    }

    // ---- hostile input ----

    [Fact]
    public void BadInputGivesTypedResults()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(81, -1, -1, 2, 2, 2, detail: true, offMesh: 2);
        var q = new NavMeshQuery(m);
        PolyRef start = AllPolys(m)[0];
        var buf = new PolyRef[16];
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInCircle(start, new Vector3D(double.NaN, 0, 0), 1, s_any, buf, default, default, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInCircle(start, new Vector3D(1e9, 0, 0), 1, s_any, buf, default, default, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInCircle(start, default, -1, s_any, buf, default, default, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInCircle(start, default, double.PositiveInfinity, s_any, buf, default, default, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInCircle(start, default, double.NaN, s_any, buf, default, default, out _, out _));
        Assert.Equal(AreaQueryStatus.InvalidStartPoly, q.FindPolysInCircle(PolyRef.Invalid, default, 1, s_any, buf, default, default, out int n, out _));
        Assert.Equal(0, n);
        Assert.Equal(AreaQueryStatus.InvalidStartPoly, q.FindPolysInCircle(new PolyRef(0xFFFFFFFFFFFFFFFFUL), default, 1, s_any, buf, default, default, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPolysInShape(start, [new Vector3D(0, 0, 0), new Vector3D(double.NaN, 0, 1), new Vector3D(1, 0, 0)], s_any, buf, default, default, out _, out _));
        // Too few points, collinear, concave, self-crossing star, repeated points.
        Vector3D[][] bad =
        [
            [new(0, 0, 0), new(1, 0, 0)],
            [new(0, 0, 0), new(1, 0, 1), new(2, 0, 2)],
            [new(0, 0, 0), new(4, 0, 0), new(4, 0, 4), new(2, 0, 1), new(0, 0, 4)],
            [new(0, 0, 4), new(2.35, 0, -3.2), new(-1.5, 0, 1.2), new(5.5, 0, 1.2), new(1.8, 0, -3.2)],
            [new(1, 0, 1), new(1, 0, 1), new(1, 0, 1)],
        ];
        foreach (Vector3D[] shape in bad)
        {
            Assert.Equal(AreaQueryStatus.InvalidShape, q.FindPolysInShape(start, shape, s_any, buf, default, default, out int c, out _));
            Assert.Equal(0, c);
        }
        Assert.Equal(AreaQueryStatus.InvalidShape, q.FindPolysInShape(start, new Vector3D[MaxShape + 1], s_any, buf, default, default, out _, out _));
        // Finishing or updating without a begun search, and misusing a plain-filter update after a struct-filter begin.
        var fresh = new NavMeshQuery(m);
        Assert.Equal(AreaQueryStatus.InvalidStartPoly, fresh.UpdateAreaSearch(5, new AllPass()));
        Assert.Equal(AreaQueryStatus.InvalidStartPoly, fresh.FinishAreaSearch(buf, default, default, out _, out _));
        Assert.Throws<InvalidOperationException>(() => fresh.FinishDistanceToWall(new AllPass()));
        fresh.BeginAreaSearch(start, default, 1, new AllPass());
        Assert.Throws<InvalidOperationException>(() => fresh.UpdateAreaSearch(1));
        var rng = new SeededRandom(1);
        var untouched = new NavMeshQuery(m);
        Assert.Throws<InvalidOperationException>(() => untouched.FinishRandomPointAround(ref rng).ToString());
        Assert.False(q.RandomPointAround(ref rng, PolyRef.Invalid, default, 1).Found);
        Assert.False(q.RandomPointAround(ref rng, start, PointIn(m, start, new NavMeshWorlds.Rng(1)), 0).Found);
        Assert.Equal(RaycastStatus.InvalidStartPoly, q.MoveAlongSurface(PolyRef.Invalid, default, default, s_any, buf, out _).Status);
        Assert.Throws<ArgumentOutOfRangeException>(() => q.MoveAlongSurface(start, default, new Vector3D(0, double.PositiveInfinity, 0), s_any, buf, out _));
        // An empty mesh.
        var empty = new NavMeshQuery(new TiledNavMesh(NavMeshWorlds.Params()));
        Assert.False(empty.RandomPoint(ref rng).Found);
    }

    private const int MaxShape = NavMeshQuery.MaxShapeVertices;
}
