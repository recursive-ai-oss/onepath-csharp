using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class NavMeshQueryTests
{
    private struct AllPass : IQueryFilter
    {
        public readonly bool Passes(ushort flags, byte area) => true;

        public readonly double AreaCost(byte area) => 1.0;

        public readonly double MinAreaCost => 1.0;
    }

    private static QueryFilter MixedFilter(NavMeshWorlds.Rng r)
    {
        var f = new QueryFilter { ExcludeFlags = r.Int(3) == 0 ? (ushort)4 : (ushort)0 };
        f.SetAreaCost(1, 1.0 + r.Int(4));
        f.SetAreaCost(2, 0.5 + (0.5 * r.Int(3)));
        if (r.Int(4) == 0)
        {
            f.SetAreaCost(3, double.PositiveInfinity);
        }
        return f;
    }

    private static List<PolyRef> AllPolys(TiledNavMesh m)
    {
        var l = new List<PolyRef>();
        for (int s = 0; s < m.SlotCount; s++)
        {
            NavTile? t = m.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                l.Add(m.GetSlotRef(s).Poly(p));
            }
        }
        return l;
    }

    private static Vector3D RandomPointIn(TiledNavMesh m, PolyRef pr, NavMeshWorlds.Rng r)
    {
        m.TryGetPoly(pr, out NavTile t, out int pi);
        int n = t.PolyVertexCounts[pi];
        int b = t.PolyFirstIndex(pi);
        var w = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            w[i] = 1 + r.Int(1000);
            sum += w[i];
        }
        double x = 0, y = 0, z = 0;
        for (int i = 0; i < n; i++)
        {
            int v = 3 * t.PolyIndices[b + i];
            x += w[i] / sum * t.Vertices[v];
            y += w[i] / sum * t.Vertices[v + 1];
            z += w[i] / sum * t.Vertices[v + 2];
        }
        return new Vector3D(t.OriginX + x, t.OriginY + y, t.OriginZ + z);
    }

    // ---- oracle: Dijkstra over crossing states, written from the documented cost model ----

    private sealed class State
    {
        public PolyRef Arrival;
        public Vector3D Pos;
        public double G = double.PositiveInfinity;
        public bool Done;
    }

    private static double OracleCost(TiledNavMesh m, PolyRef sp, PolyRef ep, Vector3D s, Vector3D e, QueryFilter f)
    {
        var states = new List<State> { new() { Arrival = sp, Pos = s, G = 0 } };
        var index = new Dictionary<(ulong, int), State>();
        double best = double.PositiveInfinity;
        while (true)
        {
            State? cur = null;
            foreach (State st in states)
            {
                if (!st.Done && (cur is null || st.G < cur.G))
                {
                    cur = st;
                }
            }
            if (cur is null || cur.G >= best)
            {
                break;
            }
            cur.Done = true;
            m.TryGetPoly(cur.Arrival, out NavTile t, out int pi);
            double c = f.GetAreaCost(t.PolyAreas[pi]);
            if (cur.Arrival == ep)
            {
                best = Math.Min(best, cur.G + (Vector3D.Distance(cur.Pos, e) * c));
            }
            int n = t.PolyVertexCounts[pi];
            int b = t.PolyFirstIndex(pi);
            void Go(ulong from, int exit, PolyRef to, Vector3D pos, double cost)
            {
                if (!m.TryGetPoly(to, out NavTile qt, out int qi) || !Pass(f, qt.PolyFlags[qi], qt.PolyAreas[qi]))
                {
                    return;
                }
                if (!index.TryGetValue((from, exit), out State? st))
                {
                    st = new State { Arrival = to, Pos = pos };
                    index[(from, exit)] = st;
                    states.Add(st);
                }
                if (!st.Done && cur.G + cost < st.G)
                {
                    st.G = cur.G + cost;
                }
            }
            for (int k = 0; k < n; k++)
            {
                int nb = t.EdgeNeighbors[b + k];
                if (nb == 0)
                {
                    continue;
                }
                int a = 3 * t.PolyIndices[b + k], d = 3 * t.PolyIndices[b + ((k + 1) % n)];
                var mid = new Vector3D(t.OriginX + (((double)t.Vertices[a] + t.Vertices[d]) * 0.5), t.OriginY + (((double)t.Vertices[a + 1] + t.Vertices[d + 1]) * 0.5), t.OriginZ + (((double)t.Vertices[a + 2] + t.Vertices[d + 2]) * 0.5));
                Go(cur.Arrival.Value, k, cur.Arrival.Tile.Poly(nb - 1), mid, Vector3D.Distance(cur.Pos, mid) * c);
            }
            var links = m.GetLinks(cur.Arrival);
            for (int j = 0; j < links.Length; j++)
            {
                NavLink l = links[j];
                if (l.Kind == NavLinkKind.Portal)
                {
                    int a = 3 * t.PolyIndices[b + l.Index], d = 3 * t.PolyIndices[b + ((l.Index + 1) % n)];
                    double u = (l.RangeMin + l.RangeMax) * 0.5 / 255.0;
                    var mid = new Vector3D(t.OriginX + t.Vertices[a] + ((t.Vertices[d] - (double)t.Vertices[a]) * u), t.OriginY + t.Vertices[a + 1] + ((t.Vertices[d + 1] - (double)t.Vertices[a + 1]) * u), t.OriginZ + t.Vertices[a + 2] + ((t.Vertices[d + 2] - (double)t.Vertices[a + 2]) * u));
                    Go(cur.Arrival.Value, n + j, l.Target, mid, Vector3D.Distance(cur.Pos, mid) * c);
                }
                else
                {
                    NavTile o = m.GetTileBySlot(l.OwnerSlot)!;
                    OffMeshLink ol = o.OffMeshLinks[l.Index];
                    if (!Pass(f, ol.PolyFlags, ol.Area))
                    {
                        continue;
                    }
                    bool fw = l.Kind == NavLinkKind.OffMesh;
                    var p0 = new Vector3D(o.OriginX + (fw ? ol.StartX : ol.EndX), o.OriginY + (fw ? ol.StartY : ol.EndY), o.OriginZ + (fw ? ol.StartZ : ol.EndZ));
                    var p1 = new Vector3D(o.OriginX + (fw ? ol.EndX : ol.StartX), o.OriginY + (fw ? ol.EndY : ol.StartY), o.OriginZ + (fw ? ol.EndZ : ol.StartZ));
                    Go(cur.Arrival.Value, n + j, l.Target, p1, (Vector3D.Distance(cur.Pos, p0) * c) + (Vector3D.Distance(p0, p1) * f.GetAreaCost(ol.Area)));
                }
            }
        }
        return best;
    }

    private static bool Pass(QueryFilter f, ushort flags, byte area) =>
        (f.IncludeFlags == 0xFFFF || (flags & f.IncludeFlags) != 0) && (flags & f.ExcludeFlags) == 0 && !double.IsPositiveInfinity(f.GetAreaCost(area));

    [Fact]
    public void PathCostEqualsDijkstraOnTheCrossingGraph()
    {
        int checkedPairs = 0, partial = 0;
        for (ulong seed = 1; seed <= 12; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, 0, 0, 2, 2, 1 + (int)(seed % 3), detail: false, offMesh: (int)(seed % 3));
            var q = new NavMeshQuery(m, 65536, PathNodeModel.Crossing);
            var r = new NavMeshWorlds.Rng(seed * 31);
            List<PolyRef> polys = AllPolys(m);
            var corridor = new PolyRef[2048];
            for (int k = 0; k < 12; k++)
            {
                QueryFilter f = MixedFilter(r);
                PolyRef a = polys[r.Int(polys.Count)], b = polys[r.Int(polys.Count)];
                m.TryGetPoly(a, out NavTile ta, out int ia);
                m.TryGetPoly(b, out NavTile tb, out int ib);
                if (!Pass(f, ta.PolyFlags[ia], ta.PolyAreas[ia]) || !Pass(f, tb.PolyFlags[ib], tb.PolyAreas[ib]))
                {
                    continue;
                }
                Vector3D pa = RandomPointIn(m, a, r), pb = RandomPointIn(m, b, r);
                QueryStatus st = q.BeginPath(a, b, pa, pb, f);
                Assert.Equal(QueryStatus.InProgress, st);
                st = q.UpdatePath(int.MaxValue);
                QueryStatus fin = q.FinishPath(corridor, out int cnt, out _, out double cost);
                Assert.Equal(st, fin);
                double oracle = OracleCost(m, a, b, pa, pb, f);
                if (double.IsPositiveInfinity(oracle))
                {
                    Assert.Equal(QueryStatus.Partial, st);
                    partial++;
                }
                else
                {
                    Assert.Equal(QueryStatus.Success, st);
                    Assert.True(Math.Abs(cost - oracle) <= 1e-9 * Math.Max(1, oracle), $"seed {seed} pair {k}: {cost} vs {oracle}");
                    Assert.Equal(a, corridor[0]);
                    Assert.Equal(b, corridor[cnt - 1]);
                    checkedPairs++;
                }
            }
        }
        Assert.True(checkedPairs > 40, $"only {checkedPairs} reachable pairs");
        Assert.True(partial >= 0);
    }

    [Fact]
    public void CorridorsAreConnectedAndLoopFree()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(5, 0, 0, 3, 2, 2, false, 2);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(1);
        List<PolyRef> polys = AllPolys(m);
        for (int k = 0; k < 60; k++)
        {
            PolyRef a = polys[r.Int(polys.Count)], b = polys[r.Int(polys.Count)];
            var corridor = new PolyRef[4096];
            QueryStatus st = q.FindPath(a, b, RandomPointIn(m, a, r), RandomPointIn(m, b, r), new AllPass(), corridor, out int cnt);
            Assert.True(st is QueryStatus.Success or QueryStatus.Partial);
            var seen = new HashSet<ulong>();
            for (int i = 0; i < cnt; i++)
            {
                Assert.True(seen.Add(corridor[i].Value), "polygon repeated in corridor");
            }
            Assert.Equal(QueryStatus.Success, q.FindStraightPath(RandomPointIn(m, corridor[0], r), RandomPointIn(m, corridor[cnt - 1], r), corridor.AsSpan(0, cnt), new StraightPathPoint[4096], out _, out _));
        }
    }

    // ---- straight path against a visibility oracle ----

    private static bool Segs(double ax, double az, double bx, double bz, double cx, double cz, double dx, double dz)
    {
        const double eps = 1e-9;
        static double O(double px, double pz, double qx, double qz, double rx, double rz) => ((qx - px) * (rz - pz)) - ((qz - pz) * (rx - px));
        double o1 = O(ax, az, bx, bz, cx, cz), o2 = O(ax, az, bx, bz, dx, dz), o3 = O(cx, cz, dx, dz, ax, az), o4 = O(cx, cz, dx, dz, bx, bz);
        double l1 = Math.Sqrt(((bx - ax) * (bx - ax)) + ((bz - az) * (bz - az))) + 1e-300, l2 = Math.Sqrt(((dx - cx) * (dx - cx)) + ((dz - cz) * (dz - cz))) + 1e-300;
        bool side1 = !((o1 / l1 > eps && o2 / l1 > eps) || (o1 / l1 < -eps && o2 / l1 < -eps));
        bool side2 = !((o3 / l2 > eps && o4 / l2 > eps) || (o3 / l2 < -eps && o4 / l2 < -eps));
        return side1 && side2;
    }

    private static (double lx, double lz, double rx, double rz)? OraclePortal(NavTile ta, int pa, NavTile tb, int pb)
    {
        double hl = double.PositiveInfinity, hh = double.NegativeInfinity;
        double bx0 = 0, bz0 = 0, bx1 = 0, bz1 = 0;
        int na = ta.PolyVertexCounts[pa], nb = tb.PolyVertexCounts[pb];
        for (int i = 0; i < na; i++)
        {
            int a0 = 3 * ta.PolyIndices[ta.PolyFirstIndex(pa) + i], a1 = 3 * ta.PolyIndices[ta.PolyFirstIndex(pa) + ((i + 1) % na)];
            double ax = ta.OriginX + ta.Vertices[a0], az = ta.OriginZ + ta.Vertices[a0 + 2], cx = ta.OriginX + ta.Vertices[a1], cz = ta.OriginZ + ta.Vertices[a1 + 2];
            double ex = cx - ax, ez = cz - az, len2 = (ex * ex) + (ez * ez);
            for (int j = 0; j < nb; j++)
            {
                int b0 = 3 * tb.PolyIndices[tb.PolyFirstIndex(pb) + j], b1 = 3 * tb.PolyIndices[tb.PolyFirstIndex(pb) + ((j + 1) % nb)];
                double px = tb.OriginX + tb.Vertices[b0], pz = tb.OriginZ + tb.Vertices[b0 + 2], qx = tb.OriginX + tb.Vertices[b1], qz = tb.OriginZ + tb.Vertices[b1 + 2];
                // Collinear with the A edge?
                if (Math.Abs((ex * (pz - az)) - (ez * (px - ax))) > 1e-9 || Math.Abs((ex * (qz - az)) - (ez * (qx - ax))) > 1e-9)
                {
                    continue;
                }
                double su0 = ((px - ax) * ex + (pz - az) * ez) / len2, su1 = ((qx - ax) * ex + (qz - az) * ez) / len2;
                double lo = Math.Max(0, Math.Min(su0, su1)), hi = Math.Min(1, Math.Max(su0, su1));
                if (hi > lo + 1e-12)
                {
                    if (lo < hl)
                    {
                        hl = lo;
                        bx0 = ax + (ex * lo);
                        bz0 = az + (ez * lo);
                    }
                    if (hi > hh)
                    {
                        hh = hi;
                        bx1 = ax + (ex * hi);
                        bz1 = az + (ez * hi);
                    }
                }
            }
        }
        return hh > hl ? (bx0, bz0, bx1, bz1) : null;
    }

    [Fact]
    public void StraightPathIsTheShortestPathInsideTheCorridor()
    {
        int checkedPaths = 0;
        int multi = 0;
        for (ulong seed = 1; seed <= 8; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, 0, 0, 2, 2, 1, detail: seed % 2 == 0, offMesh: 0);
            var q = new NavMeshQuery(m);
            var r = new NavMeshWorlds.Rng(seed + 100);
            List<PolyRef> polys = AllPolys(m);
            for (int k = 0; k < 25; k++)
            {
                PolyRef a = polys[r.Int(polys.Count)], b = polys[r.Int(polys.Count)];
                Vector3D pa = RandomPointIn(m, a, r), pb = RandomPointIn(m, b, r);
                var corridor = new PolyRef[4096];
                QueryStatus st = q.FindPath(a, b, pa, pb, new AllPass(), corridor, out int cnt);
                if (st != QueryStatus.Success)
                {
                    continue;
                }
                var pts = new StraightPathPoint[4096];
                Assert.Equal(QueryStatus.Success, q.FindStraightPath(pa, pb, corridor.AsSpan(0, cnt), pts, out int pc, out _));
                // The result starts and ends at the (snapped) endpoints, and every point lies on its portal.
                Assert.True(pts[0].Flags.HasFlag(StraightPathMarks.Start));
                Assert.True(pts[pc - 1].Flags.HasFlag(StraightPathMarks.End));
                double ours = 0;
                for (int i = 1; i < pc; i++)
                {
                    ours += Math.Sqrt(Math.Pow(pts[i].Position.X - pts[i - 1].Position.X, 2) + Math.Pow(pts[i].Position.Z - pts[i - 1].Position.Z, 2));
                }
                // Oracle: shortest path through the portal sequence on the visibility graph.
                var portals = new List<(double lx, double lz, double rx, double rz)>();
                bool ok = true;
                for (int i = 0; i + 1 < cnt && ok; i++)
                {
                    m.TryGetPoly(corridor[i], out NavTile ta, out int ia);
                    m.TryGetPoly(corridor[i + 1], out NavTile tb, out int ib);
                    var pt = OraclePortal(ta, ia, tb, ib);
                    if (pt is null)
                    {
                        ok = false;
                    }
                    else
                    {
                        portals.Add(pt.Value);
                    }
                }
                Assert.True(ok);
                double oracle = VisibilityLength(pts[0].Position, pts[pc - 1].Position, portals);
                Assert.True(Math.Abs(ours - oracle) < 1e-6, $"seed {seed} k {k}: {ours} vs {oracle}");
                checkedPaths++;
                if (cnt > 2)
                {
                    multi++;
                }
            }
        }
        Assert.True(checkedPaths > 60 && multi > 20, $"{checkedPaths} {multi}");
    }

    private static double VisibilityLength(Vector3D s, Vector3D e, List<(double lx, double lz, double rx, double rz)> portals)
    {
        int n = portals.Count;
        // nodes: 0 = start (portal index -1), then 2 per portal, then end (index n)
        var nx = new List<double> { s.X };
        var nz = new List<double> { s.Z };
        var idx = new List<int> { -1 };
        for (int i = 0; i < n; i++)
        {
            nx.Add(portals[i].lx);
            nz.Add(portals[i].lz);
            idx.Add(i);
            nx.Add(portals[i].rx);
            nz.Add(portals[i].rz);
            idx.Add(i);
        }
        nx.Add(e.X);
        nz.Add(e.Z);
        idx.Add(n);
        var dp = new double[nx.Count];
        Array.Fill(dp, double.PositiveInfinity);
        dp[0] = 0;
        for (int v = 1; v < nx.Count; v++)
        {
            for (int u = 0; u < v; u++)
            {
                if (idx[u] >= idx[v] || double.IsPositiveInfinity(dp[u]))
                {
                    continue;
                }
                bool vis = true;
                for (int k = idx[u] + 1; k < idx[v] && vis; k++)
                {
                    vis = Segs(nx[u], nz[u], nx[v], nz[v], portals[k].lx, portals[k].lz, portals[k].rx, portals[k].rz);
                }
                if (vis)
                {
                    dp[v] = Math.Min(dp[v], dp[u] + Math.Sqrt(Math.Pow(nx[v] - nx[u], 2) + Math.Pow(nz[v] - nz[u], 2)));
                }
            }
        }
        return dp[^1];
    }

    // ---- raycast against a brute-force segment/boundary intersection ----

    [Fact]
    public void RaycastMatchesBruteForceWallIntersection()
    {
        int hits = 0, misses = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, 0, 0, 2, 2, 1, detail: false, offMesh: 0);
            var q = new NavMeshQuery(m);
            var r = new NavMeshWorlds.Rng(seed * 7);
            List<PolyRef> polys = AllPolys(m);
            var walls = new List<(double ax, double az, double bx, double bz)>();
            foreach (PolyRef pr in polys)
            {
                m.TryGetPoly(pr, out NavTile t, out int pi);
                int n = t.PolyVertexCounts[pi], b = t.PolyFirstIndex(pi);
                var links = m.GetLinks(pr);
                for (int e = 0; e < n; e++)
                {
                    bool open = t.EdgeNeighbors[b + e] != 0;
                    foreach (NavLink l in links)
                    {
                        open |= l.Kind == NavLinkKind.Portal && l.Index == e;
                    }
                    if (!open)
                    {
                        int a = 3 * t.PolyIndices[b + e], c = 3 * t.PolyIndices[b + ((e + 1) % n)];
                        walls.Add((t.OriginX + t.Vertices[a], t.OriginZ + t.Vertices[a + 2], t.OriginX + t.Vertices[c], t.OriginZ + t.Vertices[c + 2]));
                    }
                }
            }
            var visited = new PolyRef[512];
            for (int k = 0; k < 80; k++)
            {
                PolyRef a = polys[r.Int(polys.Count)];
                Vector3D s = RandomPointIn(m, a, r);
                // Non-grid-aligned end so the ray never runs along edges or through vertices.
                var e = new Vector3D(-16 + (r.Int(100000) / 100000.0 * 16.0) + 0.0001234, 0, -16 + (r.Int(100000) / 100000.0 * 16.0) + 0.0000777);
                RaycastResult res = q.Raycast(a, s, e, new AllPass(), visited, out int vc);
                Assert.Equal(RaycastStatus.Ok, res.Status);
                double bt = 1.0;
                double dx = e.X - s.X, dz = e.Z - s.Z;
                foreach (var w in walls)
                {
                    double ex = w.bx - w.ax, ez = w.bz - w.az;
                    double den = (dx * ez) - (dz * ex);
                    if (den == 0)
                    {
                        continue;
                    }
                    double t = (((w.ax - s.X) * ez) - ((w.az - s.Z) * ex)) / den;
                    double u = (((w.ax - s.X) * dz) - ((w.az - s.Z) * dx)) / den;
                    if (t >= 0 && t < bt && u >= 0 && u <= 1)
                    {
                        bt = t;
                    }
                }
                if (bt < 1.0)
                {
                    Assert.True(res.Hit, $"seed {seed} k {k}: brute hit at {bt}");
                    Assert.True(Math.Abs(res.T - bt) < 1e-9, $"{res.T} vs {bt}");
                    Assert.Equal(visited[vc - 1], res.HitPoly);
                    // The normal points against the ray.
                    Assert.True((res.Normal.X * dx) + (res.Normal.Z * dz) > 0);
                    hits++;
                }
                else
                {
                    Assert.False(res.Hit, $"seed {seed} k {k}");
                    Assert.Equal(1.0, res.T);
                    misses++;
                }
            }
        }
        Assert.True(hits > 40 && misses > 5, $"{hits} {misses}");
    }

    // ---- nearest against brute force ----

    private static (double d2, double cx, double cy, double cz) BruteClosest(NavTile t, int p, Vector3D c)
    {
        int n = t.PolyVertexCounts[p], b = t.PolyFirstIndex(p);
        double px = c.X - t.OriginX, py = c.Y - t.OriginY, pz = c.Z - t.OriginZ;
        double X(int i) => t.Vertices[3 * t.PolyIndices[b + i]];
        double Y(int i) => t.Vertices[(3 * t.PolyIndices[b + i]) + 1];
        double Z(int i) => t.Vertices[(3 * t.PolyIndices[b + i]) + 2];
        // inside: fan height
        bool inside = true;
        for (int e = 0; e < n && inside; e++)
        {
            int f = (e + 1) % n;
            inside = ((X(f) - X(e)) * (pz - Z(e))) - ((Z(f) - Z(e)) * (px - X(e))) >= 0;
        }
        double cx, cy, cz;
        if (inside)
        {
            cx = px;
            cz = pz;
            cy = double.NaN;
            double bestMin = double.NegativeInfinity;
            for (int i = 1; i + 1 < n; i++)
            {
                double d = ((Z(i) - Z(i + 1)) * (X(0) - X(i + 1))) + ((X(i + 1) - X(i)) * (Z(0) - Z(i + 1)));
                if (d == 0)
                {
                    continue;
                }
                double u = (((Z(i) - Z(i + 1)) * (px - X(i + 1))) + ((X(i + 1) - X(i)) * (pz - Z(i + 1)))) / d;
                double w = (((Z(i + 1) - Z(0)) * (px - X(i + 1))) + ((X(0) - X(i + 1)) * (pz - Z(i + 1)))) / d;
                double qq = 1 - u - w;
                double mn = Math.Min(u, Math.Min(w, qq));
                if (mn > bestMin)
                {
                    bestMin = mn;
                    cy = (u * Y(0)) + (w * Y(i)) + (qq * Y(i + 1));
                }
                if (mn >= -1e-9)
                {
                    break;
                }
            }
        }
        else
        {
            double best = double.PositiveInfinity;
            cx = cy = cz = 0;
            for (int e = 0; e < n; e++)
            {
                int f = (e + 1) % n;
                double ex = X(f) - X(e), ez = Z(f) - Z(e);
                double s = Math.Clamp((((px - X(e)) * ex) + ((pz - Z(e)) * ez)) / ((ex * ex) + (ez * ez)), 0, 1);
                double qx = X(e) + (ex * s), qz = Z(e) + (ez * s);
                double d2 = ((px - qx) * (px - qx)) + ((pz - qz) * (pz - qz));
                if (d2 < best)
                {
                    best = d2;
                    cx = qx;
                    cz = qz;
                    cy = Y(e) + ((Y(f) - Y(e)) * s);
                }
            }
        }
        return (((cx - px) * (cx - px)) + ((cy - py) * (cy - py)) + ((cz - pz) * (cz - pz)), cx, cy, cz);
    }

    [Fact]
    public void NearestMatchesBruteForceOnMultiLayerMeshes()
    {
        int found = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, -1, -1, 3, 3, 3, detail: false, offMesh: 0);
            var q = new NavMeshQuery(m);
            var r = new NavMeshWorlds.Rng(seed);
            List<PolyRef> polys = AllPolys(m);
            for (int k = 0; k < 100; k++)
            {
                var c = new Vector3D(-24 + (r.Int(100000) / 100000.0 * 24), r.Int(100000) / 100000.0 * 6 - 0.5, -24 + (r.Int(100000) / 100000.0 * 24));
                var ext = new Vector3D(0.5 + r.Int(3), 0.5 + r.Int(4), 0.5 + r.Int(3));
                var f = new QueryFilter { ExcludeFlags = r.Int(2) == 0 ? (ushort)2 : (ushort)0 };
                NearestPoly got = q.FindNearestPoly(c, ext, f);
                double bestD = double.PositiveInfinity;
                PolyRef bestRef = PolyRef.Invalid;
                (int, int, int, int) bestKey = default;
                foreach (PolyRef pr in polys)
                {
                    m.TryGetPoly(pr, out NavTile t, out int pi);
                    if (!Pass(f, t.PolyFlags[pi], t.PolyAreas[pi]))
                    {
                        continue;
                    }
                    // Polygon box overlap with the query box.
                    int n = t.PolyVertexCounts[pi], b = t.PolyFirstIndex(pi);
                    double x0 = double.MaxValue, y0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, y1 = double.MinValue, z1 = double.MinValue;
                    for (int i = 0; i < n; i++)
                    {
                        int v = 3 * t.PolyIndices[b + i];
                        x0 = Math.Min(x0, t.OriginX + t.Vertices[v]);
                        x1 = Math.Max(x1, t.OriginX + t.Vertices[v]);
                        y0 = Math.Min(y0, t.OriginY + t.Vertices[v + 1]);
                        y1 = Math.Max(y1, t.OriginY + t.Vertices[v + 1]);
                        z0 = Math.Min(z0, t.OriginZ + t.Vertices[v + 2]);
                        z1 = Math.Max(z1, t.OriginZ + t.Vertices[v + 2]);
                    }
                    if (x0 > c.X + ext.X || x1 < c.X - ext.X || y0 > c.Y + ext.Y || y1 < c.Y - ext.Y || z0 > c.Z + ext.Z || z1 < c.Z - ext.Z)
                    {
                        continue;
                    }
                    var (d2, _, _, _) = BruteClosest(t, pi, c);
                    var key = (t.TileX, t.TileZ, t.Layer, pi);
                    if (d2 < bestD || (d2 == bestD && key.CompareTo(bestKey) < 0))
                    {
                        bestD = d2;
                        bestRef = pr;
                        bestKey = key;
                    }
                }
                if (!bestRef.IsSet)
                {
                    Assert.False(got.Poly.IsSet);
                    continue;
                }
                found++;
                Assert.Equal(bestRef, got.Poly);
                Assert.True(Math.Abs(got.DistanceSquared - bestD) < 1e-9, $"{got.DistanceSquared} vs {bestD}");
            }
        }
        Assert.True(found > 200);
    }

    [Fact]
    public void NearestPicksTheLayerWhoseFloorIsVerticallyClosest()
    {
        // Layer 0 floor is at about 0-0.4, layer 1 at about 4-4.4 (same footprint where both exist).
        TiledNavMesh m = NavMeshWorlds.RandomMesh(3, 0, 0, 1, 1, 2, false, 0);
        var q = new NavMeshQuery(m);
        int checkedPoints = 0;
        foreach (PolyRef pr in AllPolys(m))
        {
            m.TryGetPoly(pr, out NavTile t, out int pi);
            if (t.Layer != 1)
            {
                continue;
            }
            Vector3D p = RandomPointIn(m, pr, new NavMeshWorlds.Rng((ulong)pi));
            NearestPoly n = q.FindNearestPoly(new Vector3D(p.X, p.Y + 0.3, p.Z), new Vector3D(0.1, 3, 0.1));
            if (n.Poly.IsSet && n.OverPolygon)
            {
                m.TryGetPoly(n.Poly, out NavTile nt, out _);
                Assert.True(Math.Abs(n.Point.Y - (p.Y + 0.3)) < 2.5, "chose a floor more than half a layer gap away");
                Assert.Equal(1, nt.Layer);
                checkedPoints++;
            }
        }
        Assert.True(checkedPoints > 5);
    }

    // ---- sliced, async ----

    [Fact]
    public void SlicedAndAsyncResultsEqualTheSyncResult()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(8, 0, 0, 3, 3, 2, false, 2);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(17);
        List<PolyRef> polys = AllPolys(m);
        for (int k = 0; k < 25; k++)
        {
            QueryFilter f = MixedFilter(r);
            PolyRef a = polys[r.Int(polys.Count)], b = polys[r.Int(polys.Count)];
            Vector3D pa = RandomPointIn(m, a, r), pb = RandomPointIn(m, b, r);
            var want = new PolyRef[4096];
            QueryStatus ws = q.BeginPath(a, b, pa, pb, f);
            if (ws != QueryStatus.InProgress)
            {
                continue;
            }
            q.UpdatePath(int.MaxValue);
            QueryStatus wf = q.FinishPath(want, out int wc, out _, out double wcost);
            int wIter = q.Iterations;
            foreach (int budget in new[] { 1, 2, 3, 7, 50, 1000 })
            {
                q.BeginPath(a, b, pa, pb, f);
                int calls = 0;
                QueryStatus s;
                do
                {
                    s = q.UpdatePath(budget);
                    calls++;
                }
                while (s == QueryStatus.InProgress);
                var got = new PolyRef[4096];
                QueryStatus gf = q.FinishPath(got, out int gc, out _, out double gcost);
                Assert.Equal(wf, gf);
                Assert.Equal(wc, gc);
                Assert.Equal(wcost, gcost);
                Assert.True(want.AsSpan(0, wc).SequenceEqual(got.AsSpan(0, gc)));
                Assert.Equal(wIter, q.Iterations);
                Assert.True(calls >= 1);
            }
        }
    }

    [Fact]
    public async Task AsyncEqualsSyncForTheOnePathCall()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(9, 0, 0, 3, 3, 2, false, 1);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(3);
        for (int k = 0; k < 15; k++)
        {
            var a = new Vector3D(-12 + r.Int(24) + 0.37, 0.2, -12 + r.Int(24) + 0.61);
            var b = new Vector3D(-12 + r.Int(24) + 0.11, 0.2, -12 + r.Int(24) + 0.83);
            var opt = new PathOptions { SearchExtents = new Vector3D(2, 5, 2) };
            PathResult sync = q.FindPath(a, b, opt, cancellationToken: TestContext.Current.CancellationToken);
            foreach (int slice in new[] { 1, 5, 64 })
            {
                PathResult asy = await q.FindPathAsync(a, b, opt, slice, TestContext.Current.CancellationToken);
                Assert.Equal(sync.Status, asy.Status);
                Assert.Equal(sync.Cost, asy.Cost);
                Assert.True(System.Linq.Enumerable.SequenceEqual(sync.Points, asy.Points));
                Assert.True(System.Linq.Enumerable.SequenceEqual(sync.Corridor, asy.Corridor));
            }
        }
        await Assert.ThrowsAnyAsync<OperationCanceledException>(async () =>
        {
            var cts = new System.Threading.CancellationTokenSource();
            cts.Cancel();
            await q.FindPathAsync(new Vector3D(-14.3, 0.2, -14.3), new Vector3D(-1.7, 0.2, -1.7), null, 1, cts.Token);
        });
    }

    // ---- behaviours ----

    [Fact]
    public void FiltersChangeThePathAndUnreachableGivesAPartialPath()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        // A 3-wide corridor of squares along X in tile (0,0); the middle square can be forbidden.
        double ox = p.TileOriginX(0), oz = p.TileOriginZ(0);
        double[] v = [ox, 0, oz, ox + 2, 0, oz, ox + 4, 0, oz, ox + 6, 0, oz, ox, 0, oz + 2, ox + 2, 0, oz + 2, ox + 4, 0, oz + 2, ox + 6, 0, oz + 2];
        NavTileData d = NavTileAssembler.FromPolygons(p, 0, 0, 0, 0, v, [4, 4, 4], [0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6], [0, 1, 0], [1, 2, 1]);
        var m = new TiledNavMesh(p);
        TileRef tr = m.AddTile(NavTile.Create(d, p));
        var q = new NavMeshQuery(m);
        var a = new Vector3D(ox + 1, 0, oz + 1);
        var b = new Vector3D(ox + 5, 0, oz + 1);
        PathResult ok = q.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, ok.Status);
        Assert.Equal(3, ok.Corridor.Count);
        Assert.Equal(4.0, ok.Length, 9);
        var blocked = new PathOptions { Filter = new QueryFilter { ExcludeFlags = 2 } };
        PathResult part = q.FindPath(a, b, blocked, cancellationToken: TestContext.Current.CancellationToken);
        // The end polygon is reachable only through the excluded one: the end is "nearby" but the search stops at the first square.
        Assert.Equal(PathStatus.Partial, part.Status);
        Assert.Single(part.Corridor);
        Assert.Equal(tr.Poly(0), part.Corridor[0]);
        // Costly middle area: cost grows, path same corridor (no alternative).
        var costly = new PathOptions { Filter = new QueryFilter().SetAreaCost(1, 10) };
        PathResult c = q.FindPath(a, b, costly, cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(c.Cost > ok.Cost * 3);
    }

    [Fact]
    public void OffMeshLinksAreTraversedAndMarked()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        NavTileData Square(int tx, bool link)
        {
            double ox = p.TileOriginX(tx), oz = p.TileOriginZ(0);
            double[] v = [ox + 1, 0, oz + 1, ox + 3, 0, oz + 1, ox + 3, 0, oz + 3, ox + 1, 0, oz + 3];
            NavTileData d = NavTileAssembler.FromPolygons(p, tx, 0, 0, 0, v, [4], [0, 1, 2, 3]);
            if (link)
            {
                d.OffMeshLinks = [new OffMeshLink { StartX = 2, StartY = 0, StartZ = 2, EndX = 18, EndY = 0, EndZ = 2, Radius = 0.5f, Bidirectional = true, Area = 9, UserId = 1 }];
            }
            return d;
        }
        var m = new TiledNavMesh(p);
        m.AddTile(NavTile.Create(Square(0, true), p));
        m.AddTile(NavTile.Create(Square(2, false), p));
        var q = new NavMeshQuery(m);
        Vector3D a = new(p.TileOriginX(0) + 1.5, 0, p.TileOriginZ(0) + 1.5), b = new(p.TileOriginX(2) + 2.5, 0, p.TileOriginZ(0) + 2.5);
        PathResult fwd = q.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, fwd.Status);
        Assert.Equal(4, fwd.Points.Count);
        Assert.True(fwd.Points[1].Flags.HasFlag(StraightPathMarks.OffMeshConnection));
        Assert.Equal(p.TileOriginX(0) + 2, fwd.Points[1].Position.X);
        Assert.Equal(p.TileOriginX(0) + 18, fwd.Points[2].Position.X);
        PathResult back = q.FindPath(b, a, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, back.Status);
        Assert.True(back.Points[1].Flags.HasFlag(StraightPathMarks.OffMeshConnection));
        // The forbidden area closes the link.
        PathResult closed = q.FindPath(a, b, new PathOptions { Filter = new QueryFilter().SetAreaCost(9, double.PositiveInfinity) }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Partial, closed.Status);
    }

    [Fact]
    public void HostileInputsGetTypedErrorsOrFailureStatuses()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(2, 0, 0, 2, 2, 1, false, 0);
        var q = new NavMeshQuery(m);
        List<PolyRef> polys = AllPolys(m);
        var r = new NavMeshWorlds.Rng(1);
        Vector3D good = RandomPointIn(m, polys[0], r);
        var buf = new PolyRef[16];
        var pts = new StraightPathPoint[16];
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindNearestPoly(new Vector3D(double.NaN, 0, 0), new Vector3D(1, 1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindNearestPoly(new Vector3D(3e6, 0, 0), new Vector3D(1, 1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindNearestPoly(good, new Vector3D(-1, 1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindNearestPoly(good, new Vector3D(double.PositiveInfinity, 1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindPath(good, new Vector3D(0, double.NaN, 0), cancellationToken: TestContext.Current.CancellationToken));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.BeginPath(polys[0], polys[1], new Vector3D(double.PositiveInfinity, 0, 0), good));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.Raycast(polys[0], good, new Vector3D(0, 0, double.NaN), null, buf, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => q.FindStraightPath(good, new Vector3D(1e9, 0, 0), polys.GetRange(0, 1).ToArray(), pts, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => new NavMeshQuery(m, 1));
        Assert.Equal(QueryStatus.InvalidStartPoly, q.BeginPath(PolyRef.Invalid, polys[0], good, good));
        Assert.Equal(QueryStatus.InvalidEndPoly, q.BeginPath(polys[0], new PolyRef(ulong.MaxValue), good, good));
        Assert.Equal(RaycastStatus.InvalidStartPoly, q.Raycast(new PolyRef(123), good, good, null, buf, out _).Status);
        Assert.Equal(RaycastStatus.StartOutsidePolygon, q.Raycast(polys[0], new Vector3D(good.X + 100, 0, good.Z), good, null, buf, out _).Status);
        Assert.False(q.ClosestPointOnPoly(PolyRef.Invalid, good, out _, out _));
        Assert.Equal(QueryStatus.InvalidCorridor, q.FindStraightPath(good, good, ReadOnlySpan<PolyRef>.Empty, pts, out _, out _));
        // Two polygons that do not touch are not a corridor.
        PolyRef far = polys[^1];
        Assert.Equal(QueryStatus.InvalidCorridor, q.FindStraightPath(good, good, new[] { polys[0], far }, pts, out _, out _));
        // Stale after the tile is removed.
        TileRef t0 = polys[0].Tile;
        m.RemoveTile(t0);
        Assert.Equal(QueryStatus.InvalidStartPoly, q.BeginPath(polys[0], polys[^1], good, good));
        Assert.False(q.FindNearestPoly(good, new Vector3D(0.01, 0.01, 0.01)).Poly == polys[0]);
        Assert.Equal(RaycastStatus.InvalidStartPoly, q.Raycast(polys[0], good, good, null, buf, out _).Status);
        Assert.Throws<InvalidOperationException>(() =>
        {
            q.BeginPath(polys[^1], polys[^1], RandomPointIn(m, polys[^1], r), good, new AllPass());
            q.UpdatePath(1);
        });
        // Tiny buffers truncate rather than fail.
        var tinyPath = new PolyRef[1];
        PolyRef x = polys[^1];
        Assert.True(q.FindPath(x, x, RandomPointIn(m, x, r), RandomPointIn(m, x, r), new AllPass(), tinyPath, out int tc) == QueryStatus.Success && tc == 1);
        Assert.False(q.FindPath(new Vector3D(1e5, 0, 1e5), good, cancellationToken: TestContext.Current.CancellationToken).HasPath);
    }

    [Fact]
    public void SteadyStateQueriesDoNotAllocate()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(4, 0, 0, 2, 2, 2, detail: true, offMesh: 1);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(5);
        List<PolyRef> polys = AllPolys(m);
        var corridor = new PolyRef[2048];
        var pts = new StraightPathPoint[2048];
        var visited = new PolyRef[2048];
        var filter = new AllPass();
        var ps = new Vector3D[16];
        var pp = new PolyRef[16];
        for (int i = 0; i < 16; i++)
        {
            pp[i] = polys[r.Int(polys.Count)];
            ps[i] = RandomPointIn(m, pp[i], r);
        }
        void Round()
        {
            for (int i = 0; i + 1 < 16; i++)
            {
                _ = q.FindNearestPoly(ps[i], new Vector3D(2, 4, 2), in filter);
                if (q.FindPath(pp[i], pp[i + 1], ps[i], ps[i + 1], in filter, corridor, out int n) != QueryStatus.Success)
                {
                    continue;
                }
                q.FindStraightPath(ps[i], ps[i + 1], corridor.AsSpan(0, n), pts, out _, out _);
                _ = q.Raycast(pp[i], ps[i], ps[i + 1], in filter, visited, out _);
            }
        }
        for (int i = 0; i < 5; i++)
        {
            Round();
        }
        long allocated = AllocationProbe.SteadyState(() =>
        {
            for (int i = 0; i < 20; i++)
            {
                Round();
            }
        });
        Assert.Equal(0, allocated);
    }

    [Fact]
    public void TierOneFindPathWorksOnTheMesh()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(6, 0, 0, 2, 2, 1, false, 0);
        PathResult res = m.FindPath(new Vector3D(-15.5, 0.2, -15.5), new Vector3D(-1.5, 0.2, -1.5), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(res.HasPath);
        Assert.True(res.Points.Count >= 2);
        Assert.True(res.Length > 0);
    }
}
