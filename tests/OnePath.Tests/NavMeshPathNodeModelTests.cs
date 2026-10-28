using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>The one-node-per-polygon search model, and the detail-height lookup grid that keeps straight-path set-up cheap.</summary>
public class NavMeshPathNodeModelTests
{
    private static QueryFilter MixedFilter(NavMeshWorlds.Rng r)
    {
        var f = new QueryFilter { ExcludeFlags = r.Int(3) == 0 ? (ushort)4 : (ushort)0 };
        f = f.WithAreaCost(1, 1.0 + r.Int(4));
        f = f.WithAreaCost(2, 0.5 + (0.5 * r.Int(3)));
        if (r.Int(4) == 0)
        {
            f = f.WithAreaCost(3, double.PositiveInfinity);
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

    private static Vector3D Centre(TiledNavMesh m, PolyRef pr)
    {
        m.TryGetPoly(pr, out NavTile t, out int pi);
        int n = t.PolyVertexCounts[pi];
        int b = t.PolyFirstIndex(pi);
        double x = 0, y = 0, z = 0;
        for (int i = 0; i < n; i++)
        {
            int v = 3 * t.PolyIndices[b + i];
            x += t.Vertices[v];
            y += t.Vertices[v + 1];
            z += t.Vertices[v + 2];
        }
        return new Vector3D(t.OriginX + (x / n), t.OriginY + (y / n), t.OriginZ + (z / n));
    }

    [Fact]
    public void PolygonModelQueryObjectFollowsMeshEdits()
    {
        // One long-lived query object must give what a fresh one gives after tiles are removed and put back.
        TiledNavMesh m = NavMeshWorlds.RandomMesh(5, -2, -2, 4, 4, 2, detail: false, offMesh: 2);
        var reused = new NavMeshQuery(m, 65536, PathNodeModel.Polygon);
        var filter = new QueryFilter();
        List<PolyRef> polys = AllPolys(m);
        var rng = new NavMeshWorlds.Rng(99);
        var pairs = new List<(PolyRef, PolyRef)>();
        for (int i = 0; i < 40; i++)
        {
            pairs.Add((polys[rng.Int(polys.Count)], polys[rng.Int(polys.Count)]));
        }
        QueryStatus Run(NavMeshQuery q, PolyRef s, PolyRef e, PolyRef[] buffer, out int count)
        {
            QueryStatus st = q.BeginPath(s, e, Centre(m, s), Centre(m, e), filter);
            if (st != QueryStatus.InProgress)
            {
                count = 0;
                return st;
            }
            q.UpdatePath(int.MaxValue);
            return q.FinishPath(buffer, out count, out _, out _);
        }

        void Compare()
        {
            var fresh = new NavMeshQuery(m, 65536, PathNodeModel.Polygon);
            var a = new PolyRef[512];
            var b = new PolyRef[512];
            foreach ((PolyRef s, PolyRef e) in pairs)
            {
                if (!m.TryGetPoly(s, out _, out _) || !m.TryGetPoly(e, out _, out _))
                {
                    continue;
                }
                QueryStatus sa = Run(reused, s, e, a, out int na);
                QueryStatus sb = Run(fresh, s, e, b, out int nb);
                Assert.Equal(sb, sa);
                Assert.Equal(nb, na);
                Assert.True(a.AsSpan(0, na).SequenceEqual(b.AsSpan(0, nb)));
            }
        }

        Compare();
        NavTile removed = m.GetTileBySlot(3)!;
        TileRef removedRef = m.GetSlotRef(3);
        Assert.True(m.RemoveTile(removedRef));
        Compare();
        m.AddTile(removed);
        Compare();
    }

    [Fact]
    public void PolygonModelFindsTheSameReachabilityAsCrossingAndReturnsConnectedLoopFreeCorridors()
    {
        int reachable = 0, unreachable = 0;
        for (ulong seed = 1; seed <= 10; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, 0, 0, 3, 2, 1 + (int)(seed % 3), detail: false, offMesh: (int)(seed % 3));
            var cross = new NavMeshQuery(m, 65536, PathNodeModel.Crossing);
            var poly = new NavMeshQuery(m, 65536, PathNodeModel.Polygon);
            Assert.Equal(PathNodeModel.Polygon, poly.NodeModel);
            var r = new NavMeshWorlds.Rng(seed * 7);
            List<PolyRef> all = AllPolys(m);
            var c1 = new PolyRef[4096];
            var c2 = new PolyRef[4096];
            for (int k = 0; k < 30; k++)
            {
                QueryFilter f = MixedFilter(r);
                PolyRef a = all[r.Int(all.Count)], b = all[r.Int(all.Count)];
                Vector3D pa = Centre(m, a), pb = Centre(m, b);
                QueryStatus s1 = cross.BeginPath(a, b, pa, pb, f);
                QueryStatus s2 = poly.BeginPath(a, b, pa, pb, f);
                Assert.Equal(s1, s2);
                if (s1 != QueryStatus.InProgress)
                {
                    continue;
                }
                s1 = cross.UpdatePath(int.MaxValue);
                s2 = poly.UpdatePath(int.MaxValue);
                Assert.Equal(s1, s2);
                QueryStatus f1 = cross.FinishPath(c1, out int n1, out _, out double cost1);
                QueryStatus f2 = poly.FinishPath(c2, out int n2, out _, out double cost2);
                Assert.Equal(f1, f2);
                Assert.Equal(a, c2[0]);
                if (s2 == QueryStatus.Success)
                {
                    reachable++;
                    Assert.Equal(b, c2[n2 - 1]);
                    // A route's cost is at least the straight distance at the cheapest area cost.
                    Assert.True(cost2 >= Vector3D.Distance(pa, pb) * 0.5 - 1e-9);
                    // Each polygon once, and every consecutive pair really connects.
                    var seen = new HashSet<ulong>();
                    for (int i = 0; i < n2; i++)
                    {
                        Assert.True(seen.Add(c2[i].Value));
                    }
                    Assert.Equal(QueryStatus.Success, poly.FindStraightPath(pa, pb, c2.AsSpan(0, n2), new StraightPathPoint[4096], out _, out _));
                    // The cheapest-route search over crossings is the exact optimum of its own cost model; the
                    // polygon model's route is a real route too, never cheaper than the shortest possible walk.
                    Assert.True(cost1 > 0 || a == b);
                }
                else
                {
                    unreachable++;
                    // An unreachable goal still gets a loop-free corridor from the start that string-pulls, as the crossing model's does.
                    Assert.Equal(QueryStatus.Partial, f2);
                    Assert.True(n2 >= 1 && n1 >= 1);
                    var seen = new HashSet<ulong>();
                    for (int i = 0; i < n2; i++)
                    {
                        Assert.True(seen.Add(c2[i].Value));
                    }
                    Assert.True(poly.ClosestPointOnPoly(c2[n2 - 1], pb, out Vector3D end, out _));
                    Assert.Equal(QueryStatus.Success, poly.FindStraightPath(pa, end, c2.AsSpan(0, n2), new StraightPathPoint[4096], out _, out _));
                }
            }
        }
        Assert.True(reachable > 40, $"only {reachable} reachable pairs");
        Assert.True(unreachable > 0, "no unreachable pair was drawn");
    }

    [Fact]
    public void PolygonModelSlicedEqualsUnsliced()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(8, 0, 0, 3, 3, 2, false, 2);
        var q = new NavMeshQuery(m, 65536, PathNodeModel.Polygon);
        var r = new NavMeshWorlds.Rng(17);
        List<PolyRef> all = AllPolys(m);
        for (int k = 0; k < 25; k++)
        {
            QueryFilter f = MixedFilter(r);
            PolyRef a = all[r.Int(all.Count)], b = all[r.Int(all.Count)];
            Vector3D pa = Centre(m, a), pb = Centre(m, b);
            var want = new PolyRef[4096];
            if (q.BeginPath(a, b, pa, pb, f) != QueryStatus.InProgress)
            {
                continue;
            }
            q.UpdatePath(int.MaxValue);
            QueryStatus wf = q.FinishPath(want, out int wc, out _, out double wcost);
            int wIter = q.Iterations;
            foreach (int budget in new[] { 1, 2, 3, 7, 50, 1000 })
            {
                q.BeginPath(a, b, pa, pb, f);
                QueryStatus s;
                do
                {
                    s = q.UpdatePath(budget);
                }
                while (s == QueryStatus.InProgress);
                var got = new PolyRef[4096];
                QueryStatus gf = q.FinishPath(got, out int gc, out _, out double gcost);
                Assert.Equal(wf, gf);
                Assert.Equal(wc, gc);
                Assert.Equal(wcost, gcost);
                Assert.True(want.AsSpan(0, wc).SequenceEqual(got.AsSpan(0, gc)));
                Assert.Equal(wIter, q.Iterations);
            }
        }
    }

    [Fact]
    public void PolygonModelSearchGrowsItsTableAndStillFindsLongRoutes()
    {
        // A wide mesh needs far more than the table's starting size; the route must still be found and be the same on a second run.
        TiledNavMesh m = NavMeshWorlds.RandomMesh(3, -2, -2, 4, 4, 1, false, 0);
        foreach (PathNodeModel model in new[] { PathNodeModel.Polygon, PathNodeModel.Crossing })
        {
            var q = new NavMeshQuery(m, 65536, model);
            List<PolyRef> all = AllPolys(m);
            PolyRef a = all[0], b = all[^1];
            var c1 = new PolyRef[4096];
            var c2 = new PolyRef[4096];
            QueryStatus s1 = q.FindPath(a, b, Centre(m, a), Centre(m, b), new PlainAll(), c1, out int n1);
            int it1 = q.Iterations;
            QueryStatus s2 = q.FindPath(a, b, Centre(m, a), Centre(m, b), new PlainAll(), c2, out int n2);
            Assert.Equal(s1, s2);
            Assert.Equal(n1, n2);
            Assert.Equal(it1, q.Iterations);
            Assert.True(c1.AsSpan(0, n1).SequenceEqual(c2.AsSpan(0, n2)));
        }
    }

    private struct PlainAll : IQueryFilter
    {
        public readonly bool Passes(ushort flags, byte area) => true;

        public readonly double AreaCost(byte area) => 1.0;

        public readonly double MinAreaCost => 1.0;
    }

    [Fact]
    public void UnknownNodeModelIsRejected()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(1, 0, 0, 1, 1, 1, false, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => new NavMeshQuery(m, 100, (PathNodeModel)7));
    }

    // The documented rule: the first detail triangle, in order, whose barycentric coordinates are all >= -1e-9.
    private static double ScanHeight(NavTile t, double x, double z, out bool found)
    {
        ReadOnlySpan<float> dv = t.DetailVertices;
        ReadOnlySpan<byte> dt = t.DetailTriangles;
        int n = t.PolyVertexCounts[0];
        found = false;
        for (int k = 0; k < t.DetailMeshes[0].TriangleCount; k++)
        {
            double[] ax = new double[3], ay = new double[3], az = new double[3];
            for (int c = 0; c < 3; c++)
            {
                int i = dt[(4 * k) + c];
                int di = 3 * (i - n);
                ax[c] = dv[di];
                ay[c] = dv[di + 1];
                az[c] = dv[di + 2];
            }
            double d = ((az[1] - az[2]) * (ax[0] - ax[2])) + ((ax[2] - ax[1]) * (az[0] - az[2]));
            if (d == 0)
            {
                continue;
            }
            double u = (((az[1] - az[2]) * (x - ax[2])) + ((ax[2] - ax[1]) * (z - az[2]))) / d;
            double w = (((az[2] - az[0]) * (x - ax[2])) + ((ax[0] - ax[2]) * (z - az[2]))) / d;
            double q = 1.0 - u - w;
            if (Math.Min(u, Math.Min(w, q)) >= -1e-9)
            {
                found = true;
                return (u * ay[0]) + (w * ay[1]) + (q * ay[2]);
            }
        }
        return 0;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void DetailHeightLookupEqualsAFullScanBitForBit(bool shuffle)
    {
        TiledNavMesh m = NavMeshWorlds.BumpyMesh(shuffle, out int count);
        Assert.True(count >= 12);
        var q = new NavMeshQuery(m);
        m.TryGetPoly(AllPolys(m)[0], out NavTile t, out _);
        var r = new NavMeshWorlds.Rng(99);
        int checkedPoints = 0;
        for (int i = 0; i < 4000; i++)
        {
            // A mix of random points, points on grid lines and diagonals, and the cell corners themselves.
            double x = i % 4 == 0 ? r.Int(9) : r.Int(8192) / 1024.0;
            double z = i % 4 == 1 ? r.Int(9) : (i % 4 == 2 ? x : r.Int(8192) / 1024.0);
            var pos = new Vector3D(t.OriginX + x, 0, t.OriginZ + z);
            Assert.True(q.ClosestPointOnPoly(AllPolys(m)[0], pos, out Vector3D c, out bool over));
            Assert.True(over);
            double want = ScanHeight(t, x, z, out bool found);
            Assert.True(found);
            Assert.Equal(t.OriginY + want, c.Y);
            checkedPoints++;
        }
        Assert.Equal(4000, checkedPoints);
    }
}
