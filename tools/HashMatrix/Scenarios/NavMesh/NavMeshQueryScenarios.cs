using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for the navigation mesh query layer: fixed meshes, fixed query sets, every result written by bit pattern.</summary>
internal static class NavMeshQueryScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("navmesh/query-nearest", Nearest);
        Scenarios.Register("navmesh/query-path", ctx => Paths(ctx, PathNodeModel.Crossing));
        Scenarios.Register("navmesh/query-path-polygon", ctx => Paths(ctx, PathNodeModel.Polygon));
        Scenarios.Register("navmesh/query-detail-height", DetailHeights);
        Scenarios.Register("navmesh/query-raycast", Rays);
        Scenarios.Register("navmesh/query-random", RandomPoints);
        Scenarios.Register("navmesh/query-areas", Areas);
    }

    private static void Write(ScenarioContext ctx, Vector3D v)
    {
        ctx.Write(v.X);
        ctx.Write(v.Y);
        ctx.Write(v.Z);
    }

    private static Vector3D Spot(NavMeshWorlds.Rng r) =>
        new(-24 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0), (r.Int(1 << 20) / (double)(1 << 20) * 5.0) - 0.25, -24 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0));

    private static void Nearest(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(11, -1, -1, 3, 3, 3, detail: true, offMesh: 0);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(1);
        var f = new QueryFilter { ExcludeFlags = 4 };
        for (int i = 0; i < 300; i++)
        {
            NearestPoly n = q.FindNearestPoly(Spot(r), new Vector3D(1 + r.Int(3), 1 + r.Int(4), 1 + r.Int(3)), i % 2 == 0 ? null : f);
            ctx.Write((long)n.Poly.Value);
            Write(ctx, n.Point);
            ctx.Write(n.OverPolygon ? 1 : 0);
            ctx.Write(n.DistanceSquared);
        }
    }

    private static void Paths(ScenarioContext ctx, PathNodeModel model)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(12, -1, -1, 3, 3, 2, detail: true, offMesh: 2);
        var q = new NavMeshQuery(m, 65536, model);
        var r = new NavMeshWorlds.Rng(2);
        for (int i = 0; i < 120; i++)
        {
            var o = new PathOptions
            {
                SearchExtents = new Vector3D(2, 5, 2),
                Filter = i % 3 == 1 ? new QueryFilter { ExcludeFlags = 2 }.WithAreaCost(1, 3).WithAreaCost(9, 2) : null,
            };
            PathResult p = q.FindPath(Spot(r), Spot(r), o);
            ctx.Write((int)p.Detail);
            ctx.Write(p.Cost);
            ctx.Write(p.Corridor.Count);
            foreach (PolyRef c in p.Corridor)
            {
                ctx.Write((long)c.Value);
            }
            foreach (StraightPathPoint pt in p.Points)
            {
                Write(ctx, pt.Position);
                ctx.Write((int)pt.Flags);
                ctx.Write((long)pt.Poly.Value);
            }
        }
    }

    // Closest points on one polygon with a finely triangulated detail surface (the height lookup grid).
    private static void DetailHeights(ScenarioContext ctx)
    {
        foreach (bool shuffle in new[] { false, true })
        {
            TiledNavMesh m = NavMeshWorlds.BumpyMesh(shuffle, out _);
            var q = new NavMeshQuery(m);
            m.TryGetPoly(m.GetSlotRef(0).Poly(0), out NavTile t, out _);
            var r = new NavMeshWorlds.Rng(41);
            for (int i = 0; i < 400; i++)
            {
                var p = new Vector3D(t.OriginX - 0.5 + (r.Int(9 << 10) / 1024.0), 0.5, t.OriginZ - 0.5 + (r.Int(9 << 10) / 1024.0));
                q.ClosestPointOnPoly(m.GetSlotRef(0).Poly(0), p, out Vector3D c, out bool over);
                Write(ctx, c);
                ctx.Write(over ? 1 : 0);
            }
        }
    }

    private static void Rays(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(13, -1, -1, 3, 3, 2, detail: false, offMesh: 0);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(3);
        var visited = new PolyRef[256];
        for (int i = 0; i < 300; i++)
        {
            Vector3D a = Spot(r);
            NearestPoly n = q.FindNearestPoly(a, new Vector3D(2, 5, 2));
            if (!n.Poly.IsSet)
            {
                ctx.Write(0);
                continue;
            }
            RaycastResult res = q.Raycast(n.Poly, n.Point, Spot(r), null, visited, out int vc);
            ctx.Write((int)res.Status);
            ctx.Write(res.Hit ? 1 : 0);
            ctx.Write(res.T);
            Write(ctx, res.Normal);
            ctx.Write(res.HitEdge);
            ctx.Write(vc);
            for (int k = 0; k < vc; k++)
            {
                ctx.Write((long)visited[k].Value);
            }
        }
    }

    private static void RandomPoints(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(14, -1, -1, 3, 3, 3, detail: true, offMesh: 0);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(4);
        var rng = new SeededRandom(0x5EED);
        var f = new QueryFilter { ExcludeFlags = 4 };
        for (int i = 0; i < 400; i++)
        {
            RandomPointResult p = q.RandomPoint(ref rng, i % 2 == 0 ? null : f);
            ctx.Write((long)p.Poly.Value);
            Write(ctx, p.Point);
        }
        for (int i = 0; i < 200; i++)
        {
            Vector3D c = Spot(r);
            NearestPoly n = q.FindNearestPoly(c, new Vector3D(2, 5, 2));
            if (!n.Poly.IsSet)
            {
                ctx.Write(0);
                continue;
            }
            RandomPointResult p = q.RandomPointAround(ref rng, n.Poly, n.Point, 1 + r.Int(5), i % 3 == 0 ? f : null);
            ctx.Write((long)p.Poly.Value);
            Write(ctx, p.Point);
        }
        ctx.Write(rng.NextUInt64());
    }

    private static void Areas(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(15, -1, -1, 3, 3, 3, detail: true, offMesh: 2);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(5);
        var f = new QueryFilter { ExcludeFlags = 2 }.WithAreaCost(3, double.PositiveInfinity);
        var polys = new PolyRef[512];
        var parents = new PolyRef[512];
        var dist = new double[512];
        var visited = new PolyRef[64];
        var shape = new Vector3D[4];
        for (int i = 0; i < 200; i++)
        {
            Vector3D c = Spot(r);
            NearestPoly n = q.FindNearestPoly(c, new Vector3D(2, 5, 2));
            if (!n.Poly.IsSet)
            {
                ctx.Write(0);
                continue;
            }
            QueryFilter? filter = i % 2 == 0 ? null : f;
            double rad = 0.5 + r.Int(6);
            AreaQueryStatus s = q.FindPolysInCircle(n.Poly, n.Point, rad, filter, polys, parents, dist, out int cnt, out bool tr);
            ctx.Write((int)s);
            ctx.Write(cnt);
            ctx.Write(tr ? 1 : 0);
            for (int k = 0; k < cnt; k++)
            {
                ctx.Write((long)polys[k].Value);
                ctx.Write((long)parents[k].Value);
                ctx.Write(dist[k]);
            }
            shape[0] = new Vector3D(n.Point.X - rad, 0, n.Point.Z - 1);
            shape[1] = new Vector3D(n.Point.X + rad, 0, n.Point.Z - rad);
            shape[2] = new Vector3D(n.Point.X + 1, 0, n.Point.Z + rad);
            shape[3] = new Vector3D(n.Point.X - 1, 0, n.Point.Z + 1);
            s = q.FindPolysInShape(n.Poly, shape, filter, polys, parents, dist, out cnt, out tr);
            ctx.Write((int)s);
            ctx.Write(cnt);
            for (int k = 0; k < cnt; k++)
            {
                ctx.Write((long)polys[k].Value);
            }
            q.FindLocalNeighbourhood(n.Poly, n.Point, rad, filter, polys, parents, out cnt, out _);
            ctx.Write(cnt);
            for (int k = 0; k < cnt; k++)
            {
                ctx.Write((long)polys[k].Value);
            }
            WallHit w = q.DistanceToWall(n.Poly, n.Point, rad, filter, out _);
            ctx.Write(w.Found ? 1 : 0);
            ctx.Write(w.Distance);
            Write(ctx, w.Point);
            Write(ctx, w.Normal);
            ctx.Write((long)w.Poly.Value);
            ctx.Write(w.Edge);
            MoveResult mv = q.MoveAlongSurface(n.Poly, n.Point, Spot(r), filter, visited, out int vc);
            ctx.Write((int)mv.Status);
            Write(ctx, mv.Position);
            ctx.Write((long)mv.Poly.Value);
            ctx.Write(mv.Blocked ? 1 : 0);
            ctx.Write(vc);
        }
    }
}
