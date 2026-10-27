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
        Scenarios.Register("navmesh/query-path", Paths);
        Scenarios.Register("navmesh/query-raycast", Rays);
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

    private static void Paths(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(12, -1, -1, 3, 3, 2, detail: true, offMesh: 2);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(2);
        for (int i = 0; i < 120; i++)
        {
            var o = new PathOptions { SearchExtents = new Vector3D(2, 5, 2) };
            if (i % 3 == 1)
            {
                o.Filter = new QueryFilter { ExcludeFlags = 2 }.SetAreaCost(1, 3).SetAreaCost(9, 2);
            }
            PathResult p = q.FindPath(Spot(r), Spot(r), o);
            ctx.Write((int)p.Status);
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
}
