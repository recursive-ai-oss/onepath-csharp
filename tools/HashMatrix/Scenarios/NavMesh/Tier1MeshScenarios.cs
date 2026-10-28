using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// The one-call mesh forms. <c>navmesh/pooled-paths</c>: paths through the pooled query equal paths through fresh query objects
/// (checked inside, bit for bit) while the mesh is edited between calls, and are written to the hash. <c>navmesh/tier1-queries</c>:
/// sample, raycast and random point on a built mesh.
/// </summary>
internal static class Tier1MeshScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("navmesh/pooled-paths", PooledPaths);
        Scenarios.Register("navmesh/tier1-queries", Tier1Queries);
    }

    private static void Write(ScenarioContext ctx, Vector3D v)
    {
        ctx.Write(v.X);
        ctx.Write(v.Y);
        ctx.Write(v.Z);
    }

    private static Vector3D Spot(NavMeshWorlds.Rng r) =>
        new(-24 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0), (r.Int(1 << 20) / (double)(1 << 20) * 5.0) - 0.25, -24 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0));

    private static bool Same(PathResult a, PathResult b)
    {
        if (a.Detail != b.Detail || a.Points.Count != b.Points.Count || a.Corridor.Count != b.Corridor.Count
            || BitConverter.DoubleToInt64Bits(a.Cost) != BitConverter.DoubleToInt64Bits(b.Cost))
        {
            return false;
        }
        for (int i = 0; i < a.Points.Count; i++)
        {
            Vector3D p = a.Points[i].Position, q = b.Points[i].Position;
            if (BitConverter.DoubleToInt64Bits(p.X) != BitConverter.DoubleToInt64Bits(q.X) || BitConverter.DoubleToInt64Bits(p.Y) != BitConverter.DoubleToInt64Bits(q.Y)
                || BitConverter.DoubleToInt64Bits(p.Z) != BitConverter.DoubleToInt64Bits(q.Z))
            {
                return false;
            }
        }
        for (int i = 0; i < a.Corridor.Count; i++)
        {
            if (a.Corridor[i].Value != b.Corridor[i].Value)
            {
                return false;
            }
        }
        return true;
    }

    private static void PooledPaths(ScenarioContext ctx)
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(12, -1, -1, 3, 3, 2, detail: true, offMesh: 2);
        var r = new NavMeshWorlds.Rng(2);
        for (int i = 0; i < 160; i++)
        {
            if (i % 40 == 20)
            {
                // Edit between calls: the pooled query must follow it.
                NavMeshMutations.Apply(m, new NavMeshWorlds.Rng(300 + (ulong)i), 12);
            }
            var o = new PathOptions
            {
                SearchExtents = new Vector3D(2, 5, 2),
                Filter = i % 3 == 1 ? new QueryFilter { ExcludeFlags = 2 }.WithAreaCost(1, 3) : null,
            };
            Vector3D a = Spot(r), b = Spot(r);
            PathResult pooled = m.FindPath(a, b, o);
            PathResult fresh = new NavMeshQuery(m).FindPath(a, b, o);
            if (!Same(pooled, fresh))
            {
                throw new InvalidOperationException($"pooled path differs from a fresh query's at call {i}");
            }
            ctx.Write((long)pooled.Detail);
            ctx.Write(pooled.Points.Count);
            ctx.Write(pooled.Cost);
            for (int k = 0; k < pooled.Points.Count; k++)
            {
                Write(ctx, pooled.Points[k].Position);
            }
        }
    }

    private static void Tier1Queries(ScenarioContext ctx)
    {
        const int W = 24, H = 24;
        var cells = new bool[W * H];
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                cells[(y * W) + x] = !(x % 6 == 3 && y % 8 != 4) && !(x > 14 && y > 15 && x < 19);
            }
        }
        NavMesh nav = NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(-12, -12));
        var rng = new SeededRandom(0x5EED);
        var r = new NavMeshWorlds.Rng(77);
        for (int i = 0; i < 200; i++)
        {
            var p = new Vector3D(-14 + (r.Int(1 << 20) / (double)(1 << 20) * 28.0), 0, -14 + (r.Int(1 << 20) / (double)(1 << 20) * 28.0));
            bool ok = nav.TrySamplePosition(p, 1.5, out Vector3D snapped);
            ctx.Write(ok ? 1 : 0);
            Write(ctx, snapped);
            var q = new Vector3D(-12 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0), 0, -12 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0));
            bool blocked = nav.Raycast(p, q, out NavMeshHit hit);
            ctx.Write(blocked ? 1 : 0);
            Write(ctx, hit.Position);
            Write(ctx, hit.Normal);
            ctx.Write(hit.Distance);
            bool got = nav.TryGetRandomPoint(ref rng, out Vector3D rp);
            ctx.Write(got ? 1 : 0);
            Write(ctx, rp);
        }
    }
}
