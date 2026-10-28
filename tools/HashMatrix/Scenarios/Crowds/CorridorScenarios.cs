using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Path corridor and follower scenarios: agents walk generated meshes with a fixed time step while the mesh is edited on a
/// fixed schedule; the state bytes of every follower are written every few steps, and the corners and corridor at the end.
/// </summary>
internal static class CorridorScenarios
{
    private const double Dt = 1.0 / 30.0;

    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("crowds/corridor-follow-field", FollowField);
        Scenarios.Register("crowds/corridor-follow-edits", FollowWithEdits);
        Scenarios.Register("crowds/corridor-moves", Moves);
    }

    private static void WriteState(ScenarioContext ctx, PathFollower f)
    {
        ctx.Write(f.ByteSize);
        ctx.Write(f.ToBytes());
    }

    private static void FollowField(ScenarioContext ctx)
    {
        TiledNavMesh mesh = CorridorWorlds.Field(32, 5);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(3);
        var agents = new List<PathFollower>();
        for (int i = 0; i < 12; i++)
        {
            var f = new PathFollower(i % 3 == 0 ? FollowerOptions.Small : FollowerOptions.Humanoid);
            if (f.Place(q, CorridorWorlds.RandomFieldPoint(32, rng)))
            {
                ctx.Write((int)f.SetDestination(q, CorridorWorlds.RandomFieldPoint(32, rng)));
                agents.Add(f);
            }
        }
        for (int step = 0; step < 500; step++)
        {
            foreach (PathFollower f in agents)
            {
                f.Step(q, Dt);
            }
            if (step % 25 == 0)
            {
                foreach (PathFollower f in agents)
                {
                    WriteState(ctx, f);
                }
            }
            if (step == 250)
            {
                // New orders mid-way.
                foreach (PathFollower f in agents)
                {
                    ctx.Write((int)f.SetDestination(q, CorridorWorlds.RandomFieldPoint(32, rng)));
                }
            }
        }
        foreach (PathFollower f in agents)
        {
            WriteState(ctx, f);
            foreach (StraightPathPoint c in f.Corners)
            {
                ctx.Write(c.Position.X);
                ctx.Write(c.Position.Y);
                ctx.Write(c.Position.Z);
                ctx.Write((int)c.Flags);
                ctx.Write((long)c.Poly.Value);
            }
        }
    }

    private static void FollowWithEdits(ScenarioContext ctx)
    {
        for (ulong world = 1; world <= 3; world++)
        {
            TiledNavMesh mesh = CorridorWorlds.TwoFloors(world * 4093);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(world);
            var mut = new NavMeshWorlds.Rng(world + 100);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            var agents = new List<PathFollower>();
            for (int i = 0; i < 8; i++)
            {
                var f = new PathFollower(FollowerOptions.Humanoid with { ReplanBudget = 64 });
                if (f.Place(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng)))
                {
                    ctx.Write((int)f.SetDestination(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng)));
                    agents.Add(f);
                }
            }
            for (int step = 0; step < 400; step++)
            {
                foreach (PathFollower f in agents)
                {
                    f.Step(q, Dt);
                }
                if (step % 40 == 20)
                {
                    NavMeshMutations.Apply(mesh, mut, world * 4093);
                }
                if (step % 20 == 0)
                {
                    foreach (PathFollower f in agents)
                    {
                        WriteState(ctx, f);
                    }
                }
            }
            // Snapshot, run on, restore, run on again: the bytes after the second run must equal those after the first.
            byte[][] saved = new byte[agents.Count][];
            for (int i = 0; i < agents.Count; i++)
            {
                saved[i] = agents[i].ToBytes();
            }
            for (int pass = 0; pass < 2; pass++)
            {
                for (int step = 0; step < 150; step++)
                {
                    foreach (PathFollower f in agents)
                    {
                        f.Step(q, Dt);
                    }
                }
                foreach (PathFollower f in agents)
                {
                    WriteState(ctx, f);
                }
                for (int i = 0; i < agents.Count; i++)
                {
                    agents[i].Read(saved[i]);
                }
            }
        }
    }

    private static void Moves(ScenarioContext ctx)
    {
        TiledNavMesh mesh = CorridorWorlds.Maze(8, 3, out List<Vector3D> rooms);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(9);
        var corners = new StraightPathPoint[64];
        for (int trial = 0; trial < 20; trial++)
        {
            NearestPoly na = q.FindNearestPoly(rooms[rng.Int(rooms.Count)], CorridorWorlds.Extents);
            var c = new PathCorridor(256);
            c.Reset(na.Poly, na.Point);
            c.SetTarget(rooms[rng.Int(rooms.Count)]);
            ctx.Write((int)c.Replan(q));
            for (int move = 0; move < 30; move++)
            {
                if (move % 3 == 2)
                {
                    c.MoveTarget(q, rooms[rng.Int(rooms.Count)]);
                }
                else
                {
                    c.FindCorners(q, corners, out int n);
                    c.MovePosition(q, Vector3D.Lerp(c.Position, corners[n > 1 ? 1 : 0].Position, 0.4 + (0.2 * rng.Int(3))));
                }
                if (move % 7 == 6)
                {
                    c.OptimizeVisibility(q, c.Target, 20, null, 16);
                    c.OptimizeTopology(q, null, 32, 8);
                }
                ctx.Write(c.ByteSize);
                ctx.Write(c.ToBytes());
            }
        }
    }
}
