using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Crowd scenarios: crowds of 10, 100 and 1000 agents walk generated meshes with a fixed time step (the number of worker threads comes from
/// the runner and must not change a bit), the state bytes of every agent are written at intervals, and the wall and corner-limit queries are hashed.
/// </summary>
internal static class CrowdScenarios
{
    private const double Dt = CrowdWorlds.Dt;

    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("crowds/orca-field-10", ctx => Run(ctx, CrowdWorlds.RandomField(8, 10, 5, ctx.Threads), 300, 50));
        Scenarios.Register("crowds/orca-door-100", ctx => Run(ctx, CrowdWorlds.ThroughDoor(100, 2, ctx.Threads), 250, 50));
        Scenarios.Register("crowds/orca-crossing-100", ctx => Run(ctx, CrowdWorlds.Crossing(100, 3, ctx.Threads), 250, 50));
        Scenarios.Register("crowds/orca-crossing-1000", ctx => Run(ctx, CrowdWorlds.Crossing(1000, 4, ctx.Threads), 40, 10));
        Scenarios.Register("crowds/orca-two-floors-edits", TwoFloorsWithEdits);
        Scenarios.Register("crowds/walls-query", WallsQuery);
    }

    private static void WriteAgents(ScenarioContext ctx, CrowdSim sim)
    {
        foreach (Agent a in sim.Agents)
        {
            if (!a.IsActive)
            {
                continue;
            }
            ctx.Write(a.Path.ByteSize);
            ctx.Write(a.Path.ToBytes());
        }
    }

    private static void Run(ScenarioContext ctx, CrowdSim sim, int steps, int every)
    {
        ctx.Write(sim.Agents.Count);
        for (int step = 0; step < steps; step++)
        {
            sim.Crowd.Step(Dt);
            if (step % every == every - 1)
            {
                WriteAgents(ctx, sim);
            }
        }
        WriteAgents(ctx, sim);
    }

    private static void TwoFloorsWithEdits(ScenarioContext ctx)
    {
        for (ulong world = 1; world <= 2; world++)
        {
            CrowdSim sim = CrowdWorlds.TwoFloors(12, world * 31, ctx.Threads);
            ctx.Write(sim.Agents.Count);
            var mut = new NavMeshWorlds.Rng(world + 7);
            for (int step = 0; step < 500; step++)
            {
                sim.Crowd.Step(Dt);
                if (step % 100 == 50)
                {
                    NavMeshMutations.Apply(sim.Mesh, mut, world * 31);
                }
                if (step % 50 == 49)
                {
                    WriteAgents(ctx, sim);
                }
                if (step == 300 && sim.Agents.Count > 2)
                {
                    sim.Crowd.Remove(sim.Agents[1]);
                    sim.Agents[0].SetDestination(new Vector3D(0.5, 0, 0.5));
                }
            }
            WriteAgents(ctx, sim);
        }
    }

    private static void WallsQuery(ScenarioContext ctx)
    {
        var walls = new WallSegment[64];
        var corners = new StraightPathPoint[16];
        for (ulong seed = 1; seed <= 4; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Field(24, seed);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed);
            for (int k = 0; k < 60; k++)
            {
                Vector3D a = CorridorWorlds.RandomFieldPoint(24, rng), b = CorridorWorlds.RandomFieldPoint(24, rng);
                NearestPoly na = q.FindNearestPoly(a, CorridorWorlds.Extents), nb = q.FindNearestPoly(b, CorridorWorlds.Extents);
                if (!na.Poly.IsSet || !nb.Poly.IsSet)
                {
                    continue;
                }
                ctx.Write((int)q.FindWalls(na.Poly, na.Point, 1.0 + (k % 7), (QueryFilter?)null, walls, out int count, out bool truncated));
                ctx.Write(count);
                ctx.Write(truncated ? 1 : 0);
                for (int i = 0; i < count; i++)
                {
                    WallSegment w = walls[i];
                    ctx.Write(w.AX);
                    ctx.Write(w.AY);
                    ctx.Write(w.AZ);
                    ctx.Write(w.BX);
                    ctx.Write(w.BY);
                    ctx.Write(w.BZ);
                    ctx.Write((long)w.Poly.Value);
                    ctx.Write(w.Edge);
                }
                var path = new PolyRef[256];
                if (q.FindPath(na.Poly, nb.Poly, na.Point, nb.Point, new CorridorWorlds.AllPass(), path, out int pc) != QueryStatus.Success)
                {
                    continue;
                }
                ctx.Write((int)q.FindStraightPath(na.Point, nb.Point, path.AsSpan(0, pc), corners, 1 + (k % 5), out int cc, out bool ct));
                ctx.Write(cc);
                ctx.Write(ct ? 1 : 0);
                for (int i = 0; i < cc; i++)
                {
                    ctx.Write(corners[i].Position.X);
                    ctx.Write(corners[i].Position.Y);
                    ctx.Write(corners[i].Position.Z);
                    ctx.Write((int)corners[i].Flags);
                }
            }
        }
    }
}
