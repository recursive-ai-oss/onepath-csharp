using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// A scripted history for rollback checks: what happens at step <c>s</c> (mesh edits, agents added and removed, new destinations)
/// depends only on <c>s</c> and the crowd's current state, so a run that was rolled back and replayed meets exactly the actions of
/// a run that never left.
/// </summary>
internal static class CrowdRollback
{
    public const double Dt = 1.0 / 30.0;
    public const ulong WorldSeed = 14;

    private static CrowdSim Create(TiledNavMesh mesh, int threads) =>
        new(mesh, new Crowd(mesh, new CrowdOptions { MaxThreads = threads }), new List<Agent>());

    /// <summary>A new crowd (no agents) over a new copy of the world.</summary>
    public static CrowdSim EmptyOverWorld(int threads) => Create(CorridorWorlds.TwoFloors(WorldSeed), threads);

    /// <summary>The world with 24 agents walking between random places, some of them over off-mesh links.</summary>
    public static CrowdSim Populated(int threads) => CrowdWorlds.TwoFloors(24, WorldSeed, threads);

    /// <summary>Runs steps <paramref name="from"/> up to <paramref name="to"/> of the script; <paramref name="meshEdits"/> turns the mesh edits on.</summary>
    public static void Run(CrowdSim sim, int from, int to, bool meshEdits, Action<int>? afterStep = null)
    {
        Crowd crowd = sim.Crowd;
        for (int s = from; s < to; s++)
        {
            Act(crowd, s, meshEdits);
            crowd.Step(Dt);
            afterStep?.Invoke(s);
        }
    }

    private static void Act(Crowd crowd, int s, bool meshEdits)
    {
        TiledNavMesh mesh = crowd.Mesh;
        if (meshEdits && s % 40 == 20)
        {
            NavMeshMutations.Apply(mesh, new NavMeshWorlds.Rng(1000 + (ulong)s), WorldSeed);
        }
        const long idSpan = 40; // ids are 0.. in the order added; one that is gone or not yet added simply finds no agent
        if (s % 25 == 10 && crowd.GetAgent((s * 7L) % idSpan) is { } victim)
        {
            crowd.Remove(victim);
        }
        if (s % 30 == 5)
        {
            var rng = new NavMeshWorlds.Rng(5000 + (ulong)s);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            if (polys.Count > 0)
            {
                try
                {
                    Agent a = crowd.Add(CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), Agent.Small);
                    a.SetDestination(CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng));
                }
                catch (ArgumentException)
                {
                    // No walkable polygon near the random point after an edit: nothing is added.
                }
            }
        }
        if (s % 35 == 17 && crowd.GetAgent((s * 3L) % idSpan) is { } mover)
        {
            var rng = new NavMeshWorlds.Rng(9000 + (ulong)s);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            if (polys.Count > 0)
            {
                mover.SetDestination(CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng));
            }
        }
        if (s % 45 == 30 && crowd.GetAgent((s * 5L) % idSpan) is { } p)
        {
            p.Priority = 1 + (s % 4);
        }
    }

    /// <summary>The number of agents standing in an off-mesh crossing.</summary>
    public static int Crossing(Crowd crowd)
    {
        var list = new List<Agent>();
        crowd.CopyAgents(list);
        int n = 0;
        foreach (Agent a in list)
        {
            if (a.State == FollowerState.OffMeshLink)
            {
                n++;
            }
        }
        return n;
    }
}
