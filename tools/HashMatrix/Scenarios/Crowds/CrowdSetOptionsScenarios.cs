using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Live agent settings: every third agent gets slower and wider settings partway through a run. The run is snapshotted before and
/// after the change; restoring either snapshot and replaying must reproduce the straight run byte for byte, and the held agent
/// handles must stay valid and show the settings of the restored moment.
/// </summary>
internal static class CrowdSetOptionsScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("crowds/set-options", Run);

    private static void Change(CrowdSim sim)
    {
        for (int i = 0; i < sim.Agents.Count; i += 3)
        {
            Agent a = sim.Agents[i];
            if (a.IsActive)
            {
                a.SetOptions(a.Options with { MaxSpeed = a.Options.MaxSpeed / 2, Radius = a.Options.Radius * 1.25, Quality = AvoidanceQuality.High, Priority = 2 });
            }
        }
    }

    private static void Run(ScenarioContext ctx)
    {
        const int Before = 50, Total = 140;
        var straight = new byte[Total][];
        CrowdSim a = CrowdRollback.Populated(ctx.Threads);
        CrowdRollback.Run(a, 0, Before, false);
        byte[] pre = a.Crowd.SaveState();
        Change(a);
        byte[] post = a.Crowd.SaveState();
        CrowdRollback.Run(a, Before, Total, false, s => straight[s] = a.Crowd.SaveState());

        CrowdSim b = CrowdRollback.Populated(ctx.Threads);
        CrowdRollback.Run(b, 0, Before, false);
        b.Crowd.RestoreState(pre);
        Change(b);
        if (!b.Crowd.SaveState().AsSpan().SequenceEqual(post))
        {
            throw new InvalidOperationException("the same change on the same state gave different bytes");
        }
        CrowdRollback.Run(b, Before, 100, false);

        // Back to the moment after the change: same handles, and the replay equals the straight run.
        b.Crowd.RestoreState(post);
        CrowdRollback.Run(b, Before, Total, false, s =>
        {
            if (!b.Crowd.SaveState().AsSpan().SequenceEqual(straight[s]))
            {
                throw new InvalidOperationException($"replay after SetOptions differs from the straight run after step {s}");
            }
        });

        // Back to the moment before: the held agents are the same objects and show the old settings again.
        b.Crowd.RestoreState(pre);
        for (int i = 0; i < b.Agents.Count; i += 3)
        {
            Agent h = b.Agents[i];
            if (h.IsActive && h.Options.Priority != 1)
            {
                throw new InvalidOperationException("restoring the earlier snapshot did not restore the agent's settings");
            }
        }

        ctx.Write(pre.Length);
        ctx.Write(pre);
        ctx.Write(post.Length);
        ctx.Write(post);
        for (int s = Before + 9; s < Total; s += 10)
        {
            ctx.Write(straight[s].Length);
            ctx.Write(straight[s]);
        }
    }
}
