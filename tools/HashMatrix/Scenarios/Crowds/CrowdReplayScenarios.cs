using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Recorded crowd runs: a scripted crowd (adds, removes, targets, stops, teleports) is recorded; the replay bytes, the live records and the
/// per-step hashes are hashed, and playing the replay must give the recorded hashes (the scenario fails outright if it does not).
/// </summary>
internal static class CrowdReplayScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("crowds/replay-record", ctx => Run(ctx, record: true));
        Scenarios.Register("crowds/replay-play", ctx => Run(ctx, record: false));
    }

    private static void Run(ScenarioContext ctx, bool record)
    {
        TiledNavMesh mesh = CrowdWorlds.Arena(30, 20);
        var crowd = new Crowd(mesh, new CrowdOptions { MaxThreads = ctx.Threads });
        var rec = new CrowdRecorder(crowd, CrowdWorlds.Dt, keepSteps: true);
        var records = new List<byte[]>();
        rec.Record = r => records.Add(r.ToArray());
        var rng = new NavMeshWorlds.Rng(5);
        var agents = new List<Agent>();
        Vector3D At(int x0, int x1) => new(x0 + (rng.Int(1000) / 1000.0 * (x1 - x0)), 0, 1 + (rng.Int(1000) / 1000.0 * 18));
        for (int i = 0; i < 12; i++)
        {
            agents.Add(rec.Add(At(1, 12), Agent.Small));
            rec.SetDestination(agents[i], At(18, 29));
        }

        for (int s = 0; s < 90; s++)
        {
            if (s == 20)
            {
                agents.Add(rec.Add(At(1, 12), Agent.Small));
                rec.SetDestination(agents[^1], At(18, 29));
            }
            else if (s == 35)
            {
                rec.Remove(agents[2]);
                agents.RemoveAt(2);
            }
            else if (s == 50)
            {
                rec.SetDestination(agents[0], At(1, 12));
                rec.Stop(agents[1]);
            }
            else if (s == 70)
            {
                rec.Teleport(agents[3], At(18, 29));
            }

            rec.Step();
        }

        CrowdReplay replay = rec.ToReplay();
        byte[] bytes = replay.ToBytes();
        CrowdReplay read = CrowdReplay.Read(bytes);
        if (read.FindMismatch() is int bad)
        {
            throw new InvalidOperationException($"replay differs from the recording at step {bad}");
        }

        if (!read.ToBytes().AsSpan().SequenceEqual(bytes))
        {
            throw new InvalidOperationException("read then write changed the replay bytes");
        }

        if (record)
        {
            ctx.Write(bytes);
            ctx.Write(records.Count);
            foreach (byte[] r in records)
            {
                ctx.Write(r);
            }
        }
        else
        {
            var player = new CrowdPlayer(read.Setup.Span);
            for (int i = 0; i < read.Steps.Count; i++)
            {
                player.Apply(read.Steps[i].Span);
                player.Step();
                ctx.Write((long)player.Crowd.StateHash());
            }
        }
    }
}
