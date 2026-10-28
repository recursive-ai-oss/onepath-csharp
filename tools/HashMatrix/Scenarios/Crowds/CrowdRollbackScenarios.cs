using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Rollback scenarios: a crowd with scripted edits runs straight through; a second one is snapshotted, runs on into another timeline,
/// is restored and replays. The state bytes of every tenth step and the snapshot bytes are hashed, and the scenario fails outright if
/// the replay differs from the straight run at any step, so a platform where rollback breaks is named by the scenario itself.
/// </summary>
internal static class CrowdRollbackScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("crowds/rollback-two-floors-edits", ctx => Run(ctx, true));
        Scenarios.Register("crowds/rollback-two-floors", ctx => Run(ctx, false));
        Scenarios.Register("crowds/state-hash", StateHash);
    }

    // The state hash after every step of a scripted run (written as a long), checked in the scenario against the hash of the
    // span-saved bytes and, after a restore, against the hash of the step it was taken at; also writes the final span snapshot.
    private static void StateHash(ScenarioContext ctx)
    {
        CrowdSim sim = CrowdRollback.Populated(ctx.Threads);
        byte[] buf = Array.Empty<byte>();
        byte[]? snap = null;
        ulong snapHash = 0;
        CrowdRollback.Run(sim, 0, 160, false, s =>
        {
            ulong h = sim.Crowd.StateHash();
            ctx.Write((long)h);
            if (s % 40 == 39)
            {
                if (buf.Length < sim.Crowd.GetStateSize())
                {
                    buf = new byte[sim.Crowd.GetStateSize() + 64];
                }
                if (!sim.Crowd.TrySaveState(buf, out int n) || !buf.AsSpan(0, n).SequenceEqual(sim.Crowd.SaveState()))
                {
                    throw new InvalidOperationException($"span snapshot differs from SaveState after step {s}");
                }
                ulong f = 0xCBF29CE484222325UL;
                for (int i = 0; i < n; i++)
                {
                    f = (f ^ buf[i]) * 0x100000001B3UL;
                }
                if (f != h)
                {
                    throw new InvalidOperationException($"state hash is not FNV-1a of the snapshot bytes after step {s}");
                }
                if (s == 79)
                {
                    snap = buf.AsSpan(0, n).ToArray();
                    snapHash = h;
                }
            }
        });
        sim.Crowd.RestoreState(snap);
        if (sim.Crowd.StateHash() != snapHash)
        {
            throw new InvalidOperationException("restored state hash differs from the hash at the snapshot");
        }
        ctx.Write((long)sim.Crowd.StateHash());
        ctx.Write(snap);
    }

    private static void Run(ScenarioContext ctx, bool meshEdits)
    {
        const int Total = 220, SnapAt = 80, Diverge = 140;
        var straight = new byte[Total][];
        CrowdSim a = CrowdRollback.Populated(ctx.Threads);
        CrowdRollback.Run(a, 0, Total, meshEdits, s => straight[s] = a.Crowd.SaveState());

        CrowdSim b = CrowdRollback.Populated(ctx.Threads);
        CrowdRollback.Run(b, 0, SnapAt, meshEdits);
        byte[] snap = b.Crowd.SaveStateWithMesh();
        CrowdRollback.Run(b, SnapAt, Diverge, meshEdits);
        b.Crowd.RestoreStateWithMesh(snap);
        CrowdRollback.Run(b, SnapAt, Total, meshEdits, s =>
        {
            if (!b.Crowd.SaveState().AsSpan().SequenceEqual(straight[s]))
            {
                throw new InvalidOperationException($"rollback replay differs from the straight run after step {s}");
            }
        });

        ctx.Write(snap.Length);
        ctx.Write(snap);
        for (int s = 9; s < Total; s += 10)
        {
            ctx.Write(straight[s].Length);
            ctx.Write(straight[s]);
        }
    }
}
