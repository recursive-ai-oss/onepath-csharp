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
