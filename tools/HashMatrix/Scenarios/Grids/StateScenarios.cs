using System;
using System.Linq;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Snapshot and rollback scenarios: for every world in <see cref="StateWorlds"/>, the snapshot bytes and the transcript of the run that
/// was rolled back. Each scenario also checks the rolled-back transcript against the straight one, so a rollback that
/// changes a result fails the run on every platform instead of only moving a hash.
/// </summary>
internal static class StateScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        foreach (string kind in StateWorlds.Kinds)
        {
            string k = kind;
            Scenarios.Register("grids/state/" + k, ctx =>
            {
                for (ulong seed = 1; seed <= 2; seed++)
                {
                    byte[] straight = StateWorlds.Run(k, seed, rollback: false);
                    byte[] rolled = StateWorlds.Run(k, seed, rollback: true);
                    if (!straight.AsSpan().SequenceEqual(rolled))
                    {
                        throw new InvalidOperationException($"Rollback changed the results of world '{k}' (seed {seed}).");
                    }
                    ctx.Write(rolled);
                    ctx.Write(StateWorlds.Snapshot(k, seed));
                }
            });
        }
    }
}
