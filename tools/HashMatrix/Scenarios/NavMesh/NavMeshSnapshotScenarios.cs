using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Snapshot scenarios: the bytes of snapshots of meshes driven through fixed mutation sequences, in both modes, and the
/// state after an excursion and a restore (which must equal the state before it).
/// </summary>
internal static class NavMeshSnapshotScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("navmesh/snapshot-bytes", Bytes);

    private static void Bytes(ScenarioContext ctx)
    {
        for (ulong seed = 1; seed <= 4; seed++)
        {
            ulong world = seed * 104729;
            TiledNavMesh m = NavMeshWorlds.RandomMesh(world, -1, -1, 2, 2, 2, detail: seed % 2 == 1, offMesh: 2);
            var r = new NavMeshWorlds.Rng(seed);
            for (int i = 0; i < 40 + (int)(seed * 15); i++)
            {
                NavMeshMutations.Apply(m, r, world);
            }
            byte[] full = m.Snapshot();
            byte[] referenced = m.Snapshot(SnapshotMode.Referenced);
            ctx.Write(full.Length);
            ctx.Write(full);
            ctx.Write(referenced);
            var wild = new NavMeshWorlds.Rng(seed + 77);
            for (int i = 0; i < 25; i++)
            {
                NavMeshMutations.Apply(m, wild, world);
            }
            ctx.Write(m.Snapshot());
            m.Restore(full);
            ctx.Write(m.Snapshot());
            ctx.Write(NavMeshMutations.Digest(m, seed));
        }
    }
}
