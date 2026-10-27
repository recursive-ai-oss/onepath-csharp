using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.NavMesh;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for the tiled navigation mesh: serialized bytes and stitched links of fixed meshes.</summary>
internal static class NavMeshScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("navmesh/format-bytes", FormatBytes);
        Scenarios.Register("navmesh/stitched-links", StitchedLinks);
    }

    private static void FormatBytes(ScenarioContext ctx)
    {
        for (ulong seed = 1; seed <= 3; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed * 7919, -1, -1, 3, 2, (int)seed, detail: seed != 2, offMesh: (int)seed - 1);
            byte[] bytes = NavMeshSerializer.WriteMesh(m);
            ctx.Write(bytes.Length);
            ctx.Write(bytes);
            // Reading back and writing again must give the same bytes; a difference shows as a changed hash.
            ctx.Write(NavMeshSerializer.WriteMesh(NavMeshSerializer.ReadMesh(bytes)));
        }
    }

    // A tile set edited in a fixed sequence: stitched links, slot reuse and generations are part of the output.
    private static void StitchedLinks(ScenarioContext ctx)
    {
        NavMeshParams p = NavMeshWorlds.Params();
        TiledNavMesh m = NavMeshWorlds.RandomMesh(424242, 0, 0, 3, 3, 3, detail: false, offMesh: 0);
        NavMeshWorlds.DumpLinks(m, v => ctx.Write(v));
        m.RemoveTile(1, 1, 0);
        m.ReplaceTile(NavMeshWorlds.MakeTile(p, 99, 2, 2, 1, true, 0));
        m.AddTile(NavMeshWorlds.MakeTile(p, 5, 1, 1, 0, false, 2));
        m.AddTile(NavMeshWorlds.MakeTile(p, 6, 3, 1, 0, false, 3));
        NavMeshWorlds.DumpLinks(m, v => ctx.Write(v));
        ctx.Write(NavMeshSerializer.WriteMesh(m));
        var dbg = RecursiveAI.OnePath.Debug.NavMeshDebug.Collect(m);
        ctx.Write(dbg.Polygons.Count);
        ctx.Write(dbg.Links.Count);
        foreach (var l in dbg.Links)
        {
            ctx.Write(l.A.X);
            ctx.Write(l.A.Z);
            ctx.Write(l.B.Y);
        }
    }
}
