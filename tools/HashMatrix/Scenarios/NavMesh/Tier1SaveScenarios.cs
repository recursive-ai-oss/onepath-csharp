using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>The tier-1 save envelopes: a mesh and a two-class world are saved, loaded and saved again (the bytes must repeat); the bytes are hashed.</summary>
internal static class Tier1SaveScenarios
{
    [ModuleInitializer]
    internal static void Init() => Scenarios.Register("navmesh/tier1-save", Run);

    private static void Run(ScenarioContext ctx)
    {
        var cells = new bool[16 * 16];
        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = !((i % 16 == 5 && i / 16 != 3) || (i / 16 == 8 && i % 16 > 9));
        }
        NavSettings s = NavSettings.Humanoid with { CellSize = 0.5, TileSize = 8, OffMeshLinks = [OffMeshLinkDefinition.Create(new Vector3D(4.5, 0, 6), new Vector3D(6.5, 0, 6), 9)] };
        byte[] mesh = NavMesh.Build2D(cells, 16, 16, 1.0, new Vector2D(0, 0), s).Save();
        if (!NavMesh.Load(mesh).Save().AsSpan().SequenceEqual(mesh))
        {
            throw new InvalidOperationException("a loaded mesh saved different bytes");
        }
        var ring = new Vector2D[] { new(0, 0), new(16, 0), new(16, 16), new(0, 16) };
        byte[] world = NavWorld.Build2D([new Outline(ring)], s with { OffMeshLinks = [] }, NavSettings.Small with { CellSize = 0.5, TileSize = 8 }).Save();
        if (!NavWorld.Load(world).Save().AsSpan().SequenceEqual(world))
        {
            throw new InvalidOperationException("a loaded world saved different bytes");
        }
        ctx.Write(mesh.Length);
        ctx.Write(mesh);
        ctx.Write(world.Length);
        ctx.Write(world);
    }
}
