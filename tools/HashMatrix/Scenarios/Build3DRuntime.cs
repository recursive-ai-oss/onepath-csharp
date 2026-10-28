using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for run-time changes of a 3D-built mesh: obstacles and area volumes rebuilt from the tile cache in slices, and the state snapshot.</summary>
internal static class Build3DRuntimeScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build3d/tilecache", TileCacheRun);
    }

    private static void Mesh(ScenarioContext ctx, TileCache c)
    {
        byte[] bytes = NavMeshSerializer.WriteMesh(c.Mesh);
        ctx.Write(bytes.Length);
        ctx.Write(bytes);
    }

    private static void TileCacheRun(ScenarioContext ctx)
    {
        // A rolling terrain over 30 x 30 units with an extra floor box, rebuilt after obstacles and volumes come and go.
        var v = new List<double>();
        var ix = new List<int>();
        for (int z = 0; z < 30; z++)
        {
            for (int x = 0; x < 30; x++)
            {
                int a = v.Count / 3;
                foreach ((double px, double pz) in new[] { (x, z), (x + 1.0, z), (x + 1.0, z + 1.0), (x, z + 1.0) })
                {
                    v.AddRange(new[] { px, (0.02 * px) + (0.002 * (px - 20) * (px - 20)) + (0.01 * pz) + ((((int)px * 7) + ((int)pz * 13)) % 5 * 0.01), pz });
                }
                ix.AddRange(new[] { a, a + 3, a + 2, a, a + 2, a + 1 });
            }
        }
        var settings = new NavSettings { CellSize = 0.25, CellHeight = 0.1, TileSize = 12, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };
        TileCache c = new Mesh3DPipeline(settings).BuildSoupCached(v.ToArray(), ix.ToArray());
        Mesh(ctx, c);
        int a1 = c.AddObstacle(TileObstacle.Cylinder(11.5, -1, 12, 2.3, 6));
        int a2 = c.AddObstacle(TileObstacle.Box(20, -1, 4, 24, 6, 9));
        int a3 = c.AddObstacle(TileObstacle.OrientedBox(8, 0, 22, 3, 5, 0.7, 0.6));
        int vol = c.AddVolume(new ConvexVolume(new double[] { 2, 2, 14, 3, 12, 14, 3, 11 }, -2, 8, 6));
        ctx.Write(c.PendingCount);
        ctx.Write(c.Update(2));
        byte[] snap = c.Snapshot();
        ctx.Write(snap.Length);
        ctx.Write(snap);
        c.RemoveObstacle(a2);
        ctx.Write(c.PendingCount);
        ctx.Write(c.UpdateAll());
        Mesh(ctx, c);
        c.Restore(snap);
        byte[] again = c.Snapshot(SnapshotMode.Referenced);
        ctx.Write(again.Length);
        ctx.Write(again);
        c.UpdateAll();
        Mesh(ctx, c);
        c.RemoveObstacle(a1);
        c.RemoveObstacle(a3);
        c.RemoveVolume(vol);
        c.UpdateAll();
        Mesh(ctx, c);
        ctx.Write(a1 + a2 + a3);
    }
}
