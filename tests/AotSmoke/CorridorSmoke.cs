using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

/// <summary>An agent walks a corridor of a small mesh with a fixed time step, its state is snapshotted and resumed; used by the NativeAOT and WebAssembly smoke apps.</summary>
internal static class CorridorSmoke
{
    public static bool Run()
    {
        // An L-shaped walkable area: a 20 x 4 strip with a 4 x 20 strip rising from its right end.
        var cells = new bool[20 * 20];
        for (int y = 0; y < 20; y++)
        {
            for (int x = 0; x < 20; x++)
            {
                cells[(y * 20) + x] = y < 4 || x >= 16;
            }
        }
        NavMesh nm = NavMesh.Build2D(cells, 20, 20, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0, CellSize = 0.5, TileSize = 16 });
        var q = new NavMeshQuery(nm.Tiles);
        var f = new PathFollower();
        if (!f.Place(q, new Vector3D(1.5, 0, 1.5)) || f.SetDestination(q, new Vector3D(17.5, 0, 18.5)) != QueryStatus.Success)
        {
            return false;
        }
        byte[]? saved = null;
        for (int i = 0; i < 2000 && f.HasPath; i++)
        {
            f.Step(q, 1.0 / 30);
            if (i == 100)
            {
                saved = f.ToBytes();
            }
        }
        // Resume from the snapshot: the same steps must reach the same state.
        var g = new PathFollower();
        g.Read(saved!);
        for (int i = 101; i < 2000 && g.HasPath; i++)
        {
            g.Step(q, 1.0 / 30);
        }
        bool arrived = f.State == FollowerState.Arrived && f.ReachedTarget && Math.Abs(f.Position.X - 17.5) < 0.02 && Math.Abs(f.Position.Z - 18.5) < 0.02;
        bool same = f.ToBytes().AsSpan().SequenceEqual(g.ToBytes());
        // Bad input is rejected with the documented errors.
        bool rejected = false;
        try
        {
            f.Step(q, double.NaN);
        }
        catch (ArgumentOutOfRangeException)
        {
            rejected = true;
        }
        return arrived && same && rejected;
    }
}
