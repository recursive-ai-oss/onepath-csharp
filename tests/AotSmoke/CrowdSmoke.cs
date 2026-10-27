using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Debug;

/// <summary>A crowd crosses a room through a doorway with threads and without, the results are compared bit for bit; used by the NativeAOT and WebAssembly smoke apps.</summary>
internal static class CrowdSmoke
{
    public static bool Run()
    {
        // Two 12 x 12 rooms joined by a 2-cell door in the wall between them.
        const int W = 25, H = 12;
        var cells = new bool[W * H];
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                cells[(y * W) + x] = x != 12 || y is 5 or 6;
            }
        }
        NavMesh nm = NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0, CellSize = 0.5, TileSize = 16 });
        byte[] one = Walk(nm, 1, out bool arrived, out bool onMesh, out bool debug);
        byte[] many = Walk(nm, 4, out _, out _, out _);
        // Bad input is rejected with the documented errors.
        bool rejected = false;
        try
        {
            nm.CreateCrowd().Step(0);
        }
        catch (ArgumentOutOfRangeException)
        {
            rejected = true;
        }
        return arrived && onMesh && debug && rejected && one.AsSpan().SequenceEqual(many);
    }

    private static byte[] Walk(NavMesh nm, int threads, out bool arrived, out bool onMesh, out bool debug)
    {
        Crowd crowd = nm.CreateCrowd(new CrowdOptions { MaxThreads = threads, RecordConstraints = true });
        var agents = new List<Agent>();
        for (int i = 0; i < 24; i++)
        {
            Agent a = crowd.Add(new Vector3D(1.5 + (i % 4), 0, 1.5 + ((i / 4) * 1.5)), Agent.Small);
            a.SetDestination(new Vector3D(14.5 + (i % 4), 0, 1.5 + ((i / 4) * 1.5)));
            agents.Add(a);
        }
        onMesh = true;
        for (int step = 0; step < 1500; step++)
        {
            crowd.Step(1.0 / 30);
            foreach (Agent a in agents)
            {
                onMesh &= a.Position.X >= 0 && a.Position.X <= 25 && a.Position.Z >= 0 && a.Position.Z <= 12;
            }
        }
        arrived = true;
        var bytes = new List<byte>();
        foreach (Agent a in agents)
        {
            arrived &= a.State == RecursiveAI.OnePath.Crowds.Corridors.FollowerState.Arrived;
            bytes.AddRange(a.Path.ToBytes());
        }
        debug = CrowdDebug.Collect(crowd).Agents.Count == agents.Count;
        return bytes.ToArray();
    }
}
