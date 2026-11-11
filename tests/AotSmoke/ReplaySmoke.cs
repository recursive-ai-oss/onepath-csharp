using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Replay;

/// <summary>A recorded crowd run is written, read back and played to its recorded hashes; damaged bytes are refused; used by the NativeAOT and WebAssembly smoke apps.</summary>
internal static class ReplaySmoke
{
    public static bool Run()
    {
        const int W = 20, H = 12;
        var cells = new bool[W * H];
        Array.Fill(cells, true);
        NavMesh nm = NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0, CellSize = 0.5, TileSize = 16 });
        var rec = new CrowdRecorder(nm.CreateCrowd(), 1.0 / 30.0, keepSteps: true);
        var live = new List<byte[]>();
        rec.Record = r => live.Add(r.ToArray());
        var agents = new List<Agent>();
        for (int i = 0; i < 5; i++)
        {
            agents.Add(rec.Add(new Vector3D(2, 0, 2 + i), Agent.Small));
            rec.SetDestination(agents[i], new Vector3D(17, 0, 2 + i));
        }

        for (int s = 0; s < 40; s++)
        {
            if (s == 10)
            {
                rec.Remove(agents[1]);
                rec.Teleport(agents[2], new Vector3D(8, 0, 8));
            }

            rec.Step();
        }

        byte[] bytes = rec.ToReplay().ToBytes();
        CrowdReplay read = CrowdReplay.Read(bytes);
        bool played = read.FindMismatch() is null && read.ToBytes().AsSpan().SequenceEqual(bytes) && live.Count == 5 + 5 + 2 + 40;
        bool rejected = false;
        try
        {
            CrowdReplay.Read(bytes.AsSpan(0, bytes.Length - 3));
        }
        catch (CrowdReplayException)
        {
            rejected = true;
        }

        return played && rejected;
    }
}
