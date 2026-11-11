using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.Scope.Tests;

/// <summary>Generated worlds and the scripted crowd runs the tests and the committed files come from.</summary>
internal static class Worlds
{
    public const double Dt = 1.0 / 30.0;
    public const int Steps = 240;

    private sealed class Rng(ulong seed)
    {
        private ulong _s = seed;

        public int Next(int n)
        {
            _s += 0x9E3779B97F4A7C15UL;
            ulong z = _s;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return (int)((z ^ (z >> 31)) % (ulong)n);
        }
    }

    /// <summary>A 40 x 24 arena split by a wall with a four-cell door; flat, so the module shows it in 2D.</summary>
    public static NavMesh Arena2D()
    {
        const int W = 40, H = 24;
        var cells = new bool[W * H];
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                cells[(y * W) + x] = x != 20 || (y >= 10 && y < 14);
            }
        }

        return NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0.0, CellSize = 0.5, TileSize = 16 });
    }

    /// <summary>The triangles of a floor with a raised platform reached by a ramp and a wall block: two levels, so the module shows it in 3D.</summary>
    public static (double[] Vertices, int[] Indices) Terrain3D()
    {
        var v = new List<double>();
        var t = new List<int>();
        void Quad(double x0, double y0, double z0, double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3)
        {
            int b = v.Count / 3;
            v.AddRange([x0, y0, z0, x1, y1, z1, x2, y2, z2, x3, y3, z3]);
            t.AddRange([b, b + 1, b + 2, b, b + 2, b + 3]);
        }

        // Floor (counter-clockwise seen from above), ramp from x = 24 to 30 up to a platform at height 2 from x = 30 to 36.
        Quad(0, 0, 0, 0, 0, 24, 24, 0, 24, 24, 0, 0);
        Quad(24, 0, 0, 24, 0, 24, 30, 2, 24, 30, 2, 0);
        Quad(30, 2, 0, 30, 2, 24, 36, 2, 24, 36, 2, 0);
        // A block on the floor, its top at height 3 (too high to climb), as a closed box seen from outside.
        double x0 = 8, x1 = 12, z0 = 8, z1 = 14, h = 3;
        Quad(x0, h, z0, x0, h, z1, x1, h, z1, x1, h, z0);
        Quad(x0, 0, z0, x0, h, z0, x1, h, z0, x1, 0, z0);
        Quad(x1, 0, z1, x1, h, z1, x0, h, z1, x0, 0, z1);
        Quad(x0, 0, z1, x0, h, z1, x0, h, z0, x0, 0, z0);
        Quad(x1, 0, z0, x1, h, z0, x1, h, z1, x1, 0, z1);
        return ([.. v], [.. t]);
    }

    public static NavSettings Settings3D => NavSettings.Humanoid with { AgentRadius = 0.4, AgentHeight = 1.8, WalkableClimb = 0.6, CellSize = 0.3, CellHeight = 0.15, TileSize = 16, MinRegionArea = 8, MergeRegionArea = 20 };

    public static NavMesh Mesh3D()
    {
        (double[] v, int[] i) = Terrain3D();
        return NavMesh.Build(v, i, Settings3D);
    }

    /// <summary>What a host does to its crowd: the scripted run is written against this, so a recorder and a bare crowd run the same script.</summary>
    public interface IHost
    {
        Crowd Crowd { get; }

        Agent Add(Vector3D at, AgentOptions options);

        void Remove(Agent a);

        void Go(Agent a, Vector3D to);

        void Stop(Agent a);

        void Teleport(Agent a, Vector3D to);

        void Step();
    }

    public sealed class BareHost(Crowd crowd) : IHost
    {
        public Crowd Crowd { get; } = crowd;

        public Agent Add(Vector3D at, AgentOptions options) => Crowd.Add(at, options);

        public void Remove(Agent a) => Crowd.Remove(a);

        public void Go(Agent a, Vector3D to) => a.SetDestination(to);

        public void Stop(Agent a) => a.Stop();

        public void Teleport(Agent a, Vector3D to) => a.Teleport(to);

        public void Step() => Crowd.Step(Dt);
    }

    public sealed class RecordingHost(CrowdRecorder rec) : IHost
    {
        public Crowd Crowd => rec.Crowd;

        public Agent Add(Vector3D at, AgentOptions options) => rec.Add(at, options);

        public void Remove(Agent a) => rec.Remove(a);

        public void Go(Agent a, Vector3D to) => rec.SetDestination(a, to);

        public void Stop(Agent a) => rec.Stop(a);

        public void Teleport(Agent a, Vector3D to) => rec.Teleport(a, to);

        public void Step() => rec.Step();
    }

    /// <summary>
    /// Thirty agents on the left walk to the right; mid-run four more join, three leave, eight change target, two stop, one teleports,
    /// then two leave and two join. <paramref name="y"/> is the floor height; <paramref name="left"/>/<paramref name="right"/> are x ranges.
    /// </summary>
    public static void RunScript(IHost host, ulong seed, double y, double zMax, double leftMax, double rightMin, double rightMax, int steps = Steps, Action<int>? afterStep = null)
    {
        var rng = new Rng(seed);
        AgentOptions small = Agent.Small;
        var agents = new List<Agent>();
        Vector3D Left() => new(1 + (rng.Next(1000) / 1000.0 * (leftMax - 2)), y, 1 + (rng.Next(1000) / 1000.0 * (zMax - 2)));
        Vector3D Right() => new(rightMin + (rng.Next(1000) / 1000.0 * (rightMax - rightMin)), y, 1 + (rng.Next(1000) / 1000.0 * (zMax - 2)));
        for (int i = 0; i < 30; i++)
        {
            agents.Add(host.Add(Left(), small));
            host.Go(agents[i], Right());
        }

        for (int s = 0; s < steps; s++)
        {
            switch (s)
            {
                case 40:
                    for (int i = 0; i < 4; i++)
                    {
                        Agent a = host.Add(Left(), small);
                        agents.Add(a);
                        host.Go(a, Right());
                    }

                    break;
                case 60:
                    for (int i = 0; i < 3; i++)
                    {
                        host.Remove(agents[(i * 7) % agents.Count]);
                        agents.RemoveAt((i * 7) % agents.Count);
                    }

                    break;
                case 90:
                    for (int i = 0; i < 8; i++)
                    {
                        host.Go(agents[(i * 3) % agents.Count], Left());
                    }

                    break;
                case 120:
                    host.Stop(agents[1]);
                    host.Stop(agents[5]);
                    break;
                case 150:
                    host.Teleport(agents[2], Right());
                    break;
                case 180:
                    host.Remove(agents[0]);
                    agents.RemoveAt(0);
                    host.Remove(agents[3]);
                    agents.RemoveAt(3);
                    Agent n1 = host.Add(Left(), small);
                    agents.Add(n1);
                    host.Go(n1, Right());
                    agents.Add(host.Add(Right(), small));
                    break;
            }

            host.Step();
            afterStep?.Invoke(s);
        }
    }

    public static CrowdReplay Record2D(out List<ulong> hostHashes)
    {
        NavMesh mesh = Arena2D();
        var rec = new CrowdRecorder(mesh.CreateCrowd(), Dt, keepSteps: true);
        var hashes = new List<ulong>();
        RunScript(new RecordingHost(rec), 11, 0, 24, 18, 22, 39, afterStep: _ => hashes.Add(rec.Crowd.StateHash()));
        hostHashes = hashes;
        return rec.ToReplay();
    }

    public static CrowdReplay Record3D(out List<ulong> hostHashes)
    {
        NavMesh mesh = Mesh3D();
        var rec = new CrowdRecorder(mesh.CreateCrowd(), Dt, keepSteps: true);
        var hashes = new List<ulong>();
        RunScript(new RecordingHost(rec), 12, 0, 24, 7, 14, 23, afterStep: _ => hashes.Add(rec.Crowd.StateHash()));
        hostHashes = hashes;
        return rec.ToReplay();
    }


    /// <summary>A 48 x 32 grid with a ragged wall that has two gaps, and a few scattered blocks.</summary>
    public static Grid MazeGrid()
    {
        var g = new Grid(48, 32);
        for (int y = 0; y < 32; y++)
        {
            if (y is not (6 or 7 or 22))
            {
                g.SetWalkable(24, y, false);
            }
        }

        for (int i = 0; i < 40; i++)
        {
            g.SetWalkable(3 + ((i * 7) % 18), 2 + ((i * 11) % 28), false);
            g.SetWalkable(28 + ((i * 5) % 18), 1 + ((i * 13) % 30), false);
        }

        foreach (Cell c in new[] { new Cell(2, 16), new Cell(45, 16), new Cell(24, 6), new Cell(24, 7), new Cell(24, 22) })
        {
            g.SetWalkable(c.X, c.Y, true);
        }

        return g;
    }

    public static IReadOnlyList<IReadOnlyList<GridRunOp>> GridEdits(int steps, bool search)
    {
        var list = new List<IReadOnlyList<GridRunOp>>();
        for (int i = 0; i < steps; i++)
        {
            GridRunOp[] ops = i switch
            {
                10 => [GridRunOp.SetArea(24, 6, 255), GridRunOp.SetArea(24, 7, 255)],
                25 => [GridRunOp.SetGoal(44, 4)],
                40 => [GridRunOp.SetArea(24, 6, 0)],
                55 when search => [GridRunOp.SetStart(3, 28)],
                _ => [],
            };
            list.Add(ops);
        }

        return list;
    }

    public static byte[] GridSearchFile(GridAlgorithm algorithm, int agentSize = 1)
    {
        var w = new ArrayBufferWriter<byte>();
        GridRun.WriteSearch(MazeGrid(), new GridOptions { Algorithm = algorithm, AgentSize = agentSize }, new Cell(2, 16), new Cell(45, 16), 24, GridEdits(80, true), w);
        return w.WrittenSpan.ToArray();
    }

    public static byte[] GridFlowFile()
    {
        var w = new ArrayBufferWriter<byte>();
        GridRun.WriteFlow(MazeGrid(), null, [new Cell(45, 16), new Cell(2, 30)], 60, GridEdits(60, false), w);
        return w.WrittenSpan.ToArray();
    }

    public static byte[] BuildInputBytes()
    {
        (double[] v, int[] i) = Terrain3D();
        var w = new ArrayBufferWriter<byte>();
        BuildInput.Write(v, i, Settings3D, w);
        return w.WrittenSpan.ToArray();
    }

    public static string DataPath(string file)
    {
        string p = Path.Combine(AppContext.BaseDirectory, "testdata", file);
        return p;
    }
}
