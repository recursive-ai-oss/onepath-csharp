using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for crowds. The input is a program: the first bytes pick a mesh, the thread count and options, the rest is read as
/// operations (add and remove agents, set destinations, teleport, change a priority, step, edit the mesh, collect debug geometry).
/// Coordinates, time steps and options are mostly sane but sometimes not: those calls must throw the documented
/// <see cref="ArgumentException"/> (or a subclass) or <see cref="InvalidOperationException"/> and change nothing. After every step each
/// agent must have a finite position that is on the mesh and a finite velocity, and the agent count must match. Any other
/// exception, or a broken invariant, crashes the target.
/// </summary>
internal static class CrowdFuzz
{
    // Built on first use, not in a type initializer: the coverage instrumentation is not ready inside one.
    private static TiledNavMesh[]? s_meshes;
    private static byte[][]? s_base;

    private ref struct Input
    {
        private readonly ReadOnlySpan<byte> _d;
        private int _i;

        public Input(ReadOnlySpan<byte> d)
        {
            _d = d;
            _i = 0;
        }

        public bool More => _i < _d.Length;

        public int Byte() => _i < _d.Length ? _d[_i++] : 0;

        public double Coord(double lo, double hi)
        {
            int b = Byte();
            int sel = Byte();
            if (sel == 255)
            {
                return b switch { < 64 => double.NaN, < 128 => double.PositiveInfinity, < 192 => 3e6, _ => -2097152.0 };
            }
            return lo + ((((b << 8) | sel) / 65536.0) * (hi - lo));
        }
    }

    public static void Run(byte[] data)
    {
        if (data.Length < 4)
        {
            return;
        }
        if (s_meshes is null || s_base is null)
        {
            TiledNavMesh[] meshes = [CrowdWorlds.Field2(7, 3), CorridorWorlds.TwoFloors(7), CrowdWorlds.Doorway(8, 12, 2)];
            s_base = [meshes[0].Snapshot(), meshes[1].Snapshot(), meshes[2].Snapshot()];
            s_meshes = meshes;
        }
        int which = data[0] % s_meshes.Length;
        TiledNavMesh mesh = s_meshes[which];
        mesh.Restore(s_base[which]);
        var options = new CrowdOptions
        {
            MaxThreads = 1 + (data[1] % 3),
            MaxAgents = 1 + (data[1] >> 2) % 40,
            RecordConstraints = (data[2] & 1) != 0,
            MaxWallsPerAgent = 4 + (data[2] >> 1) % 20,
            WallRefreshDistance = 0.25 + ((data[2] >> 3) % 8) * 0.25,
            QueryNodes = 256,
            Filter = (data[3] & 1) != 0 ? new QueryFilter { ExcludeFlags = (ushort)((data[3] >> 1) & 3) } : null,
        };
        var crowd = new Crowd(mesh, options);
        var q = new NavMeshQuery(mesh, 1024);
        var inp = new Input(data.AsSpan(4));
        double x0 = which == 1 ? -16 : 0;
        double w = which == 0 ? 14 : which == 1 ? 16 : 17;
        double h = which == 0 ? 14 : which == 1 ? 16 : 12;
        var mut = new NavMeshWorlds.Rng(data[3]);
        var agents = new List<Agent>();
        bool edited = false;
        for (int op = 0; op < 120 && inp.More; op++)
        {
            switch (inp.Byte() % 10)
            {
                case 0:
                case 1:
                {
                    int p = inp.Byte();
                    AgentOptions ao = p switch
                    {
                        < 100 => Agent.Small,
                        < 190 => Agent.Humanoid,
                        < 220 => Agent.Large,
                        < 235 => Agent.Small with { Radius = -1 },
                        < 245 => Agent.Small with { Quality = AvoidanceQuality.Off, SeparationDistance = 1.5 },
                        _ => Agent.Small with { Priority = 3, Quality = AvoidanceQuality.High, SeparationDistance = 2, SidePreference = 0 },
                    };
                    var pos = new Vector3D(inp.Coord(x0 - 1, x0 + w + 1), inp.Coord(-1, 5), inp.Coord(x0 - 1, x0 + h + 1));
                    try
                    {
                        agents.Add(crowd.Add(pos, ao));
                    }
                    catch (ArgumentException)
                    {
                    }
                    catch (InvalidOperationException)
                    {
                    }
                    break;
                }
                case 2:
                    if (agents.Count > 0)
                    {
                        Agent a = agents[inp.Byte() % agents.Count];
                        var dest = new Vector3D(inp.Coord(x0 - 1, x0 + w + 1), inp.Coord(-1, 5), inp.Coord(x0 - 1, x0 + h + 1));
                        try
                        {
                            a.SetDestination(dest);
                        }
                        catch (ArgumentException)
                        {
                        }
                        catch (InvalidOperationException)
                        {
                        }
                    }
                    break;
                case 3:
                    if (agents.Count > 0)
                    {
                        int i = inp.Byte() % agents.Count;
                        crowd.Remove(agents[i]);
                        agents.RemoveAt(i);
                    }
                    break;
                case 4:
                    if (agents.Count > 0)
                    {
                        Agent a = agents[inp.Byte() % agents.Count];
                        var pos = new Vector3D(inp.Coord(x0 - 1, x0 + w + 1), inp.Coord(-1, 5), inp.Coord(x0 - 1, x0 + h + 1));
                        try
                        {
                            a.Teleport(pos);
                        }
                        catch (ArgumentException)
                        {
                        }
                    }
                    break;
                case 5:
                    if (agents.Count > 0)
                    {
                        Agent a = agents[inp.Byte() % agents.Count];
                        double pr = inp.Byte() switch { 0 => 0, 1 => double.NaN, int v => v / 16.0 };
                        try
                        {
                            a.Priority = pr;
                        }
                        catch (ArgumentOutOfRangeException)
                        {
                        }
                    }
                    break;
                case 6:
                case 7:
                {
                    int n = 1 + inp.Byte() % 24;
                    int sel = inp.Byte();
                    double dt = sel == 255 ? (inp.Byte() < 128 ? 0.0 : double.NaN) : 1.0 / (10 + (sel % 50));
                    for (int s = 0; s < n; s++)
                    {
                        try
                        {
                            crowd.Step(dt);
                        }
                        catch (ArgumentOutOfRangeException) when (!(dt > 0 && double.IsFinite(dt)))
                        {
                            break;
                        }
                        Check(crowd, agents, q, edited);
                    }
                    break;
                }
                case 8:
                    NavMeshMutations.Apply(mesh, mut, data[3]);
                    edited = true;
                    break;
                default:
                    _ = CrowdDebug.Collect(crowd);
                    break;
            }
        }
        if (crowd.Count != agents.Count)
        {
            throw new InvalidOperationException("The crowd's count does not match the agents added and removed.");
        }
    }

    // After the mesh was edited an agent may stand where the mesh no longer is (until it is relocated), so only finiteness is checked then.
    private static void Check(Crowd crowd, List<Agent> agents, NavMeshQuery q, bool edited)
    {
        foreach (Agent a in agents)
        {
            if (!a.IsActive)
            {
                throw new InvalidOperationException("An agent the host still holds was deactivated.");
            }
            Vector3D p = a.Position, v = a.Velocity;
            if (!double.IsFinite(p.X) || !double.IsFinite(p.Y) || !double.IsFinite(p.Z) || !double.IsFinite(v.X) || !double.IsFinite(v.Z))
            {
                throw new InvalidOperationException($"Agent {a.Id} has a non-finite state.");
            }
            if (!edited && a.State != FollowerState.OffMeshLink && !CrowdWorlds.OnMesh(q, p))
            {
                throw new InvalidOperationException($"Agent {a.Id} left the mesh at {p}.");
            }
        }
        if (crowd.Count != agents.Count)
        {
            throw new InvalidOperationException("The crowd's count does not match.");
        }
    }
}
