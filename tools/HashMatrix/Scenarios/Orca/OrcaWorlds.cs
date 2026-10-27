using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Paper-style ORCA scenarios shared by the hash-matrix scenarios, the tests and the benchmarks: agents that walk to goals at a
/// preferred speed. Only IEEE-exact arithmetic is used to lay the agents out, so every platform builds the same start state.
/// </summary>
internal sealed class OrcaWorld
{
    public const double TimeStep = 0.1;
    public const double Radius = 0.5;
    public const double Speed = 1.0;
    public const double ArriveDistance = 0.1;

    public OrcaWorld(OrcaSimulator sim, Vector2D[] goals)
    {
        Sim = sim;
        Goals = goals;
    }

    private ulong _tick;

    /// <summary>When set, agents left of <see cref="GateX"/> first aim at the point just past the gate on the x axis (the preferred velocity is the caller's route choice).</summary>
    public Vector2D? Gate { get; init; }

    public double GateX { get; init; }

    public OrcaSimulator Sim { get; }

    public Vector2D[] Goals { get; }

    public static OrcaAgent DefaultAgent(Vector2D position) => new()
    {
        Position = position,
        Radius = Radius,
        MaxSpeed = 1.5 * Speed,
        NeighborDistance = 12,
        MaxNeighbors = 10,
        TimeHorizon = 5,
        ObstacleTimeHorizon = 2,
    };

    /// <summary>Points the preferred velocity of each agent at its goal (zero once within <see cref="ArriveDistance"/>).</summary>
    public void SetPreferredVelocities()
    {
        for (int i = 0; i < Sim.Count; i++)
        {
            ref OrcaAgent a = ref Sim[i];
            Vector2D target = Gate is Vector2D gate && a.Position.X < GateX ? gate : Goals[i];
            Vector2D d = target - a.Position;
            double len = d.Length();
            Vector2D pref = len <= ArriveDistance ? Vector2D.Zero : (len > Speed ? d / len * Speed : d);
            // A tiny pseudo-random nudge breaks exact symmetry (two agents meeting exactly head on would otherwise stop), as the
            // paper's authors' demos do; it is a function of (agent, step) only.
            ulong h = Mix(((ulong)i << 32) ^ _tick);
            double nx = ((h >> 11) * (1.0 / 9007199254740992.0)) - 0.5;
            double ny = (((h * 0x9E3779B97F4A7C15UL) >> 11) * (1.0 / 9007199254740992.0)) - 0.5;
            a.PreferredVelocity = pref + (new Vector2D(nx, ny) * 0.01);
        }
        _tick++;
    }

    private static ulong Mix(ulong z)
    {
        z += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    public void Step()
    {
        SetPreferredVelocities();
        Sim.Step(TimeStep);
    }

    public int ArrivedCount()
    {
        int count = 0;
        for (int i = 0; i < Sim.Count; i++)
        {
            if ((Goals[i] - Sim[i].Position).Length() <= 4 * ArriveDistance)
            {
                count++;
            }
        }
        return count;
    }

    /// <summary>The deepest overlap (sum of radii minus distance, positive when overlapping) over all agent pairs, by a sort-and-sweep on x.</summary>
    public double MaxPenetration()
    {
        int n = Sim.Count;
        var order = new int[n];
        for (int i = 0; i < n; i++)
        {
            order[i] = i;
        }
        Array.Sort(order, (p, q) => Sim[p].Position.X.CompareTo(Sim[q].Position.X));
        double worst = double.NegativeInfinity;
        for (int ii = 0; ii < n; ii++)
        {
            ref OrcaAgent a = ref Sim[order[ii]];
            for (int jj = ii + 1; jj < n; jj++)
            {
                ref OrcaAgent b = ref Sim[order[jj]];
                double reach = a.Radius + b.Radius;
                if (b.Position.X - a.Position.X > reach)
                {
                    break;
                }
                worst = Math.Max(worst, reach - (b.Position - a.Position).Length());
            }
        }
        return worst;
    }

    /// <summary>Agents evenly spread on a circle, each walking to the antipodal point.</summary>
    public static OrcaWorld Circle(int n, int threads = 1)
    {
        var sim = new OrcaSimulator { MaxThreads = threads };
        var goals = new Vector2D[n];
        double ring = (0.18 * n) + 6;
        // Rotate by a fixed small angle with the rational parametrisation (1 - t^2, 2t) / (1 + t^2); no trigonometry.
        double t = 3.14159265358979 / n;
        double cs = (1 - (t * t)) / (1 + (t * t));
        double sn = 2 * t / (1 + (t * t));
        double x = ring, y = 0;
        for (int i = 0; i < n; i++)
        {
            sim.AddAgent(DefaultAgent(new Vector2D(x, y)));
            double nx = (x * cs) - (y * sn);
            double ny = (x * sn) + (y * cs);
            x = nx;
            y = ny;
        }
        // The ring does not close exactly, so the goal is the position of the agent half way round.
        for (int i = 0; i < n; i++)
        {
            goals[i] = sim[(i + (n / 2)) % n].Position;
        }
        return new OrcaWorld(sim, goals);
    }

    /// <summary>Two square blocks of m x m agents whose paths cross at right angles.</summary>
    public static OrcaWorld Crossing(int m, int threads = 1)
    {
        var sim = new OrcaSimulator { MaxThreads = threads };
        var goals = new List<Vector2D>();
        double pitch = 1.3;
        double far = (m * pitch) + 8;
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < m; j++)
            {
                double u = (i - ((m - 1) / 2.0)) * pitch;
                double v = (j - ((m - 1) / 2.0)) * pitch;
                sim.AddAgent(DefaultAgent(new Vector2D(-far + u, v)));
                goals.Add(new Vector2D(far + u, v));
                sim.AddAgent(DefaultAgent(new Vector2D(u, -far + v)));
                goals.Add(new Vector2D(u, far + v));
            }
        }
        return new OrcaWorld(sim, goals.ToArray());
    }

    /// <summary>A wall across the middle with a 3-unit gap; agents in two columns walk through it from the left side to the right side.</summary>
    public static OrcaWorld Gap(int n, int threads = 1)
    {
        var sim = new OrcaSimulator { MaxThreads = threads };
        sim.AddObstacle(new OrcaSegment(new Vector2D(0, -30), new Vector2D(0, -1.5)));
        sim.AddObstacle(new OrcaSegment(new Vector2D(0, 1.5), new Vector2D(0, 30)));
        var goals = new Vector2D[n];
        for (int i = 0; i < n; i++)
        {
            int col = i % 4;
            int row = i / 4;
            double y = ((row % 8) - 3.5) * 1.2;
            sim.AddAgent(DefaultAgent(new Vector2D(-6 - (col * 1.3) - ((row / 8) * 5.5), y)));
            goals[i] = new Vector2D(-sim[i].Position.X, y);
        }
        return new OrcaWorld(sim, goals) { Gate = new Vector2D(1.5, 0), GateX = 0.5 };
    }

    /// <summary>A corridor of two long walls; one group walks each way.</summary>
    public static OrcaWorld Corridor(int n, int threads = 1)
    {
        var sim = new OrcaSimulator { MaxThreads = threads };
        sim.AddObstacle(new OrcaSegment(new Vector2D(-40, 5), new Vector2D(40, 5)));
        sim.AddObstacle(new OrcaSegment(new Vector2D(-40, -5), new Vector2D(40, -5)));
        var goals = new Vector2D[n];
        for (int i = 0; i < n; i++)
        {
            double y = (((i / 2) % 7) - 3) * 1.25;
            double x = ((i / 14) * 1.4) + 8;
            if (i % 2 == 0)
            {
                sim.AddAgent(DefaultAgent(new Vector2D(-x, y)));
                goals[i] = new Vector2D(x, y);
            }
            else
            {
                sim.AddAgent(DefaultAgent(new Vector2D(x, y + 0.6)));
                goals[i] = new Vector2D(-x, y + 0.6);
            }
        }
        return new OrcaWorld(sim, goals);
    }
}
