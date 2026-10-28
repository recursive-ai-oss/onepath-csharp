using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the crowd rows, with the metric names of the reference drivers.
/// <list type="bullet">
/// <item><c>circle</c> (against RVO2 and RVO2-CS): N agents spread on a circle walk to the antipodal point with no obstacles, the reference's
/// documented default agent settings (neighbour distance 15, 10 neighbours, horizons 10 and 5, radius 2, speed 2), dt 0.25; our side is the
/// ORCA solver itself (<see cref="OrcaSimulator"/>), one thread unless <c>--threads</c> says otherwise (<c>--threads auto</c> = every core).</item>
/// <item><c>crowd</c> (against DetourCrowd and DotRecast): N agents with the harness's start/goal rule on the point set, speed 3.5, acceleration 8,
/// dt 0.05; our side is <see cref="Crowd"/>, one thread. With an <c>--obj</c> level the mesh is our 3D builder's on the reference settings and the points are
/// used as they are, like the query rows; without one it is the format test generator's with the points scaled onto it (<c>mesh_source</c> says which).</item>
/// </list>
/// Quality rows ours prints in addition: <c>arrived</c>, <c>moving_at_end</c>, <c>worst_overlap</c> (deepest agent-agent penetration at the end, units),
/// <c>mean_overlap</c> over agent pairs closer than the sum of their radii.
/// </summary>
internal static class OursCrowd
{
    public static List<Row> Run(SuiteRow row, Paths paths)
    {
        string[] t = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        bool circle = t.Length > 0 && t[0] == "circle";
        // A native reference is fully optimized from its first instruction; our code starts in the JIT's unoptimized tier and is
        // promoted after a number of calls and a 100 ms quiet period, which a few dozen steps of a small crowd never reach. The
        // method is "warm, in-process": run the whole row once untimed, let the promotions finish, then run it for the numbers.
        if (Arg(t, "prewarm", "1") != "0")
        {
            // Repeat untimed runs (a pause after each so promotions finish) until two in a row agree within 3%: the JIT has settled.
            double previous = 0;
            for (int run = 0; run < 10; run++)
            {
                List<Row> rows = circle ? Circle(row, t) : City(row, t);
                System.Threading.Thread.Sleep(500);
                double rate = 0;
                foreach (Row r in rows)
                {
                    if (r.Metric == "agents_per_ms")
                    {
                        rate = double.Parse(r.Value, CultureInfo.InvariantCulture);
                    }
                }
                if (run >= 2 && previous > 0 && Math.Abs(rate - previous) <= 0.03 * previous)
                {
                    break;
                }
                previous = rate;
            }
        }
        return circle ? Circle(row, t) : City(row, t);
    }

    private static string Arg(string[] t, string name, string fallback)
    {
        for (int i = 0; i + 1 < t.Length; i++)
        {
            if (t[i] == "--" + name)
            {
                return t[i + 1];
            }
        }
        return fallback;
    }

    private static int Int(string[] t, string name, int fallback) => int.Parse(Arg(t, name, fallback.ToString(CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);

    // "--threads auto" runs on every core, the footing of a reference that parallelises over all of them by default (RVO2-CS).
    private static int Threads(string[] t) =>
        Arg(t, "threads", "1") == "auto" ? Environment.ProcessorCount : Int(t, "threads", 1);

    private static double Num(string[] t, string name, double fallback) => double.Parse(Arg(t, name, fallback.ToString("R", CultureInfo.InvariantCulture)), CultureInfo.InvariantCulture);

    private static Row R(SuiteRow row, string metric, double v, string unit) =>
        new(row.Part, row.Id, OurScenarios.Side, metric, v.ToString("R", CultureInfo.InvariantCulture), unit);

    private static double Ms(long start) => (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;

    private static List<Row> Circle(SuiteRow row, string[] t)
    {
        int n = Int(t, "agents", 250), steps = Int(t, "steps", 200), warm = Int(t, "warmup", 20), threads = Threads(t);
        double dt = Num(t, "dt", 0.25);
        double radiusOfCircle = 200.0 * Math.Sqrt(n / 250.0);
        var sim = new OrcaSimulator { MaxThreads = threads };
        var goals = new Vector2D[n];
        for (int i = 0; i < n; i++)
        {
            double ang = 2.0 * Math.PI * i / n;
            // The antipodal point is known in closed form on the circle: position and its negative.
            var pos = new Vector2D(radiusOfCircle * CosSin(ang, true), radiusOfCircle * CosSin(ang, false));
            goals[i] = new Vector2D(-pos.X, -pos.Y);
            sim.AddAgent(new OrcaAgent { Position = pos, Radius = 2, MaxSpeed = 2, NeighborDistance = 15, MaxNeighbors = 10, TimeHorizon = 10, ObstacleTimeHorizon = 5 });
        }
        void Step()
        {
            for (int i = 0; i < n; i++)
            {
                Vector2D d = goals[i] - sim[i].Position;
                if (Vector2D.Dot(d, d) > 1.0)
                {
                    d = d / d.Length();
                }
                sim[i].PreferredVelocity = d;
            }
            sim.Step(dt);
        }
        for (int i = 0; i < warm; i++)
        {
            Step();
        }
        long start = Stopwatch.GetTimestamp();
        for (int i = 0; i < steps; i++)
        {
            Step();
        }
        double ms = Ms(start);
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            sum += sim[i].Position.X + (2.0 * sim[i].Position.Y);
        }
        return new List<Row>
        {
            R(row, "agents", n, "count"),
            R(row, "steps", steps, "count"),
            R(row, "time", ms, "ms"),
            R(row, "agents_per_ms", n * (double)steps / ms, "agent-steps/ms"),
            R(row, "position_checksum", sum, "units"),
            R(row, "threads", threads, "count"),
            R(row, "peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
        };
    }

    // Library trigonometry is banned in the library, not in the bench; the benchmark may use Math.
    private static double CosSin(double a, bool cos) => cos ? Math.Cos(a) : Math.Sin(a);

    private static List<Row> City(SuiteRow row, string[] t)
    {
        string? pointsFile = Arg(t, "points", "");
        int n = Int(t, "agents", 100), steps = Int(t, "steps", 100), warm = Int(t, "warmup", 20), threads = Int(t, "threads", 1);
        double dt = Num(t, "dt", 0.05);
        if (!File.Exists(pointsFile))
        {
            return new List<Row> { new(row.Part, row.Id, OurScenarios.Side, "error", "missing --points file", "text") };
        }
        var raw = new List<Vector3D>();
        foreach (string line in File.ReadLines(pointsFile))
        {
            string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (f.Length >= 3 && f[0][0] != '#')
            {
                raw.Add(new Vector3D(double.Parse(f[0], CultureInfo.InvariantCulture), double.Parse(f[1], CultureInfo.InvariantCulture), double.Parse(f[2], CultureInfo.InvariantCulture)));
            }
        }
        double x0 = raw.Min(p => p.X), x1 = raw.Max(p => p.X), z0 = raw.Min(p => p.Z), z1 = raw.Max(p => p.Z);
        TiledNavMesh mesh;
        List<Vector3D> pts;
        string meshSource;
        string? objFile = Arg(t, "obj", "");
        if (File.Exists(objFile))
        {
            // The level's own mesh, built with the reference settings (solo unless the row says tiled), and the points as they are.
            bool tiled = Arg(t, "mode", "solo") == "tiled";
            int tileCells = Int(t, "tile-size", 64);
            (double[] ov, int[] oi) = OursBuild.LoadObj(objFile);
            mesh = OursBuild.Build(ov, oi, tiled, tileCells).Tiles;
            pts = raw;
            meshSource = "3D builder, " + OursBuild.Describe(tiled, tileCells);
        }
        else
        {
            mesh = NavMeshWorlds.RandomMesh(7, -4, -4, 8, 8, 1, detail: false, offMesh: 0);
            pts = raw.Select(p => new Vector3D(-47 + ((p.X - x0) / Math.Max(1e-9, x1 - x0) * 62), 0.2, -47 + ((p.Z - z0) / Math.Max(1e-9, z1 - z0) * 62))).ToList();
            meshSource = "format test generator (8x8 tiles); points scaled onto it";
        }
        var q = new NavMeshQuery(mesh);
        var usable = new List<NearestPoly>();
        foreach (Vector3D p in pts)
        {
            NearestPoly np = q.FindNearestPoly(p, new Vector3D(2, 4, 2));
            if (np.Poly.IsSet)
            {
                usable.Add(np);
            }
        }
        if (usable.Count < 2)
        {
            return new List<Row> { new(row.Part, row.Id, OurScenarios.Side, "error", "not enough usable points", "text") };
        }
        // The reference harness's agent: radius 0.6, speed 3.5, acceleration 8, query range 12 radii, separation on.
        AgentOptions opt = Agent.Humanoid with { SeparationDistance = 2.4, SeparationWeight = 0.3 };
        var crowd = new Crowd(mesh, new CrowdOptions { MaxThreads = threads, MaxAgents = Math.Max(n, 1) });
        var agents = new List<Agent>(n);
        for (int i = 0; i < n; i++)
        {
            NearestPoly st = usable[i % usable.Count];
            NearestPoly goal = usable[((i * 7) + (usable.Count / 2)) % usable.Count];
            try
            {
                Agent a = crowd.Add(st.Point, opt);
                a.SetDestination(goal.Point);
                agents.Add(a);
            }
            catch (ArgumentException)
            {
            }
        }
        for (int i = 0; i < warm; i++)
        {
            crowd.Step(dt);
        }
        long start = Stopwatch.GetTimestamp();
        for (int i = 0; i < steps; i++)
        {
            crowd.Step(dt);
        }
        double ms = Ms(start);
        int moving = 0, arrived = 0;
        foreach (Agent a in agents)
        {
            Vector3D v = a.Velocity;
            if (Math.Sqrt((v.X * v.X) + (v.Z * v.Z)) > 0.1)
            {
                moving++;
            }
            if (a.State == FollowerState.Arrived)
            {
                arrived++;
            }
        }
        double worst = 0, sumOverlap = 0;
        long overlapping = 0;
        if (agents.Count <= 2000)
        {
            for (int i = 0; i < agents.Count; i++)
            {
                for (int j = i + 1; j < agents.Count; j++)
                {
                    double dx = agents[i].Position.X - agents[j].Position.X, dz = agents[i].Position.Z - agents[j].Position.Z;
                    double ov = (2 * opt.Radius) - Math.Sqrt((dx * dx) + (dz * dz));
                    if (ov > 0 && Math.Abs(agents[i].Position.Y - agents[j].Position.Y) < 1.0)
                    {
                        worst = Math.Max(worst, ov);
                        sumOverlap += ov;
                        overlapping++;
                    }
                }
            }
        }
        var rows = new List<Row>
        {
            R(row, "agents", agents.Count, "count"),
            R(row, "steps", steps, "count"),
            R(row, "time", ms, "ms"),
            R(row, "agents_per_ms", agents.Count * (double)steps / ms, "agent-steps/ms"),
            R(row, "moving_at_end", moving, "count"),
            R(row, "arrived", arrived, "count"),
            R(row, "threads", threads, "count"),
            R(row, "peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
            new Row(row.Part, row.Id, OurScenarios.Side, "mesh_source", meshSource, "text"),
        };
        if (agents.Count <= 2000)
        {
            rows.Add(R(row, "worst_overlap", worst, "units"));
            rows.Add(R(row, "mean_overlap", overlapping > 0 ? sumOverlap / overlapping : 0, "units"));
        }
        return rows;
    }
}
