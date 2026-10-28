using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.GridOptima;

/// <summary>
/// Checks path costs (A*, JPS, JPS+, a flow field or HPA*) against the published optimal lengths of Moving AI Lab scenario sets.
/// Usage: <c>GridOptima --maps DIR --set NAME [--threads N] [--limit N] [--tolerance RELATIVE] [--alg astar|jps|jpsP|jps+|jpsP+|astar-generic|jps-generic|jps+-generic|flow|hpa] [--chunk N] [--smooth 0|1]</c> (hpa reports suboptimality instead of mismatches), where DIR holds
/// <c>NAME-map.zip</c> and <c>NAME-scen.zip</c> (as downloaded from the benchmark site; never committed) or a folder NAME of loose files.
/// Prints one summary line: scenarios, mismatches, maximum absolute difference. Exit code 1 if any mismatch.
/// <c>--replan lpa|dstar</c> instead checks incremental replanning: per map, <c>--per-map</c> scenarios; after each of <c>--rounds</c> random wall toggles
/// near the current path, the replanned cost must equal a fresh A* search (the scenario's published length is checked before the first toggle).
/// Options: <c>--swamp 0|1</c>, <c>--water 0|1</c> choose how S and W terrain is read.
/// </summary>
public static class Program
{
    public static int Main(string[] args)
    {
        int algAt = Array.IndexOf(args, "--alg");
        if (algAt >= 0 && algAt + 1 < args.Length && AnyAngleQuality.Wants(args[algAt + 1]))
        {
            return AnyAngleQuality.Run(args);
        }
        if (Array.IndexOf(args, "--voxel") >= 0)
        {
            return VoxelOptima.Run(args);
        }
        string? dir = null;
        string? set = null;
        int threads = 1;
        long limit = long.MaxValue;
        double tolerance = 1e-6; // relative: the published lengths carry single-precision-class error
        double altSqrt2 = 0;
        string alg = "astar";
        string? replan = null;
        int perMap = 20;
        int rounds = 10;
        int hpaChunk = 16;
        bool hpaSmooth = true;
        var options = new MovingAiOptions();
        for (int i = 0; i + 1 < args.Length; i += 2)
        {
            switch (args[i])
            {
                case "--maps": dir = args[i + 1]; break;
                case "--set": set = args[i + 1]; break;
                case "--threads": threads = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--limit": limit = long.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--tolerance": tolerance = double.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--alg": alg = args[i + 1]; break;
                case "--replan": replan = args[i + 1]; break;
                case "--per-map": perMap = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--rounds": rounds = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--chunk": hpaChunk = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--smooth": hpaSmooth = args[i + 1] != "0"; break;
                case "--alt-sqrt2": altSqrt2 = double.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--swamp": options.SwampWalkable = args[i + 1] == "1"; break;
                case "--water": options.WaterWalkable = args[i + 1] == "1"; break;
                default: Console.Error.WriteLine($"unknown option {args[i]}"); return 2;
            }
        }
        if (dir is null || set is null)
        {
            Console.Error.WriteLine("usage: GridOptima --maps DIR --set NAME [--threads N] [--limit N] [--tolerance RELATIVE] [--swamp 0|1] [--water 0|1] [--alt-sqrt2 X] [--alg astar|jps|jps+|flow|hpa] [--chunk N] [--smooth 0|1] [--replan lpa|dstar [--per-map N] [--rounds N]]  |  --alg theta|lazytheta (--random N | --maps DIR --set NAME), a quality report");
            return 2;
        }

        // Inputs come from NAME-map.zip and NAME-scen.zip in DIR, or from loose files in DIR/NAME.
        var mapFiles = new SortedDictionary<string, byte[]>(StringComparer.Ordinal);
        var scenFiles = new SortedDictionary<string, byte[]>(StringComparer.Ordinal);
        string mapZip = Path.Combine(dir, set + "-map.zip");
        if (File.Exists(mapZip))
        {
            using var maps = ZipFile.OpenRead(mapZip);
            foreach (ZipArchiveEntry e in maps.Entries)
            {
                if (e.Name.EndsWith(".map", StringComparison.Ordinal))
                {
                    mapFiles[e.Name] = Read(e);
                }
            }
            using var scens = ZipFile.OpenRead(Path.Combine(dir, set + "-scen.zip"));
            foreach (ZipArchiveEntry e in scens.Entries)
            {
                if (e.Name.EndsWith(".scen", StringComparison.Ordinal))
                {
                    scenFiles[e.Name] = Read(e);
                }
            }
        }
        else
        {
            foreach (string f in Directory.GetFiles(Path.Combine(dir, set), "*.map"))
            {
                mapFiles[Path.GetFileName(f)] = File.ReadAllBytes(f);
            }
            foreach (string f in Directory.GetFiles(Path.Combine(dir, set), "*.scen"))
            {
                scenFiles[Path.GetFileName(f)] = File.ReadAllBytes(f);
            }
        }
        var jobs = new List<(string Name, ScenarioEntry[] Entries, MovingAiMap Map)>();
        foreach (var (scenName, scenBytes) in scenFiles)
        {
            string mapName = scenName[..^5];
            if (!mapFiles.TryGetValue(mapName, out byte[]? mapBytes))
            {
                Console.Error.WriteLine($"no map for {scenName}");
                return 2;
            }
            jobs.Add((mapName, MovingAiScenarios.Parse(scenBytes), MovingAiMap.Parse(mapBytes)));
        }

        if (replan is not null)
        {
            return ReplanCheck.Run(set, replan, jobs, options, threads, perMap, rounds);
        }

        if (alg == "hpa")
        {
            return RunHpa(set, jobs, options, threads, limit, hpaChunk, hpaSmooth);
        }

        long total = 0;
        long mismatches = 0;
        double maxDelta = 0;
        double maxRel = 0;
        object sync = new();
        long budget = limit;
        var po = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, threads) };
        Parallel.ForEach(jobs, po, job =>
        {
            Grid grid = job.Map.ToGrid(options);
            Searcher search = Searcher.Create(alg, grid);
            long n = 0;
            long bad = 0;
            double worst = 0;
            double worstRel = 0;
            foreach (ScenarioEntry e in job.Entries)
            {
                if (Interlocked.Decrement(ref budget) < 0)
                {
                    break;
                }
                n++;
                double ours = double.PositiveInfinity;
                if (e.MapWidth == grid.Width && e.MapHeight == grid.Height && grid.Contains(e.StartX, e.StartY) && grid.Contains(e.GoalX, e.GoalY)
                    && grid.IsWalkable(e.StartX, e.StartY) && grid.IsWalkable(e.GoalX, e.GoalY))
                {
                    if (search.Run((e.StartY * grid.Width) + e.StartX, (e.GoalY * grid.Width) + e.GoalX))
                    {
                        ours = search.PathCost;
                        if (altSqrt2 > 0)
                        {
                            // Re-price the found path with a different diagonal constant (straight steps cost 1).
                            int[] p = new int[search.PathLength()];
                            search.CopyPath(p);
                            long diag = 0;
                            for (int k = 1; k < p.Length; k++)
                            {
                                if (((p[k] % grid.Width) != (p[k - 1] % grid.Width)) && ((p[k] / grid.Width) != (p[k - 1] / grid.Width)))
                                {
                                    diag++;
                                }
                            }
                            ours = (p.Length - 1 - diag) + (diag * altSqrt2);
                        }
                    }
                }
                double d = Math.Abs(ours - e.OptimalLength);
                if (!(d <= (tolerance * Math.Max(1.0, e.OptimalLength))))
                {
                    bad++;
                    if (bad <= 3)
                    {
                        Console.Error.WriteLine($"MISMATCH {job.Name} ({e.StartX},{e.StartY})->({e.GoalX},{e.GoalY}) published {e.OptimalLength.ToString("R", CultureInfo.InvariantCulture)} ours {ours.ToString("R", CultureInfo.InvariantCulture)}");
                    }
                    d = double.IsFinite(d) ? d : double.MaxValue;
                }
                if (d > worst)
                {
                    worst = d;
                }
                if (d / Math.Max(1.0, e.OptimalLength) > worstRel)
                {
                    worstRel = d / Math.Max(1.0, e.OptimalLength);
                }
            }
            lock (sync)
            {
                total += n;
                mismatches += bad;
                if (worst > maxDelta)
                {
                    maxDelta = worst;
                }
                if (worstRel > maxRel)
                {
                    maxRel = worstRel;
                }
            }
        });
        Console.WriteLine($"{set}: maps {jobs.Count}, scenarios {total}, mismatches {mismatches}, max |delta| {maxDelta.ToString("E3", CultureInfo.InvariantCulture)}, max |delta|/max(1,length) {maxRel.ToString("E3", CultureInfo.InvariantCulture)}");
        return mismatches == 0 ? 0 : 1;
    }

    // HPA* is near optimal, so the published lengths are a yardstick, not an equality: every refined path must be valid
    // (each step a legal move, the cost its own sum) and found whenever the scenario has a path; the report gives the mean
    // and maximum percentage over the optimal length, as the HPA* paper does.
    private static int RunHpa(string set, List<(string Name, ScenarioEntry[] Entries, MovingAiMap Map)> jobs, MovingAiOptions options, int threads, long limit, int chunk, bool smooth)
    {
        long total = 0;
        long bad = 0;
        long counted = 0;
        double sumPct = 0;
        double maxPct = 0;
        double sumNodes = 0;
        object sync = new();
        long budget = limit;
        var po = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, threads) };
        Parallel.ForEach(jobs, po, job =>
        {
            Grid grid = job.Map.ToGrid(options);
            using var hierarchy = new HpaHierarchy(grid, new HpaOptions { ChunkSize = chunk });
            var search = new HpaSearch(hierarchy) { Smoothing = smooth };
            SquareTopology topology = grid.CreateTopology();
            long n = 0;
            long invalid = 0;
            long c = 0;
            double pct = 0;
            double worst = 0;
            double nodes = 0;
            foreach (ScenarioEntry e in job.Entries)
            {
                if (Interlocked.Decrement(ref budget) < 0)
                {
                    break;
                }
                n++;
                if (!(e.MapWidth == grid.Width && e.MapHeight == grid.Height && grid.Contains(e.StartX, e.StartY) && grid.Contains(e.GoalX, e.GoalY)
                    && grid.IsWalkable(e.StartX, e.StartY) && grid.IsWalkable(e.GoalX, e.GoalY)))
                {
                    invalid++;
                    continue;
                }
                search.Begin((e.StartY * grid.Width) + e.StartX, (e.GoalY * grid.Width) + e.GoalX);
                if (search.Run() != SearchStatus.Found || !PathIsValid(topology, search))
                {
                    invalid++;
                    if (invalid <= 3)
                    {
                        Console.Error.WriteLine($"INVALID {job.Name} ({e.StartX},{e.StartY})->({e.GoalX},{e.GoalY})");
                    }
                    continue;
                }
                double over = e.OptimalLength > 0 ? ((search.PathCost - e.OptimalLength) / e.OptimalLength) * 100.0 : 0.0;
                if (over < -1e-4)
                {
                    invalid++; // shorter than the published optimum: the path or the published length is wrong
                    continue;
                }
                c++;
                pct += over;
                worst = Math.Max(worst, over);
                nodes += search.ExpandedCount;
            }
            lock (sync)
            {
                total += n;
                bad += invalid;
                counted += c;
                sumPct += pct;
                maxPct = Math.Max(maxPct, worst);
                sumNodes += nodes;
            }
        });
        string f = "F3";
        Console.WriteLine($"{set} hpa chunk {chunk} smoothing {(smooth ? 1 : 0)}: maps {jobs.Count}, scenarios {total}, invalid-or-missing {bad}, mean over optimal {(counted > 0 ? sumPct / counted : 0).ToString(f, CultureInfo.InvariantCulture)}%, max over optimal {maxPct.ToString(f, CultureInfo.InvariantCulture)}%, mean abstract expansions {(counted > 0 ? sumNodes / counted : 0).ToString("F1", CultureInfo.InvariantCulture)}");
        return bad == 0 ? 0 : 1;
    }

    private static bool PathIsValid(SquareTopology topology, HpaSearch search)
    {
        int[] p = new int[search.PathLength()];
        search.CopyPath(p);
        double sum = 0;
        for (int i = 1; i < p.Length; i++)
        {
            var finder = new StepFinder { Target = p[i], Cost = double.NaN };
            topology.VisitNeighbors(p[i - 1], ref finder);
            if (double.IsNaN(finder.Cost))
            {
                return false;
            }
            sum += finder.Cost;
        }
        return Math.Abs(sum - search.PathCost) <= 1e-6;
    }

    private struct StepFinder : INeighborVisitor
    {
        public int Target;
        public double Cost;

        public void Visit(int neighbor, double baseCost)
        {
            if (neighbor == Target)
            {
                Cost = baseCost;
            }
        }
    }

    /// <summary>One search per worker thread, whichever algorithm was asked for.</summary>
    private sealed class Searcher
    {
        public Func<int, int, bool> Run = null!;
        public Func<double> Cost = null!;
        public Func<int[]> Path = null!;

        public double PathCost => Cost();

        public int PathLength() => Path().Length;

        public void CopyPath(int[] destination) => Path().CopyTo(destination, 0);

        private static Searcher Fast<TEngine>(TEngine engine, Func<int, int, SearchStatus> run)
            where TEngine : class, IOctileEngine => new()
            {
                Run = (s, g) => run(s, g) == SearchStatus.Found,
                Cost = () => engine.PathCost,
                Path = () => { int[] p = new int[engine.PathLength()]; engine.CopyPath(p); return p; },
            };

        public static Searcher Create(string alg, Grid grid)
        {
            var buffers = new GridSearchBuffers(grid.CellCount);
            var heuristic = new OctileHeuristic(grid.Width);
            switch (alg)
            {
                case "astar":
                {
                    var a = new OctileAStar(grid);
                    return Fast(a, (s, g) => a.Run(s, g, false, int.MaxValue, false));
                }
                case "jps" or "jpsP":
                {
                    var j = new OctileJump<BitFinder>(new BitFinder(new OctileMap(grid), alg == "jpsP"), grid.Width, grid.Height);
                    return Fast(j, (s, g) => j.Run(s, g, false, int.MaxValue, false));
                }
                case "jps+" or "jpsP+":
                {
                    var j = new OctileJump<TableFinder>(new TableFinder(new JpsPlusIndex(grid), new OctileMap(grid), alg == "jpsP+"), grid.Width, grid.Height);
                    return Fast(j, (s, g) => j.Run(s, g, false, int.MaxValue, false));
                }
                case "astar-generic":
                {
                    var a = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(grid.CreateTopology(), default, heuristic, buffers);
                    return new Searcher
                    {
                        Run = (s, g) => { a.Begin(s, g); return a.Run() == SearchStatus.Found; },
                        Cost = () => a.PathCost,
                        Path = () => { int[] p = new int[a.PathLength()]; a.CopyPath(p); return p; },
                    };
                }
                case "jps-generic":
                {
                    var j = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(new BlockMap(grid)), heuristic, 1.0, buffers);
                    return new Searcher
                    {
                        Run = (s, g) => { j.Begin(s, g); return j.Run() == SearchStatus.Found; },
                        Cost = () => j.PathCost,
                        Path = () => { int[] p = new int[j.PathLength()]; j.CopyPath(p); return p; },
                    };
                }
                case "jps+-generic":
                {
                    var index = new JpsPlusIndex(grid);
                    var j = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), heuristic, 1.0, buffers);
                    return new Searcher
                    {
                        Run = (s, g) => { j.Begin(s, g); return j.Run() == SearchStatus.Found; },
                        Cost = () => j.PathCost,
                        Path = () => { int[] p = new int[j.PathLength()]; j.CopyPath(p); return p; },
                    };
                }
                case "flow":
                {
                    // Flow field to the scenario's goal: the integration cost at the start must equal the published optimum.
                    var f = new FlowField<SquareTopology, DefaultStepCost>(grid.CreateTopology(), default);
                    int start = 0;
                    return new Searcher
                    {
                        Run = (s, g) => { start = s; f.Begin([g]); f.Run(); return !double.IsPositiveInfinity(f.Cost(s)); },
                        Cost = () => f.Cost(start),
                        Path = () => throw new NotSupportedException("--alt-sqrt2 needs a path-returning algorithm"),
                    };
                }
                default:
                    throw new ArgumentException($"unknown algorithm '{alg}' (astar, jps, jpsP, jps+, jpsP+, the -generic forms, or flow)");
            }
        }
    }

    private static byte[] Read(ZipArchiveEntry e)
    {
        using Stream s = e.Open();
        using var ms = new MemoryStream(checked((int)e.Length));
        s.CopyTo(ms);
        return ms.ToArray();
    }
}
