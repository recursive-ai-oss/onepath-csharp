using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Compression;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Tests.Grids;

namespace RecursiveAI.OnePath.GridOptima;

/// <summary>
/// Quality report for the any-angle searches (<c>--alg theta|lazytheta</c>), which have no published optimal costs to match.
/// On Moving AI scenarios (<c>--maps DIR --set NAME</c>): the ratio of the any-angle length to the published eight-connected optimum
/// (must be at most 1), every 64th path re-checked segment by segment with the independent exact visibility test, and, on
/// deterministic 40 x 40 windows cut from each map, the ratio to the visibility-graph optimum (shortest path over all
/// line-of-sight links between cell centres). With <c>--random N</c>: the same ratios on N random 24 x 24 grids.
/// </summary>
internal static class AnyAngleQuality
{
    internal static bool Wants(string alg) => alg is "theta" or "lazytheta";

    internal static int Run(string[] args)
    {
        string? dir = null;
        string? set = null;
        string alg = "theta";
        int threads = 1;
        long limit = long.MaxValue;
        int random = 0;
        for (int i = 0; i + 1 < args.Length; i += 2)
        {
            switch (args[i])
            {
                case "--maps": dir = args[i + 1]; break;
                case "--set": set = args[i + 1]; break;
                case "--alg": alg = args[i + 1]; break;
                case "--threads": threads = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--limit": limit = long.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--random": random = int.Parse(args[i + 1], CultureInfo.InvariantCulture); break;
                case "--tolerance": break;
                case "--swamp": break;
                case "--water": break;
                default: Console.Error.WriteLine($"unknown option {args[i]}"); return 2;
            }
        }
        var options = new GridOptions { Algorithm = alg == "theta" ? GridAlgorithm.ThetaStar : GridAlgorithm.LazyThetaStar };
        var windowRatios = new Stats();
        int status = 0;
        if (random > 0)
        {
            RandomGrids(random, options, windowRatios);
            Console.WriteLine($"random {random} grids ({alg}): {windowRatios.Describe("to visibility graph")}");
            return windowRatios.Violations == 0 ? 0 : 1;
        }
        if (dir is null || set is null)
        {
            Console.Error.WriteLine("usage: GridOptima --alg theta|lazytheta (--random N | --maps DIR --set NAME [--threads N] [--limit N])");
            return 2;
        }
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
            if (!mapFiles.TryGetValue(scenName[..^5], out byte[]? mapBytes))
            {
                Console.Error.WriteLine($"no map for {scenName}");
                return 2;
            }
            jobs.Add((scenName[..^5], MovingAiScenarios.Parse(scenBytes), MovingAiMap.Parse(mapBytes)));
        }

        var vsOptimal = new Stats();
        long invalid = 0;
        long checkedPaths = 0;
        long budget = limit;
        object sync = new();
        Parallel.ForEach(jobs, new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, threads) }, job =>
        {
            Grid grid = job.Map.ToGrid();
            var local = new Stats();
            var win = new Stats();
            long bad = 0;
            long verified = 0;
            long index = 0;
            foreach (ScenarioEntry e in job.Entries)
            {
                if (Interlocked.Decrement(ref budget) < 0)
                {
                    break;
                }
                index++;
                if (e.MapWidth != grid.Width || e.MapHeight != grid.Height || !grid.IsWalkable(e.StartX, e.StartY) || !grid.IsWalkable(e.GoalX, e.GoalY))
                {
                    continue;
                }
                GridPath p = grid.FindPath(new Cell(e.StartX, e.StartY), new Cell(e.GoalX, e.GoalY), options);
                if (!p.Found)
                {
                    bad++;
                    continue;
                }
                if (e.OptimalLength > 0)
                {
                    local.Add(p.Cost / e.OptimalLength, 1 + 1e-6);
                }
                if (index % 64 == 1)
                {
                    verified++;
                    ReadOnlySpan<Cell> c = p.Cells;
                    for (int k = 1; k < c.Length; k++)
                    {
                        if (!AnyAngleOracle.Visible(grid, c[k - 1], c[k]))
                        {
                            bad++;
                        }
                    }
                }
            }
            Windows(grid, job.Name, options, win);
            lock (sync)
            {
                vsOptimal.Merge(local);
                windowRatios.Merge(win);
                invalid += bad;
                checkedPaths += verified;
            }
        });
        Console.WriteLine($"{set} ({alg}): vs published 8-connected optimum: {vsOptimal.Describe("to octile optimum")}; paths re-verified {checkedPaths}, invalid {invalid}");
        Console.WriteLine($"{set} ({alg}): 40x40 windows: {windowRatios.Describe("to visibility graph")}");
        status = invalid == 0 && vsOptimal.Violations == 0 && windowRatios.Violations == 0 ? 0 : 1;
        return status;
    }

    // Eight deterministic windows per map, five random walkable pairs each, against the visibility-graph optimum.
    private static void Windows(Grid grid, string name, GridOptions options, Stats stats)
    {
        ulong state = 0x9E3779B97F4A7C15UL;
        foreach (char ch in name)
        {
            state = (state ^ ch) * 0x100000001B3UL;
        }
        int Next(int n)
        {
            state = (state * 6364136223846793005UL) + 1442695040888963407UL;
            return (int)((state >> 33) % (ulong)n);
        }
        const int W = 40;
        if (grid.Width < W || grid.Height < W)
        {
            return;
        }
        for (int w = 0; w < 8; w++)
        {
            int ox = Next(grid.Width - W + 1);
            int oy = Next(grid.Height - W + 1);
            var crop = new Grid(W, W);
            var free = new List<Cell>();
            for (int y = 0; y < W; y++)
            {
                for (int x = 0; x < W; x++)
                {
                    if (grid.IsWalkable(ox + x, oy + y))
                    {
                        free.Add(new Cell(x, y));
                    }
                    else
                    {
                        crop.SetArea(x, y, AreaCosts.Wall);
                    }
                }
            }
            if (free.Count < 2)
            {
                continue;
            }
            for (int q = 0; q < 5; q++)
            {
                Cell a = free[Next(free.Count)];
                Cell b = free[Next(free.Count)];
                Compare(crop, a, b, options, stats);
            }
        }
    }

    private static void RandomGrids(int count, GridOptions options, Stats stats)
    {
        var rng = new SeededRandom(0xA9E);
        for (int i = 0; i < count; i++)
        {
            var g = new Grid(24, 24);
            double density = rng.NextDouble() * 0.4;
            var free = new List<Cell>();
            for (int y = 0; y < 24; y++)
            {
                for (int x = 0; x < 24; x++)
                {
                    if (rng.NextDouble() < density)
                    {
                        g.SetArea(x, y, AreaCosts.Wall);
                    }
                    else
                    {
                        free.Add(new Cell(x, y));
                    }
                }
            }
            if (free.Count < 2)
            {
                continue;
            }
            for (int q = 0; q < 3; q++)
            {
                Compare(g, free[rng.NextInt(free.Count)], free[rng.NextInt(free.Count)], options, stats);
            }
        }
    }

    private static void Compare(Grid g, Cell a, Cell b, GridOptions options, Stats stats)
    {
        double vis = AnyAngleOracle.VisibilityGraphDistance(g, a, b);
        GridPath p = g.FindPath(a, b, options);
        if (double.IsPositiveInfinity(vis))
        {
            if (p.Found)
            {
                stats.Violations++;
            }
            return;
        }
        if (!p.Found)
        {
            stats.Violations++;
            return;
        }
        if (vis > 0)
        {
            stats.AddLower(p.Cost / vis);
        }
    }

    private static byte[] Read(ZipArchiveEntry e)
    {
        using Stream s = e.Open();
        using var ms = new MemoryStream(checked((int)e.Length));
        s.CopyTo(ms);
        return ms.ToArray();
    }

    private sealed class Stats
    {
        public long Count;
        public double Sum;
        public double Max;
        public long Over; // ratios above the allowed bound
        public long Violations;

        public void Add(double ratio, double allowed)
        {
            Count++;
            Sum += ratio;
            Max = Math.Max(Max, ratio);
            if (ratio > allowed)
            {
                Over++;
                Violations++;
            }
        }

        // Ratio to the optimum: must be at least 1 (a shorter path than the optimum would be invalid).
        public void AddLower(double ratio)
        {
            Count++;
            Sum += ratio;
            Max = Math.Max(Max, ratio);
            if (ratio < 1 - 1e-9)
            {
                Violations++;
            }
        }

        public void Merge(Stats o)
        {
            Count += o.Count;
            Sum += o.Sum;
            Max = Math.Max(Max, o.Max);
            Over += o.Over;
            Violations += o.Violations;
        }

        public string Describe(string what) =>
            $"{Count} queries, mean ratio {what} {(Count == 0 ? 0 : Sum / Count).ToString("F5", CultureInfo.InvariantCulture)}, max {Max.ToString("F5", CultureInfo.InvariantCulture)}, violations {Violations}";
    }
}
