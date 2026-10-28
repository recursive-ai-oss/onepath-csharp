using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.GridOptima;

/// <summary>
/// Checks voxel A* costs against the published optimal lengths of the Moving AI Lab voxel scenario sets.
/// Usage: <c>GridOptima --voxel --maps DIR --set NAME [--only A1,A2] [--threads N] [--limit N] [--per-map N] [--tolerance ABS] [--conn 6|18|26] [--rule cut|squeeze|all] [--verbose]</c>,
/// where DIR/NAME holds the loose <c>*.3dmap</c> and <c>*.3dmap.3dscen</c> files unpacked by <c>tools/maps/maps.sh</c> (never committed).
/// Prints one line per map and a summary line: scenarios, mismatches, largest absolute difference. Exit code 1 if any mismatch.
/// </summary>
internal static class VoxelOptima
{
    internal static int Run(string[] args)
    {
        string? dir = null;
        string? set = null;
        string[]? only = null;
        int threads = 2;
        long limit = long.MaxValue;
        int perMap = int.MaxValue;
        double tolerance = 1e-6;
        bool verbose = false;
        VoxelConnectivity conn = VoxelConnectivity.TwentySix;
        VoxelCornerRule corners = VoxelCornerRule.NoCornerCutting;
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == "--voxel")
            {
                continue;
            }
            if (args[i] == "--verbose")
            {
                verbose = true;
                continue;
            }
            if (i + 1 >= args.Length)
            {
                Console.Error.WriteLine($"missing value for {args[i]}");
                return 2;
            }
            string v = args[++i];
            switch (args[i - 1])
            {
                case "--maps": dir = v; break;
                case "--set": set = v; break;
                case "--only": only = v.Split(','); break;
                case "--threads": threads = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--limit": limit = long.Parse(v, CultureInfo.InvariantCulture); break;
                case "--per-map": perMap = int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--tolerance": tolerance = double.Parse(v, CultureInfo.InvariantCulture); break;
                case "--conn": conn = (VoxelConnectivity)int.Parse(v, CultureInfo.InvariantCulture); break;
                case "--rule":
                    corners = v switch
                    {
                        "cut" => VoxelCornerRule.NoCornerCutting,
                        "squeeze" => VoxelCornerRule.NoSqueezing,
                        "all" => VoxelCornerRule.AllowAll,
                        _ => throw new ArgumentException("--rule is cut, squeeze or all"),
                    };
                    break;
                default: Console.Error.WriteLine($"unknown option {args[i - 1]}"); return 2;
            }
        }
        var options = new VoxelOptions { Connectivity = conn, Corners = corners };
        if (dir is null || set is null)
        {
            Console.Error.WriteLine("usage: GridOptima --voxel --maps DIR --set NAME [--only A1,A2] [--threads N] [--limit N] [--per-map N] [--tolerance ABS] [--conn 6|18|26] [--rule cut|squeeze|all]");
            return 2;
        }
        string folder = Path.Combine(dir, set);
        var names = new List<string>();
        foreach (string f in Directory.GetFiles(folder, "*.3dmap"))
        {
            string n = Path.GetFileNameWithoutExtension(f);
            if ((only is null || Array.IndexOf(only, n) >= 0) && File.Exists(f + ".3dscen"))
            {
                names.Add(n);
            }
        }
        names.Sort(StringComparer.Ordinal);

        long total = 0;
        long mismatches = 0;
        double maxDelta = 0;
        long budget = limit;
        object sync = new();
        Parallel.ForEach(names, new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, threads) }, name =>
        {
            VoxelMap map = VoxelMap.Parse(File.ReadAllBytes(Path.Combine(folder, name + ".3dmap")));
            VoxelScenarioEntry[] entries = VoxelScenarios.Parse(File.ReadAllBytes(Path.Combine(folder, name + ".3dmap.3dscen")));
            VoxelGrid grid = map.ToGrid();
            long n = 0;
            long bad = 0;
            double worst = 0;
            foreach (VoxelScenarioEntry e in entries)
            {
                if (n >= perMap)
                {
                    break;
                }
                if (Interlocked.Decrement(ref budget) < 0)
                {
                    break;
                }
                n++;
                VoxelPath p = grid.FindPath(new VoxelCell(e.StartX, e.StartY, e.StartZ), new VoxelCell(e.GoalX, e.GoalY, e.GoalZ), options);
                double delta = p.Found ? Math.Abs(p.Cost - e.OptimalLength) : double.PositiveInfinity;
                if (verbose)
                {
                    Console.WriteLine($"{name} #{n}: expanded {p.ExpandedCount} cost {p.Cost:F6} published {e.OptimalLength:F6}");
                }
                if (!(delta <= tolerance))
                {
                    bad++;
                    if (bad <= 3)
                    {
                        Console.Error.WriteLine($"{name}: ({e.StartX},{e.StartY},{e.StartZ})->({e.GoalX},{e.GoalY},{e.GoalZ}) published {e.OptimalLength:F8} ours {p.Cost:F8} ({p.Status})");
                    }
                }
                else if (delta > worst)
                {
                    worst = delta;
                }
            }
            lock (sync)
            {
                total += n;
                mismatches += bad;
                maxDelta = Math.Max(maxDelta, worst);
                Console.WriteLine($"{name}: scenarios {n} mismatches {bad} max |diff| among matches {worst:E2}");
            }
        });
        Console.WriteLine($"voxel {set} conn {(int)options.Connectivity} rule {options.Corners} maps {names.Count} scenarios {total} mismatches {mismatches} max |diff| among matches {maxDelta:E2} tolerance {tolerance:E1}");
        return mismatches == 0 ? 0 : 1;
    }
}
