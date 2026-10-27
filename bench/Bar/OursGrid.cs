using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the grid rows: A*, JPS (block scanning) and JPS+ (8-connected, no corner cutting) over every instance of a
/// Moving AI scenario file, reporting the metrics the reference grid drivers print (see docs/benchmarking.md). As in the
/// reference drivers, the map is loaded and (for JPS+) the jump tables are built outside <c>search_time</c>; the table build is
/// reported as <c>preprocess_time</c>. <c>--alg jpsP</c> and <c>jpsP+</c> (improved pruning) return null: nothing of ours yet.
/// </summary>
internal static class GridBar
{
    public static List<Row>? Run(SuiteRow row, Paths paths)
    {
        string[] a = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        string? alg = null;
        string? scen = null;
        for (int i = 0; i + 1 < a.Length; i++)
        {
            if (a[i] == "--alg")
            {
                alg = a[i + 1];
            }
            else if (a[i] == "--scen")
            {
                scen = a[i + 1];
            }
        }
        if (alg is not ("astar" or "jps" or "jps+") || scen is null || !scen.EndsWith(".scen", StringComparison.Ordinal))
        {
            return null;
        }

        // Loading is outside the timed section, as for the reference drivers.
        Grid grid = MovingAiMap.Parse(File.ReadAllBytes(scen[..^5])).ToGrid();
        ScenarioEntry[] entries = MovingAiScenarios.Parse(File.ReadAllBytes(scen));
        var buffers = new GridSearchBuffers(grid.CellCount);
        var heuristic = new OctileHeuristic(grid.Width);
        Func<int, int, SearchStatus> run;
        Func<double> cost;
        Func<int> expandedCount;
        long preTicks = 0;
        switch (alg)
        {
            case "jps":
            {
                var map = new BlockMap(grid);
                var js = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(map), heuristic, 1.0, buffers);
                run = (from, to) => { js.Begin(from, to); return js.Run(); };
                cost = () => js.PathCost;
                expandedCount = () => js.ExpandedCount;
                break;
            }
            case "jps+":
            {
                long p0 = Stopwatch.GetTimestamp();
                var index = new JpsPlusIndex(grid);
                preTicks = Stopwatch.GetTimestamp() - p0;
                var js = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), heuristic, 1.0, buffers);
                run = (from, to) => { js.Begin(from, to); return js.Run(); };
                cost = () => js.PathCost;
                expandedCount = () => js.ExpandedCount;
                break;
            }
            default:
            {
                var astar = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(grid.CreateTopology(), default, heuristic, buffers);
                run = (from, to) => { astar.Begin(from, to); return astar.Run(); };
                cost = () => astar.PathCost;
                expandedCount = () => astar.ExpandedCount;
                break;
            }
        }

        double costSum = 0;
        long expanded = 0;
        int mismatches = 0;
        long ticks = 0;
        for (int i = 0; i < entries.Length; i++)
        {
            ScenarioEntry e = entries[i];
            int s = (e.StartY * grid.Width) + e.StartX;
            int g = (e.GoalY * grid.Width) + e.GoalX;
            long t0 = Stopwatch.GetTimestamp();
            SearchStatus st = run(s, g);
            ticks += Stopwatch.GetTimestamp() - t0;
            expanded += expandedCount();
            double c = st == SearchStatus.Found ? cost() : double.PositiveInfinity;
            costSum += c;
            if (!(Math.Abs(c - e.OptimalLength) <= 1e-6 * Math.Max(1.0, e.OptimalLength)))
            {
                mismatches++;
            }
        }
        double ms = ticks * 1000.0 / Stopwatch.Frequency;
        string scenario = row.Id;
        string side = OurScenarios.Side;
        return
        [
            new Row("grid", scenario, side, "instances", entries.Length.ToString(CultureInfo.InvariantCulture), "count"),
            new Row("grid", scenario, side, "search_time", ms.ToString("R", CultureInfo.InvariantCulture), "ms"),
            new Row("grid", scenario, side, "preprocess_time", (preTicks * 1000.0 / Stopwatch.Frequency).ToString("R", CultureInfo.InvariantCulture), "ms"),
            new Row("grid", scenario, side, "expanded", expanded.ToString(CultureInfo.InvariantCulture), "count"),
            new Row("grid", scenario, side, "cost_sum", costSum.ToString("R", CultureInfo.InvariantCulture), "cost"),
            new Row("grid", scenario, side, "cost_mismatch_vs_scenario", mismatches.ToString(CultureInfo.InvariantCulture), "count"),
        ];
    }
}
