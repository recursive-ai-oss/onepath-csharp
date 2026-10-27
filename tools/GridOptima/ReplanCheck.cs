using System;
using System.Collections.Generic;
using System.Globalization;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.GridOptima;

/// <summary>
/// Incremental replanning on Moving AI maps: LPA* or D* Lite against a fresh A* after random wall toggles along the current path.
/// Reports mismatches and the nodes expanded by repairs against the nodes expanded by searching again.
/// </summary>
internal static class ReplanCheck
{
    internal static int Run(string set, string kind, List<(string Name, ScenarioEntry[] Entries, MovingAiMap Map)> jobs, MovingAiOptions options, int threads, int perMap, int rounds)
    {
        GridAlgorithm alg = kind switch
        {
            "lpa" => GridAlgorithm.LpaStar,
            "dstar" => GridAlgorithm.DStarLite,
            _ => throw new ArgumentException($"unknown replanner '{kind}' (lpa or dstar)"),
        };
        long checks = 0;
        long mismatches = 0;
        long repairExpanded = 0;
        long freshExpanded = 0;
        long firstExpanded = 0;
        object sync = new();
        var po = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, threads) };
        Parallel.ForEach(jobs, po, job =>
        {
            Grid grid = job.Map.ToGrid(options);
            var rng = new SeededRandom(0x5EED0000UL + (ulong)job.Entries.Length);
            int step = Math.Max(1, job.Entries.Length / Math.Max(1, perMap));
            long c = 0, bad = 0, rep = 0, fresh = 0, first = 0;
            for (int idx = 0; idx < job.Entries.Length; idx += step)
            {
                ScenarioEntry e = job.Entries[idx];
                if (e.MapWidth != grid.Width || e.MapHeight != grid.Height || !grid.IsWalkable(e.StartX, e.StartY) || !grid.IsWalkable(e.GoalX, e.GoalY))
                {
                    continue;
                }
                var start = new Cell(e.StartX, e.StartY);
                var goal = new Cell(e.GoalX, e.GoalY);
                var toggled = new List<(Cell Cell, byte Old)>();
                using GridReplanner r = grid.CreateReplanner(start, goal, new GridOptions { Algorithm = alg });
                GridPath p = r.Replan();
                first += p.ExpandedCount;
                if (!p.Found || Math.Abs(p.Cost - e.OptimalLength) > 1e-6 * Math.Max(1.0, e.OptimalLength))
                {
                    bad++;
                    Console.Error.WriteLine($"MISMATCH {job.Name} first replan ({e.StartX},{e.StartY})->({e.GoalX},{e.GoalY}) published {e.OptimalLength} ours {p.Cost}");
                }
                Cell cur = start;
                for (int round = 0; round < rounds; round++)
                {
                    // toggle: wall on an interior cell of the current path, or give back an earlier wall
                    if (toggled.Count > 0 && rng.NextInt(3) == 0)
                    {
                        int k = rng.NextInt(toggled.Count);
                        grid.SetArea(toggled[k].Cell.X, toggled[k].Cell.Y, toggled[k].Old);
                        toggled.RemoveAt(k);
                    }
                    else if (p.Found && p.Cells.Length > 2)
                    {
                        Cell w = p.Cells[1 + rng.NextInt(p.Cells.Length - 2)];
                        if (w != goal && w != cur)
                        {
                            toggled.Add((w, grid.GetArea(w.X, w.Y)));
                            grid.SetWalkable(w.X, w.Y, false);
                        }
                    }
                    if (alg == GridAlgorithm.DStarLite && p.Found && p.Cells.Length > 3)
                    {
                        cur = p.Cells[Math.Min(3, p.Cells.Length - 2)];
                        if (grid.IsWalkable(cur.X, cur.Y))
                        {
                            r.MoveStart(cur);
                        }
                        else
                        {
                            cur = r.Start;
                        }
                    }
                    p = r.Replan();
                    rep += p.ExpandedCount;
                    c++;
                    GridPath f = grid.FindPath(r.Start, goal);
                    fresh += f.ExpandedCount;
                    bool same = f.Status == p.Status && (!f.Found || Math.Abs(f.Cost - p.Cost) <= 1e-9 * Math.Max(1.0, f.Cost));
                    if (!same)
                    {
                        bad++;
                        Console.Error.WriteLine($"MISMATCH {job.Name} round {round} ({r.Start.X},{r.Start.Y})->({goal.X},{goal.Y}) A* {f.Status} {f.Cost} replanned {p.Status} {p.Cost}");
                    }
                }
                for (int k = toggled.Count - 1; k >= 0; k--)
                {
                    grid.SetArea(toggled[k].Cell.X, toggled[k].Cell.Y, toggled[k].Old);
                }
            }
            lock (sync)
            {
                checks += c;
                mismatches += bad;
                repairExpanded += rep;
                freshExpanded += fresh;
                firstExpanded += first;
            }
        });
        Console.WriteLine($"{set} {kind}: maps {jobs.Count}, replans checked {checks}, mismatches {mismatches}, nodes expanded by repairs {repairExpanded} vs fresh A* {freshExpanded} (ratio {(freshExpanded == 0 ? 0 : (double)repairExpanded / freshExpanded).ToString("F3", CultureInfo.InvariantCulture)}); first searches expanded {firstExpanded}");
        return mismatches == 0 ? 0 : 1;
    }
}
