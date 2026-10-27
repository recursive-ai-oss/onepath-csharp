using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class ThetaTests
{
    private static readonly GridAlgorithm[] Algorithms = [GridAlgorithm.ThetaStar, GridAlgorithm.LazyThetaStar];

    [Theory]
    [InlineData(3_000, 0xE1UL)]
    [InlineData(300, 0xE2UL)]
    public void LineOfSightEqualsExactSegmentTest(int count, ulong seed)
    {
        var rng = new SeededRandom(seed);
        int maxDim = seed == 0xE1UL ? 12 : 40;
        for (int i = 0; i < count; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas: false);
            for (int q = 0; q < 60; q++)
            {
                var a = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
                var b = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
                bool expect = AnyAngleOracle.Visible(g, a, b);
                Assert.True(expect == GridLineOfSight.Test(g, a, b), $"{a} -> {b} on {g.Width}x{g.Height}: oracle says {expect}");
                Assert.True(expect == GridLineOfSight.Test(g, b, a), $"{b} -> {a} is not symmetric");
            }
        }
    }

    [Fact]
    public void LineOfSightRules()
    {
        // A wall touching only the corner on the line blocks it; diagonal neighbours see each other exactly when no corner is cut.
        var g = Grid.From(new bool[,]
        {
            { true, true, true },
            { true, true, false },
            { true, true, true },
        });
        Assert.False(GridLineOfSight.Test(g, new Cell(0, 0), new Cell(2, 2))); // passes through (1,1) and the corner of the wall at (2,1)
        Assert.True(GridLineOfSight.Test(g, new Cell(0, 0), new Cell(1, 1)));
        Assert.False(GridLineOfSight.Test(g, new Cell(1, 2), new Cell(2, 1))); // endpoint is a wall
        Assert.False(GridLineOfSight.Test(g, new Cell(1, 0), new Cell(2, 1)));
        Assert.True(GridLineOfSight.Test(g, new Cell(0, 0), new Cell(0, 2)));
        Assert.Throws<ArgumentOutOfRangeException>(() => GridLineOfSight.Test(g, new Cell(0, 0), new Cell(3, 0)));
    }

    private static void CheckPath(Grid g, GridPath p, Cell a, Cell b, double unit)
    {
        ReadOnlySpan<Cell> cells = p.Cells;
        Assert.Equal(a, cells[0]);
        Assert.Equal(b, cells[^1]);
        double sum = 0;
        for (int i = 1; i < cells.Length; i++)
        {
            Assert.True(AnyAngleOracle.Visible(g, cells[i - 1], cells[i]), $"no line of sight {cells[i - 1]} -> {cells[i]}");
            double dx = cells[i].X - cells[i - 1].X;
            double dy = cells[i].Y - cells[i - 1].Y;
            sum += unit * Math.Sqrt((dx * dx) + (dy * dy));
        }
        Assert.Equal(sum, p.Cost); // costs are sums of segment lengths in path order, bit for bit
    }

    [Theory]
    [InlineData(10_000, 12, 0xE3UL, 1.0)]
    [InlineData(2_000, 12, 0xE4UL, 2.5)]
    [InlineData(300, 30, 0xE5UL, 1.0)]
    public void PathsAreValidNoLongerThanAStarAndNoShorterThanTheVisibilityGraph(int count, int maxDim, ulong seed, double unit)
    {
        var rng = new SeededRandom(seed);
        for (int i = 0; i < count; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas: false);
            if (unit != 1.0)
            {
                for (byte a = 0; a <= 254; a++)
                {
                    g.Areas[a] = unit;
                }
            }
            List<Cell> free = BruteForce.WalkableStarts(g, 1);
            if (free.Count == 0)
            {
                continue;
            }
            for (int q = 0; q < 4; q++)
            {
                Cell a = free[rng.NextInt(free.Count)];
                Cell b = free[rng.NextInt(free.Count)];
                GridPath astar = g.FindPath(a, b);
                double vis = maxDim <= 12 ? AnyAngleOracle.VisibilityGraphDistance(g, a, b, unit) : double.NaN;
                foreach (GridAlgorithm alg in Algorithms)
                {
                    GridPath p = g.FindPath(a, b, new GridOptions { Algorithm = alg });
                    Assert.Equal(alg, p.Algorithm);
                    Assert.Equal(astar.Status, p.Status);
                    if (!p.Found)
                    {
                        continue;
                    }
                    CheckPath(g, p, a, b, unit);
                    Assert.True(p.Cost <= astar.Cost * (1 + 1e-12), $"{alg} {a}->{b} cost {p.Cost} exceeds A* {astar.Cost}\n" + Dump(g, p));
                    if (!double.IsNaN(vis))
                    {
                        Assert.True(p.Cost >= vis * (1 - 1e-12), $"{alg} {a}->{b} cost {p.Cost} is below the visibility-graph optimum {vis}");
                    }
                }
            }
        }
    }

    private static string Dump(Grid g, GridPath p)
    {
        var sb = new System.Text.StringBuilder();
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                sb.Append(g.GetArea(x, y) == AreaCosts.Wall ? '#' : '.');
            }
            sb.Append('\n');
        }
        foreach (Cell c in p.Cells)
        {
            sb.Append(c).Append(' ');
        }
        return sb.ToString();
    }

    [Fact]
    public void ZeroAndWeightedEstimatesStayValid()
    {
        var rng = new SeededRandom(0xE6);
        for (int i = 0; i < 1_500; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 20, areas: false);
            List<Cell> free = BruteForce.WalkableStarts(g, 1);
            if (free.Count == 0)
            {
                continue;
            }
            Cell a = free[rng.NextInt(free.Count)];
            Cell b = free[rng.NextInt(free.Count)];
            bool reachable = g.FindPath(a, b).Found;
            foreach (GridAlgorithm alg in Algorithms)
            {
                foreach (GridOptions o in new[]
                {
                    new GridOptions { Algorithm = alg, Heuristic = GridHeuristic.Zero },
                    new GridOptions { Algorithm = alg, Heuristic = GridHeuristic.Euclidean, HeuristicWeight = 3.0 },
                })
                {
                    GridPath p = g.FindPath(a, b, o);
                    Assert.Equal(alg, p.Algorithm);
                    Assert.Equal(reachable, p.Found);
                    if (p.Found)
                    {
                        CheckPath(g, p, a, b, 1.0);
                    }
                }
            }
        }
    }

    [Fact]
    public void EveryVariantReturnsValidPathsAboveTheVisibilityGraphOptimum()
    {
        var rng = new SeededRandom(0xE9);
        int paperLonger = 0;
        int total = 0;
        for (int i = 0; i < 4_000; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 12, areas: false);
            List<Cell> free = BruteForce.WalkableStarts(g, 1);
            if (free.Count == 0)
            {
                continue;
            }
            Cell ca = free[rng.NextInt(free.Count)];
            Cell cb = free[rng.NextInt(free.Count)];
            double vis = AnyAngleOracle.VisibilityGraphDistance(g, ca, cb);
            double astar = g.FindPath(ca, cb).Cost;
            foreach (ThetaVariant variant in new[] { ThetaVariant.Theta, ThetaVariant.Lazy, ThetaVariant.LazyRequeue })
            {
                var s = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), variant);
                s.Begin((ca.Y * g.Width) + ca.X, (cb.Y * g.Width) + cb.X);
                bool found = s.Run(TestContext.Current.CancellationToken) == SearchStatus.Found;
                Assert.Equal(!double.IsPositiveInfinity(vis), found);
                if (!found)
                {
                    continue;
                }
                var nodes = new int[s.PathLength()];
                s.CopyPath(nodes);
                for (int k = 1; k < nodes.Length; k++)
                {
                    var p = new Cell(nodes[k - 1] % g.Width, nodes[k - 1] / g.Width);
                    var q = new Cell(nodes[k] % g.Width, nodes[k] / g.Width);
                    Assert.True(AnyAngleOracle.Visible(g, p, q));
                }
                Assert.True(s.PathCost >= vis * (1 - 1e-12));
                if (variant == ThetaVariant.Lazy)
                {
                    total++;
                    if (s.PathCost > astar * (1 + 1e-12))
                    {
                        paperLonger++;
                    }
                }
            }
        }

        // The paper's Lazy Theta* is occasionally longer than the grid path; the re-queued variant never was (checked in the A* comparison above).
        Assert.True(paperLonger * 100 < total, $"{paperLonger} of {total}");
    }

    [Fact]
    public void OpenFieldIsAStraightLine()
    {
        var g = new Grid(60, 40);
        foreach (GridAlgorithm alg in Algorithms)
        {
            GridPath p = g.FindPath(new Cell(2, 3), new Cell(50, 29), new GridOptions { Algorithm = alg });
            Assert.True(p.Found);
            Assert.Equal(2, p.Cells.Length);
            Assert.Equal(Math.Sqrt((48.0 * 48.0) + (26.0 * 26.0)), p.Cost);
        }
    }

    [Fact]
    public void StartEqualsGoalAndWorldPoints()
    {
        var g = new Grid(10, 10) { Space = new GridSpace(100, 200, 4) };
        foreach (GridAlgorithm alg in Algorithms)
        {
            var o = new GridOptions { Algorithm = alg };
            GridPath same = g.FindPath(new Cell(3, 3), new Cell(3, 3), o);
            Assert.True(same.Found);
            Assert.Single(same.Cells.ToArray());
            Assert.Equal(0.0, same.Cost);
            GridPath w = g.FindPath(new WorldPoint(101, 201), new WorldPoint(137, 233), o);
            Assert.True(w.Found);
            Assert.Equal(w.Cells.Length, w.WorldPoints.Length);
            Assert.Equal(new WorldPoint(102, 202), w.WorldPoints[0]);
        }
    }

    [Fact]
    public void UnsupportedOptionsFallBackToAStar()
    {
        var g = new Grid(8, 8);
        var a = new Cell(0, 0);
        var b = new Cell(5, 4);
        foreach (GridAlgorithm alg in Algorithms)
        {
            GridOptions[] options =
            [
                new() { Algorithm = alg, AgentSize = 2 },
                new() { Algorithm = alg, Connectivity = Connectivity.Four },
                new() { Algorithm = alg, Diagonals = DiagonalRule.NoSqueezing },
                new() { Algorithm = alg, Heuristic = GridHeuristic.Octile },
                new() { Algorithm = alg, Blocked = c => c.X == 7 },
                new() { Algorithm = alg, StepCost = (_, _, c) => c },
            ];
            foreach (GridOptions o in options)
            {
                GridPath p = g.FindPath(a, b, o);
                Assert.True(p.Found);
                Assert.Equal(GridAlgorithm.AStar, p.Algorithm);
            }
            var mixed = new Grid(8, 8);
            mixed.Areas[1] = 3.0;
            mixed.SetArea(2, 2, 1);
            Assert.Equal(GridAlgorithm.AStar, mixed.FindPath(a, b, new GridOptions { Algorithm = alg }).Algorithm);
        }
    }

    [Fact]
    public void SlicedSearchEqualsUnsliced()
    {
        var rng = new SeededRandom(0xE7);
        for (int i = 0; i < 400; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 25, areas: false);
            List<Cell> free = BruteForce.WalkableStarts(g, 1);
            if (free.Count == 0)
            {
                continue;
            }
            int a = (free[rng.NextInt(free.Count)] is var ca) ? (ca.Y * g.Width) + ca.X : 0;
            Cell cb = free[rng.NextInt(free.Count)];
            int b = (cb.Y * g.Width) + cb.X;
            foreach (ThetaVariant variant in new[] { ThetaVariant.Theta, ThetaVariant.Lazy, ThetaVariant.LazyRequeue })
            {
                var whole = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
                var s1 = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), variant, 1.0, whole);
                s1.Begin(a, b);
                SearchStatus st1 = s1.Run(TestContext.Current.CancellationToken);
                int[] expanded1 = whole.ExpandedNodes.ToArray();
                var sliced = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
                var s2 = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), variant, 1.0, sliced);
                s2.Begin(a, b);
                int budget = 1 + rng.NextInt(7);
                SearchStatus st2;
                while ((st2 = s2.Step(budget)) == SearchStatus.Running)
                {
                }
                Assert.Equal(st1, st2);
                Assert.Equal(s1.PathCost, s2.PathCost);
                Assert.Equal(expanded1, sliced.ExpandedNodes.ToArray());
                Assert.Equal(s1.Iterations, s2.Iterations);
            }
        }
    }

    [Fact]
    public void IterationBudgetEndsTheQuery()
    {
        var g = new Grid(100, 100);
        foreach (GridAlgorithm alg in Algorithms)
        {
            GridPath p = g.FindPath(new Cell(0, 0), new Cell(99, 99), new GridOptions { Algorithm = alg, Heuristic = GridHeuristic.Zero, IterationBudget = 5 });
            Assert.Equal(PathOutcome.BudgetExhausted, p.Status);
        }
    }

    [Fact]
    public void ReusedBuffersGiveTheSameAnswer()
    {
        var rng = new SeededRandom(0xE8);
        Grid g = BruteForce.RandomGrid(ref rng, 30, areas: false);
        List<Cell> free = BruteForce.WalkableStarts(g, 1);
        var shared = new GridSearchBuffers(g.CellCount);
        for (int q = 0; q < 200; q++)
        {
            Cell ca = free[rng.NextInt(free.Count)];
            Cell cb = free[rng.NextInt(free.Count)];
            int a = (ca.Y * g.Width) + ca.X;
            int b = (cb.Y * g.Width) + cb.X;
            var reused = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), ThetaVariant.Lazy, 1.0, shared);
            var fresh = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), ThetaVariant.Lazy);
            reused.Begin(a, b);
            fresh.Begin(a, b);
            Assert.Equal(fresh.Run(TestContext.Current.CancellationToken), reused.Run(TestContext.Current.CancellationToken));
            Assert.Equal(fresh.PathCost, reused.PathCost);
        }
    }
}
