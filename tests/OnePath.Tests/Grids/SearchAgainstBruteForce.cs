using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class SearchAgainstBruteForce
{
    private const int Grids = 10_000;

    private static void Check(Grid g, int size, Connectivity conn, DiagonalRule rule, ref SeededRandom rng, GridHeuristic h, int queries)
    {
        List<Cell> starts = BruteForce.WalkableStarts(g, size);
        if (starts.Count == 0)
        {
            return;
        }
        var options = new GridOptions { AgentSize = size, Connectivity = conn, Diagonals = rule, Heuristic = h };
        for (int q = 0; q < queries; q++)
        {
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            double expect = BruteForce.Dijkstra(g, size, conn, rule, a, b);
            GridPath p = g.FindPath(a, b, options);
            if (double.IsPositiveInfinity(expect))
            {
                Assert.Equal(PathStatus.NoPath, p.Status);
                continue;
            }
            Assert.True(p.Found, $"expected a path of cost {expect}");
            Assert.True(Math.Abs(p.Cost - expect) <= 1e-9, $"cost {p.Cost} but brute force says {expect}");
            // The returned path itself must be legal, start at a, end at b and sum to the reported cost.
            ReadOnlySpan<Cell> cells = p.Cells;
            Assert.Equal(a, cells[0]);
            Assert.Equal(b, cells[^1]);
            double sum = 0;
            for (int i = 1; i < cells.Length; i++)
            {
                Assert.True(BruteForce.LegalMove(g, size, conn, rule, cells[i - 1], cells[i]), "illegal step in path");
                sum += BruteForce.MoveCost(g, cells[i - 1], cells[i]);
            }
            Assert.True(Math.Abs(sum - p.Cost) <= 1e-9);
        }
    }

    [Theory]
    [InlineData(Connectivity.Four, DiagonalRule.NoCornerCutting, 1, false, 0xA1UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, false, 0xA2UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoSqueezing, 1, false, 0xA3UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.AllowAll, 1, false, 0xA4UL)]
    [InlineData(Connectivity.Four, DiagonalRule.NoCornerCutting, 1, true, 0xA5UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, true, 0xA6UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoSqueezing, 1, true, 0xA7UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoCornerCutting, 2, false, 0xA8UL)]
    [InlineData(Connectivity.Eight, DiagonalRule.NoCornerCutting, 3, true, 0xA9UL)]
    [InlineData(Connectivity.Four, DiagonalRule.NoCornerCutting, 2, true, 0xAAUL)]
    [InlineData(Connectivity.Eight, DiagonalRule.AllowAll, 2, false, 0xABUL)]
    public void SquareGridCostsEqualDijkstra(Connectivity conn, DiagonalRule rule, int size, bool areas, ulong seed)
    {
        var rng = new SeededRandom(seed);
        for (int i = 0; i < Grids; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, size == 1 ? 10 : 12, areas);
            // The heuristic varies with the grid so Dijkstra (zero), the default and Manhattan on four-connected grids are all covered.
            GridHeuristic h = (i % 4) switch
            {
                0 => GridHeuristic.Zero,
                _ => GridHeuristic.Auto,
            };
            Check(g, size, conn, rule, ref rng, h, 2);
        }
    }

    [Theory]
    [InlineData(HexLayout.PointyOddRow, 0xB1UL)]
    [InlineData(HexLayout.PointyEvenRow, 0xB2UL)]
    [InlineData(HexLayout.FlatOddColumn, 0xB3UL)]
    [InlineData(HexLayout.FlatEvenColumn, 0xB4UL)]
    public void HexGridCostsEqualDijkstra(HexLayout layout, ulong seed)
    {
        var rng = new SeededRandom(seed);
        for (int i = 0; i < Grids; i++)
        {
            Grid sq = BruteForce.RandomGrid(ref rng, 8, areas: i % 2 == 1);
            var hex = HexGrid.From(sq.Cells, sq.Width, sq.Height, layout);
            hex.Areas.CopyFrom(sq.Areas);
            hex.Space = new GridSpace(0, 0, 1 + rng.NextInt(3));
            var cells = new List<Cell>();
            for (int y = 0; y < hex.Height; y++)
            {
                for (int x = 0; x < hex.Width; x++)
                {
                    if (hex.GetArea(x, y) != AreaCosts.Wall)
                    {
                        cells.Add(new Cell(x, y));
                    }
                }
            }
            if (cells.Count == 0)
            {
                continue;
            }
            for (int q = 0; q < 2; q++)
            {
                Cell a = cells[rng.NextInt(cells.Count)];
                Cell b = cells[rng.NextInt(cells.Count)];
                double expect = BruteForce.HexDijkstra(hex, a, b);
                GridPath p = hex.FindPath(a, b, i % 3 == 0 ? new GridOptions { Heuristic = GridHeuristic.Zero } : null, cancellationToken: TestContext.Current.CancellationToken);
                if (double.IsPositiveInfinity(expect))
                {
                    Assert.Equal(PathStatus.NoPath, p.Status);
                }
                else
                {
                    Assert.True(p.Found);
                    Assert.True(Math.Abs(p.Cost - expect) <= 1e-9, $"hex cost {p.Cost} vs {expect}");
                }
            }
        }
    }

    [Fact]
    public void SlicedSearchEqualsUnslicedForEveryBudget()
    {
        var rng = new SeededRandom(0xC0FFEE);
        for (int t = 0; t < 40; t++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 14, areas: t % 2 == 0);
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count == 0)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            SquareTopology topo = g.CreateTopology();
            int s = (a.Y * g.Width) + a.X;
            int e = (b.Y * g.Width) + b.X;
            var buffers = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
            var whole = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topo, default, new OctileHeuristic(g.Width, g.Areas.MinCost), buffers);
            whole.Begin(s, e);
            SearchStatus ws = whole.Run(TestContext.Current.CancellationToken);
            int[] wExp = buffers.ExpandedNodes.ToArray();
            int[] wPath = ws == SearchStatus.Found ? new int[whole.PathLength()] : [];
            if (ws == SearchStatus.Found)
            {
                whole.CopyPath(wPath);
            }
            int total = whole.Iterations;
            for (int budget = 1; budget <= total + 1; budget++)
            {
                var other = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
                var sliced = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topo, default, new OctileHeuristic(g.Width, g.Areas.MinCost), other);
                sliced.Begin(s, e);
                int calls = 0;
                while (sliced.Step(budget) == SearchStatus.Running)
                {
                    calls++;
                    Assert.True(calls <= total + 1);
                }
                Assert.Equal(ws, sliced.Status);
                Assert.Equal(wExp, other.ExpandedNodes.ToArray());
                Assert.Equal(whole.ExpandedCount, sliced.ExpandedCount);
                if (ws == SearchStatus.Found)
                {
                    Assert.Equal(whole.PathCost, sliced.PathCost);
                    var p = new int[sliced.PathLength()];
                    sliced.CopyPath(p);
                    Assert.Equal(wPath, p);
                }
            }
        }
    }

    [Fact]
    public void BufferReuseGivesTheSameAnswers()
    {
        var rng = new SeededRandom(77);
        Grid g = BruteForce.RandomGrid(ref rng, 14, areas: true);
        List<Cell> starts = BruteForce.WalkableStarts(g, 1);
        var first = new List<double>();
        for (int round = 0; round < 2; round++)
        {
            var r2 = new SeededRandom(5);
            for (int i = 0; i < 50; i++)
            {
                GridPath p = g.FindPath(starts[r2.NextInt(starts.Count)], starts[r2.NextInt(starts.Count)], cancellationToken: TestContext.Current.CancellationToken);
                if (round == 0)
                {
                    first.Add(p.Cost);
                }
                else
                {
                    Assert.Equal(first[i], p.Cost);
                }
            }
        }
    }

    [Fact]
    public void IterationBudgetOnTier1()
    {
        var g = new Grid(30, 30);
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(29, 29), new GridOptions { IterationBudget = 3, Heuristic = GridHeuristic.Zero }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.BudgetExhausted, p.Status);
        Assert.Equal(3, p.ExpandedCount);
    }

    [Fact]
    public void WeightedSearchStillFindsAPathAndNeverBeatsOptimal()
    {
        var rng = new SeededRandom(9);
        for (int i = 0; i < 200; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 12, areas: false);
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count == 0)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            GridPath opt = g.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
            GridPath w = g.FindPath(a, b, new GridOptions { HeuristicWeight = 3.0 }, cancellationToken: TestContext.Current.CancellationToken);
            Assert.Equal(opt.Found, w.Found);
            if (opt.Found)
            {
                Assert.True(w.Cost >= opt.Cost - 1e-9);
            }
        }
    }
}
