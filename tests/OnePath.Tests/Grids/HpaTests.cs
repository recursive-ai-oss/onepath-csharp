using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class HpaTests
{
    private static HpaOptions Options(int chunk, int size, Connectivity c, DiagonalRule r, int threshold = 6) =>
        new() { ChunkSize = chunk, AgentSize = size, Connectivity = c, Diagonals = r, EntranceThreshold = threshold };

    private static int[] Run(HpaSearch s, Grid g, Cell a, Cell b, out SearchStatus status)
    {
        s.Begin((a.Y * g.Width) + a.X, (b.Y * g.Width) + b.X);
        status = s.Run();
        if (status != SearchStatus.Found)
        {
            return [];
        }
        var cells = new int[s.PathLength()];
        s.CopyPath(cells);
        return cells;
    }

    // Checks every step against the brute-force move rules and the cost against an independent sum.
    private static double CheckPath(Grid g, int size, Connectivity c, DiagonalRule r, int[] path, Cell a, Cell b, double reported)
    {
        Assert.Equal(a, new Cell(path[0] % g.Width, path[0] / g.Width));
        Assert.Equal(b, new Cell(path[^1] % g.Width, path[^1] / g.Width));
        double sum = 0;
        for (int i = 1; i < path.Length; i++)
        {
            var p = new Cell(path[i - 1] % g.Width, path[i - 1] / g.Width);
            var q = new Cell(path[i] % g.Width, path[i] / g.Width);
            Assert.True(BruteForce.LegalMove(g, size, c, r, p, q), $"illegal step {p} -> {q}");
            sum += BruteForce.MoveCost(g, p, q);
        }
        Assert.True(Math.Abs(sum - reported) <= 1e-9, $"path sums to {sum} but reports {reported}");
        return sum;
    }

    [Theory]
    [InlineData(14, 5000, 0xE1UL, false)]
    [InlineData(14, 5000, 0xE2UL, true)]
    [InlineData(40, 300, 0xE3UL, true)]
    public void PathsAreValidNeverBeatOptimalAndExistExactlyWhenAStarFindsOne(int maxDim, int count, ulong seed, bool areas)
    {
        var rng = new SeededRandom(seed);
        Connectivity[] conns = [Connectivity.Eight, Connectivity.Four];
        DiagonalRule[] rules = [DiagonalRule.NoCornerCutting, DiagonalRule.NoSqueezing];
        for (int i = 0; i < count; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas);
            Connectivity c = conns[rng.NextInt(2)];
            DiagonalRule r = rules[rng.NextInt(2)];
            int size = 1 + (rng.NextInt(4) == 0 ? 1 : 0);
            int chunk = 2 + rng.NextInt(6);
            int threshold = 1 + rng.NextInt(8);
            List<Cell> starts = BruteForce.WalkableStarts(g, size);
            if (starts.Count == 0)
            {
                continue;
            }
            using var h = new HpaHierarchy(g, Options(chunk, size, c, r, threshold));
            var search = new HpaSearch(h);
            for (int q = 0; q < 3; q++)
            {
                Cell a = starts[rng.NextInt(starts.Count)];
                Cell b = starts[rng.NextInt(starts.Count)];
                double optimal = BruteForce.Dijkstra(g, size, c, r, a, b);
                foreach (bool smooth in new[] { false, true })
                {
                    search.Smoothing = smooth;
                    int[] path = Run(search, g, a, b, out SearchStatus st);
                    if (double.IsPositiveInfinity(optimal))
                    {
                        Assert.Equal(SearchStatus.NotFound, st);
                        continue;
                    }
                    Assert.Equal(SearchStatus.Found, st);
                    double cost = CheckPath(g, size, c, r, path, a, b, search.PathCost);
                    Assert.True(cost >= optimal - 1e-9, $"cost {cost} beats the optimum {optimal}");
                }
            }
        }
    }

    [Fact]
    public void SmoothingNeverLengthensAndOpenGridIsNearOptimal()
    {
        var g = new Grid(64, 64);
        using var h = new HpaHierarchy(g, Options(8, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting));
        var s = new HpaSearch(h);
        var raw = new HpaSearch(h) { Smoothing = false };
        Run(s, g, new Cell(1, 2), new Cell(60, 51), out _);
        Run(raw, g, new Cell(1, 2), new Cell(60, 51), out _);
        Assert.True(s.PathCost <= raw.PathCost + 1e-9);
        double optimal = BruteForce.Dijkstra(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, new Cell(1, 2), new Cell(60, 51));
        Assert.True(s.PathCost <= optimal * 1.05, $"{s.PathCost} vs optimal {optimal}");
    }

    [Theory]
    [InlineData(1, Connectivity.Eight, DiagonalRule.NoCornerCutting, 0xE4UL)]
    [InlineData(2, Connectivity.Eight, DiagonalRule.NoSqueezing, 0xE5UL)]
    [InlineData(1, Connectivity.Four, DiagonalRule.NoCornerCutting, 0xE6UL)]
    [InlineData(3, Connectivity.Eight, DiagonalRule.NoCornerCutting, 0xE7UL)]
    public void HierarchyAfterRandomEditsEqualsFullRebuild(int size, Connectivity c, DiagonalRule r, ulong seed)
    {
        var rng = new SeededRandom(seed);
        for (int round = 0; round < 60; round++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 40, areas: true);
            int chunk = 3 + rng.NextInt(8);
            using var h = new HpaHierarchy(g, Options(chunk, size, c, r, 1 + rng.NextInt(7)));
            for (int batch = 0; batch < 6; batch++)
            {
                int edits = 1 + rng.NextInt(batch < 3 ? 3 : 25);
                for (int e = 0; e < edits; e++)
                {
                    int x = rng.NextInt(g.Width);
                    int y = rng.NextInt(g.Height);
                    int roll = rng.NextInt(4);
                    g.SetArea(x, y, roll == 0 ? AreaCosts.Wall : (byte)roll == 1 ? (byte)0 : (byte)(1 + rng.NextInt(3)));
                }
                if (rng.NextInt(3) == 0)
                {
                    h.Refresh(); // sometimes between batches, sometimes lazily
                }
                using var fresh = new HpaHierarchy(g, Options(h.ChunkSize, size, c, r, h.EntranceThreshold));
                h.Refresh();
                Assert.True(h.ContentEquals(fresh), $"round {round} batch {batch}: incremental graph differs from a full rebuild");
                Assert.Equal(fresh.GetContentHash(), h.GetContentHash());
            }
        }
    }

    [Fact]
    public void AreaCostEditRebuildsEverything()
    {
        var g = new Grid(20, 20);
        for (int i = 0; i < 20; i++)
        {
            g.SetArea(i, 7, 1);
        }
        using var h = new HpaHierarchy(g, Options(5, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting));
        g.Areas[1] = 4.0;
        h.Refresh();
        using var fresh = new HpaHierarchy(g, Options(5, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting));
        Assert.True(h.ContentEquals(fresh));
    }

    [Fact]
    public void SlicedSearchGivesTheUnslicedPath()
    {
        var rng = new SeededRandom(0xE8);
        for (int i = 0; i < 200; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 30, areas: true);
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count == 0)
            {
                continue;
            }
            using var h = new HpaHierarchy(g, Options(4, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, 3));
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            var whole = new HpaSearch(h);
            int[] expected = Run(whole, g, a, b, out SearchStatus st);
            foreach (int budget in new[] { 1, 2, 5, 17 })
            {
                var sliced = new HpaSearch(h);
                sliced.Begin((a.Y * g.Width) + a.X, (b.Y * g.Width) + b.X);
                SearchStatus s;
                int calls = 0;
                do
                {
                    s = sliced.Step(budget);
                    calls++;
                }
                while (s == SearchStatus.Running && calls < 100000);
                Assert.Equal(st, s);
                if (st == SearchStatus.Found)
                {
                    var got = new int[sliced.PathLength()];
                    sliced.CopyPath(got);
                    Assert.Equal(expected, got);
                    Assert.Equal(whole.PathCost, sliced.PathCost);
                }
            }
        }
    }

    [Fact]
    public void TierOneOptionRunsHpaAndFallsBackWhenItCannot()
    {
        var g = new Grid(40, 40);
        for (int y = 5; y < 35; y++)
        {
            g.SetWalkable(20, y, false);
        }
        var a = new Cell(2, 20);
        var b = new Cell(37, 20);
        GridPath p = g.FindPath(a, b, new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 8 }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(p.Found);
        Assert.Equal(GridAlgorithm.Hpa, p.Algorithm);
        double optimal = g.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken).Cost;
        Assert.InRange(p.Cost, optimal - 1e-9, optimal * 1.1);

        // An edit goes through the change listener: opening the wall shortens the path next query.
        g.SetWalkable(20, 20, true);
        GridPath q = g.FindPath(a, b, new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 8 }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(q.Cost < p.Cost);
        Assert.InRange(q.Cost, 35.0 - 1e-9, 36.0);

        Assert.Equal(GridAlgorithm.AStar, g.FindPath(a, b, new GridOptions { Algorithm = GridAlgorithm.Hpa, StepCost = (_, _, c) => c }, cancellationToken: TestContext.Current.CancellationToken).Algorithm);
        Assert.Equal(GridAlgorithm.AStar, g.FindPath(a, b, new GridOptions { Algorithm = GridAlgorithm.Hpa, Diagonals = DiagonalRule.AllowAll }, cancellationToken: TestContext.Current.CancellationToken).Algorithm);

        GridPath w = g.FindPath(new WorldPoint(2.5, 20.5), new WorldPoint(37.5, 20.5), new GridOptions { Algorithm = GridAlgorithm.Hpa }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(w.Found);
        Assert.Equal(w.Cells.Length, w.Points.Length);

        GridPath s = g.FindPath(a, b, new GridOptions { Algorithm = GridAlgorithm.Hpa, IterationBudget = 1 }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.BudgetExhausted, s.Status);
        GridPath none = g.FindPath(a, a, new GridOptions { Algorithm = GridAlgorithm.Hpa }, cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(none.Found);
        Assert.Equal(0.0, none.Cost);
    }

    [Fact]
    public void InvalidInputsThrowTypedErrors()
    {
        var g = new Grid(8, 8);
        g.SetWalkable(3, 3, false);
        Assert.Throws<ArgumentException>(() => new HpaHierarchy(g, Options(4, 1, Connectivity.Eight, DiagonalRule.AllowAll)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HpaOptions { ChunkSize = 1 });
        using var h = new HpaHierarchy(g);
        var s = new HpaSearch(h);
        Assert.Throws<ArgumentOutOfRangeException>(() => s.Begin(-1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => s.Begin(0, 64));
        Assert.Throws<ArgumentException>(() => s.Begin((3 * 8) + 3, 0));
        Assert.Throws<InvalidOperationException>(() => s.Step(1));
        Assert.Throws<InvalidOperationException>(() => s.PathLength());
    }
}
