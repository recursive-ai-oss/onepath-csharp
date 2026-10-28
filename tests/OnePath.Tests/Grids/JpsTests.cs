using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class JpsTests
{
    private static GridOptions Options(GridAlgorithm a, GridHeuristic h = GridHeuristic.Auto) => new() { Algorithm = a, Heuristic = h };

    private static void CheckPath(Grid g, GridPath p, Cell a, Cell b, double expect)
    {
        Assert.True(p.Found, $"expected a path of cost {expect}");
        Assert.True(Math.Abs(p.Cost - expect) <= 1e-9, $"cost {p.Cost} but brute force says {expect}");
        ReadOnlySpan<Cell> cells = p.Cells;
        Assert.Equal(a, cells[0]);
        Assert.Equal(b, cells[^1]);
        double sum = 0;
        for (int i = 1; i < cells.Length; i++)
        {
            Assert.True(BruteForce.LegalMove(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, cells[i - 1], cells[i]), "illegal step in path");
            sum += BruteForce.MoveCost(g, cells[i - 1], cells[i]);
        }
        Assert.True(Math.Abs(sum - p.Cost) <= 1e-9, "path does not sum to its cost");
    }

    private static void CheckGrid(Grid g, ref SeededRandom rng, int queries)
    {
        List<Cell> starts = BruteForce.WalkableStarts(g, 1);
        if (starts.Count == 0)
        {
            return;
        }
        for (int q = 0; q < queries; q++)
        {
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            double expect = BruteForce.Dijkstra(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, a, b);
            foreach (GridAlgorithm alg in new[] { GridAlgorithm.Jps, GridAlgorithm.JpsPlus })
            {
                foreach (GridHeuristic h in new[] { GridHeuristic.Auto, GridHeuristic.Zero })
                {
                    GridPath p = g.FindPath(a, b, Options(alg, h));
                    Assert.Equal(alg, p.Algorithm);
                    if (double.IsPositiveInfinity(expect))
                    {
                        Assert.Equal(PathStatus.NoPath, p.Status);
                    }
                    else
                    {
                        CheckPath(g, p, a, b, expect);
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(10, 10_000, 0xD1UL, false)]
    [InlineData(10, 4_000, 0xD2UL, true)]
    [InlineData(36, 600, 0xD3UL, false)]
    public void CostsEqualDijkstraOnRandomGrids(int maxDim, int count, ulong seed, bool scaled)
    {
        var rng = new SeededRandom(seed);
        for (int i = 0; i < count; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas: false);
            if (scaled)
            {
                // Every area costs the same, but not 1: the unit cost scales the whole search.
                for (byte a = 0; a < 254; a++)
                {
                    g.Areas[a] = 2.5;
                }
                g.Areas[254] = 2.5;
            }
            CheckGrid(g, ref rng, 2);
        }
    }

    [Fact]
    public void CostsEqualDijkstraOnRoomsAndCorridors()
    {
        // Long straight runs with openings in the walls: the shapes that create forced neighbours.
        var rng = new SeededRandom(0xD4);
        for (int i = 0; i < 300; i++)
        {
            int w = 12 + rng.NextInt(20);
            int h = 12 + rng.NextInt(20);
            var g = new Grid(w, h);
            for (int k = 0; k < 4; k++)
            {
                bool horizontal = rng.NextInt(2) == 0;
                int line = 1 + rng.NextInt((horizontal ? h : w) - 2);
                int gap = rng.NextInt(horizontal ? w : h);
                for (int t = 0; t < (horizontal ? w : h); t++)
                {
                    if (t != gap && rng.NextDouble() < 0.9)
                    {
                        g.SetArea(horizontal ? t : line, horizontal ? line : t, AreaCosts.Wall);
                    }
                }
            }
            CheckGrid(g, ref rng, 3);
        }
    }

    [Fact]
    public void UnsupportedOptionsFallBackToAStar()
    {
        var g = new Grid(8, 8);
        var a = new Cell(0, 0);
        var b = new Cell(5, 4);
        GridOptions[] options =
        [
            new() { Algorithm = GridAlgorithm.Jps, AgentSize = 2 },
            new() { Algorithm = GridAlgorithm.Jps, Connectivity = Connectivity.Four },
            new() { Algorithm = GridAlgorithm.JpsPlus, Diagonals = DiagonalRule.NoSqueezing },
            new() { Algorithm = GridAlgorithm.JpsPlus, HeuristicWeight = 2.0 },
            new() { Algorithm = GridAlgorithm.Jps, StepCost = (_, _, c) => c },
            new() { Algorithm = GridAlgorithm.Jps, Blocked = _ => false },
            new() { Algorithm = GridAlgorithm.Jps, Heuristic = GridHeuristic.Manhattan },
        ];
        foreach (GridOptions o in options)
        {
            GridPath p = g.FindPath(a, b, o, cancellationToken: TestContext.Current.CancellationToken);
            Assert.True(p.Found);
            Assert.Equal(GridAlgorithm.AStar, p.Algorithm);
        }
        g.Areas[3] = 2.0; // areas now cost differently
        Assert.Equal(GridAlgorithm.AStar, g.FindPath(a, b, Options(GridAlgorithm.Jps), cancellationToken: TestContext.Current.CancellationToken).Algorithm);
        Assert.Equal(GridAlgorithm.AStar, g.FindPath(a, b, Options(GridAlgorithm.JpsPlus), cancellationToken: TestContext.Current.CancellationToken).Algorithm);
    }

    [Fact]
    public void StartEqualsGoalAndInvalidCells()
    {
        var g = new Grid(5, 5);
        g.SetWalkable(2, 2, false);
        GridPath same = g.FindPath(new Cell(1, 1), new Cell(1, 1), Options(GridAlgorithm.JpsPlus), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(same.Found);
        Assert.Equal(0.0, same.Cost);
        Assert.Single(same.Cells.ToArray());
        Assert.Equal(PathStatus.GoalInvalid, g.FindPath(new Cell(0, 0), new Cell(2, 2), Options(GridAlgorithm.Jps), cancellationToken: TestContext.Current.CancellationToken).Status);
        Assert.Equal(PathStatus.StartInvalid, g.FindPath(new Cell(9, 0), new Cell(1, 1), Options(GridAlgorithm.Jps), cancellationToken: TestContext.Current.CancellationToken).Status);
    }

    [Fact]
    public void TableEntriesEqualTheScan()
    {
        // Every positive table entry is the cell the online scan finds; every other entry is a wall distance (the scan finds nothing).
        var rng = new SeededRandom(0xD5);
        for (int i = 0; i < 300; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 24, areas: false);
            using var index = new JpsPlusIndex(g);
            var online = new OnlineJumper(g);
            for (int y = 0; y < g.Height; y++)
            {
                for (int x = 0; x < g.Width; x++)
                {
                    int node = (y * g.Width) + x;
                    for (int d = 0; d < 8; d++)
                    {
                        int jd = index.JumpDistance(d, x, y);
                        if (!g.IsWalkable(x, y))
                        {
                            Assert.Equal(0, jd);
                            continue;
                        }
                        int cell = online.Jump(node, d, node, out int steps);
                        if (jd > 0)
                        {
                            Assert.Equal(jd, steps);
                            Assert.Equal(node + (jd * (JumpRules.Dx(d) + (JumpRules.Dy(d) * g.Width))), cell);
                        }
                        else
                        {
                            Assert.Equal(-1, cell);
                            // Minus jd cells are walkable along the direction and the next one is not reachable.
                            int len = -jd;
                            for (int k = 1; k <= len; k++)
                            {
                                Assert.True(g.IsWalkable(x + (k * JumpRules.Dx(d)), y + (k * JumpRules.Dy(d))));
                            }
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void IncrementalTablesEqualRebuild()
    {
        var rng = new SeededRandom(0xD6);
        for (int seq = 0; seq < 1200; seq++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 22, areas: false);
            using var index = new JpsPlusIndex(g);
            int rounds = 1 + rng.NextInt(4);
            for (int r = 0; r < rounds; r++)
            {
                int edits = 1 + rng.NextInt(r == 0 ? 3 : 12);
                for (int e = 0; e < edits; e++)
                {
                    g.SetWalkable(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(2) == 0);
                }
                index.Refresh();
                using var fresh = new JpsPlusIndex(g);
                Assert.True(index.Tables.SequenceEqual(fresh.Tables), $"sequence {seq} round {r}: incremental tables differ from a rebuild");
            }
        }
    }

    [Fact]
    public void LargeBatchOfEditsRebuilds()
    {
        var rng = new SeededRandom(0xD7);
        Grid g = BruteForce.RandomGrid(ref rng, 40, areas: false);
        var big = new Grid(200, 120);
        using var index = new JpsPlusIndex(big);
        for (int i = 0; i < 700; i++)
        {
            big.SetWalkable(rng.NextInt(200), rng.NextInt(120), rng.NextInt(2) == 0);
        }
        index.Refresh();
        using var fresh = new JpsPlusIndex(big);
        Assert.True(index.Tables.SequenceEqual(fresh.Tables));
        Assert.NotNull(g);
    }

    [Fact]
    public void QueriesAfterEditsSeeTheNewGrid()
    {
        var rng = new SeededRandom(0xD8);
        for (int i = 0; i < 300; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 14, areas: false);
            GridOptions plus = Options(GridAlgorithm.JpsPlus);
            _ = g.FindPath(new Cell(0, 0), new Cell(0, 0), plus, cancellationToken: TestContext.Current.CancellationToken); // builds the index
            for (int e = 0; e < 5; e++)
            {
                g.SetWalkable(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(3) != 0);
                CheckGrid(g, ref rng, 1);
            }
        }
    }

    [Fact]
    public void SlicedSearchEqualsUnsliced()
    {
        var rng = new SeededRandom(0xD9);
        for (int t = 0; t < 60; t++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 20, areas: false);
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count == 0)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            int s = (a.Y * g.Width) + a.X;
            int e = (b.Y * g.Width) + b.X;
            using var index = new JpsPlusIndex(g);
            var whole = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(g.Width), 1.0, new GridSearchBuffers(g.CellCount) { RecordExpansions = true });
            whole.Begin(s, e);
            SearchStatus expect = whole.Run(TestContext.Current.CancellationToken);
            int[] expectedNodes = whole.Buffers.ExpandedNodes.ToArray();
            foreach (int budget in new[] { 1, 2, 3, 7, 50 })
            {
                var sliced = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(g.Width), 1.0, new GridSearchBuffers(g.CellCount) { RecordExpansions = true });
                sliced.Begin(s, e);
                SearchStatus st;
                do
                {
                    st = sliced.Step(budget);
                }
                while (st == SearchStatus.Running);
                Assert.Equal(expect, st);
                Assert.Equal(whole.PathCost, sliced.PathCost);
                Assert.Equal(whole.Iterations, sliced.Iterations);
                Assert.True(expectedNodes.AsSpan().SequenceEqual(sliced.Buffers.ExpandedNodes));
                if (st == SearchStatus.Found)
                {
                    int[] p1 = new int[whole.PathLength()];
                    int[] p2 = new int[sliced.PathLength()];
                    whole.CopyPath(p1);
                    sliced.CopyPath(p2);
                    Assert.True(p1.AsSpan().SequenceEqual(p2));
                }
            }
        }
    }

    [Fact]
    public void IterationBudgetGivesBudgetExhausted()
    {
        var g = new Grid(64, 64);
        for (int y = 0; y < 60; y++)
        {
            g.SetWalkable(32, y, false);
        }
        GridOptions o = new() { Algorithm = GridAlgorithm.Jps, IterationBudget = 1 };
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(63, 0), o, cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.BudgetExhausted, p.Status);
    }

    [Fact]
    public void JpsExpandsFewerNodesThanAStarOnAnOpenGridWithWalls()
    {
        var g = new Grid(128, 128);
        for (int k = 8; k < 120; k += 16)
        {
            for (int t = 0; t < 100; t++)
            {
                g.SetWalkable(k, t + (k % 32 == 8 ? 0 : 20), false);
            }
        }
        var a = new Cell(1, 1);
        var b = new Cell(126, 126);
        GridPath astar = g.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
        GridPath jps = g.FindPath(a, b, Options(GridAlgorithm.Jps), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(Math.Abs(astar.Cost - jps.Cost) <= 1e-9);
        Assert.True(jps.ExpandedCount < astar.ExpandedCount);
    }

    [Fact]
    public void BlockScanFindsTheSameJumpPointsAsTheCellScan()
    {
        // Widths and heights around word boundaries, goals anywhere, edits applied after the map exists.
        var rng = new SeededRandom(0xDA);
        int[] sizes = [1, 2, 63, 64, 65, 70, 127, 128, 129, 150];
        for (int i = 0; i < 120; i++)
        {
            int w = sizes[rng.NextInt(sizes.Length)];
            int h = sizes[rng.NextInt(sizes.Length)];
            if (w * h > 12_000)
            {
                h = Math.Max(1, 12_000 / w);
            }
            var g = new Grid(w, h);
            double density = 0.02 + (rng.NextDouble() * 0.3);
            for (int c = 0; c < w * h; c++)
            {
                if (rng.NextDouble() < density)
                {
                    g.SetWalkable(c % w, c / w, false);
                }
            }
            using var map = new BlockMap(g);
            for (int e = 0; e < 40; e++)
            {
                g.SetWalkable(rng.NextInt(w), rng.NextInt(h), rng.NextInt(2) == 0);
            }
            var block = new BlockJumper(map);
            var online = new OnlineJumper(g);
            for (int q = 0; q < 1500; q++)
            {
                int node = rng.NextInt(w * h);
                if (!g.IsWalkable(node % w, node / w))
                {
                    continue;
                }
                int goal = rng.NextInt(w * h);
                if (!g.IsWalkable(goal % w, goal / w))
                {
                    goal = node;
                }
                int d = rng.NextInt(8);
                int a = block.Jump(node, d, goal, out int sa);
                int b = online.Jump(node, d, goal, out int sb);
                Assert.True(a == b && sa == sb, $"{w}x{h} node {node} dir {d} goal {goal}: block {a}/{sa} cell scan {b}/{sb}");
            }
            for (int y = 0; y < h; y++)
            {
                for (int x = 0; x < w; x++)
                {
                    Assert.Equal(g.IsWalkable(x, y), map.IsFree(x, y));
                }
            }
        }
    }

    [Fact]
    public void AllAlgorithmsAgreeOnWideGrids()
    {
        var rng = new SeededRandom(0xDB);
        for (int i = 0; i < 40; i++)
        {
            int w = 70 + rng.NextInt(200);
            int h = 20 + rng.NextInt(60);
            var g = new Grid(w, h);
            double density = rng.NextDouble() * 0.3;
            for (int c = 0; c < w * h; c++)
            {
                if (rng.NextDouble() < density)
                {
                    g.SetWalkable(c % w, c / w, false);
                }
            }
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            for (int q = 0; q < 12; q++)
            {
                Cell a = starts[rng.NextInt(starts.Count)];
                Cell b = starts[rng.NextInt(starts.Count)];
                GridPath astar = g.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
                foreach (GridAlgorithm alg in new[] { GridAlgorithm.Jps, GridAlgorithm.JpsPlus })
                {
                    GridPath p = g.FindPath(a, b, Options(alg), cancellationToken: TestContext.Current.CancellationToken);
                    Assert.Equal(astar.Status, p.Status);
                    if (astar.Found)
                    {
                        CheckPath(g, p, a, b, astar.Cost);
                    }
                }
            }
        }
    }

    [Fact]
    public void SearchObjectSeesEditsMadeAfterItWasCreated()
    {
        var g = new Grid(3, 3);
        g.SetWalkable(1, 1, false);
        using var index = new JpsPlusIndex(g);
        var plus = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(3));
        plus.Begin(0, 8);
        Assert.Equal(SearchStatus.Found, plus.Run(TestContext.Current.CancellationToken));
        Assert.Equal(4.0, plus.PathCost);
        g.SetWalkable(1, 1, true);
        plus.Begin(0, 8);
        Assert.Equal(SearchStatus.Found, plus.Run(TestContext.Current.CancellationToken));
        Assert.Equal(2.0 * GridMath.Sqrt2, plus.PathCost);
    }
}
