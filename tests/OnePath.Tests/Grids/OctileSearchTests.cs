using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>
/// The specialised octile searches (A*, JPS, JPS+, with and without improved pruning) against Dijkstra on random grids: equal
/// optimal costs, valid unit-step paths, found/not-found agreement, with edits between queries.
/// </summary>
public class OctileSearchTests
{
    private static void CheckPath(Grid g, IOctileEngine e, Cell start, Cell goal, double expected)
    {
        int[] nodes = new int[e.PathLength()];
        e.CopyPath(nodes);
        Assert.Equal((start.Y * g.Width) + start.X, nodes[0]);
        Assert.Equal((goal.Y * g.Width) + goal.X, nodes[^1]);
        double sum = 0;
        for (int i = 1; i < nodes.Length; i++)
        {
            var a = new Cell(nodes[i - 1] % g.Width, nodes[i - 1] / g.Width);
            var b = new Cell(nodes[i] % g.Width, nodes[i] / g.Width);
            Assert.True(BruteForce.LegalMove(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, a, b), $"illegal step {a} -> {b}");
            sum += BruteForce.MoveCost(g, a, b);
        }
        Assert.Equal(expected, sum, 6);
        Assert.Equal(expected, e.PathCost, 6);
    }

    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    public void AllVariantsMatchDijkstra(int seedBase)
    {
        var rng = new SeededRandom(0xC0FFEEUL + (ulong)seedBase);
        for (int round = 0; round < 120; round++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, round % 7 == 0 ? 150 : 40, false);
            var starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count < 2)
            {
                continue;
            }
            using var map = new OctileMap(g);
            using var index = new JpsPlusIndex(g);
            var astar = new OctileAStar(g);
            var jps = new OctileJump<BitFinder>(new BitFinder(map, false), g.Width, g.Height);
            var jpsP = new OctileJump<BitFinder>(new BitFinder(map, true), g.Width, g.Height);
            var plus = new OctileJump<TableFinder>(new TableFinder(index, map, false), g.Width, g.Height);
            var plusP = new OctileJump<TableFinder>(new TableFinder(index, map, true), g.Width, g.Height);
            for (int q = 0; q < 12; q++)
            {
                Cell s = starts[rng.NextInt(starts.Count)];
                Cell t = starts[rng.NextInt(starts.Count)];
                double expected = BruteForce.Dijkstra(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, s, t);
                int a = (s.Y * g.Width) + s.X;
                int b = (t.Y * g.Width) + t.X;
                foreach (bool zero in new[] { false, true })
                {
                    if (zero && g.Width * g.Height > 600)
                    {
                        continue;
                    }
                    Check(g, astar, astar.Run(a, b, zero, int.MaxValue, true), s, t, expected);
                    Check(g, jps, jps.Run(a, b, zero, int.MaxValue, false), s, t, expected);
                    Check(g, jpsP, jpsP.Run(a, b, zero, int.MaxValue, false), s, t, expected);
                    Check(g, plus, plus.Run(a, b, zero, int.MaxValue, false), s, t, expected);
                    Check(g, plusP, plusP.Run(a, b, zero, int.MaxValue, false), s, t, expected);
                }
            }
        }
    }

    private static void Check(Grid g, IOctileEngine e, SearchStatus status, Cell s, Cell t, double expected)
    {
        if (double.IsPositiveInfinity(expected))
        {
            Assert.Equal(SearchStatus.NotFound, status);
            return;
        }
        Assert.Equal(SearchStatus.Found, status);
        CheckPath(g, e, s, t, expected);
    }

    [Fact]
    public void EditsBetweenQueriesAreSeenByEveryEngine()
    {
        var rng = new SeededRandom(77UL);
        Grid g = BruteForce.RandomGrid(ref rng, 70, false);
        var o = new GridOptions();
        for (int round = 0; round < 60; round++)
        {
            for (int k = 0; k < 6; k++)
            {
                g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)0);
            }
            var starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count < 2)
            {
                continue;
            }
            for (int q = 0; q < 5; q++)
            {
                Cell s = starts[rng.NextInt(starts.Count)];
                Cell t = starts[rng.NextInt(starts.Count)];
                double expected = BruteForce.Dijkstra(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, s, t);
                foreach (GridAlgorithm alg in new[] { GridAlgorithm.AStar, GridAlgorithm.Jps, GridAlgorithm.JpsPlus })
                {
                    GridPath p = g.FindPath(s, t, new GridOptions { Algorithm = alg });
                    if (double.IsPositiveInfinity(expected))
                    {
                        Assert.Equal(PathOutcome.NoPath, p.Status);
                    }
                    else
                    {
                        Assert.Equal(PathOutcome.Found, p.Status);
                        Assert.Equal(expected, p.Cost, 6);
                    }
                }
            }
        }
        Assert.NotNull(o);
    }

    [Fact]
    public void SlicedBudgetStopsAndFullBudgetFinishes()
    {
        var g = Grid.FromText(string.Join('\n', new string('.', 40), new string('.', 40), new string('.', 40)));
        GridPath cut = g.FindPath(new Cell(0, 0), new Cell(39, 2), new GridOptions { IterationBudget = 3 });
        Assert.Equal(PathOutcome.BudgetExhausted, cut.Status);
        GridPath full = g.FindPath(new Cell(0, 0), new Cell(39, 2));
        Assert.Equal(PathOutcome.Found, full.Status);
    }
}
