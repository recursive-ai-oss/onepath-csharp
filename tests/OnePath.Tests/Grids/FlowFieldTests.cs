using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class FlowFieldTests
{
    // Forward O(n^2) Dijkstra from one cell to every cell, straight from the definitions (shares no code with the field).
    private static double[] DijkstraAll(Grid g, int size, Connectivity conn, DiagonalRule rule, Cell start)
    {
        int n = g.Width * g.Height;
        var dist = new double[n];
        var done = new bool[n];
        Array.Fill(dist, double.PositiveInfinity);
        dist[(start.Y * g.Width) + start.X] = 0;
        for (int iter = 0; iter < n; iter++)
        {
            int u = -1;
            for (int i = 0; i < n; i++)
            {
                if (!done[i] && (u < 0 || dist[i] < dist[u]))
                {
                    u = i;
                }
            }
            if (u < 0 || double.IsPositiveInfinity(dist[u]))
            {
                break;
            }
            done[u] = true;
            var a = new Cell(u % g.Width, u / g.Width);
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    var b = new Cell(a.X + dx, a.Y + dy);
                    if (!g.Contains(b.X, b.Y) || !BruteForce.LegalMove(g, size, conn, rule, a, b))
                    {
                        continue;
                    }
                    int v = (b.Y * g.Width) + b.X;
                    double nd = dist[u] + BruteForce.MoveCost(g, a, b);
                    if (nd < dist[v])
                    {
                        dist[v] = nd;
                    }
                }
            }
        }
        return dist;
    }

    private static Cell[] PickGoals(Grid g, int size, ref SeededRandom rng)
    {
        List<Cell> ok = BruteForce.WalkableStarts(g, size);
        if (ok.Count == 0)
        {
            return [];
        }
        int k = 1 + rng.NextInt(3);
        var goals = new Cell[k];
        for (int i = 0; i < k; i++)
        {
            goals[i] = ok[rng.NextInt(ok.Count)];
        }
        return goals;
    }

    private static void CheckAgainstDijkstra(Grid g, GridFlowField f, Cell[] goals, int size, Connectivity conn, DiagonalRule rule)
    {
        // Forward Dijkstra from every cell: the cost of a cell is the least cost to any goal.
        bool exact = conn == Connectivity.Four; // dyadic costs add exactly in any order; with √2 the sums differ in the last bits
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                double want = double.PositiveInfinity;
                if (BruteForce.BlockFree(g, x, y, size))
                {
                    double[] d = DijkstraAll(g, size, conn, rule, new Cell(x, y));
                    foreach (Cell goal in goals)
                    {
                        want = Math.Min(want, d[(goal.Y * g.Width) + goal.X]);
                    }
                }
                double got = f.GetCost(new Cell(x, y));
                if (double.IsPositiveInfinity(want))
                {
                    Assert.True(double.IsPositiveInfinity(got), $"cell ({x},{y}) should be unreachable but costs {got}");
                }
                else if (exact)
                {
                    Assert.True(want == got, $"cell ({x},{y}) cost {got} but Dijkstra says {want}");
                }
                else
                {
                    Assert.True(Math.Abs(want - got) <= 1e-9 * Math.Max(1.0, want), $"cell ({x},{y}) cost {got} but Dijkstra says {want}");
                }
            }
        }

        // Following the direction field from every reachable cell reaches a goal at exactly the cell's cost.
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                double c = f.GetCost(new Cell(x, y));
                Cell[] path = f.GetPath(new Cell(x, y));
                if (double.IsPositiveInfinity(c))
                {
                    Assert.Empty(path);
                    continue;
                }
                Assert.Contains(path[^1], goals);
                double sum = 0;
                for (int i = 1; i < path.Length; i++)
                {
                    Assert.True(BruteForce.LegalMove(g, size, conn, rule, path[i - 1], path[i]), "illegal step");
                    sum += BruteForce.MoveCost(g, path[i - 1], path[i]);
                }
                Assert.True(Math.Abs(sum - c) <= 1e-9 * Math.Max(1.0, c), $"path cost {sum} vs field {c}");
            }
        }
    }

    [Fact]
    public void IntegrationFieldEqualsPerCellDijkstraOnTenThousandGrids()
    {
        var rng = new SeededRandom(0xF10A);
        int checkedGrids = 0;
        for (int i = 0; i < 10_000; i++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, i % 50 == 0 ? 14 : 7, areas: i % 2 == 1);
            int size = (i % 5 == 0) ? 2 : 1;
            Connectivity conn = (i % 3 == 0) ? Connectivity.Four : Connectivity.Eight;
            DiagonalRule rule = (DiagonalRule)(i % 3);
            Cell[] goals = PickGoals(g, size, ref rng);
            if (goals.Length == 0)
            {
                continue;
            }
            var options = new GridOptions { AgentSize = size, Connectivity = conn, Diagonals = rule };
            using GridFlowField f = g.CreateFlowField(goals, options);
            Assert.Equal(FlowFieldStatus.Complete, f.Status);
            CheckAgainstDijkstra(g, f, goals, size, conn, rule);
            checkedGrids++;
        }
        Assert.True(checkedGrids > 9_000);
    }

    [Fact]
    public void HexFieldEqualsPerCellDijkstra()
    {
        var rng = new SeededRandom(0x4E);
        HexLayout[] layouts = [HexLayout.PointyOddRow, HexLayout.PointyEvenRow, HexLayout.FlatOddColumn, HexLayout.FlatEvenColumn];
        for (int i = 0; i < 600; i++)
        {
            Grid sq = BruteForce.RandomGrid(ref rng, 6, areas: i % 2 == 1);
            var hex = HexGrid.From(sq.Cells, sq.Width, sq.Height, layouts[i % 4]);
            hex.Areas.CopyFrom(sq.Areas);
            hex.Space = new GridSpace(0, 0, 1 + rng.NextInt(3));
            var open = new List<Cell>();
            for (int y = 0; y < hex.Height; y++)
            {
                for (int x = 0; x < hex.Width; x++)
                {
                    if (hex.GetArea(x, y) != AreaCosts.Wall)
                    {
                        open.Add(new Cell(x, y));
                    }
                }
            }
            if (open.Count == 0)
            {
                continue;
            }
            var goals = new[] { open[rng.NextInt(open.Count)], open[rng.NextInt(open.Count)] };
            using GridFlowField f = hex.CreateFlowField(goals);
            for (int y = 0; y < hex.Height; y++)
            {
                for (int x = 0; x < hex.Width; x++)
                {
                    double want = double.PositiveInfinity;
                    if (hex.GetArea(x, y) != AreaCosts.Wall)
                    {
                        foreach (Cell goal in goals)
                        {
                            want = Math.Min(want, BruteForce.HexDijkstra(hex, new Cell(x, y), goal));
                        }
                    }
                    double got = f.GetCost(new Cell(x, y));
                    Assert.True(want == got, $"hex cell ({x},{y}): {got} vs {want}");
                    Cell[] path = f.GetPath(new Cell(x, y));
                    if (!double.IsPositiveInfinity(want))
                    {
                        Assert.Contains(path[^1], goals);
                    }
                }
            }
        }
    }

    private static void AssertSameField(GridFlowField a, GridFlowField b)
    {
        Assert.Equal(a.Costs.Length, b.Costs.Length);
        for (int i = 0; i < a.Costs.Length; i++)
        {
            Assert.True(BitConverter.DoubleToInt64Bits(a.Costs[i]) == BitConverter.DoubleToInt64Bits(b.Costs[i]), $"cost differs at {i}: {a.Costs[i]} vs {b.Costs[i]}");
            Assert.True(a.Next[i] == b.Next[i], $"direction differs at {i}: {a.Next[i]} vs {b.Next[i]}");
        }
    }

    [Fact]
    public void IncrementalRepairEqualsFullRecompute()
    {
        var rng = new SeededRandom(0x1ACE);
        for (int t = 0; t < 600; t++)
        {
            bool hexMode = t % 4 == 3;
            Grid sq = BruteForce.RandomGrid(ref rng, t % 20 == 0 ? 24 : 12, areas: t % 2 == 0);
            int size = (t % 6 == 1) ? 3 : (t % 6 == 2 ? 2 : 1);
            if (hexMode)
            {
                size = 1;
            }
            Connectivity conn = (t % 3 == 0) ? Connectivity.Four : Connectivity.Eight;
            var options = new GridOptions { AgentSize = size, Connectivity = conn, Diagonals = (DiagonalRule)(t % 3) };
            HexGrid? hex = hexMode ? HexGrid.From(sq.Cells, sq.Width, sq.Height, HexLayout.PointyOddRow) : null;
            if (hex is not null)
            {
                hex.Areas.CopyFrom(sq.Areas);
            }
            int goalCount = 1 + rng.NextInt(3);
            var goals = new Cell[goalCount];
            for (int i = 0; i < goalCount; i++)
            {
                goals[i] = new Cell(rng.NextInt(sq.Width), rng.NextInt(sq.Height));
            }
            using GridFlowField f = hex is not null ? hex.CreateFlowField(goals, options) : sq.CreateFlowField(goals, options);
            for (int round = 0; round < 6; round++)
            {
                int edits = 1 + rng.NextInt(round == 5 ? 12 : 3);
                for (int e = 0; e < edits; e++)
                {
                    int x = rng.NextInt(sq.Width);
                    int y = rng.NextInt(sq.Height);
                    byte area = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(4);
                    if (hex is not null)
                    {
                        hex.SetArea(x, y, area);
                    }
                    else
                    {
                        sq.SetArea(x, y, area);
                    }
                }
                if (round == 4)
                {
                    // An area multiplier change recomputes everything.
                    if (hex is not null)
                    {
                        hex.Areas[2] = 2.5;
                    }
                    else
                    {
                        sq.Areas[2] = 2.5;
                    }
                }
                f.Refresh(TestContext.Current.CancellationToken);
                using GridFlowField fresh = hex is not null ? hex.CreateFlowField(goals, options) : sq.CreateFlowField(goals, options);
                AssertSameField(f, fresh);
            }
        }
    }

    [Fact]
    public void SlicedComputationEqualsUnsliced()
    {
        var rng = new SeededRandom(0x51CE);
        for (int t = 0; t < 40; t++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 16, areas: t % 2 == 0);
            Cell[] goals = PickGoals(g, 1, ref rng);
            if (goals.Length == 0)
            {
                continue;
            }
            using GridFlowField whole = g.CreateFlowField(goals);
            foreach (int budget in new[] { 1, 2, 3, 7, 50 })
            {
                using GridFlowField sliced = g.CreateFlowField(goals, new GridOptions { IterationBudget = budget });
                int calls = 0;
                while (sliced.Status != FlowFieldStatus.Complete)
                {
                    sliced.Step(budget);
                    calls++;
                    Assert.True(calls < 100_000);
                }
                Assert.Equal(whole.Settled, sliced.Settled);
                AssertSameField(whole, sliced);
            }
        }
    }

    [Fact]
    public void FieldsComputedOnManyThreadsEqualSequentialOnes()
    {
        var rng = new SeededRandom(0x7EAD);
        Grid g = BruteForce.RandomGrid(ref rng, 60, areas: true);
        var goalSets = new Cell[64][];
        for (int i = 0; i < goalSets.Length; i++)
        {
            goalSets[i] = PickGoals(g, 1, ref rng);
        }
        double[][] seq = new double[goalSets.Length][];
        for (int i = 0; i < goalSets.Length; i++)
        {
            if (goalSets[i].Length > 0)
            {
                using GridFlowField f = g.CreateFlowField(goalSets[i]);
                seq[i] = f.Costs.ToArray();
            }
        }
        var par = new double[goalSets.Length][];
        Parallel.For(0, goalSets.Length, new ParallelOptions { MaxDegreeOfParallelism = 8, CancellationToken = TestContext.Current.CancellationToken }, i =>
        {
            if (goalSets[i].Length > 0)
            {
                using GridFlowField f = g.CreateFlowField(goalSets[i]);
                par[i] = f.Costs.ToArray();
            }
        });
        for (int i = 0; i < goalSets.Length; i++)
        {
            Assert.Equal(seq[i], par[i]);
        }
    }

    [Fact]
    public void BlockedFilterStepCostAndInvalidInput()
    {
        var g = new Grid(5, 1);
        using GridFlowField open = g.CreateFlowField([new Cell(0, 0)]);
        Assert.Equal(4.0, open.GetCost(new Cell(4, 0)));
        using GridFlowField blocked = g.CreateFlowField([new Cell(0, 0)], new GridOptions { Blocked = c => c.X == 2 });
        Assert.True(double.IsPositiveInfinity(blocked.GetCost(new Cell(4, 0))));
        Assert.Equal(1.0, blocked.GetCost(new Cell(1, 0)));
        using GridFlowField costly = g.CreateFlowField([new Cell(0, 0)], new GridOptions { StepCost = (_, _, c) => c * 3 });
        Assert.Equal(12.0, costly.GetCost(new Cell(4, 0)));
        Assert.True(open.GetDirection(new Cell(3, 0), out double dx, out double dy));
        Assert.Equal(-1.0, dx);
        Assert.Equal(0.0, dy);
        Assert.False(open.GetDirection(new Cell(0, 0), out _, out _));
        Assert.True(double.IsPositiveInfinity(open.GetCost(new Cell(9, 9))));
        Assert.Throws<ArgumentException>(() => g.CreateFlowField([]));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.CreateFlowField([new Cell(5, 0)]));
        Assert.Throws<ArgumentException>(() => new HexGrid(3, 3).CreateFlowField([new Cell(0, 0)], new GridOptions { AgentSize = 2 }));

        // A wall goal seeds nothing.
        g.SetWalkable(0, 0, false);
        using GridFlowField dead = g.CreateFlowField([new Cell(0, 0)]);
        Assert.True(double.IsPositiveInfinity(dead.GetCost(new Cell(3, 0))));

        // Moving the wall away repairs the field.
        g.SetWalkable(0, 0, true);
        dead.Refresh(TestContext.Current.CancellationToken);
        Assert.Equal(3.0, dead.GetCost(new Cell(3, 0)));

        // Large (still exact) world position lookup
        Assert.Throws<ArgumentOutOfRangeException>(() => open.GetCost(new WorldPoint(3e6, 0)));
        Assert.Equal(2.0, open.GetCost(new WorldPoint(2.5, 0.5)));
    }
}
