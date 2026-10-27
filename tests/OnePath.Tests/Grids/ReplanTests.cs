using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class ReplanTests
{
    private const double Tol = 1e-9;

    // ----- open list -----

    [Fact]
    public void OpenSetPopsInKeyOrderWithNodeIndexTies()
    {
        var rng = new SeededRandom(0x21A);
        for (int round = 0; round < 300; round++)
        {
            int n = 1 + rng.NextInt(40);
            var q = new ReplanOpenSet(n);
            var k1 = new double[n];
            var k2 = new double[n];
            var live = new bool[n];
            for (int op = 0; op < 200; op++)
            {
                int node = rng.NextInt(n);
                int what = rng.NextInt(4);
                if (what == 0)
                {
                    q.Remove(node);
                    live[node] = false;
                }
                else
                {
                    k1[node] = rng.NextInt(4);
                    k2[node] = rng.NextInt(3);
                    q.Set(node, k1[node], k2[node]);
                    live[node] = true;
                }
                Assert.Equal(live[node], q.Contains(node));
            }
            var order = new List<int>();
            for (int i = 0; i < n; i++)
            {
                if (live[i])
                {
                    order.Add(i);
                }
            }
            order.Sort((a, b) => k1[a] != k1[b] ? k1[a].CompareTo(k1[b]) : k2[a] != k2[b] ? k2[a].CompareTo(k2[b]) : a.CompareTo(b));
            Assert.Equal(order.Count, q.Count);
            foreach (int expect in order)
            {
                Assert.Equal(expect, q.Top);
                Assert.True(q.Remove(expect));
            }
            Assert.Equal(0, q.Count);
            Assert.True(double.IsPositiveInfinity(q.TopKey1));
        }
    }

    // ----- clearance and tile edits -----

    private static void AssertClearanceEqualsFresh(Grid g)
    {
        var fresh = Grid.From(g.Cells, g.Width, g.Height);
        Assert.True(fresh.Clearance.SequenceEqual(g.Clearance), "incremental clearance differs from a full recompute");
    }

    [Theory]
    [InlineData(0xC1UL, 1500, 30)]
    [InlineData(0xC2UL, 200, 90)]
    public void IncrementalClearanceEqualsFullRecompute(ulong seed, int sequences, int maxDim)
    {
        var rng = new SeededRandom(seed);
        for (int s = 0; s < sequences; s++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas: false);
            _ = g.Clearance;
            for (int step = 0; step < 8; step++)
            {
                int kind = rng.NextInt(4);
                if (kind == 0)
                {
                    g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(3) == 0 ? (byte)0 : AreaCosts.Wall);
                }
                else if (kind == 1)
                {
                    int x = rng.NextInt(g.Width);
                    int y = rng.NextInt(g.Height);
                    int w = rng.NextInt(g.Width - x + 1);
                    int h = rng.NextInt(g.Height - y + 1);
                    g.SetRect(x, y, w, h, rng.NextInt(2) == 0 ? (byte)0 : AreaCosts.Wall);
                }
                else if (kind == 2)
                {
                    int x = rng.NextInt(g.Width);
                    int y = rng.NextInt(g.Height);
                    int w = rng.NextInt(Math.Min(6, g.Width - x) + 1);
                    int h = rng.NextInt(Math.Min(6, g.Height - y) + 1);
                    var data = new byte[w * h];
                    for (int i = 0; i < data.Length; i++)
                    {
                        data[i] = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)0;
                    }
                    g.SetRect(x, y, w, h, data);
                }
                else
                {
                    // several edits before the next read: the dirty list has to cover all of them
                    for (int k = 0; k < 1 + rng.NextInt(40); k++)
                    {
                        g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(2) == 0 ? (byte)0 : AreaCosts.Wall);
                    }
                }
                if (rng.NextInt(3) != 0)
                {
                    AssertClearanceEqualsFresh(g);
                }
            }
            AssertClearanceEqualsFresh(g);
        }
    }

    [Fact]
    public void IncrementalClearanceReachesFarAcrossOpenGrid()
    {
        // Clearance reaches 255 on a big open grid: an edit has to be followed all the way up and left.
        var g = new Grid(300, 280);
        _ = g.Clearance;
        g.SetWalkable(299, 279, false);
        AssertClearanceEqualsFresh(g);
        g.SetWalkable(299, 279, true);
        AssertClearanceEqualsFresh(g);
        g.SetRect(100, 100, 20, 30, AreaCosts.Wall);
        AssertClearanceEqualsFresh(g);
        g.SetRect(0, 0, 300, 280, 0);
        AssertClearanceEqualsFresh(g);
    }

    private sealed class CountingListener : IGridChangeListener
    {
        public readonly List<Cell> Cells = [];

        public void OnCellChanged(int x, int y) => Cells.Add(new Cell(x, y));
    }

    private sealed class BatchListener : IGridBatchListener
    {
        public readonly List<(int X, int Y, int W, int H)> Rects = [];
        public int Singles;

        public void OnCellChanged(int x, int y) => Singles++;

        public void OnRectChanged(int x, int y, int width, int height) => Rects.Add((x, y, width, height));
    }

    [Fact]
    public void RectEditNotifiesOncePerBatchAndBumpsVersionOnce()
    {
        var g = new Grid(10, 8);
        g.SetWalkable(3, 3, false);
        var plain = new CountingListener();
        var batch = new BatchListener();
        g.AddListener(plain);
        g.AddListener(batch);
        int v = g.Version;
        g.SetRect(2, 2, 4, 3, AreaCosts.Wall);
        Assert.Equal(v + 1, g.Version);
        Assert.Equal(11, plain.Cells.Count); // 12 cells, (3,3) already a wall
        Assert.DoesNotContain(new Cell(3, 3), plain.Cells);
        Assert.Equal(new Cell(2, 2), plain.Cells[0]);
        Assert.Equal(new Cell(5, 4), plain.Cells[^1]);
        Assert.Equal(0, batch.Singles);
        Assert.Equal([(2, 2, 4, 3)], batch.Rects);

        g.SetRect(2, 2, 4, 3, AreaCosts.Wall); // nothing changes
        Assert.Equal(v + 1, g.Version);
        Assert.Single(batch.Rects);

        g.SetRect(0, 0, 0, 0, 0); // empty rectangles are allowed
        Assert.Equal(v + 1, g.Version);
    }

    [Fact]
    public void RectEditValidatesItsArguments()
    {
        var g = new Grid(5, 5);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetRect(-1, 0, 2, 2, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetRect(0, 0, 6, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetRect(4, 4, 2, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetRect(0, 0, -1, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetRect(int.MaxValue, 0, 2, 1, 0));
        Assert.Throws<ArgumentException>(() => g.SetRect(0, 0, 2, 2, new byte[3]));
        var h = new HexGrid(5, 5);
        Assert.Throws<ArgumentOutOfRangeException>(() => h.SetRect(3, 3, 3, 1, 0));
        Assert.Throws<ArgumentException>(() => h.SetRect(0, 0, 2, 2, new byte[5]));
    }

    [Fact]
    public void JpsPlusTablesAfterTileEditsEqualAFreshBuild()
    {
        var rng = new SeededRandom(0xC3);
        for (int s = 0; s < 400; s++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 28, areas: false);
            using var index = new JpsPlusIndex(g);
            for (int step = 0; step < 5; step++)
            {
                int x = rng.NextInt(g.Width);
                int y = rng.NextInt(g.Height);
                int w = rng.NextInt(Math.Min(5, g.Width - x) + 1);
                int h = rng.NextInt(Math.Min(5, g.Height - y) + 1);
                var data = new byte[w * h];
                for (int i = 0; i < data.Length; i++)
                {
                    data[i] = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)0;
                }
                if (rng.NextInt(2) == 0)
                {
                    g.SetRect(x, y, w, h, data);
                }
                else
                {
                    g.SetRect(x, y, w, h, AreaCosts.Wall);
                }
                index.Refresh();
                using var fresh = new JpsPlusIndex(Grid.From(g.Cells, g.Width, g.Height));
                Assert.True(fresh.Tables.SequenceEqual(index.Tables), "JPS+ tables after a tile edit differ from a fresh build");
            }
        }
    }

    // ----- replanning against from-scratch search -----

    private static void CheckPath(Grid g, GridPath p, Cell a, Cell b, GridOptions o, double expect)
    {
        Assert.True(p.Found, $"expected a path of cost {expect}, got {p.Status}");
        Assert.True(Math.Abs(p.Cost - expect) <= Tol * Math.Max(1.0, expect), $"cost {p.Cost} but brute force says {expect}");
        ReadOnlySpan<Cell> cells = p.Cells;
        Assert.Equal(a, cells[0]);
        Assert.Equal(b, cells[^1]);
        double sum = 0;
        for (int i = 1; i < cells.Length; i++)
        {
            Assert.True(BruteForce.LegalMove(g, o.AgentSize, o.Connectivity, o.Diagonals, cells[i - 1], cells[i]), "illegal step in path");
            sum += BruteForce.MoveCost(g, cells[i - 1], cells[i]);
        }
        Assert.True(Math.Abs(sum - p.Cost) <= Tol * Math.Max(1.0, sum), "path does not sum to its cost");
    }

    private static byte Pick(ref SeededRandom rng, bool areas) =>
        rng.NextInt(3) == 0 ? (byte)0 : rng.NextInt(2) == 0 ? AreaCosts.Wall : areas ? (byte)(1 + rng.NextInt(3)) : AreaCosts.Wall;

    private static void Edit(Grid g, ref SeededRandom rng, bool areas)
    {
        int kind = rng.NextInt(5);
        if (kind <= 2)
        {
            for (int k = 0; k < 1 + rng.NextInt(3); k++)
            {
                g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), Pick(ref rng, areas));
            }
        }
        else
        {
            int x = rng.NextInt(g.Width);
            int y = rng.NextInt(g.Height);
            int w = rng.NextInt(Math.Min(4, g.Width - x) + 1);
            int h = rng.NextInt(Math.Min(4, g.Height - y) + 1);
            if (kind == 3)
            {
                g.SetRect(x, y, w, h, Pick(ref rng, areas));
            }
            else
            {
                var data = new byte[w * h];
                for (int i = 0; i < data.Length; i++)
                {
                    data[i] = Pick(ref rng, areas);
                }
                g.SetRect(x, y, w, h, data);
            }
        }
    }

    private static Cell NextAlong(GridPath p, Cell current)
    {
        ReadOnlySpan<Cell> cells = p.Cells;
        if (cells.Length >= 2 && cells[0] == current)
        {
            return cells[1];
        }
        return current;
    }

    private static void RunSequences(ulong seed, int sequences, GridAlgorithm alg, Connectivity conn, DiagonalRule rule, int size, bool areas, int maxDim = 12, int steps = 10)
    {
        var rng = new SeededRandom(seed);
        int found = 0;
        int checks = 0;
        for (int s = 0; s < sequences; s++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, maxDim, areas);
            var o = new GridOptions { Algorithm = alg, Connectivity = conn, Diagonals = rule, AgentSize = size };
            List<Cell> starts = BruteForce.WalkableStarts(g, size);
            if (starts.Count < 2)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            using GridReplanner r = g.CreateReplanner(a, b, o);
            Assert.Equal(alg, r.Algorithm);
            for (int step = 0; step <= steps; step++)
            {
                if (step > 0)
                {
                    int mutations = 1 + rng.NextInt(2);
                    for (int m = 0; m < mutations; m++)
                    {
                        Edit(g, ref rng, areas);
                    }
                }
                GridPath p = r.Replan();
                Assert.Equal(alg, p.Algorithm);
                Cell cs = r.Start;
                Cell cg = r.Goal;
                checks++;
                if (!BruteForce.BlockFree(g, cs.X, cs.Y, size))
                {
                    Assert.Equal(PathStatus.StartInvalid, p.Status);
                }
                else if (!BruteForce.BlockFree(g, cg.X, cg.Y, size))
                {
                    Assert.Equal(PathStatus.GoalInvalid, p.Status);
                }
                else
                {
                    double expect = BruteForce.Dijkstra(g, size, conn, rule, cs, cg);
                    if (double.IsPositiveInfinity(expect))
                    {
                        Assert.Equal(PathStatus.NoPath, p.Status);
                    }
                    else
                    {
                        CheckPath(g, p, cs, cg, o, expect);
                        found++;
                    }
                    // Compare with the library's own A* too.
                    GridOptions plain = o.Clone();
                    plain.Algorithm = GridAlgorithm.AStar;
                    GridPath fresh = g.FindPath(cs, cg, plain);
                    Assert.Equal(fresh.Status, p.Status);
                    if (fresh.Found)
                    {
                        Assert.True(Math.Abs(fresh.Cost - p.Cost) <= Tol * Math.Max(1.0, fresh.Cost));
                    }
                }
                // the agent moves one step along the path (D* Lite keeps its work) or jumps (either mode)
                int mv = rng.NextInt(4);
                if (mv == 0 && p.Found)
                {
                    r.MoveStart(NextAlong(p, cs));
                }
                else if (mv == 1)
                {
                    r.MoveStart(starts[rng.NextInt(starts.Count)]);
                }
            }
        }
        Assert.True(found > checks / 4, $"too few successful checks ({found} of {checks}); the test is not exercising paths");
    }

    [Theory]
    [InlineData(0xA1UL, GridAlgorithm.DStarLite, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, false)]
    [InlineData(0xA2UL, GridAlgorithm.LpaStar, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, false)]
    [InlineData(0xA3UL, GridAlgorithm.DStarLite, Connectivity.Four, DiagonalRule.NoCornerCutting, 1, false)]
    [InlineData(0xA4UL, GridAlgorithm.LpaStar, Connectivity.Four, DiagonalRule.NoCornerCutting, 1, true)]
    [InlineData(0xA5UL, GridAlgorithm.DStarLite, Connectivity.Eight, DiagonalRule.NoSqueezing, 1, true)]
    [InlineData(0xA6UL, GridAlgorithm.LpaStar, Connectivity.Eight, DiagonalRule.AllowAll, 1, true)]
    [InlineData(0xA7UL, GridAlgorithm.DStarLite, Connectivity.Eight, DiagonalRule.NoCornerCutting, 2, false)]
    [InlineData(0xA8UL, GridAlgorithm.LpaStar, Connectivity.Eight, DiagonalRule.NoCornerCutting, 3, true)]
    [InlineData(0xA9UL, GridAlgorithm.DStarLite, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, true)]
    public void ReplannedCostsEqualFromScratchSearch(ulong seed, GridAlgorithm alg, Connectivity conn, DiagonalRule rule, int size, bool areas)
        => RunSequences(seed, 1200, alg, conn, rule, size, areas, maxDim: size > 1 ? 16 : 12);

    [Fact]
    public void ReplannedCostsEqualFromScratchOnLargerGrids()
    {
        RunSequences(0xAA, 120, GridAlgorithm.DStarLite, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, true, maxDim: 40, steps: 14);
        RunSequences(0xAB, 120, GridAlgorithm.LpaStar, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, false, maxDim: 40, steps: 14);
    }

    [Fact]
    public void SlicedReplanEqualsUnsliced()
    {
        var rng = new SeededRandom(0xB1);
        for (int s = 0; s < 300; s++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 20, areas: true);
            List<Cell> starts = BruteForce.WalkableStarts(g, 1);
            if (starts.Count < 2)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            GridAlgorithm alg = s % 2 == 0 ? GridAlgorithm.DStarLite : GridAlgorithm.LpaStar;
            using GridReplanner whole = g.CreateReplanner(a, b, new GridOptions { Algorithm = alg });
            using GridReplanner sliced = g.CreateReplanner(a, b, new GridOptions { Algorithm = alg });
            int budget = 1 + rng.NextInt(6);
            for (int round = 0; round < 4; round++)
            {
                if (round > 0)
                {
                    Edit(g, ref rng, true);
                }
                GridPath expect = whole.Replan();
                GridPath got;
                int calls = 0;
                int expanded = 0;
                do
                {
                    got = sliced.Replan(budget);
                    expanded += got.ExpandedCount;
                    calls++;
                    Assert.True(calls < 100_000);
                }
                while (got.Status == PathStatus.BudgetExhausted);
                Assert.Equal(expect.Status, got.Status);
                Assert.Equal(expect.Cost, got.Cost);
                Assert.True(expect.Cells.SequenceEqual(got.Cells));
                Assert.Equal(expect.ExpandedCount, expanded);
                Assert.Equal(whole.TotalIterations, sliced.TotalIterations);
            }
        }
    }

    [Fact]
    public void TileUpdatesAreRepairedLikeCellEdits()
    {
        // The same changes made as one tile edit or as single cells give the same answers.
        var rng = new SeededRandom(0xB2);
        for (int s = 0; s < 300; s++)
        {
            Grid g1 = BruteForce.RandomGrid(ref rng, 24, areas: true);
            var g2 = Grid.From(g1.Cells, g1.Width, g1.Height);
            g2.Areas.CopyFrom(g1.Areas);
            List<Cell> starts = BruteForce.WalkableStarts(g1, 1);
            if (starts.Count < 2)
            {
                continue;
            }
            Cell a = starts[rng.NextInt(starts.Count)];
            Cell b = starts[rng.NextInt(starts.Count)];
            using GridReplanner r1 = g1.CreateReplanner(a, b);
            using GridReplanner r2 = g2.CreateReplanner(a, b);
            _ = r1.Replan();
            _ = r2.Replan();
            for (int round = 0; round < 3; round++)
            {
                int x = rng.NextInt(g1.Width);
                int y = rng.NextInt(g1.Height);
                int w = rng.NextInt(Math.Min(7, g1.Width - x) + 1);
                int h = rng.NextInt(Math.Min(7, g1.Height - y) + 1);
                var data = new byte[w * h];
                for (int i = 0; i < data.Length; i++)
                {
                    data[i] = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3);
                }
                g1.SetRect(x, y, w, h, data);
                for (int j = 0; j < h; j++)
                {
                    for (int i = 0; i < w; i++)
                    {
                        g2.SetArea(x + i, y + j, data[(j * w) + i]);
                    }
                }
                GridPath p1 = r1.Replan();
                GridPath p2 = r2.Replan();
                Assert.Equal(p1.Status, p2.Status);
                Assert.True(Math.Abs(p1.Cost - p2.Cost) <= Tol * Math.Max(1.0, p1.Cost) || (double.IsInfinity(p1.Cost) && double.IsInfinity(p2.Cost)));
            }
        }
    }

    [Fact]
    public void DelegatesAndInvalidate()
    {
        var g = new Grid(12, 12);
        var blocked = new HashSet<Cell>();
        var o = new GridOptions { Blocked = c => blocked.Contains(c) };
        using GridReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(11, 0), o);
        GridPath p = r.Replan();
        Assert.True(p.Found);
        Assert.Equal(11.0, p.Cost, 9);
        for (int y = 0; y < 11; y++)
        {
            blocked.Add(new Cell(5, y));
            r.Invalidate(new Cell(5, y));
        }
        p = r.Replan();
        Assert.True(p.Found);
        Assert.Equal(g.FindPath(new Cell(0, 0), new Cell(11, 0), o).Cost, p.Cost, 9);
        Assert.True(p.Cost > 11.0);
        blocked.Add(new Cell(5, 11));
        r.Invalidate(new Cell(5, 11));
        Assert.Equal(PathStatus.NoPath, r.Replan().Status);
        blocked.Clear();
        r.InvalidateAll();
        Assert.Equal(11.0, r.Replan().Cost, 9);
        blocked.Add(new Cell(11, 0));
        Assert.Equal(PathStatus.GoalInvalid, r.Replan().Status);
    }

    [Fact]
    public void AreaTableChangesRestartTheSearch()
    {
        var g = new Grid(10, 10);
        for (int x = 0; x < 10; x++)
        {
            g.SetArea(x, 5, 1);
        }
        g.SetWalkable(9, 5, true);
        g.SetArea(9, 5, 1);
        using GridReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(0, 9));
        Assert.Equal(g.FindPath(new Cell(0, 0), new Cell(0, 9)).Cost, r.Replan().Cost, 9);
        g.Areas[1] = 4.0;
        Assert.Equal(g.FindPath(new Cell(0, 0), new Cell(0, 9)).Cost, r.Replan().Cost, 9);
        g.Areas[1] = 0.25;
        Assert.Equal(g.FindPath(new Cell(0, 0), new Cell(0, 9)).Cost, r.Replan().Cost, 9);
    }

    [Fact]
    public void ReplanningExpandsLessThanSearchingAgain()
    {
        // Wall toggles on a 64x64 map with scattered obstacles: the repair work is a fraction of fresh searches.
        var rng = new SeededRandom(0xB3);
        var g = new Grid(64, 64);
        for (int i = 0; i < 500; i++)
        {
            g.SetWalkable(rng.NextInt(64), rng.NextInt(64), false);
        }
        var a = new Cell(1, 1);
        var b = new Cell(62, 62);
        g.SetWalkable(a.X, a.Y, true);
        g.SetWalkable(b.X, b.Y, true);
        using GridReplanner r = g.CreateReplanner(a, b);
        GridPath p = r.Replan();
        long replanned = 0;
        long fresh = 0;
        for (int step = 0; step < 100; step++)
        {
            // toggle a cell near the current path
            Cell c = p.Found ? p.Cells[rng.NextInt(p.Cells.Length)] : a;
            int x = Math.Clamp(c.X + rng.NextInt(3) - 1, 0, 63);
            int y = Math.Clamp(c.Y + rng.NextInt(3) - 1, 0, 63);
            if ((x, y) != (a.X, a.Y) && (x, y) != (b.X, b.Y))
            {
                g.SetWalkable(x, y, !g.IsWalkable(x, y));
            }
            p = r.Replan();
            replanned += p.ExpandedCount;
            GridPath f = g.FindPath(a, b);
            fresh += f.ExpandedCount;
            Assert.Equal(f.Status, p.Status);
            if (f.Found)
            {
                Assert.True(Math.Abs(f.Cost - p.Cost) <= Tol * f.Cost);
            }
        }
        Assert.True(replanned < fresh, $"replanning expanded {replanned} nodes against {fresh} for searching again");
    }

    [Fact]
    public void ArgumentsAreValidated()
    {
        var g = new Grid(5, 5);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.CreateReplanner(new Cell(-1, 0), new Cell(1, 1)));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.CreateReplanner(new Cell(0, 0), new Cell(5, 1)));
        Assert.Throws<ArgumentException>(() => g.CreateReplanner(new Cell(0, 0), new Cell(1, 1), new GridOptions { HeuristicWeight = 2.0 }));
        using GridReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(4, 4));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.MoveStart(new Cell(5, 5)));
        Assert.Throws<ArgumentOutOfRangeException>(() => r.Replan(0));
        var h = new HexGrid(5, 5);
        Assert.Throws<ArgumentException>(() => h.CreateReplanner(new Cell(0, 0), new Cell(1, 1), new GridOptions { AgentSize = 2 }));
        r.Dispose();
        Assert.Throws<ObjectDisposedException>(() => r.Replan());
        // one-off queries with the replanning algorithms run A*
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(4, 4), new GridOptions { Algorithm = GridAlgorithm.DStarLite });
        Assert.True(p.Found);
        Assert.Equal(GridAlgorithm.AStar, p.Algorithm);
    }

    [Fact]
    public void StartEqualsGoalAndUnreachableGoals()
    {
        var g = new Grid(6, 6);
        for (int y = 0; y < 6; y++)
        {
            g.SetWalkable(3, y, false);
        }
        foreach (GridAlgorithm alg in new[] { GridAlgorithm.LpaStar, GridAlgorithm.DStarLite })
        {
            using GridReplanner same = g.CreateReplanner(new Cell(1, 1), new Cell(1, 1), new GridOptions { Algorithm = alg });
            GridPath p = same.Replan();
            Assert.True(p.Found);
            Assert.Equal(0.0, p.Cost);
            Assert.Single(p.Cells.ToArray());
            using GridReplanner cut = g.CreateReplanner(new Cell(0, 0), new Cell(5, 5), new GridOptions { Algorithm = alg });
            Assert.Equal(PathStatus.NoPath, cut.Replan().Status);
            g.SetWalkable(3, 2, true);
            Assert.True(cut.Replan().Found);
            g.SetWalkable(3, 2, false);
            Assert.Equal(PathStatus.NoPath, cut.Replan().Status);
        }
    }

    // ----- hex -----

    [Theory]
    [InlineData(0xE1UL, GridAlgorithm.DStarLite, HexLayout.PointyOddRow)]
    [InlineData(0xE2UL, GridAlgorithm.LpaStar, HexLayout.FlatEvenColumn)]
    [InlineData(0xE3UL, GridAlgorithm.DStarLite, HexLayout.PointyEvenRow)]
    [InlineData(0xE4UL, GridAlgorithm.LpaStar, HexLayout.FlatOddColumn)]
    public void HexReplannedCostsEqualFromScratchSearch(ulong seed, GridAlgorithm alg, HexLayout layout)
    {
        var rng = new SeededRandom(seed);
        int checks = 0;
        for (int s = 0; s < 150; s++)
        {
            int w = 3 + rng.NextInt(8);
            int h = 3 + rng.NextInt(8);
            var g = new HexGrid(w, h, layout);
            g.Areas[1] = 0.5;
            g.Areas[2] = 3.0;
            for (int i = 0; i < (w * h) / 5; i++)
            {
                g.SetArea(rng.NextInt(w), rng.NextInt(h), AreaCosts.Wall);
            }
            Cell a = new(rng.NextInt(w), rng.NextInt(h));
            Cell b = new(rng.NextInt(w), rng.NextInt(h));
            g.SetArea(a.X, a.Y, 0);
            g.SetArea(b.X, b.Y, 0);
            using HexReplanner r = g.CreateReplanner(a, b, new GridOptions { Algorithm = alg });
            for (int step = 0; step < 8; step++)
            {
                if (step > 0)
                {
                    if (rng.NextInt(3) == 0)
                    {
                        int x = rng.NextInt(w);
                        int y = rng.NextInt(h);
                        int rw = rng.NextInt(Math.Min(3, w - x) + 1);
                        int rh = rng.NextInt(Math.Min(3, h - y) + 1);
                        g.SetRect(x, y, rw, rh, rng.NextInt(2) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3));
                    }
                    else
                    {
                        g.SetArea(rng.NextInt(w), rng.NextInt(h), rng.NextInt(2) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3));
                    }
                }
                GridPath p = r.Replan();
                Cell cs = r.Start;
                Cell cg = r.Goal;
                checks++;
                if (g.GetArea(cs.X, cs.Y) == AreaCosts.Wall)
                {
                    Assert.Equal(PathStatus.StartInvalid, p.Status);
                }
                else if (g.GetArea(cg.X, cg.Y) == AreaCosts.Wall)
                {
                    Assert.Equal(PathStatus.GoalInvalid, p.Status);
                }
                else
                {
                    double expect = BruteForce.HexDijkstra(g, cs, cg);
                    if (double.IsPositiveInfinity(expect))
                    {
                        Assert.Equal(PathStatus.NoPath, p.Status);
                    }
                    else
                    {
                        Assert.True(p.Found);
                        Assert.True(Math.Abs(p.Cost - expect) <= Tol * Math.Max(1.0, expect), $"cost {p.Cost} but brute force says {expect}");
                        Assert.Equal(cs, p.Cells[0]);
                        Assert.Equal(cg, p.Cells[^1]);
                    }
                }
                if (rng.NextInt(3) == 0 && p.Found && p.Cells.Length > 1)
                {
                    r.MoveStart(p.Cells[1]);
                }
            }
        }
        Assert.True(checks > 0);
    }
}
