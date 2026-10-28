using System;
using System.Collections.Generic;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>TryReplan/CopyPath, content hash, nearest walkable cell, large-agent fits and fixed-point costs.</summary>
public class HostApiTests
{
    // ---------------- replanner ----------------

    [Theory]
    [InlineData("random", 0)]
    [InlineData("rooms", 1)]
    [InlineData("cave", 2)]
    [InlineData("areas", 3)]
    public void TryReplanEqualsReplanThroughEdits(string kind, int algorithm)
    {
        var o = new GridOptions { Algorithm = algorithm % 2 == 0 ? GridAlgorithm.DStarLite : GridAlgorithm.LpaStar };
        for (ulong seed = 1; seed <= 4; seed++)
        {
            Grid g = HostMaps.Make(kind, seed, 40, 40);
            var rng = new SeededRandom(seed * 13);
            List<Cell> ok = BruteForce.WalkableStarts(g, 1);
            if (ok.Count < 4)
            {
                continue;
            }
            Cell s = ok[0];
            Cell t = ok[^1];
            using GridReplanner viaTry = g.CreateReplanner(s, t, o);
            using GridReplanner viaPath = g.CreateReplanner(s, t, o);
            for (int step = 0; step < 40; step++)
            {
                for (int e = 0, n = rng.NextInt(4); e < n; e++)
                {
                    Cell c = new(rng.NextInt(g.Width), rng.NextInt(g.Height));
                    if (c != s && c != t)
                    {
                        g.SetArea(c.X, c.Y, rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)0);
                    }
                }
                int slice = new[] { 1, 3, 40, int.MaxValue }[step % 4];
                PathStatus a;
                int guard = 0;
                while ((a = viaTry.TryReplan(slice)) == PathStatus.BudgetExhausted)
                {
                    Assert.True(++guard < 1_000_000);
                }
                GridPath b = viaPath.Replan();
                Assert.Equal(b.Status, a);
                if (a != PathStatus.Found)
                {
                    Assert.Equal(0, viaTry.PathLength);
                    Assert.Equal(double.PositiveInfinity, viaTry.PathCost);
                    continue;
                }
                Assert.Equal(BitConverter.DoubleToInt64Bits(b.Cost), BitConverter.DoubleToInt64Bits(viaTry.PathCost));
                var cells = new Cell[viaTry.PathLength];
                Assert.Equal(cells.Length, viaTry.CopyPath(cells));
                Assert.True(b.Cells.SequenceEqual(cells));
                var idx = new int[cells.Length];
                viaTry.CopyPath(idx);
                for (int i = 0; i < idx.Length; i++)
                {
                    Assert.Equal((cells[i].Y * g.Width) + cells[i].X, idx[i]);
                }
                Assert.Throws<ArgumentException>(() => viaTry.CopyPath(new int[cells.Length - 1]));
                // And the cost is optimal.
                Assert.Equal(BruteForce.Dijkstra(g, 1, Connectivity.Eight, DiagonalRule.NoCornerCutting, s, t), viaTry.PathCost, 6);
            }
        }
    }

    // ---------------- content hash ----------------

    [Fact]
    public void ContentHashDependsOnContentOnly()
    {
        Grid a = HostMaps.Make("rooms", 4, 30, 21);
        Grid b = Grid.From(a.Cells, a.Width, a.Height);
        Assert.Equal(a.ContentHash(), b.ContentHash());
        b.Space = new GridSpace(10, 20, 0.5); // the world mapping and the edit count are not content
        b.SetArea(0, 0, 3);
        b.SetArea(0, 0, a.GetArea(0, 0));
        Assert.NotEqual(a.Version, b.Version);
        Assert.Equal(a.ContentHash(), b.ContentHash());

        ulong h = a.ContentHash();
        byte old = a.GetArea(5, 5);
        a.SetArea(5, 5, old == 7 ? (byte)6 : (byte)7);
        Assert.NotEqual(h, a.ContentHash());
        a.SetArea(5, 5, old);
        Assert.Equal(h, a.ContentHash());
        a.Areas[3] = 2.5;
        Assert.NotEqual(h, a.ContentHash());
        a.Areas[3] = 1.0;
        Assert.Equal(h, a.ContentHash());

        // Same cells, other shape.
        Assert.NotEqual(new Grid(6, 4).ContentHash(), new Grid(4, 6).ContentHash());
        Assert.NotEqual(new Grid(1, 8).ContentHash(), new Grid(8, 1).ContentHash());

        // A rollback restores the hash.
        byte[] saved = a.SaveState();
        a.SetArea(1, 1, AreaCosts.Wall);
        Assert.NotEqual(h, a.ContentHash());
        a.RestoreState(saved);
        Assert.Equal(h, a.ContentHash());
    }

    [Fact]
    public void ContentHashKnownAnswers()
    {
        // Fixed values: a change here is a change to the documented definition.
        Assert.Equal(0xE1E0E8EB680D08EAUL, Grid.FromText(".#.\n#.#\n...").ContentHash());
        Assert.Equal(0xDD24E19E471827F3UL, new Grid(1, 1).ContentHash());
        var g = new Grid(9, 2);
        g.Areas[2] = 3.5;
        g.SetArea(8, 1, 2);
        Assert.Equal(0xFD577171B6444AF9UL, g.ContentHash());
    }

    // ---------------- nearest walkable, fits ----------------

    private static bool BruteNearest(Grid g, Cell around, int size, int radius, Func<Cell, bool>? blocked, out Cell best)
    {
        best = default;
        long bestD = long.MaxValue;
        bool found = false;
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                if (!BruteForce.BlockFree(g, x, y, size) || blocked?.Invoke(new Cell(x, y)) == true)
                {
                    continue;
                }
                long dx = x - (long)around.X;
                long dy = y - (long)around.Y;
                if (Math.Max(Math.Abs(dx), Math.Abs(dy)) > radius)
                {
                    continue;
                }
                long d = (dx * dx) + (dy * dy);
                if (d < bestD)
                {
                    bestD = d;
                    best = new Cell(x, y);
                    found = true;
                }
            }
        }
        return found;
    }

    [Fact]
    public void NearestWalkableMatchesBruteForce()
    {
        var rng = new SeededRandom(0x5EA7);
        for (int round = 0; round < 400; round++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 25, false);
            int size = 1 + rng.NextInt(3);
            int radius = rng.NextInt(4) == 0 ? rng.NextInt(6) : int.MaxValue;
            Func<Cell, bool>? blocked = rng.NextInt(3) == 0 ? c => ((c.X * 7) + c.Y) % 5 == 0 : null;
            var o = new GridOptions { AgentSize = size, NearestRadius = radius, Blocked = blocked };
            for (int q = 0; q < 15; q++)
            {
                var around = new Cell(rng.NextInt(g.Width + 24) - 12, rng.NextInt(g.Height + 24) - 12);
                bool expected = BruteNearest(g, around, size, radius, blocked, out Cell want);
                bool got = g.TryFindNearestWalkable(around, out Cell have, o);
                Assert.Equal(expected, got);
                if (expected)
                {
                    // Equal distance is allowed to differ only by the documented tie rule, which brute force applies by scan order.
                    Assert.Equal(want, have);
                }
                else
                {
                    Assert.Equal(default, have);
                }
            }
        }
    }

    [Fact]
    public void NearestWalkableWorldPointAndEdges()
    {
        var g = Grid.FromText("""
            ###..
            ###..
            ###..
            """);
        g.Space = new GridSpace(100, 200, 2.0);
        Assert.True(g.TryFindNearestWalkable(new WorldPoint(101, 201), out Cell c)); // inside cell (0,0), a wall
        Assert.Equal(new Cell(3, 0), c);
        Assert.True(g.TryFindNearestWalkable(new WorldPoint(-1_000_000, 5_000), out c)); // far outside
        Assert.Equal(new Cell(3, 2), c);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.TryFindNearestWalkable(new WorldPoint(3e6, 0), out _));
        Assert.True(g.TryFindNearestWalkable(new Cell(int.MinValue, int.MaxValue), out c));
        var walls = new Grid(3, 3);
        walls.SetRect(0, 0, 3, 3, AreaCosts.Wall);
        Assert.False(walls.TryFindNearestWalkable(new Cell(1, 1), out c));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GridOptions { NearestRadius = -1 });
    }

    [Fact]
    public void CanFitAndMaxAgentSizeMatchClearance()
    {
        var rng = new SeededRandom(77);
        for (int round = 0; round < 100; round++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 20, false);
            for (int y = -1; y <= g.Height; y++)
            {
                for (int x = -1; x <= g.Width; x++)
                {
                    var c = new Cell(x, y);
                    int clear = g.MaxAgentSizeAt(c);
                    Assert.Equal(g.Contains(x, y) ? g.GetClearance(x, y) : 0, clear);
                    for (int size = 1; size <= 4; size++)
                    {
                        bool expected = g.Contains(x, y) && BruteForce.BlockFree(g, x, y, size);
                        Assert.Equal(expected, g.CanFit(c, size));
                        // It is the test FindPath applies to its end cells.
                        if (g.Contains(x, y) && g.Contains(0, 0))
                        {
                            GridPath p = g.FindPath(c, c, new GridOptions { AgentSize = size });
                            Assert.Equal(expected, p.Status != PathStatus.StartInvalid);
                        }
                    }
                }
            }
        }
        Assert.Throws<ArgumentOutOfRangeException>(() => new Grid(2, 2).CanFit(new Cell(0, 0), 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Grid(2, 2).CanFit(new Cell(0, 0), 256));
    }

    // ---------------- fixed-point cost ----------------

    [Fact]
    public void FixedCostIsTheRoundedSumOfStepCosts()
    {
        const double Q = 4294967296.0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            Grid g = HostMaps.Make(seed % 2 == 0 ? "areas" : "cave", seed, 40, 40);
            var rng = new SeededRandom(seed);
            List<Cell> ok = BruteForce.WalkableStarts(g, 1);
            for (int q = 0; q < 30 && ok.Count > 1; q++)
            {
                GridPath p = g.FindPath(ok[rng.NextInt(ok.Count)], ok[rng.NextInt(ok.Count)]);
                if (!p.Found)
                {
                    continue;
                }
                Assert.True(g.TryGetFixedCost(p.Cells, out long fixedCost));
                long manual = 0;
                for (int i = 1; i < p.Cells.Length; i++)
                {
                    double m = (g.Areas[g.GetArea(p.Cells[i - 1].X, p.Cells[i - 1].Y)] + g.Areas[g.GetArea(p.Cells[i].X, p.Cells[i].Y)]) * 0.5;
                    bool diag = p.Cells[i].X != p.Cells[i - 1].X && p.Cells[i].Y != p.Cells[i - 1].Y;
                    manual += (long)Math.Round((diag ? 1.4142135623730951 * m : m) * Q, MidpointRounding.ToEven);
                }
                Assert.Equal(manual, fixedCost);
                Assert.Equal(p.Cost, fixedCost / Q, 6); // agrees with the double cost to rounding
                Assert.True(p.TryGetStepCounts(out int straight, out int diagonal));
                Assert.Equal(p.Cells.Length - 1, straight + diagonal);
            }
        }
    }

    [Fact]
    public void FixedCostOfAUniformGridIsStepCountsTimesTheStepCosts()
    {
        var g = new Grid(30, 30);
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(20, 7));
        Assert.True(p.TryGetStepCounts(out int s, out int d));
        Assert.Equal(13, s);
        Assert.Equal(7, d);
        Assert.True(g.TryGetFixedCost(p.Cells, out long cost));
        Assert.Equal((13L * (1L << 32)) + (7L * 6074001000L), cost); // round(√2 · 2^32) = 6074001000
    }

    [Fact]
    public void FixedCostRefusesWhatItCannotPrice()
    {
        Grid g = Grid.FromText("..#\n...\n...");
        Assert.True(g.TryGetFixedCost([], out long c));
        Assert.Equal(0, c);
        Assert.True(g.TryGetFixedCost([new Cell(0, 0)], out c));
        Assert.Equal(0, c);
        Assert.False(g.TryGetFixedCost([new Cell(0, 0), new Cell(2, 0)], out c)); // two cells apart
        Assert.False(g.TryGetFixedCost([new Cell(0, 0), new Cell(0, 0)], out c)); // no move
        Assert.False(g.TryGetFixedCost([new Cell(1, 0), new Cell(2, 0)], out c)); // into a wall
        Assert.False(g.TryGetFixedCost([new Cell(0, 0), new Cell(-1, 0)], out c)); // outside
        GridPath any = g.FindPath(new Cell(0, 0), new Cell(1, 2), new GridOptions { Algorithm = GridAlgorithm.ThetaStar });
        if (!any.TryGetStepCounts(out int s, out int dg))
        {
            Assert.Equal((0, 0), (s, dg));
        }
    }
}
