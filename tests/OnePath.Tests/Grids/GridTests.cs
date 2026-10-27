using System;
using System.Collections.Generic;
using System.Text;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class OpenSetTests
{
    [Fact]
    public void PopsInFThenLargerGThenIndexOrder()
    {
        var s = new OpenSet(100);
        s.Push(5, 10, 1);
        s.Push(3, 10, 4);
        s.Push(9, 10, 4);
        s.Push(1, 9, 0);
        s.Push(7, 10, 1);
        var order = new List<int>();
        while (s.Pop(out int n, out _, out _))
        {
            order.Add(n);
        }
        Assert.Equal([1, 3, 9, 5, 7], order);
    }

    [Fact]
    public void MatchesSortedReferenceUnderRandomUpdates()
    {
        var rng = new SeededRandom(42);
        for (int round = 0; round < 200; round++)
        {
            int n = 1 + rng.NextInt(200);
            var s = new OpenSet(n);
            var keys = new Dictionary<int, (double F, double G)>();
            for (int op = 0; op < 400; op++)
            {
                int node = rng.NextInt(n);
                double f = rng.NextInt(20);
                double g = rng.NextInt(5);
                s.Push(node, f, g);
                keys[node] = (f, g);
                if (rng.NextInt(5) == 0 && s.Count > 0)
                {
                    int top = s.PeekNode();
                    Assert.True(s.Pop(out int popped, out double pf, out double pg));
                    Assert.Equal(top, popped);
                    Assert.Equal(keys[popped], (pf, pg));
                    keys.Remove(popped);
                }
            }
            Assert.Equal(keys.Count, s.Count);
            var expect = new List<KeyValuePair<int, (double F, double G)>>(keys);
            expect.Sort((a, b) =>
            {
                int c = a.Value.F.CompareTo(b.Value.F);
                if (c != 0)
                {
                    return c;
                }
                c = b.Value.G.CompareTo(a.Value.G);
                return c != 0 ? c : a.Key.CompareTo(b.Key);
            });
            foreach (var kv in expect)
            {
                Assert.True(s.Pop(out int node, out _, out _));
                Assert.Equal(kv.Key, node);
            }
            Assert.False(s.Pop(out _, out _, out _));
        }
    }

    [Fact]
    public void ClearResetsMembership()
    {
        var s = new OpenSet(10);
        s.Push(2, 1, 1);
        s.Push(4, 1, 1);
        s.Clear();
        Assert.Equal(0, s.Count);
        Assert.False(s.Contains(2));
        s.Push(2, 5, 5);
        Assert.True(s.Contains(2));
    }

    [Fact]
    public void RejectsBadInput()
    {
        var s = new OpenSet(4);
        Assert.Throws<ArgumentOutOfRangeException>(() => s.Push(4, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => s.Push(-1, 1, 1));
        Assert.Throws<ArgumentException>(() => s.Push(0, double.NaN, 1));
        Assert.Throws<InvalidOperationException>(() => s.PeekNode());
    }
}

public class ClearanceAndAreaTests
{
    [Fact]
    public void ClearanceMatchesDefinition()
    {
        var rng = new SeededRandom(3);
        for (int t = 0; t < 300; t++)
        {
            Grid g = BruteForce.RandomGrid(ref rng, 15, areas: false);
            for (int y = 0; y < g.Height; y++)
            {
                for (int x = 0; x < g.Width; x++)
                {
                    int best = 0;
                    while (BruteForce.BlockFree(g, x, y, best + 1))
                    {
                        best++;
                    }
                    Assert.Equal(best, g.GetClearance(x, y));
                }
            }
        }
    }

    [Fact]
    public void ClearanceUpdatesAfterEdits()
    {
        var g = new Grid(5, 5);
        Assert.Equal(5, g.GetClearance(0, 0));
        g.SetWalkable(2, 2, false);
        Assert.Equal(2, g.GetClearance(0, 0));
        Assert.Equal(0, g.GetClearance(2, 2));
        g.SetWalkable(2, 2, true);
        Assert.Equal(5, g.GetClearance(0, 0));
    }

    [Fact]
    public void LargerAgentsNeedWiderGaps()
    {
        // A wall with a one-cell gap: size 1 passes, size 2 cannot.
        var g = new Grid(7, 5);
        for (int y = 0; y < 5; y++)
        {
            g.SetWalkable(3, y, y == 2);
        }
        Assert.True(g.FindPath(new Cell(0, 2), new Cell(6, 2)).Found);
        Assert.Equal(PathStatus.NoPath, g.FindPath(new Cell(0, 1), new Cell(5, 1), GridOptions.ForAgentSize(2)).Status);
    }

    [Fact]
    public void AreaMultipliersSteerThePath()
    {
        // A cheap road around an expensive swamp.
        var g = new Grid(5, 3);
        g.Areas[1] = 10.0;
        for (int x = 1; x < 4; x++)
        {
            g.SetArea(x, 1, 1);
        }
        GridPath p = g.FindPath(new Cell(0, 1), new Cell(4, 1), GridOptions.FourConnected);
        Assert.True(p.Found);
        Assert.Equal(6.0, p.Cost); // up, 4 along row 0, down: 6 unit steps at cost 1
    }

    [Fact]
    public void AreaTableValidation()
    {
        var a = new AreaCosts();
        Assert.Throws<ArgumentOutOfRangeException>(() => a[255] = 2);
        Assert.Throws<ArgumentOutOfRangeException>(() => a[1] = 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => a[1] = double.NaN);
        Assert.Throws<ArgumentOutOfRangeException>(() => a[1] = double.PositiveInfinity);
        a[1] = 0.25;
        Assert.Equal(0.25, a.MinCost);
    }

    [Fact]
    public void StepCostAndBlockedDelegates()
    {
        var g = new Grid(5, 1);
        GridPath plain = g.FindPath(new Cell(0, 0), new Cell(4, 0));
        Assert.Equal(4.0, plain.Cost);
        GridPath doubled = g.FindPath(new Cell(0, 0), new Cell(4, 0), new GridOptions { StepCost = (_, _, c) => c * 2 });
        Assert.Equal(8.0, doubled.Cost);
        GridPath blocked = g.FindPath(new Cell(0, 0), new Cell(4, 0), new GridOptions { Blocked = c => c.X == 2 });
        Assert.Equal(PathStatus.NoPath, blocked.Status);
        GridPath badGoal = g.FindPath(new Cell(0, 0), new Cell(4, 0), new GridOptions { Blocked = c => c.X == 4 });
        Assert.Equal(PathStatus.GoalInvalid, badGoal.Status);
    }

    [Fact]
    public void ChangeListenersHearEdits()
    {
        var g = new Grid(3, 3);
        var heard = new List<(int, int)>();
        var l = new Recorder(heard);
        g.AddListener(l);
        g.SetWalkable(1, 2, false);
        g.SetWalkable(1, 2, false); // no change, no call
        Assert.Equal([(1, 2)], heard);
        Assert.True(g.RemoveListener(l));
    }

    private sealed class Recorder(List<(int, int)> sink) : IGridChangeListener
    {
        public void OnCellChanged(int x, int y) => sink.Add((x, y));
    }
}

public class CoordinateAndApiTests
{
    [Fact]
    public void Sqrt2ConstantIsTheCorrectlyRoundedRoot()
    {
        Assert.Equal(BitConverter.DoubleToInt64Bits(Math.Sqrt(2.0)), BitConverter.DoubleToInt64Bits(GridMath.Sqrt2));
        Assert.Equal(0x3FF6A09E667F3BCDL, BitConverter.DoubleToInt64Bits(GridMath.Sqrt2));
        Assert.Equal(BitConverter.DoubleToInt64Bits(Math.Sqrt(3.0) / 2.0), BitConverter.DoubleToInt64Bits(GridMath.HalfSqrt3));
    }

    [Theory]
    [InlineData(2097152.5)]
    [InlineData(-2097153.0)]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    public void WorldCoordinatesBeyondTheBoundAreRejectedWithTheValueNamed(double bad)
    {
        var g = new Grid(4, 4);
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => g.FindPath(new WorldPoint(bad, 0), new WorldPoint(1, 1)));
        Assert.Contains("2^21", ex.Message, StringComparison.Ordinal);
        Assert.Contains(double.IsNaN(bad) ? "NaN" : bad.ToString("R", System.Globalization.CultureInfo.InvariantCulture), ex.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => new GridSpace(bad, 0, 1));
    }

    [Fact]
    public void BoundaryValuesAreAccepted()
    {
        var g = new Grid(4, 4) { Space = new GridSpace(-2097152.0, 2097148.0, 1.0) };
        GridPath p = g.FindPath(new WorldPoint(-2097152.0 + 0.5, 2097148.0 + 0.5), new WorldPoint(-2097152.0 + 3.5, 2097148.0 + 3.5));
        Assert.True(p.Found);
        Assert.Equal(4, p.WorldPoints.Length);
        Assert.Equal(new WorldPoint(-2097152.0 + 0.5, 2097148.0 + 0.5), p.WorldPoints[0]);
    }

    [Fact]
    public void WorldMappingRoundTrips()
    {
        var g = new Grid(10, 8) { Space = new GridSpace(100.25, -40.5, 2.5) };
        for (int y = 0; y < 8; y++)
        {
            for (int x = 0; x < 10; x++)
            {
                Assert.True(g.TryGetCell(g.CellCenter(new Cell(x, y)), out Cell c));
                Assert.Equal(new Cell(x, y), c);
            }
        }
        Assert.False(g.TryGetCell(new WorldPoint(99, -40), out _));
    }

    [Fact]
    public void InvalidCellsAreReportedNotThrown()
    {
        var g = Grid.From(new bool[,] { { true, false }, { true, true } });
        Assert.Equal(PathStatus.StartInvalid, g.FindPath(new Cell(-1, 0), new Cell(0, 0)).Status);
        Assert.Equal(PathStatus.GoalInvalid, g.FindPath(new Cell(0, 0), new Cell(1, 0)).Status);
        Assert.Equal(PathStatus.GoalInvalid, g.FindPath(new Cell(0, 0), new Cell(5, 5)).Status);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.GetArea(2, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Grid(0, 3));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Grid(1 << 20, 1 << 20));
        Assert.Throws<ArgumentOutOfRangeException>(() => new GridOptions { AgentSize = 0 });
        Assert.Throws<ArgumentException>(() => new HexGrid(2, 2).FindPath(new Cell(0, 0), new Cell(1, 1), GridOptions.ForAgentSize(2)));
    }

    [Fact]
    public void StartEqualsGoalCostsNothing()
    {
        var g = new Grid(3, 3);
        GridPath p = g.FindPath(new Cell(1, 1), new Cell(1, 1));
        Assert.True(p.Found);
        Assert.Equal(0.0, p.Cost);
        Assert.Equal([new Cell(1, 1)], p.Cells.ToArray());
    }

    [Fact]
    public void ExpansionsAreRecordedAsDebugData()
    {
        var g = new Grid(10, 10);
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(9, 9), new GridOptions { RecordExpansions = true });
        Assert.Equal(p.ExpandedCount, p.ExpandedCells.Length);
        Assert.Equal(new Cell(0, 0), p.ExpandedCells[0]);
        GridPath q = g.FindPath(new Cell(0, 0), new Cell(9, 9));
        Assert.Equal(0, q.ExpandedCells.Length);
    }

    [Fact]
    public void CornerCuttingRuleIsObserved()
    {
        // Two diagonal walls: only a squeeze or a cut could connect the corners.
        var g = Grid.From(new bool[,] { { true, false }, { false, true } });
        Assert.Equal(PathStatus.NoPath, g.FindPath(new Cell(0, 0), new Cell(1, 1)).Status);
        Assert.Equal(PathStatus.NoPath, g.FindPath(new Cell(0, 0), new Cell(1, 1), GridOptions.NoSqueezing).Status);
        GridPath all = g.FindPath(new Cell(0, 0), new Cell(1, 1), new GridOptions { Diagonals = DiagonalRule.AllowAll });
        Assert.True(all.Found);
        Assert.Equal(GridMath.Sqrt2, all.Cost);
    }

    [Fact]
    public void HexNeighboursAreAtOneSpacingAndMappingRoundTrips()
    {
        foreach (HexLayout layout in Enum.GetValues<HexLayout>())
        {
            var g = new HexGrid(7, 6, layout) { Space = new GridSpace(10, 20, 3) };
            for (int y = 0; y < 6; y++)
            {
                for (int x = 0; x < 7; x++)
                {
                    var c = new Cell(x, y);
                    Assert.True(g.TryGetCell(g.CellCenter(c), out Cell back));
                    Assert.Equal(c, back);
                    HexCoord a = g.ToAxial(c);
                    Assert.Equal(c, a.ToOffset(layout));
                    WorldPoint p = g.CellCenter(c);
                    for (int d = 0; d < 6; d++)
                    {
                        Cell n = new HexCoord(a.Q + HexDirs.Q[d], a.R + HexDirs.R[d]).ToOffset(layout);
                        if (!g.Contains(n.X, n.Y))
                        {
                            continue;
                        }
                        WorldPoint m = g.CellCenter(n);
                        double dist = Math.Sqrt(((p.X - m.X) * (p.X - m.X)) + ((p.Y - m.Y) * (p.Y - m.Y)));
                        Assert.InRange(dist, 3 - 1e-9, 3 + 1e-9);
                    }
                }
            }
        }
    }

    private static class HexDirs
    {
        public static readonly int[] Q = [1, 1, 0, -1, -1, 0];
        public static readonly int[] R = [0, -1, -1, 0, 1, 1];
    }

    [Fact]
    public void HexDistanceHeuristicIsExactOnEmptyGrids()
    {
        var g = new HexGrid(9, 9);
        GridPath p = g.FindPath(new Cell(0, 0), new Cell(8, 8));
        Assert.Equal(g.ToAxial(new Cell(0, 0)).DistanceTo(g.ToAxial(new Cell(8, 8))), p.Cost);
    }
}

public class MovingAiParserTests
{
    private const string Small = "type octile\nheight 3\nwidth 4\nmap\n....\n.@T.\nGSW.\n";

    [Fact]
    public void ParsesAMap()
    {
        MovingAiMap m = MovingAiMap.Parse(Small);
        Assert.Equal(4, m.Width);
        Assert.Equal(3, m.Height);
        Grid g = m.ToGrid();
        Assert.False(g.IsWalkable(1, 1));
        Assert.False(g.IsWalkable(2, 1));
        Assert.True(g.IsWalkable(0, 2));
        Assert.True(g.IsWalkable(1, 2)); // swamp
        Assert.False(g.IsWalkable(2, 2)); // water
        Assert.True(m.ToGrid(new MovingAiOptions { WaterWalkable = true, SwampWalkable = false }).IsWalkable(2, 2));
    }

    [Fact]
    public void ToleratesWindowsLineEndsAndTrailingBlankLines()
    {
        MovingAiMap m = MovingAiMap.Parse(Small.Replace("\n", "\r\n", StringComparison.Ordinal) + "\r\n\r\n");
        Assert.Equal(4, m.Width);
    }

    [Theory]
    [InlineData("")]
    [InlineData("type octile\nheight 2\nwidth 2\n")]
    [InlineData("type octile\nheight 2\nwidth 2\nmap\n..\n")]
    [InlineData("type octile\nheight 2\nwidth 2\nmap\n...\n..\n")]
    [InlineData("type octile\nheight 2\nwidth 2\nmap\n..\n.X\n")]
    [InlineData("type octile\nheight 0\nwidth 2\nmap\n")]
    [InlineData("type octile\nheight 99999999999\nwidth 2\nmap\n")]
    [InlineData("type octile\nheight 65536\nwidth 65536\nmap\n..\n")]
    [InlineData("type octile\nheight -1\nwidth 2\nmap\n")]
    [InlineData("type octile\nwidth 2\nmap\n..\n")]
    [InlineData("bogus 1\nmap\n")]
    [InlineData("type octile\nheight 1\nwidth 1\nmap\n.\nextra\n")]
    public void RejectsMalformedMapsWithATypedError(string text)
    {
        var ex = Assert.Throws<GridFormatException>(() => MovingAiMap.Parse(text));
        Assert.False(string.IsNullOrEmpty(ex.Message));
    }

    [Fact]
    public void ErrorsNameTheLine()
    {
        var ex = Assert.Throws<GridFormatException>(() => MovingAiMap.Parse("type octile\nheight 2\nwidth 2\nmap\n..\n.X\n"));
        Assert.Equal(6, ex.Line);
        Assert.Contains("line 6", ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void ParsesScenarios()
    {
        string text = "version 1\n0\tmaze.map\t512\t512\t1\t2\t3\t4\t5.65685425\n1 maze.map 512 512 10 20 30 40 12.5\n\n";
        ScenarioEntry[] s = MovingAiScenarios.Parse(text);
        Assert.Equal(2, s.Length);
        Assert.Equal(new ScenarioEntry(0, "maze.map", 512, 512, 1, 2, 3, 4, 5.65685425), s[0]);
        Assert.Equal(12.5, s[1].OptimalLength);
    }

    [Theory]
    [InlineData("")]
    [InlineData("version 2\n")]
    [InlineData("0\tm.map\t1\t1\t0\t0\t0\t0\t0\n")]
    [InlineData("version 1\n0\tm.map\t1\t1\t0\t0\t0\t0\n")]
    [InlineData("version 1\n0\tm.map\t1\t1\t0\t0\t0\t0\t0\t0\n")]
    [InlineData("version 1\nx\tm.map\t1\t1\t0\t0\t0\t0\t1.5\n")]
    [InlineData("version 1\n0\tm.map\t1\t1\t0\t0\t0\t0\tNaN\n")]
    [InlineData("version 1\n0\tm.map\t1\t1\t0\t0\t0\t0\t1e999\n")]
    [InlineData("version 1\n0\tm.map\t99999999999\t1\t0\t0\t0\t0\t1\n")]
    public void RejectsMalformedScenarios(string text)
    {
        Assert.Throws<GridFormatException>(() => MovingAiScenarios.Parse(text));
    }

    [Fact]
    public void NonAsciiTextIsRejected()
    {
        Assert.Throws<GridFormatException>(() => MovingAiMap.Parse("type octile\nheight 1\nwidth 1\nmap\n\u00e9\n"));
        Assert.Throws<GridFormatException>(() => MovingAiMap.Parse(Encoding.UTF8.GetBytes("type octile\nheight 1\nwidth 1\nmap\n\u00e9\n")));
    }

    [Fact]
    public void HostileRandomBytesNeverEscapeTheTypedError()
    {
        var rng = new SeededRandom(1234);
        string[] seeds = [Small, "version 1\n0\ta.map\t4\t3\t0\t0\t3\t2\t4.2\n"];
        for (int i = 0; i < 3000; i++)
        {
            byte[] data = Encoding.ASCII.GetBytes(seeds[i & 1]);
            int edits = 1 + rng.NextInt(4);
            for (int e = 0; e < edits; e++)
            {
                data[rng.NextInt(data.Length)] = (byte)rng.NextInt(256);
            }
            try
            {
                MovingAiMap.Parse(data);
            }
            catch (GridFormatException)
            {
            }
            try
            {
                MovingAiScenarios.Parse(data);
            }
            catch (GridFormatException)
            {
            }
        }
    }
}
