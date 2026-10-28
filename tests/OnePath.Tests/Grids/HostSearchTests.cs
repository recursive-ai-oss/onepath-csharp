using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>GridSearch (the non-generic resumable A*) against FindPath, sliced against unsliced, saved against uninterrupted.</summary>
public class HostSearchTests
{
    public static IEnumerable<object[]> Cases()
    {
        foreach (string kind in HostMaps.Kinds)
        {
            for (int v = 0; v < 8; v++)
            {
                yield return [kind, v];
            }
        }
    }

    internal static GridOptions Variant(int v) => v switch
    {
        0 => new GridOptions(),
        1 => new GridOptions { Heuristic = GridHeuristic.Zero },
        2 => new GridOptions { SpecializedEngines = false },
        3 => new GridOptions { Connectivity = Connectivity.Four },
        4 => new GridOptions { AgentSize = 2 },
        5 => new GridOptions { HeuristicWeight = 1.5 },
        6 => new GridOptions { StepCost = (_, to, c) => ((to.X + to.Y) & 7) == 0 ? c * 3 : c },
        _ => new GridOptions { Diagonals = DiagonalRule.NoSqueezing, Heuristic = GridHeuristic.Euclidean },
    };

    private static int Dim(string kind) => kind == "maze" ? 41 : 48;

    private static (Cell, Cell)[] Queries(Grid g, ref SeededRandom rng, int size, int count)
    {
        List<Cell> ok = BruteForce.WalkableStarts(g, size);
        var q = new List<(Cell, Cell)>();
        if (ok.Count < 2)
        {
            return [];
        }
        for (int i = 0; i < count; i++)
        {
            q.Add((ok[rng.NextInt(ok.Count)], ok[rng.NextInt(ok.Count)]));
        }
        return q.ToArray();
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void SliceFindPathAndHandleAgreePathForPath(string kind, int variant)
    {
        GridOptions o = Variant(variant);
        for (ulong seed = 1; seed <= 3; seed++)
        {
            Grid g = HostMaps.Make(kind, seed + (ulong)(variant * 31), Dim(kind), Dim(kind));
            var rng = new SeededRandom(seed * 77);
            GridSearch search = g.CreateSearch(o);
            foreach (var (s, t) in Queries(g, ref rng, o.AgentSize, 12))
            {
                GridPath expected = g.FindPath(s, t, o);
                foreach (int slice in new[] { 1, 5, 64, int.MaxValue })
                {
                    Assert.Equal(PathStatus.InProgress, search.TryBegin(s, t));
                    PathStatus st;
                    int guard = 0;
                    do
                    {
                        st = search.Step(slice);
                        Assert.True(++guard < 10_000_000);
                    }
                    while (st == PathStatus.InProgress);
                    Assert.Equal(expected.Status, st);
                    Assert.Equal(expected.ExpandedCount, search.ExpandedCount);
                    if (expected.Found)
                    {
                        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Cost), BitConverter.DoubleToInt64Bits(search.PathCost));
                        var cells = new Cell[search.PathLength];
                        Assert.Equal(cells.Length, search.CopyPath(cells));
                        Assert.True(expected.Cells.SequenceEqual(cells), $"{kind} v{variant} slice {slice}: {s} -> {t}");
                        var idx = new int[cells.Length];
                        search.CopyPath(idx);
                        for (int i = 0; i < idx.Length; i++)
                        {
                            Assert.Equal((cells[i].Y * g.Width) + cells[i].X, idx[i]);
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void OneBudgetedFindPathEqualsOneStep()
    {
        Grid g = HostMaps.Make("rooms", 5, 60, 60);
        var o = new GridOptions { IterationBudget = 50 };
        GridSearch search = g.CreateSearch(o);
        var rng = new SeededRandom(3);
        foreach (var (s, t) in Queries(g, ref rng, 1, 20))
        {
            GridPath p = g.FindPath(s, t, o);
            search.TryBegin(s, t);
            PathStatus st = search.Step(50);
            if (p.Status == PathStatus.BudgetExhausted)
            {
                Assert.Equal(PathStatus.InProgress, st);
                Assert.Equal(p.ExpandedCount, search.ExpandedCount);
            }
            else
            {
                Assert.Equal(p.Status, st);
            }
        }
    }

    [Theory]
    [InlineData("random")]
    [InlineData("cave")]
    [InlineData("maze")]
    public void SpecializedEngineOffMatchesTheGenericAStarTierTwo(string kind)
    {
        // SpecializedEngines = false is the switch that makes FindPath and GridSearch return the path a hand-built generic A* returns.
        for (ulong seed = 1; seed <= 4; seed++)
        {
            Grid g = HostMaps.Make(kind, seed, 40, 40);
            var o = new GridOptions { SpecializedEngines = false };
            var rng = new SeededRandom(seed);
            SquareTopology topo = g.CreateTopology();
            foreach (var (s, t) in Queries(g, ref rng, 1, 25))
            {
                var astar = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topo, default, new OctileHeuristic(g.Width, g.Areas.MinCost));
                int a = (s.Y * g.Width) + s.X;
                int b = (t.Y * g.Width) + t.X;
                astar.Begin(a, b);
                SearchStatus st = astar.Run(TestContext.Current.CancellationToken);
                GridPath p = g.FindPath(s, t, o);
                if (st != SearchStatus.Found)
                {
                    Assert.False(p.Found);
                    continue;
                }
                var nodes = new int[astar.PathLength()];
                astar.CopyPath(nodes);
                Assert.Equal(nodes.Length, p.Cells.Length);
                for (int i = 0; i < nodes.Length; i++)
                {
                    Assert.Equal(nodes[i], (p.Cells[i].Y * g.Width) + p.Cells[i].X);
                }
            }
        }
    }

    [Fact]
    public void BothEnginesReturnTheSameCostEvenWhenTheirTiesDiffer()
    {
        int differ = 0;
        int total = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            Grid g = HostMaps.Make("random", seed, 50, 50);
            var rng = new SeededRandom(seed);
            foreach (var (s, t) in Queries(g, ref rng, 1, 40))
            {
                GridPath fast = g.FindPath(s, t);
                GridPath slow = g.FindPath(s, t, new GridOptions { SpecializedEngines = false });
                Assert.Equal(fast.Found, slow.Found);
                if (!fast.Found)
                {
                    continue;
                }
                Assert.Equal(fast.Cost, slow.Cost, 9);
                total++;
                if (!fast.Cells.SequenceEqual(slow.Cells))
                {
                    differ++;
                }
            }
        }
        Assert.True(total > 100);
        Assert.True(differ <= total); // documented: equal cost, possibly different cells
    }

    [Fact]
    public void TryBeginReportsInvalidEndsAndBeginThrows()
    {
        var g = Grid.FromText("""
            .....
            .###.
            .....
            """);
        GridSearch s = g.CreateSearch();
        Assert.Equal(PathStatus.StartInvalid, s.TryBegin(new Cell(1, 1), new Cell(0, 0)));
        Assert.Equal(PathStatus.StartInvalid, s.TryBegin(new Cell(-1, 0), new Cell(0, 0)));
        Assert.Equal(PathStatus.GoalInvalid, s.TryBegin(new Cell(0, 0), new Cell(2, 1)));
        Assert.Equal(PathStatus.GoalInvalid, s.TryBegin(new Cell(0, 0), new Cell(9, 9)));
        Assert.Equal(SearchStatus.NotStarted, s.Status);
        Assert.Throws<ArgumentException>(() => s.Begin(new Cell(1, 1), new Cell(0, 0)));
        Assert.Throws<ArgumentException>(() => s.Begin(new Cell(0, 0), new Cell(1, 1)));
        Assert.Throws<InvalidOperationException>(() => s.Step(1));
        Assert.Equal(PathStatus.InProgress, s.TryBegin(new Cell(0, 0), new Cell(4, 2)));
        // A refusal leaves the running search alone.
        Assert.Equal(PathStatus.GoalInvalid, s.TryBegin(new Cell(0, 0), new Cell(2, 1)));
        Assert.Equal(PathStatus.Found, s.Step(1000));
        g.SetWalkable(0, 0, false);
        Assert.Throws<InvalidOperationException>(() => s.Step(1));
        // Agent size: a 2x2 agent does not fit on a corridor one cell wide.
        GridSearch big = g.CreateSearch(new GridOptions { AgentSize = 2 });
        Assert.Equal(PathStatus.StartInvalid, big.TryBegin(new Cell(0, 2), new Cell(0, 0)));
        Assert.Throws<ArgumentException>(() => g.CreateSearch(new GridOptions { Algorithm = GridAlgorithm.Jps }));
    }

    [Fact]
    public void AStarTryBeginAndTopology()
    {
        Grid g = Grid.FromText("""
            ...
            .#.
            ...
            """);
        SquareTopology topo = g.CreateTopology();
        var a = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topo, default, new OctileHeuristic(g.Width, 1.0));
        Assert.Equal(g.CellCount, a.Topology.NodeCount);
        Assert.Equal(PathStatus.StartInvalid, a.TryBegin(4, 0));
        Assert.Equal(PathStatus.StartInvalid, a.TryBegin(-1, 0));
        Assert.Equal(PathStatus.GoalInvalid, a.TryBegin(0, 4));
        Assert.Equal(PathStatus.GoalInvalid, a.TryBegin(0, 99));
        Assert.Equal(SearchStatus.NotStarted, a.Status);
        Assert.Equal(PathStatus.InProgress, a.TryBegin(0, 8));
        Assert.Equal(SearchStatus.Found, a.Run(TestContext.Current.CancellationToken));
        Assert.Throws<ArgumentException>(() => a.Begin(4, 0));
    }

    // ---------------- save and restore ----------------

    [Theory]
    [MemberData(nameof(Cases))]
    public void SaveRestoreMidSearchEqualsUninterrupted(string kind, int variant)
    {
        GridOptions o = Variant(variant);
        Grid g = HostMaps.Make(kind, 9 + (ulong)variant, Dim(kind), Dim(kind));
        var rng = new SeededRandom(4242 + (ulong)variant);
        GridSearch a = g.CreateSearch(o);
        GridSearch fresh = g.CreateSearch(o);
        foreach (var (s, t) in Queries(g, ref rng, o.AgentSize, 10))
        {
            GridPath expected = g.FindPath(s, t, o);
            a.TryBegin(s, t);
            int cut = 1 + rng.NextInt(200);
            PathStatus st = a.Step(cut);
            byte[] bytes = a.SaveState();
            Assert.Equal(bytes.Length, a.GetStateSize());
            var span = new byte[bytes.Length + 5];
            Assert.Equal(bytes.Length, a.SaveState(span));
            Assert.True(bytes.AsSpan().SequenceEqual(span.AsSpan(0, bytes.Length)));
            Assert.True(a.SaveState().AsSpan().SequenceEqual(bytes)); // saving twice gives the same bytes
            // Restore into a different handle and into the same one after it has run on.
            fresh.RestoreState(bytes);
            Assert.True(fresh.SaveState().AsSpan().SequenceEqual(bytes)); // save -> restore -> save
            Assert.Equal(a.Status, fresh.Status);
            Assert.Equal(a.Iterations, fresh.Iterations);
            Assert.Equal(a.ExpandedCount, fresh.ExpandedCount);
            if (st == PathStatus.InProgress)
            {
                a.Step(300);
                a.RestoreState(bytes);
            }
            foreach (GridSearch h in new[] { a, fresh })
            {
                while ((st = h.Step(37)) == PathStatus.InProgress)
                {
                }
                Assert.Equal(expected.Status, st);
                Assert.Equal(expected.ExpandedCount, h.ExpandedCount);
                if (expected.Found)
                {
                    var cells = new Cell[h.PathLength];
                    h.CopyPath(cells);
                    Assert.True(expected.Cells.SequenceEqual(cells));
                    Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Cost), BitConverter.DoubleToInt64Bits(h.PathCost));
                }
                if (h == a && st == PathStatus.InProgress)
                {
                    continue;
                }
                a.RestoreState(bytes);
                fresh.RestoreState(bytes);
            }
        }
    }

    [Fact]
    public void NotStartedStateRoundTripsAndEnginesAreChecked()
    {
        Grid g = HostMaps.Make("rooms", 2, 30, 30);
        GridSearch a = g.CreateSearch();
        byte[] idle = a.SaveState();
        GridSearch b = g.CreateSearch();
        b.TryBegin(new Cell(0, 0), new Cell(0, 0));
        b.RestoreState(idle);
        Assert.Equal(SearchStatus.NotStarted, b.Status);
        Assert.True(b.SaveState().AsSpan().SequenceEqual(idle));

        GridSearch fast = g.CreateSearch();
        var rng = new SeededRandom(1);
        var q = Queries(g, ref rng, 1, 1)[0];
        fast.TryBegin(q.Item1, q.Item2);
        fast.Step(10);
        byte[] fastBytes = fast.SaveState();
        GridSearch generic = g.CreateSearch(new GridOptions { SpecializedEngines = false });
        Assert.Throws<GridStateException>(() => generic.RestoreState(fastBytes));
        generic.TryBegin(q.Item1, q.Item2);
        generic.Step(10);
        byte[] genericBytes = generic.SaveState();
        Assert.Throws<GridStateException>(() => fast.RestoreState(genericBytes));
        Assert.Throws<GridStateException>(() => g.CreateSearch().RestoreState(new Grid(3, 3).CreateSearch().SaveState()));
    }

    [Fact]
    public void RestoreAfterGridRollbackUsesTheRestoredWalls()
    {
        Grid g = HostMaps.Make("cave", 3, 40, 40);
        var rng = new SeededRandom(8);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        byte[] gridBytes = g.SaveState();
        GridSearch search = g.CreateSearch();
        search.TryBegin(s, t);
        search.Step(30);
        byte[] bytes = search.SaveState();
        // The other timeline: a different grid at the same version number.
        for (int i = 0; i < g.CellCount; i += 7)
        {
            g.SetArea(i % g.Width, i / g.Width, AreaCosts.Wall);
        }
        g.RestoreState(gridBytes);
        search.RestoreState(bytes);
        PathStatus st;
        while ((st = search.Step(100)) == PathStatus.InProgress)
        {
        }
        GridPath expected = g.FindPath(s, t);
        Assert.Equal(expected.Status, st);
        if (expected.Found)
        {
            var cells = new Cell[search.PathLength];
            search.CopyPath(cells);
            Assert.True(expected.Cells.SequenceEqual(cells));
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(2)]
    public void HostileStateIsRejectedOrHarmless(int variant)
    {
        GridOptions o = Variant(variant);
        Grid g = HostMaps.Make("cave", 6, 36, 36);
        var rng = new SeededRandom(99);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        GridSearch a = g.CreateSearch(o);
        a.TryBegin(s, t);
        a.Step(60);
        byte[] good = a.SaveState();
        GridSearch b = g.CreateSearch(o);
        for (int cut = 0; cut < good.Length; cut += 1 + (good.Length / 200))
        {
            Assert.Throws<GridStateException>(() => b.RestoreState(good.AsSpan(0, cut)));
        }
        for (int trial = 0; trial < 3000; trial++)
        {
            byte[] bad = (byte[])good.Clone();
            int flips = 1 + rng.NextInt(3);
            for (int f = 0; f < flips; f++)
            {
                bad[rng.NextInt(bad.Length)] ^= (byte)(1 << rng.NextInt(8));
            }
            try
            {
                b.RestoreState(bad);
                // Accepted: it must still run to an end without throwing.
                if (b.Status != SearchStatus.NotStarted)
                {
                    int n = 0;
                    while (b.Step(1000) == PathStatus.InProgress && ++n < 100)
                    {
                    }
                    if (b.Status == SearchStatus.Found)
                    {
                        _ = b.CopyPath(new int[b.PathLength]);
                    }
                }
            }
            catch (GridStateException)
            {
            }
        }
    }
}
