using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>GridSearch with JPS and JPS+: the same paths as FindPath however it is sliced, saved and restored, and states that are rejected when they cannot be run from.</summary>
public class HostJumpSearchTests
{
    public static IEnumerable<object[]> Cases()
    {
        foreach (string kind in HostMaps.Kinds)
        {
            for (int v = 0; v < 9; v++)
            {
                yield return [kind, v];
            }
        }
    }

    // 0-2 and 8-9 run the specialised octile engines, 3-5 the generic jump point search, 6-7 fall back to plain A* as FindPath does.
    internal static GridOptions Variant(int v) => v switch
    {
        0 => new GridOptions { Algorithm = GridAlgorithm.Jps },
        1 => new GridOptions { Algorithm = GridAlgorithm.JpsPlus },
        2 => new GridOptions { Algorithm = GridAlgorithm.Jps, Heuristic = GridHeuristic.Zero },
        3 => new GridOptions { Algorithm = GridAlgorithm.Jps, SpecializedEngines = false },
        4 => new GridOptions { Algorithm = GridAlgorithm.JpsPlus, SpecializedEngines = false },
        5 => new GridOptions { Algorithm = GridAlgorithm.Jps, Heuristic = GridHeuristic.Euclidean },
        6 => new GridOptions { Algorithm = GridAlgorithm.Jps, Connectivity = Connectivity.Four },
        7 => new GridOptions { Algorithm = GridAlgorithm.JpsPlus, AgentSize = 2 },
        _ => new GridOptions { Algorithm = GridAlgorithm.JpsPlus, Heuristic = GridHeuristic.Zero, SpecializedEngines = false },
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

    private static PathStatus Finish(GridSearch s, int slice)
    {
        PathStatus st;
        int guard = 0;
        do
        {
            st = s.Step(slice);
            Assert.True(++guard < 10_000_000);
        }
        while (st == PathStatus.InProgress);
        return st;
    }

    private static void AssertSame(GridPath expected, GridSearch h, PathStatus st, string what)
    {
        Assert.Equal(expected.Status, st);
        Assert.Equal(expected.ExpandedCount, h.ExpandedCount);
        if (!expected.Found)
        {
            Assert.Equal(double.PositiveInfinity, h.PathCost);
            return;
        }
        Assert.Equal(BitConverter.DoubleToInt64Bits(expected.Cost), BitConverter.DoubleToInt64Bits(h.PathCost));
        var cells = new Cell[h.PathLength];
        Assert.Equal(cells.Length, h.CopyPath(cells));
        Assert.True(expected.Cells.SequenceEqual(cells), what);
        var idx = new int[cells.Length];
        h.CopyPath(idx);
        for (int i = 0; i < idx.Length; i++)
        {
            Assert.Equal((cells[i].Y * h.Grid.Width) + cells[i].X, idx[i]);
        }
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void SlicedJumpSearchEqualsFindPathPathForPath(string kind, int variant)
    {
        GridOptions o = Variant(variant);
        for (ulong seed = 1; seed <= 3; seed++)
        {
            Grid g = HostMaps.Make(kind, seed + (ulong)(variant * 31), Dim(kind), Dim(kind));
            var rng = new SeededRandom(seed * 77);
            GridSearch search = g.CreateSearch(o);
            foreach (var (s, t) in Queries(g, ref rng, o.AgentSize, 12))
            {
                GridPath expected = g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken);
                foreach (int slice in new[] { 1, 3, 64, int.MaxValue })
                {
                    Assert.Equal(PathStatus.InProgress, search.TryBegin(s, t));
                    AssertSame(expected, search, Finish(search, slice), $"{kind} v{variant} slice {slice}: {s} -> {t}");
                }
            }
        }
    }

    [Fact]
    public void JumpSearchesSelectTheEngineFindPathUses()
    {
        Grid g = HostMaps.Make("rooms", 4, 40, 40);
        Assert.True(g.CreateSearch(Variant(0)).UsesSpecializedEngine);
        Assert.True(g.CreateSearch(Variant(1)).UsesSpecializedEngine);
        Assert.False(g.CreateSearch(Variant(3)).UsesSpecializedEngine);
        // Begin the search so the engine is chosen: a search the jump rules cannot serve runs plain A*, as FindPath does.
        GridSearch four = g.CreateSearch(Variant(6));
        var rng = new SeededRandom(1);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        four.TryBegin(s, t);
        Assert.False(four.UsesSpecializedEngine);
        Assert.Throws<ArgumentException>(() => g.CreateSearch(new GridOptions { Algorithm = GridAlgorithm.ThetaStar }));
    }

    [Fact]
    public void JumpIterationsCountPopsOfJumpPoints()
    {
        // One iteration is one pop, so a budget of n pops stops after exactly n iterations unless the search ends first.
        Grid g = HostMaps.Make("cave", 2, 48, 48);
        var rng = new SeededRandom(5);
        foreach (int v in new[] { 0, 1, 3, 4 })
        {
            GridSearch s = g.CreateSearch(Variant(v));
            foreach (var (a, b) in Queries(g, ref rng, 1, 6))
            {
                s.TryBegin(a, b);
                int total = 0;
                PathStatus st;
                while ((st = s.Step(4)) == PathStatus.InProgress)
                {
                    total += 4;
                    Assert.Equal(total, s.Iterations);
                }
                Assert.True(s.Iterations - total <= 4 && s.Iterations >= total);
            }
        }
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void SaveRestoreMidJumpSearchEqualsUninterrupted(string kind, int variant)
    {
        GridOptions o = Variant(variant);
        Grid g = HostMaps.Make(kind, 9 + (ulong)variant, Dim(kind), Dim(kind));
        var rng = new SeededRandom(4242 + (ulong)variant);
        GridSearch a = g.CreateSearch(o);
        GridSearch fresh = g.CreateSearch(o);
        foreach (var (s, t) in Queries(g, ref rng, o.AgentSize, 10))
        {
            GridPath expected = g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken);
            a.TryBegin(s, t);
            int cut = 1 + rng.NextInt(60);
            PathStatus st = a.Step(cut);
            byte[] bytes = a.SaveState();
            Assert.Equal(bytes.Length, a.GetStateSize());
            var span = new byte[bytes.Length + 5];
            Assert.Equal(bytes.Length, a.SaveState(span));
            Assert.True(bytes.AsSpan().SequenceEqual(span.AsSpan(0, bytes.Length)));
            Assert.True(a.SaveState().AsSpan().SequenceEqual(bytes)); // saving twice gives the same bytes
            Assert.Throws<ArgumentException>(() => a.SaveState(new byte[bytes.Length - 1]));
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
                st = h.Status;
                while (st == PathStatus.InProgress)
                {
                    st = h.Step(7);
                }
                Assert.True(expected.ExpandedCount == h.ExpandedCount, $"{kind} v{variant}: {s} -> {t} cut {cut} first {(h == a ? "a" : "fresh")} iters {h.Iterations} expected {expected.ExpandedCount} got {h.ExpandedCount} status {expected.Status}");
                AssertSame(expected, h, st, $"{kind} v{variant}: {s} -> {t} cut {cut}");
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4)]
    public void StateOfAFinishedAndAnIdleJumpSearchRoundTrips(int variant)
    {
        GridOptions o = Variant(variant);
        Grid g = HostMaps.Make("rooms", 2, 40, 40);
        GridSearch a = g.CreateSearch(o);
        byte[] idle = a.SaveState();
        var rng = new SeededRandom(2);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        GridSearch b = g.CreateSearch(o);
        b.TryBegin(s, s);
        b.RestoreState(idle);
        Assert.Equal(PathStatus.None, b.Status);
        Assert.True(b.SaveState().AsSpan().SequenceEqual(idle));
        a.TryBegin(s, t);
        PathStatus st = Finish(a, 50);
        byte[] done = a.SaveState();
        b.RestoreState(done);
        Assert.Equal(a.Status, b.Status);
        Assert.True(b.SaveState().AsSpan().SequenceEqual(done));
        if (st == PathStatus.Found)
        {
            Assert.Equal(BitConverter.DoubleToInt64Bits(a.PathCost), BitConverter.DoubleToInt64Bits(b.PathCost));
        }
    }

    [Fact]
    public void StatesOfOtherEnginesAreRefused()
    {
        Grid g = HostMaps.Make("cave", 3, 40, 40);
        var rng = new SeededRandom(1);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        var saved = new Dictionary<int, byte[]>();
        foreach (int v in new[] { 0, 1, 3, 4 })
        {
            GridSearch x = g.CreateSearch(Variant(v));
            x.TryBegin(s, t);
            x.Step(5);
            saved[v] = x.SaveState();
        }
        GridSearch astar = g.CreateSearch();
        astar.TryBegin(s, t);
        astar.Step(5);
        byte[] astarBytes = astar.SaveState();
        foreach (int target in new[] { 0, 1, 3, 4 })
        {
            GridSearch x = g.CreateSearch(Variant(target));
            Assert.Throws<GridStateException>(() => x.RestoreState(astarBytes));
            foreach (int from in new[] { 0, 1, 3, 4 })
            {
                if (from == target)
                {
                    x.RestoreState(saved[from]);
                    continue;
                }
                if (from + target == 7)
                {
                    // The generic block-map and JPS+ engines write the same kind of state and find the same jump points, so each takes the other's.
                    x.RestoreState(saved[from]);
                    continue;
                }
                // JPS and JPS+ over the specialised engines differ in the finder byte; the generic ones in the embedded kind or by mode.
                Assert.Throws<GridStateException>(() => x.RestoreState(saved[from]));
            }
        }
        // An A* search must not take a jump state either.
        Assert.Throws<GridStateException>(() => astar.RestoreState(saved[0]));
        Assert.Throws<GridStateException>(() => astar.RestoreState(saved[3]));
    }

    [Fact]
    public void RestoreAfterGridRollbackUsesTheRestoredWalls()
    {
        foreach (int v in new[] { 0, 1, 3, 4 })
        {
            Grid g = HostMaps.Make("cave", 3, 40, 40);
            var rng = new SeededRandom(8);
            var (s, t) = Queries(g, ref rng, 1, 1)[0];
            GridOptions o = Variant(v);
            byte[] gridBytes = g.SaveState();
            GridSearch search = g.CreateSearch(o);
            search.TryBegin(s, t);
            search.Step(12);
            byte[] bytes = search.SaveState();
            // The other timeline: a different grid at the same version number, searched meanwhile.
            for (int i = 0; i < g.CellCount; i += 7)
            {
                g.SetArea(i % g.Width, i / g.Width, AreaCosts.Wall);
            }
            _ = g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken);
            g.RestoreState(gridBytes);
            search.RestoreState(bytes);
            PathStatus st = Finish(search, 9);
            AssertSame(g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken), search, st, $"variant {v}");
            // A new search after the rollback follows the restored grid too.
            search.TryBegin(s, t);
            AssertSame(g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken), search, Finish(search, 100), $"variant {v} again");
        }
    }

    [Fact]
    public void EditsBetweenSearchesAreSeenByTheReusedEngines()
    {
        foreach (int v in new[] { 0, 1, 3, 4 })
        {
            Grid g = HostMaps.Make("rooms", 6, 48, 48);
            var rng = new SeededRandom(11);
            GridOptions o = Variant(v);
            GridSearch search = g.CreateSearch(o);
            for (int round = 0; round < 12; round++)
            {
                g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), (round & 1) == 0 ? AreaCosts.Wall : (byte)0);
                foreach (var (s, t) in Queries(g, ref rng, 1, 4))
                {
                    search.TryBegin(s, t);
                    AssertSame(g.FindPath(s, t, o, cancellationToken: TestContext.Current.CancellationToken), search, Finish(search, 13), $"variant {v} round {round}");
                }
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4)]
    public void HostileJumpStateIsRejectedOrHarmless(int variant)
    {
        GridOptions o = Variant(variant);
        Grid g = HostMaps.Make("cave", 6, 36, 36);
        var rng = new SeededRandom(99);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        GridSearch a = g.CreateSearch(o);
        a.TryBegin(s, t);
        a.Step(15);
        byte[] good = a.SaveState();
        GridSearch b = g.CreateSearch(o);
        for (int cut = 0; cut < good.Length; cut += 1 + (good.Length / 200))
        {
            Assert.Throws<GridStateException>(() => b.RestoreState(good.AsSpan(0, cut)));
        }
        for (int trial = 0; trial < 4000; trial++)
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
                if (b.Status != PathStatus.None)
                {
                    int n = 0;
                    while (b.Step(1000) == PathStatus.InProgress && ++n < 100)
                    {
                    }
                    if (b.Status == PathStatus.Found)
                    {
                        _ = b.CopyPath(new int[b.PathLength]);
                        _ = b.CopyPath(new Cell[b.PathLength]);
                    }
                }
            }
            catch (GridStateException)
            {
            }
        }
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void SpecialisedJumpStateThatStepCannotRunFromIsRejected(int variant)
    {
        // Layout of the state: 17 header bytes, the finder byte, then status (offset 18), zero-estimate flag, start, goal, expanded,
        // iterations, pathA, pathB, visited (7 x i32 from offset 20), then 17-byte records (cell, a, b, parent, flags), then the open list.
        Grid g = HostMaps.Make("cave", 6, 36, 36);
        var rng = new SeededRandom(7);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        GridSearch a = g.CreateSearch(Variant(variant));
        a.TryBegin(s, t);
        a.Step(6);
        Assert.Equal(PathStatus.InProgress, a.Status);
        byte[] running = a.SaveState();
        Assert.Equal((byte)SearchStatus.Running, running[18]);
        GridSearch b = g.CreateSearch(Variant(variant));
        b.RestoreState(running);

        // Found without the goal closed.
        byte[] bad = (byte[])running.Clone();
        bad[18] = (byte)SearchStatus.Found;
        Assert.Throws<GridStateException>(() => b.RestoreState(bad));

        // NotFound with open cells.
        bad = (byte[])running.Clone();
        bad[18] = (byte)SearchStatus.NotFound;
        Assert.Throws<GridStateException>(() => b.RestoreState(bad));

        // A finished search whose status says Running while the goal is closed.
        a.TryBegin(s, t);
        Finish(a, 1000);
        if (a.Status == PathStatus.Found)
        {
            bad = a.SaveState();
            bad[18] = (byte)SearchStatus.Running;
            Assert.Throws<GridStateException>(() => b.RestoreState(bad));
            bad = a.SaveState();
            bad[18 + 2 + 20] ^= 1; // the iteration counter no longer matches the closed cells
            Assert.Throws<GridStateException>(() => b.RestoreState(bad));
        }

        // A record naming a wall, a parent that is not visited, a changed cost.
        int first = 18 + 30;
        int records = BitConverter.ToInt32(running, 44);
        Assert.True(records >= 2);
        for (int rec = 0; rec < records; rec++)
        {
            int o = first + (rec * 17);
            foreach (int field in new[] { 4, 8, 12 })
            {
                bad = (byte[])running.Clone();
                bad[o + field] ^= 0x40;
                if (field == 4 || field == 8)
                {
                    // A changed cost is caught whether or not the record is the start (the start must cost 0).
                    Assert.Throws<GridStateException>(() => b.RestoreState(bad));
                }
                else
                {
                    try
                    {
                        b.RestoreState(bad);
                        Assert.Equal(-1, BitConverter.ToInt32(running, o + 12)); // only a no-op flip may pass
                    }
                    catch (GridStateException)
                    {
                    }
                }
            }
        }
    }

    [Fact]
    public void StartInsideAWallCellOfTheStateIsRejected()
    {
        Grid g = HostMaps.Make("rooms", 1, 30, 30);
        var rng = new SeededRandom(2);
        var (s, t) = Queries(g, ref rng, 1, 1)[0];
        GridSearch a = g.CreateSearch(Variant(0));
        a.TryBegin(s, t);
        a.Step(3);
        byte[] bytes = a.SaveState();
        // Put a wall where the search had visited, then restore the old state into the edited grid.
        int goalCell = (t.Y * g.Width) + t.X;
        g.SetArea(s.X, s.Y, AreaCosts.Wall);
        GridSearch b = g.CreateSearch(Variant(0));
        Assert.Throws<GridStateException>(() => b.RestoreState(bytes));
        Assert.True(goalCell >= 0);
    }
}
