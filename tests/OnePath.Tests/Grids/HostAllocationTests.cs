using System;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>The calls a game tick makes allocate nothing in steady state.</summary>
public class HostAllocationTests
{
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void GridSearchTickCallsAllocateNothing(bool genericEngine)
    {
        Grid g = HostMaps.Make("cave", 3, 64, 64);
        var o = new GridOptions { SpecializedEngines = !genericEngine };
        var rng = new SeededRandom(5);
        System.Collections.Generic.List<Cell> ok = BruteForce.WalkableStarts(g, 1);
        Cell s = ok[0];
        Cell t = ok[^1];
        GridSearch search = g.CreateSearch(o);
        var cells = new Cell[g.CellCount];
        var idx = new int[g.CellCount];
        var bytes = new byte[1 << 20];
        long a = AllocationProbe.SteadyState(() =>
        {
            if (search.TryBegin(s, t) != PathStatus.InProgress)
            {
                return false;
            }
            PathStatus st;
            int slices = 0;
            while ((st = search.Step(25)) == PathStatus.InProgress)
            {
                slices++;
            }
            if (st != PathStatus.Found)
            {
                return false;
            }
            search.CopyPath(cells);
            search.CopyPath(idx);
            return slices > 0;
        });
        Assert.Equal(0, a);

        if (!genericEngine)
        {
            long b = AllocationProbe.SteadyState(() =>
            {
                search.TryBegin(s, t);
                search.Step(40);
                int n = search.SaveState(bytes);
                search.Step(40);
                search.RestoreState(bytes.AsSpan(0, n));
                _ = search.GetStateSize();
                return n > 0;
            });
            Assert.Equal(0, b);
        }
        _ = rng;
    }

    [Fact]
    public void TryReplanAndCopyPathAllocateNothing()
    {
        Grid g = HostMaps.Make("random", 2, 64, 64);
        System.Collections.Generic.List<Cell> ok = BruteForce.WalkableStarts(g, 1);
        Cell s = ok[0];
        Cell t = ok[^1];
        Assert.True(g.FindPath(s, t).Found);
        using GridReplanner r = g.CreateReplanner(s, t);
        var path = new int[g.CellCount];
        var cells = new Cell[g.CellCount];
        Cell flip = ok[ok.Count / 2];
        bool wall = false;
        Assert.Equal(PathStatus.Found, r.TryReplan());
        long a = AllocationProbe.SteadyState(
            () =>
            {
                PathStatus st = r.TryReplan(100);
                while (st == PathStatus.BudgetExhausted)
                {
                    st = r.TryReplan(100);
                }
                if (st != PathStatus.Found)
                {
                    return false;
                }
                r.CopyPath(path);
                r.CopyPath(cells);
                return r.PathLength > 1;
            },
            attempts: 30,
            prepare: () =>
            {
                wall = !wall;
                if (flip != s && flip != t)
                {
                    g.SetArea(flip.X, flip.Y, wall ? AreaCosts.Wall : (byte)0);
                }
            });
        Assert.Equal(0, a);
    }

    [Fact]
    public void HashNearestAndFixedCostAllocateNothing()
    {
        Grid g = HostMaps.Make("areas", 2, 64, 64);
        System.Collections.Generic.List<Cell> ok = BruteForce.WalkableStarts(g, 1);
        GridPath p = g.FindPath(ok[0], ok[^1]);
        Cell[] cells = p.Cells.ToArray();
        var o = new GridOptions { AgentSize = 2 };
        Assert.Equal(0, AllocationProbe.SteadyState(() =>
        {
            ulong h = g.ContentHash();
            bool n = g.TryFindNearestWalkable(new Cell(-3, 70), out Cell c, o);
            bool f = g.TryGetFixedCost(cells, out long cost);
            bool w = g.TryFindNearestWalkable(new WorldPoint(3.5, 4.5), out _);
            return h != 0 && n && f && w && c.X >= 0 && cost > 0;
        }));
    }
}
