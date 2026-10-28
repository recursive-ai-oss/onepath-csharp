using System;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>The calls a game tick makes on a jump point GridSearch, and the generic engine's save, allocate nothing in steady state.</summary>
public class HostJumpAllocationTests
{
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(3)]
    [InlineData(4)]
    public void JumpSearchTickCallsAllocateNothing(int variant)
    {
        Grid g = HostMaps.Make("cave", 3, 64, 64);
        System.Collections.Generic.List<Cell> ok = BruteForce.WalkableStarts(g, 1);
        Cell s = ok[0];
        Cell t = ok[^1];
        GridSearch search = g.CreateSearch(HostJumpSearchTests.Variant(variant));
        var cells = new Cell[g.CellCount];
        var idx = new int[g.CellCount];
        var bytes = new byte[1 << 22];
        Assert.Equal(0, AllocationProbe.SteadyState(() =>
        {
            if (search.TryBegin(s, t) != PathStatus.InProgress)
            {
                return false;
            }
            PathStatus st;
            int slices = 0;
            while ((st = search.Step(5)) == PathStatus.InProgress)
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
        }));
        Assert.Equal(0, AllocationProbe.SteadyState(() =>
        {
            search.TryBegin(s, t);
            search.Step(10);
            int n = search.SaveState(bytes);
            search.Step(10);
            search.RestoreState(bytes.AsSpan(0, n));
            _ = search.GetStateSize();
            return n > 0;
        }));
    }

    [Fact]
    public void GenericAStarSaveAllocatesNothing()
    {
        Grid g = HostMaps.Make("cave", 3, 64, 64);
        System.Collections.Generic.List<Cell> ok = BruteForce.WalkableStarts(g, 1);
        GridSearch search = g.CreateSearch(new GridOptions { SpecializedEngines = false });
        var bytes = new byte[1 << 22];
        Assert.Equal(0, AllocationProbe.SteadyState(() =>
        {
            search.TryBegin(ok[0], ok[^1]);
            search.Step(40);
            int n = search.SaveState(bytes);
            _ = search.GetStateSize();
            search.Step(40);
            search.RestoreState(bytes.AsSpan(0, n));
            return n > 0;
        }));
    }
}
