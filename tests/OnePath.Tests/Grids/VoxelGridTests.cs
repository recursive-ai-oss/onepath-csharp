using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class VoxelGridTests
{
    private sealed class Recorder : IVoxelChangeListener
    {
        public readonly List<VoxelCell> Changes = [];

        public void OnVoxelChanged(int x, int y, int z) => Changes.Add(new VoxelCell(x, y, z));
    }

    [Fact]
    public void ChunksAreAllocatedOnlyWhereVoxelsDiffer()
    {
        var g = new VoxelGrid(100, 40, 33);
        Assert.Equal(7 * 3 * 3, g.ChunksX * g.ChunksY * g.ChunksZ);
        Assert.Equal(0, g.AllocatedChunks);
        g.SetArea(50, 20, 32, AreaCosts.Wall);
        g.SetArea(51, 20, 32, AreaCosts.Wall);
        Assert.Equal(1, g.AllocatedChunks);
        Assert.Equal(AreaCosts.Wall, g.GetArea(51, 20, 32));
        Assert.Equal(0, g.GetArea(52, 20, 32));
        Assert.True(g.GetChunk(0, 0, 0).IsEmpty);
        ReadOnlySpan<byte> c = g.GetChunk(3, 1, 2);
        Assert.Equal(4096, c.Length);
        Assert.Equal(AreaCosts.Wall, c[(50 & 15) | ((20 & 15) << 4) | ((32 & 15) << 8)]);
        g.SetArea(0, 0, 0, 0); // no change: nothing allocated
        Assert.Equal(1, g.AllocatedChunks);
    }

    [Fact]
    public void SolidBlockCanBeCarved()
    {
        var g = new VoxelGrid(20, 20, 20, AreaCosts.Wall);
        Assert.Equal(AreaCosts.Wall, g.GetArea(3, 3, 3));
        Assert.Equal(PathOutcome.StartInvalid, g.FindPath(new VoxelCell(1, 1, 1), new VoxelCell(2, 2, 2)).Status);
        for (int x = 1; x <= 18; x++)
        {
            g.SetWalkable(x, 5, 5, true);
        }
        VoxelPath p = g.FindPath(new VoxelCell(1, 5, 5), new VoxelCell(18, 5, 5));
        Assert.True(p.Found);
        Assert.Equal(17.0, p.Cost);
    }

    [Fact]
    public void ListenersHearEveryRealChange()
    {
        var g = new VoxelGrid(4, 4, 4);
        var r = new Recorder();
        g.AddListener(r);
        g.SetWalkable(1, 2, 3, false);
        g.SetWalkable(1, 2, 3, false);
        g.SetArea(0, 0, 0, 7);
        Assert.Equal([new VoxelCell(1, 2, 3), new VoxelCell(0, 0, 0)], r.Changes);
        Assert.Equal(2, g.Version);
        Assert.True(g.RemoveListener(r));
        g.SetWalkable(2, 2, 2, false);
        Assert.Equal(2, r.Changes.Count);
    }

    [Fact]
    public void EditedGridEqualsGridBuiltFromScratch()
    {
        var rng = new SeededRandom(77);
        for (int t = 0; t < 40; t++)
        {
            VoxelGrid edited = VoxelBruteForce.Random(ref rng, 12, 11, 10, 0.25, true);
            VoxelOptions options = t % 2 == 0 ? VoxelOptions.Default : VoxelOptions.EighteenConnected;
            // Several rounds: edit some voxels, query, then compare with a grid constructed directly with the final contents.
            for (int round = 0; round < 5; round++)
            {
                for (int e = 0; e < 20; e++)
                {
                    int x = rng.NextInt(12);
                    int y = rng.NextInt(11);
                    int z = rng.NextInt(10);
                    edited.SetArea(x, y, z, rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(4));
                }
                VoxelGrid fresh = new(12, 11, 10);
                fresh.Areas.CopyFrom(edited.Areas);
                for (int z = 0; z < 10; z++)
                {
                    for (int y = 0; y < 11; y++)
                    {
                        for (int x = 0; x < 12; x++)
                        {
                            fresh.SetArea(x, y, z, edited.GetArea(x, y, z));
                        }
                    }
                }
                var cells = VoxelBruteForce.OpenCells(edited);
                if (cells.Count < 2)
                {
                    continue;
                }
                for (int q = 0; q < 3; q++)
                {
                    VoxelCell a = cells[rng.NextInt(cells.Count)];
                    VoxelCell b = cells[rng.NextInt(cells.Count)];
                    VoxelPath p1 = edited.FindPath(a, b, options);
                    VoxelPath p2 = fresh.FindPath(a, b, options);
                    Assert.Equal(p2.Status, p1.Status);
                    Assert.Equal(p2.Cost, p1.Cost);
                    Assert.True(p1.Cells.SequenceEqual(p2.Cells));
                }
            }
        }
    }

    [Fact]
    public void SlicedSearchEqualsUnsliced()
    {
        var rng = new SeededRandom(99);
        for (int t = 0; t < 30; t++)
        {
            VoxelGrid g = VoxelBruteForce.Random(ref rng, 14, 12, 13, 0.25, t % 2 == 0);
            var cells = VoxelBruteForce.OpenCells(g);
            VoxelCell a = cells[rng.NextInt(cells.Count)];
            VoxelCell b = cells[rng.NextInt(cells.Count)];
            VoxelTopology topo = g.CreateTopology();
            int n0 = g.NodeIndex(a.X, a.Y, a.Z);
            int n1 = g.NodeIndex(b.X, b.Y, b.Z);
            var h = new VoxelHeuristic(g.SizeX, g.SizeY, VoxelHeuristicKind.Octile26, g.Areas.MinCost);
            var whole = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, h);
            whole.Begin(n0, n1);
            SearchStatus expect = whole.Run(TestContext.Current.CancellationToken);
            foreach (int budget in new[] { 1, 2, 7, 50 })
            {
                var sliced = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, h);
                sliced.Begin(n0, n1);
                int calls = 0;
                while (sliced.Step(budget) == SearchStatus.Running)
                {
                    calls++;
                    Assert.True(calls < 100_000);
                }
                Assert.Equal(expect, sliced.Status);
                Assert.Equal(whole.Iterations, sliced.Iterations);
                if (expect == SearchStatus.Found)
                {
                    Assert.Equal(whole.PathCost, sliced.PathCost);
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
        var g = new VoxelGrid(30, 30, 30);
        VoxelPath p = g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(29, 29, 0), new VoxelOptions { IterationBudget = 3, Heuristic = GridHeuristic.Zero });
        Assert.Equal(PathOutcome.BudgetExhausted, p.Status);
    }

    [Fact]
    public void CornerRulesBehaveOnATwoByTwoByTwoBlock()
    {
        // Open space except the two face cells beside an edge move from (0,0,0) to (1,1,0).
        var g = new VoxelGrid(2, 2, 1);
        g.SetWalkable(1, 0, 0, false);
        g.SetWalkable(0, 1, 0, false);
        var a = new VoxelCell(0, 0, 0);
        var b = new VoxelCell(1, 1, 0);
        Assert.Equal(PathOutcome.NoPath, g.FindPath(a, b, new VoxelOptions { Corners = VoxelCornerRule.NoCornerCutting }).Status);
        Assert.Equal(PathOutcome.NoPath, g.FindPath(a, b, new VoxelOptions { Corners = VoxelCornerRule.NoSqueezing }).Status);
        Assert.Equal(GridMath.Sqrt2, g.FindPath(a, b, new VoxelOptions { Corners = VoxelCornerRule.AllowAll }).Cost);
        g.SetWalkable(1, 0, 0, true);
        Assert.Equal(2.0, g.FindPath(a, b, new VoxelOptions { Corners = VoxelCornerRule.NoCornerCutting }).Cost);
        Assert.Equal(GridMath.Sqrt2, g.FindPath(a, b, new VoxelOptions { Corners = VoxelCornerRule.NoSqueezing }).Cost);
    }

    [Fact]
    public void CornerMoveCostsRootThree()
    {
        var g = new VoxelGrid(2, 2, 2);
        VoxelPath p = g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(1, 1, 1));
        Assert.Equal(VoxelMath.Sqrt3, p.Cost);
        Assert.Equal(GridMath.Sqrt2 + 1.0, g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(1, 1, 1), VoxelOptions.EighteenConnected).Cost);
        Assert.Equal(3.0, g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(1, 1, 1), VoxelOptions.SixConnected).Cost);
    }

    [Fact]
    public void WorldPositionsAreCheckedAndMapped()
    {
        var g = new VoxelGrid(10, 10, 10) { Space = new VoxelSpace(-5.0, 100.0, 2.0, 0.5) };
        VoxelPath p = g.FindPath(new VoxelPoint(-4.9, 100.1, 2.1), new VoxelPoint(-3.4, 100.1, 2.1));
        Assert.True(p.Found);
        Assert.Equal(4, p.Cells.Length);
        Assert.Equal(new VoxelPoint(-4.75, 100.25, 2.25), p.WorldPoints[0]);
        Assert.Equal(PathOutcome.GoalInvalid, g.FindPath(new VoxelPoint(-4.9, 100.1, 2.1), new VoxelPoint(50, 100.1, 2.1)).Status);
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => g.FindPath(new VoxelPoint(3e6, 0, 0), new VoxelPoint(0, 0, 0)));
        Assert.Contains("2^21", ex.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.FindPath(new VoxelPoint(0, double.NaN, 0), new VoxelPoint(0, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => new VoxelSpace(0, 0, 2097153.0, 1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new VoxelSpace(0, 0, 0, 0.0));
    }

    [Fact]
    public void DelegatesAdjustCostsAndBlocking()
    {
        var g = new VoxelGrid(5, 1, 1);
        var o = new VoxelOptions { StepCost = (_, to, c) => to.X == 2 ? c * 10 : c };
        Assert.Equal(13.0, g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(4, 0, 0), o).Cost);
        var blocked = new VoxelOptions { Blocked = c => c.X == 2 };
        Assert.Equal(PathOutcome.NoPath, g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(4, 0, 0), blocked).Status);
        Assert.Equal(PathOutcome.GoalInvalid, g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(2, 0, 0), blocked).Status);
    }

    [Fact]
    public void InvalidInputIsRejectedWithTypedErrors()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new VoxelGrid(0, 1, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new VoxelGrid(1 << 11, 1 << 10, 1 << 10));
        var g = new VoxelGrid(3, 3, 3);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.GetArea(3, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.SetArea(0, -1, 0, 1));
        Assert.Equal(PathOutcome.StartInvalid, g.FindPath(new VoxelCell(9, 0, 0), new VoxelCell(0, 0, 0)).Status);
        Assert.Throws<ArgumentOutOfRangeException>(() => g.CreateTopology((VoxelConnectivity)7));
        Assert.Throws<ArgumentOutOfRangeException>(() => new VoxelOptions { HeuristicWeight = double.NaN });
    }

    [Fact]
    public void WeightedSearchStaysWithinTheWeight()
    {
        var rng = new SeededRandom(5);
        VoxelGrid g = VoxelBruteForce.Random(ref rng, 12, 12, 12, 0.3, false);
        var cells = VoxelBruteForce.OpenCells(g);
        for (int q = 0; q < 30; q++)
        {
            VoxelCell a = cells[rng.NextInt(cells.Count)];
            VoxelCell b = cells[rng.NextInt(cells.Count)];
            VoxelPath opt = g.FindPath(a, b);
            VoxelPath w = g.FindPath(a, b, new VoxelOptions { HeuristicWeight = 2.0 });
            Assert.Equal(opt.Found, w.Found);
            if (opt.Found)
            {
                Assert.InRange(w.Cost, opt.Cost - 1e-9, (2.0 * opt.Cost) + 1e-9);
            }
        }
    }
}

public class VoxelMovingAiFormatTests
{
    [Fact]
    public void ParsesAMapAndBuildsTheGrid()
    {
        VoxelMap m = VoxelMap.Parse("voxel 4 3 2\n0 0 0\n1 2 1\n3 1 1\n");
        Assert.Equal((4, 3, 2), (m.SizeX, m.SizeY, m.SizeZ));
        Assert.Equal(3, m.FilledCount);
        VoxelGrid g = m.ToGrid();
        Assert.False(g.IsWalkable(1, 2, 1));
        Assert.False(g.IsWalkable(0, 0, 0));
        Assert.True(g.IsWalkable(2, 2, 1));
    }

    [Fact]
    public void ParsesScenarios()
    {
        VoxelScenarioEntry[] e = VoxelScenarios.Parse("version 1\nA1.3dmap\n101 109 191 577 273 142 562.04094761 1.005\r\n732 289 150 575 277 143 165.89876598 1.010\n");
        Assert.Equal(2, e.Length);
        Assert.Equal(new VoxelScenarioEntry(101, 109, 191, 577, 273, 142, 562.04094761, 1.005), e[0]);
    }

    [Theory]
    [InlineData("")]
    [InlineData("voxel 3 3\n")]
    [InlineData("vox 3 3 3\n")]
    [InlineData("voxel 0 3 3\n")]
    [InlineData("voxel 3000000 3000000 3000000\n")]
    [InlineData("voxel 3 3 3\n3 0 0\n")]
    [InlineData("voxel 3 3 3\n1 2\n")]
    [InlineData("voxel 3 3 3\n-1 2 0\n")]
    [InlineData("voxel 3 3 3\n99999999999 2 0\n")]
    public void BadMapsGiveFormatExceptionsNamingTheLine(string text)
    {
        var ex = Assert.Throws<GridFormatException>(() => VoxelMap.Parse(text));
        Assert.Contains("line", ex.Message, StringComparison.Ordinal);
    }

    [Theory]
    [InlineData("")]
    [InlineData("version 2\nA\n")]
    [InlineData("version 1\n")]
    [InlineData("version 1\nA\n1 2 3 4 5 6 7\n")]
    [InlineData("version 1\nA\n1 2 3 4 5 6 x 1\n")]
    public void BadScenariosGiveFormatExceptions(string text) => Assert.Throws<GridFormatException>(() => VoxelScenarios.Parse(text));

    [Fact]
    public void HugeNumberFieldIsRejectedNotOverflowed()
    {
        string huge = new('9', 200_000);
        Assert.Throws<GridFormatException>(() => VoxelScenarios.Parse("version 1\nA\n1 2 3 4 5 6 " + huge + " 1\n"));
    }
}
