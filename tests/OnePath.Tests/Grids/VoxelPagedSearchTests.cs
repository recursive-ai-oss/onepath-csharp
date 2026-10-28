using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

// The one-line voxel query keeps its state in pages; the dense tier-2 search is the reference it must equal exactly.
public class VoxelPagedSearchTests
{
    [Fact]
    public void PagedQueryEqualsDenseSearchInCostPathAndExpansionOrder()
    {
        var rng = new SeededRandom(0x9A6ED);
        VoxelConnectivity[] conns = [VoxelConnectivity.Six, VoxelConnectivity.Eighteen, VoxelConnectivity.TwentySix];
        for (int t = 0; t < 60; t++)
        {
            VoxelGrid g = VoxelBruteForce.Random(ref rng, 18 + rng.NextInt(20), 17 + rng.NextInt(20), 16 + rng.NextInt(20), 0.1 + (rng.NextDouble() * 0.3), t % 2 == 0);
            List<VoxelCell> open = VoxelBruteForce.OpenCells(g);
            var options = new VoxelOptions
            {
                Connectivity = conns[t % 3],
                Corners = (VoxelCornerRule)(t % 3),
                HeuristicWeight = t % 5 == 4 ? 1.7 : 1.0,
                RecordExpansions = true,
            };
            VoxelTopology topo = g.CreateTopology(options.Connectivity, options.Corners);
            var buffers = new GridSearchBuffers(g.VoxelCount) { RecordExpansions = true };
            for (int q = 0; q < 6; q++)
            {
                VoxelCell a = open[rng.NextInt(open.Count)];
                VoxelCell b = open[rng.NextInt(open.Count)];
                var h = new VoxelHeuristic(g.SizeX, g.SizeY, options.Connectivity == VoxelConnectivity.Six ? VoxelHeuristicKind.Manhattan : options.Connectivity == VoxelConnectivity.Eighteen ? VoxelHeuristicKind.Octile18 : VoxelHeuristicKind.Octile26, g.Areas.MinCost);
                var dense = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, h, buffers) { HeuristicWeight = options.HeuristicWeight };
                dense.Begin(g.NodeIndex(a.X, a.Y, a.Z), g.NodeIndex(b.X, b.Y, b.Z));
                SearchStatus status = dense.Run(TestContext.Current.CancellationToken);

                VoxelPath p = g.FindPath(a, b, options);
                Assert.Equal(status == SearchStatus.Found, p.Found);
                Assert.Equal(dense.ExpandedCount, p.ExpandedCount);
                ReadOnlySpan<int> expect = buffers.ExpandedNodes;
                Assert.Equal(expect.Length, p.ExpandedCells.Length);
                for (int i = 0; i < expect.Length; i++)
                {
                    Assert.Equal(g.CellOf(expect[i]), p.ExpandedCells[i]);
                }
                if (p.Found)
                {
                    Assert.Equal(dense.PathCost, p.Cost);
                    var nodes = new int[dense.PathLength()];
                    dense.CopyPath(nodes);
                    Assert.Equal(nodes.Length, p.Cells.Length);
                    for (int i = 0; i < nodes.Length; i++)
                    {
                        Assert.Equal(g.CellOf(nodes[i]), p.Cells[i]);
                    }
                }
            }
        }
    }

    [Fact]
    public void BudgetedQueryStopsAtTheSameIterationAsTheDenseSearch()
    {
        var rng = new SeededRandom(0xB0D6E7);
        VoxelGrid g = VoxelBruteForce.Random(ref rng, 30, 28, 26, 0.2, false);
        List<VoxelCell> open = VoxelBruteForce.OpenCells(g);
        VoxelCell a = open[3];
        VoxelCell b = open[open.Count - 5];
        VoxelPath full = g.FindPath(a, b);
        foreach (int budget in new[] { 1, 2, 7, 50, 300 })
        {
            VoxelPath p = g.FindPath(a, b, new VoxelOptions { IterationBudget = budget });
            if (budget < full.ExpandedCount + 1)
            {
                Assert.Equal(PathStatus.BudgetExhausted, p.Status);
                Assert.Equal(budget, p.ExpandedCount);
            }
        }
    }

    [Fact]
    public void SearchMemoryFollowsTheSearchNotTheGrid()
    {
        // 1300 x 1300 x 300 = 507 million voxels: a dense search state would need about 10 GB.
        var g = new VoxelGrid(1300, 1300, 300);
        long before = GC.GetTotalMemory(true);
        VoxelPath p = g.FindPath(new VoxelCell(10, 10, 10), new VoxelCell(60, 40, 30));
        long after = GC.GetTotalMemory(false);
        Assert.True(p.Found);
        Assert.True(after - before < 64L * 1024 * 1024, $"the query grew the heap by {after - before} bytes");
    }

    [Fact]
    public void ReusedStateGivesTheSameAnswersAfterOtherQueries()
    {
        var rng = new SeededRandom(0x5EED);
        VoxelGrid g = VoxelBruteForce.Random(ref rng, 40, 35, 30, 0.25, true);
        List<VoxelCell> open = VoxelBruteForce.OpenCells(g);
        var pairs = new List<(VoxelCell, VoxelCell)>();
        var first = new List<VoxelPath>();
        for (int i = 0; i < 20; i++)
        {
            pairs.Add((open[rng.NextInt(open.Count)], open[rng.NextInt(open.Count)]));
            first.Add(g.FindPath(pairs[i].Item1, pairs[i].Item2));
        }
        for (int i = 19; i >= 0; i--)
        {
            VoxelPath again = g.FindPath(pairs[i].Item1, pairs[i].Item2);
            Assert.Equal(first[i].Status, again.Status);
            Assert.Equal(first[i].Cost, again.Cost);
            Assert.Equal(first[i].ExpandedCount, again.ExpandedCount);
            Assert.True(first[i].Cells.SequenceEqual(again.Cells));
        }
    }
}
