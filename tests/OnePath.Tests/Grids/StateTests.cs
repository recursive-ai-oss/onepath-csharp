using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using RecursiveAI.OnePath.HashMatrix;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

public class StateTests
{
    public static TheoryData<string, ulong> Worlds()
    {
        var data = new TheoryData<string, ulong>();
        foreach (string kind in StateWorlds.Kinds)
        {
            for (ulong seed = 1; seed <= 4; seed++)
            {
                data.Add(kind, seed);
            }
        }
        return data;
    }

    // The core claim: snapshot, wander off, restore, carry on == never having left.
    [Theory]
    [MemberData(nameof(Worlds))]
    public void RolledBackRunEqualsNeverRolledBackRun(string kind, ulong seed)
    {
        byte[] straight = StateWorlds.Run(kind, seed, rollback: false);
        byte[] rolled = StateWorlds.Run(kind, seed, rollback: true);
        Assert.True(straight.Length > 64);
        Assert.Equal(straight, rolled);
    }

    [Theory]
    [InlineData("grid-astar")]
    [InlineData("dstar")]
    [InlineData("flow-repair")]
    [InlineData("graph")]
    [InlineData("sliced-jpsplus")]
    public void SnapshotBytesAreRepeatable(string kind)
    {
        Assert.Equal(StateWorlds.Snapshot(kind, 9), StateWorlds.Snapshot(kind, 9));
        Assert.NotEqual(StateWorlds.Snapshot(kind, 9), StateWorlds.Snapshot(kind, 10));
    }

    private static Grid Sample()
    {
        Grid g = Grid.FromText("""
            ..........
            .##..###..
            .#2..#.#..
            .#.33#.#..
            ...#...#..
            ##.#.###..
            """);
        g.Areas[2] = 3.5;
        g.Areas[3] = 0.25;
        g.Space = new GridSpace(-4.5, 8.25, 0.5);
        return g;
    }

    [Fact]
    public void GridRoundTripsByteForByteAndKeepsVersionAndSpace()
    {
        Grid g = Sample();
        g.SetArea(0, 0, AreaCosts.Wall);
        byte[] s = g.SaveState();
        Grid copy = Grid.FromState(s);
        Assert.Equal(s, copy.SaveState());
        Assert.Equal(g.Version, copy.Version);
        Assert.Equal(g.Space, copy.Space);
        Assert.Equal(g.Areas.Version, copy.Areas.Version);
        Assert.Equal(3.5, copy.Areas[2]);
        Assert.Equal(0.25, copy.Areas.MinCost);
        Assert.True(g.Cells.SequenceEqual(copy.Cells));

        var h = HexGrid.FromText("....\n.##.\n....", HexLayout.FlatEvenColumn);
        h.Areas[1] = 2.0;
        h.SetArea(0, 0, 1);
        Assert.Equal(h.SaveState(), HexGrid.FromState(h.SaveState()).SaveState());

        var v = new VoxelGrid(20, 5, 5, 0);
        v.SetArea(19, 4, 4, AreaCosts.Wall);
        v.SetArea(1, 1, 1, 3);
        Assert.Equal(v.SaveState(), VoxelGrid.FromState(v.SaveState()).SaveState());
        Assert.Equal(v.AllocatedChunks, VoxelGrid.FromState(v.SaveState()).AllocatedChunks);

        var gr = Graph.From([(0, 1, 2.0), (1, 2, 3.0), (2, 0, 1.0)], directed: true);
        gr.RemoveEdge(1);
        Graph gr2 = Graph.FromState(gr.SaveState());
        Assert.Equal(gr.SaveState(), gr2.SaveState());
        Assert.Equal(gr.Compile().Fingerprint(), gr2.Compile().Fingerprint());
    }

    [Fact]
    public void RestoringAGridTellsItsListenersAndDropsDerivedData()
    {
        Grid g = Sample();
        byte[] before = g.SaveState();
        GridPath hpaBefore = g.FindPath(new Cell(0, 0), new Cell(9, 5), new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 4 });
        var seen = new List<Cell>();
        g.AddListener(new Recorder(seen));
        g.SetArea(4, 4, AreaCosts.Wall);
        g.SetArea(5, 4, AreaCosts.Wall);
        int version = g.Version;
        seen.Clear();
        g.RestoreState(before);
        Assert.Equal([new Cell(4, 4), new Cell(5, 4)], seen);
        Assert.True(g.Version < version);
        Assert.False(g.GetArea(4, 4) == AreaCosts.Wall);
        GridPath hpaAfter = g.FindPath(new Cell(0, 0), new Cell(9, 5), new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 4 });
        Assert.Equal(hpaBefore.Cost, hpaAfter.Cost);
        Assert.True(hpaBefore.Cells.SequenceEqual(hpaAfter.Cells));
    }

    private sealed class Recorder(List<Cell> into) : IGridChangeListener
    {
        public void OnCellChanged(int x, int y) => into.Add(new Cell(x, y));
    }

    [Fact]
    public void VoxelListenersSeeTheVoxelsThatDiffer()
    {
        var v = new VoxelGrid(20, 18, 17);
        v.SetArea(3, 3, 3, AreaCosts.Wall);
        byte[] s = v.SaveState();
        var seen = new List<VoxelCell>();
        v.AddListener(new VoxelRecorder(seen));
        v.SetArea(3, 3, 3, 0);
        v.SetArea(19, 17, 16, AreaCosts.Wall);
        v.SetArea(0, 0, 0, 5);
        seen.Clear();
        v.RestoreState(s);
        Assert.Equal([new VoxelCell(0, 0, 0), new VoxelCell(3, 3, 3), new VoxelCell(19, 17, 16)], seen);
        Assert.Equal(AreaCosts.Wall, v.GetArea(3, 3, 3));
    }

    private sealed class VoxelRecorder(List<VoxelCell> into) : IVoxelChangeListener
    {
        public void OnVoxelChanged(int x, int y, int z) => into.Add(new VoxelCell(x, y, z));
    }

    // ---------- hostile input ----------

    private static IEnumerable<(string Name, byte[] State, Action<byte[]> Restore)> Blobs()
    {
        Grid g = Sample();
        yield return ("grid", g.SaveState(), b => Sample().RestoreState(b));
        var h = HexGrid.FromText("....\n.##.\n....", HexLayout.PointyOddRow);
        yield return ("hex", h.SaveState(), b => HexGrid.FromText("....\n.##.\n....", HexLayout.PointyOddRow).RestoreState(b));
        var v = new VoxelGrid(18, 3, 3);
        v.SetArea(17, 2, 2, 1);
        yield return ("voxel", v.SaveState(), b => new VoxelGrid(18, 3, 3).RestoreState(b));
        var gr = Graph.From([(0, 1, 2.0), (1, 2, 3.0)]);
        yield return ("graph", gr.SaveState(), b => new Graph().RestoreState(b));
        GridReplanner rp = Sample().CreateReplanner(new Cell(0, 0), new Cell(9, 5));
        rp.Replan(5);
        yield return ("replanner", rp.SaveState(), b => Sample().CreateReplanner(new Cell(0, 0), new Cell(9, 5)).RestoreState(b));
        Grid gf = Sample();
        GridFlowField ff = gf.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 6 });
        yield return ("flow", ff.SaveState(), b => Sample().CreateFlowField(new Cell(9, 5)).RestoreState(b));
        Grid ga = Sample();
        var a = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(ga.CreateTopology(), default, new OctileHeuristic(ga.Width));
        a.Begin(0, 59);
        a.Step(9);
        yield return ("astar", a.SaveState(), b =>
        {
            Grid gb = Sample();
            var s = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(gb.CreateTopology(), default, new OctileHeuristic(gb.Width));
            s.RestoreState(b);
            if (s.Step(2000) == SearchStatus.Found)
            {
                _ = s.PathLength(); // must terminate: restored parent links are checked for cycles
            }
        });
        var done = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(ga.CreateTopology(), default, new OctileHeuristic(ga.Width));
        done.Begin(0, 59);
        done.Run(TestContext.Current.CancellationToken);
        yield return ("astar-done", done.SaveState(), b =>
        {
            Grid gb = Sample();
            var s = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(gb.CreateTopology(), default, new OctileHeuristic(gb.Width));
            s.RestoreState(b);
            if (s.Status == SearchStatus.Found)
            {
                _ = s.PathLength(); // must terminate: a restored parent chain is checked
            }
        });
    }

    [Fact]
    public void EveryTruncationIsATypedError()
    {
        foreach ((string name, byte[] state, Action<byte[]> restore) in Blobs())
        {
            restore(state); // the good bytes restore
            for (int cut = 0; cut < state.Length; cut++)
            {
                byte[] part = state.AsSpan(0, cut).ToArray();
                Assert.Throws<GridStateException>(() => restore(part));
            }
            byte[] extra = [.. state, 0];
            Assert.Throws<GridStateException>(() => restore(extra));
            _ = name;
        }
    }

    [Fact]
    public void CorruptedBytesRestoreOrFailWithATypedError()
    {
        var rng = new SeededRandom(0xFEED);
        foreach ((string name, byte[] state, Action<byte[]> restore) in Blobs())
        {
            for (int round = 0; round < 1500; round++)
            {
                byte[] bad = (byte[])state.Clone();
                int edits = 1 + rng.NextInt(3);
                for (int e = 0; e < edits; e++)
                {
                    int at = rng.NextInt(bad.Length);
                    bad[at] = rng.NextInt(3) == 0 ? (byte)(rng.NextInt(2) == 0 ? 0xFF : 0) : (byte)rng.NextInt(256);
                }
                try
                {
                    restore(bad);
                }
                catch (GridStateException)
                {
                }
                catch (Exception ex)
                {
                    Assert.Fail($"{name}: restoring corrupted bytes threw {ex.GetType().Name}: {ex.Message}");
                }
            }
        }
    }

    [Fact]
    public void ACyclicParentLinkIsRejected()
    {
        Grid g = Sample();
        var a = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width));
        a.Begin(0, 59);
        a.Step(12);
        byte[] s = a.SaveState();
        // Visited entries start after the 8-byte header and 41 bytes of fields plus the visited count; each is node, closed, g, parent.
        int first = 8 + 4 + 1 + 8 + 4 + 4 + 4 + 4 + 8 + 4;
        int node0 = BitConverter.ToInt32(s, first);
        Assert.Equal(0, node0); // the start node is visited first
        int parentAt = first + 4 + 1 + 8;
        BitConverter.TryWriteBytes(s.AsSpan(parentAt, 4), 0); // the start becomes its own parent
        var b = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width));
        var ex = Assert.Throws<GridStateException>(() => b.RestoreState(s));
        Assert.Contains("cycle", ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void WrongKindAndWrongObjectAreRejectedWithTheirNames()
    {
        byte[] gridState = Sample().SaveState();
        var ex = Assert.Throws<GridStateException>(() => HexGrid.FromState(gridState));
        Assert.Contains("kind", ex.Message, StringComparison.Ordinal);
        var small = new Grid(4, 4);
        ex = Assert.Throws<GridStateException>(() => small.RestoreState(gridState));
        Assert.Contains("width", ex.Message, StringComparison.Ordinal);
        Assert.Equal(0, small.Version); // nothing was changed
        Assert.Throws<GridStateException>(() => Grid.FromState([1, 2, 3]));
        byte[] badMagic = (byte[])gridState.Clone();
        badMagic[0] ^= 1;
        Assert.Throws<GridStateException>(() => Grid.FromState(badMagic));
        byte[] badVersion = (byte[])gridState.Clone();
        badVersion[4] = 9;
        ex = Assert.Throws<GridStateException>(() => Grid.FromState(badVersion));
        Assert.Contains("version", ex.Message, StringComparison.Ordinal);
    }

    [Fact]
    public void ARestoreThatFailsLeavesTheGridAndReplannerAsTheyWere()
    {
        Grid g = Sample();
        byte[] s = g.SaveState();
        g.SetArea(0, 0, AreaCosts.Wall);
        byte[] changed = g.SaveState();
        byte[] broken = changed.AsSpan(0, changed.Length - 3).ToArray();
        Assert.Throws<GridStateException>(() => g.RestoreState(broken));
        Assert.Equal(changed, g.SaveState());
        GridReplanner r = g.CreateReplanner(new Cell(2, 0), new Cell(9, 5));
        GridPath before = r.Replan();
        byte[] rs = r.SaveState();
        Assert.Throws<GridStateException>(() => r.RestoreState(rs.AsSpan(0, rs.Length - 5).ToArray()));
        Assert.Equal(rs, r.SaveState());
        Assert.Equal(before.Cost, r.Replan().Cost);
        Assert.NotEqual(s, changed);
    }

    // A searched-with-hooks object keeps working when the options match.
    [Fact]
    public void TheHeuristicWeightMustMatch()
    {
        Grid g = Sample();
        var a = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width)) { HeuristicWeight = 2.0 };
        a.Begin(0, 59);
        a.Step(4);
        var b = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width));
        var ex = Assert.Throws<GridStateException>(() => b.RestoreState(a.SaveState()));
        Assert.Contains("weight", ex.Message, StringComparison.Ordinal);
        b.HeuristicWeight = 2.0;
        b.RestoreState(a.SaveState());
        Assert.Equal(a.Run(TestContext.Current.CancellationToken), b.Run(TestContext.Current.CancellationToken));
        Assert.Equal(a.PathCost, b.PathCost);
    }

    // ---------- debug geometry ----------

    [Theory]
    [InlineData(GridAlgorithm.AStar)]
    [InlineData(GridAlgorithm.Jps)]
    [InlineData(GridAlgorithm.JpsPlus)]
    [InlineData(GridAlgorithm.ThetaStar)]
    [InlineData(GridAlgorithm.LazyThetaStar)]
    [InlineData(GridAlgorithm.Hpa)]
    public void DebugExpansionsMatchTheSearch(GridAlgorithm algorithm)
    {
        var grid = new Grid(40, 30);
        var rng = new SeededRandom(77);
        for (int i = 0; i < 260; i++)
        {
            grid.SetArea(rng.NextInt(40), rng.NextInt(30), AreaCosts.Wall);
        }
        grid.SetArea(0, 0, 0);
        grid.SetArea(39, 29, 0);
        GridPath quiet = grid.FindPath(new Cell(0, 0), new Cell(39, 29), new GridOptions { Algorithm = algorithm, HpaChunkSize = 8 });
        Assert.Empty(quiet.ExpandedCells.ToArray()); // opt-in: the happy path records nothing
        GridPath p = grid.FindPath(grid.CellCenter(new Cell(0, 0)), grid.CellCenter(new Cell(39, 29)), new GridOptions { Algorithm = algorithm, HpaChunkSize = 8, RecordExpansions = true });
        GridPathDebug d = GridDebug.Describe(p);
        Assert.Equal(p.ExpandedCount, d.ExpandedCount);
        Assert.Equal(p.ExpandedCount, d.ExpandedCells.Count);
        Assert.Equal(p.Cells.Length, d.Cells.Count);
        Assert.Equal(p.Cells.Length, d.WorldPoints.Count);
        Assert.Equal(quiet.Cost, d.Cost);
        Assert.Equal(algorithm, d.Algorithm);
    }

    [Fact]
    public void DebugExpansionsOfReplannersVoxelsHexAndGraphs()
    {
        Grid grid = Sample();
        foreach (GridAlgorithm alg in new[] { GridAlgorithm.DStarLite, GridAlgorithm.LpaStar })
        {
            GridReplanner r = grid.CreateReplanner(new Cell(0, 0), new Cell(9, 5), new GridOptions { Algorithm = alg, RecordExpansions = true });
            GridPath first = r.Replan();
            Assert.Equal(first.ExpandedCount, first.ExpandedCells.Length);
            Assert.True(first.ExpandedCount > 0);
            grid.SetArea(8, 4, AreaCosts.Wall);
            GridPath again = r.Replan();
            Assert.Equal(again.ExpandedCount, again.ExpandedCells.Length);
            grid.SetArea(8, 4, 0);
            GridPath quiet = grid.CreateReplanner(new Cell(0, 0), new Cell(9, 5), new GridOptions { Algorithm = alg }).Replan();
            Assert.Empty(quiet.ExpandedCells.ToArray());
        }

        var hex = HexGrid.FromText("......\n.####.\n......");
        HexReplanner hr = hex.CreateReplanner(new Cell(0, 0), new Cell(5, 2), new GridOptions { RecordExpansions = true });
        GridPath hp = hr.Replan();
        Assert.Equal(hp.ExpandedCount, GridDebug.Describe(hp).ExpandedCells.Count);

        var vox = new VoxelGrid(10, 10, 10);
        VoxelPath vp = vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(9, 9, 9), new VoxelOptions { RecordExpansions = true });
        VoxelPathDebug vd = GridDebug.Describe(vp);
        Assert.Equal(vp.ExpandedCount, vd.ExpandedCells.Count);
        Assert.Equal(vp.Cells.Length, vd.Cells.Count);

        Graph graph = Graph.From([(0, 1, 1.0), (1, 2, 1.0), (0, 2, 5.0), (2, 3, 1.0)]);
        GraphPath gp = graph.FindPath(0, 3, new GraphOptions { RecordExpansions = true }, TestContext.Current.CancellationToken);
        GraphPathDebug gd = GridDebug.Describe(gp);
        Assert.Equal(gp.ExpandedCount, gd.ExpandedNodes.Count);
        Assert.Equal([0, 1, 2, 3], gd.Nodes);
    }

    [Fact]
    public void FlowFieldAndHpaDebugData()
    {
        Grid g = Sample();
        GridFlowField f = g.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 5 });
        FlowFieldDebug partial = GridDebug.Describe(f);
        Assert.Equal(FlowFieldStatus.Running, partial.Status);
        Assert.Equal(60, partial.Costs.Count);
        f.Run(TestContext.Current.CancellationToken);
        FlowFieldDebug full = GridDebug.Describe(f);
        Assert.Equal(FlowFieldStatus.Complete, full.Status);
        Assert.Equal(0.0, full.Costs[(5 * 10) + 9]);
        Assert.Equal([new Cell(9, 5)], full.Goals);
        Assert.True(full.TryGetStep(0, 0, out int dx, out int dy));
        Assert.True(dx is >= -1 and <= 1 && dy is >= -1 and <= 1 && (dx != 0 || dy != 0));
        Assert.False(full.TryGetStep(9, 5, out _, out _));
        for (int i = 0; i < 60; i++)
        {
            Assert.Equal(f.Costs[i], full.Costs[i]);
            Assert.Equal(f.Next[i], full.NextCell[i]);
        }

        var big = new Grid(32, 32);
        for (int y = 0; y < 32; y++)
        {
            big.SetArea(15, y, AreaCosts.Wall);
        }
        big.SetArea(15, 20, 0);
        using var hpa = new HpaHierarchy(big, new HpaOptions { ChunkSize = 8 });
        HpaGraphDebug hd = GridDebug.Describe(hpa);
        Assert.Equal(hpa.NodeCount, hd.Nodes.Count);
        Assert.Equal(hd.Nodes.Count, hd.NodeCluster.Count);
        Assert.Contains(hd.Edges, e => e.IsInter);
        Assert.Contains(hd.Edges, e => !e.IsInter);
        Assert.All(hd.Edges, e => Assert.True(e.Cost > 0.0 && !double.IsInfinity(e.Cost)));
    }
}
