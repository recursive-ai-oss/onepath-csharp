using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.Grids;

/// <summary>Snapshot, restore and debug data of grids, voxels, graphs, replanners, flow fields and sliced searches, for the NativeAOT and WebAssembly smoke apps.</summary>
internal static class StateSmoke
{
    public static bool Run()
    {
        const string Map = "..........\n.##..###..\n.#2..#.#..\n.#.33#.#..\n...#...#..\n##.#.###..";
        Grid g = Grid.FromText(Map);
        g.Areas[2] = 3.5;
        byte[] saved = g.SaveState();
        GridPath before = g.FindPath(new Cell(0, 0), new Cell(9, 5));
        g.SetArea(4, 4, AreaCosts.Wall);
        g.SetArea(5, 0, AreaCosts.Wall);
        g.RestoreState(saved);
        bool ok = saved.AsSpan().SequenceEqual(g.SaveState()) && g.FindPath(new Cell(0, 0), new Cell(9, 5)).Cost == before.Cost;

        // A replanner and a flow field are saved mid-run and carried on in fresh objects over a fresh grid.
        GridReplanner rp = g.CreateReplanner(new Cell(0, 0), new Cell(9, 5), new GridOptions { RecordExpansions = true });
        rp.Replan(7);
        byte[] rpState = rp.SaveState();
        GridFlowField flow = g.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 8 });
        byte[] flowState = flow.SaveState();
        Grid g2 = Grid.FromState(saved);
        GridReplanner rp2 = g2.CreateReplanner(new Cell(0, 0), new Cell(9, 5), new GridOptions { RecordExpansions = true });
        rp2.RestoreState(rpState);
        GridPath a = rp.Replan();
        GridPath b = rp2.Replan();
        ok &= a.Found && a.Cost == b.Cost && a.ExpandedCount == b.ExpandedCount && a.ExpandedCells.Length == a.ExpandedCount;
        GridFlowField flow2 = g2.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 8 });
        flow2.RestoreState(flowState);
        flow.Run();
        flow2.Run();
        FlowFieldDebug fd = GridDebug.Describe(flow2);
        ok &= flow.GetCost(new Cell(0, 0)) == flow2.GetCost(new Cell(0, 0)) && fd.Status == FlowFieldStatus.Complete && fd.Costs[(5 * 10) + 9] == 0.0;

        // A sliced A* resumes in a new object.
        var s1 = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width));
        s1.Begin(0, 59);
        s1.Step(9);
        var s2 = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(g.Width));
        s2.RestoreState(s1.SaveState());
        ok &= s1.Run() == SearchStatus.Found && s2.Run() == SearchStatus.Found && s1.PathCost == s2.PathCost;

        var hex = HexGrid.FromText("....\n.##.\n....");
        byte[] hs = hex.SaveState();
        hex.SetArea(0, 0, AreaCosts.Wall);
        hex.RestoreState(hs);
        ok &= hs.AsSpan().SequenceEqual(hex.SaveState());

        var vox = new VoxelGrid(18, 3, 3);
        vox.SetArea(17, 2, 2, 1);
        byte[] vs = vox.SaveState();
        vox.SetArea(2, 2, 2, AreaCosts.Wall);
        vox.RestoreState(vs);
        ok &= vs.AsSpan().SequenceEqual(vox.SaveState()) && GridDebug.Describe(vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(17, 2, 2), new VoxelOptions { RecordExpansions = true })).Cells.Count > 0;

        Graph gr = Graph.From([(0, 1, 2.0), (1, 2, 3.0), (0, 2, 9.0)]);
        byte[] gs = gr.SaveState();
        gr.RemoveEdge(0);
        gr.RestoreState(gs);
        ok &= gs.AsSpan().SequenceEqual(gr.SaveState()) && gr.FindPath(0, 2).Cost == 5.0;

        try
        {
            Grid.FromState(saved.AsSpan(0, saved.Length - 1));
            ok = false;
        }
        catch (GridStateException)
        {
        }
        return ok;
    }
}
