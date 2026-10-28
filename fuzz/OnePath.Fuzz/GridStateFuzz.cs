using System;
using System.IO;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for the grid, voxel, graph, replanner, flow-field and sliced-search state readers. The first byte picks the object
/// kind; bit 7 of it says "patch" (the rest are (offset, value) pairs written into a valid state of that kind, so the fuzzer
/// gets past the header and into the counts and node lists) or "raw" (the rest is the state). Restore must throw the typed
/// <see cref="GridStateException"/> or succeed; after a successful restore a bounded follow-up (a query, a few steps, a
/// re-save that must restore to the same bytes) must not throw. Anything else crashes the target.
/// </summary>
internal static class GridStateFuzz
{
    private const int Kinds = 10;

    private static Grid Square()
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
        return g;
    }

    private static HexGrid Hex() => HexGrid.FromText("......\n.####.\n..2...\n......", HexLayout.PointyOddRow);

    private static VoxelGrid Voxel()
    {
        var v = new VoxelGrid(18, 4, 4);
        v.SetArea(17, 3, 3, 1);
        v.SetArea(5, 1, 1, AreaCosts.Wall);
        return v;
    }

    private static Graph MakeGraph()
    {
        Graph g = Graph.From([(0, 1, 2.0), (1, 2, 3.0), (2, 3, 1.0), (0, 3, 9.0)], directed: true);
        g.RemoveEdge(1);
        return g;
    }

    private static AStar<SquareTopology, DefaultStepCost, OctileHeuristic> MakeSearch(Grid g) =>
        new(g.CreateTopology(), default, new OctileHeuristic(g.Width), new GridSearchBuffers(g.CellCount) { RecordExpansions = true });

    // A valid state for each kind, in the order of the kind byte.
    private static byte[] Valid(int kind)
    {
        Grid g = Square();
        switch (kind)
        {
            case 0: return g.SaveState();
            case 1: return Hex().SaveState();
            case 2: return Voxel().SaveState();
            case 3: return MakeGraph().SaveState();
            case 4:
                {
                    GridReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(9, 5));
                    r.Replan(6);
                    return r.SaveState();
                }
            case 5:
                {
                    HexGrid h = Hex();
                    HexReplanner r = h.CreateReplanner(new Cell(0, 0), new Cell(5, 3));
                    r.Replan(5);
                    return r.SaveState();
                }
            case 6: return g.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 9 }).SaveState();
            case 7:
                {
                    GridFlowField f = g.CreateFlowField(new Cell(9, 5));
                    return f.SaveState();
                }
            case 8:
                {
                    var s = MakeSearch(g);
                    s.Begin(0, 59);
                    s.Step(11);
                    return s.SaveState();
                }
            default:
                {
                    var s = MakeSearch(g);
                    s.Begin(0, 59);
                    s.Run();
                    return s.SaveState();
                }
        }
    }

    public static void Run(byte[] data)
    {
        if (data.Length == 0)
        {
            return;
        }
        int kind = (data[0] & 0x7F) % Kinds;
        byte[] state;
        if ((data[0] & 0x80) != 0)
        {
            state = Valid(kind);
            for (int i = 1; i + 1 < data.Length; i += 2)
            {
                state[data[i] % state.Length] = data[i + 1];
            }
        }
        else
        {
            state = data[1..];
        }
        try
        {
            Restore(kind, state);
        }
        catch (GridStateException)
        {
        }
    }

    private static void Restore(int kind, byte[] state)
    {
        Grid g = Square();
        switch (kind)
        {
            case 0:
                {
                    g.RestoreState(state);
                    Again(g.SaveState, g.RestoreState);
                    _ = g.FindPath(new Cell(0, 0), new Cell(9, 5), new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 4 });
                    break;
                }
            case 1:
                {
                    HexGrid h = Hex();
                    h.RestoreState(state);
                    Again(h.SaveState, h.RestoreState);
                    break;
                }
            case 2:
                {
                    VoxelGrid v = Voxel();
                    v.RestoreState(state);
                    Again(v.SaveState, v.RestoreState);
                    _ = v.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(17, 3, 3));
                    break;
                }
            case 3:
                {
                    var gr = new Graph();
                    gr.RestoreState(state);
                    Again(gr.SaveState, gr.RestoreState);
                    if (gr.NodeCount is > 1 and <= 4096)
                    {
                        // Isolated nodes cost no bytes in a state, so a small state can name 2^26 nodes (as new Graph(n) can); the query's
                        // buffers would then be gigabytes. Restore itself allocates nothing for them.
                        _ = gr.FindPath(0, gr.NodeCount - 1);
                    }
                    break;
                }
            case 4:
                {
                    GridReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(9, 5));
                    r.RestoreState(state);
                    Again(r.SaveState, r.RestoreState);
                    try
                    {
                        _ = r.Replan(5000);
                    }
                    catch (InvalidOperationException)
                    {
                        // A restored state is trusted to hold consistent costs (docs/grids-state.md); an inconsistent one can fail to yield a path.
                    }
                    break;
                }
            case 5:
                {
                    HexGrid h = Hex();
                    HexReplanner r = h.CreateReplanner(new Cell(0, 0), new Cell(5, 3));
                    r.RestoreState(state);
                    Again(r.SaveState, r.RestoreState);
                    try
                    {
                        _ = r.Replan(5000);
                    }
                    catch (InvalidOperationException)
                    {
                    }
                    break;
                }
            case 6:
            case 7:
                {
                    GridFlowField f = g.CreateFlowField(new Cell(9, 5), new GridOptions { IterationBudget = 9 });
                    f.RestoreState(state);
                    Again(f.SaveState, f.RestoreState);
                    f.Step(50);
                    try
                    {
                        _ = f.GetPath(new Cell(0, 0));
                    }
                    catch (InvalidOperationException)
                    {
                        // A restored cost field is trusted to be consistent (docs/grids-state.md); an inconsistent one can have steps that cycle.
                    }
                    break;
                }
            default:
                {
                    var s = MakeSearch(g);
                    s.RestoreState(state);
                    Again(s.SaveState, s.RestoreState);
                    SearchStatus st = s.Step(500);
                    if (st == SearchStatus.Found)
                    {
                        _ = s.PathLength();
                    }
                    break;
                }
        }
    }

    // A state the library wrote must restore and write the same bytes again.
    private static void Again(Func<byte[]> save, RestoreAction restore)
    {
        byte[] first = save();
        restore(first);
        if (!first.AsSpan().SequenceEqual(save()))
        {
            throw new InvalidOperationException("A restored state did not save to the same bytes.");
        }
    }

    private delegate void RestoreAction(ReadOnlySpan<byte> state);

    internal static void WriteSeeds(string dir)
    {
        Directory.CreateDirectory(dir);
        for (int kind = 0; kind < Kinds; kind++)
        {
            byte[] valid = Valid(kind);
            File.WriteAllBytes(Path.Combine(dir, $"raw-{kind}.bin"), [(byte)kind, .. valid]);
            File.WriteAllBytes(Path.Combine(dir, $"patch-{kind}.bin"), [(byte)(0x80 | kind), 20, 0x7F, 12, 0x01]);
        }
    }
}
