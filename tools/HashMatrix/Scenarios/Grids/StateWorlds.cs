using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Threading;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Deterministic worlds for the rollback checks, shared by the tests and the hash-matrix runner. Each world runs a
/// seeded sequence of edits and queries, takes a snapshot, and then either carries on (<c>rollback: false</c>) or
/// wanders off in a different direction, restores the snapshot and carries on (<c>rollback: true</c>). The transcript of
/// everything after the snapshot point must be byte-identical both ways.
/// </summary>
internal static class StateWorlds
{
    internal static readonly string[] Kinds =
    [
        "grid-astar", "grid-jps", "grid-jpsplus", "grid-theta", "grid-lazytheta", "grid-hpa", "grid-4",
        "hex-astar", "voxel", "dstar", "lpa", "hex-dstar", "flow-partial", "flow-repair", "hex-flow-partial", "graph",
        "sliced-astar", "sliced-jps", "sliced-jpsplus", "sliced-theta", "sliced-graph",
    ];

    /// <summary>Little-endian transcript builder.</summary>
    internal sealed class Tx
    {
        private readonly List<byte> _bytes = [];
        private readonly byte[] _scratch = new byte[8];

        internal void I(int v)
        {
            BinaryPrimitives.WriteInt32LittleEndian(_scratch, v);
            _bytes.AddRange(_scratch.AsSpan(0, 4).ToArray());
        }

        internal void L(long v)
        {
            BinaryPrimitives.WriteInt64LittleEndian(_scratch, v);
            _bytes.AddRange(_scratch.AsSpan(0, 8).ToArray());
        }

        internal void D(double v)
        {
            BinaryPrimitives.WriteInt64LittleEndian(_scratch, BitConverter.DoubleToInt64Bits(v));
            _bytes.AddRange(_scratch.AsSpan(0, 8).ToArray());
        }

        internal void B(ReadOnlySpan<byte> b)
        {
            I(b.Length);
            _bytes.AddRange(b.ToArray());
        }

        internal void Path(GridPath p)
        {
            I((int)p.Status);
            D(p.Cost);
            I(p.Cells.Length);
            foreach (Cell c in p.Cells)
            {
                I(c.X);
                I(c.Y);
            }
            I(p.Points.Length);
            foreach (WorldPoint w in p.Points)
            {
                D(w.X);
                D(w.Y);
            }
            I(p.ExpandedCount);
            I(p.ExpandedCells.Length);
            foreach (Cell c in p.ExpandedCells)
            {
                I(c.X);
                I(c.Y);
            }
        }

        internal byte[] ToArray() => _bytes.ToArray();
    }

    /// <summary>Runs one world and returns the transcript after the snapshot point.</summary>
    internal static byte[] Run(string kind, ulong seed, bool rollback) => Go(kind, seed, rollback, out _);

    /// <summary>Returns the snapshot bytes the world takes (all of them, concatenated with lengths).</summary>
    internal static byte[] Snapshot(string kind, ulong seed)
    {
        Go(kind, seed, false, out byte[] snap);
        return snap;
    }

    private static byte[] Go(string kind, ulong seed, bool rollback, out byte[] snapshot)
    {
        switch (kind)
        {
            case "grid-astar": return SquareGrid(GridAlgorithm.AStar, true, seed, rollback, Connectivity.Eight, out snapshot);
            case "grid-4": return SquareGrid(GridAlgorithm.AStar, false, seed, rollback, Connectivity.Four, out snapshot);
            case "grid-jps": return SquareGrid(GridAlgorithm.Jps, true, seed, rollback, Connectivity.Eight, out snapshot);
            case "grid-jpsplus": return SquareGrid(GridAlgorithm.JpsPlus, true, seed, rollback, Connectivity.Eight, out snapshot);
            case "grid-theta": return SquareGrid(GridAlgorithm.ThetaStar, true, seed, rollback, Connectivity.Eight, out snapshot);
            case "grid-lazytheta": return SquareGrid(GridAlgorithm.LazyThetaStar, true, seed, rollback, Connectivity.Eight, out snapshot);
            case "grid-hpa": return SquareGrid(GridAlgorithm.Hpa, false, seed, rollback, Connectivity.Eight, out snapshot);
            case "hex-astar": return HexWorld(seed, rollback, out snapshot);
            case "voxel": return VoxelWorld(seed, rollback, out snapshot);
            case "dstar": return Replanners(GridAlgorithm.DStarLite, false, seed, rollback, out snapshot);
            case "lpa": return Replanners(GridAlgorithm.LpaStar, false, seed, rollback, out snapshot);
            case "hex-dstar": return Replanners(GridAlgorithm.DStarLite, true, seed, rollback, out snapshot);
            case "flow-partial": return Flow(false, false, seed, rollback, out snapshot);
            case "hex-flow-partial": return Flow(true, false, seed, rollback, out snapshot);
            case "flow-repair": return Flow(false, true, seed, rollback, out snapshot);
            case "graph": return GraphWorld(seed, rollback, out snapshot);
            case "sliced-astar": return Sliced(0, seed, rollback, out snapshot);
            case "sliced-jps": return Sliced(1, seed, rollback, out snapshot);
            case "sliced-jpsplus": return Sliced(2, seed, rollback, out snapshot);
            case "sliced-theta": return Sliced(3, seed, rollback, out snapshot);
            case "sliced-graph": return Sliced(4, seed, rollback, out snapshot);
            default: throw new ArgumentException("Unknown world " + kind, nameof(kind));
        }
    }

    // ---------- shared pieces ----------

    private static byte[] Concat(params byte[][] parts)
    {
        var tx = new Tx();
        foreach (byte[] p in parts)
        {
            tx.B(p);
        }
        return tx.ToArray();
    }

    private static Grid MakeGrid(ulong seed, int w, int h, bool varied)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        if (varied)
        {
            g.Areas[1] = 2.5;
            g.Areas[2] = 0.75;
        }
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                double r = rng.NextDouble();
                if (r < 0.2)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (varied && r < 0.35)
                {
                    g.SetArea(x, y, (byte)(1 + rng.NextInt(2)));
                }
            }
        }
        return g;
    }

    private static void EditGrid(Grid g, ref SeededRandom rng, int count, bool varied)
    {
        for (int i = 0; i < count; i++)
        {
            byte area = rng.NextInt(3) switch { 0 => AreaCosts.Wall, 1 => (byte)0, _ => varied ? (byte)(1 + rng.NextInt(2)) : (byte)0 };
            if (rng.NextInt(4) == 0)
            {
                int x = rng.NextInt(g.Width - 3);
                int y = rng.NextInt(g.Height - 3);
                g.SetRect(x, y, 1 + rng.NextInt(3), 1 + rng.NextInt(3), area);
            }
            else
            {
                g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), area);
            }
        }
    }

    private static bool PickWalkable(Grid g, ref SeededRandom rng, out Cell c)
    {
        for (int tries = 0; tries < 50; tries++)
        {
            c = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
            if (g.IsWalkable(c.X, c.Y))
            {
                return true;
            }
        }
        c = default;
        return false;
    }

    private static void SquareQueries(Grid g, GridAlgorithm alg, Connectivity conn, ref SeededRandom rng, int count, Tx tx)
    {
        for (int i = 0; i < count; i++)
        {
            if (!PickWalkable(g, ref rng, out Cell a) || !PickWalkable(g, ref rng, out Cell b))
            {
                continue;
            }
            var o = new GridOptions { Algorithm = alg, Connectivity = conn, RecordExpansions = true, HpaChunkSize = 8 };
            GridPath p = (i & 1) == 0 ? g.FindPath(a, b, o) : g.FindPath(g.CellCenter(a), g.CellCenter(b), o);
            tx.Path(p);
        }
    }

    // ---------- square grids ----------

    private static byte[] SquareGrid(GridAlgorithm alg, bool uniform, ulong seed, bool rollback, Connectivity conn, out byte[] snapshot)
    {
        bool varied = !uniform;
        Grid g = MakeGrid(seed, 34, 26, varied);
        var rng = new SeededRandom(seed ^ 0x1111);
        EditGrid(g, ref rng, 12, varied);
        SquareQueries(g, alg, conn, ref rng, 6, new Tx()); // warm every cache the algorithm keeps
        byte[] s = g.SaveState();
        snapshot = s;
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0x7777);
            g.Areas[0] = 3.0; // breaks uniform costs, changes the clearance and index rebuilds
            g.Areas[1] = 0.5;
            EditGrid(g, ref other, 40, varied);
            SquareQueries(g, alg, conn, ref other, 8, new Tx());
            g.Space = new GridSpace(10, 20, 2);
            g.RestoreState(s);
        }
        var tx = new Tx();
        EditGrid(g, ref rng, 9, varied);
        SquareQueries(g, alg, conn, ref rng, 14, tx);
        EditGrid(g, ref rng, 25, varied);
        SquareQueries(g, alg, conn, ref rng, 14, tx);
        tx.I(g.Version);
        tx.B(g.SaveState());
        return tx.ToArray();
    }

    // ---------- hex ----------

    private static byte[] HexWorld(ulong seed, bool rollback, out byte[] snapshot)
    {
        Grid sq = MakeGrid(seed, 30, 24, true);
        var g = HexGrid.From(sq.Cells, sq.Width, sq.Height, HexLayout.FlatOddColumn);
        g.Areas.CopyFrom(sq.Areas);
        var rng = new SeededRandom(seed ^ 0x2222);
        HexEdit(g, ref rng, 12);
        HexQueries(g, ref rng, 4, new Tx());
        byte[] s = g.SaveState();
        snapshot = s;
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0x8888);
            g.Areas[1] = 4.0;
            HexEdit(g, ref other, 40);
            HexQueries(g, ref other, 6, new Tx());
            g.RestoreState(s);
        }
        var tx = new Tx();
        HexEdit(g, ref rng, 10);
        HexQueries(g, ref rng, 12, tx);
        tx.I(g.Version);
        tx.B(g.SaveState());
        return tx.ToArray();
    }

    private static void HexEdit(HexGrid g, ref SeededRandom rng, int count)
    {
        for (int i = 0; i < count; i++)
        {
            byte area = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3);
            g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), area);
        }
    }

    private static bool HexPick(HexGrid g, ref SeededRandom rng, out Cell c)
    {
        for (int tries = 0; tries < 50; tries++)
        {
            c = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
            if (g.GetArea(c.X, c.Y) != AreaCosts.Wall)
            {
                return true;
            }
        }
        c = default;
        return false;
    }

    private static void HexQueries(HexGrid g, ref SeededRandom rng, int count, Tx tx)
    {
        for (int i = 0; i < count; i++)
        {
            if (HexPick(g, ref rng, out Cell a) && HexPick(g, ref rng, out Cell b))
            {
                tx.Path(g.FindPath(a, b, new GridOptions { RecordExpansions = true }));
            }
        }
    }

    // ---------- voxel ----------

    private static void VoxelEdit(VoxelGrid g, ref SeededRandom rng, int count)
    {
        for (int i = 0; i < count; i++)
        {
            byte area = rng.NextInt(3) == 0 ? (byte)0 : AreaCosts.Wall;
            g.SetArea(rng.NextInt(g.SizeX), rng.NextInt(g.SizeY), rng.NextInt(g.SizeZ), area);
        }
    }

    private static void VoxelQueries(VoxelGrid g, ref SeededRandom rng, int count, Tx tx)
    {
        for (int i = 0; i < count; i++)
        {
            var a = new VoxelCell(rng.NextInt(g.SizeX), rng.NextInt(g.SizeY), rng.NextInt(g.SizeZ));
            var b = new VoxelCell(rng.NextInt(g.SizeX), rng.NextInt(g.SizeY), rng.NextInt(g.SizeZ));
            VoxelPath p = (i & 1) == 0 ? g.FindPath(a, b, new VoxelOptions { RecordExpansions = true }) : g.FindPath(g.CellCenter(a), g.CellCenter(b), new VoxelOptions { RecordExpansions = true });
            tx.I((int)p.Status);
            tx.D(p.Cost);
            tx.I(p.ExpandedCount);
            foreach (VoxelCell c in p.Cells)
            {
                tx.I(c.X);
                tx.I(c.Y);
                tx.I(c.Z);
            }
            tx.I(p.ExpandedCells.Length);
            foreach (VoxelPoint w in p.Points)
            {
                tx.D(w.X);
                tx.D(w.Y);
                tx.D(w.Z);
            }
        }
    }

    private static byte[] VoxelWorld(ulong seed, bool rollback, out byte[] snapshot)
    {
        var g = new VoxelGrid(18, 14, 12);
        var rng = new SeededRandom(seed ^ 0x3333);
        VoxelEdit(g, ref rng, 700);
        VoxelQueries(g, ref rng, 3, new Tx());
        byte[] s = g.SaveState();
        snapshot = s;
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0x9999);
            g.Areas[0] = 2.0;
            VoxelEdit(g, ref other, 900);
            VoxelQueries(g, ref other, 3, new Tx());
            g.RestoreState(s);
        }
        var tx = new Tx();
        VoxelEdit(g, ref rng, 100);
        VoxelQueries(g, ref rng, 8, tx);
        tx.I(g.Version);
        tx.I(g.AllocatedChunks);
        tx.B(g.SaveState());
        return tx.ToArray();
    }

    // ---------- replanners ----------

    private sealed class Rep
    {
        internal Grid? Square;
        internal HexGrid? Hex;
        internal GridReplanner? SquareRep;
        internal HexReplanner? HexRep;

        internal Cell Start => SquareRep?.Start ?? HexRep!.Start;

        internal Cell Goal => SquareRep?.Goal ?? HexRep!.Goal;
    }

    private static Rep NewRep(GridAlgorithm alg, bool hex, ulong seed)
    {
        var o = new GridOptions { Algorithm = alg, RecordExpansions = true };
        var r = new Rep();
        if (hex)
        {
            Grid sq = MakeGrid(seed, 26, 20, true);
            r.Hex = HexGrid.From(sq.Cells, sq.Width, sq.Height, HexLayout.PointyOddRow);
            r.Hex.Areas.CopyFrom(sq.Areas);
            r.HexRep = r.Hex.CreateReplanner(new Cell(1, 1), new Cell(r.Hex.Width - 2, r.Hex.Height - 2), o);
        }
        else
        {
            r.Square = MakeGrid(seed, 30, 22, true);
            r.SquareRep = r.Square.CreateReplanner(new Cell(1, 1), new Cell(r.Square.Width - 2, r.Square.Height - 2), o);
        }
        return r;
    }

    private static void RepSteps(Rep r, ref SeededRandom rng, int steps, Tx tx)
    {
        for (int s = 0; s < steps; s++)
        {
            int w = r.Square?.Width ?? r.Hex!.Width;
            int h = r.Square?.Height ?? r.Hex!.Height;
            for (int e = 0; e < 1 + rng.NextInt(5); e++)
            {
                byte area = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3);
                int x = rng.NextInt(w);
                int y = rng.NextInt(h);
                if (r.Square is not null)
                {
                    if (rng.NextInt(5) == 0)
                    {
                        r.Square.SetRect(Math.Min(x, w - 3), Math.Min(y, h - 3), 3, 2, area);
                    }
                    else
                    {
                        r.Square.SetArea(x, y, area);
                    }
                }
                else
                {
                    r.Hex!.SetArea(x, y, area);
                }
            }
            int kind = rng.NextInt(8);
            var move = new Cell(rng.NextInt(w), rng.NextInt(h));
            if (kind == 0)
            {
                r.SquareRep?.MoveStart(move);
                r.HexRep?.MoveStart(move);
            }
            else if (kind == 1)
            {
                r.SquareRep?.SetGoal(move);
                r.HexRep?.SetGoal(move);
            }
            else if (kind == 2 && r.Square is not null)
            {
                r.Square.Areas[1] = 1.0 + rng.NextInt(5);
            }
            int budget = rng.NextInt(3) == 0 ? 1 + rng.NextInt(40) : int.MaxValue;
            GridPath p = r.SquareRep?.Replan(budget) ?? r.HexRep!.Replan(budget);
            tx.Path(p);
            tx.D(r.SquareRep?.TotalExpanded ?? r.HexRep!.TotalExpanded);
            tx.D(r.SquareRep?.TotalIterations ?? r.HexRep!.TotalIterations);
            tx.I(r.Start.X);
            tx.I(r.Goal.Y);
            if (budget != int.MaxValue)
            {
                p = r.SquareRep?.Replan() ?? r.HexRep!.Replan();
                tx.Path(p);
            }
        }
    }

    private static byte[] Replanners(GridAlgorithm alg, bool hex, ulong seed, bool rollback, out byte[] snapshot)
    {
        Rep r = NewRep(alg, hex, seed);
        var rng = new SeededRandom(seed ^ 0x4444);
        RepSteps(r, ref rng, 6, new Tx());
        // Stop in the middle of a budgeted replan so the saved state has a half-settled queue.
        r.SquareRep?.Replan(7);
        r.HexRep?.Replan(7);
        byte[] gridState = r.Square?.SaveState() ?? r.Hex!.SaveState();
        byte[] repState = r.SquareRep?.SaveState() ?? r.HexRep!.SaveState();
        snapshot = Concat(gridState, repState);
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0xAAAA);
            RepSteps(r, ref other, 7, new Tx());
            if (r.Square is not null)
            {
                r.Square.RestoreState(gridState);
                r.SquareRep!.RestoreState(repState);
            }
            else
            {
                r.Hex!.RestoreState(gridState);
                r.HexRep!.RestoreState(repState);
            }
        }
        var tx = new Tx();
        RepSteps(r, ref rng, 9, tx);
        tx.B(r.SquareRep?.SaveState() ?? r.HexRep!.SaveState());
        return tx.ToArray();
    }

    // ---------- flow fields ----------

    private static void PutField(GridFlowField f, Tx tx)
    {
        tx.I((int)f.Status);
        tx.I(f.Settled);
        foreach (double d in f.Costs)
        {
            tx.D(d);
        }
        foreach (int n in f.Next)
        {
            tx.I(n);
        }
    }

    private static byte[] Flow(bool hex, bool repair, ulong seed, bool rollback, out byte[] snapshot)
    {
        var o = new GridOptions { IterationBudget = repair ? int.MaxValue : 60 };
        Grid? sq = null;
        HexGrid? hx = null;
        if (hex)
        {
            Grid t = MakeGrid(seed, 28, 22, true);
            hx = HexGrid.From(t.Cells, t.Width, t.Height, HexLayout.FlatOddColumn);
            hx.Areas.CopyFrom(t.Areas);
        }
        else
        {
            sq = MakeGrid(seed, 32, 24, true);
        }
        int w = sq?.Width ?? hx!.Width;
        int h = sq?.Height ?? hx!.Height;
        var rng = new SeededRandom(seed ^ 0x5555);
        Cell[] goals = [new Cell(rng.NextInt(w), rng.NextInt(h)), new Cell(rng.NextInt(w), rng.NextInt(h))];
        GridFlowField f = sq is not null ? sq.CreateFlowField(goals, o) : hx!.CreateFlowField(goals, o);
        Action<int> edit = n =>
        {
            for (int i = 0; i < n; i++)
            {
                byte area = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3);
                int x = rng.NextInt(w);
                int y = rng.NextInt(h);
                if (sq is not null)
                {
                    sq.SetArea(x, y, area);
                }
                else
                {
                    hx!.SetArea(x, y, area);
                }
            }
        };
        if (repair)
        {
            f.Run();
            edit(6);
            using var cancelled = new CancellationTokenSource();
            cancelled.Cancel();
            try
            {
                f.Refresh(cancelled.Token); // stops straight after the repair is set up: a half-done repair
            }
            catch (OperationCanceledException)
            {
            }
        }
        else
        {
            f.Step(40);
        }
        byte[] gridState = sq?.SaveState() ?? hx!.SaveState();
        byte[] fieldState = f.SaveState();
        snapshot = Concat(gridState, fieldState);
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0xBBBB);
            for (int i = 0; i < 30; i++)
            {
                byte area = other.NextInt(2) == 0 ? AreaCosts.Wall : (byte)0;
                if (sq is not null)
                {
                    sq.SetArea(other.NextInt(w), other.NextInt(h), area);
                }
                else
                {
                    hx!.SetArea(other.NextInt(w), other.NextInt(h), area);
                }
            }
            f.Run();
            f.Refresh();
            if (sq is not null)
            {
                sq.RestoreState(gridState);
            }
            else
            {
                hx!.RestoreState(gridState);
            }
            f.RestoreState(fieldState);
        }
        var tx = new Tx();
        PutField(f, tx);
        f.Run();
        PutField(f, tx);
        for (int round = 0; round < 4; round++)
        {
            edit(5);
            f.Refresh();
            PutField(f, tx);
        }
        if (sq is not null)
        {
            sq.Areas[1] = 4.0;
            f.Refresh();
            PutField(f, tx);
        }
        tx.B(f.SaveState());
        return tx.ToArray();
    }

    // ---------- graphs ----------

    private static Graph MakeGraph(ulong seed, int nodes, int edges)
    {
        var rng = new SeededRandom(seed);
        var g = new Graph(nodes);
        for (int i = 0; i < edges; i++)
        {
            g.AddEdge(rng.NextInt(nodes), rng.NextInt(nodes), 1.0 + rng.NextInt(20));
        }
        return g;
    }

    private static void GraphEdit(Graph g, ref SeededRandom rng, int count, Tx tx)
    {
        for (int i = 0; i < count; i++)
        {
            switch (rng.NextInt(4))
            {
                case 0:
                    tx.I(g.AddEdge(rng.NextInt(g.NodeCount), rng.NextInt(g.NodeCount), 1.0 + rng.NextInt(20)));
                    break;
                case 1:
                    tx.I(g.RemoveEdge(rng.NextInt(Math.Max(1, g.EdgeCount))) ? 1 : 0);
                    break;
                case 2:
                    tx.I(g.AddNode());
                    break;
                default:
                    try
                    {
                        g.SetWeight(rng.NextInt(40), 0.5 + rng.NextInt(30));
                        tx.I(1);
                    }
                    catch (ArgumentException)
                    {
                        tx.I(0); // that edge was removed
                    }
                    break;
            }
        }
    }

    private static byte[] GraphWorld(ulong seed, bool rollback, out byte[] snapshot)
    {
        Graph g = MakeGraph(seed, 70, 260);
        var rng = new SeededRandom(seed ^ 0x6666);
        GraphEdit(g, ref rng, 15, new Tx());
        _ = g.FindPath(0, 5);
        byte[] s = g.SaveState();
        snapshot = s;
        if (rollback)
        {
            var other = new SeededRandom(seed ^ 0xCCCC);
            GraphEdit(g, ref other, 60, new Tx());
            _ = g.FindPath(1, 6);
            g.RestoreState(s);
        }
        var tx = new Tx();
        GraphEdit(g, ref rng, 12, tx);
        tx.I(g.NodeCount);
        tx.I(g.EdgeCount);
        tx.L((long)g.Compile().Fingerprint());
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 4 });
        for (int i = 0; i < 12; i++)
        {
            int a = rng.NextInt(g.NodeCount);
            int b = rng.NextInt(g.NodeCount);
            foreach (GraphOptions o in new[] { new GraphOptions { RecordExpansions = true }, GraphOptions.Bidirectional, GraphOptions.WithLandmarks(lm) })
            {
                GraphPath p = g.FindPath(a, b, o);
                tx.I((int)p.Status);
                tx.D(p.Cost);
                tx.I(p.ExpandedCount);
                foreach (int n in p.Nodes)
                {
                    tx.I(n);
                }
                foreach (int n in p.ExpandedNodes)
                {
                    tx.I(n);
                }
            }
        }
        tx.B(g.SaveState());
        return tx.ToArray();
    }

    // ---------- sliced searches ----------

    private static void PutSearch(Tx tx, SearchStatus status, int expanded, int iterations, double cost, GridSearchBuffers b)
    {
        tx.I((int)status);
        tx.I(expanded);
        tx.I(iterations);
        tx.D(cost);
        tx.I(b.ExpandedNodes.Length);
        foreach (int n in b.ExpandedNodes)
        {
            tx.I(n);
        }
    }

    private static byte[] Sliced(int which, ulong seed, bool rollback, out byte[] snapshot)
    {
        Grid g = MakeGrid(seed, 40, 30, false);
        var rng = new SeededRandom(seed ^ 0xDDDD);
        Graph gr = MakeGraph(seed, 120, 500);
        PickWalkable(g, ref rng, out Cell a);
        PickWalkable(g, ref rng, out Cell b);
        int start = (a.Y * g.Width) + a.X;
        int goal = (b.Y * g.Width) + b.X;
        int gstart = rng.NextInt(120);
        int ggoal = rng.NextInt(120);
        int w = g.Width;
        using var index = new JpsPlusIndex(g);
        using var map = new BlockMap(g);
        CsrGraph csr = gr.Compile();
        GridSearchBuffers Buf() => new(Math.Max(g.CellCount, 120)) { RecordExpansions = true };

        // One closure per search kind: make, begin, step, status, counters, path.
        Func<(Func<int, SearchStatus> step, Func<byte[]> save, Action<byte[]> restore, Func<(SearchStatus, int, int, double, GridSearchBuffers)> look, Action<Tx> tail)> make = which switch
        {
            0 => () =>
            {
                var s = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(w), Buf());
                s.Begin(start, goal);
                return (s.Step, s.SaveState, x => s.RestoreState(x), () => (s.Status, s.ExpandedCount, s.Iterations, s.PathCost, s.Buffers), tx => PathTail(tx, s));
            },
            1 => () =>
            {
                var s = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(map), new OctileHeuristic(w), 1.0, Buf());
                s.Begin(start, goal);
                return (s.Step, s.SaveState, x => s.RestoreState(x), () => (s.Status, s.ExpandedCount, s.Iterations, s.PathCost, s.Buffers), tx => PathTail(tx, s));
            },
            2 => () =>
            {
                var s = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(w), 1.0, Buf());
                s.Begin(start, goal);
                return (s.Step, s.SaveState, x => s.RestoreState(x), () => (s.Status, s.ExpandedCount, s.Iterations, s.PathCost, s.Buffers), tx => PathTail(tx, s));
            },
            3 => () =>
            {
                var s = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(w), ThetaVariant.LazyRequeue, 1.0, Buf());
                s.Begin(start, goal);
                return (s.Step, s.SaveState, x => s.RestoreState(x), () => (s.Status, s.ExpandedCount, s.Iterations, s.PathCost, s.Buffers), tx => PathTail(tx, s));
            },
            _ => () =>
            {
                var s = new GraphAStar<DefaultGraphCost, ZeroGraphHeuristic>(csr, default, default, Buf());
                s.Begin(gstart, ggoal);
                return (s.Step, s.SaveState, x => s.RestoreState(x), () => (s.Status, s.ExpandedCount, s.Iterations, s.PathCost, s.Buffers), tx => PathTail(tx, s));
            },
        };

        var first = make();
        int k = 1 + rng.NextInt(60);
        first.step(k);
        byte[] saved = first.save();
        snapshot = saved;
        var current = first;
        if (rollback)
        {
            first.step(1 + new SeededRandom(seed ^ 0xEEEE).NextInt(200)); // move on, then come back through a second, fresh object
            current = make();
            current.restore(saved);
        }
        var t = new Tx();
        int guard = 0;
        while (current.look().Item1 == SearchStatus.Running && guard++ < 100000)
        {
            SearchStatus st = current.step(1 + rng.NextInt(25));
            (SearchStatus status, int expanded, int iterations, double cost, GridSearchBuffers buf) = current.look();
            PutSearch(t, st, expanded, iterations, cost, buf);
        }
        current.tail(t);
        t.B(current.save());
        return t.ToArray();
    }

    private static void PathTail<T1, T2, T3>(Tx tx, AStar<T1, T2, T3> s)
        where T1 : struct, IGridTopology
        where T2 : struct, IStepCost
        where T3 : struct, IHeuristic
    {
        if (s.Status == SearchStatus.Found)
        {
            var path = new int[s.PathLength()];
            s.CopyPath(path);
            foreach (int n in path)
            {
                tx.I(n);
            }
        }
    }

    private static void PathTail<T1, T2>(Tx tx, JumpSearch<T1, T2> s)
        where T1 : struct, IJumper
        where T2 : struct, IHeuristic
    {
        if (s.Status == SearchStatus.Found)
        {
            var path = new int[s.PathLength()];
            s.CopyPath(path);
            foreach (int n in path)
            {
                tx.I(n);
            }
        }
    }

    private static void PathTail<T1>(Tx tx, ThetaSearch<T1> s)
        where T1 : struct, IHeuristic
    {
        if (s.Status == SearchStatus.Found)
        {
            var path = new int[s.PathLength()];
            s.CopyPath(path);
            foreach (int n in path)
            {
                tx.I(n);
            }
        }
    }

    private static void PathTail<T1, T2>(Tx tx, GraphAStar<T1, T2> s)
        where T1 : struct, IGraphCost
        where T2 : struct, IGraphHeuristic
    {
        if (s.Status == SearchStatus.Found)
        {
            var path = new int[s.PathLength()];
            s.CopyPath(path);
            foreach (int n in path)
            {
                tx.I(n);
            }
        }
    }
}
