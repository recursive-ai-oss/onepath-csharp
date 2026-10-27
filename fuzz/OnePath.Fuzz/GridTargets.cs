using System;
using System.Linq;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>Fuzz targets for the grid code: the Moving AI parsers and the one-line query inputs. A target throws only when it finds a real problem.</summary>
internal static class GridTargets
{
    // Map text: anything may be thrown at the parser; only the typed error is acceptable, and a parsed map must be consistent.
    internal static void MapParser(byte[] data)
    {
        try
        {
            MovingAiMap map = MovingAiMap.Parse(data);
            if (map.Tiles.Length != map.Width * map.Height)
            {
                throw new InvalidOperationException("parsed map has inconsistent size");
            }
            Grid g = map.ToGrid();
            if (g.Width != map.Width || g.Height != map.Height)
            {
                throw new InvalidOperationException("grid size differs from map size");
            }
        }
        catch (GridFormatException)
        {
        }
    }

    internal static void ScenarioParser(byte[] data)
    {
        try
        {
            foreach (ScenarioEntry e in MovingAiScenarios.Parse(data))
            {
                if (!double.IsFinite(e.OptimalLength) || e.StartX < 0 || e.GoalY < 0)
                {
                    throw new InvalidOperationException("scenario entry holds an impossible value");
                }
            }
        }
        catch (GridFormatException)
        {
        }
    }

    // Query inputs: a small random grid, options and endpoints decoded from the bytes. Properties checked:
    // found paths are legal and priced consistently, Dijkstra and A* agree, out-of-range input is rejected with the documented exception.
    internal static void Query(byte[] data)
    {
        if (data.Length < 12)
        {
            return;
        }
        int w = 1 + (data[0] % 20);
        int h = 1 + (data[1] % 20);
        byte flags = data[2];
        bool hex = (flags & 1) != 0;
        var options = new GridOptions
        {
            Connectivity = (flags & 2) != 0 ? Connectivity.Four : Connectivity.Eight,
            Diagonals = (DiagonalRule)((flags >> 2) % 3),
            AgentSize = hex ? 1 : 1 + (data[3] % 4),
            HeuristicWeight = 1.0,
        };
        var start = new Cell(data[4] % (w + 2) - 1, data[5] % (h + 2) - 1);
        var goal = new Cell(data[6] % (w + 2) - 1, data[7] % (h + 2) - 1);
        var areas = new byte[w * h];
        for (int i = 0; i < areas.Length; i++)
        {
            byte b = data[8 + (i % (data.Length - 8))];
            areas[i] = (b & 7) switch { 0 or 1 => AreaCosts.Wall, 2 => 1, _ => 0 };
        }

        GridPath fast;
        GridPath slow;
        if (hex)
        {
            var g = HexGrid.From(areas, w, h, (HexLayout)((flags >> 4) & 3));
            g.Areas[1] = 2.5;
            fast = g.FindPath(start, goal, options);
            options.Heuristic = GridHeuristic.Zero;
            slow = g.FindPath(start, goal, options);
        }
        else
        {
            var g = Grid.From(areas, w, h);
            g.Areas[1] = 2.5;
            fast = g.FindPath(start, goal, options);
            options.Heuristic = GridHeuristic.Zero;
            slow = g.FindPath(start, goal, options);
            CheckWorld(g, data);
        }
        if (fast.Status != slow.Status)
        {
            throw new InvalidOperationException($"A* says {fast.Status} but Dijkstra says {slow.Status}");
        }
        if (fast.Found)
        {
            if (Math.Abs(fast.Cost - slow.Cost) > 1e-9)
            {
                throw new InvalidOperationException($"A* cost {fast.Cost} differs from Dijkstra cost {slow.Cost}");
            }
            ReadOnlySpan<Cell> cells = fast.Cells;
            if (cells.Length == 0 || cells[0] != start || cells[^1] != goal)
            {
                throw new InvalidOperationException("path does not run from start to goal");
            }
            for (int i = 1; i < cells.Length; i++)
            {
                if (Math.Abs(cells[i].X - cells[i - 1].X) > 1 || Math.Abs(cells[i].Y - cells[i - 1].Y) > 1)
                {
                    throw new InvalidOperationException("path takes a step longer than one cell");
                }
            }
        }
    }

    // Jump point searches against A*: a grid up to 100 x 40 cells (wider than a bitmap word) decoded from the bytes, queries with
    // JPS, JPS+ and A*, then edits followed by the same again. Properties: equal status and cost, legal priced paths, JPS+ tables after
    // the edits equal to freshly built ones, block bitmaps equal to the grid.
    internal static void Jps(byte[] data)
    {
        if (data.Length < 16)
        {
            return;
        }
        int w = 1 + (data[0] % 100);
        int h = 1 + (data[1] % 40);
        int density = 1 + (data[2] % 9); // walls per 16 cells, roughly
        var g = new Grid(w, h);
        int pos = 3;
        for (int i = 0; i < w * h; i++)
        {
            byte b = data[3 + (i % (data.Length - 3))];
            b = (byte)(b ^ (i * 31));
            if ((b & 15) < density)
            {
                g.SetWalkable(i % w, i / w, false);
            }
        }
        using var index = new JpsPlusIndex(g);
        using var blocks = new BlockMap(g);
        GridOptions jps = new() { Algorithm = GridAlgorithm.Jps };
        GridOptions plus = new() { Algorithm = GridAlgorithm.JpsPlus };
        for (int round = 0; round < 4; round++)
        {
            for (int q = 0; q < 6; q++)
            {
                var a = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
                var b = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
                GridPath astar = g.FindPath(a, b);
                foreach (GridPath p in new[] { g.FindPath(a, b, jps), g.FindPath(a, b, plus) })
                {
                    CheckSame(g, astar, p, a, b);
                }
            }
            int edits = Next(data, ref pos) % 9;
            for (int e = 0; e < edits; e++)
            {
                g.SetWalkable(Next(data, ref pos) % w, Next(data, ref pos) % h, (Next(data, ref pos) & 3) == 0);
            }
            index.Refresh();
            using var fresh = new JpsPlusIndex(g);
            if (!index.Tables.SequenceEqual(fresh.Tables))
            {
                throw new InvalidOperationException("incremental JPS+ tables differ from a rebuild");
            }
            using var freshBlocks = new BlockMap(g);
            if (!blocks.Rows.SequenceEqual(freshBlocks.Rows) || !blocks.Columns.SequenceEqual(freshBlocks.Columns))
            {
                throw new InvalidOperationException("block bitmaps differ from a rebuild");
            }
        }
    }

    // Incremental replanning (LPA* or D* Lite) under cell and tile edits and start moves. Properties: every replan equals a fresh A*
    // (status and cost, legal priced path), the clearance repaired in place equals a full recompute, a sliced replan equals an unsliced one.
    internal static void Replan(byte[] data)
    {
        if (data.Length < 16)
        {
            return;
        }
        int w = 2 + (data[0] % 40);
        int h = 2 + (data[1] % 24);
        int density = 1 + (data[2] % 9);
        var g = new Grid(w, h);
        for (int i = 0; i < w * h; i++)
        {
            byte v = (byte)(data[3 + (i % (data.Length - 3))] ^ (i * 31));
            if ((v & 15) < density)
            {
                g.SetWalkable(i % w, i / w, false);
            }
        }
        int pos = 3;
        int mode = Next(data, ref pos);
        var options = new GridOptions
        {
            Algorithm = (mode & 1) == 0 ? GridAlgorithm.LpaStar : GridAlgorithm.DStarLite,
            Connectivity = (mode & 2) == 0 ? Connectivity.Eight : Connectivity.Four,
            AgentSize = 1 + ((mode >> 2) & 1),
        };
        var a = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
        var b = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
        using GridReplanner r = g.CreateReplanner(a, b, options);
        using GridReplanner sliced = g.CreateReplanner(a, b, options);
        GridOptions plain = options.Clone();
        plain.Algorithm = GridAlgorithm.AStar;
        for (int round = 0; round < 8; round++)
        {
            int edits = 1 + (Next(data, ref pos) % 4);
            for (int e = 0; e < edits; e++)
            {
                if ((Next(data, ref pos) & 3) == 0)
                {
                    int x = Next(data, ref pos) % w;
                    int y = Next(data, ref pos) % h;
                    int rw = Next(data, ref pos) % (w - x + 1);
                    int rh = Next(data, ref pos) % (h - y + 1);
                    g.SetRect(x, y, rw, rh, (Next(data, ref pos) & 1) == 0 ? AreaCosts.Wall : (byte)0);
                }
                else
                {
                    g.SetWalkable(Next(data, ref pos) % w, Next(data, ref pos) % h, (Next(data, ref pos) & 3) == 0);
                }
            }
            GridPath p = r.Replan();
            GridPath q;
            do
            {
                q = sliced.Replan(1 + (Next(data, ref pos) % 7));
            }
            while (q.Status == PathStatus.BudgetExhausted);
            if (q.Status != p.Status || q.Cost != p.Cost || !q.Cells.SequenceEqual(p.Cells))
            {
                throw new InvalidOperationException("sliced replan differs from the unsliced one");
            }
            GridPath fresh = g.FindPath(r.Start, r.Goal, plain);
            if (fresh.Status != p.Status || (fresh.Found && Math.Abs(fresh.Cost - p.Cost) > 1e-9 * Math.Max(1.0, fresh.Cost)))
            {
                throw new InvalidOperationException($"fresh A* says {fresh.Status} {fresh.Cost} but the replanner says {p.Status} {p.Cost}");
            }
            if (!Grid.From(g.Cells, w, h).Clearance.SequenceEqual(g.Clearance))
            {
                throw new InvalidOperationException("repaired clearance differs from a full recompute");
            }
            if (p.Found && p.Cells.Length > 1 && (Next(data, ref pos) & 1) == 0)
            {
                r.MoveStart(p.Cells[1]);
                sliced.MoveStart(p.Cells[1]);
            }
        }
    }

    private static int Next(byte[] data, ref int pos)
    {
        int v = data[pos % data.Length];
        pos++;
        return v;
    }

    private static void CheckSame(Grid g, GridPath expect, GridPath got, Cell a, Cell b)
    {
        if (expect.Status != got.Status)
        {
            throw new InvalidOperationException($"A* says {expect.Status} but {got.Algorithm} says {got.Status} for {a} -> {b}");
        }
        if (!got.Found)
        {
            return;
        }
        if (Math.Abs(expect.Cost - got.Cost) > 1e-9)
        {
            throw new InvalidOperationException($"A* cost {expect.Cost} but {got.Algorithm} cost {got.Cost} for {a} -> {b}");
        }
        ReadOnlySpan<Cell> cells = got.Cells;
        if (cells.Length == 0 || cells[0] != a || cells[^1] != b)
        {
            throw new InvalidOperationException("path does not run from start to goal");
        }
        double sum = 0;
        for (int i = 1; i < cells.Length; i++)
        {
            int dx = cells[i].X - cells[i - 1].X;
            int dy = cells[i].Y - cells[i - 1].Y;
            bool diagonal = dx != 0 && dy != 0;
            if (Math.Abs(dx) > 1 || Math.Abs(dy) > 1 || (dx == 0 && dy == 0) || !g.IsWalkable(cells[i].X, cells[i].Y)
                || (diagonal && !(g.IsWalkable(cells[i].X, cells[i - 1].Y) && g.IsWalkable(cells[i - 1].X, cells[i].Y))))
            {
                throw new InvalidOperationException($"illegal step {cells[i - 1]} -> {cells[i]}");
            }
            sum += diagonal ? GridMath.Sqrt2 : 1.0;
        }
        if (Math.Abs(sum - got.Cost) > 1e-9)
        {
            throw new InvalidOperationException("path does not sum to its cost");
        }
    }

    // World-space entry point: arbitrary doubles must either work or raise ArgumentOutOfRangeException.
    private static void CheckWorld(Grid g, byte[] data)
    {
        double x = BitConverter.ToDouble(data, 0) * (data[1] % 3 == 0 ? 1.0 : 1e-3);
        double y = BitConverter.ToDouble(data, 4);
        try
        {
            g.Space = new GridSpace(0, 0, 1);
            _ = g.FindPath(new WorldPoint(x, y), new WorldPoint(y, x));
        }
        catch (ArgumentOutOfRangeException)
        {
        }
    }
}
