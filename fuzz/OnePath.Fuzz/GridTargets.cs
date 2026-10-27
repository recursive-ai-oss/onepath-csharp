using System;
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
