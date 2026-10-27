using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Fuzz;

internal static class FlowTargets
{
    // Flow fields on a grid (square or hex) up to 40 x 24 decoded from the bytes, with 1-3 goals: following the direction field from
    // every cell reaches a goal at the cell's cost, then edit rounds: the repaired field must equal a freshly computed one bit for
    // bit, and a sliced computation must equal the unsliced one.
    internal static void Run(byte[] data)
    {
        if (data.Length < 16)
        {
            return;
        }
        int w = 1 + (data[0] % 40);
        int h = 1 + (data[1] % 24);
        int density = 1 + (data[2] % 9);
        bool hex = (data[3] & 1) != 0;
        int size = hex ? 1 : 1 + ((data[3] >> 1) % 3);
        var options = new GridOptions
        {
            AgentSize = size,
            Connectivity = (data[3] & 8) != 0 ? Connectivity.Four : Connectivity.Eight,
            Diagonals = (DiagonalRule)((data[3] >> 4) % 3),
        };
        var g = new Grid(w, h);
        var hg = new HexGrid(w, h);
        for (int i = 0; i < w * h; i++)
        {
            byte b = (byte)(data[4 + (i % (data.Length - 4))] ^ (i * 31));
            if ((b & 15) < density)
            {
                g.SetArea(i % w, i / w, AreaCosts.Wall);
                hg.SetArea(i % w, i / w, AreaCosts.Wall);
            }
            else if ((b >> 6) == 1)
            {
                g.SetArea(i % w, i / w, 1);
                hg.SetArea(i % w, i / w, 1);
            }
        }
        g.Areas[1] = 2.5;
        hg.Areas[1] = 2.5;
        int pos = 4;
        int goalCount = 1 + (Next(data, ref pos) % 3);
        var goals = new Cell[goalCount];
        for (int i = 0; i < goalCount; i++)
        {
            goals[i] = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
        }
        using GridFlowField f = hex ? hg.CreateFlowField(goals, options) : g.CreateFlowField(goals, options);
        for (int round = 0; round < 4; round++)
        {
            CheckFollow(f, goals, w, h);
            using GridFlowField fresh = hex ? hg.CreateFlowField(goals, options) : g.CreateFlowField(goals, options);
            CheckSame(f, fresh);
            using GridFlowField sliced = hex ? hg.CreateFlowField(goals, new GridOptions { AgentSize = 1, IterationBudget = 1 + (Next(data, ref pos) % 17) })
                                             : g.CreateFlowField(goals, new GridOptions { AgentSize = size, Connectivity = options.Connectivity, Diagonals = options.Diagonals, IterationBudget = 1 + (Next(data, ref pos) % 17) });
            while (sliced.Status != FlowFieldStatus.Complete)
            {
                sliced.Step(1 + (Next(data, ref pos) % 17));
            }
            CheckSame(sliced, fresh);
            int edits = 1 + (Next(data, ref pos) % 8);
            for (int e = 0; e < edits; e++)
            {
                int x = Next(data, ref pos) % w;
                int y = Next(data, ref pos) % h;
                byte area = (Next(data, ref pos) & 3) == 0 ? AreaCosts.Wall : (byte)(Next(data, ref pos) & 1);
                if (hex)
                {
                    hg.SetArea(x, y, area);
                }
                else
                {
                    g.SetArea(x, y, area);
                }
            }
            f.Refresh();
        }
    }

    private static void CheckFollow(GridFlowField f, Cell[] goals, int w, int h)
    {
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                Cell[] path = f.GetPath(new Cell(x, y));
                bool reachable = !double.IsPositiveInfinity(f.GetCost(new Cell(x, y)));
                if (reachable != (path.Length > 0))
                {
                    throw new InvalidOperationException($"cell ({x},{y}) reachability disagrees with its path");
                }
                if (reachable && Array.IndexOf(goals, path[^1]) < 0)
                {
                    throw new InvalidOperationException($"path from ({x},{y}) does not end at a goal");
                }
            }
        }
    }

    private static void CheckSame(GridFlowField a, GridFlowField b)
    {
        if (!a.Costs.SequenceEqual(b.Costs) || !a.Next.SequenceEqual(b.Next))
        {
            throw new InvalidOperationException("repaired or sliced flow field differs from a fresh one");
        }
    }

    private static int Next(byte[] data, ref int pos)
    {
        int v = data[pos % data.Length];
        pos++;
        return v;
    }
}
