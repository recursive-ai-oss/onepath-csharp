using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>
/// Reference answers for any-angle search, written from the definitions and sharing nothing with the production walk:
/// a segment-versus-closed-square test in exact integer arithmetic (separating axes), and the shortest path over the
/// complete visibility graph of cell centres.
/// </summary>
internal static class AnyAngleOracle
{
    /// <summary>True if the segment between the two cell centres touches no wall (closed unit squares), by testing every cell of the bounding box.</summary>
    internal static bool Visible(Grid g, Cell a, Cell b)
    {
        if (g.GetArea(a.X, a.Y) == AreaCosts.Wall || g.GetArea(b.X, b.Y) == AreaCosts.Wall)
        {
            return false;
        }
        int x0 = Math.Min(a.X, b.X);
        int x1 = Math.Max(a.X, b.X);
        int y0 = Math.Min(a.Y, b.Y);
        int y1 = Math.Max(a.Y, b.Y);
        for (int y = y0; y <= y1; y++)
        {
            for (int x = x0; x <= x1; x++)
            {
                if (g.GetArea(x, y) == AreaCosts.Wall && Touches(a, b, x, y))
                {
                    return false;
                }
            }
        }
        return true;
    }

    // Doubled coordinates: centres at (2x+1, 2y+1), the cell is the closed square [2cx, 2cx+2] x [2cy, 2cy+2].
    // A segment and a convex box are disjoint iff an axis separates them: x, y, or the segment's normal.
    private static bool Touches(Cell a, Cell b, int cx, int cy)
    {
        long px = (2L * a.X) + 1;
        long py = (2L * a.Y) + 1;
        long qx = (2L * b.X) + 1;
        long qy = (2L * b.Y) + 1;
        long bx0 = 2L * cx;
        long by0 = 2L * cy;
        long bx1 = bx0 + 2;
        long by1 = by0 + 2;
        if (Math.Max(px, qx) < bx0 || Math.Min(px, qx) > bx1 || Math.Max(py, qy) < by0 || Math.Min(py, qy) > by1)
        {
            return false;
        }
        long ex = qx - px;
        long ey = qy - py;
        int positive = 0;
        int negative = 0;
        foreach ((long x, long y) in new[] { (bx0, by0), (bx1, by0), (bx0, by1), (bx1, by1) })
        {
            long cross = (ex * (y - py)) - (ey * (x - px));
            if (cross > 0)
            {
                positive++;
            }
            else if (cross < 0)
            {
                negative++;
            }
        }
        return !(positive == 4 || negative == 4);
    }

    /// <summary>Length of the shortest path from start to goal that turns only at cell centres and whose every segment has line of sight; +∞ if none.</summary>
    internal static double VisibilityGraphDistance(Grid g, Cell start, Cell goal, double unit = 1.0)
    {
        var cells = new List<Cell>();
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                if (g.GetArea(x, y) != AreaCosts.Wall)
                {
                    cells.Add(new Cell(x, y));
                }
            }
        }
        int n = cells.Count;
        int s = cells.IndexOf(start);
        int t = cells.IndexOf(goal);
        var dist = new double[n];
        var done = new bool[n];
        Array.Fill(dist, double.PositiveInfinity);
        dist[s] = 0;
        for (int iter = 0; iter < n; iter++)
        {
            int u = -1;
            for (int i = 0; i < n; i++)
            {
                if (!done[i] && (u < 0 || dist[i] < dist[u]))
                {
                    u = i;
                }
            }
            if (u < 0 || double.IsPositiveInfinity(dist[u]) || u == t)
            {
                break;
            }
            done[u] = true;
            for (int v = 0; v < n; v++)
            {
                if (done[v])
                {
                    continue;
                }
                double dx = cells[v].X - cells[u].X;
                double dy = cells[v].Y - cells[u].Y;
                double d = dist[u] + (unit * Math.Sqrt((dx * dx) + (dy * dy)));
                if (d < dist[v] && Visible(g, cells[u], cells[v]))
                {
                    dist[v] = d;
                }
            }
        }
        return dist[t];
    }
}
