using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>
/// Independent reference implementations written straight from the definitions (naive block checks,
/// O(n^2) Dijkstra) so the production code is checked against something that shares none of its tricks.
/// </summary>
internal static class BruteForce
{
    internal static bool BlockFree(Grid g, int x, int y, int size)
    {
        if (x < 0 || y < 0 || x + size > g.Width || y + size > g.Height)
        {
            return false;
        }
        for (int j = 0; j < size; j++)
        {
            for (int i = 0; i < size; i++)
            {
                if (g.GetArea(x + i, y + j) == AreaCosts.Wall)
                {
                    return false;
                }
            }
        }
        return true;
    }

    internal static bool LegalMove(Grid g, int size, Connectivity conn, DiagonalRule rule, Cell a, Cell b)
    {
        int dx = b.X - a.X;
        int dy = b.Y - a.Y;
        if (Math.Abs(dx) > 1 || Math.Abs(dy) > 1 || (dx == 0 && dy == 0))
        {
            return false;
        }
        if (!BlockFree(g, a.X, a.Y, size) || !BlockFree(g, b.X, b.Y, size))
        {
            return false;
        }
        if (dx == 0 || dy == 0)
        {
            return true;
        }
        if (conn == Connectivity.Four)
        {
            return false;
        }
        bool p = BlockFree(g, a.X + dx, a.Y, size);
        bool q = BlockFree(g, a.X, a.Y + dy, size);
        return rule switch
        {
            DiagonalRule.NoCornerCutting => p && q,
            DiagonalRule.NoSqueezing => p || q,
            _ => true,
        };
    }

    internal static double MoveCost(Grid g, Cell a, Cell b)
    {
        bool diag = a.X != b.X && a.Y != b.Y;
        double len = diag ? Math.Sqrt(2.0) : 1.0;
        double ma = g.Areas[g.GetArea(a.X, a.Y)];
        double mb = g.Areas[g.GetArea(b.X, b.Y)];
        return len * ((ma + mb) * 0.5);
    }

    /// <summary>Least cost by O(n^2) Dijkstra; +inf if unreachable.</summary>
    internal static double Dijkstra(Grid g, int size, Connectivity conn, DiagonalRule rule, Cell start, Cell goal)
    {
        int n = g.Width * g.Height;
        var dist = new double[n];
        var done = new bool[n];
        Array.Fill(dist, double.PositiveInfinity);
        dist[(start.Y * g.Width) + start.X] = 0;
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
            if (u < 0 || double.IsPositiveInfinity(dist[u]))
            {
                break;
            }
            done[u] = true;
            var a = new Cell(u % g.Width, u / g.Width);
            for (int dy = -1; dy <= 1; dy++)
            {
                for (int dx = -1; dx <= 1; dx++)
                {
                    var b = new Cell(a.X + dx, a.Y + dy);
                    if (!g.Contains(b.X, b.Y) || !LegalMove(g, size, conn, rule, a, b))
                    {
                        continue;
                    }
                    int v = (b.Y * g.Width) + b.X;
                    double nd = dist[u] + MoveCost(g, a, b);
                    if (nd < dist[v])
                    {
                        dist[v] = nd;
                    }
                }
            }
        }
        return dist[(goal.Y * g.Width) + goal.X];
    }

    /// <summary>Least cost on a hex grid; neighbours are found geometrically (centres exactly one spacing apart).</summary>
    internal static double HexDijkstra(HexGrid g, Cell start, Cell goal)
    {
        int n = g.Width * g.Height;
        var centres = new WorldPoint[n];
        for (int i = 0; i < n; i++)
        {
            centres[i] = g.CellCenter(new Cell(i % g.Width, i / g.Width));
        }
        double size = g.Space.CellSize;
        var dist = new double[n];
        var done = new bool[n];
        Array.Fill(dist, double.PositiveInfinity);
        dist[(start.Y * g.Width) + start.X] = 0;
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
            if (u < 0 || double.IsPositiveInfinity(dist[u]))
            {
                break;
            }
            done[u] = true;
            for (int v = 0; v < n; v++)
            {
                if (v == u)
                {
                    continue;
                }
                double ddx = centres[u].X - centres[v].X;
                double ddy = centres[u].Y - centres[v].Y;
                if (Math.Abs(Math.Sqrt((ddx * ddx) + (ddy * ddy)) - size) > 1e-9)
                {
                    continue;
                }
                byte au = g.GetArea(u % g.Width, u / g.Width);
                byte av = g.GetArea(v % g.Width, v / g.Width);
                if (au == AreaCosts.Wall || av == AreaCosts.Wall)
                {
                    continue;
                }
                double nd = dist[u] + ((g.Areas[au] + g.Areas[av]) * 0.5);
                if (nd < dist[v])
                {
                    dist[v] = nd;
                }
            }
        }
        return dist[(goal.Y * g.Width) + goal.X];
    }

    internal static Grid RandomGrid(ref SeededRandom rng, int maxDim, bool areas)
    {
        int w = 2 + rng.NextInt(maxDim - 1);
        int h = 2 + rng.NextInt(maxDim - 1);
        var g = new Grid(w, h);
        double density = rng.NextDouble() * 0.45;
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (rng.NextDouble() < density)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (areas)
                {
                    g.SetArea(x, y, (byte)rng.NextInt(4));
                }
            }
        }
        if (areas)
        {
            g.Areas[0] = 1.0;
            g.Areas[1] = 0.5;
            g.Areas[2] = 3.0;
            g.Areas[3] = 7.25;
        }
        return g;
    }

    internal static List<Cell> WalkableStarts(Grid g, int size)
    {
        var l = new List<Cell>();
        for (int y = 0; y < g.Height; y++)
        {
            for (int x = 0; x < g.Width; x++)
            {
                if (BlockFree(g, x, y, size))
                {
                    l.Add(new Cell(x, y));
                }
            }
        }
        return l;
    }
}
