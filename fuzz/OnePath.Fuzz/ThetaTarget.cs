using System;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Tests.Grids;

namespace RecursiveAI.OnePath.Fuzz;

internal static class ThetaTarget
{
    // Any-angle searches against A* on a grid up to 100 x 40 cells decoded from the bytes. Properties: the same reachability as A*;
    // start and goal right; every segment passes the library's visibility test and the independent exact one; cost equal to the
    // segment lengths added in order; Theta* and the re-queued Lazy Theta* never longer than the A* path; line of sight symmetric.
    internal static void Run(byte[] data)
    {
        if (data.Length < 16)
        {
            return;
        }
        int w = 1 + (data[0] % 100);
        int h = 1 + (data[1] % 40);
        int density = 1 + (data[2] % 9);
        var g = new Grid(w, h);
        for (int i = 0; i < w * h; i++)
        {
            byte b = data[3 + (i % (data.Length - 3))];
            b = (byte)(b ^ (i * 31));
            if ((b & 15) < density)
            {
                g.SetWalkable(i % w, i / w, false);
            }
        }
        int pos = 3;
        for (int q = 0; q < 8; q++)
        {
            var a = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
            var b = new Cell(Next(data, ref pos) % w, Next(data, ref pos) % h);
            if (GridLineOfSight.Test(g, a, b) != GridLineOfSight.Test(g, b, a))
            {
                throw new InvalidOperationException($"line of sight {a} -> {b} is not symmetric");
            }
            if (Math.Max(Math.Abs(a.X - b.X), Math.Abs(a.Y - b.Y)) < 40 && GridLineOfSight.Test(g, a, b) != AnyAngleOracle.Visible(g, a, b))
            {
                throw new InvalidOperationException($"line of sight {a} -> {b} differs from the exact test");
            }
            if (!g.IsWalkable(a.X, a.Y) || !g.IsWalkable(b.X, b.Y))
            {
                continue;
            }
            GridPath astar = g.FindPath(a, b);
            foreach (GridAlgorithm alg in new[] { GridAlgorithm.ThetaStar, GridAlgorithm.LazyThetaStar })
            {
                GridPath p = g.FindPath(a, b, new GridOptions { Algorithm = alg });
                if (p.Status != astar.Status)
                {
                    throw new InvalidOperationException($"A* says {astar.Status} but {alg} says {p.Status} for {a} -> {b}");
                }
                if (!p.Found)
                {
                    continue;
                }
                ReadOnlySpan<Cell> c = p.Cells;
                if (c.Length == 0 || c[0] != a || c[^1] != b)
                {
                    throw new InvalidOperationException("path does not run from start to goal");
                }
                double sum = 0;
                for (int k = 1; k < c.Length; k++)
                {
                    if (!GridLineOfSight.Test(g, c[k - 1], c[k]) || (Math.Max(Math.Abs(c[k].X - c[k - 1].X), Math.Abs(c[k].Y - c[k - 1].Y)) < 40 && !AnyAngleOracle.Visible(g, c[k - 1], c[k])))
                    {
                        throw new InvalidOperationException($"{alg}: no line of sight {c[k - 1]} -> {c[k]}");
                    }
                    double dx = c[k].X - c[k - 1].X;
                    double dy = c[k].Y - c[k - 1].Y;
                    sum += Math.Sqrt((dx * dx) + (dy * dy));
                }
                if (sum != p.Cost)
                {
                    throw new InvalidOperationException($"{alg}: cost {p.Cost} but the segments add up to {sum}");
                }
                if (p.Cost > astar.Cost * (1 + 1e-12))
                {
                    throw new InvalidOperationException($"{alg}: cost {p.Cost} exceeds the A* cost {astar.Cost} for {a} -> {b}");
                }
            }
        }
    }

    private static int Next(byte[] data, ref int pos)
    {
        int v = data[pos % data.Length];
        pos++;
        return v;
    }
}
