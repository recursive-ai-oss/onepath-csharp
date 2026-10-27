using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Exact line-of-sight between cell centres on a square grid (tier 3; the visibility test of Theta* and Lazy Theta*).
/// <para>
/// Two cells see each other when the straight segment between their centres touches no wall cell, where a wall is the
/// closed unit square: a segment that only grazes a wall's corner is blocked. That makes visibility between diagonal
/// neighbours exactly the no-corner-cutting rule, so every legal grid move is a line of sight. The test is integer
/// arithmetic only (a supercover walk, Amanatides &amp; Woo style, with the corner case decided by an exact integer
/// comparison), so it answers identically on every platform. Cells outside the grid count as walls.
/// </para>
/// </summary>
public static class GridLineOfSight
{
    /// <summary>Whether the segment between the centres of two cells touches only walkable cells.</summary>
    /// <param name="grid">The grid.</param>
    /// <param name="a">First cell.</param>
    /// <param name="b">Second cell.</param>
    /// <returns>True if both cells are walkable and nothing blocks the segment between their centres. Symmetric in <paramref name="a"/> and <paramref name="b"/>.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A cell lies outside the grid.</exception>
    public static bool Test(Grid grid, Cell a, Cell b)
    {
        ArgumentNullException.ThrowIfNull(grid);
        if (!grid.Contains(a.X, a.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(a), a, $"Cell {a} is outside the {grid.Width}x{grid.Height} grid.");
        }
        if (!grid.Contains(b.X, b.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(b), b, $"Cell {b} is outside the {grid.Width}x{grid.Height} grid.");
        }
        return Visible(grid.Cells, grid.Width, a.X, a.Y, b.X, b.Y);
    }

    // The walk visits, in order from (x0, y0), every cell the closed segment between the two centres touches.
    // With nx = |dx| and ny = |dy|, after ix vertical and iy horizontal crossings the next vertical edge is met at
    // parameter (ix + 1/2)/nx and the next horizontal one at (iy + 1/2)/ny; comparing the two fractions by cross-multiplying
    // keeps it exact. Equal means the segment passes through a lattice corner: the two cells beside it are touched too.
    // Both endpoints must lie inside the grid (callers guarantee it), so every visited cell does as well.
    internal static bool Visible(ReadOnlySpan<byte> cells, int width, int x0, int y0, int x1, int y1)
    {
        int dx = x1 - x0;
        int dy = y1 - y0;
        int sx = dx < 0 ? -1 : 1;
        int sy = dy < 0 ? -1 : 1;
        long nx = dx < 0 ? -(long)dx : dx;
        long ny = dy < 0 ? -(long)dy : dy;
        int x = x0;
        int y = y0;
        if (cells[(y * width) + x] == AreaCosts.Wall)
        {
            return false;
        }
        if (nx == 0 || ny == 0)
        {
            // Along a row or column the centre line stays inside the cells of that line.
            int step = nx == 0 ? sy * width : sx;
            int i = (y * width) + x;
            for (long k = nx + ny; k > 0; k--)
            {
                i += step;
                if (cells[i] == AreaCosts.Wall)
                {
                    return false;
                }
            }
            return true;
        }
        long ix = 0;
        long iy = 0;
        while (ix < nx || iy < ny)
        {
            long decision = (((2 * ix) + 1) * ny) - (((2 * iy) + 1) * nx);
            if (decision == 0)
            {
                if (cells[(y * width) + x + sx] == AreaCosts.Wall || cells[((y + sy) * width) + x] == AreaCosts.Wall)
                {
                    return false;
                }
                x += sx;
                y += sy;
                ix++;
                iy++;
            }
            else if (decision < 0)
            {
                x += sx;
                ix++;
            }
            else
            {
                y += sy;
                iy++;
            }
            if (cells[(y * width) + x] == AreaCosts.Wall)
            {
                return false;
            }
        }
        return true;
    }
}
