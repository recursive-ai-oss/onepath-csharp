using System;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>
/// A uniform grid of square cells (side 2^Shift grid units) over an integer bounding box, used to find the points and
/// segments near a segment without scanning all of them (the bucket scheme of de Berg et al., "Computational Geometry",
/// ch. 7, applied to snap rounding). Cell membership is exact integer arithmetic, so the cells a segment visits depend
/// only on its endpoints; callers still test candidates with the exact predicates, so the grid only narrows the search.
/// </summary>
internal sealed class CellGrid
{
    public readonly long MinX;
    public readonly long MinY;
    public readonly int Shift;
    public readonly int Cols;
    public readonly int Rows;

    private CellGrid(long minX, long minY, int shift, int cols, int rows)
    {
        MinX = minX;
        MinY = minY;
        Shift = shift;
        Cols = cols;
        Rows = rows;
    }

    public int CellCount => Cols * Rows;

    /// <summary>
    /// A grid over the box whose cells are about <paramref name="typicalSpan"/> wide (at least 2 units) and whose cell
    /// count stays near <paramref name="budget"/> or below.
    /// </summary>
    public static CellGrid Create(long minX, long minY, long maxX, long maxY, long typicalSpan, long budget)
    {
        int shift = 1;
        while (shift < 40 && (1L << shift) < typicalSpan)
        {
            shift++;
        }
        long cap = Math.Max(64, budget);
        while (true)
        {
            long cols = ((maxX - minX) >> shift) + 1;
            long rows = ((maxY - minY) >> shift) + 1;
            // cols * rows can exceed a long for a huge box and a small cell, so compare by division.
            if (cols <= cap / rows || shift >= 60)
            {
                return new CellGrid(minX, minY, shift, (int)cols, (int)rows);
            }
            shift++;
        }
    }

    public int ColOf(long x) => (int)((x - MinX) >> Shift);

    public int RowOf(long y) => (int)((y - MinY) >> Shift);

    /// <summary>
    /// The cells a closed segment passes through, as a row range per column: for column <c>c0 + i</c> the rows
    /// <c>lo[i]..hi[i]</c> (inclusive). A cell the segment only touches at an edge may be included; none it crosses is missed.
    /// </summary>
    public void Traverse(P a, P b, int[] lo, int[] hi, out int c0, out int c1)
    {
        if (a.X > b.X)
        {
            (a, b) = (b, a);
        }
        c0 = ColOf(a.X);
        c1 = ColOf(b.X);
        long dx = b.X - a.X, dy = b.Y - a.Y;
        if (dx == 0)
        {
            int r0 = RowOf(Math.Min(a.Y, b.Y)), r1 = RowOf(Math.Max(a.Y, b.Y));
            lo[0] = r0;
            hi[0] = r1;
            return;
        }
        for (int c = c0; c <= c1; c++)
        {
            long xs = Math.Max(a.X, MinX + ((long)c << Shift));
            long xe = Math.Min(b.X, MinX + (((long)c + 1) << Shift));
            int ra = RowOf(YFloor(a, dx, dy, xs));
            int rb = RowOf(YFloor(a, dx, dy, xe));
            int i = c - c0;
            if (ra <= rb)
            {
                lo[i] = ra;
                hi[i] = rb;
            }
            else
            {
                lo[i] = rb;
                hi[i] = ra;
            }
        }
    }

    // floor of the segment's y at x (a.X <= x <= b.X, dx > 0).
    private static long YFloor(P a, long dx, long dy, long x)
    {
        long t = x - a.X;
        if (dy == 0)
        {
            return a.Y;
        }
        if (dx < (1L << 31) && Math.Abs(dy) < (1L << 31))
        {
            long num = t * dy;
            long q = num / dx;
            if ((num % dx) < 0)
            {
                q--;
            }
            return a.Y + q;
        }
        return a.Y + (long)Geo.FloorDiv((Int128)t * dy, dx);
    }
}
