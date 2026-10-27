using System;

namespace RecursiveAI.OnePath;

/// <summary>
/// How a rectangular array of hexagons is laid out (offset conventions). Pointy layouts have hexes with a
/// vertex on top and shift alternate rows; flat layouts have a flat top and shift alternate columns. The odd/even
/// name says which rows or columns are shifted (right or down) by half a hex relative to the others.
/// </summary>
public enum HexLayout
{
    /// <summary>Pointy-topped; odd rows are shifted right.</summary>
    PointyOddRow = 0,

    /// <summary>Pointy-topped; even rows are shifted right.</summary>
    PointyEvenRow = 1,

    /// <summary>Flat-topped; odd columns are shifted down.</summary>
    FlatOddColumn = 2,

    /// <summary>Flat-topped; even columns are shifted down.</summary>
    FlatEvenColumn = 3,
}

/// <summary>A hex position in axial coordinates (q, r); the cube coordinates are (q, r, -q-r).</summary>
/// <param name="Q">The q axis.</param>
/// <param name="R">The r axis.</param>
public readonly record struct HexCoord(int Q, int R)
{
    /// <summary>Hex distance: the number of steps between two hexes.</summary>
    /// <param name="other">The other hex.</param>
    /// <returns>The step count.</returns>
    public int DistanceTo(HexCoord other)
    {
        int dq = Q - other.Q;
        int dr = R - other.R;
        int ds = -dq - dr;
        return (Math.Abs(dq) + Math.Abs(dr) + Math.Abs(ds)) / 2;
    }

    /// <summary>The axial coordinates of the cell (<paramref name="col"/>, <paramref name="row"/>) in an offset layout.</summary>
    /// <param name="layout">The layout.</param>
    /// <param name="col">Column of the offset array.</param>
    /// <param name="row">Row of the offset array.</param>
    /// <returns>The axial coordinates.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The layout is not a defined value.</exception>
    public static HexCoord FromOffset(HexLayout layout, int col, int row)
    {
        switch (layout)
        {
            case HexLayout.PointyOddRow:
                return new HexCoord(col - ((row - (row & 1)) / 2), row);
            case HexLayout.PointyEvenRow:
                return new HexCoord(col - ((row + (row & 1)) / 2), row);
            case HexLayout.FlatOddColumn:
                return new HexCoord(col, row - ((col - (col & 1)) / 2));
            case HexLayout.FlatEvenColumn:
                return new HexCoord(col, row - ((col + (col & 1)) / 2));
            default:
                throw new ArgumentOutOfRangeException(nameof(layout), layout, "Unknown hex layout.");
        }
    }

    /// <summary>The offset-array cell of this hex in the given layout.</summary>
    /// <param name="layout">The layout.</param>
    /// <returns>The cell (column, row); it may lie outside any particular grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The layout is not a defined value.</exception>
    public Cell ToOffset(HexLayout layout)
    {
        switch (layout)
        {
            case HexLayout.PointyOddRow:
                return new Cell(Q + ((R - (R & 1)) / 2), R);
            case HexLayout.PointyEvenRow:
                return new Cell(Q + ((R + (R & 1)) / 2), R);
            case HexLayout.FlatOddColumn:
                return new Cell(Q, R + ((Q - (Q & 1)) / 2));
            case HexLayout.FlatEvenColumn:
                return new Cell(Q, R + ((Q + (Q & 1)) / 2));
            default:
                throw new ArgumentOutOfRangeException(nameof(layout), layout, "Unknown hex layout.");
        }
    }

    // Axial direction vectors, in the order neighbours are visited: (1,0), (1,-1), (0,-1), (-1,0), (-1,1), (0,1).
    internal static ReadOnlySpan<int> DirectionQ => [1, 1, 0, -1, -1, 0];

    internal static ReadOnlySpan<int> DirectionR => [0, -1, -1, 0, 1, 1];
}
