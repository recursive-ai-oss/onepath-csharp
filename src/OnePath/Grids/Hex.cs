using System;

namespace RecursiveAI.OnePath.Grids;

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

/// <summary>
/// A view of a <see cref="HexGrid"/>: the topology A* searches. Create it with <see cref="HexGrid.CreateTopology"/>.
/// Neighbours are visited in axial direction order (1,0), (1,-1), (0,-1), (-1,0), (-1,1), (0,1). Every move has
/// length 1 and costs <c>(m[from] + m[to]) * 0.5</c>.
/// </summary>
public readonly struct HexTopology : IGridTopology
{
    private readonly byte[] _cells;
    private readonly double[] _multipliers;
    private readonly int _width;
    private readonly int _height;
    private readonly HexLayout _layout;

    internal HexTopology(HexGrid grid)
    {
        _cells = grid.CellArray;
        _multipliers = grid.Areas.Multipliers;
        _width = grid.Width;
        _height = grid.Height;
        _layout = grid.Layout;
    }

    /// <summary>Grid width in cells.</summary>
    public int Width => _width;

    /// <summary>Grid height in cells.</summary>
    public int Height => _height;

    /// <summary>The offset layout.</summary>
    public HexLayout Layout => _layout;

    /// <inheritdoc/>
    public int NodeCount => _width * _height;

    /// <inheritdoc/>
    public bool IsWalkable(int node) => (uint)node < (uint)_cells.Length && _cells[node] != AreaCosts.Wall;

    /// <inheritdoc/>
    public void VisitNeighbors<TVisitor>(int node, ref TVisitor visitor)
        where TVisitor : struct, INeighborVisitor
    {
        int row = node / _width;
        int col = node - (row * _width);
        HexCoord here = HexCoord.FromOffset(_layout, col, row);
        double mFrom = _multipliers[_cells[node]];
        ReadOnlySpan<int> dq = HexCoord.DirectionQ;
        ReadOnlySpan<int> dr = HexCoord.DirectionR;
        for (int d = 0; d < 6; d++)
        {
            Cell c = new HexCoord(here.Q + dq[d], here.R + dr[d]).ToOffset(_layout);
            if ((uint)c.X >= (uint)_width || (uint)c.Y >= (uint)_height)
            {
                continue;
            }
            int to = (c.Y * _width) + c.X;
            byte area = _cells[to];
            if (area != AreaCosts.Wall)
            {
                visitor.Visit(to, (mFrom + _multipliers[area]) * 0.5);
            }
        }
    }
}

/// <summary>Hex distance <c>scale * steps</c> on a hex grid; exact and admissible for unit steps.</summary>
public struct HexHeuristic : IHeuristic
{
    private readonly int _width;
    private readonly HexLayout _layout;
    private readonly double _scale;
    private HexCoord _goal;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="width">Grid width in cells; positive.</param>
    /// <param name="layout">The grid's layout.</param>
    /// <param name="scale">Multiplier on the distance, normally <see cref="AreaCosts.MinCost"/>.</param>
    public HexHeuristic(int width, HexLayout layout, double scale = 1.0)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        _width = width;
        _layout = layout;
        _scale = scale;
        _goal = default;
    }

    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
        int row = goal / _width;
        _goal = HexCoord.FromOffset(_layout, goal - (row * _width), row);
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node)
    {
        int row = node / _width;
        return _scale * HexCoord.FromOffset(_layout, node - (row * _width), row).DistanceTo(_goal);
    }
}
