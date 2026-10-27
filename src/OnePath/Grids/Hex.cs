using System;

namespace RecursiveAI.OnePath.Grids;

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
