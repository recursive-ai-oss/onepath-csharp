using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// A view of a <see cref="Grid"/> for one agent size and movement rule: the topology A* searches.
/// Create it with <see cref="Grid.CreateTopology"/>. It reads the grid's arrays, so area edits show
/// through, but create a new view after editing cells (clearance is recomputed then), and do not edit the grid while a search runs.
/// <para>
/// Neighbour order (normative): north (0,-1), east (1,0), south (0,1), west (-1,0), then north-east (1,-1),
/// south-east (1,1), south-west (-1,1), north-west (-1,-1). A cell is walkable for agent size <c>s</c> when its
/// true clearance is at least <c>s</c>. The cost of a move is <c>length * ((m[from] + m[to]) * 0.5)</c>, with length 1
/// for straight and <see cref="GridMath.Sqrt2"/> for diagonal moves and <c>m</c> the area multiplier of the
/// cell's area id.
/// </para>
/// </summary>
public readonly struct SquareTopology : IGridTopology
{
    private readonly byte[] _clearance;
    private readonly byte[] _cells;
    private readonly double[] _multipliers;
    private readonly int _width;
    private readonly int _height;
    private readonly int _size;
    private readonly bool _eight;
    private readonly DiagonalRule _rule;

    internal SquareTopology(Grid grid, int agentSize, Connectivity connectivity, DiagonalRule rule)
    {
        _clearance = grid.ClearanceArray;
        _cells = grid.CellArray;
        _multipliers = grid.Areas.Multipliers;
        _width = grid.Width;
        _height = grid.Height;
        _size = agentSize;
        _eight = connectivity == Connectivity.Eight;
        _rule = rule;
    }

    /// <summary>Grid width in cells.</summary>
    public int Width => _width;

    /// <summary>Grid height in cells.</summary>
    public int Height => _height;

    /// <summary>The agent size in cells this view is for.</summary>
    public int AgentSize => _size;

    /// <inheritdoc/>
    public int NodeCount => _width * _height;

    /// <inheritdoc/>
    public bool IsWalkable(int node) => (uint)node < (uint)_clearance.Length && _clearance[node] >= _size;

    /// <inheritdoc/>
    public void VisitNeighbors<TVisitor>(int node, ref TVisitor visitor)
        where TVisitor : struct, INeighborVisitor
    {
        int w = _width;
        int y = node / w;
        int x = node - (y * w);
        byte[] clear = _clearance;
        int size = _size;
        double mFrom = _multipliers[_cells[node]];

        bool north = y > 0 && clear[node - w] >= size;
        bool east = x + 1 < w && clear[node + 1] >= size;
        bool south = y + 1 < _height && clear[node + w] >= size;
        bool west = x > 0 && clear[node - 1] >= size;

        if (north)
        {
            visitor.Visit(node - w, (mFrom + _multipliers[_cells[node - w]]) * 0.5);
        }
        if (east)
        {
            visitor.Visit(node + 1, (mFrom + _multipliers[_cells[node + 1]]) * 0.5);
        }
        if (south)
        {
            visitor.Visit(node + w, (mFrom + _multipliers[_cells[node + w]]) * 0.5);
        }
        if (west)
        {
            visitor.Visit(node - 1, (mFrom + _multipliers[_cells[node - 1]]) * 0.5);
        }
        if (!_eight)
        {
            return;
        }

        // Diagonal moves. The two cells beside a move are checked per the diagonal rule; the default
        // forbids cutting corners (Sturtevant 2012, benchmarks for grid-based pathfinding).
        if (Allowed(north, east) && y > 0 && x + 1 < w && clear[node - w + 1] >= size)
        {
            visitor.Visit(node - w + 1, GridMath.Sqrt2 * ((mFrom + _multipliers[_cells[node - w + 1]]) * 0.5));
        }
        if (Allowed(south, east) && y + 1 < _height && x + 1 < w && clear[node + w + 1] >= size)
        {
            visitor.Visit(node + w + 1, GridMath.Sqrt2 * ((mFrom + _multipliers[_cells[node + w + 1]]) * 0.5));
        }
        if (Allowed(south, west) && y + 1 < _height && x > 0 && clear[node + w - 1] >= size)
        {
            visitor.Visit(node + w - 1, GridMath.Sqrt2 * ((mFrom + _multipliers[_cells[node + w - 1]]) * 0.5));
        }
        if (Allowed(north, west) && y > 0 && x > 0 && clear[node - w - 1] >= size)
        {
            visitor.Visit(node - w - 1, GridMath.Sqrt2 * ((mFrom + _multipliers[_cells[node - w - 1]]) * 0.5));
        }
    }

    private bool Allowed(bool a, bool b) => _rule switch
    {
        DiagonalRule.NoCornerCutting => a && b,
        DiagonalRule.NoSqueezing => a || b,
        _ => true,
    };
}
