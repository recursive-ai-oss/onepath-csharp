namespace RecursiveAI.OnePath.Grids;

/// <summary>Receives the moves out of one node. Implemented by struct visitors so the calls inline.</summary>
public interface INeighborVisitor
{
    /// <summary>Called once per legal move out of the node being expanded, in the topology's documented order.</summary>
    /// <param name="neighbor">Index of the node moved to.</param>
    /// <param name="baseCost">Cost of the move from the grid's own rules (step length times the area multiplier).</param>
    void Visit(int neighbor, double baseCost);
}

/// <summary>
/// A graph over grid cells: node count and neighbour enumeration. Nodes are cell indices (row-major).
/// Implement this to search a grid-like structure with <see cref="AStar{TTopology, TCost, THeuristic}"/>.
/// </summary>
public interface IGridTopology
{
    /// <summary>Number of nodes; valid node indices are 0..NodeCount-1.</summary>
    int NodeCount { get; }

    /// <summary>True if an agent may stand on <paramref name="node"/> (used to validate start and goal).</summary>
    bool IsWalkable(int node);

    /// <summary>Calls <paramref name="visitor"/> for every legal move out of <paramref name="node"/>, in a fixed order.</summary>
    /// <typeparam name="TVisitor">The visitor type; a struct so the call inlines.</typeparam>
    /// <param name="node">The node being expanded.</param>
    /// <param name="visitor">The visitor.</param>
    void VisitNeighbors<TVisitor>(int node, ref TVisitor visitor)
        where TVisitor : struct, INeighborVisitor;
}

/// <summary>Adjusts the cost of a move. The hook for custom costs and extra blocking.</summary>
public interface IStepCost
{
    /// <summary>The final cost of moving from <paramref name="source"/> to <paramref name="target"/>.</summary>
    /// <param name="source">Node being expanded.</param>
    /// <param name="target">Neighbour.</param>
    /// <param name="baseCost">The grid's own cost for the move.</param>
    /// <returns>A cost of zero or more; +∞ or NaN forbids the move. Costs below the grid's own cost can make the heuristic inadmissible.</returns>
    double Cost(int source, int target, double baseCost);
}

/// <summary>A distance estimate towards the goal of the current search.</summary>
public interface IHeuristic
{
    /// <summary>Called once at the start of every search with the goal node.</summary>
    /// <param name="goal">The goal node.</param>
    void SetGoal(int goal);

    /// <summary>The estimated cost from <paramref name="node"/> to the goal. Admissible when it never exceeds the true cost.</summary>
    /// <param name="node">A node index.</param>
    double Estimate(int node);
}

/// <summary>The grid's own costs, unchanged.</summary>
public readonly struct DefaultStepCost : IStepCost
{
    /// <inheritdoc/>
    public double Cost(int source, int target, double baseCost) => baseCost;
}

/// <summary>No estimate: the search is Dijkstra's algorithm.</summary>
public struct ZeroHeuristic : IHeuristic
{
    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node) => 0.0;
}

/// <summary>
/// Manhattan distance <c>scale * (|dx| + |dy|)</c> on a square grid of the given width. Admissible for
/// four-connected movement when <c>scale</c> is at most the smallest area multiplier.
/// </summary>
public struct ManhattanHeuristic : IHeuristic
{
    private readonly int _width;
    private readonly double _scale;
    private int _gx;
    private int _gy;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="width">Grid width in cells; positive.</param>
    /// <param name="scale">Multiplier applied to the distance, normally <see cref="AreaCosts.MinCost"/>.</param>
    public ManhattanHeuristic(int width, double scale = 1.0)
    {
        System.ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        _width = width;
        _scale = scale;
        _gx = _gy = 0;
    }

    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
        _gy = goal / _width;
        _gx = goal - (_gy * _width);
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node)
    {
        int y = node / _width;
        int x = node - (y * _width);
        int dx = x > _gx ? x - _gx : _gx - x;
        int dy = y > _gy ? y - _gy : _gy - y;
        return _scale * (dx + dy);
    }
}

/// <summary>
/// Octile distance <c>scale * ((max - min) + min * √2)</c> on a square grid of the given width: the exact
/// shortest cost on an empty eight-connected grid with unit straight steps and √2 diagonals.
/// </summary>
public struct OctileHeuristic : IHeuristic
{
    private readonly int _width;
    private readonly double _scale;
    private int _gx;
    private int _gy;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="width">Grid width in cells; positive.</param>
    /// <param name="scale">Multiplier applied to the distance, normally <see cref="AreaCosts.MinCost"/>.</param>
    public OctileHeuristic(int width, double scale = 1.0)
    {
        System.ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        _width = width;
        _scale = scale;
        _gx = _gy = 0;
    }

    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
        _gy = goal / _width;
        _gx = goal - (_gy * _width);
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node)
    {
        int y = node / _width;
        int x = node - (y * _width);
        int dx = x > _gx ? x - _gx : _gx - x;
        int dy = y > _gy ? y - _gy : _gy - y;
        int min = dx < dy ? dx : dy;
        int max = dx < dy ? dy : dx;
        return _scale * ((max - min) + (min * GridMath.Sqrt2));
    }
}

/// <summary>Constants shared by the grid code.</summary>
public static class GridMath
{
    /// <summary>
    /// The length of a diagonal step: √2 as the double nearest to the true value (0x3FF6A09E667F3BCD,
    /// 1.4142135623730951). IEEE-754 square root is correctly rounded, so <c>Math.Sqrt(2.0)</c> yields the same
    /// bits; the literal avoids relying on it. Costs are sums of this constant and 1, so they are exact
    /// up to ordinary double rounding and identical on every platform.
    /// </summary>
    public const double Sqrt2 = 1.4142135623730951;

    /// <summary>√3 / 2 as the nearest double (0.8660254037844386), the row spacing of a hex grid in units of the centre spacing.</summary>
    public const double HalfSqrt3 = 0.8660254037844386;
}
