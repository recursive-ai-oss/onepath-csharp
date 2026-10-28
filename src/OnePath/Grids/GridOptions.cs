using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// Options for the one-line grid queries (<see cref="Grid.FindPath(Cell, Cell, GridOptions?, System.Threading.CancellationToken)"/> and the hex equivalent).
/// Every default is documented here; the static presets cover the usual cases. Costs and filters are plain
/// delegates: nothing here asks for a struct callback (those live in <see cref="AStar{TTopology, TCost, THeuristic}"/>).
/// </summary>
public sealed record GridOptions
{
    /// <summary>The moves allowed on a square grid. Default <see cref="OnePath.Connectivity.Eight"/>. Ignored on hex grids (always six).</summary>
    public Connectivity Connectivity { get; init; } = Connectivity.Eight;

    /// <summary>When diagonal moves are allowed. Default <see cref="DiagonalRule.NoCornerCutting"/>. Ignored on hex grids and four-connected searches.</summary>
    public DiagonalRule Diagonals { get; init; } = DiagonalRule.NoCornerCutting;

    /// <summary>
    /// Agent size in cells (1..255): the agent occupies a square block whose top-left cell is its position
    /// (see docs/grids.md). Default 1. Hex grids support only 1.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 1..255.</exception>
    public int AgentSize { get; init => field = Require.That(value is >= 1 and <= 255, value, nameof(AgentSize), "1 to 255"); } = 1;

    /// <summary>The distance estimate. Default <see cref="GridHeuristic.Auto"/>.</summary>
    public GridHeuristic Heuristic { get; init; } = GridHeuristic.Auto;

    /// <summary>
    /// Multiplier on the estimate; 1 (the default) gives optimal paths. Above 1 searches are faster and paths may
    /// be longer than optimal.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is negative, infinite or NaN.</exception>
    public double HeuristicWeight { get; init => field = Require.That(value >= 0.0 && double.IsFinite(value), value, nameof(HeuristicWeight), "finite and not negative"); } = 1.0;

    /// <summary>
    /// Optional cost adjustment: called with (from, to, grid's own cost) for every move considered; return the cost
    /// to use. Return +∞ or NaN to forbid the move. Returning less than the grid's own cost may make the estimate
    /// inadmissible (paths may then not be optimal). Must be deterministic. Default none.
    /// </summary>
    public Func<Cell, Cell, double, double>? StepCost { get; init; }

    /// <summary>Optional extra blocking: cells for which this returns true cannot be entered (nor used as start or goal). Default none.</summary>
    public Func<Cell, bool>? Blocked { get; init; }

    /// <summary>
    /// The search algorithm. Default <see cref="GridAlgorithm.AStar"/>. <see cref="GridAlgorithm.Jps"/> and <see cref="GridAlgorithm.JpsPlus"/>
    /// give the same optimal cost on the grids they apply to (see <see cref="GridAlgorithm"/> for the conditions; otherwise A* runs
    /// and <see cref="GridPath.Algorithm"/> reports it) but may return a different path of that cost.
    /// </summary>
    public GridAlgorithm Algorithm { get; init; } = GridAlgorithm.AStar;

    /// <summary>Chunk side in cells for <see cref="GridAlgorithm.Hpa"/> (2..1024). Default 16.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 2..1024.</exception>
    public int HpaChunkSize { get; init => field = Require.That(value is >= 2 and <= 1024, value, nameof(HpaChunkSize), "2 to 1024"); } = 16;

    /// <summary>Whether <see cref="GridAlgorithm.Hpa"/> smooths the refined path (see <see cref="HpaSearch"/>). Default true.</summary>
    public bool HpaSmoothing { get; init; } = true;

    /// <summary>
    /// When true the result lists the expanded cells in order (debug data; <see cref="GridPath.ExpandedCells"/> then has <see cref="GridPath.ExpandedCount"/> entries).
    /// For a replanner it lists the nodes each <c>Replan</c> settled or reset. Default false, and then nothing is recorded.
    /// </summary>
    public bool RecordExpansions { get; init; }

    /// <summary>
    /// Whether plain A*, JPS and JPS+ may run on the specialised octile engines when the options allow it (eight moves, no corner
    /// cutting, agent size 1, one cost for every area, no hooks, weight 1, octile or zero estimate). Default true.
    /// <para>
    /// The specialised engine keeps costs as exact counts of straight and diagonal steps and breaks ties on the cost so far rounded to
    /// float32 and then the cell index. The generic engine (<see cref="AStar{TTopology, TCost, THeuristic}"/>, what you get with
    /// <see langword="false"/> and what you build yourself in tier 2) sums double costs and breaks ties on the double cost so far. Both
    /// return optimal paths of the same cost, but among equal-cost paths they may pick different ones. A host that needs the same
    /// path from <c>FindPath</c>, <see cref="Grid.CreateSearch"/> and its own generic <c>AStar</c> sets this to false for all of them.
    /// </para>
    /// </summary>
    public bool SpecializedEngines { get; init; } = true;

    /// <summary>
    /// The farthest ring, in cells (Chebyshev distance from the query cell), <see cref="Grid.TryFindNearestWalkable(Cell, out Cell, GridOptions?)"/>
    /// examines. Default <see cref="int.MaxValue"/>: the whole grid.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is negative.</exception>
    public int NearestRadius { get; init => field = Require.That(value >= 0, value, nameof(NearestRadius), "0 or more"); } = int.MaxValue;

    /// <summary>
    /// Most iterations (node expansions) a query may use before it gives up with <see cref="PathStatus.BudgetExhausted"/>;
    /// counts iterations, never time. Default unlimited.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is below 1.</exception>
    public int IterationBudget { get; init => field = Require.That(value >= 1, value, nameof(IterationBudget), "at least 1"); } = int.MaxValue;

    /// <summary>
    /// Iterations (node expansions) <see cref="Grid.FindPathAsync(Cell, Cell, GridOptions?, System.Threading.CancellationToken)"/> searches between
    /// yields to the caller's context. Counts iterations, never time, and never changes the result. Default 256.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is below 1.</exception>
    public int IterationsPerSlice { get; init => field = Require.That(value >= 1, value, nameof(IterationsPerSlice), "at least 1"); } = 256;

    /// <summary>Eight-connected, no corner cutting, agent size 1: the Moving AI benchmark convention.</summary>
    public static GridOptions Default => new();

    /// <summary>Four-connected moves, agent size 1.</summary>
    public static GridOptions FourConnected => new() { Connectivity = Connectivity.Four };

    /// <summary>Eight-connected with diagonals allowed unless both neighbouring cells are blocked.</summary>
    public static GridOptions NoSqueezing => new() { Diagonals = DiagonalRule.NoSqueezing };

    /// <summary>Default options for an agent of the given size in cells.</summary>
    /// <param name="cells">Agent size, 1..255.</param>
    /// <returns>A new options object.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The size is outside 1..255.</exception>
    public static GridOptions ForAgentSize(int cells) => new() { AgentSize = cells };
}

/// <summary>The result of a one-line grid query. Immutable.</summary>
public sealed partial class GridPath
{
    private static readonly Cell[] NoCells = [];
    private readonly Cell[] _cells;
    private readonly Cell[] _expanded;
    private readonly WorldPoint[] _world;

    internal GridPath(PathStatus status, double cost, Cell[]? cells, int expandedCount, Cell[]? expanded, WorldPoint[]? world)
    {
        Status = status;
        Cost = cost;
        _cells = cells ?? NoCells;
        ExpandedCount = expandedCount;
        _expanded = expanded ?? NoCells;
        _world = world ?? [];
    }

    /// <summary>The algorithm that actually ran (it differs from <see cref="GridOptions.Algorithm"/> when the options did not allow it).</summary>
    public GridAlgorithm Algorithm { get; internal init; }

    /// <summary>How the query ended.</summary>
    public PathStatus Status { get; }

    /// <summary>True when a path to the goal was found.</summary>
    public bool Found => Status == PathStatus.Found;

    /// <summary>True when the result holds a usable path (found, or best effort toward the goal). Grid and graph searches never return a partial path.</summary>
    public bool HasPath => Status.HasPath();

    /// <summary>True for a best-effort path that stops short of the goal; always false here.</summary>
    public bool IsPartial => Status == PathStatus.Partial;

    /// <summary>The path cost (+∞ when no path was found).</summary>
    public double Cost { get; }

    /// <summary>The cells of the path, start first; empty when no path was found.</summary>
    public ReadOnlySpan<Cell> Cells => _cells;

    /// <summary>The world-space cell centres of the path, filled by the world-space query overloads and empty otherwise.</summary>
    public ReadOnlySpan<WorldPoint> Points => _world;

    /// <summary>
    /// Length of the polyline through <see cref="Points"/>, in world units: 0 when there is no path, NaN when a path exists but the query
    /// produced no points (the cell or node overloads).
    /// </summary>
    public double Length
    {
        get
        {
            if (_cells.Length == 0)
            {
                return 0;
            }
            if (_world.Length == 0)
            {
                return double.NaN;
            }
            double l = 0;
            for (int i = 1; i < _world.Length; i++)
            {
                l += Math.Sqrt(((_world[i].X-_world[i-1].X)*(_world[i].X-_world[i-1].X))+((_world[i].Y-_world[i-1].Y)*(_world[i].Y-_world[i-1].Y)));
            }
            return l;
        }
    }

    /// <summary>Number of nodes the search expanded.</summary>
    public int ExpandedCount { get; }

    /// <summary>Debug data: the expanded cells in order; empty unless <see cref="GridOptions.RecordExpansions"/> was set.</summary>
    public ReadOnlySpan<Cell> ExpandedCells => _expanded;
}
