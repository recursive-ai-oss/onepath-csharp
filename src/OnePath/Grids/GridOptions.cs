using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// Options for the one-line grid queries (<see cref="Grid.FindPath(Cell, Cell, GridOptions?)"/> and the hex equivalent).
/// Every default is documented here; the static presets cover the usual cases. Costs and filters are plain
/// delegates: nothing here asks for a struct callback (those live in <see cref="AStar{TTopology, TCost, THeuristic}"/>).
/// </summary>
public sealed class GridOptions
{
    private int _agentSize = 1;
    private double _heuristicWeight = 1.0;
    private int _iterationBudget = int.MaxValue;
    private int _hpaChunkSize = 16;

    /// <summary>The moves allowed on a square grid. Default <see cref="OnePath.Connectivity.Eight"/>. Ignored on hex grids (always six).</summary>
    public Connectivity Connectivity { get; set; } = Connectivity.Eight;

    /// <summary>When diagonal moves are allowed. Default <see cref="DiagonalRule.NoCornerCutting"/>. Ignored on hex grids and four-connected searches.</summary>
    public DiagonalRule Diagonals { get; set; } = DiagonalRule.NoCornerCutting;

    /// <summary>
    /// Agent size in cells (1..255): the agent occupies a square block whose top-left cell is its position
    /// (see docs/grids.md). Default 1. Hex grids support only 1.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 1..255.</exception>
    public int AgentSize
    {
        get => _agentSize;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 1);
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value, 255);
            _agentSize = value;
        }
    }

    /// <summary>The distance estimate. Default <see cref="GridHeuristic.Auto"/>.</summary>
    public GridHeuristic Heuristic { get; set; } = GridHeuristic.Auto;

    /// <summary>
    /// Multiplier on the estimate; 1 (the default) gives optimal paths. Above 1 searches are faster and paths may
    /// be longer than optimal.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is negative, infinite or NaN.</exception>
    public double HeuristicWeight
    {
        get => _heuristicWeight;
        set
        {
            if (!(value >= 0.0) || double.IsPositiveInfinity(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "The heuristic weight must be finite and not negative.");
            }
            _heuristicWeight = value;
        }
    }

    /// <summary>
    /// Optional cost adjustment: called with (from, to, grid's own cost) for every move considered; return the cost
    /// to use. Return +∞ or NaN to forbid the move. Returning less than the grid's own cost may make the estimate
    /// inadmissible (paths may then not be optimal). Must be deterministic. Default none.
    /// </summary>
    public Func<Cell, Cell, double, double>? StepCost { get; set; }

    /// <summary>Optional extra blocking: cells for which this returns true cannot be entered (nor used as start or goal). Default none.</summary>
    public Func<Cell, bool>? Blocked { get; set; }

    /// <summary>
    /// The search algorithm. Default <see cref="GridAlgorithm.AStar"/>. <see cref="GridAlgorithm.Jps"/> and <see cref="GridAlgorithm.JpsPlus"/>
    /// give the same optimal cost on the grids they apply to (see <see cref="GridAlgorithm"/> for the conditions; otherwise A* runs
    /// and <see cref="GridPath.Algorithm"/> reports it) but may return a different path of that cost.
    /// </summary>
    public GridAlgorithm Algorithm { get; set; } = GridAlgorithm.AStar;

    /// <summary>Chunk side in cells for <see cref="GridAlgorithm.Hpa"/> (2..1024). Default 16.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 2..1024.</exception>
    public int HpaChunkSize
    {
        get => _hpaChunkSize;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 2);
            ArgumentOutOfRangeException.ThrowIfGreaterThan(value, 1024);
            _hpaChunkSize = value;
        }
    }

    /// <summary>Whether <see cref="GridAlgorithm.Hpa"/> smooths the refined path (see <see cref="HpaSearch"/>). Default true.</summary>
    public bool HpaSmoothing { get; set; } = true;

    /// <summary>
    /// When true the result lists the expanded cells in order (debug data; <see cref="GridPath.ExpandedCells"/> then has <see cref="GridPath.ExpandedCount"/> entries).
    /// For a replanner it lists the nodes each <c>Replan</c> settled or reset. Default false, and then nothing is recorded.
    /// </summary>
    public bool RecordExpansions { get; set; }

    /// <summary>
    /// Most iterations (node expansions) a query may use before it gives up with <see cref="PathOutcome.BudgetExhausted"/>;
    /// counts iterations, never time. Default unlimited.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is below 1.</exception>
    public int IterationBudget
    {
        get => _iterationBudget;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 1);
            _iterationBudget = value;
        }
    }

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

    /// <summary>A copy of these options.</summary>
    /// <returns>A new, independent options object.</returns>
    public GridOptions Clone() => (GridOptions)MemberwiseClone();
}

/// <summary>The result of a one-line grid query. Immutable.</summary>
public sealed class GridPath
{
    private static readonly Cell[] NoCells = [];
    private readonly Cell[] _cells;
    private readonly Cell[] _expanded;
    private readonly WorldPoint[] _world;

    internal GridPath(PathOutcome status, double cost, Cell[]? cells, int expandedCount, Cell[]? expanded, WorldPoint[]? world)
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
    public PathOutcome Status { get; }

    /// <summary>True when a path was found.</summary>
    public bool Found => Status == PathOutcome.Found;

    /// <summary>The path cost (+∞ when no path was found).</summary>
    public double Cost { get; }

    /// <summary>The cells of the path, start first; empty when no path was found.</summary>
    public ReadOnlySpan<Cell> Cells => _cells;

    /// <summary>The world-space cell centres of the path, filled by the world-space query overloads and empty otherwise.</summary>
    public ReadOnlySpan<WorldPoint> WorldPoints => _world;

    /// <summary>Number of nodes the search expanded.</summary>
    public int ExpandedCount { get; }

    /// <summary>Debug data: the expanded cells in order; empty unless <see cref="GridOptions.RecordExpansions"/> was set.</summary>
    public ReadOnlySpan<Cell> ExpandedCells => _expanded;
}
