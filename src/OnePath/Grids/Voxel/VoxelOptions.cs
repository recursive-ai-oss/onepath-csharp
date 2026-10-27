using System;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath;

/// <summary>Which neighbour moves a voxel search may make.</summary>
public enum VoxelConnectivity
{
    /// <summary>The six face neighbours (cost 1).</summary>
    Six = 6,

    /// <summary>Faces plus the twelve edge neighbours (cost √2).</summary>
    Eighteen = 18,

    /// <summary>Faces, edges and the eight corner neighbours (cost √3). The Moving AI voxel benchmark's model.</summary>
    TwentySix = 26,
}

/// <summary>
/// When an edge or corner move is allowed. The cells a move sweeps past are the ones that differ from the start by
/// part of the move: for an edge move the two face neighbours it passes between; for a corner move the three face
/// and three edge neighbours inside its cube.
/// </summary>
public enum VoxelCornerRule
{
    /// <summary>Every swept cell must be open. The default and the Moving AI benchmark convention.</summary>
    NoCornerCutting = 0,

    /// <summary>
    /// The move may squeeze past solid cells only when a detour of face steps inside its cube exists: an edge move needs
    /// one of its two swept faces open; a corner move needs a face-connected monotone route through open cells.
    /// </summary>
    NoSqueezing = 1,

    /// <summary>Only the destination matters; the move may cut through the edge or corner between solid cells.</summary>
    AllowAll = 2,
}

/// <summary>Options of a one-line voxel query. Plain values and delegates, no struct callbacks.</summary>
public sealed class VoxelOptions
{
    private double _heuristicWeight = 1.0;
    private int _iterationBudget = int.MaxValue;

    /// <summary>The moves allowed. Default 26 (<see cref="VoxelConnectivity.TwentySix"/>).</summary>
    public VoxelConnectivity Connectivity { get; set; } = VoxelConnectivity.TwentySix;

    /// <summary>Corner rule. Default <see cref="VoxelCornerRule.NoCornerCutting"/>.</summary>
    public VoxelCornerRule Corners { get; set; } = VoxelCornerRule.NoCornerCutting;

    /// <summary>
    /// The distance estimate. Default <see cref="GridHeuristic.Auto"/>: the exact empty-grid distance of the connectivity
    /// (Manhattan for six moves, the 3D octile distances for 18 and 26), scaled by the smallest area multiplier.
    /// <see cref="GridHeuristic.Octile"/> picks the same estimate explicitly, <see cref="GridHeuristic.Zero"/> gives Dijkstra,
    /// and <see cref="GridHeuristic.Manhattan"/> overestimates for 18 and 26 moves (the paths are then not always optimal).
    /// </summary>
    public GridHeuristic Heuristic { get; set; } = GridHeuristic.Auto;

    /// <summary>
    /// Multiplier on the estimate, 1 by default. Above 1 the search is faster and no longer optimal. Must be finite and not negative.
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
    /// Optional cost adjustment: called with (from, to, grid's own cost) for every move considered; return the cost to use.
    /// Return +∞ or NaN to forbid the move. Returning less than the grid's own cost may make the estimate inadmissible.
    /// Must be deterministic. Default none.
    /// </summary>
    public Func<VoxelCell, VoxelCell, double, double>? StepCost { get; set; }

    /// <summary>Optional extra blocking: voxels for which this returns true cannot be entered (nor used as start or goal). Default none.</summary>
    public Func<VoxelCell, bool>? Blocked { get; set; }

    /// <summary>When true the result lists the expanded voxels in order (debug data). Default false.</summary>
    public bool RecordExpansions { get; set; }

    /// <summary>
    /// Most iterations (node expansions) a query may use before it gives up with <see cref="PathOutcome.BudgetExhausted"/>;
    /// counts iterations, never time. Default unlimited. For a resumable search use <see cref="AStar{TTopology, TCost, THeuristic}"/>.
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

    /// <summary>26-connected, no corner cutting: the Moving AI voxel benchmark convention.</summary>
    public static VoxelOptions Default => new();

    /// <summary>Face moves only.</summary>
    public static VoxelOptions SixConnected => new() { Connectivity = VoxelConnectivity.Six };

    /// <summary>Face and edge moves.</summary>
    public static VoxelOptions EighteenConnected => new() { Connectivity = VoxelConnectivity.Eighteen };

    /// <summary>A copy of these options.</summary>
    /// <returns>A new, independent options object.</returns>
    public VoxelOptions Clone() => (VoxelOptions)MemberwiseClone();
}

/// <summary>The result of a one-line voxel query. Immutable.</summary>
public sealed class VoxelPath
{
    private static readonly VoxelCell[] NoCells = [];
    private readonly VoxelCell[] _cells;
    private readonly VoxelCell[] _expanded;
    private readonly VoxelPoint[] _world;

    internal VoxelPath(PathOutcome status, double cost, VoxelCell[]? cells, int expandedCount, VoxelCell[]? expanded, VoxelPoint[]? world)
    {
        Status = status;
        Cost = cost;
        _cells = cells ?? NoCells;
        ExpandedCount = expandedCount;
        _expanded = expanded ?? NoCells;
        _world = world ?? [];
    }

    /// <summary>How the query ended.</summary>
    public PathOutcome Status { get; }

    /// <summary>True when a path was found.</summary>
    public bool Found => Status == PathOutcome.Found;

    /// <summary>The path cost (+∞ when no path was found).</summary>
    public double Cost { get; }

    /// <summary>The voxels of the path, start first; empty when no path was found.</summary>
    public ReadOnlySpan<VoxelCell> Cells => _cells;

    /// <summary>The world-space voxel centres of the path, filled by the world-space query overload and empty otherwise.</summary>
    public ReadOnlySpan<VoxelPoint> WorldPoints => _world;

    /// <summary>Number of nodes the search expanded.</summary>
    public int ExpandedCount { get; }

    /// <summary>Debug data: the expanded voxels in order; empty unless <see cref="VoxelOptions.RecordExpansions"/> was set.</summary>
    public ReadOnlySpan<VoxelCell> ExpandedCells => _expanded;

    internal static VoxelPath Invalid(PathOutcome status) => new(status, double.PositiveInfinity, null, 0, null, null);
}

// Glue between the one-line queries and the generic search.
internal static class VoxelSearchRunner
{
    internal struct DelegateStepCost : IStepCost
    {
        public int SizeX;
        public int SizeY;
        public Func<VoxelCell, VoxelCell, double, double>? Step;
        public Func<VoxelCell, bool>? Blocked;

        public readonly double Cost(int source, int target, double baseCost)
        {
            VoxelCell to = VoxelGrid.Decode(target, SizeX, SizeY);
            if (Blocked is not null && Blocked(to))
            {
                return double.PositiveInfinity;
            }
            return Step is null ? baseCost : Step(VoxelGrid.Decode(source, SizeX, SizeY), to, baseCost);
        }
    }

    internal static VoxelPath Run<TCost>(
        VoxelTopology topology,
        TCost cost,
        VoxelHeuristic heuristic,
        VoxelSearchState state,
        VoxelCell start,
        VoxelCell goal,
        VoxelOptions options,
        Func<VoxelCell, VoxelPoint>? centre)
        where TCost : struct, IStepCost
    {
        SearchStatus status = state.Run(topology, cost, heuristic, start, goal, options.HeuristicWeight, options.IterationBudget, options.RecordExpansions,
            out int goalSlot, out double pathCost, out int expandedCount);
        VoxelCell[]? expanded = null;
        if (options.RecordExpansions)
        {
            ReadOnlySpan<int> e = state.ExpandedSlots;
            expanded = new VoxelCell[e.Length];
            for (int i = 0; i < e.Length; i++)
            {
                expanded[i] = state.CellOfSlot(e[i]);
            }
        }
        if (status == SearchStatus.Running)
        {
            return new VoxelPath(PathOutcome.BudgetExhausted, double.PositiveInfinity, null, expandedCount, expanded, null);
        }
        if (status == SearchStatus.NotFound)
        {
            return new VoxelPath(PathOutcome.NoPath, double.PositiveInfinity, null, expandedCount, expanded, null);
        }
        int length = 0;
        for (int s = goalSlot; s >= 0; s = state.ParentOf(s))
        {
            length++;
        }
        var cells = new VoxelCell[length];
        VoxelPoint[]? world = centre is null ? null : new VoxelPoint[length];
        int at = length;
        for (int s = goalSlot; s >= 0; s = state.ParentOf(s))
        {
            at--;
            cells[at] = state.CellOfSlot(s);
            if (world is not null)
            {
                world[at] = centre!(cells[at]);
            }
        }
        return new VoxelPath(PathOutcome.Found, pathCost, cells, expandedCount, expanded, world);
    }
}
