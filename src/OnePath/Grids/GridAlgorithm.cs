using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>Which algorithm a one-line square-grid query uses.</summary>
public enum GridAlgorithm
{
    /// <summary>A* over cells (the default). Handles every option.</summary>
    AStar = 0,

    /// <summary>
    /// Jump Point Search (Harabor &amp; Grastien 2011) with the no-corner-cutting rule. It needs eight-connected moves,
    /// <see cref="DiagonalRule.NoCornerCutting"/>, agent size 1, one cost multiplier for every area, no cost or blocking
    /// callbacks, a heuristic weight of 1 and the octile or zero heuristic; for any other query the search runs as
    /// <see cref="AStar"/> and <see cref="GridPath.Algorithm"/> says so.
    /// </summary>
    Jps = 1,

    /// <summary>
    /// JPS+ (jump distances precomputed per cell and direction, kept up to date under cell edits). Same requirements as
    /// <see cref="Jps"/>, and the longer side of the grid must be at most <see cref="JpsPlusIndex.MaxSide"/> cells; otherwise
    /// the query falls back to <see cref="Jps"/> or <see cref="AStar"/>. The first query on a grid builds the tables.
    /// </summary>
    JpsPlus = 2,

    /// <summary>
    /// Theta* (Nash et al. 2007): any-angle paths, returned as waypoints with a line of sight between consecutive ones, found
    /// close to (not always exactly) the shortest. Needs eight moves, <see cref="DiagonalRule.NoCornerCutting"/>, agent size 1, one
    /// cost multiplier for every area, no cost or blocking callbacks and the <see cref="GridHeuristic.Auto"/> (straight-line),
    /// <see cref="GridHeuristic.Euclidean"/> or <see cref="GridHeuristic.Zero"/> estimate; otherwise A* runs and
    /// <see cref="GridPath.Algorithm"/> says so. <see cref="GridPath.Cost"/> is the Euclidean length.
    /// </summary>
    ThetaStar = 3,

    /// <summary>Lazy Theta* (Nash et al. 2010): Theta* that tests the line of sight when a node is expanded instead of when it is generated, with the corrected-cost re-insertion of <see cref="ThetaVariant.LazyRequeue"/>. Same requirements and result form as <see cref="ThetaStar"/>.</summary>
    LazyThetaStar = 4,

    /// <summary>
    /// Lifelong Planning A* (Koenig, Likhachev &amp; Furcy 2004) for <see cref="Grid.CreateReplanner"/>: a fixed start, repaired after
    /// every grid edit. A one-off <c>FindPath</c> with this value runs A* (and says so in <see cref="GridPath.Algorithm"/>).
    /// </summary>
    LpaStar = 8,

    /// <summary>
    /// D* Lite (Koenig &amp; Likhachev 2002) for <see cref="Grid.CreateReplanner"/>: searches from the goal so the start can move while the grid
    /// changes. The replanner's default. A one-off <c>FindPath</c> with this value runs A*.
    /// </summary>
    DStarLite = 9,

    /// <summary>
    /// HPA* (Botea, Müller &amp; Schaeffer 2004): a hierarchy of chunks, near-optimal paths (optionally smoothed), updated incrementally
    /// when cells change (see <see cref="HpaHierarchy"/>; tuned by <see cref="GridOptions.HpaChunkSize"/> and <see cref="GridOptions.HpaSmoothing"/>).
    /// Needs no cost or blocking callbacks and not <see cref="DiagonalRule.AllowAll"/> on eight-connected grids; it ignores the
    /// heuristic options. Any other query runs as <see cref="AStar"/> and <see cref="GridPath.Algorithm"/> says so.
    /// The first query for an agent size and chunk size builds the hierarchy.
    /// </summary>
    Hpa = 5,
}
