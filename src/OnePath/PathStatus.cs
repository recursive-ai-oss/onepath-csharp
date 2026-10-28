namespace RecursiveAI.OnePath;

/// <summary>
/// How a path query ended. Every one-call path result (<c>GridPath</c>, <c>VoxelPath</c>, <c>GraphPath</c>,
/// <c>PathResult</c>) and <c>Agent.SetDestination</c> report it, so calling code reads one vocabulary whatever the world is made of.
/// The numeric values are stable: they are written into hashes and logs.
/// </summary>
public enum PathStatus
{
    /// <summary>No query has produced a result: the value of <c>default(PathStatus)</c>, so an unassigned result never reads as success.</summary>
    None = 0,

    /// <summary>A path to the goal was found; it is optimal for the cost model unless a heuristic weight above 1 or an approximate algorithm was chosen.</summary>
    Found = 1,

    /// <summary>The start is outside the world, not walkable for the agent, or has no walkable surface nearby.</summary>
    StartInvalid = 2,

    /// <summary>The goal is outside the world, not walkable for the agent, or has no walkable surface nearby.</summary>
    GoalInvalid = 3,

    /// <summary>The search exhausted every reachable node without reaching the goal and the query does not return best-effort paths.</summary>
    NoPath = 4,

    /// <summary>The iteration budget ran out before the search finished (a sliced query can be continued).</summary>
    BudgetExhausted = 5,

    /// <summary>The goal cannot be reached (or a node limit was hit); the path leads to the reachable place closest to the goal. Navmesh queries and crowd agents report this; grid and graph queries report <see cref="NoPath"/>.</summary>
    Partial = 6,

    /// <summary>The search has not finished and the host's next steps continue it. Crowd agents report this for a long route that is still being extended.</summary>
    InProgress = 7,
}

/// <summary>Helpers on <see cref="PathStatus"/>.</summary>
public static class PathStatusExtensions
{
    /// <summary>True for <see cref="PathStatus.Found"/> and <see cref="PathStatus.Partial"/>: the result holds a usable path.</summary>
    /// <param name="status">The status.</param>
    /// <returns>Whether a path is available.</returns>
    public static bool HasPath(this PathStatus status) => status is PathStatus.Found or PathStatus.Partial;
}
