namespace RecursiveAI.OnePath;

/// <summary>
/// How a path query ended. Every one-call path result (<c>GridPath</c>, <c>VoxelPath</c>, <c>GraphPath</c>,
/// <c>PathResult</c>) and <c>Agent.SetDestination</c> report it, so calling code reads one vocabulary whatever the world is made of.
/// The numeric values are stable: they are written into hashes and logs.
/// </summary>
public enum PathStatus
{
    /// <summary>A path to the goal was found; it is optimal for the cost model unless a heuristic weight above 1 or an approximate algorithm was chosen.</summary>
    Found = 0,

    /// <summary>The start is outside the world, not walkable for the agent, or has no walkable surface nearby.</summary>
    StartInvalid = 1,

    /// <summary>The goal is outside the world, not walkable for the agent, or has no walkable surface nearby.</summary>
    GoalInvalid = 2,

    /// <summary>The search exhausted every reachable node without reaching the goal and the query does not return best-effort paths.</summary>
    NoPath = 3,

    /// <summary>The iteration budget ran out before the search finished (a sliced query can be continued).</summary>
    BudgetExhausted = 4,

    /// <summary>The goal cannot be reached (or a node limit was hit); the path leads to the reachable place closest to the goal. Navmesh queries and crowd agents report this; grid and graph queries report <see cref="NoPath"/>.</summary>
    Partial = 5,

    /// <summary>The search has not finished and the host's next steps continue it. Crowd agents report this for a long route that is still being extended.</summary>
    InProgress = 6,
}

/// <summary>Helpers on <see cref="PathStatus"/>.</summary>
public static class PathStatusExtensions
{
    /// <summary>True for <see cref="PathStatus.Found"/> and <see cref="PathStatus.Partial"/>: the result holds a usable path.</summary>
    /// <param name="status">The status.</param>
    /// <returns>Whether a path is available.</returns>
    public static bool HasPath(this PathStatus status) => status is PathStatus.Found or PathStatus.Partial;
}
