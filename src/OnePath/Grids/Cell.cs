namespace RecursiveAI.OnePath;

/// <summary>A cell of a grid: <see cref="X"/> is the column, <see cref="Y"/> the row, both zero-based.</summary>
/// <param name="X">Column, 0 at the left.</param>
/// <param name="Y">Row, 0 at the top.</param>
public readonly record struct Cell(int X, int Y);

/// <summary>A position in world space, in double precision. Coordinates must lie within ±2^21 (see <see cref="GridSpace"/>).</summary>
/// <param name="X">World x.</param>
/// <param name="Y">World y.</param>
public readonly record struct WorldPoint(double X, double Y);

/// <summary>Why a grid query ended.</summary>
public enum PathOutcome
{
    /// <summary>A path was found; it is optimal for the cost model unless a heuristic weight above 1 was used.</summary>
    Found = 0,

    /// <summary>The start is outside the grid or not walkable for the agent.</summary>
    StartInvalid = 1,

    /// <summary>The goal is outside the grid or not walkable for the agent.</summary>
    GoalInvalid = 2,

    /// <summary>The search exhausted every reachable node without reaching the goal.</summary>
    NoPath = 3,

    /// <summary>The iteration budget ran out before the search finished.</summary>
    BudgetExhausted = 4,
}
