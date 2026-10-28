namespace RecursiveAI.OnePath;

/// <summary>A cell of a grid: <see cref="X"/> is the column, <see cref="Y"/> the row, both zero-based.</summary>
/// <param name="X">Column, 0 at the left.</param>
/// <param name="Y">Row, 0 at the top.</param>
public readonly record struct Cell(int X, int Y);

/// <summary>A position in world space, in double precision. Coordinates must lie within ±2^21 (see <see cref="GridSpace"/>).</summary>
/// <param name="X">World x.</param>
/// <param name="Y">World y.</param>
public readonly record struct WorldPoint(double X, double Y);
