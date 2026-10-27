using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Direction numbering and pruning rules shared by the jump point searches (tier 3).
/// Directions are numbered like the neighbour order of <see cref="SquareTopology"/>: 0 north (0,-1), 1 east (1,0), 2 south (0,1),
/// 3 west (-1,0), 4 north-east (1,-1), 5 south-east (1,1), 6 south-west (-1,1), 7 north-west (-1,-1).
/// </summary>
public static class JumpRules
{
    /// <summary>Number of directions.</summary>
    public const int DirectionCount = 8;

    private static readonly int[] DxTable = [0, 1, 0, -1, 1, 1, -1, -1];
    private static readonly int[] DyTable = [-1, 0, 1, 0, -1, 1, 1, -1];

    // Index (dy + 1) * 3 + (dx + 1); the centre entry (no move) is -1.
    private static readonly int[] DirOf = [7, 0, 4, 3, -1, 1, 6, 2, 5];

    /// <summary>The column step of a direction.</summary>
    /// <param name="direction">0..7.</param>
    /// <returns>-1, 0 or 1.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The direction is outside 0..7.</exception>
    public static int Dx(int direction) => DxTable[CheckDir(direction)];

    /// <summary>The row step of a direction.</summary>
    /// <param name="direction">0..7.</param>
    /// <returns>-1, 0 or 1.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The direction is outside 0..7.</exception>
    public static int Dy(int direction) => DyTable[CheckDir(direction)];

    /// <summary>The direction for a step.</summary>
    /// <param name="dx">-1, 0 or 1.</param>
    /// <param name="dy">-1, 0 or 1; not both zero.</param>
    /// <returns>0..7.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A step is outside -1..1 or both are zero.</exception>
    public static int Direction(int dx, int dy)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(dx, -1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(dx, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(dy, -1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(dy, 1);
        int d = DirOf[((dy + 1) * 3) + dx + 1];
        if (d < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(dx), "A step needs a non-zero direction.");
        }
        return d;
    }

    /// <summary>
    /// The directions worth searching from a cell reached by a move of direction (<paramref name="dx"/>, <paramref name="dy"/>)
    /// (the sign of the move from the parent cell to this one), as a bit set over the direction numbers: the pruned
    /// neighbour set of Harabor &amp; Grastien 2011 under the no-corner-cutting rule.
    /// <list type="bullet">
    /// <item>Diagonal arrival: the diagonal itself and its two cardinal components. (A diagonal step into a cell needs both
    /// side cells free, so no neighbour is forced.)</item>
    /// <item>Straight arrival: straight on, plus on each side where the side cell is free but the cell beside the parent is a
    /// wall (that side cell cannot be reached more cheaply from the parent): the side cell's cardinal direction and
    /// the diagonal that continues straight-and-sideways.</item>
    /// </list>
    /// </summary>
    /// <typeparam name="TFree">Walkability lookup.</typeparam>
    /// <param name="free">Tells whether a cell is walkable (false outside the grid).</param>
    /// <param name="x">Column of the cell.</param>
    /// <param name="y">Row of the cell.</param>
    /// <param name="dx">Sign of the arrival step, -1..1.</param>
    /// <param name="dy">Sign of the arrival step, -1..1.</param>
    /// <returns>A bit set: bit <c>d</c> set means direction <c>d</c> is searched.</returns>
    public static int Successors<TFree>(TFree free, int x, int y, int dx, int dy)
        where TFree : struct, IFreeCells
    {
        if (dx != 0 && dy != 0)
        {
            return (1 << DirOf[((dy + 1) * 3) + dx + 1]) | (1 << DirOf[3 + dx + 1]) | (1 << DirOf[((dy + 1) * 3) + 1]);
        }
        int mask = 1 << DirOf[((dy + 1) * 3) + dx + 1];
        // Sides are perpendicular to the arrival step: (dy, dx) and its negation.
        int sx = dy;
        int sy = dx;
        if (free.IsFree(x + sx, y + sy) && !free.IsFree(x + sx - dx, y + sy - dy))
        {
            mask |= (1 << DirOf[((sy + 1) * 3) + sx + 1]) | (1 << DirOf[((sy + dy + 1) * 3) + sx + dx + 1]);
        }
        if (free.IsFree(x - sx, y - sy) && !free.IsFree(x - sx - dx, y - sy - dy))
        {
            mask |= (1 << DirOf[((1 - sy) * 3) + 1 - sx]) | (1 << DirOf[((dy - sy + 1) * 3) + dx - sx + 1]);
        }
        return mask;
    }

    /// <summary>
    /// Whether a straight move of direction (<paramref name="dx"/>, <paramref name="dy"/>) arriving at the walkable cell
    /// (<paramref name="x"/>, <paramref name="y"/>) makes it a jump point: a side cell is free while the cell beside the
    /// previous cell is a wall, so the side cell is reachable optimally only through this cell
    /// (Harabor &amp; Grastien 2011, forced neighbours; with corner cutting forbidden the forced cell is the side cell
    /// itself, one step later than in the corner-cutting formulation).
    /// </summary>
    /// <typeparam name="TFree">Walkability lookup.</typeparam>
    /// <param name="free">Tells whether a cell is walkable.</param>
    /// <param name="x">Column of the cell.</param>
    /// <param name="y">Row of the cell.</param>
    /// <param name="dx">Step of the straight move; exactly one of dx, dy is non-zero.</param>
    /// <param name="dy">Step of the straight move.</param>
    /// <returns>True if the cell has a forced neighbour for this arrival direction.</returns>
    public static bool IsForced<TFree>(TFree free, int x, int y, int dx, int dy)
        where TFree : struct, IFreeCells
    {
        int sx = dy;
        int sy = dx;
        return (free.IsFree(x + sx, y + sy) && !free.IsFree(x + sx - dx, y + sy - dy))
            || (free.IsFree(x - sx, y - sy) && !free.IsFree(x - sx - dx, y - sy - dy));
    }

    private static int CheckDir(int direction)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)direction, 7u);
        return direction;
    }
}

/// <summary>Walkability of cells for the jump point searches.</summary>
public interface IFreeCells
{
    /// <summary>True if (<paramref name="x"/>, <paramref name="y"/>) is inside the grid and not a wall.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>True if walkable.</returns>
    bool IsFree(int x, int y);
}

/// <summary>A view of a grid's cells as walkable or not (area 255 is a wall). Reads the grid's array, so edits show through.</summary>
public readonly struct GridFreeCells : IFreeCells
{
    private readonly byte[] _cells;
    private readonly int _width;
    private readonly int _height;

    /// <summary>Creates the view.</summary>
    /// <param name="grid">The grid.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    public GridFreeCells(Grid grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        _cells = grid.CellArray;
        _width = grid.Width;
        _height = grid.Height;
    }

    /// <summary>Columns.</summary>
    public int Width => _width;

    /// <summary>Rows.</summary>
    public int Height => _height;

    /// <inheritdoc/>
    public bool IsFree(int x, int y) => (uint)x < (uint)_width && (uint)y < (uint)_height && _cells[(y * _width) + x] != AreaCosts.Wall;
}

/// <summary>
/// Finds the next jump point from a cell in one direction (tier 2/3). Implementations: <see cref="OnlineJumper"/> (scans the grid
/// cell by cell, the plain algorithm), <see cref="BlockJumper"/> (scans 64 cells per word) and <see cref="TableJumper"/> (reads
/// <see cref="JpsPlusIndex"/> tables).
/// </summary>
public interface IJumper : IFreeCells
{
    /// <summary>Columns of the grid.</summary>
    int Width { get; }

    /// <summary>Rows of the grid.</summary>
    int Height { get; }

    /// <summary>Called at the start of every search so a jumper that caches grid-derived data can bring it up to date (a no-op for the scanning jumpers).</summary>
    void BeginSearch();

    /// <summary>
    /// Walks from <paramref name="node"/> in <paramref name="direction"/> until a jump point or the goal and returns it.
    /// The result is the same cell for every implementation, except that a table-driven jump may stop earlier at
    /// a cell in line with the goal during a diagonal run (see docs/grids.md); both are valid search nodes.
    /// </summary>
    /// <param name="node">Cell index (row-major) to start from; not included in the walk.</param>
    /// <param name="direction">0..7, see <see cref="JumpRules"/>.</param>
    /// <param name="goal">Goal cell index.</param>
    /// <param name="steps">Number of unit steps to the returned cell (0 if none).</param>
    /// <returns>The cell index reached, or -1 if the walk meets a wall first.</returns>
    int Jump(int node, int direction, int goal, out int steps);
}
