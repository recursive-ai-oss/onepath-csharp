using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Jump point search by scanning the grid one cell at a time (Harabor &amp; Grastien 2011): straight scans stop at a wall, the goal or a cell with a
/// forced neighbour; diagonal runs test both straight directions from every cell. Iterative; no recursion.
/// </summary>
public readonly struct OnlineJumper : IJumper
{
    private readonly byte[] _cells;
    private readonly int _width;
    private readonly int _height;

    /// <summary>Creates a jumper over a grid's cells (edits show through).</summary>
    /// <param name="grid">The grid.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    public OnlineJumper(Grid grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        _cells = grid.CellArray;
        _width = grid.Width;
        _height = grid.Height;
    }

    /// <inheritdoc/>
    public int Width => _width;

    /// <inheritdoc/>
    public int Height => _height;

    /// <inheritdoc/>
    public bool IsFree(int x, int y) => (uint)x < (uint)_width && (uint)y < (uint)_height && _cells[(y * _width) + x] != AreaCosts.Wall;

    /// <inheritdoc/>
    public void BeginSearch()
    {
    }

    /// <inheritdoc/>
    public int Jump(int node, int direction, int goal, out int steps)
    {
        int y = node / _width;
        int x = node - (y * _width);
        int dx = JumpRules.Dx(direction);
        int dy = JumpRules.Dy(direction);
        int gy = goal / _width;
        int gx = goal - (gy * _width);
        steps = dx != 0 && dy != 0 ? Diagonal(x, y, dx, dy, gx, gy) : Straight(x, y, dx, dy, gx, gy);
        return steps == 0 ? -1 : node + (steps * (dx + (dy * _width)));
    }

    // Returns the number of steps to the first cell that is the goal or has a forced neighbour; 0 if a wall comes first.
    private int Straight(int x, int y, int dx, int dy, int gx, int gy)
    {
        int n = 0;
        while (true)
        {
            x += dx;
            y += dy;
            n++;
            if (!IsFree(x, y))
            {
                return 0;
            }
            if ((x == gx && y == gy) || JumpRules.IsForced(this, x, y, dx, dy))
            {
                return n;
            }
        }
    }

    private int Diagonal(int x, int y, int dx, int dy, int gx, int gy)
    {
        int n = 0;
        while (true)
        {
            // No corner cutting: both side cells and the target must be free.
            if (!(IsFree(x + dx, y) && IsFree(x, y + dy) && IsFree(x + dx, y + dy)))
            {
                return 0;
            }
            x += dx;
            y += dy;
            n++;
            if ((x == gx && y == gy) || Straight(x, y, dx, 0, gx, gy) != 0 || Straight(x, y, 0, dy, gx, gy) != 0)
            {
                return n;
            }
        }
    }
}

/// <summary>
/// JPS+ lookups (Game AI Pro 2, "JPS+: An Extreme A* Speed Optimization for Static Uniform Cost Grids"): the jump distance
/// of every cell and direction is read from a <see cref="JpsPlusIndex"/>, so a jump costs one array read. The goal
/// is handled by bounding (docs/grids.md): a goal on the straight line within the stored distance is returned; during
/// a diagonal run a goal in the run's quadrant stops the run at the cell level with the goal.
/// </summary>
public readonly struct TableJumper : IJumper
{
    private readonly JpsPlusIndex _index;
    private readonly short[] _table;
    private readonly int _width;
    private readonly int _height;
    private readonly byte[] _cells;

    /// <summary>Creates a jumper over an index. The index is brought up to date with pending cell edits.</summary>
    /// <param name="index">The tables.</param>
    /// <exception cref="ArgumentNullException"><paramref name="index"/> is null.</exception>
    public TableJumper(JpsPlusIndex index)
    {
        ArgumentNullException.ThrowIfNull(index);
        index.Refresh();
        _index = index;
        _table = index.Table;
        _width = index.Grid.Width;
        _height = index.Grid.Height;
        _cells = index.Grid.CellArray;
    }

    /// <summary>The index this jumper reads.</summary>
    public JpsPlusIndex Index => _index;

    /// <inheritdoc/>
    public int Width => _width;

    /// <inheritdoc/>
    public int Height => _height;

    /// <inheritdoc/>
    public bool IsFree(int x, int y) => (uint)x < (uint)_width && (uint)y < (uint)_height && _cells[(y * _width) + x] != AreaCosts.Wall;

    /// <inheritdoc/>
    public void BeginSearch() => _index.Refresh();

    /// <inheritdoc/>
    public int Jump(int node, int direction, int goal, out int steps)
    {
        int jd = _table[(node << 3) + direction];
        int len = jd > 0 ? jd : -jd;
        int y = node / _width;
        int x = node - (y * _width);
        int gy = goal / _width;
        int gx = goal - (gy * _width);
        int dx = JumpRules.Dx(direction);
        int dy = JumpRules.Dy(direction);
        int gdx = (gx - x) * dx;
        int gdy = (gy - y) * dy;
        if (dx == 0 || dy == 0)
        {
            // Straight: the goal on the line, ahead, within the stored distance.
            if (dx != 0 ? (gy == y && gdx > 0) : (gx == x && gdy > 0))
            {
                int k = dx != 0 ? gdx : gdy;
                if (k <= len)
                {
                    steps = k;
                    return goal;
                }
            }
        }
        else
        {
            // Diagonal: if the goal lies in the quadrant, stop at the cell level with it, when the run gets that far.
            int m = Math.Min(gdx, gdy);
            if (gdx > 0 && gdy > 0 && m <= len)
            {
                steps = m;
                return node + (m * (dx + (dy * _width)));
            }
        }
        if (jd > 0)
        {
            steps = jd;
            return node + (jd * (dx + (dy * _width)));
        }
        steps = 0;
        return -1;
    }
}
