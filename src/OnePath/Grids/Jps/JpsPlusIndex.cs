using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// JPS+ tables for a <see cref="Grid"/>: for every cell and each of the eight directions (numbered as in <see cref="JumpRules"/>)
/// the distance to the next jump point (positive) or to the last walkable cell before a wall (zero or negative: minus
/// that distance). Rabin's JPS+ (Game AI Pro 2) stores the same quantity. Stored cell by cell (the eight directions of a cell side by side),
/// 16-bit signed. A wall cell has all zeros.
/// <para>
/// The index subscribes to the grid's cell edits and, before the next query (<see cref="Refresh"/>), repairs only the
/// entries an edit can change: the three rows around an edited cell for east and west, the three columns for north and south, then
/// the diagonal entries that depend on them, followed along each diagonal until nothing changes. The result is identical to
/// <see cref="Rebuild"/>. A large batch of edits (more than a 64th of the cells) is answered by a rebuild. Preprocessing is
/// O(8 * cells) time and 16 * cells bytes. Not thread-safe; the grid must not change while a search runs. Call
/// <see cref="Dispose"/> to stop listening.
/// </para>
/// </summary>
public sealed class JpsPlusIndex : IGridBatchListener, IDisposable
{
    /// <summary>The longest side (columns or rows) an index supports: distances are 16-bit.</summary>
    public const int MaxSide = 32766;

    private readonly Grid _grid;
    private readonly byte[] _cells;
    private readonly short[] _t;
    private readonly int _w;
    private readonly int _h;
    private readonly int _n;
    private readonly List<int> _pending = [];
    private bool _all;
    private int _version;

    /// <summary>Builds the tables for a grid and starts following its edits.</summary>
    /// <param name="grid">The grid.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A side of the grid is longer than <see cref="MaxSide"/>.</exception>
    public JpsPlusIndex(Grid grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        if (grid.Width > MaxSide || grid.Height > MaxSide)
        {
            throw new ArgumentOutOfRangeException(nameof(grid), $"A JPS+ index supports sides up to {MaxSide} cells; the grid is {grid.Width}x{grid.Height}.");
        }
        _grid = grid;
        _cells = grid.CellArray;
        _w = grid.Width;
        _h = grid.Height;
        _n = grid.CellCount;
        _t = new short[8 * _n];
        BuildAll();
        _version = grid.Version;
        grid.AddListener(this);
    }

    /// <summary>The grid this index describes.</summary>
    public Grid Grid => _grid;

    /// <summary>
    /// All tables in the order <c>Tables[(direction * CellCount) + cell]</c>, as a new copy each time (the searches read the
    /// cell-major store, which keeps the eight entries of a cell on one cache line). Call <see cref="Refresh"/> first after edits.
    /// </summary>
    public ReadOnlySpan<short> Tables
    {
        get
        {
            var copy = new short[_t.Length];
            for (int cell = 0; cell < _n; cell++)
            {
                for (int d = 0; d < 8; d++)
                {
                    copy[(d * _n) + cell] = _t[(cell << 3) + d];
                }
            }
            return copy;
        }
    }

    internal short[] Table => _t;

    /// <summary>The stored distance for a cell and direction.</summary>
    /// <param name="direction">0..7.</param>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>Positive: steps to the jump point; otherwise minus the steps to the last walkable cell.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The direction or the cell is out of range.</exception>
    public int JumpDistance(int direction, int x, int y)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)direction, 7u);
        if (!_grid.Contains(x, y))
        {
            throw new ArgumentOutOfRangeException(nameof(x), $"Cell ({x}, {y}) is outside the {_w}x{_h} grid.");
        }
        return _t[(((y * _w) + x) << 3) + direction];
    }

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y)
    {
        if (_all)
        {
            return;
        }
        if (_pending.Count >= Math.Max(256, _n / 64))
        {
            _all = true;
            _pending.Clear();
            return;
        }
        _pending.Add((y * _w) + x);
    }

    /// <inheritdoc/>
    public void OnRectChanged(int x, int y, int width, int height)
    {
        if (_all)
        {
            return;
        }
        if (_pending.Count + ((long)width * height) > Math.Max(256, _n / 64))
        {
            _all = true;
            _pending.Clear();
            return;
        }
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                _pending.Add(((y + j) * _w) + x + i);
            }
        }
    }

    /// <summary>Stops following the grid's edits.</summary>
    public void Dispose() => _grid.RemoveListener(this);

    /// <summary>Applies the edits made since the last call (called by the searches before they read the tables).</summary>
    public void Refresh()
    {
        if (_grid.Version == _version)
        {
            return;
        }
        if (_all)
        {
            BuildAll();
        }
        else
        {
            ApplyEdits();
        }
        _all = false;
        _pending.Clear();
        _version = _grid.Version;
    }

    /// <summary>Recomputes every table from the grid (what the constructor does).</summary>
    public void Rebuild()
    {
        BuildAll();
        _all = false;
        _pending.Clear();
        _version = _grid.Version;
    }

    private bool Free(int x, int y) => (uint)x < (uint)_w && (uint)y < (uint)_h && _cells[(y * _w) + x] != AreaCosts.Wall;

    private readonly struct Cells : IFreeCells
    {
        private readonly JpsPlusIndex _owner;

        public Cells(JpsPlusIndex owner) => _owner = owner;

        public bool IsFree(int x, int y) => _owner.Free(x, y);
    }

    // One table entry from its neighbour one step ahead in the same direction (which must already be current).
    private short Compute(int x, int y, int dir)
    {
        if (!Free(x, y))
        {
            return 0;
        }
        int dx = JumpRules.Dx(dir);
        int dy = JumpRules.Dy(dir);
        int qx = x + dx;
        int qy = y + dy;
        int q = (qy * _w) + qx;
        if (dx == 0 || dy == 0)
        {
            if (!Free(qx, qy))
            {
                return 0;
            }
            if (JumpRules.IsForced(new Cells(this), qx, qy, dx, dy))
            {
                return 1;
            }
        }
        else
        {
            // No corner cutting: both side cells and the target must be free.
            if (!(Free(qx, y) && Free(x, qy) && Free(qx, qy)))
            {
                return 0;
            }
            // The cell is a diagonal jump point if a straight scan from the next cell finds a jump point.
            if (_t[(q << 3) + JumpRules.Direction(dx, 0)] > 0 || _t[(q << 3) + JumpRules.Direction(0, dy)] > 0)
            {
                return 1;
            }
        }
        int ahead = _t[(q << 3) + dir];
        return (short)(ahead > 0 ? ahead + 1 : ahead - 1);
    }

    private void BuildAll()
    {
        for (int dir = 0; dir < 8; dir++)
        {
            int dx = JumpRules.Dx(dir);
            int dy = JumpRules.Dy(dir);
            // Visit cells so that the cell one step ahead is done first (the straight directions 0..3 come before the diagonals).
            for (int j = 0; j < _h; j++)
            {
                int y = dy > 0 ? _h - 1 - j : j;
                for (int i = 0; i < _w; i++)
                {
                    int x = dx > 0 ? _w - 1 - i : i;
                    _t[(((y * _w) + x) << 3) + dir] = Compute(x, y, dir);
                }
            }
        }
    }

    private void ApplyEdits()
    {
        var rows = new List<int>();
        var cols = new List<int>();
        foreach (int c in _pending)
        {
            int y = c / _w;
            int x = c - (y * _w);
            for (int k = -1; k <= 1; k++)
            {
                if ((uint)(y + k) < (uint)_h)
                {
                    rows.Add(y + k);
                }
                if ((uint)(x + k) < (uint)_w)
                {
                    cols.Add(x + k);
                }
            }
        }
        rows.Sort();
        cols.Sort();
        var seeds = new List<int>(_pending);
        int last = -1;
        foreach (int r in rows)
        {
            if (r == last)
            {
                continue;
            }
            last = r;
            RepairLine(1, r, true, seeds);
            RepairLine(3, r, true, seeds);
        }
        last = -1;
        foreach (int c in cols)
        {
            if (c == last)
            {
                continue;
            }
            last = c;
            RepairLine(0, c, false, seeds);
            RepairLine(2, c, false, seeds);
        }

        // Diagonal entries read: walkability of the cell, its two side cells and the target, the straight tables'
        // sign at the target, and the same diagonal's entry at the target.
        var work = new Stack<int>();
        foreach (int e in seeds)
        {
            int ey = e / _w;
            int ex = e - (ey * _w);
            for (int d = 4; d < 8; d++)
            {
                int dx = JumpRules.Dx(d);
                int dy = JumpRules.Dy(d);
                Push(work, ex, ey, d);
                Push(work, ex - dx, ey, d);
                Push(work, ex, ey - dy, d);
                Push(work, ex - dx, ey - dy, d);
            }
        }
        while (work.Count > 0)
        {
            int item = work.Pop();
            int d = 4 + (item & 3);
            int cell = item >> 2;
            int y = cell / _w;
            int x = cell - (y * _w);
            short v = Compute(x, y, d);
            int slot = (cell << 3) + d;
            if (_t[slot] != v)
            {
                _t[slot] = v;
                Push(work, x - JumpRules.Dx(d), y - JumpRules.Dy(d), d);
            }
        }
    }

    private void Push(Stack<int> work, int x, int y, int dir)
    {
        if ((uint)x < (uint)_w && (uint)y < (uint)_h)
        {
            work.Push((((y * _w) + x) << 2) | (dir - 4));
        }
    }

    // Recompute one row (east or west) or column (north or south); cells whose sign changed seed the diagonal repair.
    private void RepairLine(int dir, int line, bool isRow, List<int> seeds)
    {
        int step = dir is 1 or 2 ? -1 : 1; // visit from the far end towards the start of the direction
        int len = isRow ? _w : _h;
        for (int i = 0; i < len; i++)
        {
            int p = step < 0 ? len - 1 - i : i;
            int x = isRow ? p : line;
            int y = isRow ? line : p;
            int slot = (((y * _w) + x) << 3) + dir;
            short v = Compute(x, y, dir);
            short old = _t[slot];
            if (v != old)
            {
                _t[slot] = v;
                if ((old > 0) != (v > 0))
                {
                    seeds.Add((y * _w) + x);
                }
            }
        }
    }
}
