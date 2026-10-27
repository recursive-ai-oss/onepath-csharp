using System;
using System.Numerics;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// The walkable cells of a <see cref="Grid"/> as bitmaps, once by rows and once by columns (the same cells transposed), so a
/// straight scan reads 64 cells per word. This is the block-based scanning of Harabor &amp; Grastien 2014 ("Improving Jump
/// Point Search"). Bit <c>i</c> of word <c>k</c> of a line is the cell at position <c>64 * k + i</c>; cells beyond the line end are
/// 0 (blocked). The map follows the grid's cell edits as they happen (O(1) per edit). Not thread-safe against edits.
/// Call <see cref="Dispose"/> to stop listening.
/// </summary>
public sealed class BlockMap : IGridChangeListener, IDisposable
{
    private readonly Grid _grid;
    private readonly byte[] _cells;
    private readonly int _w;
    private readonly int _h;
    private readonly int _rowWords;
    private readonly int _colWords;
    private readonly ulong[] _rows;
    private readonly ulong[] _cols;

    /// <summary>Builds the bitmaps for a grid and starts following its edits.</summary>
    /// <param name="grid">The grid.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    public BlockMap(Grid grid)
    {
        ArgumentNullException.ThrowIfNull(grid);
        _grid = grid;
        _cells = grid.CellArray;
        _w = grid.Width;
        _h = grid.Height;
        _rowWords = (_w + 63) >> 6;
        _colWords = (_h + 63) >> 6;
        _rows = new ulong[_h * _rowWords];
        _cols = new ulong[_w * _colWords];
        for (int y = 0; y < _h; y++)
        {
            for (int x = 0; x < _w; x++)
            {
                if (_cells[(y * _w) + x] != AreaCosts.Wall)
                {
                    _rows[(y * _rowWords) + (x >> 6)] |= 1UL << (x & 63);
                    _cols[(x * _colWords) + (y >> 6)] |= 1UL << (y & 63);
                }
            }
        }
        grid.AddListener(this);
    }

    /// <summary>The grid this map describes.</summary>
    public Grid Grid => _grid;

    /// <summary>Columns.</summary>
    public int Width => _w;

    /// <summary>Rows.</summary>
    public int Height => _h;

    /// <summary>Words per row of <see cref="Rows"/>.</summary>
    public int RowWords => _rowWords;

    /// <summary>Words per column of <see cref="Columns"/>.</summary>
    public int ColumnWords => _colWords;

    /// <summary>Walkable bits by rows: row <c>y</c> starts at word <c>y * RowWords</c>.</summary>
    public ReadOnlySpan<ulong> Rows => _rows;

    /// <summary>Walkable bits by columns: column <c>x</c> starts at word <c>x * ColumnWords</c>, bit <c>y</c> is row <c>y</c>.</summary>
    public ReadOnlySpan<ulong> Columns => _cols;

    internal ulong[] RowArray => _rows;

    internal ulong[] ColumnArray => _cols;

    /// <summary>True if the cell is inside the grid and walkable.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>True if walkable.</returns>
    public bool IsFree(int x, int y) => (uint)x < (uint)_w && (uint)y < (uint)_h && ((_rows[(y * _rowWords) + (x >> 6)] >> (x & 63)) & 1) != 0;

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y)
    {
        bool free = _cells[(y * _w) + x] != AreaCosts.Wall;
        ulong rowBit = 1UL << (x & 63);
        ulong colBit = 1UL << (y & 63);
        if (free)
        {
            _rows[(y * _rowWords) + (x >> 6)] |= rowBit;
            _cols[(x * _colWords) + (y >> 6)] |= colBit;
        }
        else
        {
            _rows[(y * _rowWords) + (x >> 6)] &= ~rowBit;
            _cols[(x * _colWords) + (y >> 6)] &= ~colBit;
        }
    }

    /// <summary>Stops following the grid's edits.</summary>
    public void Dispose() => _grid.RemoveListener(this);

    // Scan one line of a bitmap forwards from position pos (not included). Returns the number of steps to the first cell that
    // is the goal (goalPos, or -1 for none) or has a forced neighbour, or 0 if a wall (or the line end) comes first. A cell
    // has a forced neighbour when a cell beside the line is free and the one beside the previous line cell is not
    // (docs/grids.md); those bits are found for 64 cells at once from the two adjacent lines.
    internal static int ScanForward(ulong[] bits, int stride, int lines, int line, int pos, int goalPos)
    {
        int p0 = pos + 1;
        int k = p0 >> 6;
        if (k >= stride)
        {
            return 0;
        }
        int self = line * stride;
        int up = line > 0 ? (line - 1) * stride : -1;
        int down = line + 1 < lines ? (line + 1) * stride : -1;
        ulong mask = ~0UL << (p0 & 63);
        for (; k < stride; k++)
        {
            ulong w = bits[self + k];
            ulong a = up >= 0 ? bits[up + k] : 0;
            ulong b = down >= 0 ? bits[down + k] : 0;
            ulong ac = up >= 0 && k > 0 ? bits[up + k - 1] >> 63 : 0;
            ulong bc = down >= 0 && k > 0 ? bits[down + k - 1] >> 63 : 0;
            ulong forced = (a & ~((a << 1) | ac)) | (b & ~((b << 1) | bc));
            ulong stops = (~w | forced) & mask;
            int s = stops != 0 ? (k << 6) + BitOperations.TrailingZeroCount(stops) : int.MaxValue;
            if (goalPos >= p0 && (goalPos >> 6) == k && goalPos <= s)
            {
                return goalPos - pos;
            }
            if (stops != 0)
            {
                return ((w >> (s & 63)) & 1) == 0 ? 0 : s - pos;
            }
            mask = ~0UL;
        }
        return 0;
    }

    // The same scan towards smaller positions.
    internal static int ScanBackward(ulong[] bits, int stride, int lines, int line, int pos, int goalPos)
    {
        int p0 = pos - 1;
        if (p0 < 0)
        {
            return 0;
        }
        int k = p0 >> 6;
        int self = line * stride;
        int up = line > 0 ? (line - 1) * stride : -1;
        int down = line + 1 < lines ? (line + 1) * stride : -1;
        ulong mask = ~0UL >> (63 - (p0 & 63));
        for (; k >= 0; k--)
        {
            ulong w = bits[self + k];
            ulong a = up >= 0 ? bits[up + k] : 0;
            ulong b = down >= 0 ? bits[down + k] : 0;
            ulong ac = up >= 0 && k + 1 < stride ? bits[up + k + 1] << 63 : 0;
            ulong bc = down >= 0 && k + 1 < stride ? bits[down + k + 1] << 63 : 0;
            ulong forced = (a & ~((a >> 1) | ac)) | (b & ~((b >> 1) | bc));
            ulong stops = (~w | forced) & mask;
            int s = stops != 0 ? (k << 6) + 63 - BitOperations.LeadingZeroCount(stops) : int.MinValue;
            if (goalPos >= 0 && goalPos <= p0 && (goalPos >> 6) == k && goalPos >= s)
            {
                return pos - goalPos;
            }
            if (stops != 0)
            {
                return ((w >> (s & 63)) & 1) == 0 ? 0 : pos - s;
            }
            mask = ~0UL;
        }
        return 0;
    }
}

/// <summary>
/// Jump point search with block-based scanning (Harabor &amp; Grastien 2014): the same jump points as <see cref="OnlineJumper"/>,
/// found with word operations on a <see cref="BlockMap"/>. Iterative; no recursion.
/// </summary>
public readonly struct BlockJumper : IJumper
{
    private readonly BlockMap _map;
    private readonly ulong[] _rows;
    private readonly ulong[] _cols;
    private readonly int _rowWords;
    private readonly int _colWords;
    private readonly int _width;
    private readonly int _height;

    /// <summary>Creates a jumper over a block map.</summary>
    /// <param name="map">The bitmaps.</param>
    /// <exception cref="ArgumentNullException"><paramref name="map"/> is null.</exception>
    public BlockJumper(BlockMap map)
    {
        ArgumentNullException.ThrowIfNull(map);
        _map = map;
        _rows = map.RowArray;
        _cols = map.ColumnArray;
        _rowWords = map.RowWords;
        _colWords = map.ColumnWords;
        _width = map.Width;
        _height = map.Height;
    }

    /// <summary>The map this jumper reads.</summary>
    public BlockMap Map => _map;

    /// <inheritdoc/>
    public int Width => _width;

    /// <inheritdoc/>
    public int Height => _height;

    /// <inheritdoc/>
    public bool IsFree(int x, int y) => _map.IsFree(x, y);

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

    private int Straight(int x, int y, int dx, int dy, int gx, int gy)
    {
        if (dy == 0)
        {
            int goalPos = gy == y ? gx : -1;
            return dx > 0
                ? BlockMap.ScanForward(_rows, _rowWords, _height, y, x, goalPos)
                : BlockMap.ScanBackward(_rows, _rowWords, _height, y, x, goalPos);
        }
        else
        {
            int goalPos = gx == x ? gy : -1;
            return dy > 0
                ? BlockMap.ScanForward(_cols, _colWords, _width, x, y, goalPos)
                : BlockMap.ScanBackward(_cols, _colWords, _width, x, y, goalPos);
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
