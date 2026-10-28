using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Walkability of a grid in the forms the octile searches read: for every cell a byte telling which of its eight neighbours are
/// walkable (<see cref="Neighbors"/>), and the walkable cells as bit rows and bit columns padded with a wall all round (the
/// block scanning of Harabor &amp; Grastien 2014, "Improving Jump Point Search"). The padding means a scan always meets a wall before
/// it leaves the array, so the scanning loops need no bounds tests. The map follows the grid's cell edits as they happen.
/// Directions are numbered as in <see cref="JumpRules"/>: 0 north, 1 east, 2 south, 3 west, 4 north-east, 5 south-east, 6 south-west, 7 north-west.
/// </summary>
internal sealed class OctileMap : IGridChangeListener, IDisposable
{
    public static readonly int[] Dx = [0, 1, 0, -1, 1, 1, -1, -1];

    public static readonly int[] Dy = [-1, 0, 1, 0, -1, 1, 1, -1];

    /// <summary>Bits of the neighbour byte a diagonal move needs: its two straight components and the diagonal itself (no corner cutting).</summary>
    public static readonly byte[] DiagonalNeed = [0, 0, 0, 0, 0x13, 0x26, 0x4C, 0x89];

    /// <summary>
    /// <c>Successors[(arrival &lt;&lt; 8) | neighbours]</c>: the directions a node reached by a move of direction <c>arrival</c> searches,
    /// as a bit set, given which neighbours are walkable (<see cref="JumpRules.Successors"/> evaluated once for every case).
    /// </summary>
    public static readonly byte[] Successors = BuildSuccessors();

    private readonly Grid _grid;
    private readonly byte[] _cells;
    private readonly int _w;
    private readonly int _h;

    public OctileMap(Grid grid)
    {
        _grid = grid;
        _cells = grid.CellArray;
        _w = grid.Width;
        _h = grid.Height;
        RowStride = ((_w + 64) >> 6) + 2;
        ColStride = ((_h + 64) >> 6) + 2;
        long rows = (long)(_h + 2) * RowStride;
        long cols = (long)(_w + 2) * ColStride;
        if (rows > int.MaxValue || cols > int.MaxValue)
        {
            throw new ArgumentOutOfRangeException(nameof(grid), "The grid is too large for this search.");
        }
        Rows = new ulong[rows];
        Cols = new ulong[cols];
        RowStopForward = new ulong[rows];
        RowStopBackward = new ulong[rows];
        ColStopForward = new ulong[cols];
        ColStopBackward = new ulong[cols];
        Array.Fill(RowStopForward, ~0UL);
        Array.Fill(RowStopBackward, ~0UL);
        Array.Fill(ColStopForward, ~0UL);
        Array.Fill(ColStopBackward, ~0UL);
        Neighbors = new byte[(long)_w * _h];
        for (int y = 0; y < _h; y++)
        {
            for (int x = 0; x < _w; x++)
            {
                if (IsFree(x, y))
                {
                    Rows[((y + 1) * RowStride) + ((x + 64) >> 6)] |= 1UL << ((x + 64) & 63);
                    Cols[((x + 1) * ColStride) + ((y + 64) >> 6)] |= 1UL << ((y + 64) & 63);
                }
                Neighbors[(y * _w) + x] = ComputeNeighbors(x, y);
            }
        }
        for (int y = 0; y < _h; y++)
        {
            for (int k = 1; k < RowStride - 1; k++)
            {
                StopWord(Rows, RowStopForward, RowStopBackward, RowStride, y + 1, k);
            }
        }
        for (int x = 0; x < _w; x++)
        {
            for (int k = 1; k < ColStride - 1; k++)
            {
                StopWord(Cols, ColStopForward, ColStopBackward, ColStride, x + 1, k);
            }
        }
        grid.AddListener(this);
    }

    public int Width => _w;

    public int Height => _h;

    /// <summary>Words per row of <see cref="Rows"/>.</summary>
    public int RowStride { get; }

    /// <summary>Words per column of <see cref="Cols"/>.</summary>
    public int ColStride { get; }

    /// <summary>Bit rows: row <c>y</c> starts at word <c>(y + 1) * RowStride</c>, cell <c>x</c> is bit <c>x + 64</c>. Set = walkable.</summary>
    public ulong[] Rows { get; }

    /// <summary>Bit columns: column <c>x</c> starts at word <c>(x + 1) * ColStride</c>, cell <c>y</c> is bit <c>y + 64</c>. Set = walkable.</summary>
    public ulong[] Cols { get; }

    /// <summary>
    /// For <see cref="Rows"/>, the cells where an eastward scan stops: walls, and free cells with a forced neighbour (a cell beside the line is free
    /// while the one beside the previous line cell is not). Same layout as <see cref="Rows"/>; padding words are all ones. Precomputed so a scan
    /// step reads one word instead of five.
    /// </summary>
    public ulong[] RowStopForward { get; }

    /// <summary>As <see cref="RowStopForward"/> for a westward scan.</summary>
    public ulong[] RowStopBackward { get; }

    /// <summary>As <see cref="RowStopForward"/> for <see cref="Cols"/>, a southward scan.</summary>
    public ulong[] ColStopForward { get; }

    /// <summary>As <see cref="RowStopForward"/> for <see cref="Cols"/>, a northward scan.</summary>
    public ulong[] ColStopBackward { get; }

    /// <summary>Bit <c>d</c> of <c>Neighbors[y * Width + x]</c> is set when the neighbour in direction <c>d</c> is inside the grid and walkable.</summary>
    public byte[] Neighbors { get; }

    public Grid Grid => _grid;

    private bool IsFree(int x, int y) => (uint)x < (uint)_w && (uint)y < (uint)_h && _cells[(y * _w) + x] != AreaCosts.Wall;

    private byte ComputeNeighbors(int x, int y)
    {
        int m = 0;
        for (int d = 0; d < 8; d++)
        {
            if (IsFree(x + Dx[d], y + Dy[d]))
            {
                m |= 1 << d;
            }
        }
        return (byte)m;
    }

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y)
    {
        ulong rowBit = 1UL << ((x + 64) & 63);
        ulong colBit = 1UL << ((y + 64) & 63);
        int ri = ((y + 1) * RowStride) + ((x + 64) >> 6);
        int ci = ((x + 1) * ColStride) + ((y + 64) >> 6);
        if (IsFree(x, y))
        {
            Rows[ri] |= rowBit;
            Cols[ci] |= colBit;
        }
        else
        {
            Rows[ri] &= ~rowBit;
            Cols[ci] &= ~colBit;
        }
        // The cell is a forced-neighbour cause for the lines beside it and for the next or previous cell along its own line, which may sit in the next or previous word.
        for (int j = y - 1; j <= y + 1; j++)
        {
            if ((uint)j < (uint)_h)
            {
                for (int k = Math.Max(1, ((x + 64) >> 6) - 1); k <= Math.Min(RowStride - 2, ((x + 64) >> 6) + 1); k++)
                {
                    StopWord(Rows, RowStopForward, RowStopBackward, RowStride, j + 1, k);
                }
            }
        }
        for (int i = x - 1; i <= x + 1; i++)
        {
            if ((uint)i < (uint)_w)
            {
                for (int k = Math.Max(1, ((y + 64) >> 6) - 1); k <= Math.Min(ColStride - 2, ((y + 64) >> 6) + 1); k++)
                {
                    StopWord(Cols, ColStopForward, ColStopBackward, ColStride, i + 1, k);
                }
            }
        }
        for (int j = -1; j <= 1; j++)
        {
            for (int i = -1; i <= 1; i++)
            {
                int nx = x + i;
                int ny = y + j;
                if ((uint)nx < (uint)_w && (uint)ny < (uint)_h)
                {
                    Neighbors[(ny * _w) + nx] = ComputeNeighbors(nx, ny);
                }
            }
        }
    }

    /// <summary>Stops following the grid's edits.</summary>
    public void Dispose() => _grid.RemoveListener(this);

    private readonly struct NeighborhoodCells(int neighbors) : IFreeCells
    {
        // The 3x3 block around the cell at (1, 1); everything the pruning rules read lies inside it.
        public bool IsFree(int x, int y)
        {
            int ox = x - 1;
            int oy = y - 1;
            if (ox == 0 && oy == 0)
            {
                return true;
            }
            if (ox < -1 || ox > 1 || oy < -1 || oy > 1)
            {
                return false;
            }
            return (neighbors & (1 << JumpRules.Direction(ox, oy))) != 0;
        }
    }

    private static byte[] BuildSuccessors()
    {
        var table = new byte[8 * 256];
        for (int arrival = 0; arrival < 8; arrival++)
        {
            for (int nb = 0; nb < 256; nb++)
            {
                table[(arrival << 8) | nb] = (byte)JumpRules.Successors(new NeighborhoodCells(nb), 1, 1, Dx[arrival], Dy[arrival]);
            }
        }
        return table;
    }

    // Recomputes the stop words of word k of line `line` (padded index: the line before it is line - 1).
    // A free cell has a forced neighbour in a scan along the line when a cell beside the line is free while the one beside the previous line cell is not
    // (Harabor & Grastien 2011); walls always stop.
    private static void StopWord(ulong[] bits, ulong[] forward, ulong[] backward, int stride, int line, int k)
    {
        int self = line * stride;
        int up = self - stride;
        int down = self + stride;
        ulong w = bits[self + k];
        ulong a = bits[up + k];
        ulong b = bits[down + k];
        ulong aPrev = bits[up + k - 1] >> 63;
        ulong bPrev = bits[down + k - 1] >> 63;
        ulong aNext = bits[up + k + 1] << 63;
        ulong bNext = bits[down + k + 1] << 63;
        forward[self + k] = ~w | (a & ~((a << 1) | aPrev)) | (b & ~((b << 1) | bPrev));
        backward[self + k] = ~w | (a & ~((a >> 1) | aNext)) | (b & ~((b >> 1) | bNext));
    }

    // Scan one line of a bit map forwards (towards larger positions) from the cell at bit position `pos`, which is not included.
    // `self` is the word index of the line start. `stops` holds the stop words of the line (see RowStopForward).
    // Returns the number of steps to the first cell that is the goal (bit goalPos, or -1 for none) or has a forced neighbour, counting the step;
    // 0 if a wall comes first.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ScanForward(ulong[] bits, ulong[] stops, int self, int pos, int goalPos)
    {
        int p0 = pos + 1;
        int k = p0 >> 6;
        ulong mask = ~0UL << (p0 & 63);
        while (true)
        {
            ulong s = stops[self + k] & mask;
            if ((goalPos >> 6) == k)
            {
                s |= (1UL << (goalPos & 63)) & mask;
            }
            if (s != 0)
            {
                int bit = BitOperations.TrailingZeroCount(s);
                // A wall stops the scan without a result; a forced neighbour or the goal is a free cell.
                return ((bits[self + k] >> bit) & 1) == 0 ? 0 : (k << 6) + bit - pos;
            }
            k++;
            mask = ~0UL;
        }
    }

    // The same scan towards smaller positions.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int ScanBackward(ulong[] bits, ulong[] stops, int self, int pos, int goalPos)
    {
        int p0 = pos - 1;
        int k = p0 >> 6;
        ulong mask = ~0UL >> (63 - (p0 & 63));
        while (true)
        {
            ulong s = stops[self + k] & mask;
            if ((goalPos >> 6) == k)
            {
                s |= (1UL << (goalPos & 63)) & mask;
            }
            if (s != 0)
            {
                int bit = 63 - BitOperations.LeadingZeroCount(s);
                return ((bits[self + k] >> bit) & 1) == 0 ? 0 : pos - ((k << 6) + bit);
            }
            k--;
            mask = ~0UL;
        }
    }
}
