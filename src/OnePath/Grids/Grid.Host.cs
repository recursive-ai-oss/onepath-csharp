using System;
using System.Buffers.Binary;
using System.Numerics;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

// Queries a deterministic game host asks every tick: a resumable search handle, a content hash, snapping to walkable ground,
// fits for large agents and exact integer path costs.
public sealed partial class Grid
{
    /// <summary>
    /// Creates a resumable A* search (see <see cref="GridSearch"/>) over this grid. It runs the engine <see cref="FindPath(Cell, Cell, GridOptions?)"/>
    /// runs for the same options, so both return the same path.
    /// </summary>
    /// <param name="options">Options (copied); null means <see cref="GridOptions.Default"/>. <see cref="GridOptions.Algorithm"/> must be <see cref="GridAlgorithm.AStar"/>; <see cref="GridOptions.IterationBudget"/> and <see cref="GridOptions.RecordExpansions"/> do not apply (budgets are the <c>Step</c> argument).</param>
    /// <returns>A new search that has not begun.</returns>
    /// <exception cref="ArgumentException">The options select another algorithm.</exception>
    public GridSearch CreateSearch(GridOptions? options = null) => new(this, options ?? DefaultOptions);

    // ---- content hash -------------------------------------------------------------------------------------------------

    private const ulong HashP1 = 0x9E3779B185EBCA87UL;
    private const ulong HashP2 = 0xC2B2AE3D27D4EB4FUL;
    private const ulong HashP3 = 0x165667B19E3779F9UL;

    /// <summary>
    /// A 64-bit hash of what decides search results: the dimensions, every cell's area id and the 255 area multipliers. It does not
    /// include <see cref="Version"/>, <see cref="Space"/>, edit history or any option, so two grids with the same cells and area
    /// costs hash equal wherever and however they were built. Costs O(cells), allocates nothing.
    /// <para>
    /// Definition (normative, stable across platforms and versions of this package; docs/grids.md): feed 64-bit little-endian words
    /// <c>w</c> into <c>h = rotl(h ^ (w · P2), 31) · P1 + P3</c> starting from <c>h = P1 ^ P2</c>, with
    /// <c>P1 = 0x9E3779B185EBCA87</c>, <c>P2 = 0xC2B2AE3D27D4EB4F</c>, <c>P3 = 0x165667B19E3779F9</c>. The words are, in order: width
    /// in the low 32 bits and height in the high 32 bits; the cells row-major, eight per word (the last word zero-padded); each
    /// area multiplier 0..254 as its IEEE binary64 bit pattern. The result is <c>h ^ (h &gt;&gt; 33)</c> multiplied by P2, xor-shifted
    /// right 29, multiplied by P3 and xor-shifted right 32.
    /// </para>
    /// </summary>
    /// <returns>The hash.</returns>
    public ulong ContentHash()
    {
        ulong h = HashP1 ^ HashP2;
        h = HashWord(h, (uint)Width | ((ulong)(uint)Height << 32));
        ReadOnlySpan<byte> cells = _cells;
        int i = 0;
        for (; i + 8 <= cells.Length; i += 8)
        {
            h = HashWord(h, BinaryPrimitives.ReadUInt64LittleEndian(cells[i..]));
        }
        if (i < cells.Length)
        {
            ulong tail = 0;
            for (int k = 0; i + k < cells.Length; k++)
            {
                tail |= (ulong)cells[i + k] << (8 * k);
            }
            h = HashWord(h, tail);
        }
        double[] m = Areas.Multipliers;
        for (int a = 0; a < AreaCosts.Count; a++)
        {
            h = HashWord(h, (ulong)BitConverter.DoubleToInt64Bits(m[a]));
        }
        h ^= h >> 33;
        h *= HashP2;
        h ^= h >> 29;
        h *= HashP3;
        h ^= h >> 32;
        return h;
    }

    private static ulong HashWord(ulong h, ulong w) => (BitOperations.RotateLeft(h ^ (w * HashP2), 31) * HashP1) + HashP3;

    // ---- agents larger than one cell, snapping -------------------------------------------------------------------------

    /// <summary>
    /// True when an agent that occupies <paramref name="agentSize"/> × <paramref name="agentSize"/> cells with <paramref name="cell"/> as its
    /// top-left cell stands on walkable ground only. This is the test <see cref="FindPath(Cell, Cell, GridOptions?)"/> applies to its
    /// start and goal for <see cref="GridOptions.AgentSize"/>, equal to <c>GetClearance(x, y) &gt;= agentSize</c>.
    /// </summary>
    /// <param name="cell">The top-left cell of the footprint.</param>
    /// <param name="agentSize">Footprint side in cells, 1 to 255.</param>
    /// <returns>False when any footprint cell is a wall or outside the grid (and for a cell outside the grid).</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="agentSize"/> is outside 1–255.</exception>
    public bool CanFit(Cell cell, int agentSize)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(agentSize, 1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(agentSize, 255);
        return Contains(cell.X, cell.Y) && ClearanceArray[(cell.Y * Width) + cell.X] >= agentSize;
    }

    /// <summary>
    /// The largest footprint side that fits with <paramref name="cell"/> as its top-left cell (0 for a wall or a cell outside the grid; at most 255).
    /// Same as <see cref="GetClearance(int, int)"/> without the exception.
    /// </summary>
    /// <param name="cell">The cell.</param>
    /// <returns>The side in cells.</returns>
    public int MaxAgentSizeAt(Cell cell) => Contains(cell.X, cell.Y) ? ClearanceArray[(cell.Y * Width) + cell.X] : 0;

    /// <summary>
    /// Finds the walkable cell nearest to <paramref name="around"/> that a search with <paramref name="options"/> would accept as an end
    /// cell: it fits <see cref="GridOptions.AgentSize"/> and <see cref="GridOptions.Blocked"/> does not refuse it. Distance is the
    /// Euclidean distance between cell indexes; ties go to the smaller row, then the smaller column, so the answer does not depend on
    /// the scan order. <paramref name="around"/> may lie outside the grid. Examines rings up to <see cref="GridOptions.NearestRadius"/>.
    /// Allocates nothing (apart from what a <c>Blocked</c> delegate does).
    /// </summary>
    /// <param name="around">The cell to start from.</param>
    /// <param name="nearest">The answer; default when none.</param>
    /// <param name="options">Options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>True when a cell was found within the radius.</returns>
    public bool TryFindNearestWalkable(Cell around, out Cell nearest, GridOptions? options = null)
    {
        GridOptions o = options ?? DefaultOptions;
        byte[] clear = ClearanceArray;
        int size = o.AgentSize;
        Func<Cell, bool>? blocked = o.Blocked;
        long cx = around.X;
        long cy = around.Y;
        // Rings are measured from the query cell; the first ring that can touch the grid is the distance to its rectangle.
        long outX = cx < 0 ? -cx : (cx >= Width ? cx - (Width - 1) : 0);
        long outY = cy < 0 ? -cy : (cy >= Height ? cy - (Height - 1) : 0);
        long first = Math.Max(outX, outY);
        long last = first + Math.Max(Width, Height);
        if (o.NearestRadius != int.MaxValue)
        {
            last = Math.Min(last, o.NearestRadius); // int.MaxValue means the whole grid, even for a query cell 2^31 away
        }
        bool found = false;
        long bestD2 = long.MaxValue;
        long bestX = 0;
        long bestY = 0;
        for (long r = first; r <= last; r++)
        {
            if (found && r * r > bestD2)
            {
                break;
            }
            long x0 = Math.Max(0, cx - r);
            long x1 = Math.Min(Width - 1, cx + r);
            for (int side = 0; side < 2; side++)
            {
                long y = side == 0 ? cy - r : cy + r;
                if (y < 0 || y >= Height || (side == 1 && r == 0))
                {
                    continue;
                }
                for (long x = x0; x <= x1; x++)
                {
                    Probe(x, y);
                }
            }
            long y0 = Math.Max(0, cy - r + 1);
            long y1 = Math.Min(Height - 1, cy + r - 1);
            for (int side = 0; side < 2; side++)
            {
                long x = side == 0 ? cx - r : cx + r;
                if (x < 0 || x >= Width || (side == 1 && r == 0))
                {
                    continue;
                }
                for (long y = y0; y <= y1; y++)
                {
                    Probe(x, y);
                }
            }
        }
        nearest = found ? new Cell((int)bestX, (int)bestY) : default;
        return found;

        void Probe(long x, long y)
        {
            if (clear[(y * Width) + x] < size)
            {
                return;
            }
            long dx = x - cx;
            long dy = y - cy;
            long d2 = (dx * dx) + (dy * dy);
            if (found && (d2 > bestD2 || (d2 == bestD2 && (y > bestY || (y == bestY && x > bestX)))))
            {
                return;
            }
            if (blocked is not null && blocked(new Cell((int)x, (int)y)))
            {
                return;
            }
            found = true;
            bestD2 = d2;
            bestX = x;
            bestY = y;
        }
    }

    /// <summary>
    /// Finds the walkable cell nearest to a world point; see <see cref="TryFindNearestWalkable(Cell, out Cell, GridOptions?)"/>. A point outside
    /// the grid snaps as if it lay in the nearest outside cell.
    /// </summary>
    /// <param name="point">World position within ±2^21.</param>
    /// <param name="nearest">The answer; default when none.</param>
    /// <param name="options">Options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>True when a cell was found.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate lies beyond ±2^21.</exception>
    public bool TryFindNearestWalkable(WorldPoint point, out Cell nearest, GridOptions? options = null)
    {
        GridSpace.CheckCoordinate(point.X, nameof(point));
        GridSpace.CheckCoordinate(point.Y, nameof(point));
        GridSpace s = Space;
        double size = s.SizeOrOne;
        double fx = Math.Floor((point.X - s.OriginX) / size);
        double fy = Math.Floor((point.Y - s.OriginY) / size);
        const double Limit = 1 << 30; // far outside any grid; keeps the cast exact
        fx = Math.Clamp(fx, -Limit, Limit);
        fy = Math.Clamp(fy, -Limit, Limit);
        return TryFindNearestWalkable(new Cell((int)fx, (int)fy), out nearest, options);
    }

    // ---- exact integer costs --------------------------------------------------------------------------------------------

    /// <summary>
    /// The cost of a path of adjacent cells as an exact integer in Q31.32 (the value times 2^32), so a host can compare and sum
    /// costs without floating point. Each step costs the double the default search charges for it: the mean of the two cells' area
    /// multipliers, times √2 for a diagonal step (<c>1.4142135623730951</c>); that double is multiplied by 2^32 (exact) and rounded
    /// half to even; the steps are added as integers. The result is the same on every platform and independent of the order of
    /// anything but the path. The cost of <see cref="GridOptions.StepCost"/> hooks is not included, and the current area table is used.
    /// </summary>
    /// <param name="path">Cells, each adjacent (including diagonally) to the previous one, e.g. <see cref="GridPath.Cells"/> of an A*, JPS+ expanded or replanner path. Fewer than two cells cost 0.</param>
    /// <param name="cost">The cost in 2^-32 units; 0 when the method returns false.</param>
    /// <returns>False when a step is not between adjacent cells, a cell is outside the grid or a wall, or the total overflows 63 bits.</returns>
    public bool TryGetFixedCost(ReadOnlySpan<Cell> path, out long cost)
    {
        cost = 0;
        double[] m = Areas.Multipliers;
        long total = 0;
        for (int i = 1; i < path.Length; i++)
        {
            Cell a = path[i - 1];
            Cell b = path[i];
            if (!Contains(a.X, a.Y) || !Contains(b.X, b.Y))
            {
                return false;
            }
            int dx = b.X - a.X;
            int dy = b.Y - a.Y;
            int adx = dx < 0 ? -dx : dx;
            int ady = dy < 0 ? -dy : dy;
            byte ca = _cells[(a.Y * Width) + a.X];
            byte cb = _cells[(b.Y * Width) + b.X];
            if (adx > 1 || ady > 1 || adx + ady == 0 || ca == AreaCosts.Wall || cb == AreaCosts.Wall)
            {
                return false;
            }
            double step = (m[ca] + m[cb]) * 0.5;
            if (adx + ady == 2)
            {
                step = GridMath.Sqrt2 * step;
            }
            double scaled = step * 4294967296.0;
            if (!(scaled < 4.0e18))
            {
                return false;
            }
            long q = (long)Math.Round(scaled, MidpointRounding.ToEven);
            if (total > long.MaxValue - q)
            {
                return false;
            }
            total += q;
        }
        cost = total;
        return true;
    }
}

public sealed partial class GridPath
{
    /// <summary>
    /// Counts the straight and diagonal steps of the path exactly (the integer pair behind an octile cost <c>straight + diagonal·√2</c>
    /// on a uniform grid).
    /// </summary>
    /// <param name="straight">Steps to a side neighbour.</param>
    /// <param name="diagonal">Steps to a diagonal neighbour.</param>
    /// <returns>False when a step is not between adjacent cells (any-angle and abstract paths); the counts are then 0.</returns>
    public bool TryGetStepCounts(out int straight, out int diagonal)
    {
        straight = 0;
        diagonal = 0;
        for (int i = 1; i < _cells.Length; i++)
        {
            int dx = Math.Abs(_cells[i].X - _cells[i - 1].X);
            int dy = Math.Abs(_cells[i].Y - _cells[i - 1].Y);
            if (dx > 1 || dy > 1 || dx + dy == 0)
            {
                straight = 0;
                diagonal = 0;
                return false;
            }
            if (dx + dy == 1)
            {
                straight++;
            }
            else
            {
                diagonal++;
            }
        }
        return true;
    }
}
