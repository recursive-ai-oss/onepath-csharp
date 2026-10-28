using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePath.Grids;

// Save and restore of a begun octile search (GridSearch, specialised engine; docs/grids-state.md).
//
// Only what decides the rest of the search is written: the visited cells with their exact cost pair, arrival direction and closed
// flag. The open list is the visited cells that are not closed; its pop order is a total order on (f, g, cell), so the heap's
// internal layout never matters and is rebuilt on restore.
internal sealed partial class OctileAStar
{
    private const int RecordBytes = 13; // i32 cell, i32 a, i32 b, u8 flags (bit 0 closed, bits 1-3 arrival direction)
    private const int FixedBytes = 2 + (7 * 4); // status, zero flag, then start, goal, expanded, iterations, pathA, pathB, visited

    internal int Start => ToCellOrMinus(_startInner);

    internal int Goal => ToCellOrMinus(_goal);

    private int ToCellOrMinus(int inner) => _status == SearchStatus.NotStarted ? -1 : ToCell(inner);

    internal void SyncWithGrid() => Refresh();

    private int CountVisited()
    {
        if (_status == SearchStatus.NotStarted)
        {
            return 0;
        }
        uint openBase = _generation << 4;
        int count = 0;
        AStarNode[] nodes = _nodes;
        for (int i = 0; i < nodes.Length; i++)
        {
            uint t = nodes[i].Tag;
            if (t != WallTag && t >= openBase)
            {
                count++;
            }
        }
        return count;
    }

    /// <summary>Bytes <see cref="WriteState"/> writes.</summary>
    internal int StateLength() => FixedBytes + (CountVisited() * RecordBytes);

    /// <summary>Writes the search into <paramref name="d"/> (at least <see cref="StateLength"/> bytes); returns the count.</summary>
    internal int WriteState(Span<byte> d)
    {
        int visited = CountVisited();
        int p = 0;
        d[p++] = (byte)_status;
        d[p++] = _useEstimate ? (byte)0 : (byte)1;
        Put(d, ref p, _status == SearchStatus.NotStarted ? -1 : ToCell(_startInner));
        Put(d, ref p, _status == SearchStatus.NotStarted ? -1 : ToCell(_goal));
        Put(d, ref p, _expandedCount);
        Put(d, ref p, _iterations);
        Put(d, ref p, _pathA);
        Put(d, ref p, _pathB);
        Put(d, ref p, visited);
        if (visited > 0)
        {
            uint openBase = _generation << 4;
            AStarNode[] nodes = _nodes;
            for (int i = 0; i < nodes.Length; i++)
            {
                uint t = nodes[i].Tag;
                if (t == WallTag || t < openBase)
                {
                    continue;
                }
                Put(d, ref p, ToCell(i));
                Put(d, ref p, nodes[i].A);
                Put(d, ref p, nodes[i].B);
                d[p++] = (byte)(t & 15u); // closed bit 0, direction bits 1-3
            }
        }
        return p;
    }

    private static void Put(Span<byte> d, ref int p, int v)
    {
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(p, 4), v);
        p += 4;
    }

    /// <summary>
    /// Reads a search written by <see cref="WriteState"/> into this engine. The engine is unusable after a failure until begun
    /// or restored again.
    /// </summary>
    internal void ReadState(ref StateReader r)
    {
        _status = SearchStatus.NotStarted;
        Refresh();
        byte status = r.U8();
        if (status > (byte)SearchStatus.NotFound)
        {
            throw r.Fail($"search status {status} is not defined.");
        }
        bool zero = r.Bool();
        int start = r.I32();
        int goal = r.I32();
        int expanded = r.I32();
        int iterations = r.I32();
        int pathA = r.I32();
        int pathB = r.I32();
        int cells = _w * _h;
        bool started = status != (byte)SearchStatus.NotStarted;
        if (started ? ((uint)start >= (uint)cells || (uint)goal >= (uint)cells) : (start != -1 || goal != -1))
        {
            throw r.Fail($"start {start} or goal {goal} is outside the {cells} cells.");
        }
        if (expanded < 0 || iterations < 0 || pathA < 0 || pathB < 0)
        {
            throw r.Fail("a counter or the path cost is negative.");
        }
        int visited = r.Count(cells, RecordBytes, "the number of visited cells");
        if (!started)
        {
            if (visited != 0)
            {
                throw r.Fail("a search that was not begun lists visited cells.");
            }
            r.End();
            return;
        }
        if (visited == 0)
        {
            throw r.Fail("a begun search visited no cell.");
        }
        if (_generation >= (uint.MaxValue >> 4) - 2)
        {
            Array.Fill(_nodes, new AStarNode { Tag = WallTag });
            _generation = 0;
            _version = -1;
            Refresh();
        }
        _generation++;
        _expanded.Clear();
        _open.Clear();
        uint openBase = _generation << 4;
        AStarNode[] nodes = _nodes;
        int previous = -1;
        bool startSeen = false;
        for (int k = 0; k < visited; k++)
        {
            int cell = r.I32();
            if ((uint)cell >= (uint)cells || cell <= previous)
            {
                throw r.Fail($"visited cells must be inside the grid and in increasing order but {cell} follows {previous}.");
            }
            previous = cell;
            int a = r.I32();
            int b = r.I32();
            byte flags = r.U8();
            if (flags > 15)
            {
                throw r.Fail($"the flags of cell {cell} hold {flags}; the high bits must be 0.");
            }
            int inner = ToInner(cell);
            if (nodes[inner].Tag == WallTag)
            {
                throw r.Fail($"cell {cell} is visited in the state but is a wall in the grid being restored.");
            }
            if (a < 0 || b < 0)
            {
                throw r.Fail($"cell {cell} has a negative cost count.");
            }
            nodes[inner].A = a;
            nodes[inner].B = b;
            nodes[inner].Tag = openBase | flags;
            startSeen |= cell == start;
        }
        if (!startSeen)
        {
            throw r.Fail("the start cell is not among the visited cells.");
        }
        _startInner = ToInner(start);
        _goal = ToInner(goal);
        _gy = _goal / _stride;
        _gx = _goal - (_gy * _stride);
        _useEstimate = !zero;
        // Second pass: every visited cell but the start has a closed parent one step before it (costs are exact pairs, so the
        // relation is checked exactly); this also rules out cycles, as the pair's total grows along every link.
        for (int i = 0; i < nodes.Length; i++)
        {
            uint t = nodes[i].Tag;
            if (t == WallTag || t < openBase)
            {
                continue;
            }
            if (i == _startInner)
            {
                // The start is closed once the search has popped it; before the first pop it is the only visited cell.
                if (nodes[i].A != 0 || nodes[i].B != 0 || ((t & 1u) == 0 && (visited != 1 || iterations != 0)))
                {
                    throw r.Fail("the start cell must have cost 0 and be closed unless the search has not popped anything yet.");
                }
                continue;
            }
            int dir = (int)((t >> 1) & 7);
            int pr = Parent(i);
            uint pt = nodes[pr].Tag;
            if (pt == WallTag || pt < openBase || (pt & 1u) == 0)
            {
                throw r.Fail($"cell {ToCell(i)} has a parent that is not a closed visited cell.");
            }
            int expectA = nodes[pr].A + (dir < 4 ? 1 : 0);
            int expectB = nodes[pr].B + (dir < 4 ? 0 : 1);
            if (nodes[i].A != expectA || nodes[i].B != expectB)
            {
                throw r.Fail($"the cost of cell {ToCell(i)} does not follow from its parent's.");
            }
        }
        if (status == (byte)SearchStatus.Found)
        {
            uint gt = nodes[_goal].Tag;
            if (gt == WallTag || gt < openBase || (gt & 1u) == 0 || nodes[_goal].A != pathA || nodes[_goal].B != pathB)
            {
                throw r.Fail("a found search must have the goal closed with the path cost it reports.");
            }
        }
        // Rebuild the open list: the visited cells that are not closed.
        _status = (SearchStatus)status;
        for (int i = 0; i < nodes.Length; i++)
        {
            uint t = nodes[i].Tag;
            if (t == WallTag || t < openBase || (t & 1u) != 0)
            {
                continue;
            }
            int y = i / _stride;
            int x = i - (y * _stride);
            double g = nodes[i].A + (nodes[i].B * Sqrt2);
            _open.Insert(Key(nodes[i].A, nodes[i].B, x, y), (float)g, i);
        }
        _expandedCount = expanded;
        _iterations = iterations;
        _pathA = pathA;
        _pathB = pathB;
        r.End();
    }
}
