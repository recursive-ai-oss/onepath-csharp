using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePath.Grids;

// Save and restore of a begun jump point search (GridSearch, specialised engines; docs/grids-state.md).
//
// Written: the visited cells with their exact cost pair, parent, arrival direction and closed flag, then the open list exactly as it
// lies (heap array, same-f stack, level), stale entries included. Rebuilding it from the open cells would not do: entries of the
// current f level pop last-in first-out, so the order among equal-f entries depends on the history of the pushes.
internal sealed partial class OctileJump<TFinder>
{
    private byte[] _listed = []; // scratch of ReadState: which cells have an open-list entry
    private const int RecordBytes = 17; // i32 cell, i32 a, i32 b, i32 parent, u8 flags (bit 0 closed, bits 1-3 direction of the last move)
    private const int FixedBytes = 2 + (7 * 4); // status, zero flag, then start, goal, expanded, iterations, pathA, pathB, visited

    public int Start => _status == SearchStatus.NotStarted ? -1 : _start;

    public int Goal => _status == SearchStatus.NotStarted ? -1 : _core.Goal;

    private int CountVisited()
    {
        if (_status == SearchStatus.NotStarted)
        {
            return 0;
        }
        uint openBase = _core.OpenBase;
        int count = 0;
        SearchNode[] nodes = _core.Nodes;
        for (int i = 0; i < nodes.Length; i++)
        {
            if (nodes[i].Tag >= openBase)
            {
                count++;
            }
        }
        return count;
    }

    /// <summary>Bytes <see cref="WriteState"/> writes.</summary>
    public int StateLength() => FixedBytes + (CountVisited() * RecordBytes) + HeapBytes();

    private int HeapBytes() => 8 + 4 + (_core.Heap.HeapCount * 16) + 4 + (_core.Heap.StackCount * 4);

    /// <summary>Writes the search into <paramref name="d"/> (at least <see cref="StateLength"/> bytes); returns the count.</summary>
    public int WriteState(Span<byte> d)
    {
        int visited = CountVisited();
        int p = 0;
        d[p++] = (byte)_status;
        d[p++] = _core.UseEstimate ? (byte)0 : (byte)1;
        Put(d, ref p, Start);
        Put(d, ref p, Goal);
        Put(d, ref p, _expandedCount);
        Put(d, ref p, _iterations);
        Put(d, ref p, _pathA);
        Put(d, ref p, _pathB);
        Put(d, ref p, visited);
        if (visited > 0)
        {
            uint openBase = _core.OpenBase;
            SearchNode[] nodes = _core.Nodes;
            for (int i = 0; i < nodes.Length; i++)
            {
                uint t = nodes[i].Tag;
                if (t < openBase)
                {
                    continue;
                }
                Put(d, ref p, i);
                Put(d, ref p, nodes[i].A);
                Put(d, ref p, nodes[i].B);
                Put(d, ref p, nodes[i].Parent);
                d[p++] = (byte)(t & 15u);
            }
        }
        FastHeap heap = _core.Heap;
        BinaryPrimitives.WriteInt64LittleEndian(d.Slice(p, 8), BitConverter.DoubleToInt64Bits(heap.Level));
        p += 8;
        Put(d, ref p, heap.HeapCount);
        for (int i = 0; i < heap.HeapCount; i++)
        {
            HeapItem e = heap.HeapAt(i);
            BinaryPrimitives.WriteInt64LittleEndian(d.Slice(p, 8), BitConverter.DoubleToInt64Bits(e.F));
            BinaryPrimitives.WriteUInt64LittleEndian(d.Slice(p + 8, 8), e.T);
            p += 16;
        }
        Put(d, ref p, heap.StackCount);
        for (int i = 0; i < heap.StackCount; i++)
        {
            Put(d, ref p, heap.StackAt(i));
        }
        return p;
    }

    private static void Put(Span<byte> d, ref int p, int v)
    {
        BinaryPrimitives.WriteInt32LittleEndian(d.Slice(p, 4), v);
        p += 4;
    }

    /// <summary>
    /// Reads a search written by <see cref="WriteState"/> into this engine. Everything a step relies on is checked: cells in range
    /// and walkable, every parent a closed visited cell one valid hop back with the cost pair that hop gives, the goal closed exactly
    /// when the search found it, the counters equal to what the closed cells imply. The engine is unusable after a failure until
    /// begun or restored again.
    /// </summary>
    public void ReadState(ref StateReader r, byte[] cells)
    {
        _status = SearchStatus.NotStarted;
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
        SearchNode[] nodes = _core.Nodes;
        int count = nodes.Length;
        bool started = status != (byte)SearchStatus.NotStarted;
        if (started ? ((uint)start >= (uint)count || (uint)goal >= (uint)count) : (start != -1 || goal != -1))
        {
            throw r.Fail($"start {start} or goal {goal} is outside the {count} cells.");
        }
        if (expanded < 0 || iterations < 0 || pathA < 0 || pathB < 0)
        {
            throw r.Fail("a counter or the path cost is negative.");
        }
        int visited = r.Count(count, RecordBytes, "the number of visited cells");
        if (!started)
        {
            if (visited != 0 || expanded != 0 || iterations != 0 || pathA != 0 || pathB != 0)
            {
                throw r.Fail("a search that was not begun has progress.");
            }
            r.End();
            return;
        }
        if (visited == 0)
        {
            throw r.Fail("a begun search visited no cell.");
        }
        if (cells[start] == AreaCosts.Wall || cells[goal] == AreaCosts.Wall)
        {
            throw r.Fail("the start or goal cell is a wall.");
        }
        NextGeneration();
        uint openBase = _core.OpenBase;
        int previous = -1;
        int closed = 0;
        int open = 0;
        for (int k = 0; k < visited; k++)
        {
            int cell = r.I32();
            int a = r.I32();
            int b = r.I32();
            int parent = r.I32();
            byte flags = r.U8();
            if ((uint)cell >= (uint)count || cell <= previous)
            {
                throw r.Fail($"visited cells must be in increasing order inside the grid but {cell} follows {previous}.");
            }
            previous = cell;
            if (cells[cell] == AreaCosts.Wall)
            {
                throw r.Fail($"visited cell {cell} is a wall.");
            }
            if (a < 0 || b < 0 || flags > 15 || parent < -1 || parent >= count)
            {
                throw r.Fail($"visited cell {cell} has a negative cost, an undefined flag or a parent outside the grid.");
            }
            if ((flags & 1) != 0)
            {
                closed++;
            }
            else
            {
                open++;
            }
            nodes[cell].A = a;
            nodes[cell].B = b;
            nodes[cell].Parent = parent;
            nodes[cell].Tag = openBase | flags;
        }
        // Second pass: every visited cell but the start hangs from a closed visited cell by a valid hop, which also rules out cycles
        // (a hop of at least one step makes the cost pair's total strictly larger along every link).
        bool startSeen = false;
        for (int i = 0; i < count; i++)
        {
            uint t = nodes[i].Tag;
            if (t < openBase)
            {
                continue;
            }
            if (i == start)
            {
                startSeen = true;
                if (nodes[i].A != 0 || nodes[i].B != 0 || nodes[i].Parent != -1 || ((t & 1u) == 0 && (visited != 1 || iterations != 0)))
                {
                    throw r.Fail("the start cell must have cost 0, no parent, and be closed unless the search has not popped anything yet.");
                }
                continue;
            }
            int parent = nodes[i].Parent;
            if (parent < 0 || parent == i || nodes[parent].Tag < openBase || (nodes[parent].Tag & 1u) == 0)
            {
                throw r.Fail($"cell {i} has a parent that is not a closed visited cell.");
            }
            int dy = (i / _w) - (parent / _w);
            int dx = (i - ((i / _w) * _w)) - (parent - ((parent / _w) * _w));
            int ax = dx < 0 ? -dx : dx;
            int ay = dy < 0 ? -dy : dy;
            int min = ax < ay ? ax : ay;
            int max = ax < ay ? ay : ax;
            if ((long)nodes[i].A != (long)nodes[parent].A + (max - min) || (long)nodes[i].B != (long)nodes[parent].B + min)
            {
                throw r.Fail($"the cost of cell {i} does not follow from its parent's.");
            }
            int d = (int)((t >> 1) & 7u);
            int ddx = OctileMap.Dx[d];
            int ddy = OctileMap.Dy[d];
            if ((ddx != 0 && ddx != Math.Sign(dx)) || (ddy != 0 && ddy != Math.Sign(dy)))
            {
                throw r.Fail($"the arrival direction of cell {i} does not point along its hop.");
            }
        }
        if (!startSeen)
        {
            throw r.Fail("the start cell is not among the visited cells.");
        }
        uint goalTag = nodes[goal].Tag;
        bool goalClosed = goalTag >= openBase && (goalTag & 1u) != 0;
        bool found = status == (byte)SearchStatus.Found;
        if (goalClosed != found)
        {
            throw r.Fail(found ? "a found search must have the goal closed." : "only a found search may have the goal closed.");
        }
        if (found && (nodes[goal].A != pathA || nodes[goal].B != pathB))
        {
            throw r.Fail("the path cost does not match the goal's cost.");
        }
        if (!found && (pathA != 0 || pathB != 0))
        {
            throw r.Fail("an unfinished search has no path cost.");
        }
        if (status == (byte)SearchStatus.NotFound && open != 0)
        {
            throw r.Fail("a search that ran out of nodes has open cells.");
        }
        if (iterations != closed || expanded != iterations - (found ? 1 : 0))
        {
            throw r.Fail("the counters do not match the closed cells.");
        }
        // The open list as saved. Entries may name closed cells (stale routes the search skips) but only visited ones, every open
        // cell must have an entry, and the heap array must be a heap, or pops would come in an order the search never produces.
        JumpCore core = _core;
        double level = r.F64();
        FastHeap heap = core.Heap;
        heap.BeginLoad(level);
        if (_listed.Length < count)
        {
            _listed = new byte[count];
        }
        Array.Clear(_listed, 0, count);
        int heapCount = r.Count(1 << 28, 16, "the open heap entry count");
        for (int i = 0; i < heapCount; i++)
        {
            double f = r.NotNaN("an open entry's f");
            ulong t = (ulong)r.I64();
            int node = (int)(uint)t;
            if ((uint)node >= (uint)count || nodes[node].Tag < openBase)
            {
                throw r.Fail($"open entry {node} is not a visited cell.");
            }
            _listed[node] = 1;
            heap.LoadHeap(f, t);
            if (i > 0 && FastHeap.Less(heap.HeapAt(i), heap.HeapAt((i - 1) >> 2)))
            {
                throw r.Fail("the open entries are not in heap order.");
            }
        }
        int stackCount = r.Count(1 << 28, 4, "the open stack entry count");
        if (stackCount > 0 && double.IsNaN(level))
        {
            throw r.Fail("the open stack has entries but no level.");
        }
        for (int i = 0; i < stackCount; i++)
        {
            int node = r.I32();
            if ((uint)node >= (uint)count || nodes[node].Tag < openBase)
            {
                throw r.Fail($"open entry {node} is not a visited cell.");
            }
            _listed[node] = 1;
            heap.LoadStack(node);
        }
        for (int i = 0; i < count; i++)
        {
            uint t = nodes[i].Tag;
            if (t >= openBase && (t & 1u) == 0 && _listed[i] == 0)
            {
                throw r.Fail($"open cell {i} has no entry in the open list.");
            }
        }
        if (status == (byte)SearchStatus.NotFound && heap.Count != 0)
        {
            throw r.Fail("a search that ran out of nodes has open entries.");
        }
        _expanded.Clear();
        core.UseEstimate = !zero;
        core.Goal = goal;
        core.GoalY = goal / _w;
        core.GoalX = goal - (core.GoalY * _w);
        _start = start;
        _status = (SearchStatus)status;
        _expandedCount = expanded;
        _iterations = iterations;
        _pathA = pathA;
        _pathB = pathB;
        r.End();
    }
}
