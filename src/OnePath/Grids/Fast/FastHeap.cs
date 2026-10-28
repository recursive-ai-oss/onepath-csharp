using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Per-node record of the octile searches: the cost so far as an exact pair (straight steps, diagonal steps), the parent node,
/// and a tag holding the search generation, the arrival direction and the closed flag.
/// <para>
/// Costs are kept as integer pairs because the cost of a path is <c>a + b·√2</c>: two paths with equal pairs have identical
/// double keys, so ties between equally good nodes are exact instead of depending on rounding noise, and the tie-break
/// (larger cost so far first) is applied where it was meant to be. Distinct pairs differ by at least about 1e-6 for any
/// grid that fits in memory, far above double rounding, so ordering by the double value is exact too.
/// </para>
/// </summary>
internal struct SearchNode
{
    public int A;
    public int B;
    public int Parent;

    /// <summary><c>generation &lt;&lt; 4 | direction &lt;&lt; 1 | closed</c>.</summary>
    public uint Tag;
}

/// <summary>
/// An open-list entry: <c>F</c> and a tie key <c>T</c> packing the cost so far (rounded to float32, which is IEEE-exact, so the
/// tie-break is the same everywhere) and the node index, so that a smaller <c>T</c> means a larger cost so far, then a smaller node index.
/// </summary>
internal struct HeapItem
{
    public double F;
    public ulong T;

    public readonly int Node => (int)(uint)T;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ulong Tie(float g, int node) => ((ulong)~BitConverter.SingleToUInt32Bits(g) << 32) | (uint)node;
}

/// <summary>
/// The open list of the octile searches: a 4-ary min-heap without decrease-key (a node whose cost improves is pushed again and
/// the stale entry is skipped when it pops, because the node is closed by then). Order, fully specified: smaller <c>F</c>
/// first; on equal <c>F</c> larger <c>G</c> first; on equal <c>G</c> smaller node index first.
/// </summary>
internal sealed class FastHeap
{
    private HeapItem[] _h = new HeapItem[256];
    private int _n;

    // Entries whose F equals the F of the entry popped last (the current level) go on a stack instead of the heap: with a consistent
    // estimate a successor often has the same F as its parent, and a stack push or pop costs a few instructions.
    private int[] _stack = new int[256];
    private int _sn;
    private double _level = double.NaN;

    public int Count => _n + _sn;

    public void Clear()
    {
        _n = 0;
        _sn = 0;
        _level = double.NaN;
    }

    // State access for GridSearch's save and restore. The pop order depends on how the entries were pushed (the current level is a
    // stack), so a saved search keeps the heap array, the stack and the level exactly as they lie.
    internal int HeapCount => _n;

    internal int StackCount => _sn;

    internal double Level => _level;

    internal HeapItem HeapAt(int i) => _h[i];

    internal int StackAt(int i) => _stack[i];

    internal void BeginLoad(double level)
    {
        Clear();
        _level = level;
    }

    internal void LoadHeap(double f, ulong t)
    {
        if (_n == _h.Length)
        {
            Array.Resize(ref _h, _h.Length * 2);
        }
        _h[_n++] = new HeapItem { F = f, T = t };
    }

    internal void LoadStack(int node)
    {
        if (_sn == _stack.Length)
        {
            Array.Resize(ref _stack, _sn * 2);
        }
        _stack[_sn++] = node;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Less(in HeapItem a, in HeapItem b) => a.F < b.F || (a.F == b.F && a.T < b.T);

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public void Push(double f, float g, int node)
    {
        if (f == _level)
        {
            if (_sn == _stack.Length)
            {
                Array.Resize(ref _stack, _sn * 2);
            }
            _stack[_sn++] = node;
            return;
        }
        if (_n == _h.Length)
        {
            Array.Resize(ref _h, _h.Length * 2);
        }
        HeapItem[] h = _h;
        int i = _n++;
        var e = new HeapItem { F = f, T = HeapItem.Tie(g, node) };
        while (i > 0)
        {
            int p = (i - 1) >> 2;
            if (!Less(in e, in h[p]))
            {
                break;
            }
            h[i] = h[p];
            i = p;
        }
        h[i] = e;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public bool Pop(out HeapItem top)
    {
        if (_sn > 0 && (_n == 0 || !(_h[0].F < _level)))
        {
            top = new HeapItem { F = _level, T = (uint)_stack[--_sn] };
            return true;
        }
        if (_n == 0)
        {
            top = default;
            return false;
        }
        HeapItem[] h = _h;
        top = h[0];
        _level = top.F;
        int n = --_n;
        if (n > 0)
        {
            // Bottom-up: the last item usually belongs near the bottom, so move the hole down along the smaller children
            // without comparing against it, then sift it up from there.
            HeapItem e = h[n];
            int i = 0;
            while (true)
            {
                int c = (i << 2) + 1;
                if (c >= n)
                {
                    break;
                }
                int best = c;
                int end = c + 4 < n ? c + 4 : n;
                for (int k = c + 1; k < end; k++)
                {
                    if (Less(in h[k], in h[best]))
                    {
                        best = k;
                    }
                }
                h[i] = h[best];
                i = best;
            }
            while (i > 0)
            {
                int parent = (i - 1) >> 2;
                if (!Less(in e, in h[parent]))
                {
                    break;
                }
                h[i] = h[parent];
                i = parent;
            }
            h[i] = e;
        }
        return true;
    }
}

/// <summary>Node record of <see cref="OctileAStar"/>: cost pair, heap slot, and <c>generation &lt;&lt; 4 | arrival direction &lt;&lt; 1 | closed</c>.</summary>
internal struct AStarNode
{
    public int A;
    public int B;
    public int Pos;
    public uint Tag;
}

/// <summary>
/// An indexed 4-ary heap (decrease-key) over <see cref="AStarNode"/> records: the heap slot of each open node lives in its record so
/// moves update it without a second array. Same order as <see cref="FastHeap"/>.
/// </summary>
internal sealed class IndexedHeap
{
    private HeapItem[] _h = new HeapItem[256];
    private int _n;
    private readonly AStarNode[] _nodes;

    public IndexedHeap(AStarNode[] nodes) => _nodes = nodes;

    public int Count => _n;

    public void Clear() => _n = 0;

    /// <summary>Inserts a node that is not in the heap.</summary>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public void Insert(double f, float g, int node)
    {
        if (_n == _h.Length)
        {
            Array.Resize(ref _h, _h.Length * 2);
        }
        SiftUp(_n++, new HeapItem { F = f, T = HeapItem.Tie(g, node) });
    }

    /// <summary>Lowers the keys of a node that is in the heap at <paramref name="slot"/>.</summary>
    public void Decrease(int slot, double f, float g, int node) => SiftUp(slot, new HeapItem { F = f, T = HeapItem.Tie(g, node) });

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void SiftUp(int i, HeapItem e)
    {
        HeapItem[] h = _h;
        AStarNode[] nodes = _nodes;
        while (i > 0)
        {
            int p = (i - 1) >> 2;
            if (!FastHeap.Less(in e, in h[p]))
            {
                break;
            }
            h[i] = h[p];
            nodes[h[i].Node].Pos = i;
            i = p;
        }
        h[i] = e;
        nodes[e.Node].Pos = i;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public bool Pop(out HeapItem top)
    {
        if (_n == 0)
        {
            top = default;
            return false;
        }
        HeapItem[] h = _h;
        AStarNode[] nodes = _nodes;
        top = h[0];
        int n = --_n;
        if (n > 0)
        {
            HeapItem e = h[n];
            int i = 0;
            while (true)
            {
                int c = (i << 2) + 1;
                if (c >= n)
                {
                    break;
                }
                int best = c;
                int end = c + 4 < n ? c + 4 : n;
                for (int k = c + 1; k < end; k++)
                {
                    if (FastHeap.Less(in h[k], in h[best]))
                    {
                        best = k;
                    }
                }
                h[i] = h[best];
                nodes[h[i].Node].Pos = i;
                i = best;
            }
            while (i > 0)
            {
                int parent = (i - 1) >> 2;
                if (!FastHeap.Less(in e, in h[parent]))
                {
                    break;
                }
                h[i] = h[parent];
                nodes[h[i].Node].Pos = i;
                i = parent;
            }
            h[i] = e;
            nodes[e.Node].Pos = i;
        }
        return true;
    }
}
