using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// The open list of a best-first search: an indexed 4-ary min-heap over node indices with decrease-key.
/// The order is fully specified, so every platform pops nodes in the same sequence:
/// <list type="number">
/// <item>smaller <c>f</c> first;</item>
/// <item>on equal <c>f</c>, larger <c>g</c> first (deeper nodes, which tends to reach the goal sooner);</item>
/// <item>on equal <c>g</c>, smaller node index first.</item>
/// </list>
/// Keys are compared as IEEE doubles with <c>==</c> and <c>&lt;</c>; NaN keys are rejected. A node is in the
/// heap at most once, so the order never depends on insertion history. The arity is an implementation
/// detail and does not affect the pop order.
/// </summary>
public sealed partial class OpenSet
{
    private struct Entry
    {
        public double F;
        public double G;
        public int Node;
    }

    private readonly int[] _position; // heap slot of each node, or -1
    private Entry[] _heap = new Entry[64];
    private int _count;

    /// <summary>Creates an empty open set for nodes 0..<paramref name="nodeCapacity"/>-1.</summary>
    /// <param name="nodeCapacity">Number of distinct node indices; not negative.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="nodeCapacity"/> is negative.</exception>
    public OpenSet(int nodeCapacity)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(nodeCapacity);
        _position = new int[nodeCapacity];
        Array.Fill(_position, -1);
    }

    /// <summary>Number of nodes in the set.</summary>
    public int Count => _count;

    /// <summary>Number of distinct node indices this set accepts.</summary>
    public int NodeCapacity => _position.Length;

    /// <summary>True if <paramref name="node"/> is in the set. Indices outside the capacity are never in it.</summary>
    /// <param name="node">Node index.</param>
    public bool Contains(int node) => (uint)node < (uint)_position.Length && _position[node] >= 0;

    /// <summary>Empties the set in time proportional to its size.</summary>
    public void Clear()
    {
        for (int i = 0; i < _count; i++)
        {
            _position[_heap[i].Node] = -1;
        }
        _count = 0;
    }

    /// <summary>Inserts a node, or replaces its keys if it is already present (moving it up or down as needed).</summary>
    /// <param name="node">Node index, below <see cref="NodeCapacity"/>.</param>
    /// <param name="f">Priority (cost so far plus estimate); not NaN.</param>
    /// <param name="g">Cost so far, the first tie-break; not NaN.</param>
    /// <exception cref="ArgumentOutOfRangeException">The node is outside the capacity.</exception>
    /// <exception cref="ArgumentException">A key is NaN.</exception>
    public void Push(int node, double f, double g)
    {
        if ((uint)node >= (uint)_position.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(node), node, "Node index is outside the open set's capacity.");
        }
        if (double.IsNaN(f) || double.IsNaN(g))
        {
            throw new ArgumentException("Open set keys must not be NaN.", nameof(f));
        }
        var e = new Entry { F = f, G = g, Node = node };
        int slot = _position[node];
        if (slot < 0)
        {
            if (_count == _heap.Length)
            {
                Array.Resize(ref _heap, _heap.Length * 2);
            }
            slot = _count++;
            SiftUp(slot, e);
            return;
        }
        if (Less(in e, in _heap[slot]))
        {
            SiftUp(slot, e);
        }
        else
        {
            SiftDown(slot, e);
        }
    }

    /// <summary>The node that <see cref="Pop"/> would return.</summary>
    /// <exception cref="InvalidOperationException">The set is empty.</exception>
    public int PeekNode()
    {
        if (_count == 0)
        {
            throw new InvalidOperationException("The open set is empty.");
        }
        return _heap[0].Node;
    }

    /// <summary>The <c>f</c> key of the node that <see cref="Pop"/> would return (bidirectional searches compare the two open sets' best keys).</summary>
    /// <exception cref="InvalidOperationException">The set is empty.</exception>
    public double PeekF()
    {
        if (_count == 0)
        {
            throw new InvalidOperationException("The open set is empty.");
        }
        return _heap[0].F;
    }

    /// <summary>Removes and returns the best node.</summary>
    /// <param name="node">The node.</param>
    /// <param name="f">Its priority.</param>
    /// <param name="g">Its cost so far.</param>
    /// <returns>False if the set was empty.</returns>
    public bool Pop(out int node, out double f, out double g)
    {
        if (_count == 0)
        {
            node = -1;
            f = g = 0.0;
            return false;
        }
        Entry top = _heap[0];
        node = top.Node;
        f = top.F;
        g = top.G;
        _position[node] = -1;
        _count--;
        if (_count > 0)
        {
            SiftDown(0, _heap[_count]);
        }
        return true;
    }

    // The node in heap slot i (0 <= i < Count); state validation only.
    internal int NodeAt(int slot) => _heap[slot].Node;

    private static bool Less(in Entry a, in Entry b)
    {
        if (a.F != b.F)
        {
            return a.F < b.F;
        }
        if (a.G != b.G)
        {
            return a.G > b.G;
        }
        return a.Node < b.Node;
    }

    private void SiftUp(int slot, Entry e)
    {
        while (slot > 0)
        {
            int parent = (slot - 1) >> 2;
            if (!Less(in e, in _heap[parent]))
            {
                break;
            }
            _heap[slot] = _heap[parent];
            _position[_heap[slot].Node] = slot;
            slot = parent;
        }
        _heap[slot] = e;
        _position[e.Node] = slot;
    }

    private void SiftDown(int slot, Entry e)
    {
        int count = _count;
        while (true)
        {
            int first = (slot << 2) + 1;
            if (first >= count)
            {
                break;
            }
            int best = first;
            int last = Math.Min(first + 4, count);
            for (int c = first + 1; c < last; c++)
            {
                if (Less(in _heap[c], in _heap[best]))
                {
                    best = c;
                }
            }
            if (!Less(in _heap[best], in e))
            {
                break;
            }
            _heap[slot] = _heap[best];
            _position[_heap[slot].Node] = slot;
            slot = best;
        }
        _heap[slot] = e;
        _position[e.Node] = slot;
    }
}
