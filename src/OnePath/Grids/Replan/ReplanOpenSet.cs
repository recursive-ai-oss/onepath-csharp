using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// An indexed binary min-heap over node indices with two-part keys, the open list of LPA* and D* Lite
/// (Koenig &amp; Likhachev 2002, "U"): insert, change the key of, remove and peek any node.
/// <para>
/// Order (normative): smaller first key, then smaller second key, then smaller node index; keys within a relative 1e-11 of each other
/// count as equal (see <c>Less</c>). The node index makes the order total in practice, so the pop order does not depend on insertion history.
/// </para>
/// </summary>
public sealed partial class ReplanOpenSet
{
    private readonly int[] _heap;
    private readonly int[] _position; // index into _heap, or -1 when the node is not queued
    private readonly double[] _key1;
    private readonly double[] _key2;
    private int _count;

    /// <summary>Creates an empty queue for nodes 0..<paramref name="nodeCount"/>-1.</summary>
    /// <param name="nodeCount">Number of nodes; at least 1.</param>
    /// <exception cref="ArgumentOutOfRangeException">The count is below 1.</exception>
    public ReplanOpenSet(int nodeCount)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(nodeCount, 1);
        _heap = new int[nodeCount];
        _position = new int[nodeCount];
        _key1 = new double[nodeCount];
        _key2 = new double[nodeCount];
        Array.Fill(_position, -1);
    }

    /// <summary>Number of queued nodes.</summary>
    public int Count => _count;

    /// <summary>True if the node is queued.</summary>
    /// <param name="node">The node.</param>
    /// <returns>True when queued.</returns>
    public bool Contains(int node) => _position[node] >= 0;

    /// <summary>The node with the smallest key.</summary>
    /// <exception cref="InvalidOperationException">The open list is empty.</exception>
    public int Top => _count > 0 ? _heap[0] : throw new InvalidOperationException("The open list is empty.");

    /// <summary>The first key of the top node, or +∞ when the queue is empty.</summary>
    public double TopKey1 => _count > 0 ? _key1[_heap[0]] : double.PositiveInfinity;

    /// <summary>The second key of the top node, or +∞ when the queue is empty.</summary>
    public double TopKey2 => _count > 0 ? _key2[_heap[0]] : double.PositiveInfinity;

    /// <summary>The stored first key of an open listd node.</summary>
    /// <param name="node">A queued node.</param>
    /// <returns>The key.</returns>
    public double Key1(int node) => _key1[node];

    /// <summary>The stored second key of an open listd node.</summary>
    /// <param name="node">A queued node.</param>
    /// <returns>The key.</returns>
    public double Key2(int node) => _key2[node];

    /// <summary>Queues the node, or changes its keys if it is queued already.</summary>
    /// <param name="node">The node.</param>
    /// <param name="key1">First key (compared first).</param>
    /// <param name="key2">Second key.</param>
    public void Set(int node, double key1, double key2)
    {
        _key1[node] = key1;
        _key2[node] = key2;
        int p = _position[node];
        if (p < 0)
        {
            p = _count++;
            _heap[p] = node;
            _position[node] = p;
            SiftUp(p);
            return;
        }
        SiftUp(p);
        SiftDown(_position[node]);
    }

    /// <summary>Removes the node if it is queued.</summary>
    /// <param name="node">The node.</param>
    /// <returns>True if it was queued.</returns>
    public bool Remove(int node)
    {
        int p = _position[node];
        if (p < 0)
        {
            return false;
        }
        _position[node] = -1;
        int last = --_count;
        if (p != last)
        {
            int moved = _heap[last];
            _heap[p] = moved;
            _position[moved] = p;
            SiftUp(p);
            SiftDown(_position[moved]);
        }
        return true;
    }

    /// <summary>Empties the queue.</summary>
    public void Clear()
    {
        for (int i = 0; i < _count; i++)
        {
            _position[_heap[i]] = -1;
        }
        _count = 0;
    }

    // Keys are sums of doubles added in different orders, so keys that are equal on paper can differ in the last bits and would
    // be ordered by that noise instead of by the second key. Two keys within a relative 1e-11 therefore count as equal.
    // The comparison is deterministic, so every platform pops the same node.
    internal const double Relative = 1e-11;

    internal static double Slack(double a) => double.IsInfinity(a) ? 0.0 : Relative * Math.Max(1.0, Math.Abs(a));

    private bool Less(int a, int b)
    {
        double a1 = _key1[a];
        double b1 = _key1[b];
        double d1 = a1 - b1;
        if (a1 != b1 && !(Math.Abs(d1) <= Slack(a1)))
        {
            return a1 < b1;
        }
        double a2 = _key2[a];
        double b2 = _key2[b];
        double d2 = a2 - b2;
        if (a2 != b2 && !(Math.Abs(d2) <= Slack(a2)))
        {
            return a2 < b2;
        }
        return a < b;
    }

    private void SiftUp(int p)
    {
        int node = _heap[p];
        while (p > 0)
        {
            int parent = (p - 1) >> 1;
            int pn = _heap[parent];
            if (!Less(node, pn))
            {
                break;
            }
            _heap[p] = pn;
            _position[pn] = p;
            p = parent;
        }
        _heap[p] = node;
        _position[node] = p;
    }

    private void SiftDown(int p)
    {
        int node = _heap[p];
        while (true)
        {
            int child = (p << 1) + 1;
            if (child >= _count)
            {
                break;
            }
            if (child + 1 < _count && Less(_heap[child + 1], _heap[child]))
            {
                child++;
            }
            int cn = _heap[child];
            if (!Less(cn, node))
            {
                break;
            }
            _heap[p] = cn;
            _position[cn] = p;
            p = child;
        }
        _heap[p] = node;
        _position[node] = p;
    }
}
