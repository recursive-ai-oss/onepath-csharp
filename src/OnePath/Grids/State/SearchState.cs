using System;

namespace RecursiveAI.OnePath.Grids;

// State hooks of the open sets and search buffers (docs/grids-state.md). Heaps are written slot by slot in heap order, not
// sorted, so a restored heap pops, and reorders on later pushes, exactly as the original would.

public sealed partial class OpenSet
{
    internal void WriteState(StateWriter w)
    {
        w.I32(_count);
        for (int i = 0; i < _count; i++)
        {
            Entry e = _heap[i];
            w.I32(e.Node);
            w.F64(e.F);
            w.F64(e.G);
        }
    }

    internal void ReadState(ref StateReader r)
    {
        int n = r.Count(_position.Length, 20, "the open entry count");
        Clear();
        if (_heap.Length < n)
        {
            _heap = new Entry[Math.Max(64, n)];
        }
        for (int i = 0; i < n; i++)
        {
            int node = r.Node(_position.Length, "open entry node");
            double f = r.NotNaN("an open entry's F");
            double g = r.NotNaN("an open entry's G");
            if (_position[node] >= 0)
            {
                throw r.Fail($"node {node} is in the open list twice.");
            }
            _heap[i] = new Entry { F = f, G = g, Node = node };
            _position[node] = i;
            _count = i + 1; // keeps Clear() able to undo a partial read
        }
    }
}

public sealed partial class ReplanOpenSet
{
    internal void WriteState(StateWriter w)
    {
        w.I32(_count);
        for (int i = 0; i < _count; i++)
        {
            int node = _heap[i];
            w.I32(node);
            w.F64(_key1[node]);
            w.F64(_key2[node]);
        }
    }

    internal void ReadState(ref StateReader r)
    {
        int n = r.Count(_position.Length, 20, "the queue entry count");
        Clear();
        for (int i = 0; i < n; i++)
        {
            int node = r.Node(_position.Length, "queue entry node");
            double k1 = r.NotNaN("a queue key");
            double k2 = r.NotNaN("a queue key");
            if (_position[node] >= 0)
            {
                throw r.Fail($"node {node} is in the queue twice.");
            }
            _heap[i] = node;
            _key1[node] = k1;
            _key2[node] = k2;
            _position[node] = i;
            _count = i + 1;
        }
    }
}

public sealed partial class GridSearchBuffers
{
    // Writes the search in progress over the first nodeCount nodes: nodes seen by the current search in index order (index, closed flag,
    // g, parent), the open heap, and the recorded expansion list. Marks are written relative to the current search, so the
    // epoch counter itself (which never affects a result) is not part of the state.
    internal void WriteState(StateWriter w, int nodeCount)
    {
        uint b = _base;
        int seen = 0;
        for (int i = 0; i < nodeCount; i++)
        {
            if (Mark[i] >= b)
            {
                seen++;
            }
        }
        w.I32(seen);
        for (int i = 0; i < nodeCount; i++)
        {
            uint m = Mark[i];
            if (m >= b)
            {
                w.I32(i);
                w.U8(m == b ? (byte)0 : (byte)1);
                w.F64(G[i]);
                w.I32(Parent[i]);
            }
        }
        Open.WriteState(w);
        w.I32(_expandedCount);
        for (int i = 0; i < _expandedCount; i++)
        {
            w.I32(_expanded[i]);
        }
    }

    // True when every visited node's parent is -1 or another visited node that costs no more, and the links contain no cycle, so that
    // following them from any node ends (path extraction walks them). Valid searches always satisfy this; a corrupt state may not.
    internal bool ParentLinksAcyclic(int nodeCount)
    {
        uint b = _base;
        var colour = new byte[nodeCount]; // 0 not seen, 1 on the walk being made, 2 known to end
        for (int i = 0; i < nodeCount; i++)
        {
            if (Mark[i] < b || colour[i] == 2)
            {
                continue;
            }
            int n = i;
            while (n >= 0 && colour[n] == 0)
            {
                if (Mark[n] < b)
                {
                    return false;
                }
                int parent = Parent[n];
                if (parent >= 0 && (Mark[parent] < b || !(G[parent] <= G[n])))
                {
                    // A parent costs no more than its child. Later relaxations only ever pick a strictly cheaper parent, so this
                    // keeps the links cycle-free for the rest of the search, not just now.
                    return false;
                }
                colour[n] = 1;
                n = Parent[n];
            }
            if (n >= 0 && colour[n] == 1)
            {
                return false;
            }
            for (int m = i; m >= 0 && colour[m] == 1; m = Parent[m])
            {
                colour[m] = 2;
            }
        }
        return true;
    }

    internal void ReadState(ref StateReader r, int nodeCount)
    {
        BeginSearch(nodeCount);
        uint b = _base;
        int seen = r.Count(nodeCount, 17, "the number of visited nodes");
        int previous = -1;
        for (int k = 0; k < seen; k++)
        {
            int i = r.Node(nodeCount, "visited node");
            if (i <= previous)
            {
                throw r.Fail($"visited nodes must be in increasing order but {i} follows {previous}.");
            }
            previous = i;
            byte closed = r.U8();
            if (closed > 1)
            {
                throw r.Fail($"a closed flag holds {closed}; it must be 0 or 1.");
            }
            double g = r.NotNaN("a path cost");
            int parent = r.I32();
            if (parent < -1 || parent >= nodeCount)
            {
                throw r.Fail($"parent {parent} of node {i} is outside the {nodeCount} nodes.");
            }
            Mark[i] = b + closed;
            G[i] = g;
            Parent[i] = parent;
        }
        Open.ReadState(ref r);
        int e = r.Count(int.MaxValue, 4, "the expansion count");
        _expandedCount = 0;
        for (int k = 0; k < e; k++)
        {
            RecordExpanded(r.Node(nodeCount, "expanded node"));
        }
    }
}
