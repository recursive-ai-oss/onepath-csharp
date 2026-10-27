using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

public sealed partial class FlowField<TTopology, TCost>
{
    // Layout (docs/grids-state.md): node count, status, settled, repairing flag, goals, the finite costs and the chosen next nodes
    // (both in index order), the open heap in heap order, and while a repair runs the nodes whose direction it will recompute.
    internal void WriteState(StateWriter w)
    {
        w.I32(_dist.Length);
        w.U8((byte)_status);
        w.I32(_settled);
        w.Bool(_repairing);
        w.I32(_goals.Count);
        for (int i = 0; i < _goals.Count; i++)
        {
            w.I32(_goals[i]);
        }
        int finite = 0;
        int chosen = 0;
        for (int i = 0; i < _dist.Length; i++)
        {
            if (!double.IsPositiveInfinity(_dist[i]))
            {
                finite++;
            }
            if (_next[i] >= 0)
            {
                chosen++;
            }
        }
        w.I32(finite);
        for (int i = 0; i < _dist.Length; i++)
        {
            if (!double.IsPositiveInfinity(_dist[i]))
            {
                w.I32(i);
                w.F64(_dist[i]);
            }
        }
        w.I32(chosen);
        for (int i = 0; i < _next.Length; i++)
        {
            if (_next[i] >= 0)
            {
                w.I32(i);
                w.I32(_next[i]);
            }
        }
        _open.WriteState(w);
        if (_repairing)
        {
            w.I32(_touched.Count);
            for (int i = 0; i < _touched.Count; i++)
            {
                w.I32(_touched[i]);
            }
        }
    }

    internal void ReadState(ref StateReader r)
    {
        int n = _dist.Length;
        r.Expect(n, "the node count");
        byte status = r.U8();
        if (status is not ((byte)FlowFieldStatus.Running or (byte)FlowFieldStatus.Complete))
        {
            throw r.Fail($"flow field status {status} is not valid; a field that was saved is running or complete.");
        }
        int settled = r.I32();
        bool repairing = r.Bool();
        if (settled < 0)
        {
            throw r.Fail("the settled count is negative.");
        }
        if (repairing && status != (byte)FlowFieldStatus.Running)
        {
            throw r.Fail("a repair is marked as running on a field that is not running.");
        }
        int goalCount = r.Count(n, 4, "the goal count");
        var goals = new int[goalCount];
        for (int i = 0; i < goalCount; i++)
        {
            goals[i] = r.Node(n, "goal");
            if (i > 0 && goals[i] <= goals[i - 1])
            {
                throw r.Fail("goals must be in strictly increasing order.");
            }
        }
        _status = FlowFieldStatus.NotStarted; // unusable until the whole state has been read
        _goals.Clear();
        _goals.AddRange(goals);
        Array.Fill(_dist, double.PositiveInfinity);
        Array.Fill(_next, -1);
        int finite = r.Count(n, 12, "the number of reached nodes");
        int previous = -1;
        for (int k = 0; k < finite; k++)
        {
            int i = r.Node(n, "reached node");
            if (i <= previous)
            {
                throw r.Fail($"reached nodes must be in increasing order but {i} follows {previous}.");
            }
            previous = i;
            _dist[i] = r.NotNaN("a cost");
        }
        int chosen = r.Count(n, 8, "the number of direction entries");
        previous = -1;
        for (int k = 0; k < chosen; k++)
        {
            int i = r.Node(n, "direction node");
            if (i <= previous)
            {
                throw r.Fail($"direction nodes must be in increasing order but {i} follows {previous}.");
            }
            previous = i;
            _next[i] = r.Node(n, "next node");
        }
        _open.ReadState(ref r);
        _touched.Clear();
        if (repairing)
        {
            int touched = r.Count(n, 4, "the touched node count");
            _stamp ??= new uint[n];
            _touchStamp ??= new uint[n];
            Array.Clear(_stamp);
            Array.Clear(_touchStamp);
            _stampBase = 1;
            for (int k = 0; k < touched; k++)
            {
                int node = r.Node(n, "touched node");
                if (_touchStamp[node] != 0)
                {
                    throw r.Fail($"node {node} is touched twice.");
                }
                _touchStamp[node] = 1;
                _touched.Add(node);
            }
        }
        // Paths follow these steps cell by cell; every step must lead to a cell that is reached and no dearer, so a path cannot cycle
        // (this holds at every moment of a run or repair, since a cell whose route is invalidated loses its step too).
        for (int i = 0; i < n; i++)
        {
            int next = _next[i];
            if (next >= 0 && (double.IsPositiveInfinity(_dist[i]) || !(_dist[next] <= _dist[i])))
            {
                throw r.Fail($"cell {i} steps to cell {next}, which is not reached or costs more.");
            }
        }
        _settled = settled;
        _repairing = repairing;
        _status = (FlowFieldStatus)status;
    }
}
