using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePath.Grids;

public sealed partial class ReplanSearch<TTopology, TCost, THeuristic>
{
    // Layout (docs/grids-state.md): node count, mode, start, goal, initialized, found, iterations, expanded, last expanded; and when
    // initialized: km, overflow flag, pending edits, the nodes with a finite g or rhs in index order, and the queue in heap order.
    // An uninitialised search (never planned, or restarted) has no meaningful g/rhs/queue: Initialize overwrites them.
    internal override void WriteState(StateWriter w)
    {
        w.I32(_nodeCount);
        w.U8((byte)_mode);
        w.I32(_start);
        w.I32(_goal);
        w.Bool(_initialized);
        w.Bool(_found);
        w.I64(_iterations);
        w.I64(_expanded);
        w.I32(_lastExpanded);
        if (!_initialized)
        {
            return;
        }
        w.F64(_km);
        w.Bool(_pendingOverflow);
        w.I32(_pending.Count);
        for (int i = 0; i < _pending.Count; i++)
        {
            w.I32(_pending[i]);
        }
        int live = 0;
        for (int i = 0; i < _nodeCount; i++)
        {
            if (!double.IsPositiveInfinity(_g[i]) || !double.IsPositiveInfinity(_rhs[i]))
            {
                live++;
            }
        }
        w.I32(live);
        for (int i = 0; i < _nodeCount; i++)
        {
            if (!double.IsPositiveInfinity(_g[i]) || !double.IsPositiveInfinity(_rhs[i]))
            {
                w.I32(i);
                w.F64(_g[i]);
                w.F64(_rhs[i]);
            }
        }
        _queue.WriteState(w);
    }

    internal override void ReadState(ref StateReader r)
    {
        r.Expect(_nodeCount, "the node count");
        byte mode = r.U8();
        if (mode != (byte)_mode)
        {
            throw r.Fail(string.Create(CultureInfo.InvariantCulture, $"the state is for search mode {mode} but this search runs mode {(byte)_mode}."));
        }
        int start = r.Node(_nodeCount, "start");
        int goal = r.Node(_nodeCount, "goal");
        bool initialized = r.Bool();
        bool found = r.Bool();
        long iterations = r.I64();
        long expanded = r.I64();
        int lastExpanded = r.I32();
        if (iterations < 0 || expanded < 0 || lastExpanded < 0)
        {
            throw r.Fail("a search counter is negative.");
        }
        _initialized = false; // unusable until the whole state has been read
        _pending.Clear();
        _pendingOverflow = false;
        _queue.Clear();
        Array.Fill(_g, double.PositiveInfinity);
        Array.Fill(_rhs, double.PositiveInfinity);
        _start = start;
        _goal = goal;
        _found = found;
        _iterations = iterations;
        _expanded = expanded;
        _lastExpanded = lastExpanded;
        _lastNodeCount = 0;
        if (!initialized)
        {
            return;
        }
        double km = r.NotNaN("km");
        bool overflow = r.Bool();
        int pending = r.Count(_nodeCount, 4, "the pending edit count");
        if (overflow && pending != 0)
        {
            throw r.Fail("pending edits are listed although the overflow flag is set.");
        }
        for (int i = 0; i < pending; i++)
        {
            int e = r.I32();
            int node = e < 0 ? ~e : e;
            if ((uint)node >= (uint)_nodeCount)
            {
                throw r.Fail($"pending edit {e} names a node outside the {_nodeCount} nodes.");
            }
            _pending.Add(e);
        }
        int live = r.Count(_nodeCount, 20, "the number of planned nodes");
        int previous = -1;
        for (int k = 0; k < live; k++)
        {
            int i = r.Node(_nodeCount, "planned node");
            if (i <= previous)
            {
                throw r.Fail($"planned nodes must be in increasing order but {i} follows {previous}.");
            }
            previous = i;
            _g[i] = r.NotNaN("g");
            _rhs[i] = r.NotNaN("rhs");
        }
        _queue.ReadState(ref r);
        _km = km;
        _pendingOverflow = overflow;
        _initialized = true;
        _heuristic.SetGoal(Focus);
    }
}
