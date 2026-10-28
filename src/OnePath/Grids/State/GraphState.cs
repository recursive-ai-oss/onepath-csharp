using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

public sealed partial class Graph
{
    /// <summary>
    /// Writes the graph — node count and every edge ever added in id order, with its weight and whether it is still alive — to
    /// bytes for a host that rolls back. Little-endian, in the order documented in docs/grids-state.md. Edge ids survive a
    /// round trip, removed edges stay removed, and compiled data is rebuilt on demand.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        lock (_sync)
        {
            var w = new StateWriter(StateKind.Graph);
            w.I32(_nodeCount);
            w.I32(_edges.Count);
            for (int i = 0; i < _edges.Count; i++)
            {
                GraphEdge e = _edges[i];
                w.I32(e.From);
                w.I32(e.To);
                w.F64(e.Weight);
                w.Bool(_alive[i]);
            }
            return w.ToArray();
        }
    }

    /// <summary>Creates a graph from bytes written by <see cref="SaveState"/>.</summary>
    /// <param name="state">The bytes.</param>
    /// <returns>A graph equal to the saved one, with the same edge ids.</returns>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or name nodes or weights that are not valid.</exception>
    public static Graph FromState(ReadOnlySpan<byte> state)
    {
        var g = new Graph();
        g.RestoreState(state);
        return g;
    }

    /// <summary>
    /// Replaces this graph's nodes and edges with those of a state written by <see cref="SaveState"/>. Later queries, compiled
    /// graphs and landmark fingerprints equal those of the saved graph. On a bad state nothing is changed.
    /// Must not overlap a running query.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or name nodes or weights that are not valid.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.Graph);
        int nodes = r.I32();
        if ((uint)nodes > CsrGraph.MaxNodes)
        {
            throw r.Fail($"the node count {nodes} is outside 0..{CsrGraph.MaxNodes}.");
        }
        int count = r.Count(int.MaxValue, 17, "the edge count");
        var edges = new List<GraphEdge>(count);
        var alive = new List<bool>(count);
        int live = 0;
        for (int i = 0; i < count; i++)
        {
            int from = r.I32();
            int to = r.I32();
            double weight = r.F64();
            bool a = r.Bool();
            if ((uint)from >= (uint)nodes || (uint)to >= (uint)nodes)
            {
                throw r.Fail($"edge {i} runs {from} -> {to}, outside the {nodes} nodes.");
            }
            if (!(weight >= 0.0) || double.IsPositiveInfinity(weight))
            {
                throw r.Fail($"edge {i} has weight {weight}; weights must be finite and not negative.");
            }
            edges.Add(new GraphEdge(from, to, weight));
            alive.Add(a);
            if (a)
            {
                live++;
            }
        }
        r.End();
        Invalidate();
        lock (_sync)
        {
            _edges.Clear();
            _edges.AddRange(edges);
            _alive.Clear();
            _alive.AddRange(alive);
            _nodeCount = nodes;
            _liveEdges = live;
        }
    }
}
