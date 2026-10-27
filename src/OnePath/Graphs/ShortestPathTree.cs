using System;
using System.Buffers.Binary;
using System.Globalization;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Graphs;

/// <summary>Single-source shortest paths over a <see cref="CsrGraph"/> (Dijkstra), used by landmark precomputation.</summary>
public static class ShortestPathTree
{
    /// <summary>
    /// Computes distances from <paramref name="source"/> (or to it, over the reverse edges) with Dijkstra's algorithm using the
    /// grid core's <see cref="OpenSet"/>, so ties settle in node-id order.
    /// </summary>
    /// <param name="graph">The graph.</param>
    /// <param name="source">Source node.</param>
    /// <param name="reverse">False: <c>dist[v]</c> = d(source, v). True: <c>dist[v]</c> = d(v, source).</param>
    /// <param name="dist">Receives the distances (+∞ for unreachable nodes); at least the node count.</param>
    /// <param name="parent">Receives each node's predecessor on its shortest path (−1 for the source and unreachable nodes); at least the node count.</param>
    /// <param name="order">Receives the settled nodes in settle order; at least the node count.</param>
    /// <returns>The number of nodes settled (the length of the used prefix of <paramref name="order"/>).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="graph"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="source"/> is outside the graph.</exception>
    /// <exception cref="ArgumentException">An output span is shorter than the node count.</exception>
    public static int Compute(CsrGraph graph, int source, bool reverse, Span<double> dist, Span<int> parent, Span<int> order)
    {
        ArgumentNullException.ThrowIfNull(graph);
        int n = graph.NodeCount;
        if ((uint)source >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(source), source, "Source node is outside the graph.");
        }
        if (dist.Length < n || parent.Length < n || order.Length < n)
        {
            throw new ArgumentException($"The output spans must hold the graph's {n} nodes.");
        }
        int[] offsets = reverse ? graph.ReverseOffsetArray : graph.OffsetArray;
        int[] nodes = reverse ? graph.SourceArray : graph.TargetArray;
        double[] weights = reverse ? graph.ReverseWeightArray : graph.WeightArray;
        dist[..n].Fill(double.PositiveInfinity);
        parent[..n].Fill(-1);
        var open = new OpenSet(n);
        dist[source] = 0.0;
        open.Push(source, 0.0, 0.0);
        int settled = 0;
        while (open.Pop(out int u, out _, out _))
        {
            order[settled++] = u;
            double du = dist[u];
            int end = offsets[u + 1];
            for (int e = offsets[u]; e < end; e++)
            {
                int v = nodes[e];
                double dv = du + weights[e];
                if (dv < dist[v])
                {
                    dist[v] = dv;
                    parent[v] = u;
                    open.Push(v, dv, dv);
                }
            }
        }
        return settled;
    }
}
