using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>Fuzz targets for graphs: random small graphs against brute force, landmark deserialization, and point-graph input.</summary>
internal static class GraphTargets
{
    // A small graph decoded from the bytes; every search variant must agree with a brute-force shortest distance,
    // and returned paths must follow edges at the stated cost.
    internal static void Query(byte[] data)
    {
        if (data.Length < 8)
        {
            return;
        }
        int n = 1 + (data[0] % 24);
        bool directed = (data[1] & 1) != 0;
        int landmarkCount = 1 + (data[2] % 5);
        var selection = (data[2] & 0x80) != 0 ? LandmarkSelection.Farthest : LandmarkSelection.Avoid;
        var edges = new List<GraphEdge>();
        int pos = 3;
        while (pos + 2 < data.Length && edges.Count < 200)
        {
            edges.Add(new GraphEdge(data[pos] % n, data[pos + 1] % n, data[pos + 2] % 16));
            pos += 3;
        }
        Graph g = Graph.From(edges, n, directed);
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = landmarkCount, Selection = selection, Seed = data[0] });
        double[,] best = new double[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                best[i, j] = i == j ? 0 : double.PositiveInfinity;
            }
        }
        foreach (GraphEdge e in edges)
        {
            best[e.From, e.To] = Math.Min(best[e.From, e.To], e.Weight);
            if (!directed)
            {
                best[e.To, e.From] = Math.Min(best[e.To, e.From], e.Weight);
            }
        }
        for (int k = 0; k < n; k++)
        {
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                {
                    best[i, j] = Math.Min(best[i, j], best[i, k] + best[k, j]);
                }
            }
        }
        for (int s = 0; s < n; s++)
        {
            for (int t = 0; t < n; t++)
            {
                foreach (GraphAlgorithm a in new[] { GraphAlgorithm.Dijkstra, GraphAlgorithm.AStar, GraphAlgorithm.BidirectionalDijkstra, GraphAlgorithm.BidirectionalAStar })
                {
                    GraphPath p = g.FindPath(s, t, new GraphOptions { Algorithm = a, Landmarks = lm });
                    if (double.IsPositiveInfinity(best[s, t]))
                    {
                        if (p.Status != PathStatus.NoPath)
                        {
                            throw new InvalidOperationException($"{a} {s}->{t}: expected no path, got {p.Status}");
                        }
                        continue;
                    }
                    if (!p.Found || p.Cost != best[s, t] || p.Nodes[0] != s || p.Nodes[^1] != t)
                    {
                        throw new InvalidOperationException($"{a} {s}->{t}: cost {p.Cost} status {p.Status}, expected {best[s, t]}");
                    }
                    double sum = 0;
                    for (int i = 0; i + 1 < p.Nodes.Length; i++)
                    {
                        double w = double.PositiveInfinity;
                        foreach (GraphEdge e in edges)
                        {
                            if ((e.From == p.Nodes[i] && e.To == p.Nodes[i + 1]) || (!directed && e.To == p.Nodes[i] && e.From == p.Nodes[i + 1]))
                            {
                                w = Math.Min(w, e.Weight);
                            }
                        }
                        sum += w;
                    }
                    if (sum != p.Cost)
                    {
                        throw new InvalidOperationException($"{a} {s}->{t}: path walks at cost {sum}, stated {p.Cost}");
                    }
                }
            }
        }
    }

    private static readonly CsrGraph LandmarkGraph = CsrGraph.Create(6, [new GraphEdge(0, 1, 1), new GraphEdge(1, 2, 2), new GraphEdge(2, 3, 1), new GraphEdge(3, 4, 5), new GraphEdge(4, 5, 1), new GraphEdge(5, 0, 2)]);

    // Valid landmark tables for the fixed graph above (the reader checks a fingerprint, so a mutator needs a valid start).
    internal static void WriteSeeds(string directory)
    {
        System.IO.Directory.CreateDirectory(directory);
        System.IO.File.WriteAllBytes(System.IO.Path.Combine(directory, "landmarks2.bin"), Landmarks.Build(LandmarkGraph, 2).Serialize());
        System.IO.File.WriteAllBytes(System.IO.Path.Combine(directory, "landmarks4.bin"), Landmarks.Build(LandmarkGraph, 4, LandmarkSelection.Farthest, 3).Serialize());
    }

    // Arbitrary bytes into the landmark reader: only the typed error is acceptable; anything accepted must be usable.
    internal static void LandmarkData(byte[] data)
    {
        try
        {
            Landmarks lm = Landmarks.Deserialize(data, LandmarkGraph);
            for (int a = 0; a < 6; a++)
            {
                for (int b = 0; b < 6; b++)
                {
                    double v = lm.LowerBound(a, b);
                    if (double.IsNaN(v) || v < 0)
                    {
                        throw new InvalidOperationException("landmark bound is not a non-negative number");
                    }
                }
            }
            if (!lm.Serialize().AsSpan().SequenceEqual(data))
            {
                throw new InvalidOperationException("accepted landmark data does not round-trip");
            }
        }
        catch (GraphFormatException)
        {
        }
    }

    // Points decoded from doubles in the bytes (NaN, infinities and huge values included) and the linking options.
    internal static void PointInput(byte[] data)
    {
        if (data.Length < 8)
        {
            return;
        }
        int count = data[0] % 20;
        double distance = BitConverter.ToDouble(data, 0) is double d && double.IsFinite(d) ? Math.Abs(d % 50.0) : data[1] % 30;
        int nearest = data[2] % 5;
        var pts = new List<Point3>();
        int pos = 3;
        for (int i = 0; i < count && pos + 3 <= data.Length; i++, pos += 3)
        {
            double Coord(byte b) => b == 255 ? double.NaN : b == 254 ? 1e9 : (b - 100) * 0.75;
            pts.Add(new Point3(Coord(data[pos]), Coord(data[pos + 1]), Coord(data[pos + 2])));
        }
        try
        {
            PointGraph g = PointGraph.From(pts, new PointGraphOptions { MaxDistance = distance < 1e-8 ? 0 : distance, NearestCount = nearest, LinkValid = (a, b) => ((int)(a.X + b.X) & 1) == 0 });
            if (g.NodeCount != pts.Count)
            {
                throw new InvalidOperationException("node count differs from point count");
            }
            for (int s = 0; s < g.NodeCount; s++)
            {
                GraphPath p = g.FindPath(s, g.NodeCount - 1, new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalAStar });
                GraphPath q = g.FindPath(s, g.NodeCount - 1, new GraphOptions { Algorithm = GraphAlgorithm.Dijkstra });
                if (p.Status != q.Status || Math.Abs(p.Cost - q.Cost) > 1e-9 * Math.Max(1.0, q.Cost))
                {
                    throw new InvalidOperationException("bidirectional A* and Dijkstra disagree");
                }
            }
        }
        catch (ArgumentOutOfRangeException)
        {
        }
    }
}
