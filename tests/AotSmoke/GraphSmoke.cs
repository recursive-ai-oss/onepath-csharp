using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;

/// <summary>A graph run used by the NativeAOT and WebAssembly smoke apps: tier-1 queries with delegates, point graphs, landmarks (build, serialize, read) and the struct-generic searches.</summary>
internal static class GraphSmoke
{
    public static bool Run()
    {
        // Tier 1: edge list -> path with every algorithm.
        var g = Graph.From(new[] { (0, 1, 1.0), (1, 2, 1.0), (2, 3, 1.0), (0, 3, 5.0), (3, 4, 2.0) });
        bool ok = true;
        foreach (GraphAlgorithm a in new[] { GraphAlgorithm.Dijkstra, GraphAlgorithm.AStar, GraphAlgorithm.BidirectionalDijkstra, GraphAlgorithm.BidirectionalAStar })
        {
            GraphPath p = g.FindPath(0, 4, new GraphOptions { Algorithm = a, Heuristic = (_, _) => 0.0 });
            ok &= p.Found && p.Cost == 5.0 && p.Nodes.Length == 5;
        }
        GraphPath hooked = g.FindPath(0, 4, new GraphOptions { Blocked = n => n == 2, EdgeCost = (_, _, w) => w * 2 });
        ok &= hooked.Found && hooked.Cost == 14.0;

        // Landmarks: build, serialize, read back, search with them.
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 2 });
        Landmarks back = Landmarks.Deserialize(lm.Serialize(), g.Compile());
        GraphPath alt = g.FindPath(0, 4, new GraphOptions { Landmarks = back, Algorithm = GraphAlgorithm.BidirectionalAStar });
        ok &= alt.Found && alt.Cost == 5.0;

        // Point graph: links by distance and by nearest count, a raycast-style delegate, a query by position.
        var pts = new List<Point3> { new(0, 0), new(10, 0), new(20, 0), new(20, 10), new(0, 10) };
        var pg = PointGraph.From(pts, new PointGraphOptions { MaxDistance = 11, LinkValid = (_, _) => true });
        GraphPath pp = pg.FindPath(new Point3(1, 1), new Point3(19, 11));
        ok &= pp.Found && pp.Cost == 30.0 && pp.Points.Length == 4;
        var knn = PointGraph.From(pts, new PointGraphOptions { NearestCount = 2 });
        ok &= knn.FindPath(0, 3).Found;

        // Tier 2 and 3: struct-generic search over the CSR form.
        CsrGraph csr = g.Compile();
        var search = new GraphAStar<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new RecursiveAI.OnePath.Grids.GridSearchBuffers(csr.NodeCount));
        search.Begin(0, 4);
        ok &= search.Run() == RecursiveAI.OnePath.Grids.SearchStatus.Found && search.PathCost == 5.0;
        return ok;
    }
}
