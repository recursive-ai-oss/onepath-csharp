using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using Xunit;
using Xunit.Sdk;

namespace RecursiveAI.OnePath.Tests.Graphs;

/// <summary>Test graph families and an independent brute-force Dijkstra (O(V^2) over a dense matrix of cheapest parallel edges).</summary>
internal static class GraphWorlds
{
    internal sealed record World(string Family, int Nodes, List<GraphEdge> Edges, bool Directed, Point3[]? Positions);

    internal static World Make(int index)
    {
        var rng = new SeededRandom(0xC0FFEE00UL + (ulong)index);
        switch (index % 4)
        {
            case 0: // sparse, directed, zero weights included
            {
                int n = 2 + rng.NextInt(60);
                int m = n + rng.NextInt(n * 2);
                var edges = new List<GraphEdge>();
                for (int i = 0; i < m; i++)
                {
                    edges.Add(new GraphEdge(rng.NextInt(n), rng.NextInt(n), rng.NextInt(21)));
                }
                return new World("sparse", n, edges, true, null);
            }

            case 1: // dense, undirected
            {
                int n = 2 + rng.NextInt(24);
                var edges = new List<GraphEdge>();
                for (int a = 0; a < n; a++)
                {
                    for (int b = a + 1; b < n; b++)
                    {
                        if (rng.NextDouble() < 0.6)
                        {
                            edges.Add(new GraphEdge(a, b, 1 + rng.NextInt(30)));
                        }
                    }
                }
                return new World("dense", n, edges, false, null);
            }

            case 2: // grid-like, undirected, positions are cell coordinates, weights >= distance
            {
                int w = 2 + rng.NextInt(9);
                int h = 2 + rng.NextInt(9);
                var edges = new List<GraphEdge>();
                var pos = new Point3[w * h];
                for (int y = 0; y < h; y++)
                {
                    for (int x = 0; x < w; x++)
                    {
                        pos[(y * w) + x] = new Point3(x, y);
                        if (x + 1 < w && rng.NextDouble() < 0.9)
                        {
                            edges.Add(new GraphEdge((y * w) + x, (y * w) + x + 1, 1 + rng.NextInt(4)));
                        }
                        if (y + 1 < h && rng.NextDouble() < 0.9)
                        {
                            edges.Add(new GraphEdge((y * w) + x, ((y + 1) * w) + x, 1 + rng.NextInt(4)));
                        }
                    }
                }
                return new World("grid-like", w * h, edges, false, pos);
            }

            default: // road-like: scattered points, links to near neighbours, a few long fast-looking edges
            {
                int n = 8 + rng.NextInt(80);
                var pos = new Point3[n];
                for (int i = 0; i < n; i++)
                {
                    pos[i] = new Point3(rng.NextInt(100), rng.NextInt(100));
                }
                var edges = new List<GraphEdge>();
                for (int a = 0; a < n; a++)
                {
                    var near = Enumerable.Range(0, n).Where(b => b != a).OrderBy(b => pos[a].DistanceTo(pos[b])).ThenBy(b => b).Take(3);
                    foreach (int b in near)
                    {
                        edges.Add(new GraphEdge(a, b, Math.Ceiling(pos[a].DistanceTo(pos[b]))));
                    }
                }
                for (int i = 0; i < n / 8; i++)
                {
                    int a = rng.NextInt(n);
                    int b = rng.NextInt(n);
                    edges.Add(new GraphEdge(a, b, Math.Ceiling(pos[a].DistanceTo(pos[b])) + rng.NextInt(3)));
                }
                return new World("road-like", n, edges, false, pos);
            }
        }
    }

    internal static GraphPath Find(Graph g, int s, int t, GraphOptions? o = null) => g.FindPath(s, t, o, TestContext.Current.CancellationToken);

    internal static GraphPath Find(PointGraph g, int s, int t, GraphOptions? o = null) => g.FindPath(s, t, o, TestContext.Current.CancellationToken);

    internal static GraphPath Find(PointGraph g, Point3 s, Point3 t, GraphOptions? o = null) => g.FindPath(s, t, o, TestContext.Current.CancellationToken);

    internal static double[] BruteDijkstra(World w, int source)
    {
        int n = w.Nodes;
        var best = new double[n, n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                best[i, j] = double.PositiveInfinity;
            }
        }
        foreach (GraphEdge e in w.Edges)
        {
            best[e.From, e.To] = Math.Min(best[e.From, e.To], e.Weight);
            if (!w.Directed)
            {
                best[e.To, e.From] = Math.Min(best[e.To, e.From], e.Weight);
            }
        }
        var dist = new double[n];
        Array.Fill(dist, double.PositiveInfinity);
        var done = new bool[n];
        dist[source] = 0;
        for (int it = 0; it < n; it++)
        {
            int u = -1;
            for (int v = 0; v < n; v++)
            {
                if (!done[v] && (u < 0 || dist[v] < dist[u]))
                {
                    u = v;
                }
            }
            if (u < 0 || double.IsPositiveInfinity(dist[u]))
            {
                break;
            }
            done[u] = true;
            for (int v = 0; v < n; v++)
            {
                dist[v] = Math.Min(dist[v], dist[u] + best[u, v]);
            }
        }
        return dist;
    }

    // Cost of walking `nodes` along the cheapest parallel edges of the world; NaN if a step has no edge.
    internal static double WalkCost(World w, ReadOnlySpan<int> nodes)
    {
        double sum = 0;
        for (int i = 0; i + 1 < nodes.Length; i++)
        {
            double best = double.PositiveInfinity;
            foreach (GraphEdge e in w.Edges)
            {
                if ((e.From == nodes[i] && e.To == nodes[i + 1]) || (!w.Directed && e.To == nodes[i] && e.From == nodes[i + 1]))
                {
                    best = Math.Min(best, e.Weight);
                }
            }
            if (double.IsPositiveInfinity(best))
            {
                return double.NaN;
            }
            sum += best;
        }
        return sum;
    }
}

public class GraphSearchTests
{
    private static readonly GraphAlgorithm[] Variants =
    [
        GraphAlgorithm.Dijkstra,
        GraphAlgorithm.AStar,
        GraphAlgorithm.BidirectionalDijkstra,
        GraphAlgorithm.BidirectionalAStar,
    ];

    [Fact]
    public void AllVariantsEqualBruteForceDijkstraOnTenThousandRandomGraphs()
    {
        const int Graphs = 12000;
        long checkedQueries = 0;
        for (int index = 0; index < Graphs; index++)
        {
            GraphWorlds.World w = GraphWorlds.Make(index);
            Graph g = Graph.From(w.Edges, w.Nodes, w.Directed);
            Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 1 + (index % 5), Selection = index % 2 == 0 ? LandmarkSelection.Avoid : LandmarkSelection.Farthest, Seed = (ulong)index });
            Func<int, int, double>? euclid = w.Positions is null ? null : (a, b) => w.Positions[a].DistanceTo(w.Positions[b]);
            var rng = new SeededRandom((ulong)index * 7919UL);
            for (int q = 0; q < 3; q++)
            {
                int s = rng.NextInt(w.Nodes);
                double[] truth = GraphWorlds.BruteDijkstra(w, s);
                int t = rng.NextInt(w.Nodes);
                // heuristics: none, landmarks, geometry, both
                var optionSets = new List<GraphOptions> { new(), new() { Landmarks = lm } };
                if (euclid is not null)
                {
                    optionSets.Add(new GraphOptions { Heuristic = euclid });
                    optionSets.Add(new GraphOptions { Heuristic = euclid, Landmarks = lm });
                }
                foreach (GraphOptions baseOptions in optionSets)
                {
                    foreach (GraphAlgorithm algorithm in Variants)
                    {
                        var options = new GraphOptions { Algorithm = algorithm, Landmarks = baseOptions.Landmarks, Heuristic = baseOptions.Heuristic };
                        GraphPath p = GraphWorlds.Find(g, s, t, options);
                        string label = $"graph {index} ({w.Family}) {s}->{t} {algorithm} lm={options.Landmarks is not null} h={options.Heuristic is not null}";
                        if (double.IsPositiveInfinity(truth[t]))
                        {
                            Assert.True(p.Status == PathStatus.NoPath, label + " should have no path");
                            continue;
                        }
                        Assert.True(p.Found, label + " should find a path");
                        Assert.True(p.Cost == truth[t], $"{label}: cost {p.Cost} vs {truth[t]}");
                        Assert.Equal(s, p.Nodes[0]);
                        Assert.Equal(t, p.Nodes[^1]);
                        Assert.True(GraphWorlds.WalkCost(w, p.Nodes) == p.Cost, label + ": path does not follow edges at the stated cost");
                        checkedQueries++;
                    }
                }
            }
        }
        Assert.True(checkedQueries > 100000);
    }

    [Fact]
    public void FractionalWeightsAgreeWithinRounding()
    {
        var rng = new SeededRandom(99);
        for (int trial = 0; trial < 300; trial++)
        {
            int n = 30 + rng.NextInt(100);
            var pos = new Point3[n];
            for (int i = 0; i < n; i++)
            {
                pos[i] = new Point3(rng.NextDouble() * 1000, rng.NextDouble() * 1000);
            }
            var links = new List<(int, int)>();
            for (int i = 0; i < n * 3; i++)
            {
                links.Add((rng.NextInt(n), rng.NextInt(n)));
            }
            var pg = PointGraph.From(pos, new PointGraphOptions { Links = links });
            Landmarks lm = pg.BuildLandmarks(new LandmarkOptions { Count = 4 });
            int s = rng.NextInt(n);
            int t = rng.NextInt(n);
            GraphPath reference = GraphWorlds.Find(pg, s, t, new GraphOptions { Algorithm = GraphAlgorithm.Dijkstra });
            foreach (GraphAlgorithm a in Variants)
            {
                GraphPath p = GraphWorlds.Find(pg, s, t, new GraphOptions { Algorithm = a, Landmarks = lm });
                Assert.Equal(reference.Status, p.Status);
                if (reference.Found)
                {
                    Assert.True(Math.Abs(p.Cost - reference.Cost) <= 1e-9 * Math.Max(1.0, reference.Cost), $"{a}: {p.Cost} vs {reference.Cost}");
                }
            }
        }
    }

    [Fact]
    public void EdgeCostAndBlockedHooksApplyInTheEdgeDirection()
    {
        // Directed 0 -> 1 -> 2 and 0 -> 2; the hook makes the direct edge forbidden, in both directions of search.
        var g = Graph.From(new[] { (0, 1, 1.0), (1, 2, 1.0), (0, 2, 1.0) }, directed: true);
        foreach (GraphAlgorithm a in Variants)
        {
            GraphPath p = GraphWorlds.Find(g, 0, 2, new GraphOptions { Algorithm = a, EdgeCost = (from, to, w) => from == 0 && to == 2 ? double.PositiveInfinity : w * 3 });
            Assert.Equal([0, 1, 2], p.Nodes.ToArray());
            Assert.Equal(6.0, p.Cost);
            GraphPath blocked = GraphWorlds.Find(g, 0, 2, new GraphOptions { Algorithm = a, Blocked = n => n == 1 });
            Assert.Equal([0, 2], blocked.Nodes.ToArray());
            Assert.Equal(PathStatus.StartInvalid, GraphWorlds.Find(g, 0, 2, new GraphOptions { Algorithm = a, Blocked = n => n == 0 }).Status);
            Assert.Equal(PathStatus.GoalInvalid, GraphWorlds.Find(g, 0, 7, new GraphOptions { Algorithm = a }).Status);
        }
    }

    [Fact]
    public void BidirectionalDoesNotStopAtTheFirstMeeting()
    {
        // The first meeting costs 10 (a -> m -> b); the cheaper route a -> x -> y -> b costs 6.
        var g = Graph.From(new[] { (0, 1, 5.0), (1, 3, 5.0), (0, 2, 2.0), (2, 4, 2.0), (4, 3, 2.0) }, directed: true);
        foreach (GraphAlgorithm a in new[] { GraphAlgorithm.BidirectionalDijkstra, GraphAlgorithm.BidirectionalAStar })
        {
            GraphPath p = GraphWorlds.Find(g, 0, 3, new GraphOptions { Algorithm = a });
            Assert.Equal(6.0, p.Cost);
            Assert.Equal([0, 2, 4, 3], p.Nodes.ToArray());
        }
    }

    [Fact]
    public void StartEqualsGoalAndUnreachable()
    {
        var g = Graph.From(new[] { (0, 1, 1.0) }, nodeCount: 3, directed: true);
        foreach (GraphAlgorithm a in Variants)
        {
            GraphPath same = GraphWorlds.Find(g, 2, 2, new GraphOptions { Algorithm = a });
            Assert.True(same.Found);
            Assert.Equal(0.0, same.Cost);
            Assert.Equal([2], same.Nodes.ToArray());
            Assert.Equal(PathStatus.NoPath, GraphWorlds.Find(g, 1, 0, new GraphOptions { Algorithm = a }).Status);
            Assert.Equal(PathStatus.NoPath, GraphWorlds.Find(g, 0, 2, new GraphOptions { Algorithm = a, Landmarks = g.BuildLandmarks(new LandmarkOptions { Count = 2 }) }).Status);
        }
    }

    [Fact]
    public void SlicedSearchEqualsUnsliced()
    {
        for (int index = 0; index < 400; index++)
        {
            GraphWorlds.World w = GraphWorlds.Make(index);
            CsrGraph csr = Graph.From(w.Edges, w.Nodes, w.Directed).Compile();
            Landmarks lm = Landmarks.Build(csr, 3, LandmarkSelection.Avoid, 5);
            int s = index % w.Nodes;
            int t = (index * 7 + 3) % w.Nodes;
            var capacity = csr.NodeCount;

            var whole = new GraphAStar<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(capacity));
            whole.Begin(s, t);
            whole.Run(TestContext.Current.CancellationToken);
            var whole2 = new BidirectionalGraphSearch<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(capacity), new GridSearchBuffers(capacity));
            whole2.Begin(s, t);
            whole2.Run(TestContext.Current.CancellationToken);

            foreach (int budget in new[] { 1, 2, 3, 7 })
            {
                var sliced = new GraphAStar<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(capacity));
                sliced.Begin(s, t);
                while (sliced.Step(budget) == SearchStatus.Running)
                {
                }
                Assert.Equal(whole.Status, sliced.Status);
                Assert.Equal(whole.PathCost, sliced.PathCost);
                Assert.Equal(whole.ExpandedCount, sliced.ExpandedCount);
                Assert.Equal(whole.Iterations, sliced.Iterations);

                var sliced2 = new BidirectionalGraphSearch<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(capacity), new GridSearchBuffers(capacity));
                sliced2.Begin(s, t);
                while (sliced2.Step(budget) == SearchStatus.Running)
                {
                }
                Assert.Equal(whole2.Status, sliced2.Status);
                Assert.Equal(whole2.PathCost, sliced2.PathCost);
                Assert.Equal(whole2.ExpandedCount, sliced2.ExpandedCount);
                if (whole2.Status == SearchStatus.Found)
                {
                    var a = new int[whole2.PathLength()];
                    var b = new int[sliced2.PathLength()];
                    whole2.CopyPath(a);
                    sliced2.CopyPath(b);
                    Assert.Equal(a, b);
                }
            }
        }
    }

    [Fact]
    public void TierOneBudgetEndsWithBudgetExhaustedAndFullBudgetMatches()
    {
        GraphWorlds.World w = GraphWorlds.Make(3); // road-like
        Graph g = Graph.From(w.Edges, w.Nodes, w.Directed);
        GraphPath full = GraphWorlds.Find(g, 0, w.Nodes - 1, new GraphOptions { RecordExpansions = true });
        if (!full.Found || full.ExpandedCount < 2)
        {
            return;
        }
        GraphPath cut = GraphWorlds.Find(g, 0, w.Nodes - 1, new GraphOptions { IterationBudget = 1 });
        Assert.Equal(PathStatus.BudgetExhausted, cut.Status);
        GraphPath enough = GraphWorlds.Find(g, 0, w.Nodes - 1, new GraphOptions { IterationBudget = full.ExpandedCount + 1 });
        Assert.Equal(full.Cost, enough.Cost);
        Assert.Equal(full.ExpandedNodes.ToArray(), GraphWorlds.Find(g, 0, w.Nodes - 1, new GraphOptions { RecordExpansions = true }).ExpandedNodes.ToArray());
    }

    [Fact]
    public void ExpansionCountsOfAltAndBidirectionalOnRoadLikeGraphs()
    {
        // Counts only, no timing. A larger generated planar road-like graph: jittered lattice with random removals and a few long highways.
        var rng = new SeededRandom(2024);
        const int W = 80;
        const int H = 80;
        var pos = new Point3[W * H];
        var edges = new List<GraphEdge>();
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                pos[(y * W) + x] = new Point3((x * 10) + rng.NextInt(5), (y * 10) + rng.NextInt(5));
            }
        }
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                int a = (y * W) + x;
                if (x + 1 < W && rng.NextDouble() < 0.85)
                {
                    edges.Add(new GraphEdge(a, a + 1, Math.Ceiling(pos[a].DistanceTo(pos[a + 1]))));
                }
                if (y + 1 < H && rng.NextDouble() < 0.85)
                {
                    edges.Add(new GraphEdge(a, a + W, Math.Ceiling(pos[a].DistanceTo(pos[a + W]))));
                }
            }
        }
        for (int i = 0; i < 40; i++)
        {
            int a = rng.NextInt(W * H);
            int b = rng.NextInt(W * H);
            edges.Add(new GraphEdge(a, b, Math.Ceiling(pos[a].DistanceTo(pos[b]) * 0.5) + Math.Ceiling(pos[a].DistanceTo(pos[b]) * 0.5)));
        }
        Graph g = Graph.From(edges, W * H, false);
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 16, Selection = LandmarkSelection.Avoid, Seed = 7 });
        Landmarks far = g.BuildLandmarks(new LandmarkOptions { Count = 16, Selection = LandmarkSelection.Farthest, Seed = 7 });
        long dij = 0, ast = 0, alt = 0, altFar = 0, bidiDij = 0, bidiAlt = 0, bidiAst = 0;
        int found = 0;
        for (int q = 0; q < 60; q++)
        {
            int s = rng.NextInt(W * H);
            int t = rng.NextInt(W * H);
            GraphPath d = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.Dijkstra });
            if (!d.Found || s == t)
            {
                continue;
            }
            found++;
            Func<int, int, double> euclid = (a, b) => pos[a].DistanceTo(pos[b]);
            GraphPath a1 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.AStar, Heuristic = euclid });
            GraphPath a2 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.AStar, Landmarks = lm });
            GraphPath a3 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.AStar, Landmarks = far });
            GraphPath b1 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalDijkstra });
            GraphPath b2 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalAStar, Landmarks = lm });
            GraphPath b3 = GraphWorlds.Find(g, s, t, new GraphOptions { Algorithm = GraphAlgorithm.BidirectionalAStar, Heuristic = euclid });
            foreach (GraphPath p in new[] { a1, a2, a3, b1, b2, b3 })
            {
                Assert.Equal(d.Cost, p.Cost);
            }
            dij += d.ExpandedCount;
            ast += a1.ExpandedCount;
            alt += a2.ExpandedCount;
            altFar += a3.ExpandedCount;
            bidiDij += b1.ExpandedCount;
            bidiAlt += b2.ExpandedCount;
            bidiAst += b3.ExpandedCount;
        }
        Assert.True(found > 30);
        Console.WriteLine($"expansions over {found} queries on a {W * H}-node road-like graph: dijkstra {dij}, A*(euclid) {ast}, ALT(avoid) {alt}, ALT(farthest) {altFar}, bidirectional dijkstra {bidiDij}, bidirectional A*(ALT) {bidiAlt}, bidirectional A*(euclid) {bidiAst}");
        Assert.True(alt < ast, $"ALT {alt} vs A* {ast}");
        Assert.True(bidiDij < dij, $"bidirectional {bidiDij} vs dijkstra {dij}");
        Assert.True(bidiAlt < ast, $"bidirectional ALT {bidiAlt} vs A* {ast}");
    }

    [Fact]
    public void EditsRebuildAndStayStable()
    {
        var g = new Graph(3);
        int e01 = g.AddEdge(0, 1, 1);
        g.AddEdge(1, 2, 1);
        int e02 = g.AddEdge(0, 2, 5);
        Assert.Equal(2.0, GraphWorlds.Find(g, 0, 2).Cost);
        Assert.True(g.RemoveEdge(e01));
        Assert.False(g.RemoveEdge(e01));
        Assert.Equal(5.0, GraphWorlds.Find(g, 0, 2).Cost);
        g.SetWeight(e02, 0.5);
        Assert.Equal(0.5, GraphWorlds.Find(g, 0, 2).Cost);
        int n3 = g.AddNode();
        Assert.Equal(3, n3);
        int e23 = g.AddEdge(2, 3, 2);
        Assert.Equal(e02 + 1, e23); // ids are never reused
        Assert.Equal(2.5, GraphWorlds.Find(g, 0, 3).Cost);
        Assert.Equal(3, g.EdgeCount);
        Assert.Throws<ArgumentException>(() => g.AddEdge(0, 9, 1));
        Assert.Throws<ArgumentException>(() => g.AddEdge(0, 1, -1));
        Assert.Throws<ArgumentException>(() => g.AddEdge(0, 1, double.NaN));
        Assert.Throws<ArgumentException>(() => g.AddEdge(0, 1, double.PositiveInfinity));
        Assert.Throws<ArgumentOutOfRangeException>(() => g.RemoveEdge(99));

        // Landmarks built before an edit are refused afterwards.
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 2 });
        g.AddEdge(1, 3, 0.25);
        Assert.Throws<InvalidOperationException>(() => GraphWorlds.Find(g, 0, 3, new GraphOptions { Landmarks = lm }));
    }

    [Fact]
    public void CsrOrderAndValidation()
    {
        CsrGraph c = CsrGraph.Create(3, [new GraphEdge(1, 2, 4), new GraphEdge(0, 1, 1), new GraphEdge(1, 0, 2), new GraphEdge(1, 2, 3)]);
        Assert.Equal([0, 1, 4, 4], c.Offsets.ToArray());
        Assert.Equal([1, 2, 0, 2], c.Targets.ToArray());   // input order within a node
        Assert.Equal([1.0, 4.0, 2.0, 3.0], c.Weights.ToArray());
        Assert.Equal([1, 0, 1, 1], c.Sources.ToArray()); // into 0: from 1; into 1: from 0; into 2: from 1, 1
        Assert.Throws<ArgumentException>(() => CsrGraph.Create(2, [new GraphEdge(0, 2, 1)]));
        Assert.Throws<ArgumentException>(() => CsrGraph.Create(2, [new GraphEdge(0, 1, double.NaN)]));
        Assert.Throws<ArgumentOutOfRangeException>(() => CsrGraph.Create(-1, []));
        Assert.Equal(0, CsrGraph.Create(0, []).EdgeCount);
    }
}

public class LandmarkTests
{
    [Fact]
    public void LowerBoundNeverExceedsTrueDistanceAndIsConsistent()
    {
        for (int index = 0; index < 200; index++)
        {
            GraphWorlds.World w = GraphWorlds.Make(index);
            CsrGraph csr = Graph.From(w.Edges, w.Nodes, w.Directed).Compile();
            Landmarks lm = Landmarks.Build(csr, 4, index % 2 == 0 ? LandmarkSelection.Avoid : LandmarkSelection.Farthest, (ulong)index);
            Assert.True(lm.Count >= 1 && lm.Count <= Math.Min(4, w.Nodes));
            Assert.Equal(lm.Count, lm.Nodes.ToArray().Distinct().Count());
            for (int s = 0; s < w.Nodes; s++)
            {
                double[] truth = GraphWorlds.BruteDijkstra(w, s);
                for (int t = 0; t < w.Nodes; t++)
                {
                    double b = lm.LowerBound(s, t);
                    Assert.True(b <= truth[t], $"graph {index}: bound {b} > distance {truth[t]} for {s}->{t}");
                    if (!double.IsPositiveInfinity(truth[t]) && s != t)
                    {
                        // consistency along every edge out of s towards t
                        for (int e = csr.Offsets[s]; e < csr.Offsets[s + 1]; e++)
                        {
                            Assert.True(lm.LowerBound(s, t) <= csr.Weights[e] + lm.LowerBound(csr.Targets[e], t));
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void SelectionIsDeterministicAndSeedDependent()
    {
        CsrGraph csr = Graph.From(GraphWorlds.Make(3).Edges, GraphWorlds.Make(3).Nodes, false).Compile();
        byte[] a = Landmarks.Build(csr, 5, LandmarkSelection.Avoid, 1).Serialize();
        byte[] b = Landmarks.Build(csr, 5, LandmarkSelection.Avoid, 1).Serialize();
        Assert.Equal(a, b);
        Assert.NotEqual(Landmarks.Build(csr, 5, LandmarkSelection.Avoid, 2).Nodes.ToArray(), Landmarks.Build(csr, 5, LandmarkSelection.Avoid, 1).Nodes.ToArray());
    }

    [Fact]
    public void SerializeRoundTripAndHostileData()
    {
        GraphWorlds.World w = GraphWorlds.Make(7);
        CsrGraph csr = Graph.From(w.Edges, w.Nodes, w.Directed).Compile();
        Landmarks lm = Landmarks.Build(csr, 3);
        byte[] bytes = lm.Serialize();
        Landmarks back = Landmarks.Deserialize(bytes, csr);
        Assert.Equal(bytes, back.Serialize());
        for (int s = 0; s < w.Nodes; s++)
        {
            for (int t = 0; t < w.Nodes; t++)
            {
                Assert.Equal(lm.LowerBound(s, t), back.LowerBound(s, t));
            }
        }

        // Another graph, truncation, bad magic and version, and out-of-range fields are typed errors.
        CsrGraph other = CsrGraph.Create(w.Nodes, [new GraphEdge(0, 1, 1)]);
        Assert.Throws<GraphStateException>(() => Landmarks.Deserialize(bytes, other));
        Assert.Throws<GraphStateException>(() => Landmarks.Deserialize(bytes.AsSpan(0, bytes.Length - 1), csr));
        Assert.Throws<GraphStateException>(() => Landmarks.Deserialize([], csr));
        foreach (int at in new[] { 0, 4, 8, 12, 16, 24, 30, bytes.Length - 1 })
        {
            byte[] bad = (byte[])bytes.Clone();
            bad[at] ^= 0xFF;
            try
            {
                Landmarks.Deserialize(bad, csr);
            }
            catch (GraphStateException)
            {
            }
        }
        // A negative distance is refused.
        byte[] neg = (byte[])bytes.Clone();
        int tableStart = 24 + (4 * lm.Count);
        neg[tableStart + 15] = 0xBF; // sign and exponent of the second entry
        Assert.Throws<GraphStateException>(() => Landmarks.Deserialize(neg, csr));
    }
}

public class PointGraphTests
{
    private static List<Point3> RandomPoints(int n, ulong seed, bool threeD = false)
    {
        var rng = new SeededRandom(seed);
        var pts = new List<Point3>();
        for (int i = 0; i < n; i++)
        {
            pts.Add(new Point3(rng.NextInt(2000) / 10.0, rng.NextInt(2000) / 10.0, threeD ? rng.NextInt(300) / 10.0 : 0));
        }
        return pts;
    }

    // The link set as pairs of positions, so graphs built from permuted input compare equal.
    private static HashSet<(Point3, Point3)> LinkSet(PointGraph g)
    {
        var set = new HashSet<(Point3, Point3)>();
        CsrGraph c = g.Graph;
        for (int a = 0; a < c.NodeCount; a++)
        {
            for (int e = c.Offsets[a]; e < c.Offsets[a + 1]; e++)
            {
                int b = c.Targets[e];
                if (a < b)
                {
                    set.Add((g.Positions[a], g.Positions[b]));
                }
            }
        }
        return set;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void MaxDistanceLinksMatchBruteForceAndIgnoreInputOrder(bool threeD)
    {
        List<Point3> pts = RandomPoints(150, 5, threeD).Distinct().ToList();
        const double D = 17.5;
        var g = PointGraph.From(pts, new PointGraphOptions { MaxDistance = D });
        var expected = new HashSet<(Point3, Point3)>();
        for (int i = 0; i < pts.Count; i++)
        {
            for (int j = i + 1; j < pts.Count; j++)
            {
                if (pts[i].DistanceTo(pts[j]) <= D)
                {
                    expected.Add((pts[i], pts[j]));
                }
            }
        }
        Assert.Equal(expected.Count, LinkSet(g).Count);
        foreach ((Point3 a, Point3 b) in LinkSet(g))
        {
            Assert.True(expected.Contains((a, b)) || expected.Contains((b, a)));
        }
        var shuffled = pts.OrderBy(p => (p.X * 31) + (p.Y * 17) + (p.Z * 7) + (p.X % 3)).Reverse().ToList();
        var g2 = PointGraph.From(shuffled, new PointGraphOptions { MaxDistance = D });
        Assert.Equal(Canon(LinkSet(g)), Canon(LinkSet(g2)));
    }

    private static List<(Point3, Point3)> Canon(HashSet<(Point3, Point3)> s)
    {
        return s.Select(t => Order(t.Item1, t.Item2)).OrderBy(t => t.Item1.X).ThenBy(t => t.Item1.Y).ThenBy(t => t.Item1.Z).ThenBy(t => t.Item2.X).ThenBy(t => t.Item2.Y).ThenBy(t => t.Item2.Z).ToList();
    }

    private static (Point3, Point3) Order(Point3 a, Point3 b) =>
        (a.X, a.Y, a.Z).CompareTo((b.X, b.Y, b.Z)) <= 0 ? (a, b) : (b, a);

    [Theory]
    [InlineData(false, 0.0)]
    [InlineData(true, 0.0)]
    [InlineData(false, 25.0)]
    public void NearestLinksMatchBruteForce(bool threeD, double cap)
    {
        // Distinct positions with distinct distances to avoid ties, so the answer is independent of index order too.
        var rng = new SeededRandom(77);
        var pts = new List<Point3>();
        for (int i = 0; i < 120; i++)
        {
            pts.Add(new Point3(rng.NextDouble() * 200, rng.NextDouble() * 200, threeD ? rng.NextDouble() * 30 : 0));
        }
        const int K = 3;
        var g = PointGraph.From(pts, new PointGraphOptions { NearestCount = K, MaxDistance = cap });
        var expected = new HashSet<(int, int)>();
        for (int i = 0; i < pts.Count; i++)
        {
            var order = Enumerable.Range(0, pts.Count).Where(j => j != i).OrderBy(j => pts[i].DistanceTo(pts[j])).ThenBy(j => j).Take(K);
            foreach (int j in order)
            {
                if (cap == 0.0 || pts[i].DistanceTo(pts[j]) <= cap)
                {
                    expected.Add((Math.Min(i, j), Math.Max(i, j)));
                }
            }
        }
        var actual = new HashSet<(int, int)>();
        CsrGraph c = g.Graph;
        for (int a = 0; a < c.NodeCount; a++)
        {
            for (int e = c.Offsets[a]; e < c.Offsets[a + 1]; e++)
            {
                if (a < c.Targets[e])
                {
                    actual.Add((a, c.Targets[e]));
                }
            }
        }
        Assert.Equal(expected.OrderBy(x => x).ToList(), actual.OrderBy(x => x).ToList());
        // permutation: same set of position pairs
        var perm = pts.AsEnumerable().Reverse().ToList();
        var g2 = PointGraph.From(perm, new PointGraphOptions { NearestCount = K, MaxDistance = cap });
        Assert.Equal(Canon(LinkSet(g)), Canon(LinkSet(g2)));
    }

    [Fact]
    public void LinkValidFiltersAndNearestSkipsInvalid()
    {
        // Points on a line; a wall between x = 5 and x = 6 blocks sight.
        var pts = Enumerable.Range(0, 12).Select(i => new Point3(i, 0)).ToList();
        Func<Point3, Point3, bool> sees = (a, b) => !(Math.Min(a.X, b.X) <= 5 && Math.Max(a.X, b.X) >= 6);
        var g = PointGraph.From(pts, new PointGraphOptions { MaxDistance = 3, LinkValid = sees });
        Assert.False(GraphWorlds.Find(g, 0, 11).Found);
        Assert.True(GraphWorlds.Find(g, 0, 5).Found);
        var k = PointGraph.From(pts, new PointGraphOptions { NearestCount = 2, LinkValid = sees });
        Assert.False(GraphWorlds.Find(k, 0, 11).Found);
        // each of the points next to the wall still got two links on its own side
        Assert.Equal(5.0, GraphWorlds.Find(k, 0, 5).Cost);
        var explicitLinks = PointGraph.From(pts, new PointGraphOptions { Links = [(5, 6)], LinkValid = (_, _) => false });
        Assert.Equal(1.0, GraphWorlds.Find(explicitLinks, 5, 6).Cost); // explicit links are not validated
    }

    [Fact]
    public void FindPathByPositionSnapsToNearestNodeAndFillsPositions()
    {
        var pts = new List<Point3> { new(0, 0), new(10, 0), new(10, 10), new(0, 10) };
        var g = PointGraph.From(pts, new PointGraphOptions { Links = [(0, 1), (1, 2), (2, 3)] });
        GraphPath p = GraphWorlds.Find(g, new Point3(1, 1), new Point3(1, 9));
        Assert.True(p.Found);
        Assert.Equal(30.0, p.Cost);
        Assert.Equal(pts.ToArray(), p.Points.ToArray());
        Assert.Equal(GraphAlgorithm.AStar, p.Algorithm);
        Assert.Equal(-1, PointGraph.From(new List<Point3>()).NearestNode(new Point3(0, 0)));
        Assert.Equal(PathStatus.StartInvalid, GraphWorlds.Find(PointGraph.From(new List<Point3>()), new Point3(0, 0), new Point3(1, 1)).Status);
    }

    [Fact]
    public void CoordinateRuleAndOptionValidation()
    {
        double bound = 2097152.0;
        var ok = PointGraph.From(new[] { new Point3(bound, -bound, 0) });
        Assert.Equal(1, ok.NodeCount);
        ArgumentOutOfRangeException ex = Assert.Throws<ArgumentOutOfRangeException>(() => PointGraph.From(new[] { new Point3(0, 0), new Point3(bound + 1, 0, 0) }));
        Assert.Contains("2097152", ex.Message);
        Assert.Contains("Point 1", ex.Message);
        Assert.Throws<ArgumentOutOfRangeException>(() => PointGraph.From(new[] { new Point3(double.NaN, 0, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => PointGraph.From(new[] { new Point3(0, double.PositiveInfinity, 0) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => ok.NearestNode(new Point3(0, 0, 1e9)));
        Assert.Throws<ArgumentOutOfRangeException>(() => PointGraph.From(new[] { new Point3(0, 0) }, new PointGraphOptions { MaxDistance = double.NaN }));
        Assert.Throws<ArgumentOutOfRangeException>(() => PointGraph.From(new[] { new Point3(0, 0) }, new PointGraphOptions { NearestCount = -1 }));
        Assert.Throws<ArgumentException>(() => PointGraph.From(new[] { new Point3(0, 0) }, new PointGraphOptions { Links = [(0, 3)] }));
        // duplicates and coincident points do not break link building
        var dup = PointGraph.From(new[] { new Point3(1, 1), new Point3(1, 1), new Point3(1, 1) }, new PointGraphOptions { MaxDistance = 1, NearestCount = 5 });
        Assert.True(GraphWorlds.Find(dup, 0, 2).Found);
        Assert.Equal(0.0, GraphWorlds.Find(dup, 0, 2).Cost);
    }
}
