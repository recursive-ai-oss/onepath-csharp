using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Graph scenarios: every search variant on generated road-like and random graphs (status, cost bits, path, expansion order),
/// landmark tables (selection and serialized bytes), point-graph link building with fractional distances, and queries
/// split across threads. Any difference in tie-breaking, selection or arithmetic changes the hash.
/// </summary>
internal static class GraphScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("graphs/search/variants", Variants);
        Scenarios.Register("graphs/search/sliced", Sliced);
        Scenarios.Register("graphs/search/threads", Threads);
        Scenarios.Register("graphs/landmarks/tables", LandmarkTables);
        Scenarios.Register("graphs/point/links-and-paths", PointLinks);
    }

    private static (Point3[] Pos, Graph G) RoadLike(ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed);
        var pos = new Point3[w * h];
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                pos[(y * w) + x] = new Point3((x * 10) + (rng.NextDouble() * 6), (y * 10) + (rng.NextDouble() * 6));
            }
        }
        var edges = new List<GraphEdge>();
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                int a = (y * w) + x;
                if (x + 1 < w && rng.NextDouble() < 0.85)
                {
                    edges.Add(new GraphEdge(a, a + 1, pos[a].DistanceTo(pos[a + 1]) * (1.0 + (rng.NextDouble() * 0.5))));
                }
                if (y + 1 < h && rng.NextDouble() < 0.85)
                {
                    edges.Add(new GraphEdge(a, a + w, pos[a].DistanceTo(pos[a + w]) * (1.0 + (rng.NextDouble() * 0.5))));
                }
            }
        }
        for (int i = 0; i < 12; i++)
        {
            int a = rng.NextInt(w * h);
            int b = rng.NextInt(w * h);
            edges.Add(new GraphEdge(a, b, pos[a].DistanceTo(pos[b]) * 1.1));
        }
        return (pos, Graph.From(edges, w * h, false));
    }

    private static void Write(ScenarioContext ctx, GraphPath p)
    {
        ctx.Write((int)p.Status);
        ctx.Write(p.Cost);
        ctx.Write(p.ExpandedCount);
        ctx.Write((int)p.Algorithm);
        ctx.Write(p.Nodes.Length);
        foreach (int n in p.Nodes)
        {
            ctx.Write(n);
        }
        ctx.Write(p.ExpandedNodes.Length);
        foreach (int n in p.ExpandedNodes)
        {
            ctx.Write(n);
        }
    }

    private static void Variants(ScenarioContext ctx)
    {
        (Point3[] pos, Graph g) = RoadLike(0x7001, 30, 30);
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 8, Selection = LandmarkSelection.Avoid, Seed = 3 });
        Func<int, int, double> euclid = (a, b) => pos[a].DistanceTo(pos[b]);
        var rng = new SeededRandom(0x7002);
        foreach (GraphAlgorithm alg in new[] { GraphAlgorithm.Dijkstra, GraphAlgorithm.AStar, GraphAlgorithm.BidirectionalDijkstra, GraphAlgorithm.BidirectionalAStar })
        {
            for (int q = 0; q < 12; q++)
            {
                int s = rng.NextInt(900);
                int t = rng.NextInt(900);
                Write(ctx, g.FindPath(s, t, new GraphOptions { Algorithm = alg, RecordExpansions = true, Landmarks = lm, Heuristic = (q & 1) == 0 ? euclid : null }));
            }
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        (_, Graph g) = RoadLike(0x7003, 20, 20);
        CsrGraph csr = g.Compile();
        Landmarks lm = Landmarks.Build(csr, 4, LandmarkSelection.Farthest, 9);
        var rng = new SeededRandom(0x7004);
        for (int q = 0; q < 10; q++)
        {
            int s = rng.NextInt(400);
            int t = rng.NextInt(400);
            var one = new GraphAStar<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(400));
            var sliced = new GraphAStar<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(400));
            one.Begin(s, t);
            one.Run();
            sliced.Begin(s, t);
            int budget = 1 + (q % 5);
            while (sliced.Step(budget) == SearchStatus.Running)
            {
            }
            ctx.Write((int)one.Status);
            ctx.Write(one.PathCost);
            ctx.Write(one.Iterations);
            ctx.Write(sliced.PathCost == one.PathCost && sliced.Iterations == one.Iterations ? 1 : 0);
            var bi = new BidirectionalGraphSearch<DefaultGraphCost, AltGraphHeuristic>(csr, default, new AltGraphHeuristic(lm), new GridSearchBuffers(400), new GridSearchBuffers(400));
            bi.Begin(s, t);
            while (bi.Step(budget) == SearchStatus.Running)
            {
            }
            ctx.Write((int)bi.Status);
            ctx.Write(bi.PathCost);
            ctx.Write(bi.Iterations);
            if (bi.Status == SearchStatus.Found)
            {
                var nodes = new int[bi.PathLength()];
                bi.CopyPath(nodes);
                foreach (int n in nodes)
                {
                    ctx.Write(n);
                }
            }
        }
    }

    // Queries are split by index across ctx.Threads workers over the shared read-only graph and written in index order.
    private static void Threads(ScenarioContext ctx)
    {
        (_, Graph g) = RoadLike(0x7005, 25, 25);
        Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 6, Selection = LandmarkSelection.Avoid, Seed = 11 });
        g.Compile();
        const int Queries = 64;
        var results = new GraphPath[Queries];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, options, i =>
        {
            var rng = new SeededRandom(0x7100 + (ulong)i);
            results[i] = g.FindPath(rng.NextInt(625), rng.NextInt(625), new GraphOptions { Algorithm = (i & 1) == 0 ? GraphAlgorithm.AStar : GraphAlgorithm.BidirectionalAStar, Landmarks = lm });
        });
        foreach (GraphPath p in results)
        {
            Write(ctx, p);
        }
    }

    private static void LandmarkTables(ScenarioContext ctx)
    {
        (_, Graph g) = RoadLike(0x7006, 18, 18);
        foreach (LandmarkSelection sel in new[] { LandmarkSelection.Avoid, LandmarkSelection.Farthest })
        {
            Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 6, Selection = sel, Seed = 5 });
            ctx.Write(lm.Serialize());
            ctx.Write(unchecked((long)g.Compile().Fingerprint()));
        }
    }

    private static void PointLinks(ScenarioContext ctx)
    {
        var rng = new SeededRandom(0x7007);
        var pts = new List<Point3>();
        for (int i = 0; i < 300; i++)
        {
            pts.Add(new Point3(rng.NextDouble() * 500, rng.NextDouble() * 500, rng.NextDouble() * 40));
        }
        var configs = new[]
        {
            new PointGraphOptions { MaxDistance = 40 },
            new PointGraphOptions { NearestCount = 4 },
            new PointGraphOptions { NearestCount = 3, MaxDistance = 60, LinkValid = (a, b) => ((a.X + b.Y) % 7.0) > 1.0 },
        };
        foreach (PointGraphOptions o in configs)
        {
            PointGraph pg = PointGraph.From(pts, o);
            CsrGraph c = pg.Graph;
            ctx.Write(c.EdgeCount);
            for (int i = 0; i < c.EdgeCount; i++)
            {
                ctx.Write(c.Targets[i]);
                ctx.Write(c.Weights[i]);
            }
            for (int q = 0; q < 6; q++)
            {
                Write(ctx, pg.FindPath(rng.NextInt(300), rng.NextInt(300), new GraphOptions { Algorithm = (q & 1) == 0 ? GraphAlgorithm.AStar : GraphAlgorithm.BidirectionalAStar }));
            }
        }
    }
}
