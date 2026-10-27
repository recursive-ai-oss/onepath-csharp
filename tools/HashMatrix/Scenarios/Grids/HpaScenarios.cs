using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// HPA* scenarios: the abstract graph (content hash and every intra-edge cost), refined and smoothed paths, the graph after
/// batches of cell edits, sliced searches, and queries split across threads. Costs are written as IEEE bits.
/// </summary>
internal static class HpaScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/hpa/graph-and-paths", ctx => GraphAndPaths(ctx, 0x8001, 16, smoothing: true));
        Scenarios.Register("grids/hpa/graph-and-paths-chunk7-unsmoothed", ctx => GraphAndPaths(ctx, 0x8002, 7, smoothing: false));
        Scenarios.Register("grids/hpa/graph-after-edits", GraphAfterEdits);
        Scenarios.Register("grids/hpa/sliced-equals-unsliced", Sliced);
        Scenarios.Register("grids/hpa/threads", Threads);
    }

    private static Grid MakeGrid(ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                double r = rng.NextDouble();
                if (r < 0.14)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (r < 0.25)
                {
                    g.SetArea(x, y, 1);
                }
            }
        }
        g.Areas[1] = 3.5;
        for (int k = 0; k < 4; k++)
        {
            int x = 12 + (k * (w / 5));
            int gap = rng.NextInt(h);
            for (int y = 0; y < h; y++)
            {
                if (y != gap)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
            }
        }
        return g;
    }

    private static void WritePath(ScenarioContext ctx, GridPath p)
    {
        ctx.Write((int)p.Status);
        ctx.Write((int)p.Algorithm);
        ctx.Write(p.Cost);
        ctx.Write(p.ExpandedCount);
        ctx.Write(p.Cells.Length);
        foreach (Cell c in p.Cells)
        {
            ctx.Write(c.X);
            ctx.Write(c.Y);
        }
    }

    private static void GraphAndPaths(ScenarioContext ctx, ulong seed, int chunk, bool smoothing)
    {
        const int W = 140;
        const int H = 80;
        Grid g = MakeGrid(seed, W, H);
        var options = new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = chunk, HpaSmoothing = smoothing };
        var rng = new SeededRandom(seed ^ 0xFFFF);
        WritePath(ctx, g.FindPath(new Cell(0, 0), new Cell(1, 1), options));
        using var h = new HpaHierarchy(g, new HpaOptions { ChunkSize = chunk });
        ctx.Write((long)h.GetContentHash());
        ctx.Write(h.NodeCount);
        for (int q = 0; q < 120; q++)
        {
            var a = new Cell(rng.NextInt(W), rng.NextInt(H));
            var b = new Cell(rng.NextInt(W), rng.NextInt(H));
            WritePath(ctx, g.FindPath(a, b, options));
        }
    }

    private static void GraphAfterEdits(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x8003, 60, 40);
        using var h = new HpaHierarchy(g, new HpaOptions { ChunkSize = 8, AgentSize = 2 });
        var rng = new SeededRandom(0x8004);
        for (int round = 0; round < 8; round++)
        {
            for (int e = 0; e < 9; e++)
            {
                g.SetArea(rng.NextInt(60), rng.NextInt(40), rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(2));
            }
            h.Refresh();
            ctx.Write((long)h.GetContentHash());
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x8005, 90, 50);
        using var h = new HpaHierarchy(g, new HpaOptions { ChunkSize = 10 });
        var rng = new SeededRandom(0x8006);
        for (int q = 0; q < 30; q++)
        {
            int a = rng.NextInt(90 * 50);
            int b = rng.NextInt(90 * 50);
            if (!g.IsWalkable(a % 90, a / 90) || !g.IsWalkable(b % 90, b / 90))
            {
                continue;
            }
            int budget = 1 + rng.NextInt(30);
            var search = new HpaSearch(h);
            search.Begin(a, b);
            int calls = 0;
            while (search.Step(budget) == SearchStatus.Running)
            {
                calls++;
            }
            ctx.Write(calls);
            ctx.Write((int)search.Status);
            ctx.Write(search.PathCost);
            if (search.Status == SearchStatus.Found)
            {
                var p = new int[search.PathLength()];
                search.CopyPath(p);
                foreach (int n in p)
                {
                    ctx.Write(n);
                }
            }
        }
    }

    // Queries are split by index across ctx.Threads workers over the shared read-only grid and written in index order.
    private static void Threads(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x8007, 100, 64);
        var options = new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 12 };
        _ = g.FindPath(new Cell(0, 0), new Cell(0, 0), options); // build the hierarchy before the workers start
        const int Queries = 96;
        var results = new GridPath[Queries];
        var po = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, po, i =>
        {
            var rng = new SeededRandom(0x8100 + (ulong)i);
            var a = new Cell(rng.NextInt(100), rng.NextInt(64));
            var b = new Cell(rng.NextInt(100), rng.NextInt(64));
            results[i] = g.FindPath(a, b, options);
        });
        foreach (GridPath p in results)
        {
            WritePath(ctx, p);
        }
    }
}
