using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Any-angle scenarios: line-of-sight answers, Theta* and Lazy Theta* queries on generated maps, sliced against unsliced, and
/// queries split across threads. Status, cost bits, waypoints and expansion counts are written.
/// </summary>
internal static class ThetaScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/theta/line-of-sight", LineOfSight);
        Scenarios.Register("grids/theta/theta-star", ctx => Queries(ctx, 0x7101, GridAlgorithm.ThetaStar));
        Scenarios.Register("grids/theta/lazy-theta-star", ctx => Queries(ctx, 0x7102, GridAlgorithm.LazyThetaStar));
        Scenarios.Register("grids/theta/sliced-equals-unsliced", Sliced);
        Scenarios.Register("grids/theta/threads", Threads);
    }

    private static Grid MakeGrid(ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        double density = 0.08 + (rng.NextDouble() * 0.2);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (rng.NextDouble() < density)
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

    private static void LineOfSight(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7100, 90, 60);
        var rng = new SeededRandom(0x7110);
        for (int i = 0; i < 4000; i++)
        {
            var a = new Cell(rng.NextInt(90), rng.NextInt(60));
            var b = new Cell(rng.NextInt(90), rng.NextInt(60));
            ctx.Write(GridLineOfSight.Test(g, a, b) ? 1 : 0);
        }
    }

    private static void Queries(ScenarioContext ctx, ulong seed, GridAlgorithm algorithm)
    {
        const int W = 120;
        const int H = 80;
        Grid g = MakeGrid(seed, W, H);
        var rng = new SeededRandom(seed ^ 0xFFFF);
        var options = new GridOptions { Algorithm = algorithm };
        var zero = new GridOptions { Algorithm = algorithm, Heuristic = GridHeuristic.Zero };
        for (int q = 0; q < 120; q++)
        {
            var a = new Cell(rng.NextInt(W), rng.NextInt(H));
            var b = new Cell(rng.NextInt(W), rng.NextInt(H));
            WritePath(ctx, g.FindPath(a, b, (q & 7) == 7 ? zero : options));
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7103, 80, 50);
        var rng = new SeededRandom(0x7104);
        foreach (ThetaVariant variant in new[] { ThetaVariant.Theta, ThetaVariant.Lazy, ThetaVariant.LazyRequeue })
        {
            for (int q = 0; q < 15; q++)
            {
                int a = rng.NextInt(80 * 50);
                int b = rng.NextInt(80 * 50);
                if (!g.IsWalkable(a % 80, a / 80) || !g.IsWalkable(b % 80, b / 80))
                {
                    continue;
                }
                int budget = 1 + rng.NextInt(40);
                var buffers = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
                var search = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(g.Width), variant, 1.0, buffers);
                search.Begin(a, b);
                int calls = 0;
                while (search.Step(budget) == SearchStatus.Running)
                {
                    calls++;
                }
                ctx.Write(calls);
                ctx.Write((int)search.Status);
                ctx.Write(search.PathCost);
                foreach (int n in buffers.ExpandedNodes)
                {
                    ctx.Write(n);
                }
            }
        }
    }

    // Queries are split by index across ctx.Threads workers over the shared read-only grid and written in index order.
    private static void Threads(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7105, 100, 64);
        var theta = new GridOptions { Algorithm = GridAlgorithm.ThetaStar };
        var lazy = new GridOptions { Algorithm = GridAlgorithm.LazyThetaStar };
        const int Queries = 96;
        var results = new GridPath[Queries];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, options, i =>
        {
            var rng = new SeededRandom(0x7200 + (ulong)i);
            var a = new Cell(rng.NextInt(100), rng.NextInt(64));
            var b = new Cell(rng.NextInt(100), rng.NextInt(64));
            results[i] = g.FindPath(a, b, (i & 1) == 0 ? theta : lazy);
        });
        foreach (GridPath p in results)
        {
            WritePath(ctx, p);
        }
    }
}
