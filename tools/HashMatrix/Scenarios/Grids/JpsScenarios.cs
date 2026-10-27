using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Jump point search scenarios: JPS (block scanning) and JPS+ on generated maps wider than a bitmap word, JPS+ tables
/// before and after cell edits, sliced against unsliced, and queries split across threads. Everything the searches decide
/// (status, cost bits, path, expansion count, table entries) is written.
/// </summary>
internal static class JpsScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/jps/block-scan", ctx => Queries(ctx, 0x7001, GridAlgorithm.Jps));
        Scenarios.Register("grids/jps/jps-plus", ctx => Queries(ctx, 0x7002, GridAlgorithm.JpsPlus));
        Scenarios.Register("grids/jps/jps-plus-tables-after-edits", TablesAfterEdits);
        Scenarios.Register("grids/jps/sliced-equals-unsliced", Sliced);
        Scenarios.Register("grids/jps/threads", Threads);
    }

    private static Grid MakeGrid(ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (rng.NextDouble() < 0.12)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
            }
        }
        // Long walls with gaps: runs that end in forced neighbours.
        for (int k = 0; k < 5; k++)
        {
            int x = 10 + (k * (w / 6));
            int gap = rng.NextInt(h);
            for (int y = 0; y < h; y++)
            {
                if (y != gap && y != ((gap + 1) % h))
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
            }
        }
        for (int k = 0; k < 3; k++)
        {
            int y = 8 + (k * (h / 4));
            int gap = rng.NextInt(w);
            for (int x = 0; x < w; x++)
            {
                if (x != gap && x != ((gap + 1) % w))
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

    private static void Queries(ScenarioContext ctx, ulong seed, GridAlgorithm algorithm)
    {
        const int W = 130;
        const int H = 70;
        Grid g = MakeGrid(seed, W, H);
        var rng = new SeededRandom(seed ^ 0xFFFF);
        var options = new GridOptions { Algorithm = algorithm };
        for (int q = 0; q < 150; q++)
        {
            var a = new Cell(rng.NextInt(W), rng.NextInt(H));
            var b = new Cell(rng.NextInt(W), rng.NextInt(H));
            WritePath(ctx, g.FindPath(a, b, options));
        }
    }

    private static void TablesAfterEdits(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7003, 45, 30);
        using var index = new JpsPlusIndex(g);
        var rng = new SeededRandom(0x7004);
        for (int round = 0; round < 6; round++)
        {
            for (int e = 0; e < 7; e++)
            {
                g.SetWalkable(rng.NextInt(45), rng.NextInt(30), rng.NextInt(3) == 0);
            }
            index.Refresh();
            foreach (short v in index.Tables)
            {
                ctx.Write((int)v);
            }
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7005, 90, 40);
        using var index = new JpsPlusIndex(g);
        var rng = new SeededRandom(0x7006);
        for (int q = 0; q < 20; q++)
        {
            int a = rng.NextInt(90 * 40);
            int b = rng.NextInt(90 * 40);
            if (!g.IsWalkable(a % 90, a / 90) || !g.IsWalkable(b % 90, b / 90))
            {
                continue;
            }
            int budget = 1 + rng.NextInt(30);
            var buffers = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
            var search = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(g.Width), 1.0, buffers);
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

    // Queries are split by index across ctx.Threads workers over the shared read-only grid and written in index order.
    private static void Threads(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7007, 100, 64);
        var jps = new GridOptions { Algorithm = GridAlgorithm.Jps };
        var plus = new GridOptions { Algorithm = GridAlgorithm.JpsPlus };
        _ = g.FindPath(new Cell(0, 0), new Cell(0, 0), jps); // build the shared structures before the workers start
        _ = g.FindPath(new Cell(0, 0), new Cell(0, 0), plus);
        const int Queries = 96;
        var results = new GridPath[Queries];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, options, i =>
        {
            var rng = new SeededRandom(0x7100 + (ulong)i);
            var a = new Cell(rng.NextInt(100), rng.NextInt(64));
            var b = new Cell(rng.NextInt(100), rng.NextInt(64));
            results[i] = g.FindPath(a, b, (i & 1) == 0 ? jps : plus);
        });
        foreach (GridPath p in results)
        {
            WritePath(ctx, p);
        }
    }
}
