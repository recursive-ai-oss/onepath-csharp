using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Grid scenarios: A* on generated maps (square, hex, area costs, agent sizes), sliced against unsliced,
/// and queries split across threads. Every result (status, cost bits, path, expansion count) is written, so any
/// difference in tie-breaking or arithmetic between platforms changes the hash.
/// </summary>
internal static class GridScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/astar/eight-connected", ctx => Square(ctx, 0x1001, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, false));
        Scenarios.Register("grids/astar/four-connected", ctx => Square(ctx, 0x1002, Connectivity.Four, DiagonalRule.NoCornerCutting, 1, false));
        Scenarios.Register("grids/astar/no-squeezing", ctx => Square(ctx, 0x1003, Connectivity.Eight, DiagonalRule.NoSqueezing, 1, false));
        Scenarios.Register("grids/astar/area-costs", ctx => Square(ctx, 0x1004, Connectivity.Eight, DiagonalRule.NoCornerCutting, 1, true));
        Scenarios.Register("grids/astar/clearance-size3", ctx => Square(ctx, 0x1005, Connectivity.Eight, DiagonalRule.NoCornerCutting, 3, true));
        Scenarios.Register("grids/astar/hex-pointy-odd", ctx => Hex(ctx, 0x2001, HexLayout.PointyOddRow));
        Scenarios.Register("grids/astar/hex-flat-even", ctx => Hex(ctx, 0x2002, HexLayout.FlatEvenColumn));
        Scenarios.Register("grids/clearance/annotation", Clearance);
        Scenarios.Register("grids/astar/sliced-equals-unsliced", Sliced);
        Scenarios.Register("grids/astar/threads", Threads);
        Scenarios.Register("grids/world/mapping", WorldMapping);
    }

    private static Grid MakeGrid(ulong seed, int w, int h, bool areas)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        // Blobs of walls on a lightly cluttered field, plus a few long walls with gaps.
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (rng.NextDouble() < 0.18)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (areas)
                {
                    g.SetArea(x, y, (byte)rng.NextInt(4));
                }
            }
        }
        for (int k = 0; k < 4; k++)
        {
            int x = 8 + (k * (w / 5));
            int gap = rng.NextInt(h);
            for (int y = 0; y < h; y++)
            {
                if (y != gap && y != ((gap + 1) % h))
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
            }
        }
        if (areas)
        {
            g.Areas[1] = 0.5;
            g.Areas[2] = 3.0;
            g.Areas[3] = 9.75;
        }
        return g;
    }

    private static void WritePath(ScenarioContext ctx, GridPath p)
    {
        ctx.Write((int)p.Status);
        ctx.Write(p.Cost);
        ctx.Write(p.ExpandedCount);
        ctx.Write(p.Cells.Length);
        foreach (Cell c in p.Cells)
        {
            ctx.Write(c.X);
            ctx.Write(c.Y);
        }
    }

    private static void Square(ScenarioContext ctx, ulong seed, Connectivity conn, DiagonalRule rule, int size, bool areas)
    {
        Grid g = MakeGrid(seed, 64, 64, areas);
        var rng = new SeededRandom(seed ^ 0xFFFF);
        var options = new GridOptions { Connectivity = conn, Diagonals = rule, AgentSize = size };
        for (int q = 0; q < 200; q++)
        {
            var a = new Cell(rng.NextInt(64), rng.NextInt(64));
            var b = new Cell(rng.NextInt(64), rng.NextInt(64));
            WritePath(ctx, g.FindPath(a, b, options));
        }
    }

    private static void Hex(ScenarioContext ctx, ulong seed, HexLayout layout)
    {
        Grid sq = MakeGrid(seed, 48, 48, true);
        var g = HexGrid.From(sq.Cells, 48, 48, layout);
        g.Areas.CopyFrom(sq.Areas);
        var rng = new SeededRandom(seed ^ 0xFFFF);
        for (int q = 0; q < 200; q++)
        {
            var a = new Cell(rng.NextInt(48), rng.NextInt(48));
            var b = new Cell(rng.NextInt(48), rng.NextInt(48));
            WritePath(ctx, g.FindPath(a, b));
        }
    }

    private static void Clearance(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x3001, 70, 50, false);
        foreach (byte c in g.Clearance)
        {
            ctx.Write((int)c);
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x4001, 40, 40, true);
        var rng = new SeededRandom(0x4002);
        SquareTopology topo = g.CreateTopology();
        for (int q = 0; q < 20; q++)
        {
            int a = rng.NextInt(1600);
            int b = rng.NextInt(1600);
            if (!topo.IsWalkable(a) || !topo.IsWalkable(b))
            {
                continue;
            }
            int budget = 1 + rng.NextInt(40);
            var buffers = new GridSearchBuffers(g.CellCount) { RecordExpansions = true };
            var search = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topo, default, new OctileHeuristic(g.Width, g.Areas.MinCost), buffers);
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

    // Queries are split by index across ctx.Threads workers, each with its own buffers over the shared
    // read-only grid, and written in index order: the bytes must not depend on the thread count.
    private static void Threads(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x5001, 64, 64, true);
        g.Clearance.ToArray(); // compute once before the workers start
        const int Queries = 96;
        var results = new GridPath[Queries];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, options, i =>
        {
            var rng = new SeededRandom(0x6000 + (ulong)i);
            var a = new Cell(rng.NextInt(64), rng.NextInt(64));
            var b = new Cell(rng.NextInt(64), rng.NextInt(64));
            results[i] = g.FindPath(a, b);
        });
        foreach (GridPath p in results)
        {
            WritePath(ctx, p);
        }
    }

    private static void WorldMapping(ScenarioContext ctx)
    {
        var g = new Grid(16, 12) { Space = new GridSpace(-1000.125, 2048.5, 0.3) };
        var hex = new HexGrid(12, 10, HexLayout.FlatOddColumn) { Space = new GridSpace(5.5, -7.25, 1.7) };
        for (int y = 0; y < 12; y++)
        {
            for (int x = 0; x < 16; x++)
            {
                WorldPoint p = g.CellCenter(new Cell(x, y));
                ctx.Write(p.X);
                ctx.Write(p.Y);
                ctx.Write(g.TryGetCell(new WorldPoint(p.X + 0.07, p.Y - 0.07), out Cell c) ? (c.X * 100) + c.Y : -1);
            }
        }
        for (int y = 0; y < 10; y++)
        {
            for (int x = 0; x < 12; x++)
            {
                WorldPoint p = hex.CellCenter(new Cell(x, y));
                ctx.Write(p.X);
                ctx.Write(p.Y);
                ctx.Write(hex.TryGetCell(new WorldPoint(p.X + 0.2, p.Y + 0.2), out Cell c) ? (c.X * 100) + c.Y : -1);
            }
        }
    }
}
