using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// The grid calls a deterministic host makes every tick: the resumable search handle (path, cost bits and saved state in the middle of
/// a search, on both engines), TryReplan, the content hash, nearest walkable cell and the fixed-point cost.
/// </summary>
internal static class HostScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/host/search-specialised", ctx => Search(ctx, 0x7001, new GridOptions(), false));
        Scenarios.Register("grids/host/search-generic", ctx => Search(ctx, 0x7002, new GridOptions { SpecializedEngines = false }, true));
        Scenarios.Register("grids/host/search-weighted-areas", ctx => Search(ctx, 0x7003, new GridOptions { HeuristicWeight = 1.5, AgentSize = 2 }, true));
        Scenarios.Register("grids/host/replan-hash-nearest", ReplanHashNearest);
    }

    private static Grid MakeGrid(ulong seed, bool areas)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(48, 40);
        for (int y = 0; y < 40; y++)
        {
            for (int x = 0; x < 48; x++)
            {
                if (rng.NextDouble() < 0.2)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (areas)
                {
                    g.SetArea(x, y, (byte)rng.NextInt(4));
                }
            }
        }
        if (areas)
        {
            g.Areas[1] = 0.5;
            g.Areas[2] = 3.0;
            g.Areas[3] = 7.25;
        }
        return g;
    }

    private static bool Pick(Grid g, ref SeededRandom rng, int size, out Cell c)
    {
        for (int i = 0; i < 200; i++)
        {
            c = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
            if (g.CanFit(c, size))
            {
                return true;
            }
        }
        c = default;
        return false;
    }

    private static void Search(ScenarioContext ctx, ulong seed, GridOptions o, bool areas)
    {
        Grid g = MakeGrid(seed, areas);
        var rng = new SeededRandom(seed + 1);
        GridSearch search = g.CreateSearch(o);
        GridSearch restored = g.CreateSearch(o);
        var cells = new Cell[g.CellCount];
        for (int q = 0; q < 24; q++)
        {
            if (!Pick(g, ref rng, o.AgentSize, out Cell s) || !Pick(g, ref rng, o.AgentSize, out Cell t))
            {
                continue;
            }
            search.TryBegin(s, t);
            int slice = 1 + rng.NextInt(60);
            PathStatus st = search.Step(slice);
            byte[] state = search.SaveState();
            ctx.Write((int)st);
            ctx.Write(state);
            while (st == PathStatus.InProgress)
            {
                st = search.Step(97);
            }
            // The copy that was saved mid-search must finish exactly as the uninterrupted one did.
            restored.RestoreState(state);
            PathStatus rs = restored.Status == SearchStatus.Found ? PathStatus.Found : restored.Step(slice);
            while (rs == PathStatus.InProgress)
            {
                rs = restored.Step(31);
            }
            if (rs != st || restored.Iterations != search.Iterations || restored.ExpandedCount != search.ExpandedCount
                || (st == PathStatus.Found && BitConverter.DoubleToInt64Bits(restored.PathCost) != BitConverter.DoubleToInt64Bits(search.PathCost)))
            {
                throw new InvalidOperationException("A restored search ended differently from the uninterrupted one.");
            }
            ctx.Write((int)st);
            ctx.Write(search.Iterations);
            ctx.Write(search.ExpandedCount);
            if (st == PathStatus.Found)
            {
                ctx.Write(BitConverter.DoubleToInt64Bits(search.PathCost));
                int n = search.CopyPath(cells);
                ctx.Write(n);
                for (int i = 0; i < n; i++)
                {
                    ctx.Write((cells[i].Y * g.Width) + cells[i].X);
                }
                if (g.TryGetFixedCost(cells.AsSpan(0, n), out long fixedCost))
                {
                    ctx.Write(fixedCost);
                }
                GridPath p = g.FindPath(s, t, o);
                if (!p.Cells.SequenceEqual(cells.AsSpan(0, n)))
                {
                    throw new InvalidOperationException("GridSearch and FindPath returned different paths.");
                }
            }
        }
    }

    private static void ReplanHashNearest(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x7101, true);
        var rng = new SeededRandom(0x7102);
        Pick(g, ref rng, 1, out Cell s);
        Pick(g, ref rng, 1, out Cell t);
        using GridReplanner r = g.CreateReplanner(s, t);
        var path = new int[g.CellCount];
        ctx.Write(unchecked((long)g.ContentHash()));
        for (int step = 0; step < 60; step++)
        {
            for (int e = 0, n = rng.NextInt(4); e < n; e++)
            {
                var c = new Cell(rng.NextInt(g.Width), rng.NextInt(g.Height));
                if (c != s && c != t)
                {
                    g.SetArea(c.X, c.Y, rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(4));
                }
            }
            PathStatus st = r.TryReplan(1 + rng.NextInt(50));
            while (st == PathStatus.BudgetExhausted)
            {
                st = r.TryReplan(1 + rng.NextInt(50));
            }
            ctx.Write((int)st);
            if (st == PathStatus.Found)
            {
                ctx.Write(BitConverter.DoubleToInt64Bits(r.PathCost));
                int n = r.CopyPath(path);
                ctx.Write(n);
                for (int i = 0; i < n; i++)
                {
                    ctx.Write(path[i]);
                }
            }
            ctx.Write(unchecked((long)g.ContentHash()));
            var around = new Cell(rng.NextInt(g.Width + 10) - 5, rng.NextInt(g.Height + 10) - 5);
            bool ok = g.TryFindNearestWalkable(around, out Cell near, new GridOptions { AgentSize = 1 + (step % 3) });
            ctx.Write(ok ? (near.Y * 1000) + near.X : -1);
        }
    }
}
