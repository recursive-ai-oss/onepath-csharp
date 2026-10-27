using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Flow-field scenarios: integration and direction fields (cost bits and next cells) on generated square and hex maps with several
/// goals, area costs and agent sizes; repair after edit rounds; sliced fields; fields computed on several threads and combined in index order.
/// </summary>
internal static class FlowScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/flow/square", ctx => Fields(ctx, 0x9001, hex: false));
        Scenarios.Register("grids/flow/hex", ctx => Fields(ctx, 0x9002, hex: true));
        Scenarios.Register("grids/flow/repair", Repair);
        Scenarios.Register("grids/flow/sliced", Sliced);
        Scenarios.Register("grids/flow/threads", Threads);
    }

    private static Grid MakeGrid(ulong seed, int w, int h)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        g.Areas[1] = 2.5;
        g.Areas[2] = 0.75;
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                double r = rng.NextDouble();
                if (r < 0.15)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (r < 0.3)
                {
                    g.SetArea(x, y, (byte)(1 + rng.NextInt(2)));
                }
            }
        }
        return g;
    }

    private static HexGrid MakeHex(ulong seed, int w, int h)
    {
        Grid sq = MakeGrid(seed, w, h);
        var hex = HexGrid.From(sq.Cells, w, h, HexLayout.FlatOddColumn);
        hex.Areas.CopyFrom(sq.Areas);
        return hex;
    }

    private static Cell[] Goals(ref SeededRandom rng, int w, int h)
    {
        int k = 1 + rng.NextInt(3);
        var goals = new Cell[k];
        for (int i = 0; i < k; i++)
        {
            goals[i] = new Cell(rng.NextInt(w), rng.NextInt(h));
        }
        return goals;
    }

    private static void WriteField(ScenarioContext ctx, GridFlowField f)
    {
        ctx.Write(f.Settled);
        foreach (double c in f.Costs)
        {
            ctx.Write(c);
        }
        foreach (int n in f.Next)
        {
            ctx.Write(n);
        }
    }

    private static void Fields(ScenarioContext ctx, ulong seed, bool hex)
    {
        const int W = 70;
        const int H = 45;
        var rng = new SeededRandom(seed ^ 0xABCD);
        Grid sq = MakeGrid(seed, W, H);
        HexGrid hg = MakeHex(seed, W, H);
        for (int i = 0; i < 6; i++)
        {
            Cell[] goals = Goals(ref rng, W, H);
            var options = new GridOptions
            {
                AgentSize = hex ? 1 : 1 + (i % 3),
                Connectivity = i % 2 == 0 ? Connectivity.Eight : Connectivity.Four,
                Diagonals = (DiagonalRule)(i % 3),
            };
            using GridFlowField f = hex ? hg.CreateFlowField(goals, options) : sq.CreateFlowField(goals, options);
            WriteField(ctx, f);
        }
    }

    private static void Repair(ScenarioContext ctx)
    {
        const int W = 60;
        const int H = 40;
        Grid g = MakeGrid(0x9003, W, H);
        var rng = new SeededRandom(0x9004);
        Cell[] goals = Goals(ref rng, W, H);
        using GridFlowField f = g.CreateFlowField(goals, new GridOptions { AgentSize = 2 });
        for (int round = 0; round < 8; round++)
        {
            int edits = 1 + rng.NextInt(6);
            for (int e = 0; e < edits; e++)
            {
                g.SetArea(rng.NextInt(W), rng.NextInt(H), rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3));
            }
            f.Refresh();
            WriteField(ctx, f);
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x9005, 50, 30);
        var rng = new SeededRandom(0x9006);
        for (int q = 0; q < 6; q++)
        {
            Cell[] goals = Goals(ref rng, 50, 30);
            int budget = 1 + rng.NextInt(40);
            using GridFlowField f = g.CreateFlowField(goals, new GridOptions { IterationBudget = budget });
            int calls = 0;
            while (f.Status != FlowFieldStatus.Complete)
            {
                f.Step(budget);
                calls++;
            }
            ctx.Write(calls);
            WriteField(ctx, f);
        }
    }

    // One field per goal set, computed by ctx.Threads workers over the shared read-only grid, written in index order.
    private static void Threads(ScenarioContext ctx)
    {
        Grid g = MakeGrid(0x9007, 80, 50);
        _ = g.Clearance;
        const int Fields = 24;
        var rng = new SeededRandom(0x9008);
        var goalSets = new Cell[Fields][];
        for (int i = 0; i < Fields; i++)
        {
            goalSets[i] = Goals(ref rng, 80, 50);
        }
        var costs = new double[Fields][];
        var next = new int[Fields][];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Fields, options, i =>
        {
            using GridFlowField f = g.CreateFlowField(goalSets[i]);
            costs[i] = f.Costs.ToArray();
            next[i] = f.Next.ToArray();
        });
        for (int i = 0; i < Fields; i++)
        {
            foreach (double c in costs[i])
            {
                ctx.Write(c);
            }
            foreach (int n in next[i])
            {
                ctx.Write(n);
            }
        }
    }
}
