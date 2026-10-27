using System.Runtime.CompilerServices;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Incremental replanning: LPA* and D* Lite on square and hex grids under cell and tile edits (status, cost bits, expansion counts and
/// paths of every replan), sliced replans, and clearance after tile edits.
/// </summary>
internal static class ReplanScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/replan/lpa-star", ctx => Square(ctx, 0x8001, GridAlgorithm.LpaStar, 0));
        Scenarios.Register("grids/replan/d-star-lite", ctx => Square(ctx, 0x8002, GridAlgorithm.DStarLite, 0));
        Scenarios.Register("grids/replan/sliced", ctx => Square(ctx, 0x8003, GridAlgorithm.DStarLite, 5));
        Scenarios.Register("grids/replan/hex", Hex);
        Scenarios.Register("grids/replan/clearance-after-tile-edits", Clearance);
    }

    private static void Write(ScenarioContext ctx, GridPath p)
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

    private static void Edit(Grid g, ref SeededRandom rng)
    {
        if (rng.NextInt(3) == 0)
        {
            int x = rng.NextInt(g.Width);
            int y = rng.NextInt(g.Height);
            int w = rng.NextInt(System.Math.Min(5, g.Width - x) + 1);
            int h = rng.NextInt(System.Math.Min(5, g.Height - y) + 1);
            var data = new byte[w * h];
            for (int i = 0; i < data.Length; i++)
            {
                data[i] = rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3);
            }
            g.SetRect(x, y, w, h, data);
        }
        else
        {
            g.SetArea(rng.NextInt(g.Width), rng.NextInt(g.Height), rng.NextInt(3) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3));
        }
    }

    // sliceBudget 0: unsliced replans; otherwise each replan is made in slices of that many iterations.
    private static void Square(ScenarioContext ctx, ulong seed, GridAlgorithm alg, int sliceBudget)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(60, 40);
        g.Areas[1] = 0.5;
        g.Areas[2] = 3.0;
        for (int i = 0; i < 400; i++)
        {
            g.SetArea(rng.NextInt(60), rng.NextInt(40), AreaCosts.Wall);
        }
        var start = new Cell(1, 1);
        var goal = new Cell(58, 38);
        g.SetWalkable(start.X, start.Y, true);
        g.SetWalkable(goal.X, goal.Y, true);
        using GridReplanner r = g.CreateReplanner(start, goal, new GridOptions { Algorithm = alg });
        for (int round = 0; round < 40; round++)
        {
            if (round > 0)
            {
                Edit(g, ref rng);
                Edit(g, ref rng);
            }
            GridPath p = sliceBudget == 0 ? r.Replan() : Slice(r, sliceBudget);
            Write(ctx, p);
            ctx.Write(r.TotalExpanded);
            ctx.Write(r.TotalIterations);
            if (alg == GridAlgorithm.DStarLite && p.Found && p.Cells.Length > 4 && round % 3 == 0)
            {
                r.MoveStart(p.Cells[3]);
            }
        }
    }

    private static GridPath Slice(GridReplanner r, int budget)
    {
        GridPath p;
        do
        {
            p = r.Replan(budget);
        }
        while (p.Status == PathOutcome.BudgetExhausted);
        return p;
    }

    private static void Hex(ScenarioContext ctx)
    {
        var rng = new SeededRandom(0x8004);
        var g = new HexGrid(40, 30, HexLayout.PointyOddRow);
        g.Areas[1] = 2.0;
        for (int i = 0; i < 200; i++)
        {
            g.SetArea(rng.NextInt(40), rng.NextInt(30), AreaCosts.Wall);
        }
        g.SetArea(0, 0, 0);
        g.SetArea(39, 29, 0);
        using HexReplanner r = g.CreateReplanner(new Cell(0, 0), new Cell(39, 29));
        for (int round = 0; round < 40; round++)
        {
            if (round > 0)
            {
                if (rng.NextInt(4) == 0)
                {
                    g.SetRect(rng.NextInt(30), rng.NextInt(20), 4, 3, rng.NextInt(2) == 0 ? AreaCosts.Wall : (byte)1);
                }
                else
                {
                    g.SetArea(rng.NextInt(40), rng.NextInt(30), rng.NextInt(2) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(2));
                }
            }
            GridPath p = r.Replan();
            Write(ctx, p);
            if (p.Found && p.Cells.Length > 3 && round % 4 == 0)
            {
                r.MoveStart(p.Cells[2]);
            }
        }
    }

    private static void Clearance(ScenarioContext ctx)
    {
        var rng = new SeededRandom(0x8005);
        var g = new Grid(300, 90);
        _ = g.Clearance;
        for (int round = 0; round < 12; round++)
        {
            for (int e = 0; e < 4; e++)
            {
                g.SetRect(rng.NextInt(280), rng.NextInt(80), 1 + rng.NextInt(12), 1 + rng.NextInt(8), rng.NextInt(2) == 0 ? AreaCosts.Wall : (byte)0);
            }
            foreach (byte v in g.Clearance)
            {
                ctx.Write((int)v);
            }
        }
    }
}
