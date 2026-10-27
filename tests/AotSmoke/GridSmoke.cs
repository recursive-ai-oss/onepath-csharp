using System;
using RecursiveAI.OnePath.Grids;

/// <summary>A small grid run used by the NativeAOT and WebAssembly smoke apps: tier-1 queries, delegates, hex, area costs, parsing and the sliced generic search.</summary>
internal static class GridSmoke
{
    public static bool Run()
    {
        var grid = Grid.From(new bool[,]
        {
            { true, true, true, true, true },
            { false, false, false, false, true },
            { true, true, true, true, true },
            { true, false, false, false, false },
            { true, true, true, true, true },
        });
        GridPath path = grid.FindPath(new Cell(0, 0), new Cell(0, 4));
        bool ok = path.Found && path.Cells.Length == 13 && path.Cost == 12.0;

        // Options with a delegate cost and a filter (the tier-1 path adapts these onto the struct-generic core).
        GridPath blocked = grid.FindPath(new Cell(0, 0), new Cell(0, 4), new GridOptions { Blocked = c => c.X == 4 && c.Y == 1, StepCost = (_, _, cost) => cost * 2 });
        ok &= blocked.Status == PathStatus.NoPath;

        // World-space query.
        grid.Space = new GridSpace(10, 20, 2);
        GridPath world = grid.FindPath(new WorldPoint(11, 21), new WorldPoint(11, 29));
        ok &= world.Found && world.WorldPoints.Length == world.Cells.Length;

        // Hex grid and area costs.
        var hex = new HexGrid(6, 6, HexLayout.FlatOddColumn);
        hex.Areas[1] = 4.0;
        hex.SetArea(2, 2, 1);
        ok &= hex.FindPath(new Cell(0, 0), new Cell(5, 5)).Found;

        // Moving AI map text and the tier-2 generic search with slicing.
        Grid fromText = MovingAiMap.Parse("type octile\nheight 3\nwidth 3\nmap\n...\n.@.\n...\n").ToGrid();
        var search = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(fromText.CreateTopology(), default, new OctileHeuristic(3));
        search.Begin(0, 8);
        while (search.Step(1) == SearchStatus.Running)
        {
        }
        ok &= search.Status == SearchStatus.Found && search.PathCost == 4.0;

        // Jump point searches: tier-1 option, the generic search over each jumper, and a table repair after an edit.
        foreach (GridAlgorithm algorithm in new[] { GridAlgorithm.Jps, GridAlgorithm.JpsPlus })
        {
            GridPath jump = grid.FindPath(new Cell(0, 0), new Cell(0, 4), new GridOptions { Algorithm = algorithm });
            ok &= jump.Found && jump.Algorithm == algorithm && jump.Cost == 12.0;
        }
        // HPA*: tier-1 option and the hierarchy with an incremental update after an edit.
        GridPath hpa = grid.FindPath(new Cell(0, 0), new Cell(0, 4), new GridOptions { Algorithm = GridAlgorithm.Hpa, HpaChunkSize = 2 });
        ok &= hpa.Found && hpa.Algorithm == GridAlgorithm.Hpa;
        using var hierarchy = new HpaHierarchy(fromText, new HpaOptions { ChunkSize = 2 });
        fromText.SetWalkable(1, 1, true);
        var hpaSearch = new HpaSearch(hierarchy);
        hpaSearch.Begin(0, 8);
        ok &= hpaSearch.Run() == SearchStatus.Found && hpaSearch.PathCost >= 2.0 * GridMath.Sqrt2 && hpaSearch.PathCost <= 4.0;
        fromText.SetWalkable(1, 1, false);

        using var index = new JpsPlusIndex(fromText);
        var plus = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(3));
        plus.Begin(0, 8);
        ok &= plus.Run() == SearchStatus.Found && plus.PathCost == 4.0;
        fromText.SetWalkable(1, 1, true);
        plus.Begin(0, 8);
        ok &= plus.Run() == SearchStatus.Found && plus.PathCost == 2.0 * GridMath.Sqrt2;
        var block = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(new BlockMap(fromText)), new OctileHeuristic(3));
        block.Begin(0, 8);
        ok &= block.Run() == SearchStatus.Found && block.PathCost == 2.0 * GridMath.Sqrt2;

        // Any-angle searches: tier-1 option and the generic search.
        var open = new Grid(30, 30);
        foreach (GridAlgorithm algorithm in new[] { GridAlgorithm.ThetaStar, GridAlgorithm.LazyThetaStar })
        {
            GridPath any = open.FindPath(new Cell(1, 2), new Cell(25, 17), new GridOptions { Algorithm = algorithm });
            ok &= any.Found && any.Algorithm == algorithm && any.Cells.Length == 2 && any.Cost == Math.Sqrt((24.0 * 24.0) + (15.0 * 15.0));
        }
        var theta = new ThetaSearch<EuclideanHeuristic>(fromText, new EuclideanHeuristic(3), ThetaVariant.LazyRequeue);
        theta.Begin(0, 8);
        while (theta.Step(1) == SearchStatus.Running)
        {
        }
        ok &= theta.Status == SearchStatus.Found && theta.PathCost == 2.0 * GridMath.Sqrt2 && GridLineOfSight.Test(fromText, new Cell(0, 0), new Cell(2, 2));
        // Incremental replanning: D* Lite and LPA* behind the tier-1 replanner, a tile edit, and the hex form.
        foreach (GridAlgorithm algorithm in new[] { GridAlgorithm.DStarLite, GridAlgorithm.LpaStar })
        {
            var replanGrid = new Grid(8, 8);
            using GridReplanner replanner = replanGrid.CreateReplanner(new Cell(0, 0), new Cell(7, 0), new GridOptions { Algorithm = algorithm });
            ok &= replanner.Replan().Cost == 7.0;
            replanGrid.SetRect(3, 0, 1, 7, AreaCosts.Wall);
            GridPath around = replanner.Replan();
            ok &= around.Found && Math.Abs(around.Cost - replanGrid.FindPath(new Cell(0, 0), new Cell(7, 0)).Cost) < 1e-9;
            replanner.MoveStart(around.Cells[1]);
            ok &= replanner.Replan(1).Status is PathStatus.BudgetExhausted or PathStatus.Found;
            ok &= replanner.Replan().Found;
        }
        var hexOpen = new HexGrid(6, 6);
        using HexReplanner hexReplanner = hexOpen.CreateReplanner(new Cell(0, 0), new Cell(5, 5));
        double before = hexReplanner.Replan().Cost;
        hexOpen.SetRect(2, 0, 1, 5, AreaCosts.Wall);
        ok &= hexReplanner.Replan().Cost >= before;
        return ok;
    }
}
