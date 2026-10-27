using System;

namespace RecursiveAI.OnePath.Grids;

// Glue between the one-line queries and ThetaSearch: applicability and result building.
internal static class AnyAngleRunner
{
    // Any-angle search assumes what the one-line API can promise: eight moves, no corner cutting, agent size 1, one cost
    // for every area, no per-move callbacks, and an estimate that is valid for straight-line movement.
    internal static bool Applies(Grid grid, GridOptions o, out double unitCost)
    {
        unitCost = 1.0;
        return o.Connectivity == Connectivity.Eight
            && o.Diagonals == DiagonalRule.NoCornerCutting
            && o.AgentSize == 1
            && o.StepCost is null
            && o.Blocked is null
            && o.Heuristic is GridHeuristic.Auto or GridHeuristic.Zero or GridHeuristic.Euclidean
            && grid.Areas.TryGetUniform(out unitCost);
    }

    internal static GridPath Run(Grid grid, GridSearchBuffers buffers, int start, int goal, GridOptions options, double unitCost, Func<Cell, WorldPoint>? centre)
    {
        GridAlgorithm used = options.Algorithm;
        ThetaVariant variant = used == GridAlgorithm.LazyThetaStar ? ThetaVariant.LazyRequeue : ThetaVariant.Theta;
        buffers.RecordExpansions = options.RecordExpansions;
        int width = grid.Width;
        return options.Heuristic == GridHeuristic.Zero
            ? Finish(new ThetaSearch<ZeroHeuristic>(grid, default, variant, unitCost, buffers), width, start, goal, options, centre, used)
            : Finish(new ThetaSearch<EuclideanHeuristic>(grid, new EuclideanHeuristic(width, unitCost), variant, unitCost, buffers), width, start, goal, options, centre, used);
    }

    private static GridPath Finish<THeuristic>(ThetaSearch<THeuristic> search, int width, int start, int goal, GridOptions options, Func<Cell, WorldPoint>? centre, GridAlgorithm used)
        where THeuristic : struct, IHeuristic
    {
        search.HeuristicWeight = options.HeuristicWeight;
        search.Begin(start, goal);
        SearchStatus status = search.Step(options.IterationBudget);
        Cell[]? expanded = null;
        if (options.RecordExpansions)
        {
            ReadOnlySpan<int> e = search.Buffers.ExpandedNodes;
            expanded = new Cell[e.Length];
            for (int i = 0; i < e.Length; i++)
            {
                expanded[i] = SearchRunner.ToCell(e[i], width);
            }
        }
        if (status == SearchStatus.Running)
        {
            return new GridPath(PathStatus.BudgetExhausted, double.PositiveInfinity, null, search.ExpandedCount, expanded, null) { Algorithm = used };
        }
        if (status == SearchStatus.NotFound)
        {
            return new GridPath(PathStatus.NoPath, double.PositiveInfinity, null, search.ExpandedCount, expanded, null) { Algorithm = used };
        }
        int[] nodes = new int[search.PathLength()];
        search.CopyPath(nodes);
        var cells = new Cell[nodes.Length];
        WorldPoint[]? world = centre is null ? null : new WorldPoint[nodes.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            cells[i] = SearchRunner.ToCell(nodes[i], width);
            if (world is not null)
            {
                world[i] = centre!(cells[i]);
            }
        }
        return new GridPath(PathStatus.Found, search.PathCost, cells, search.ExpandedCount, expanded, world) { Algorithm = used };
    }
}
