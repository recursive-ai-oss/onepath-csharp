using System;

namespace RecursiveAI.OnePath.Grids;

// Glue between the one-line queries and the generic search: delegate adapters and result building.
internal static class SearchRunner
{
    internal struct DelegateStepCost : IStepCost
    {
        public int Width;
        public Func<Cell, Cell, double, double>? Step;
        public Func<Cell, bool>? Blocked;

        public readonly double Cost(int source, int target, double baseCost)
        {
            int ty = target / Width;
            var toCell = new Cell(target - (ty * Width), ty);
            if (Blocked is not null && Blocked(toCell))
            {
                return double.PositiveInfinity;
            }
            if (Step is null)
            {
                return baseCost;
            }
            int fy = source / Width;
            return Step(new Cell(source - (fy * Width), fy), toCell, baseCost);
        }
    }

    internal static GridPath Run<TTopology, TCost, THeuristic>(
        TTopology topology,
        TCost cost,
        THeuristic heuristic,
        GridSearchBuffers buffers,
        int width,
        int start,
        int goal,
        GridOptions options,
        Func<Cell, WorldPoint>? centre)
        where TTopology : struct, IGridTopology
        where TCost : struct, IStepCost
        where THeuristic : struct, IHeuristic
    {
        buffers.RecordExpansions = options.RecordExpansions;
        var search = new AStar<TTopology, TCost, THeuristic>(topology, cost, heuristic, buffers)
        {
            HeuristicWeight = options.HeuristicWeight,
        };
        search.Begin(start, goal);
        SearchStatus status = search.Step(options.IterationBudget);
        Cell[]? expanded = null;
        if (options.RecordExpansions)
        {
            ReadOnlySpan<int> e = buffers.ExpandedNodes;
            expanded = new Cell[e.Length];
            for (int i = 0; i < e.Length; i++)
            {
                expanded[i] = ToCell(e[i], width);
            }
        }
        if (status == SearchStatus.Running)
        {
            return new GridPath(PathStatus.BudgetExhausted, double.PositiveInfinity, null, search.ExpandedCount, expanded, null);
        }
        if (status == SearchStatus.NotFound)
        {
            return new GridPath(PathStatus.NoPath, double.PositiveInfinity, null, search.ExpandedCount, expanded, null);
        }
        int[] nodes = new int[search.PathLength()];
        search.CopyPath(nodes);
        var cells = new Cell[nodes.Length];
        WorldPoint[]? world = centre is null ? null : new WorldPoint[nodes.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            cells[i] = ToCell(nodes[i], width);
            if (world is not null)
            {
                world[i] = centre!(cells[i]);
            }
        }
        return new GridPath(PathStatus.Found, search.PathCost, cells, search.ExpandedCount, expanded, world);
    }

    internal static GridPath RunJump<TJumper, THeuristic>(
        TJumper jumper,
        THeuristic heuristic,
        double unitCost,
        GridSearchBuffers buffers,
        int start,
        int goal,
        GridOptions options,
        Func<Cell, WorldPoint>? centre,
        GridAlgorithm used)
        where TJumper : struct, IJumper
        where THeuristic : struct, IHeuristic
    {
        int width = jumper.Width;
        buffers.RecordExpansions = options.RecordExpansions;
        var search = new JumpSearch<TJumper, THeuristic>(jumper, heuristic, unitCost, buffers);
        search.Begin(start, goal);
        SearchStatus status = search.Step(options.IterationBudget);
        Cell[]? expanded = null;
        if (options.RecordExpansions)
        {
            ReadOnlySpan<int> e = buffers.ExpandedNodes;
            expanded = new Cell[e.Length];
            for (int i = 0; i < e.Length; i++)
            {
                expanded[i] = ToCell(e[i], width);
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
            cells[i] = ToCell(nodes[i], width);
            if (world is not null)
            {
                world[i] = centre!(cells[i]);
            }
        }
        return new GridPath(PathStatus.Found, search.PathCost, cells, search.ExpandedCount, expanded, world) { Algorithm = used };
    }

    internal static Cell ToCell(int node, int width)
    {
        int y = node / width;
        return new Cell(node - (y * width), y);
    }
}
