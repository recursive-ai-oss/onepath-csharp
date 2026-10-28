using System;

namespace RecursiveAI.OnePath.Grids;

// Glue between the one-line query and HpaSearch: runs with the option budget and builds the result.
internal static class HpaRunner
{
    internal static GridPath Run(HpaSearch search, int start, int goal, GridOptions options, Func<Cell, WorldPoint>? centre)
    {
        int width = search.Hierarchy.Grid.Width;
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
            return new GridPath(PathStatus.BudgetExhausted, double.PositiveInfinity, null, search.ExpandedCount, expanded, null) { Algorithm = GridAlgorithm.Hpa };
        }
        if (status == SearchStatus.NotFound)
        {
            return new GridPath(PathStatus.NoPath, double.PositiveInfinity, null, search.ExpandedCount, expanded, null) { Algorithm = GridAlgorithm.Hpa };
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
        return new GridPath(PathStatus.Found, search.PathCost, cells, search.ExpandedCount, expanded, world) { Algorithm = GridAlgorithm.Hpa };
    }
}
