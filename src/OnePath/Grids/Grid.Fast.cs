using System;
using System.Threading;

using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

public sealed partial class Grid
{
    private OctileMap? _octileMap;
    private OctileAStar? _fastAStar;
    private OctileJump<BitFinder>? _fastBit;
    private OctileJump<TableFinder>? _fastTable;

    // The specialised searches serve what the one-line API can promise: eight moves, no corner cutting, agent size 1, one cost for
    // every area, no per-move callbacks, an exact (weight 1) octile or zero estimate.
    private bool FastApplies(GridOptions o, out double unitCost)
    {
        unitCost = 1.0;
        return o.SpecializedEngines
            && o.Connectivity == Connectivity.Eight
            && o.Diagonals == DiagonalRule.NoCornerCutting
            && o.AgentSize == 1
            && o.StepCost is null
            && o.Blocked is null
            && o.HeuristicWeight == 1.0
            && o.Heuristic is GridHeuristic.Auto or GridHeuristic.Octile or GridHeuristic.Zero
            && Areas.TryGetUniform(out unitCost);
    }

    // True when FindPath and GridSearch serve plain A* with the specialised octile engine under these options.
    internal bool UsesSpecializedAStar(GridOptions o, out double unitCost)
    {
        if (o.Algorithm == GridAlgorithm.AStar)
        {
            return FastApplies(o, out unitCost);
        }
        unitCost = 1.0;
        return false;
    }

    // After a state restore grid versions can repeat, so nothing derived from the grid may be trusted.
    private void DropOctileCaches()
    {
        _octileMap?.Dispose();
        _octileMap = null;
        _fastAStar = null;
        _fastBit = null;
        _fastTable = null;
    }

    private OctileMap GetOctileMap()
    {
        lock (_sync)
        {
            return _octileMap ??= new OctileMap(this);
        }
    }

    private GridPath FindPathFastAStar(int a, int b, GridOptions o, double unitCost, Func<Cell, WorldPoint>? centre)
    {
        OctileAStar engine = Interlocked.Exchange(ref _fastAStar, null) ?? new OctileAStar(this);
        try
        {
            SearchStatus status = engine.Run(a, b, o.Heuristic == GridHeuristic.Zero, o.IterationBudget, o.RecordExpansions);
            return BuildFastPath(engine, status, unitCost, o, centre, GridAlgorithm.AStar, false);
        }
        finally
        {
            _fastAStar = engine;
        }
    }

    private GridPath FindPathFastJump(int a, int b, GridOptions o, double unitCost, Func<Cell, WorldPoint>? centre, bool plus)
    {
        bool zero = o.Heuristic == GridHeuristic.Zero;
        if (plus)
        {
            OctileJump<TableFinder>? engine = Interlocked.Exchange(ref _fastTable, null);
            if (engine is null)
            {
                OctileMap map = GetOctileMap();
                lock (_sync)
                {
                    _jpsPlus ??= new JpsPlusIndex(this);
                    _jpsPlus.Refresh();
                    engine = new OctileJump<TableFinder>(new TableFinder(_jpsPlus, map, false), Width, Height);
                }
            }
            else
            {
                lock (_sync)
                {
                    _jpsPlus!.Refresh();
                }
            }
            try
            {
                SearchStatus status = engine.Run(a, b, zero, o.IterationBudget, o.RecordExpansions);
                return BuildFastPath(engine, status, unitCost, o, centre, GridAlgorithm.JpsPlus, true);
            }
            finally
            {
                _fastTable = engine;
            }
        }
        else
        {
            OctileJump<BitFinder> engine = Interlocked.Exchange(ref _fastBit, null) ?? new OctileJump<BitFinder>(new BitFinder(GetOctileMap(), false), Width, Height);
            try
            {
                SearchStatus status = engine.Run(a, b, zero, o.IterationBudget, o.RecordExpansions);
                return BuildFastPath(engine, status, unitCost, o, centre, GridAlgorithm.Jps, true);
            }
            finally
            {
                _fastBit = engine;
            }
        }
    }

    private GridPath BuildFastPath<TEngine>(TEngine engine, SearchStatus status, double unitCost, GridOptions o, Func<Cell, WorldPoint>? centre, GridAlgorithm used, bool reportAlgorithm)
        where TEngine : class, IOctileEngine
    {
        Cell[]? expanded = null;
        if (o.RecordExpansions)
        {
            var e = engine.Expanded;
            expanded = new Cell[e.Count];
            for (int i = 0; i < expanded.Length; i++)
            {
                expanded[i] = SearchRunner.ToCell(e[i], Width);
            }
        }
        PathStatus outcome = status switch
        {
            SearchStatus.Found => PathStatus.Found,
            SearchStatus.NotFound => PathStatus.NoPath,
            _ => PathStatus.BudgetExhausted,
        };
        if (outcome != PathStatus.Found)
        {
            return reportAlgorithm
                ? new GridPath(outcome, double.PositiveInfinity, null, engine.ExpandedCount, expanded, null) { Algorithm = used }
                : new GridPath(outcome, double.PositiveInfinity, null, engine.ExpandedCount, expanded, null);
        }
        int[] nodes = new int[engine.PathLength()];
        engine.CopyPath(nodes);
        var cells = new Cell[nodes.Length];
        WorldPoint[]? world = centre is null ? null : new WorldPoint[nodes.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            cells[i] = SearchRunner.ToCell(nodes[i], Width);
            if (world is not null)
            {
                world[i] = centre!(cells[i]);
            }
        }
        return reportAlgorithm
            ? new GridPath(PathStatus.Found, unitCost * engine.PathCost, cells, engine.ExpandedCount, expanded, world) { Algorithm = used }
            : new GridPath(PathStatus.Found, unitCost * engine.PathCost, cells, engine.ExpandedCount, expanded, world);
    }
}
