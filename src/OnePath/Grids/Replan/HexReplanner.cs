using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// Keeps a shortest path on a <see cref="HexGrid"/> up to date while the grid changes and the agent moves (tier 1): the hex
/// counterpart of <see cref="GridReplanner"/>, with the same behaviour and options (agent size must be 1). Create it with
/// <see cref="HexGrid.CreateReplanner"/>.
/// </summary>
public sealed partial class HexReplanner : IGridBatchListener, IDisposable
{
    private readonly HexGrid _grid;
    private readonly GridOptions _options;
    private readonly GridAlgorithm _algorithm;
    private ReplanSearch _search;
    private int _areaVersion;
    private bool _disposed;

    internal HexReplanner(HexGrid grid, Cell start, Cell goal, GridOptions options)
    {
        if (options.AgentSize != 1)
        {
            throw new ArgumentException("Hex grids support only agent size 1.", nameof(options));
        }
        if (options.HeuristicWeight != 1.0)
        {
            throw new ArgumentException("Incremental searches need a heuristic weight of 1.", nameof(options));
        }
        CheckCell(grid, start, nameof(start));
        CheckCell(grid, goal, nameof(goal));
        _grid = grid;
        _options = options.Clone();
        _algorithm = options.Algorithm == GridAlgorithm.LpaStar ? GridAlgorithm.LpaStar : GridAlgorithm.DStarLite;
        _search = Build(start, goal);
        grid.AddListener(this);
    }

    /// <summary>The algorithm in use: <see cref="GridAlgorithm.LpaStar"/> or <see cref="GridAlgorithm.DStarLite"/>.</summary>
    public GridAlgorithm Algorithm => _algorithm;

    /// <summary>The grid being followed.</summary>
    public HexGrid Grid => _grid;

    /// <summary>The current start cell (column, row).</summary>
    public Cell Start => SearchRunner.ToCell(_search.Start, _grid.Width);

    /// <summary>The current goal cell (column, row).</summary>
    public Cell Goal => SearchRunner.ToCell(_search.Goal, _grid.Width);

    /// <summary>Nodes expanded since the search was created or last restarted.</summary>
    public long TotalExpanded => _search.ExpandedCount;

    /// <summary>Queue pops (the iteration-budget unit) since the search was created or last restarted.</summary>
    public long TotalIterations => _search.Iterations;

    private static void CheckCell(HexGrid grid, Cell c, string name)
    {
        if (!grid.Contains(c.X, c.Y))
        {
            throw new ArgumentOutOfRangeException(name, c, $"Cell ({c.X}, {c.Y}) is outside the {grid.Width}x{grid.Height} grid.");
        }
    }

    private ReplanSearch Build(Cell start, Cell goal)
    {
        ReplanSearch search = BuildCore(start, goal);
        search.RecordExpansions = _options.RecordExpansions;
        return search;
    }

    private ReplanSearch BuildCore(Cell start, Cell goal)
    {
        GridOptions o = _options;
        HexTopology topology = _grid.CreateTopology();
        bool zero = o.Heuristic == GridHeuristic.Zero;
        double scale = _grid.Areas.MinCost;
        _areaVersion = _grid.Areas.Version;
        int w = _grid.Width;
        int a = (start.Y * w) + start.X;
        int b = (goal.Y * w) + goal.X;
        ReplanMode mode = _algorithm == GridAlgorithm.LpaStar ? ReplanMode.LpaStar : ReplanMode.DStarLite;
        if (o.StepCost is null && o.Blocked is null)
        {
            var cost = default(DefaultStepCost);
            return zero
                ? new ReplanSearch<HexTopology, DefaultStepCost, ZeroHeuristic>(topology, cost, default, mode, a, b)
                : new ReplanSearch<HexTopology, DefaultStepCost, HexHeuristic>(topology, cost, new HexHeuristic(w, _grid.Layout, scale), mode, a, b);
        }
        else
        {
            var cost = new SearchRunner.DelegateStepCost { Width = w, Step = o.StepCost, Blocked = o.Blocked };
            return zero
                ? new ReplanSearch<HexTopology, SearchRunner.DelegateStepCost, ZeroHeuristic>(topology, cost, default, mode, a, b)
                : new ReplanSearch<HexTopology, SearchRunner.DelegateStepCost, HexHeuristic>(topology, cost, new HexHeuristic(w, _grid.Layout, scale), mode, a, b);
        }
    }

    /// <summary>Moves the start. D* Lite keeps its earlier work; LPA* starts over.</summary>
    /// <param name="start">The new start cell.</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void MoveStart(Cell start)
    {
        CheckCell(_grid, start, nameof(start));
        _search.MoveStart((start.Y * _grid.Width) + start.X);
    }

    /// <summary>Changes the goal; the search starts over.</summary>
    /// <param name="goal">The new goal cell.</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void SetGoal(Cell goal)
    {
        CheckCell(_grid, goal, nameof(goal));
        _search.SetGoal((goal.Y * _grid.Width) + goal.X);
    }

    /// <summary>Announces that moves into or out of a cell changed for a reason the grid cannot see.</summary>
    /// <param name="cell">The cell.</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void Invalidate(Cell cell)
    {
        CheckCell(_grid, cell, nameof(cell));
        _search.MarkChanged((cell.Y * _grid.Width) + cell.X, true);
    }

    /// <summary>Forgets everything and searches from scratch at the next replan.</summary>
    public void InvalidateAll() => _search.Restart();

    /// <summary>Repairs the search for all changes since the last call and returns the current best path.</summary>
    /// <returns>The result.</returns>
    public GridPath Replan() => Replan(int.MaxValue);

    /// <summary>Like <see cref="Replan()"/> with an iteration budget; see <see cref="GridReplanner.Replan(int)"/>.</summary>
    /// <param name="iterationBudget">At least 1.</param>
    /// <returns>The result.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The budget is below 1.</exception>
    /// <exception cref="ObjectDisposedException">The replanner was disposed.</exception>
    public GridPath Replan(int iterationBudget)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentOutOfRangeException.ThrowIfLessThan(iterationBudget, 1);
        if (_grid.Areas.Version != _areaVersion)
        {
            _search = Build(Start, Goal);
        }
        PathOutcome status;
        if (_options.Blocked?.Invoke(Start) == true)
        {
            status = PathOutcome.StartInvalid;
        }
        else if (_options.Blocked?.Invoke(Goal) == true)
        {
            status = PathOutcome.GoalInvalid;
        }
        else
        {
            status = _search.Replan(iterationBudget);
        }
        int expanded = _search.LastExpanded;
        Cell[]? expandedCells = null;
        if (_options.RecordExpansions)
        {
            ReadOnlySpan<int> e = _search.LastExpandedNodes;
            expandedCells = new Cell[e.Length];
            for (int i = 0; i < e.Length; i++)
            {
                expandedCells[i] = SearchRunner.ToCell(e[i], _grid.Width);
            }
        }
        if (status != PathOutcome.Found)
        {
            return new GridPath(status, double.PositiveInfinity, null, expanded, expandedCells, null) { Algorithm = _algorithm };
        }
        int[]? nodes = _search.ExtractPath(out double cost);
        if (nodes is null)
        {
            throw new InvalidOperationException("The replanned search has a cost but no path to it; this is a bug.");
        }
        var cells = new Cell[nodes.Length];
        for (int i = 0; i < nodes.Length; i++)
        {
            cells[i] = SearchRunner.ToCell(nodes[i], _grid.Width);
        }
        return new GridPath(PathOutcome.Found, cost, cells, expanded, expandedCells, null) { Algorithm = _algorithm };
    }

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y) => _search.MarkChanged((y * _grid.Width) + x, true);

    /// <inheritdoc/>
    public void OnRectChanged(int x, int y, int width, int height)
    {
        if ((long)width * height > _grid.CellCount / 4)
        {
            _search.Restart();
            return;
        }
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                _search.MarkChanged(((y + j) * _grid.Width) + x + i, true);
            }
        }
    }

    /// <summary>Stops following the grid.</summary>
    public void Dispose()
    {
        if (!_disposed)
        {
            _disposed = true;
            _grid.RemoveListener(this);
        }
    }
}
