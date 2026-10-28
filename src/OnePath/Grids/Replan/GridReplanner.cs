using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// Keeps a shortest path on a <see cref="Grid"/> up to date while the grid changes and the agent moves (tier 1):
/// LPA* or D* Lite behind plain options. Create it with <see cref="Grid.CreateReplanner"/>.
/// <para>
/// It listens to the grid, so <see cref="Grid.SetArea"/> and <see cref="Grid.SetRect(int, int, int, int, byte)"/> edits need no further call;
/// <see cref="Replan()"/> then repairs the search instead of starting again and returns the new path. Edits that the grid cannot see
/// (a <see cref="GridOptions.Blocked"/> or <see cref="GridOptions.StepCost"/> delegate that now answers differently) are announced with
/// <see cref="Invalidate"/>. Editing the grid's <see cref="AreaCosts"/> table restarts the search at the next replan.
/// </para>
/// <para>
/// The options are read once: connectivity, diagonal rule, agent size, heuristic, delegates, and the algorithm
/// (<see cref="GridAlgorithm.LpaStar"/>, otherwise <see cref="GridAlgorithm.DStarLite"/>, which is the better choice when the start moves).
/// The heuristic weight must be 1 (incremental searches are exact only then). Not thread-safe; do not edit the grid while a replan runs.
/// </para>
/// </summary>
public sealed partial class GridReplanner : IGridBatchListener, IDisposable
{
    private readonly Grid _grid;
    private readonly GridOptions _options;
    private readonly GridAlgorithm _algorithm;
    private ReplanSearch _search;
    private int _areaVersion;
    private bool _disposed;

    internal GridReplanner(Grid grid, Cell start, Cell goal, GridOptions options)
    {
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
    public Grid Grid => _grid;

    /// <summary>The current start cell.</summary>
    public Cell Start => ToCell(_search.Start);

    /// <summary>The current goal cell.</summary>
    public Cell Goal => ToCell(_search.Goal);

    /// <summary>Nodes expanded since the search was created or last restarted.</summary>
    public long TotalExpanded => _search.ExpandedCount;

    /// <summary>Queue pops (the iteration-budget unit) since the search was created or last restarted.</summary>
    public long TotalIterations => _search.Iterations;

    private Cell ToCell(int node) => SearchRunner.ToCell(node, _grid.Width);

    private static void CheckCell(Grid grid, Cell c, string name)
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
        SquareTopology topology = _grid.CreateTopology(o.AgentSize, o.Connectivity, o.Diagonals);
        GridHeuristic h = o.Heuristic;
        if (h == GridHeuristic.Auto)
        {
            h = o.Connectivity == Connectivity.Four ? GridHeuristic.Manhattan : GridHeuristic.Octile;
        }
        double scale = _grid.Areas.MinCost;
        _areaVersion = _grid.Areas.Version;
        int w = _grid.Width;
        int a = (start.Y * w) + start.X;
        int b = (goal.Y * w) + goal.X;
        ReplanMode mode = _algorithm == GridAlgorithm.LpaStar ? ReplanMode.LpaStar : ReplanMode.DStarLite;
        if (o.StepCost is null && o.Blocked is null)
        {
            var cost = default(DefaultStepCost);
            return h switch
            {
                GridHeuristic.Zero => new ReplanSearch<SquareTopology, DefaultStepCost, ZeroHeuristic>(topology, cost, default, mode, a, b),
                GridHeuristic.Manhattan => new ReplanSearch<SquareTopology, DefaultStepCost, ManhattanHeuristic>(topology, cost, new ManhattanHeuristic(w, scale), mode, a, b),
                _ => new ReplanSearch<SquareTopology, DefaultStepCost, OctileHeuristic>(topology, cost, new OctileHeuristic(w, scale), mode, a, b),
            };
        }
        else
        {
            var cost = new SearchRunner.DelegateStepCost { Width = w, Step = o.StepCost, Blocked = o.Blocked };
            return h switch
            {
                GridHeuristic.Zero => new ReplanSearch<SquareTopology, SearchRunner.DelegateStepCost, ZeroHeuristic>(topology, cost, default, mode, a, b),
                GridHeuristic.Manhattan => new ReplanSearch<SquareTopology, SearchRunner.DelegateStepCost, ManhattanHeuristic>(topology, cost, new ManhattanHeuristic(w, scale), mode, a, b),
                _ => new ReplanSearch<SquareTopology, SearchRunner.DelegateStepCost, OctileHeuristic>(topology, cost, new OctileHeuristic(w, scale), mode, a, b),
            };
        }
    }

    /// <summary>Moves the start (the agent has moved). D* Lite keeps its earlier work; LPA* starts over.</summary>
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

    /// <summary>Announces that moves into or out of a cell changed for a reason the grid cannot see (a delegate answers differently now).</summary>
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
    /// <returns>The result. <see cref="GridPath.ExpandedCount"/> is the number of nodes this call expanded; <see cref="GridPath.Algorithm"/> the algorithm.</returns>
    public GridPath Replan() => Replan(int.MaxValue);

    /// <summary>
    /// Like <see cref="Replan()"/> but spends at most <paramref name="iterationBudget"/> queue pops; on <see cref="PathStatus.BudgetExhausted"/>
    /// call again to continue. Any sequence of budgets ends with the result of one unsliced call.
    /// </summary>
    /// <param name="iterationBudget">At least 1.</param>
    /// <returns>The result.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The budget is below 1.</exception>
    /// <exception cref="ObjectDisposedException">The replanner was disposed.</exception>
    public GridPath Replan(int iterationBudget)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        ArgumentOutOfRangeException.ThrowIfLessThan(iterationBudget, 1);
        _ = _grid.Clearance; // bring the clearance up to date before the search reads it
        if (_grid.Areas.Version != _areaVersion)
        {
            _search = Build(Start, Goal);
        }
        PathStatus status;
        if (_options.Blocked?.Invoke(Start) == true)
        {
            status = PathStatus.StartInvalid;
        }
        else if (_options.Blocked?.Invoke(Goal) == true)
        {
            status = PathStatus.GoalInvalid;
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
        if (status != PathStatus.Found)
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
            cells[i] = ToCell(nodes[i]);
        }
        return new GridPath(PathStatus.Found, cost, cells, expanded, expandedCells, null) { Algorithm = _algorithm };
    }

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y) => MarkRect(x, y, 1, 1);

    /// <inheritdoc/>
    public void OnRectChanged(int x, int y, int width, int height) => MarkRect(x, y, width, height);

    // A move between two cells reads the two cells, the cells beside a diagonal move, and (for agent size s) the s x s blocks the agent
    // occupies at both ends. An edit at (x, y) therefore changes the moves of the nodes within s cells up-left of it and one cell
    // down-right of it (docs/grids.md); those are the nodes to re-examine.
    private void MarkRect(int x, int y, int width, int height)
    {
        int s = _options.AgentSize;
        int x0 = Math.Max(0, x - s);
        int y0 = Math.Max(0, y - s);
        int x1 = Math.Min(_grid.Width - 1, x + width);
        int y1 = Math.Min(_grid.Height - 1, y + height);
        long area = (long)(x1 - x0 + 1) * (y1 - y0 + 1);
        if (area > _grid.CellCount / 4)
        {
            _search.Restart();
            return;
        }
        int w = _grid.Width;
        for (int j = y0; j <= y1; j++)
        {
            for (int i = x0; i <= x1; i++)
            {
                _search.MarkChanged((j * w) + i, false);
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
