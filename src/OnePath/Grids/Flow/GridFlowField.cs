using System;
using System.Collections.Generic;
using System.Threading;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

// Cost hook of the one-line flow fields: the options' blocked filter applies to both ends of a move (a blocked cell
// is isolated) and the step-cost delegate sees (from, to, grid cost).
internal struct FlowStepCost : IStepCost
{
    public int Width;
    public Func<Cell, Cell, double, double>? Step;
    public Func<Cell, bool>? Blocked;

    public readonly double Cost(int source, int target, double baseCost)
    {
        if (Step is null && Blocked is null)
        {
            return baseCost;
        }
        int sy = source / Width;
        var from = new Cell(source - (sy * Width), sy);
        int ty = target / Width;
        var to = new Cell(target - (ty * Width), ty);
        if (Blocked is not null && (Blocked(from) || Blocked(to)))
        {
            return double.PositiveInfinity;
        }
        return Step is null ? baseCost : Step(from, to, baseCost);
    }
}

internal interface IFlowDriver
{
    FlowFieldStatus Status { get; }

    int Settled { get; }

    ReadOnlySpan<double> Costs { get; }

    ReadOnlySpan<int> Next { get; }

    void Begin(ReadOnlySpan<int> goals);

    FlowFieldStatus Step(int iterations);

    FlowFieldStatus Run(CancellationToken token);

    void BeginRepair(ReadOnlySpan<int> dirty);

    void WriteState(StateWriter w);

    void ReadState(ref StateReader r);
}

internal sealed class FlowDriver<TTopology> : IFlowDriver
    where TTopology : struct, IGridTopology
{
    private readonly FlowField<TTopology, FlowStepCost> _field;

    public FlowDriver(TTopology topology, FlowStepCost cost) => _field = new FlowField<TTopology, FlowStepCost>(topology, cost);

    public FlowFieldStatus Status => _field.Status;

    public int Settled => _field.Settled;

    public ReadOnlySpan<double> Costs => _field.Costs;

    public ReadOnlySpan<int> Next => _field.Next;

    public void Begin(ReadOnlySpan<int> goals) => _field.Begin(goals);

    public FlowFieldStatus Step(int iterations) => _field.Step(iterations);

    public FlowFieldStatus Run(CancellationToken token) => _field.Run(token);

    public void BeginRepair(ReadOnlySpan<int> dirty) => _field.BeginRepair(dirty);

    public void WriteState(StateWriter w) => _field.WriteState(w);

    public void ReadState(ref StateReader r) => _field.ReadState(ref r);
}

/// <summary>
/// A flow field over a <see cref="Grid"/> or <see cref="HexGrid"/> for many agents heading to a set of goal cells: the exact
/// least cost from every cell to the nearest goal and, per cell, the neighbour to step to (see
/// <see cref="FlowField{TTopology, TCost}"/> for the normative rules). Create one with
/// <see cref="Grid.CreateFlowField(ReadOnlySpan{Cell}, GridOptions?)"/> or <see cref="HexGrid.CreateFlowField(ReadOnlySpan{Cell}, GridOptions?)"/>.
/// <para>
/// The options that apply are connectivity, diagonal rule, agent size, step cost, blocked filter and iteration budget (the
/// field is computed in one go unless the budget stops it; then call <see cref="Step"/> or <see cref="Run"/>). The direction
/// field is eight-way (the neighbour to move to), not a continuous gradient; <see cref="GetDirection"/> gives that move as a
/// unit vector, so following it from any reachable cell reaches a goal at exactly the cell's integration cost.
/// </para>
/// <para>
/// The field watches its grid: after cells change, <see cref="Refresh"/> repairs only the affected region (equal to a full
/// recompute); a change to an area multiplier recomputes everything. Changes the grid cannot report (the filter and
/// cost delegates giving different answers) need <see cref="Recompute"/>. Dispose the field to stop watching.
/// </para>
/// </summary>
public sealed partial class GridFlowField : IGridChangeListener, IDisposable
{
    private readonly Grid? _square;
    private readonly HexGrid? _hex;
    private readonly IFlowDriver _driver;
    private readonly int _width;
    private readonly int _height;
    private readonly int _reach;
    private readonly int[] _goals;
    private readonly List<int> _pending = [];
    private int _areasVersion;
    private bool _disposed;

    internal GridFlowField(Grid grid, ReadOnlySpan<Cell> goals, GridOptions o)
    {
        _square = grid;
        _width = grid.Width;
        _height = grid.Height;
        _reach = o.AgentSize;
        _goals = ToNodes(goals, _width, _height);
        _ = grid.Clearance; // make sure the clearance is current before the view captures it
        SquareTopology topology = grid.CreateTopology(o.AgentSize, o.Connectivity, o.Diagonals);
        _driver = new FlowDriver<SquareTopology>(topology, new FlowStepCost { Width = _width, Step = o.StepCost, Blocked = o.Blocked });
        Start(o.IterationBudget);
        _areasVersion = grid.Areas.Version;
        grid.AddListener(this);
    }

    internal GridFlowField(HexGrid grid, ReadOnlySpan<Cell> goals, GridOptions o)
    {
        if (o.AgentSize != 1)
        {
            throw new ArgumentException("Hex grids support only agent size 1.", nameof(o));
        }
        _hex = grid;
        _width = grid.Width;
        _height = grid.Height;
        _reach = 1;
        _goals = ToNodes(goals, _width, _height);
        _driver = new FlowDriver<HexTopology>(grid.CreateTopology(), new FlowStepCost { Width = _width, Step = o.StepCost, Blocked = o.Blocked });
        Start(o.IterationBudget);
        _areasVersion = grid.Areas.Version;
        grid.AddListener(this);
    }

    private static int[] ToNodes(ReadOnlySpan<Cell> goals, int width, int height)
    {
        if (goals.IsEmpty)
        {
            throw new ArgumentException("At least one goal cell is required.", nameof(goals));
        }
        var nodes = new int[goals.Length];
        for (int i = 0; i < goals.Length; i++)
        {
            Cell c = goals[i];
            if ((uint)c.X >= (uint)width || (uint)c.Y >= (uint)height)
            {
                throw new ArgumentOutOfRangeException(nameof(goals), c, $"Goal cell ({c.X}, {c.Y}) is outside the {width}x{height} grid.");
            }
            nodes[i] = (c.Y * width) + c.X;
        }
        return nodes;
    }

    private void Start(int budget)
    {
        _driver.Begin(_goals);
        _driver.Step(budget);
    }

    /// <summary>Where the computation stands; <see cref="FlowFieldStatus.Running"/> only if the iteration budget stopped it.</summary>
    public FlowFieldStatus Status => _driver.Status;

    /// <summary>Cells settled by the current computation or repair.</summary>
    public int Settled => _driver.Settled;

    /// <summary>The integration field, row-major: least cost to the nearest goal, +∞ if unreachable. Final once complete.</summary>
    public ReadOnlySpan<double> Costs => _driver.Costs;

    /// <summary>The direction field, row-major, as the index of the cell to step to, or −1 for goals and unreachable cells. Final once complete.</summary>
    public ReadOnlySpan<int> Next => _driver.Next;

    /// <summary>Continues a field stopped by the iteration budget.</summary>
    /// <param name="iterations">Cells to settle at most; at least 1.</param>
    /// <returns>The status afterwards; a sliced field ends equal to an unsliced one.</returns>
    public FlowFieldStatus Step(int iterations) => _driver.Step(iterations);

    /// <summary>Runs the computation to completion.</summary>
    /// <param name="cancellationToken">Checked between batches of iterations.</param>
    /// <returns><see cref="FlowFieldStatus.Complete"/>.</returns>
    /// <exception cref="OperationCanceledException">Cancelled.</exception>
    public FlowFieldStatus Run(CancellationToken cancellationToken = default) => _driver.Run(cancellationToken);

    /// <summary>The least cost from a cell to the nearest goal.</summary>
    /// <param name="cell">The cell.</param>
    /// <returns>The cost, +∞ for unreachable cells and cells outside the grid.</returns>
    public double GetCost(Cell cell) => InGrid(cell) ? _driver.Costs[(cell.Y * _width) + cell.X] : double.PositiveInfinity;

    /// <summary>The least cost from the cell containing a world position.</summary>
    /// <param name="position">World position, each coordinate within ±2^21.</param>
    /// <returns>The cost, +∞ if the position is outside the grid or unreachable.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    public double GetCost(WorldPoint position) => TryGetCell(position, out Cell c) ? GetCost(c) : double.PositiveInfinity;

    /// <summary>The cell an agent on <paramref name="cell"/> steps to.</summary>
    /// <param name="cell">The cell.</param>
    /// <param name="next">The neighbouring cell.</param>
    /// <returns>False for goals, unreachable cells and cells outside the grid.</returns>
    public bool TryGetNext(Cell cell, out Cell next)
    {
        next = default;
        if (!InGrid(cell))
        {
            return false;
        }
        int n = _driver.Next[(cell.Y * _width) + cell.X];
        if (n < 0)
        {
            return false;
        }
        int y = n / _width;
        next = new Cell(n - (y * _width), y);
        return true;
    }

    /// <summary>
    /// The direction of the move out of a cell as a unit vector in world axes (x right, y down on a square grid), the
    /// vector from this cell's centre towards the next one's.
    /// </summary>
    /// <param name="cell">The cell.</param>
    /// <param name="x">Unit vector x; 0 if there is no move.</param>
    /// <param name="y">Unit vector y; 0 if there is no move.</param>
    /// <returns>False for goals, unreachable cells and cells outside the grid.</returns>
    public bool GetDirection(Cell cell, out double x, out double y)
    {
        x = 0.0;
        y = 0.0;
        if (!TryGetNext(cell, out Cell next))
        {
            return false;
        }
        WorldPoint a = Centre(cell);
        WorldPoint b = Centre(next);
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        double len = Math.Sqrt((dx * dx) + (dy * dy));
        x = dx / len;
        y = dy / len;
        return true;
    }

    /// <summary>Follows the direction field from a cell to a goal.</summary>
    /// <param name="start">The cell to start from.</param>
    /// <returns>The cells from <paramref name="start"/> to a goal, or an empty array if the cell cannot reach one.</returns>
    public Cell[] GetPath(Cell start)
    {
        if (!InGrid(start) || double.IsPositiveInfinity(_driver.Costs[(start.Y * _width) + start.X]))
        {
            return [];
        }
        var cells = new List<Cell> { start };
        Cell c = start;
        while (TryGetNext(c, out Cell n))
        {
            cells.Add(n);
            c = n;
            if (cells.Count > _width * _height)
            {
                throw new InvalidOperationException("The direction field contains a cycle."); // cannot happen with positive costs
            }
        }
        return cells.ToArray();
    }

    /// <summary>Brings the field up to date with the cells edited since it was computed, recomputing only the affected region.</summary>
    /// <param name="cancellationToken">Checked between batches of iterations.</param>
    /// <exception cref="OperationCanceledException">Cancelled.</exception>
    public void Refresh(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        _driver.Run(cancellationToken);
        int areas = _square?.Areas.Version ?? _hex!.Areas.Version;
        if (areas != _areasVersion)
        {
            _pending.Clear();
            Recompute(cancellationToken);
            return;
        }
        if (_pending.Count == 0)
        {
            return;
        }
        EnsureClearance(); // refreshed in place; the topology view reads the same array
        var dirty = new List<int>();
        foreach (int cellIndex in _pending)
        {
            int y = cellIndex / _width;
            int x = cellIndex - (y * _width);
            int x0 = Math.Max(0, x - _reach);
            int x1 = Math.Min(_width - 1, x + 1);
            int y0 = Math.Max(0, y - _reach);
            int y1 = Math.Min(_height - 1, y + 1);
            for (int j = y0; j <= y1; j++)
            {
                for (int i = x0; i <= x1; i++)
                {
                    dirty.Add((j * _width) + i);
                }
            }
        }
        _pending.Clear();
        _driver.BeginRepair(dirty.ToArray());
        _driver.Run(cancellationToken);
    }

    /// <summary>Discards the field and computes it again from the grid as it is now.</summary>
    /// <param name="cancellationToken">Checked between batches of iterations.</param>
    /// <exception cref="OperationCanceledException">Cancelled.</exception>
    public void Recompute(CancellationToken cancellationToken = default)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        _pending.Clear();
        EnsureClearance();
        _driver.Begin(_goals);
        _driver.Run(cancellationToken);
        _areasVersion = _square?.Areas.Version ?? _hex!.Areas.Version;
    }

    /// <summary>Stops watching the grid.</summary>
    public void Dispose()
    {
        if (_disposed)
        {
            return;
        }
        _disposed = true;
        _square?.RemoveListener(this);
        _hex?.RemoveListener(this);
    }

    void IGridChangeListener.OnCellChanged(int x, int y) => _pending.Add((y * _width) + x);

    private void EnsureClearance()
    {
        if (_square is not null)
        {
            _ = _square.Clearance;
        }
    }

    private bool InGrid(Cell c) => (uint)c.X < (uint)_width && (uint)c.Y < (uint)_height;

    private bool TryGetCell(WorldPoint p, out Cell c) => _square is not null ? _square.TryGetCell(p, out c) : _hex!.TryGetCell(p, out c);

    private WorldPoint Centre(Cell c) => _square is not null ? _square.CellCenter(c) : _hex!.CellCenter(c);
}
