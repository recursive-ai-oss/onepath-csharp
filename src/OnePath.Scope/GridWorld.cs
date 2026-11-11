using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.Globalization;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Scope;

/// <summary>A grid search or flow field replayed deterministically. A step is the step's edits, then one slice of the engine.</summary>
internal sealed partial class GridWorld : IScopeWorld
{
    private readonly string _name;
    private readonly GridRun.Parsed _run;
    private readonly GridOptions _options;
    private Grid _grid;
    private GridSearch? _search;
    private GridFlowField? _flow;
    private Cell _start, _goal;
    private Cell[] _goals;
    private long _step;
    private readonly byte[] _frontier;
    private bool _frontierValid, _frontierKnown;
    private int _open, _closed;
    private readonly List<GridRunOp> _ops = [];

    public GridWorld(string name, GridRun.Parsed run)
    {
        _name = name;
        _run = run;
        _grid = LoadGrid(run.GridState);
        if (_grid.Width * (long)_grid.Height > GridRun.MaxCells)
        {
            throw new ScopeFormatException(name, $"expected a grid of at most {GridRun.MaxCells} cells, got {_grid.Width} x {_grid.Height}.");
        }

        foreach (Cell c in run.Points)
        {
            if (!_grid.Contains(c.X, c.Y))
            {
                throw new ScopeFormatException(name, $"expected start, goal and goal cells inside the {_grid.Width} x {_grid.Height} grid, got ({c.X}, {c.Y}).");
            }
        }

        _frontier = new byte[_grid.Width * _grid.Height];
        _goals = run.Points;
        _start = run.Points[0];
        _goal = run.Points[^1];
        _options = new GridOptions { Algorithm = run.Algorithm, Connectivity = run.Connectivity, Diagonals = run.Diagonals, AgentSize = run.AgentSize, IterationBudget = run.Slice };
        try
        {
            Restart();
        }
        catch (ArgumentException e)
        {
            throw new ScopeFormatException(name, "expected options the library accepts: " + e.Message);
        }
    }

    private Grid LoadGrid(byte[] state)
    {
        try
        {
            return Grid.FromState(state);
        }
        catch (OnePathFormatException e)
        {
            throw new ScopeFormatException(_name, "expected a grid state the library accepts: " + e.Message);
        }
    }

    public long Step => _step;

    internal bool IsFlow => _run.Kind == 2;

    /// <summary>The search's status; for a flow field, Found when complete.</summary>
    internal PathStatus Status => IsFlow ? (_flow!.Status == FlowFieldStatus.Complete ? PathStatus.Found : PathStatus.InProgress) : _search!.Status;

    internal int ExpandedCount => IsFlow ? _flow!.Settled : _search!.ExpandedCount;

    private bool Running => IsFlow ? _flow!.Status == FlowFieldStatus.Running : _search!.Status == PathStatus.InProgress;

    private void Restart()
    {
        _frontierValid = false;
        if (IsFlow)
        {
            _flow?.Dispose();
            _flow = _grid.CreateFlowField(_goals, _options);
        }
        else
        {
            // A search that cannot begin (the start or goal is a wall) keeps whatever it held before, so it is replaced by a fresh one that
            // is not begun; its saved state then says "not started", which a restore reproduces.
            _search ??= _grid.CreateSearch(_options);
            if (_search.TryBegin(_start, _goal) != PathStatus.InProgress)
            {
                _search = _grid.CreateSearch(_options);
            }
        }
    }

    public ulong Hash()
    {
        var w = new ArrayBufferWriter<byte>(1 << 12);
        WriteSnapshot(w);
        ulong h = 0xCBF29CE484222325UL;
        foreach (byte b in w.WrittenSpan)
        {
            h = (h ^ b) * 0x100000001B3UL;
        }

        return h;
    }

    public void Advance(ReadOnlySpan<byte> input, TimelineSink sink) => Run(input, int.MaxValue, sink);

    public void AdvanceAxis(string axis, int position, ReadOnlySpan<byte> input)
    {
        if (axis != GridSource.IterationAxis)
        {
            throw new ArgumentException($"A grid source has the time axis and '{GridSource.IterationAxis}', not '{axis}'.", nameof(axis));
        }

        if (position < 1 || position > _run.Slice)
        {
            throw new ArgumentOutOfRangeException(nameof(position), position, $"The iteration position must be 1 to {_run.Slice}.");
        }

        Run(input, position, TimelineSink.None);
    }

    private void Run(ReadOnlySpan<byte> input, int iterations, TimelineSink sink)
    {
        bool edited = ApplyOps(input, sink);
        if (edited)
        {
            Restart();
        }

        PathStatus before = Status;
        if (Running)
        {
            int n = iterations == int.MaxValue ? _run.Slice : iterations;
            if (IsFlow)
            {
                _flow!.Step(n);
            }
            else
            {
                _search!.Step(n);
            }
        }

        _frontierValid = false;
        if (iterations == int.MaxValue)
        {
            _step++;
        }

        if (!sink.IsRecording)
        {
            return;
        }

        sink.Sample(IsFlow ? "settled" : "expanded", ExpandedCount);
        if (!IsFlow)
        {
            Frontier();
            if (_frontierKnown)
            {
                sink.Sample("open", _open);
            }
        }

        if (IsFlow && edited && _flow!.Settled == 0 && _flow.Status == FlowFieldStatus.Complete)
        {
            sink.Finding("flow.goal-blocked", Severity.Warning, [ScopeInfo.GoalMarker], "No goal cell is walkable, so the flow field has no cells.", "Move the goal onto a walkable cell.");
        }

        PathStatus now = Status;
        if (now != before || edited)
        {
            switch (now)
            {
                case PathStatus.Found:
                    sink.Event("search", IsFlow ? ScopeInfo.GridRoot : ScopeInfo.GoalMarker, IsFlow ? $"flow field complete, {ExpandedCount} cells settled" : $"path found, cost {Fmt(_search!.PathCost)}, {_search.PathLength} cells, {ExpandedCount} expanded");
                    break;
                case PathStatus.NoPath:
                    sink.Event("search", ScopeInfo.GoalMarker, $"no path, {ExpandedCount} cells expanded");
                    sink.Finding("search.no-path", Severity.Warning, [ScopeInfo.StartMarker, ScopeInfo.GoalMarker], "The search exhausted every reachable cell without reaching the goal.", "Clear a wall between the start and the goal.");
                    break;
                case PathStatus.InProgress:
                    sink.Event("search", IsFlow ? ScopeInfo.GridRoot : ScopeInfo.StartMarker, IsFlow ? "flow field restarted" : "search restarted");
                    break;
                default:
                    sink.Event("search", ScopeInfo.StartMarker, $"search could not begin: {now}");
                    sink.Finding("search.invalid", Severity.Error, [ScopeInfo.StartMarker, ScopeInfo.GoalMarker], $"The search could not begin ({now}): the start or the goal is blocked or outside the walkable area.", "Move the start or goal onto a walkable cell.");
                    break;
            }
        }
    }

    private bool ApplyOps(ReadOnlySpan<byte> input, TimelineSink sink)
    {
        if (input.IsEmpty)
        {
            return false;
        }

        if (input.Length % GridRun.OpBytes != 0 || input.Length > GridRun.MaxOpsPerStep * GridRun.OpBytes)
        {
            throw new ScopeFormatException(_name, $"expected a whole number of {GridRun.OpBytes}-byte edits, got {input.Length} bytes.");
        }

        _ops.Clear();
        for (int i = 0; i < input.Length; i += GridRun.OpBytes)
        {
            var op = new GridRunOp((GridRunOpKind)input[i], BinaryPrimitives.ReadInt32LittleEndian(input[(i + 1)..]), BinaryPrimitives.ReadInt32LittleEndian(input[(i + 5)..]), input[i + 9]);
            if (op.Kind is < GridRunOpKind.SetArea or > GridRunOpKind.SetGoal || !_grid.Contains(op.X, op.Y) || (op.Kind == GridRunOpKind.SetStart && IsFlow) || (op.Kind != GridRunOpKind.SetArea && op.Area != 0))
            {
                throw new ScopeFormatException(_name, $"expected an edit of kind 1 to {(IsFlow ? 3 : 3)} on a cell inside the grid, got kind {(byte)op.Kind} at ({op.X}, {op.Y}) in edit {i / GridRun.OpBytes}.");
            }

            _ops.Add(op);
        }

        foreach (GridRunOp op in _ops)
        {
            switch (op.Kind)
            {
                case GridRunOpKind.SetArea:
                    _grid.SetArea(op.X, op.Y, op.Area);
                    if (sink.IsRecording)
                    {
                        sink.Event("edits", ScopeInfo.CellEntity((op.Y * _grid.Width) + op.X), $"cell ({op.X}, {op.Y}) set to {(op.Area == AreaCosts.Wall ? "wall" : $"area {op.Area}")}");
                    }

                    break;
                case GridRunOpKind.SetStart:
                    _start = new Cell(op.X, op.Y);
                    if (sink.IsRecording)
                    {
                        sink.Event("edits", ScopeInfo.StartMarker, $"start moved to ({op.X}, {op.Y})");
                    }

                    break;
                default:
                    _goal = new Cell(op.X, op.Y);
                    _goals = [_goal];
                    if (sink.IsRecording)
                    {
                        sink.Event("edits", ScopeInfo.GoalMarker, $"goal moved to ({op.X}, {op.Y})");
                    }

                    break;
            }
        }

        return true;
    }

    public void WriteSnapshot(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        byte[] grid = _grid.SaveState();
        byte[] state = IsFlow ? _flow!.SaveState() : _search!.SaveState();
        Cell[] points = IsFlow ? _goals : [_start, _goal];
        int size = 8 + 4 + grid.Length + 4 + (points.Length * 8) + 4 + state.Length;
        Span<byte> d = writer.GetSpan(size);
        BinaryPrimitives.WriteInt64LittleEndian(d, _step);
        BinaryPrimitives.WriteInt32LittleEndian(d[8..], grid.Length);
        grid.CopyTo(d[12..]);
        int n = 12 + grid.Length;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], points.Length);
        n += 4;
        foreach (Cell c in points)
        {
            BinaryPrimitives.WriteInt32LittleEndian(d[n..], c.X);
            BinaryPrimitives.WriteInt32LittleEndian(d[(n + 4)..], c.Y);
            n += 8;
        }

        BinaryPrimitives.WriteInt32LittleEndian(d[n..], state.Length);
        state.CopyTo(d[(n + 4)..]);
        writer.Advance(size);
    }

    public void Restore(ReadOnlySpan<byte> snapshot)
    {
        ScopeFormatException Fail(string what) => new(_name, "expected a OnePath grid snapshot: " + what);
        if (snapshot.Length < 24)
        {
            throw Fail($"at least 24 bytes, got {snapshot.Length}.");
        }

        long step = BinaryPrimitives.ReadInt64LittleEndian(snapshot);
        int gl = BinaryPrimitives.ReadInt32LittleEndian(snapshot[8..]);
        if (step < 0 || gl < 0 || gl > snapshot.Length - 12 - 8)
        {
            throw Fail($"a step of 0 or more and a grid state that fits, got step {step} and {gl} bytes.");
        }

        int n = 12 + gl;
        int pc = BinaryPrimitives.ReadInt32LittleEndian(snapshot[n..]);
        n += 4;
        if (pc < 1 || pc > GridRun.MaxGoals || (IsFlow ? false : pc != 2) || (long)pc * 8 > snapshot.Length - n - 4)
        {
            throw Fail($"{(IsFlow ? "1 to 1024 goals" : "2 points")} that fit, got {pc}.");
        }

        var points = new Cell[pc];
        for (int i = 0; i < pc; i++)
        {
            points[i] = new Cell(BinaryPrimitives.ReadInt32LittleEndian(snapshot[n..]), BinaryPrimitives.ReadInt32LittleEndian(snapshot[(n + 4)..]));
            n += 8;
        }

        int sl = BinaryPrimitives.ReadInt32LittleEndian(snapshot[n..]);
        n += 4;
        if (sl < 0 || sl != snapshot.Length - n)
        {
            throw Fail($"an engine state that ends the snapshot, got {sl} bytes with {snapshot.Length - n} left.");
        }

        try
        {
            _grid.RestoreState(snapshot.Slice(12, gl));
            foreach (Cell c in points)
            {
                if (!_grid.Contains(c.X, c.Y))
                {
                    throw Fail($"cells inside the grid, got ({c.X}, {c.Y}).");
                }
            }

            _step = step;
            _frontierValid = false;
            if (IsFlow)
            {
                _goals = points;
                _goal = points[0];
                _flow?.Dispose();
                _flow = _grid.CreateFlowField(points, _options);
                _flow.RestoreState(snapshot[n..]);
            }
            else
            {
                _start = points[0];
                _goal = points[1];
                _search!.RestoreState(snapshot[n..]);
            }
        }
        catch (OnePathFormatException e)
        {
            throw Fail(e.Message);
        }
    }

    public void EncodePoke(in PokeCall call, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        ScopeValue[] a = call.Args ?? [];
        GridRunOp op;
        switch (call.Poke)
        {
            case "move_goal":
                Need(call, a, ValueKind.Entity, ValueKind.Vector);
                if (a[0].GetEntity() != ScopeInfo.GoalMarker)
                {
                    throw new ArgumentException($"Entity {a[0].GetEntity()} is not the goal marker.", nameof(call));
                }

                Cell goalCell = CellOf(a[1].GetVector(), clamp: true);
                op = GridRunOp.SetGoal(goalCell.X, goalCell.Y);
                break;
            case "move_start" when !IsFlow:
                Need(call, a, ValueKind.Vector);
                Cell startCell = CellOf(a[0].GetVector(), clamp: true);
                op = GridRunOp.SetStart(startCell.X, startCell.Y);
                break;
            case "set_wall":
                Need(call, a, ValueKind.Vector, ValueKind.Bool);
                op = GridRunOp.SetArea(CellOf(a[0].GetVector()).X, CellOf(a[0].GetVector()).Y, a[1].GetBool() ? AreaCosts.Wall : (byte)0);
                break;
            case "set_area":
                Need(call, a, ValueKind.Vector, ValueKind.Integer);
                if (a[1].GetInteger() is < 0 or > 255)
                {
                    throw new ArgumentException($"Area {a[1].GetInteger()} must be 0 to 255 (255 is a wall).", nameof(call));
                }

                op = GridRunOp.SetArea(CellOf(a[0].GetVector()).X, CellOf(a[0].GetVector()).Y, (byte)a[1].GetInteger());
                break;
            default:
                throw new ArgumentException($"Unknown poke '{call.Poke}'; this source declares move_goal, {(IsFlow ? string.Empty : "move_start, ")}set_wall and set_area.", nameof(call));
        }

        GridRun.Encode(op, into);
    }

    private static void Need(in PokeCall call, ScopeValue[] args, params ValueKind[] kinds)
    {
        if (args.Length != kinds.Length)
        {
            throw new ArgumentException($"Poke '{call.Poke}' takes {kinds.Length} arguments, not {args.Length}.", nameof(call));
        }

        for (int i = 0; i < kinds.Length; i++)
        {
            if (args[i].Kind != kinds[i])
            {
                throw new ArgumentException($"Argument {i + 1} of poke '{call.Poke}' must be {kinds[i]}, not {args[i].Kind}.", nameof(call));
            }
        }
    }

    // Turns a view position into a cell. A drag that ends past the grid's edge is clamped to the nearest edge cell (the clamped cell is what
    // the recording holds, so the result is the same on every replay); a wall edit past the edge is refused.
    private Cell CellOf(Vector3 v, bool clamp = false)
    {
        if (!float.IsFinite(v.X) || !float.IsFinite(v.Y))
        {
            throw new ArgumentException($"Point ({v.X}, {v.Y}) must be finite.");
        }

        double fx = Math.Floor((double)v.X), fy = Math.Floor((double)v.Y);
        if (clamp)
        {
            return new Cell((int)Math.Clamp(fx, 0, _grid.Width - 1), (int)Math.Clamp(fy, 0, _grid.Height - 1));
        }

        if (fx < 0 || fy < 0 || fx >= _grid.Width || fy >= _grid.Height)
        {
            throw new ArgumentException($"Point ({v.X}, {v.Y}) is outside the {_grid.Width} x {_grid.Height} grid.");
        }

        return new Cell((int)fx, (int)fy);
    }

    public Difference? FirstDifference(IScopeWorld other)
    {
        if (other is not GridWorld o)
        {
            throw new ArgumentException("The other world is not a OnePath grid world.", nameof(other));
        }

        if (_grid.Width != o._grid.Width || _grid.Height != o._grid.Height)
        {
            return new Difference(ScopeInfo.GridRoot, "size", $"{_grid.Width} x {_grid.Height}", $"{o._grid.Width} x {o._grid.Height}");
        }

        ReadOnlySpan<byte> a = _grid.Cells, b = o._grid.Cells;
        for (int i = 0; i < a.Length; i++)
        {
            if (a[i] != b[i])
            {
                return new Difference(ScopeInfo.CellEntity(i), "area", a[i].ToString(CultureInfo.InvariantCulture), b[i].ToString(CultureInfo.InvariantCulture));
            }
        }

        if (_start != o._start)
        {
            return new Difference(ScopeInfo.StartMarker, "cell", Fmt(_start), Fmt(o._start));
        }

        if (!IsFlow && _goal != o._goal || IsFlow && !_goals.AsSpan().SequenceEqual(o._goals))
        {
            return new Difference(ScopeInfo.GoalMarker, "cell", Fmt(_goal), Fmt(o._goal));
        }

        if (IsFlow)
        {
            ReadOnlySpan<double> ca = _flow!.Costs, cb = o._flow!.Costs;
            ReadOnlySpan<int> na = _flow.Next, nb = o._flow.Next;
            for (int i = 0; i < ca.Length; i++)
            {
                if (!ca[i].Equals(cb[i]))
                {
                    return new Difference(ScopeInfo.CellEntity(i), "cost", Fmt(ca[i]), Fmt(cb[i]));
                }

                if (na[i] != nb[i])
                {
                    return new Difference(ScopeInfo.CellEntity(i), "next", na[i].ToString(CultureInfo.InvariantCulture), nb[i].ToString(CultureInfo.InvariantCulture));
                }
            }
        }
        else
        {
            Frontier();
            o.Frontier();
            if (_frontierKnown && o._frontierKnown)
            {
                for (int i = 0; i < _frontier.Length; i++)
                {
                    if (_frontier[i] != o._frontier[i])
                    {
                        return new Difference(ScopeInfo.CellEntity(i), "search state", StateName(_frontier[i]), StateName(o._frontier[i]));
                    }
                }
            }

            if (_search!.Status != o._search!.Status)
            {
                return new Difference(ScopeInfo.GridRoot, "status", _search.Status.ToString(), o._search.Status.ToString());
            }
        }

        if (_step != o._step)
        {
            return new Difference(ScopeInfo.GridRoot, "step", _step.ToString(CultureInfo.InvariantCulture), o._step.ToString(CultureInfo.InvariantCulture));
        }

        return Hash() == o.Hash() ? null : new Difference(ScopeInfo.GridRoot, "state", Hash().ToString("x16", CultureInfo.InvariantCulture), o.Hash().ToString("x16", CultureInfo.InvariantCulture));
    }

    internal static string StateName(byte s) => s switch { SearchFrontier.Open => "open", SearchFrontier.Closed => "closed", _ => "unseen" };

    private static string Fmt(Cell c) => $"({c.X}, {c.Y})";

    private static string Fmt(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Refreshes the open/closed classification of every cell from the search's saved state. Not available for a flow field.</summary>
    internal void Frontier()
    {
        if (_frontierValid || IsFlow)
        {
            return;
        }

        _frontierKnown = SearchFrontier.TryRead(_search!.SaveState(), _grid.Width, _grid.Height, _frontier, out _open, out _closed);
        _frontierValid = true;
    }

    internal bool FrontierKnown
    {
        get
        {
            Frontier();
            return _frontierKnown;
        }
    }

    internal byte CellState(int index)
    {
        Frontier();
        return _frontier[index];
    }

    public void CollectEntities(List<EntityInfo> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        into.Add(new EntityInfo(ScopeInfo.GridRoot, EntityId.None, "grid", IsFlow ? "flow field" : "grid search"));
        if (!IsFlow)
        {
            into.Add(new EntityInfo(ScopeInfo.StartMarker, ScopeInfo.GridRoot, "marker", "start"));
        }

        into.Add(new EntityInfo(ScopeInfo.GoalMarker, ScopeInfo.GridRoot, "marker", IsFlow ? "goal" : "goal"));
    }
}
