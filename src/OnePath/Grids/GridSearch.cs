using System;
using System.Buffers.Binary;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// A resumable A* search over a <see cref="Grid"/> without generic parameters: <see cref="TryBegin"/>, <see cref="Step"/> in slices of
/// a fixed number of iterations, <see cref="CopyPath(Span{Cell})"/>, and byte-exact <see cref="SaveState()"/> /
/// <see cref="RestoreState"/> for hosts that roll back. Create one with <see cref="Grid.CreateSearch"/> and reuse it: after the first
/// search, <see cref="TryBegin"/>, <see cref="Step"/>, <see cref="CopyPath(Span{Cell})"/> and <see cref="SaveState(Span{byte})"/> allocate nothing.
/// <para>
/// It runs the same engine as <see cref="Grid.FindPath(Cell, Cell, GridOptions?)"/> with <see cref="GridAlgorithm.AStar"/> and the same
/// options, so the path it finds is cell for cell the path <c>FindPath</c> returns, however the iterations are sliced.
/// <c>FindPath</c> is <c>TryBegin</c> followed by one <c>Step(IterationBudget)</c>. Only <see cref="GridAlgorithm.AStar"/> is
/// supported here (other algorithms keep their one-shot <c>FindPath</c> and, for the generic forms, their own <c>Begin</c>/<c>Step</c>).
/// </para>
/// <para>
/// The grid must not be edited between <see cref="TryBegin"/> and the end of the search (<see cref="Step"/> throws
/// <see cref="InvalidOperationException"/> if it was). Not thread-safe; use one search per thread.
/// </para>
/// </summary>
public sealed class GridSearch
{
    private readonly Grid _grid;
    private readonly GridOptions _options;
    private OctileAStar? _fast;
    private GenericGridEngine? _generic;
    private int _areaVersion = -1;
    private double _unitCost = 1.0;
    private int _beganVersion = -1;
    private int _beganRestores = -1;
    private Cell _start;
    private Cell _goal;
    private bool _begun;

    internal GridSearch(Grid grid, GridOptions options)
    {
        if (options.Algorithm != GridAlgorithm.AStar)
        {
            throw new ArgumentException($"GridSearch runs A*; the options select {options.Algorithm}. Use Grid.FindPath for other algorithms.", nameof(options));
        }
        _grid = grid;
        _options = options.Clone();
    }

    /// <summary>The grid searched.</summary>
    public Grid Grid => _grid;

    /// <summary>Where the search stands: <see cref="SearchStatus.NotStarted"/> until <see cref="TryBegin"/> succeeds.</summary>
    public SearchStatus Status => !_begun ? SearchStatus.NotStarted : _usesFast ? _fast!.Status : _generic!.Status;

    private bool _usesFast;

    /// <summary>True when the specialised octile engine serves the options (see <see cref="GridOptions.SpecializedEngines"/>); false when the generic engine does.</summary>
    public bool UsesSpecializedEngine => _usesFast;

    /// <summary>The start cell of the current search (undefined before <see cref="TryBegin"/>).</summary>
    public Cell Start => _start;

    /// <summary>The goal cell of the current search (undefined before <see cref="TryBegin"/>).</summary>
    public Cell Goal => _goal;

    /// <summary>Nodes expanded so far, the goal's pop excluded.</summary>
    public int ExpandedCount => !_begun ? 0 : _usesFast ? _fast!.ExpandedCount : _generic!.ExpandedCount;

    /// <summary>Queue pops so far (the unit of every budget).</summary>
    public int Iterations => !_begun ? 0 : _usesFast ? _fast!.Iterations : _generic!.Iterations;

    /// <summary>The cost of the path once <see cref="Status"/> is Found, otherwise +∞. Same value as <see cref="GridPath.Cost"/> from <c>FindPath</c>.</summary>
    public double PathCost => Status != SearchStatus.Found ? double.PositiveInfinity : _usesFast ? _unitCost * _fast!.PathCost : _generic!.PathCost;

    /// <summary>
    /// Starts a search, discarding the previous one. A start or goal that is outside the grid, a wall, too tight for
    /// <see cref="GridOptions.AgentSize"/> or refused by <see cref="GridOptions.Blocked"/> is reported, not thrown.
    /// </summary>
    /// <param name="start">Start cell.</param>
    /// <param name="goal">Goal cell.</param>
    /// <returns><see cref="PathStatus.InProgress"/>, or <see cref="PathStatus.StartInvalid"/> / <see cref="PathStatus.GoalInvalid"/> (start first). On a refusal the previous search is left as it was.</returns>
    public PathStatus TryBegin(Cell start, Cell goal)
    {
        _ = _grid.Clearance; // brings the clearance up to date
        PathStatus check = CheckEnds(start, goal);
        if (check != PathStatus.InProgress)
        {
            return check;
        }
        SelectEngine();
        int a = (start.Y * _grid.Width) + start.X;
        int b = (goal.Y * _grid.Width) + goal.X;
        if (_usesFast)
        {
            _fast!.Begin(a, b, _options.Heuristic == GridHeuristic.Zero);
        }
        else
        {
            _generic!.Begin(a, b);
        }
        _start = start;
        _goal = goal;
        _begun = true;
        _beganVersion = _grid.Version;
        _beganRestores = _grid.RestoreCount;
        return PathStatus.InProgress;
    }

    /// <summary>Starts a search like <see cref="TryBegin"/> but throws when an end cell is refused.</summary>
    /// <param name="start">Start cell.</param>
    /// <param name="goal">Goal cell.</param>
    /// <exception cref="ArgumentException">The start or goal cell is outside the grid, not walkable, too tight for the agent size or blocked.</exception>
    public void Begin(Cell start, Cell goal)
    {
        PathStatus s = TryBegin(start, goal);
        if (s == PathStatus.StartInvalid)
        {
            throw new ArgumentException($"The start cell ({start.X}, {start.Y}) is outside the grid or not walkable for this agent.", nameof(start));
        }
        if (s == PathStatus.GoalInvalid)
        {
            throw new ArgumentException($"The goal cell ({goal.X}, {goal.Y}) is outside the grid or not walkable for this agent.", nameof(goal));
        }
    }

    private PathStatus CheckEnds(Cell start, Cell goal)
    {
        byte[] clear = _grid.ClearanceArray;
        int size = _options.AgentSize;
        int w = _grid.Width;
        if (!_grid.Contains(start.X, start.Y) || clear[(start.Y * w) + start.X] < size || _options.Blocked?.Invoke(start) == true)
        {
            return PathStatus.StartInvalid;
        }
        if (!_grid.Contains(goal.X, goal.Y) || clear[(goal.Y * w) + goal.X] < size || _options.Blocked?.Invoke(goal) == true)
        {
            return PathStatus.GoalInvalid;
        }
        return PathStatus.InProgress;
    }

    // The engine follows the options and the area table as they are at the start of each search, exactly as FindPath decides.
    private void SelectEngine()
    {
        bool fast = _grid.UsesSpecializedAStar(_options, out double unit);
        if (fast)
        {
            _fast ??= new OctileAStar(_grid);
            _unitCost = unit;
            _usesFast = true;
        }
        else
        {
            if (_generic is null || _areaVersion != _grid.Areas.Version)
            {
                _generic = GenericGridEngine.Create(_grid, _options);
                _areaVersion = _grid.Areas.Version;
            }
            _usesFast = false;
        }
    }

    /// <summary>Continues the search by at most <paramref name="iterations"/> queue pops.</summary>
    /// <param name="iterations">At least 1.</param>
    /// <returns><see cref="PathStatus.Found"/>, <see cref="PathStatus.NoPath"/>, or <see cref="PathStatus.InProgress"/> when the slice ran out; once finished, the same result again.</returns>
    /// <exception cref="InvalidOperationException">No search was begun, or the grid was edited since.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="iterations"/> is below 1.</exception>
    public PathStatus Step(int iterations)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(iterations, 1);
        if (!_begun)
        {
            throw new InvalidOperationException("Call TryBegin before Step.");
        }
        if (_beganVersion != _grid.Version || _beganRestores != _grid.RestoreCount)
        {
            throw new InvalidOperationException("The grid was edited since the search began; begin a new search.");
        }
        SearchStatus s = _usesFast ? _fast!.Step(iterations, false) : _generic!.Step(iterations);
        return s switch
        {
            SearchStatus.Found => PathStatus.Found,
            SearchStatus.NotFound => PathStatus.NoPath,
            _ => PathStatus.InProgress,
        };
    }

    /// <summary>Number of cells on the found path, start and goal included.</summary>
    /// <exception cref="InvalidOperationException">The search has not found a path.</exception>
    public int PathLength
    {
        get
        {
            if (Status != SearchStatus.Found)
            {
                throw new InvalidOperationException("The search has not found a path.");
            }
            return _usesFast ? _fast!.PathLength() : _generic!.PathLength();
        }
    }

    /// <summary>Writes the found path as cell indexes (<c>y * Width + x</c>), start first.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> entries.</param>
    /// <returns>The number of entries written.</returns>
    /// <exception cref="InvalidOperationException">The search has not found a path.</exception>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is too small.</exception>
    public int CopyPath(Span<int> destination)
    {
        int n = PathLength;
        if (destination.Length < n)
        {
            throw new ArgumentException($"The path has {n} cells but the destination holds {destination.Length}.", nameof(destination));
        }
        if (_usesFast)
        {
            _fast!.CopyPath(destination);
        }
        else
        {
            _generic!.CopyPath(destination);
        }
        return n;
    }

    /// <summary>Writes the found path as cells, start first.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> entries.</param>
    /// <returns>The number of entries written.</returns>
    /// <exception cref="InvalidOperationException">The search has not found a path.</exception>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is too small.</exception>
    public int CopyPath(Span<Cell> destination)
    {
        int n = PathLength;
        if (destination.Length < n)
        {
            throw new ArgumentException($"The path has {n} cells but the destination holds {destination.Length}.", nameof(destination));
        }
        int w = _grid.Width;
        if (_usesFast)
        {
            _fast!.CopyCells(destination, w);
        }
        else
        {
            _generic!.CopyCells(destination, w);
        }
        return n;
    }

    /// <summary>
    /// Writes the saved state (see docs/grids-state.md) into <paramref name="destination"/>. With the specialised engine this allocates
    /// nothing, and costs a pass over the grid's cells plus the visited cells; with the generic engine it allocates the bytes.
    /// </summary>
    /// <param name="destination">At least <see cref="GetStateSize"/> bytes.</param>
    /// <returns>The number of bytes written.</returns>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is too small.</exception>
    public int SaveState(Span<byte> destination)
    {
        if (!_begun || !_usesFast)
        {
            byte[] bytes = SaveState();
            if (destination.Length < bytes.Length)
            {
                throw new ArgumentException($"The state has {bytes.Length} bytes but the destination holds {destination.Length}.", nameof(destination));
            }
            bytes.CopyTo(destination);
            return bytes.Length;
        }
        int need = HeaderBytes + _fast!.StateLength();
        if (destination.Length < need)
        {
            throw new ArgumentException($"The state has {need} bytes but the destination holds {destination.Length}.", nameof(destination));
        }
        WriteHeader(destination, 1);
        return HeaderBytes + _fast.WriteState(destination[HeaderBytes..]);
    }

    private const int HeaderBytes = 8 + 1 + 4 + 4; // magic, version, kind, mode, width, height

    private void WriteHeader(Span<byte> d, byte mode)
    {
        BinaryPrimitives.WriteUInt32LittleEndian(d, StateWriter.Magic);
        BinaryPrimitives.WriteUInt16LittleEndian(d[4..], StateWriter.FormatVersion);
        BinaryPrimitives.WriteUInt16LittleEndian(d[6..], (ushort)StateKind.GridSearch);
        d[8] = mode;
        BinaryPrimitives.WriteInt32LittleEndian(d[9..], _grid.Width);
        BinaryPrimitives.WriteInt32LittleEndian(d[13..], _grid.Height);
    }

    /// <summary>Bytes <see cref="SaveState(Span{byte})"/> needs now.</summary>
    /// <returns>The size.</returns>
    public int GetStateSize() => _begun && _usesFast ? HeaderBytes + _fast!.StateLength() : SaveState().Length;

    /// <summary>
    /// The saved state of the search: its progress, exact to the byte, the same on every platform (docs/grids-state.md). A search
    /// that was never begun saves as "not started".
    /// </summary>
    /// <returns>The bytes.</returns>
    public byte[] SaveState()
    {
        if (_begun && _usesFast)
        {
            byte[] fastBytes = new byte[HeaderBytes + _fast!.StateLength()];
            SaveState(fastBytes);
            return fastBytes;
        }
        var w = new StateWriter(StateKind.GridSearch);
        w.U8(_begun ? (byte)2 : (byte)0);
        w.I32(_grid.Width);
        w.I32(_grid.Height);
        if (_begun)
        {
            w.I32(_start.X);
            w.I32(_start.Y);
            w.I32(_goal.X);
            w.I32(_goal.Y);
            byte[] inner = _generic!.SaveState();
            w.I32(inner.Length);
            w.Bytes(inner);
        }
        return w.ToArray();
    }

    /// <summary>
    /// Puts a saved search back. The search must be created over the same (already restored) grid with the same options; the
    /// specialised engine is chosen by the state, and the grid must still allow it. The search is unusable after a failure until
    /// begun or restored again.
    /// </summary>
    /// <param name="state">Bytes from <see cref="SaveState()"/>.</param>
    /// <exception cref="GridStateException">The bytes are malformed, or do not fit this grid and these options.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        _begun = false;
        var r = new StateReader(state, StateKind.GridSearch);
        byte mode = r.U8();
        r.Expect(_grid.Width, "the grid width");
        r.Expect(_grid.Height, "the grid height");
        _ = _grid.Clearance;
        if (mode == 0)
        {
            r.End();
            return;
        }
        if (mode == 1)
        {
            if (!_grid.UsesSpecializedAStar(_options, out double unit))
            {
                throw r.Fail("the state was saved with the specialised engine but these options or this grid's area costs do not select it.");
            }
            _fast ??= new OctileAStar(_grid);
            _unitCost = unit;
            _usesFast = true;
            _fast.ReadState(ref r);
            _start = SearchRunner.ToCell(_fast.Start, _grid.Width);
            _goal = SearchRunner.ToCell(_fast.Goal, _grid.Width);
            if (_fast.Status == SearchStatus.NotStarted)
            {
                return;
            }
        }
        else if (mode == 2)
        {
            if (_grid.UsesSpecializedAStar(_options, out _))
            {
                throw r.Fail("the state was saved with the generic engine but these options select the specialised one.");
            }
            int sx = r.I32();
            int sy = r.I32();
            int gx = r.I32();
            int gy = r.I32();
            if (!_grid.Contains(sx, sy) || !_grid.Contains(gx, gy))
            {
                throw r.Fail("the start or goal cell is outside the grid.");
            }
            int len = r.Count(int.MaxValue, 1, "the length of the embedded search state");
            ReadOnlySpan<byte> inner = r.Bytes(len);
            r.End();
            _generic = GenericGridEngine.Create(_grid, _options);
            _areaVersion = _grid.Areas.Version;
            _generic.RestoreState(inner);
            _usesFast = false;
            _start = new Cell(sx, sy);
            _goal = new Cell(gx, gy);
        }
        else
        {
            throw r.Fail($"engine mode {mode} is not defined.");
        }
        _begun = true;
        _beganVersion = _grid.Version;
        _beganRestores = _grid.RestoreCount;
    }
}
