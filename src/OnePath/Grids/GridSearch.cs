using System;
using System.Buffers.Binary;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// A resumable search over a <see cref="Grid"/> without generic parameters: <see cref="TryBegin"/>, <see cref="Step"/> in slices of
/// a fixed number of iterations, <see cref="CopyPath(Span{Cell})"/>, and byte-exact <see cref="SaveState()"/> /
/// <see cref="RestoreState"/> for hosts that roll back. Create one with <see cref="Grid.CreateSearch"/> and reuse it: after the first
/// search, <see cref="TryBegin"/>, <see cref="Step"/>, <see cref="CopyPath(Span{Cell})"/>, <see cref="SaveState(Span{byte})"/> and
/// <see cref="GetStateSize"/> allocate nothing.
/// <para>
/// It runs the same engine as <see cref="Grid.FindPath(Cell, Cell, GridOptions?)"/> with the same options, so the path it finds is
/// cell for cell the path <c>FindPath</c> returns, however the iterations are sliced. <c>FindPath</c> is <c>TryBegin</c> followed by one
/// <c>Step(IterationBudget)</c>. <see cref="GridAlgorithm.AStar"/>, <see cref="GridAlgorithm.Jps"/> and <see cref="GridAlgorithm.JpsPlus"/>
/// are supported (the other algorithms keep their one-shot <c>FindPath</c> and, for the generic forms, their own <c>Begin</c>/<c>Step</c>).
/// For the jump point searches one iteration is one pop of a jump point; the scans that find jump points are not counted.
/// Options a jump point search cannot honour (four moves, agent size above 1, area costs that differ, callbacks, a weighted or Manhattan
/// estimate) make <c>FindPath</c> run plain A* instead, and so they do here.
/// </para>
/// <para>
/// The grid must not be edited between <see cref="TryBegin"/> and the end of the search (<see cref="Step"/> throws
/// <see cref="InvalidOperationException"/> if it was). Not thread-safe; use one search per thread.
/// </para>
/// </summary>
public sealed class GridSearch
{
    private enum Kind : byte
    {
        FastAStar,
        GenericAStar,
        FastJump,
        GenericJump,
    }

    private readonly Grid _grid;
    private readonly GridOptions _options;
    private OctileAStar? _fast;
    private GenericGridEngine? _generic;
    private int _genericKey = -1;
    private OctileJump<BitFinder>? _bitJump;
    private OctileJump<TableFinder>? _tableJump;
    private OctileMap? _jumpMap;
    private JpsPlusIndex? _jumpIndex;
    private IOctileJumpEngine? _jump;
    private Kind _kind;
    private bool _plus;
    private StateWriter? _writer;
    private int[] _cellScratch = [];
    private int _areaVersion = -1;
    private double _unitCost = 1.0;
    private int _beganVersion = -1;
    private int _beganRestores = -1;
    private Cell _start;
    private Cell _goal;
    private bool _begun;

    internal GridSearch(Grid grid, GridOptions options)
    {
        if (options.Algorithm is not (GridAlgorithm.AStar or GridAlgorithm.Jps or GridAlgorithm.JpsPlus))
        {
            throw new ArgumentException($"GridSearch runs A*, JPS and JPS+; the options select {options.Algorithm}. Use Grid.FindPath for other algorithms.", nameof(options));
        }
        _grid = grid;
        _options = options.Clone();
    }

    /// <summary>The grid searched.</summary>
    public Grid Grid => _grid;

    /// <summary>Where the search stands: <see cref="SearchStatus.NotStarted"/> until <see cref="TryBegin"/> succeeds.</summary>
    public SearchStatus Status => !_begun ? SearchStatus.NotStarted : _kind switch
    {
        Kind.FastAStar => _fast!.Status,
        Kind.FastJump => _jump!.Status,
        _ => _generic!.Status,
    };

    /// <summary>True when a specialised engine (the octile A* or the octile jump point search; see <see cref="GridOptions.SpecializedEngines"/>) serves the options; false when the generic engine does.</summary>
    public bool UsesSpecializedEngine => PlanKind(out _, out _) is Kind.FastAStar or Kind.FastJump;

    /// <summary>The start cell of the current search (undefined before <see cref="TryBegin"/>).</summary>
    public Cell Start => _start;

    /// <summary>The goal cell of the current search (undefined before <see cref="TryBegin"/>).</summary>
    public Cell Goal => _goal;

    /// <summary>Nodes expanded so far, the goal's pop excluded.</summary>
    public int ExpandedCount => !_begun ? 0 : _kind switch
    {
        Kind.FastAStar => _fast!.ExpandedCount,
        Kind.FastJump => _jump!.ExpandedCount,
        _ => _generic!.ExpandedCount,
    };

    /// <summary>Queue pops so far (the unit of every budget; for the jump point searches, pops of jump points).</summary>
    public int Iterations => !_begun ? 0 : _kind switch
    {
        Kind.FastAStar => _fast!.Iterations,
        Kind.FastJump => _jump!.Iterations,
        _ => _generic!.Iterations,
    };

    /// <summary>The cost of the path once <see cref="Status"/> is Found, otherwise +∞. Same value as <see cref="GridPath.Cost"/> from <c>FindPath</c>.</summary>
    public double PathCost => Status != SearchStatus.Found ? double.PositiveInfinity : _kind switch
    {
        Kind.FastAStar => _unitCost * _fast!.PathCost,
        Kind.FastJump => _unitCost * _jump!.PathCost,
        _ => _generic!.PathCost,
    };

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
        bool zero = _options.Heuristic == GridHeuristic.Zero;
        switch (_kind)
        {
            case Kind.FastAStar:
                _fast!.Begin(a, b, zero);
                break;
            case Kind.FastJump:
                _jump!.Begin(a, b, zero);
                break;
            default:
                _generic!.Begin(a, b);
                break;
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

    // Which engine serves the options and the grid as they are now, decided exactly as Grid.FindPath decides.
    private Kind PlanKind(out double unit, out bool plus)
    {
        if (_grid.UsesJumpSearch(_options, out unit, out bool fastJump, out plus))
        {
            return fastJump ? Kind.FastJump : Kind.GenericJump;
        }
        plus = false;
        return _grid.UsesSpecializedAStar(_options, out unit) ? Kind.FastAStar : Kind.GenericAStar;
    }

    // The engine follows the options and the area table as they are at the start of each search.
    private void SelectEngine()
    {
        Kind kind = PlanKind(out double unit, out bool plus);
        UseEngine(kind, unit, plus);
    }

    private void UseEngine(Kind kind, double unit, bool plus)
    {
        switch (kind)
        {
            case Kind.FastAStar:
                _fast ??= new OctileAStar(_grid);
                _unitCost = unit;
                break;
            case Kind.FastJump:
                // The finders read the grid's shared octile map and JPS+ tables; a grid restore replaces both, so an engine built
                // over the old ones is dropped.
                OctileMap map = _grid.SharedOctileMap();
                if (plus)
                {
                    JpsPlusIndex index = _grid.SharedJpsPlus();
                    if (_tableJump is null || !ReferenceEquals(_jumpMap, map) || !ReferenceEquals(_jumpIndex, index))
                    {
                        _tableJump = new OctileJump<TableFinder>(new TableFinder(index, map, false), _grid.Width, _grid.Height);
                        _bitJump = null;
                        _jumpMap = map;
                        _jumpIndex = index;
                    }
                    _jump = _tableJump;
                }
                else
                {
                    if (_bitJump is null || !ReferenceEquals(_jumpMap, map))
                    {
                        _bitJump = new OctileJump<BitFinder>(new BitFinder(map, false), _grid.Width, _grid.Height);
                        _tableJump = null;
                        _jumpMap = map;
                        _jumpIndex = null;
                    }
                    _jump = _bitJump;
                }
                _unitCost = unit;
                break;
            default:
                bool jump = kind == Kind.GenericJump;
                object? token = !jump ? null : plus ? _grid.SharedJpsPlus() : _grid.SharedBlockMap();
                int key = ((int)kind * 2) + (plus ? 1 : 0);
                if (_generic is null || _areaVersion != _grid.Areas.Version || _genericKey != key || !ReferenceEquals(_generic.Token, token))
                {
                    _generic = jump ? GenericGridEngine.CreateJump(_grid, _options, plus, unit) : GenericGridEngine.Create(_grid, _options);
                    _areaVersion = _grid.Areas.Version;
                    _genericKey = key;
                }
                break;
        }
        _kind = kind;
        _plus = plus;
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
        SearchStatus s = _kind switch
        {
            Kind.FastAStar => _fast!.Step(iterations, false),
            Kind.FastJump => _jump!.Step(iterations),
            _ => _generic!.Step(iterations),
        };
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
            return _kind switch
            {
                Kind.FastAStar => _fast!.PathLength(),
                Kind.FastJump => _jump!.PathLength(),
                _ => _generic!.PathLength(),
            };
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
        switch (_kind)
        {
            case Kind.FastAStar:
                _fast!.CopyPath(destination);
                break;
            case Kind.FastJump:
                _jump!.CopyPath(destination);
                break;
            default:
                _generic!.CopyPath(destination);
                break;
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
        switch (_kind)
        {
            case Kind.FastAStar:
                _fast!.CopyCells(destination, w);
                break;
            case Kind.FastJump:
                // Hops are expanded into cell indexes first; the scratch grows to the longest path seen and is then reused.
                if (_cellScratch.Length < n)
                {
                    _cellScratch = new int[n];
                }
                _jump!.CopyPath(_cellScratch);
                for (int i = 0; i < n; i++)
                {
                    destination[i] = SearchRunner.ToCell(_cellScratch[i], w);
                }
                break;
            default:
                _generic!.CopyCells(destination, w);
                break;
        }
        return n;
    }

    /// <summary>
    /// Writes the saved state (see docs/grids-state.md) into <paramref name="destination"/>. Allocates nothing in steady state with any
    /// engine; costs a pass over the grid's cells plus the visited cells (the generic engine also serialises its state into a buffer
    /// the search keeps, which grows to the largest state seen).
    /// </summary>
    /// <param name="destination">At least <see cref="GetStateSize"/> bytes.</param>
    /// <returns>The number of bytes written.</returns>
    /// <exception cref="ArgumentException"><paramref name="destination"/> is too small.</exception>
    public int SaveState(Span<byte> destination)
    {
        int need = Serialize(destination);
        if (destination.Length < need)
        {
            throw new ArgumentException($"The state has {need} bytes but the destination holds {destination.Length}.", nameof(destination));
        }
        return need;
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

    // Returns the size of the state; writes it only when the destination is big enough.
    private int Serialize(Span<byte> d)
    {
        if (!_begun)
        {
            if (d.Length >= HeaderBytes)
            {
                WriteHeader(d, 0);
            }
            return HeaderBytes;
        }
        if (_kind == Kind.FastAStar)
        {
            int size = HeaderBytes + _fast!.StateLength();
            if (d.Length >= size)
            {
                WriteHeader(d, 1);
                _fast.WriteState(d[HeaderBytes..]);
            }
            return size;
        }
        if (_kind == Kind.FastJump)
        {
            int size = HeaderBytes + 1 + _jump!.StateLength();
            if (d.Length >= size)
            {
                WriteHeader(d, 3);
                d[HeaderBytes] = _plus ? (byte)1 : (byte)0;
                _jump.WriteState(d[(HeaderBytes + 1)..]);
            }
            return size;
        }
        // Generic engines: their own state blob is written to a buffer this search keeps, then framed with the cells and its length.
        _writer ??= new StateWriter(StateKind.GridSearch);
        _generic!.SaveTo(_writer);
        ReadOnlySpan<byte> inner = _writer.Written;
        int total = HeaderBytes + 20 + inner.Length;
        if (d.Length >= total)
        {
            WriteHeader(d, 2);
            int p = HeaderBytes;
            BinaryPrimitives.WriteInt32LittleEndian(d[p..], _start.X);
            BinaryPrimitives.WriteInt32LittleEndian(d[(p + 4)..], _start.Y);
            BinaryPrimitives.WriteInt32LittleEndian(d[(p + 8)..], _goal.X);
            BinaryPrimitives.WriteInt32LittleEndian(d[(p + 12)..], _goal.Y);
            BinaryPrimitives.WriteInt32LittleEndian(d[(p + 16)..], inner.Length);
            inner.CopyTo(d[(p + 20)..]);
        }
        return total;
    }

    /// <summary>Bytes <see cref="SaveState(Span{byte})"/> needs now.</summary>
    /// <returns>The size.</returns>
    public int GetStateSize() => Serialize(default);

    /// <summary>
    /// The saved state of the search: its progress, exact to the byte, the same on every platform (docs/grids-state.md). A search
    /// that was never begun saves as "not started".
    /// </summary>
    /// <returns>The bytes.</returns>
    public byte[] SaveState()
    {
        var bytes = new byte[Serialize(default)];
        Serialize(bytes);
        return bytes;
    }

    /// <summary>
    /// Puts a saved search back. The search must be created over the same (already restored) grid with the same options; the
    /// engine is chosen by the options and the grid exactly as for a new search, and the state must have been written by that
    /// engine. The search is unusable after a failure until begun or restored again.
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
        if (mode > 3)
        {
            throw r.Fail($"engine mode {mode} is not defined.");
        }
        Kind kind = PlanKind(out double unit, out bool plus);
        if (mode == 1)
        {
            if (kind != Kind.FastAStar)
            {
                throw r.Fail("the state was saved with the specialised A* engine but these options or this grid's area costs do not select it.");
            }
            UseEngine(kind, unit, plus);
            _fast!.ReadState(ref r);
            _start = SearchRunner.ToCell(_fast.Start, _grid.Width);
            _goal = SearchRunner.ToCell(_fast.Goal, _grid.Width);
            if (_fast.Status == SearchStatus.NotStarted)
            {
                return;
            }
        }
        else if (mode == 3)
        {
            if (kind != Kind.FastJump)
            {
                throw r.Fail("the state was saved with the specialised jump point engine but these options or this grid do not select it.");
            }
            byte finder = r.U8();
            if (finder > 1 || (finder == 1) != plus)
            {
                throw r.Fail($"the state was saved with jump finder {finder} but these options select {(plus ? 1 : 0)}.");
            }
            UseEngine(kind, unit, plus);
            _jump!.ReadState(ref r, _grid.CellArray);
            _start = SearchRunner.ToCell(_jump.Start, _grid.Width);
            _goal = SearchRunner.ToCell(_jump.Goal, _grid.Width);
            if (_jump.Status == SearchStatus.NotStarted)
            {
                return;
            }
        }
        else
        {
            if (kind is Kind.FastAStar or Kind.FastJump)
            {
                throw r.Fail("the state was saved with the generic engine but these options select a specialised one.");
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
            UseEngine(kind, unit, plus);
            _generic!.RestoreState(inner);
            _start = new Cell(sx, sy);
            _goal = new Cell(gx, gy);
        }
        _begun = true;
        _beganVersion = _grid.Version;
        _beganRestores = _grid.RestoreCount;
    }
}
