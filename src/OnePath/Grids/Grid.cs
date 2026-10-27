using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>Told when a cell of a grid changes; incremental algorithms (replanning, caches) subscribe to this.</summary>
public interface IGridChangeListener
{
    /// <summary>Called after the cell (<paramref name="x"/>, <paramref name="y"/>) changed area (including becoming or ceasing to be a wall).</summary>
    /// <param name="x">Column of the cell.</param>
    /// <param name="y">Row of the cell.</param>
    void OnCellChanged(int x, int y);
}

/// <summary>
/// A square grid: one byte per cell holding an area id (0..254, see <see cref="AreaCosts"/>) or <see cref="AreaCosts.Wall"/>.
/// This is the tier-1 entry point (<c>Grid.From(tiles).FindPath(a, b)</c>); tier 2 and 3 build on
/// <see cref="CreateTopology"/>, <see cref="AStar{TTopology, TCost, THeuristic}"/> and <see cref="OpenSet"/>.
/// Cells are indexed row-major: <c>index = y * Width + x</c>.
/// <para>
/// Reading is safe from several threads once no thread edits the grid. Edits (<see cref="SetArea"/>) must not overlap queries.
/// </para>
/// </summary>
public sealed class Grid
{
    /// <summary>The largest number of cells a grid may have (2^28).</summary>
    public const int MaxCells = 1 << 28;

    private static readonly GridOptions DefaultOptions = new();

    private readonly byte[] _cells;
    private readonly byte[] _clearance;
    private readonly object _sync = new();
    private readonly List<IGridChangeListener> _listeners = [];
    private int _clearanceVersion = -1;
    private readonly List<ClearanceRect> _dirty = [];
    private readonly List<int> _changedScratch = [];
    private GridSearchBuffers? _spare;
    private JpsPlusIndex? _jpsPlus;
    private BlockMap? _blockMap;

    /// <summary>Creates a grid of walkable cells (area 0).</summary>
    /// <param name="width">Columns; at least 1.</param>
    /// <param name="height">Rows; at least 1.</param>
    /// <exception cref="ArgumentOutOfRangeException">A dimension is below 1 or the grid would exceed <see cref="MaxCells"/> cells.</exception>
    public Grid(int width, int height)
    {
        CheckSize(width, height);
        Width = width;
        Height = height;
        _cells = new byte[width * height];
        _clearance = new byte[_cells.Length];
        Areas = new AreaCosts();
        Space = GridSpace.Unit;
    }

    /// <summary>Columns.</summary>
    public int Width { get; }

    /// <summary>Rows.</summary>
    public int Height { get; }

    /// <summary>Number of cells.</summary>
    public int CellCount => _cells.Length;

    /// <summary>The area cost table. Edit it before querying; changes must not overlap a running search.</summary>
    public AreaCosts Areas { get; }

    /// <summary>The world-space mapping. Default: unit cells, origin (0, 0).</summary>
    public GridSpace Space { get; set; }

    /// <summary>Incremented by every cell edit.</summary>
    public int Version { get; private set; }

    /// <summary>The cell array (row-major area ids), read-only.</summary>
    public ReadOnlySpan<byte> Cells => _cells;

    internal byte[] CellArray => _cells;

    internal byte[] ClearanceArray
    {
        get
        {
            EnsureClearance();
            return _clearance;
        }
    }

    /// <summary>Creates a grid from a 2D array indexed <c>[row, column]</c>; <see langword="true"/> is walkable.</summary>
    /// <param name="walkable">Cells, rows first.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="walkable"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The array is empty or too large.</exception>
    public static Grid From(bool[,] walkable)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        int h = walkable.GetLength(0);
        int w = walkable.GetLength(1);
        var grid = new Grid(w, h);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                grid._cells[(y * w) + x] = walkable[y, x] ? (byte)0 : AreaCosts.Wall;
            }
        }
        return grid;
    }

    /// <summary>Creates a grid from row-major flags; <see langword="true"/> is walkable.</summary>
    /// <param name="walkable">Cells, row by row.</param>
    /// <param name="width">Columns.</param>
    /// <param name="height">Rows.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The size is invalid.</exception>
    /// <exception cref="ArgumentException">The span length is not <c>width * height</c>.</exception>
    public static Grid From(ReadOnlySpan<bool> walkable, int width, int height)
    {
        var grid = new Grid(width, height);
        if (walkable.Length != grid._cells.Length)
        {
            throw new ArgumentException($"Expected {grid._cells.Length} cells for a {width}x{height} grid but got {walkable.Length}.", nameof(walkable));
        }
        for (int i = 0; i < walkable.Length; i++)
        {
            grid._cells[i] = walkable[i] ? (byte)0 : AreaCosts.Wall;
        }
        return grid;
    }

    /// <summary>Creates a grid from row-major area ids (255 is a wall).</summary>
    /// <param name="areas">Cells, row by row.</param>
    /// <param name="width">Columns.</param>
    /// <param name="height">Rows.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The size is invalid.</exception>
    /// <exception cref="ArgumentException">The span length is not <c>width * height</c>.</exception>
    public static Grid From(ReadOnlySpan<byte> areas, int width, int height)
    {
        var grid = new Grid(width, height);
        if (areas.Length != grid._cells.Length)
        {
            throw new ArgumentException($"Expected {grid._cells.Length} cells for a {width}x{height} grid but got {areas.Length}.", nameof(areas));
        }
        areas.CopyTo(grid._cells);
        return grid;
    }

    /// <summary>Whether (<paramref name="x"/>, <paramref name="y"/>) lies inside the grid.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>True if inside.</returns>
    public bool Contains(int x, int y) => (uint)x < (uint)Width && (uint)y < (uint)Height;

    /// <summary>The area id of a cell.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>0..254, or <see cref="AreaCosts.Wall"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public byte GetArea(int x, int y)
    {
        CheckCell(x, y);
        return _cells[(y * Width) + x];
    }

    /// <summary>True if the cell is not a wall.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>True if walkable.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public bool IsWalkable(int x, int y) => GetArea(x, y) != AreaCosts.Wall;

    /// <summary>Sets the area of a cell (use <see cref="AreaCosts.Wall"/> for a wall) and notifies listeners if it changed.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <param name="area">New area id.</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void SetArea(int x, int y, byte area)
    {
        CheckCell(x, y);
        int i = (y * Width) + x;
        if (_cells[i] == area)
        {
            return;
        }
        _cells[i] = area;
        Version++;
        MarkDirty(x, y, x, y);
        for (int l = 0; l < _listeners.Count; l++)
        {
            _listeners[l].OnCellChanged(x, y);
        }
    }

    /// <summary>
    /// Sets every cell of a rectangle (a tile) to one area as a single edit: <see cref="Version"/> goes up once, the clearance
    /// is repaired only around the cells that changed, and listeners hear about it once
    /// (<see cref="IGridBatchListener"/>) or once per changed cell in row-major order (plain <see cref="IGridChangeListener"/>).
    /// Nothing happens (no version bump, no notification) when no cell changes.
    /// </summary>
    /// <param name="x">Left column.</param>
    /// <param name="y">Top row.</param>
    /// <param name="width">Columns, 0 or more.</param>
    /// <param name="height">Rows, 0 or more.</param>
    /// <param name="area">The area id to write (use <see cref="AreaCosts.Wall"/> for walls).</param>
    /// <exception cref="ArgumentOutOfRangeException">The rectangle is not inside the grid.</exception>
    public void SetRect(int x, int y, int width, int height, byte area) =>
        ApplyRect(x, y, width, height, area, default, false);

    /// <summary>
    /// Overwrites a rectangle (a tile) with row-major area ids as a single edit; see <see cref="SetRect(int, int, int, int, byte)"/>.
    /// </summary>
    /// <param name="x">Left column.</param>
    /// <param name="y">Top row.</param>
    /// <param name="width">Columns, 0 or more.</param>
    /// <param name="height">Rows, 0 or more.</param>
    /// <param name="areas">Exactly <c>width * height</c> area ids, row by row.</param>
    /// <exception cref="ArgumentOutOfRangeException">The rectangle is not inside the grid.</exception>
    /// <exception cref="ArgumentException"><paramref name="areas"/> has the wrong length.</exception>
    public void SetRect(int x, int y, int width, int height, ReadOnlySpan<byte> areas) =>
        ApplyRect(x, y, width, height, 0, areas, true);

    private void ApplyRect(int x, int y, int width, int height, byte fill, ReadOnlySpan<byte> source, bool useSource)
    {
        if (!RectEdit.Apply(_cells, Width, Height, x, y, width, height, fill, source, useSource, _changedScratch, out int x0, out int y0, out int x1, out int y1))
        {
            return;
        }
        Version++;
        MarkDirty(x0, y0, x1, y1);
        RectEdit.Notify(_listeners, _changedScratch, Width, x0, y0, x1, y1);
    }

    /// <summary>Makes a cell a wall or an area-0 floor.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <param name="walkable">True for floor (area 0), false for a wall.</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void SetWalkable(int x, int y, bool walkable) => SetArea(x, y, walkable ? (byte)0 : AreaCosts.Wall);

    /// <summary>Registers a listener for cell edits.</summary>
    /// <param name="listener">The listener.</param>
    /// <exception cref="ArgumentNullException">The listener is null.</exception>
    public void AddListener(IGridChangeListener listener)
    {
        ArgumentNullException.ThrowIfNull(listener);
        _listeners.Add(listener);
    }

    /// <summary>Unregisters a listener.</summary>
    /// <param name="listener">The listener.</param>
    /// <returns>True if it was registered.</returns>
    public bool RemoveListener(IGridChangeListener listener) => _listeners.Remove(listener);

    /// <summary>
    /// True clearance of a cell: the side of the largest square of walkable cells whose top-left corner is this cell
    /// (0 for a wall; capped at 255). An agent of size s can stand at the cell iff the clearance is at least s.
    /// </summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <returns>The clearance.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public int GetClearance(int x, int y)
    {
        CheckCell(x, y);
        return ClearanceArray[(y * Width) + x];
    }

    /// <summary>The clearance of every cell, row-major (see <see cref="GetClearance"/>); recomputed after edits.</summary>
    public ReadOnlySpan<byte> Clearance => ClearanceArray;

    /// <summary>The centre of a cell in world space.</summary>
    /// <param name="cell">The cell.</param>
    /// <returns>The world position.</returns>
    public WorldPoint CellCenter(Cell cell)
    {
        GridSpace s = Space;
        double size = s.SizeOrOne;
        return new WorldPoint(s.OriginX + ((cell.X + 0.5) * size), s.OriginY + ((cell.Y + 0.5) * size));
    }

    /// <summary>The cell containing a world position.</summary>
    /// <param name="point">The position; each coordinate within ±2^21.</param>
    /// <param name="cell">The cell, when inside the grid.</param>
    /// <returns>False if the position lies outside the grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    public bool TryGetCell(WorldPoint point, out Cell cell)
    {
        GridSpace.CheckCoordinate(point.X, nameof(point));
        GridSpace.CheckCoordinate(point.Y, nameof(point));
        GridSpace s = Space;
        double size = s.SizeOrOne;
        double fx = Math.Floor((point.X - s.OriginX) / size);
        double fy = Math.Floor((point.Y - s.OriginY) / size);
        if (fx >= 0.0 && fx < Width && fy >= 0.0 && fy < Height)
        {
            cell = new Cell((int)fx, (int)fy);
            return true;
        }
        cell = default;
        return false;
    }

    /// <summary>Creates the search topology for an agent size and movement rule.</summary>
    /// <param name="agentSize">Agent size in cells, 1..255.</param>
    /// <param name="connectivity">Four or eight moves.</param>
    /// <param name="diagonals">Diagonal rule (eight-connected only).</param>
    /// <returns>A view of this grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The agent size is outside 1..255.</exception>
    public SquareTopology CreateTopology(int agentSize = 1, Connectivity connectivity = Connectivity.Eight, DiagonalRule diagonals = DiagonalRule.NoCornerCutting)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(agentSize, 1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(agentSize, 255);
        return new SquareTopology(this, agentSize, connectivity, diagonals);
    }

    /// <summary>Finds a least-cost path between two cells.</summary>
    /// <param name="start">Start cell.</param>
    /// <param name="goal">Goal cell.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>The result; check <see cref="GridPath.Status"/>. Cells outside the grid or not walkable for the agent give <see cref="PathStatus.StartInvalid"/> or <see cref="PathStatus.GoalInvalid"/>.</returns>
    public GridPath FindPath(Cell start, Cell goal, GridOptions? options = null) => FindPathCore(start, goal, options ?? DefaultOptions, false);

    /// <summary>Finds a least-cost path between two world positions (their containing cells); the result carries world-space cell centres.</summary>
    /// <param name="start">Start position.</param>
    /// <param name="goal">Goal position.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>The result. Positions outside the grid give <see cref="PathStatus.StartInvalid"/> or <see cref="PathStatus.GoalInvalid"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    public GridPath FindPath(WorldPoint start, WorldPoint goal, GridOptions? options = null)
    {
        bool s = TryGetCell(start, out Cell sc);
        bool g = TryGetCell(goal, out Cell gc);
        if (!s)
        {
            return new GridPath(PathStatus.StartInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        if (!g)
        {
            return new GridPath(PathStatus.GoalInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        return FindPathCore(sc, gc, options ?? DefaultOptions, true);
    }

    /// <summary>
    /// Creates an incremental search that keeps the path from <paramref name="start"/> to <paramref name="goal"/> up to date as the grid
    /// is edited and the start moves (LPA* when <see cref="GridOptions.Algorithm"/> is <see cref="GridAlgorithm.LpaStar"/>, otherwise D* Lite).
    /// Call <see cref="GridReplanner.Replan()"/> for the first path and after changes; dispose it to stop listening.
    /// </summary>
    /// <param name="start">Start cell.</param>
    /// <param name="goal">Goal cell.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>. The heuristic weight must be 1.</param>
    /// <returns>The replanner.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A cell is outside the grid.</exception>
    /// <exception cref="ArgumentException">The heuristic weight is not 1.</exception>
    public GridReplanner CreateReplanner(Cell start, Cell goal, GridOptions? options = null) => new(this, start, goal, options ?? DefaultOptions);

    private GridPath FindPathCore(Cell start, Cell goal, GridOptions o, bool world)
    {
        EnsureClearance();
        int size = o.AgentSize;
        if (!Contains(start.X, start.Y) || _clearance[(start.Y * Width) + start.X] < size || o.Blocked?.Invoke(start) == true)
        {
            return new GridPath(PathStatus.StartInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        if (!Contains(goal.X, goal.Y) || _clearance[(goal.Y * Width) + goal.X] < size || o.Blocked?.Invoke(goal) == true)
        {
            return new GridPath(PathStatus.GoalInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        if (o.Algorithm is GridAlgorithm.Jps or GridAlgorithm.JpsPlus && JumpSearchApplies(o, out double unitCost))
        {
            return FindPathJump(start, goal, o, unitCost, world);
        }
        SquareTopology topology = CreateTopology(size, o.Connectivity, o.Diagonals);
        GridHeuristic h = o.Heuristic;
        if (h == GridHeuristic.Auto)
        {
            h = o.Connectivity == Connectivity.Four ? GridHeuristic.Manhattan : GridHeuristic.Octile;
        }
        double scale = Areas.MinCost;
        int a = (start.Y * Width) + start.X;
        int b = (goal.Y * Width) + goal.X;
        Func<Cell, WorldPoint>? centre = world ? CellCenter : null;

        GridSearchBuffers? buffers = Interlocked.Exchange(ref _spare, null);
        buffers ??= new GridSearchBuffers(CellCount);
        try
        {
            if (o.StepCost is null && o.Blocked is null)
            {
                var cost = default(DefaultStepCost);
                return h switch
                {
                    GridHeuristic.Zero => SearchRunner.Run(topology, cost, default(ZeroHeuristic), buffers, Width, a, b, o, centre),
                    GridHeuristic.Manhattan => SearchRunner.Run(topology, cost, new ManhattanHeuristic(Width, scale), buffers, Width, a, b, o, centre),
                    _ => SearchRunner.Run(topology, cost, new OctileHeuristic(Width, scale), buffers, Width, a, b, o, centre),
                };
            }
            else
            {
                var cost = new SearchRunner.DelegateStepCost { Width = Width, Step = o.StepCost, Blocked = o.Blocked };
                return h switch
                {
                    GridHeuristic.Zero => SearchRunner.Run(topology, cost, default(ZeroHeuristic), buffers, Width, a, b, o, centre),
                    GridHeuristic.Manhattan => SearchRunner.Run(topology, cost, new ManhattanHeuristic(Width, scale), buffers, Width, a, b, o, centre),
                    _ => SearchRunner.Run(topology, cost, new OctileHeuristic(Width, scale), buffers, Width, a, b, o, centre),
                };
            }
        }
        finally
        {
            _spare = buffers;
        }
    }

    // The jump point searches assume what the one-line API can promise: eight moves, no corner cutting, agent size 1, one
    // cost for every area, no per-move callbacks, an exact (weight 1) octile or zero estimate.
    private bool JumpSearchApplies(GridOptions o, out double unitCost)
    {
        unitCost = 1.0;
        return o.Connectivity == Connectivity.Eight
            && o.Diagonals == DiagonalRule.NoCornerCutting
            && o.AgentSize == 1
            && o.StepCost is null
            && o.Blocked is null
            && o.HeuristicWeight == 1.0
            && o.Heuristic != GridHeuristic.Manhattan
            && Areas.TryGetUniform(out unitCost);
    }

    private GridPath FindPathJump(Cell start, Cell goal, GridOptions o, double unitCost, bool world)
    {
        int a = (start.Y * Width) + start.X;
        int b = (goal.Y * Width) + goal.X;
        Func<Cell, WorldPoint>? centre = world ? CellCenter : null;
        bool plus = o.Algorithm == GridAlgorithm.JpsPlus && Width <= JpsPlusIndex.MaxSide && Height <= JpsPlusIndex.MaxSide;
        TableJumper table = default;
        BlockJumper block = default;
        lock (_sync)
        {
            if (plus)
            {
                _jpsPlus ??= new JpsPlusIndex(this);
                _jpsPlus.Refresh();
                table = new TableJumper(_jpsPlus);
            }
            else
            {
                _blockMap ??= new BlockMap(this);
                block = new BlockJumper(_blockMap);
            }
        }
        GridSearchBuffers? buffers = Interlocked.Exchange(ref _spare, null);
        buffers ??= new GridSearchBuffers(CellCount);
        try
        {
            GridAlgorithm used = plus ? GridAlgorithm.JpsPlus : GridAlgorithm.Jps;
            bool zero = o.Heuristic == GridHeuristic.Zero;
            if (plus)
            {
                return zero
                    ? SearchRunner.RunJump(table, default(ZeroHeuristic), unitCost, buffers, a, b, o, centre, used)
                    : SearchRunner.RunJump(table, new OctileHeuristic(Width, unitCost), unitCost, buffers, a, b, o, centre, used);
            }
            return zero
                ? SearchRunner.RunJump(block, default(ZeroHeuristic), unitCost, buffers, a, b, o, centre, used)
                : SearchRunner.RunJump(block, new OctileHeuristic(Width, unitCost), unitCost, buffers, a, b, o, centre, used);
        }
        finally
        {
            _spare = buffers;
        }
    }

    private void EnsureClearance()
    {
        if (_clearanceVersion == Version)
        {
            return;
        }
        lock (_sync)
        {
            if (_clearanceVersion == Version)
            {
                return;
            }
            if (_clearanceVersion < 0)
            {
                ComputeClearance();
            }
            else
            {
                RepairClearance();
            }
            _dirty.Clear();
            _clearanceVersion = Version;
        }
    }

    private readonly record struct ClearanceRect(int X0, int Y0, int X1, int Y1);

    // Remember which cells were edited since the clearance was last current (only once it has been computed at all).
    private void MarkDirty(int x0, int y0, int x1, int y1)
    {
        if (_clearanceVersion < 0)
        {
            return;
        }
        if (_dirty.Count >= 32)
        {
            // Too many separate edits: one bounding box is cheaper to track than a long list.
            int ax = x0, ay = y0, bx = x1, by = y1;
            foreach (ClearanceRect r in _dirty)
            {
                ax = Math.Min(ax, r.X0);
                ay = Math.Min(ay, r.Y0);
                bx = Math.Max(bx, r.X1);
                by = Math.Max(by, r.Y1);
            }
            _dirty.Clear();
            _dirty.Add(new ClearanceRect(ax, ay, bx, by));
            return;
        }
        _dirty.Add(new ClearanceRect(x0, y0, x1, y1));
    }

    // Incremental form of ComputeClearance. The clearance of a cell reads only the cells right of and below it, so a cell can
    // change only if it was edited or the cell to its right, below it or diagonally below-right changed. Rows are repaired
    // bottom to top, right to left, touching an edited cell, then following changes leftwards and upwards until a value
    // comes out unchanged. The result equals a full recompute.
    private void RepairClearance()
    {
        int w = Width;
        int h = Height;
        int maxY = -1;
        int minY = int.MaxValue;
        foreach (ClearanceRect r in _dirty)
        {
            maxY = Math.Max(maxY, r.Y1);
            minY = Math.Min(minY, r.Y0);
        }
        int prevLo = int.MaxValue;
        int prevHi = -1;
        for (int y = maxY; y >= 0; y--)
        {
            int lo = prevHi >= 0 ? prevLo - 1 : int.MaxValue;
            int hi = prevHi;
            foreach (ClearanceRect r in _dirty)
            {
                if (y >= r.Y0 && y <= r.Y1)
                {
                    lo = Math.Min(lo, r.X0);
                    hi = Math.Max(hi, r.X1);
                }
            }
            if (hi < 0)
            {
                if (y < minY)
                {
                    break;
                }
                prevLo = int.MaxValue;
                prevHi = -1;
                continue;
            }
            lo = Math.Max(lo, 0);
            hi = Math.Min(hi, w - 1);
            int row = y * w;
            int changedLo = int.MaxValue;
            int changedHi = -1;
            for (int x = hi; x >= 0; x--)
            {
                byte value;
                if (_cells[row + x] == AreaCosts.Wall)
                {
                    value = 0;
                }
                else
                {
                    int right = x + 1 < w ? _clearance[row + x + 1] : 0;
                    int down = y + 1 < h ? _clearance[row + w + x] : 0;
                    int diag = x + 1 < w && y + 1 < h ? _clearance[row + w + x + 1] : 0;
                    int m = Math.Min(right, Math.Min(down, diag));
                    value = (byte)(m < 255 ? m + 1 : 255);
                }
                bool changed = _clearance[row + x] != value;
                _clearance[row + x] = value;
                if (changed)
                {
                    changedLo = x;
                    changedHi = Math.Max(changedHi, x);
                }
                else if (x <= lo)
                {
                    break;
                }
            }
            prevLo = changedLo;
            prevHi = changedHi;
        }
    }

    // True clearance by dynamic programming from the bottom-right corner:
    // c(x,y) = 0 for a wall, else 1 + min(c(x+1,y), c(x,y+1), c(x+1,y+1)), cells outside the grid counting 0.
    // The largest all-walkable square with its top-left corner at (x,y) is one cell larger than the
    // smallest of the three squares starting one step right, down and diagonally (Harabor and Botea 2008 define
    // clearance this way for agents occupying a square of cells).
    private void ComputeClearance()
    {
        int w = Width;
        int h = Height;
        for (int y = h - 1; y >= 0; y--)
        {
            int row = y * w;
            for (int x = w - 1; x >= 0; x--)
            {
                if (_cells[row + x] == AreaCosts.Wall)
                {
                    _clearance[row + x] = 0;
                    continue;
                }
                int right = x + 1 < w ? _clearance[row + x + 1] : 0;
                int down = y + 1 < h ? _clearance[row + w + x] : 0;
                int diag = x + 1 < w && y + 1 < h ? _clearance[row + w + x + 1] : 0;
                int m = Math.Min(right, Math.Min(down, diag));
                _clearance[row + x] = (byte)(m < 255 ? m + 1 : 255);
            }
        }
    }

    private void CheckCell(int x, int y)
    {
        if (!Contains(x, y))
        {
            throw new ArgumentOutOfRangeException(x >= 0 && x < Width ? nameof(y) : nameof(x), $"Cell ({x}, {y}) is outside the {Width}x{Height} grid.");
        }
    }

    internal static void CheckSize(int width, int height)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(width, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(height, 1);
        if ((long)width * height > MaxCells)
        {
            throw new ArgumentOutOfRangeException(nameof(width), width, $"A grid may have at most {MaxCells} cells; {width}x{height} is too large.");
        }
    }
}
