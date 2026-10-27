using System;
using System.Collections.Generic;
using System.Threading;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

/// <summary>
/// A hexagonal grid stored as a rectangular offset array (see <see cref="HexLayout"/>), with the same area-cost
/// model as <see cref="Grid"/>. Moves go to the six neighbours at cost <c>(m[from] + m[to]) * 0.5</c>; the default
/// estimate is the hex distance. Agents are one hex wide. Cells are indexed row-major: <c>index = row * Width + col</c>,
/// and <see cref="Cell"/>.X is the column, .Y the row.
/// <para>
/// World mapping: <see cref="Space"/>'s origin is the centre of cell (0, 0) and its cell size is the distance between
/// adjacent centres. Pointy layouts put axial (q, r) at <c>(q + r/2, (√3/2) r) * size</c>, flat layouts at
/// <c>((√3/2) q, r + q/2) * size</c>.
/// </para>
/// </summary>
public sealed partial class HexGrid
{
    private static readonly GridOptions DefaultOptions = new();

    private readonly byte[] _cells;
    private readonly List<IGridChangeListener> _listeners = [];
    private readonly List<int> _changedScratch = [];
    private GridSearchBuffers? _spare;

    /// <summary>Creates a grid of walkable hexes.</summary>
    /// <param name="width">Columns; at least 1.</param>
    /// <param name="height">Rows; at least 1.</param>
    /// <param name="layout">The offset layout. Default pointy-topped with odd rows shifted right.</param>
    /// <exception cref="ArgumentOutOfRangeException">A dimension is below 1, the grid is too large, or the layout is undefined.</exception>
    public HexGrid(int width, int height, HexLayout layout = HexLayout.PointyOddRow)
    {
        Grid.CheckSize(width, height);
        if (layout is < HexLayout.PointyOddRow or > HexLayout.FlatEvenColumn)
        {
            throw new ArgumentOutOfRangeException(nameof(layout), layout, "Unknown hex layout.");
        }
        Width = width;
        Height = height;
        Layout = layout;
        _cells = new byte[width * height];
        Areas = new AreaCosts();
        Space = GridSpace.Unit;
    }

    /// <summary>Columns.</summary>
    public int Width { get; }

    /// <summary>Rows.</summary>
    public int Height { get; }

    /// <summary>The offset layout.</summary>
    public HexLayout Layout { get; }

    /// <summary>Number of cells.</summary>
    public int CellCount => _cells.Length;

    /// <summary>The area cost table.</summary>
    public AreaCosts Areas { get; }

    /// <summary>World mapping (origin = centre of cell (0, 0); cell size = spacing of adjacent centres).</summary>
    public GridSpace Space { get; set; }

    /// <summary>Incremented by every cell edit.</summary>
    public int Version { get; private set; }

    /// <summary>The cell array (row-major area ids), read-only.</summary>
    public ReadOnlySpan<byte> Cells => _cells;

    internal byte[] CellArray => _cells;

    /// <summary>Creates a hex grid from a 2D array indexed <c>[row, column]</c>; <see langword="true"/> is walkable.</summary>
    /// <param name="walkable">Cells, rows first.</param>
    /// <param name="layout">The offset layout of the array.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="walkable"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The array is empty or too large, or the layout is undefined.</exception>
    public static HexGrid From(bool[,] walkable, HexLayout layout = HexLayout.PointyOddRow)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        int h = walkable.GetLength(0);
        int w = walkable.GetLength(1);
        var grid = new HexGrid(w, h, layout);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                grid._cells[(y * w) + x] = walkable[y, x] ? (byte)0 : AreaCosts.Wall;
            }
        }
        return grid;
    }

    /// <summary>Creates a hex grid from row-major area ids (255 is a wall).</summary>
    /// <param name="areas">Cells, row by row.</param>
    /// <param name="width">Columns.</param>
    /// <param name="height">Rows.</param>
    /// <param name="layout">The offset layout.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The size or layout is invalid.</exception>
    /// <exception cref="ArgumentException">The span length is not <c>width * height</c>.</exception>
    public static HexGrid From(ReadOnlySpan<byte> areas, int width, int height, HexLayout layout = HexLayout.PointyOddRow)
    {
        var grid = new HexGrid(width, height, layout);
        if (areas.Length != grid._cells.Length)
        {
            throw new ArgumentException($"Expected {grid._cells.Length} cells for a {width}x{height} grid but got {areas.Length}.", nameof(areas));
        }
        areas.CopyTo(grid._cells);
        return grid;
    }

    /// <summary>
    /// Creates a grid from a text picture, one line per row of the offset array; the characters are those of <see cref="Grid.FromText"/>
    /// (<c>.</c> open, <c>#</c> wall, <c>1</c>–<c>9</c> areas).
    /// </summary>
    /// <param name="text">The picture.</param>
    /// <param name="layout">The offset layout.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="GridFormatException">The picture is malformed (see <see cref="Grid.FromText"/>).</exception>
    public static HexGrid FromText(string text, HexLayout layout = HexLayout.PointyOddRow)
    {
        byte[] cells = Grid.ParseText(text, out int width, out int height);
        return From(cells, width, height, layout);
    }

    /// <summary>Whether (<paramref name="col"/>, <paramref name="row"/>) lies inside the grid.</summary>
    /// <param name="col">Column.</param>
    /// <param name="row">Row.</param>
    /// <returns>True if inside.</returns>
    public bool Contains(int col, int row) => (uint)col < (uint)Width && (uint)row < (uint)Height;

    /// <summary>The area id of a cell.</summary>
    /// <param name="col">Column.</param>
    /// <param name="row">Row.</param>
    /// <returns>0..254, or <see cref="AreaCosts.Wall"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public byte GetArea(int col, int row)
    {
        CheckCell(col, row);
        return _cells[(row * Width) + col];
    }

    /// <summary>Sets the area of a cell and notifies listeners if it changed.</summary>
    /// <param name="col">Column.</param>
    /// <param name="row">Row.</param>
    /// <param name="area">New area id (255 = wall).</param>
    /// <exception cref="ArgumentOutOfRangeException">The cell is outside the grid.</exception>
    public void SetArea(int col, int row, byte area)
    {
        CheckCell(col, row);
        int i = (row * Width) + col;
        if (_cells[i] == area)
        {
            return;
        }
        _cells[i] = area;
        Version++;
        for (int l = 0; l < _listeners.Count; l++)
        {
            _listeners[l].OnCellChanged(col, row);
        }
    }

    /// <summary>Sets every cell of a rectangle (a tile) to one area as a single edit; see <see cref="Grid.SetRect(int, int, int, int, byte)"/>.</summary>
    /// <param name="col">Left column.</param>
    /// <param name="row">Top row.</param>
    /// <param name="width">Columns, 0 or more.</param>
    /// <param name="height">Rows, 0 or more.</param>
    /// <param name="area">New area id (255 = wall).</param>
    /// <exception cref="ArgumentOutOfRangeException">The rectangle is not inside the grid.</exception>
    public void SetRect(int col, int row, int width, int height, byte area) => ApplyRect(col, row, width, height, area, default, false);

    /// <summary>Overwrites a rectangle (a tile) with row-major area ids as a single edit; see <see cref="Grid.SetRect(int, int, int, int, ReadOnlySpan{byte})"/>.</summary>
    /// <param name="col">Left column.</param>
    /// <param name="row">Top row.</param>
    /// <param name="width">Columns, 0 or more.</param>
    /// <param name="height">Rows, 0 or more.</param>
    /// <param name="areas">Exactly <c>width * height</c> area ids, row by row.</param>
    /// <exception cref="ArgumentOutOfRangeException">The rectangle is not inside the grid.</exception>
    /// <exception cref="ArgumentException"><paramref name="areas"/> has the wrong length.</exception>
    public void SetRect(int col, int row, int width, int height, ReadOnlySpan<byte> areas) => ApplyRect(col, row, width, height, 0, areas, true);

    private void ApplyRect(int x, int y, int width, int height, byte fill, ReadOnlySpan<byte> source, bool useSource)
    {
        if (!RectEdit.Apply(_cells, Width, Height, x, y, width, height, fill, source, useSource, _changedScratch, out int x0, out int y0, out int x1, out int y1))
        {
            return;
        }
        Version++;
        RectEdit.Notify(_listeners, _changedScratch, Width, x0, y0, x1, y1);
    }

    /// <summary>Registers a listener for cell edits.</summary>
    /// <param name="listener">The listener.</param>
    /// <exception cref="ArgumentNullException">The listener is null.</exception>
    public void AddListener(IGridChangeListener listener)
    {
        ArgumentNullException.ThrowIfNull(listener);
        lock (_listeners)
        {
            _listeners.Add(listener); // locked so several flow fields may be created on different threads
        }
    }

    /// <summary>Unregisters a listener.</summary>
    /// <param name="listener">The listener.</param>
    /// <returns>True if it was registered.</returns>
    public bool RemoveListener(IGridChangeListener listener)
    {
        lock (_listeners)
        {
            return _listeners.Remove(listener);
        }
    }

    /// <summary>The axial coordinates of a cell.</summary>
    /// <param name="cell">The cell (column, row).</param>
    /// <returns>Axial (q, r).</returns>
    public HexCoord ToAxial(Cell cell) => HexCoord.FromOffset(Layout, cell.X, cell.Y);

    /// <summary>The centre of a cell in world space.</summary>
    /// <param name="cell">The cell.</param>
    /// <returns>The world position.</returns>
    public WorldPoint CellCenter(Cell cell)
    {
        HexCoord a = ToAxial(cell);
        GridSpace s = Space;
        double size = s.SizeOrOne;
        if (Layout is HexLayout.PointyOddRow or HexLayout.PointyEvenRow)
        {
            return new WorldPoint(s.OriginX + (size * (a.Q + (a.R * 0.5))), s.OriginY + (size * GridMath.HalfSqrt3 * a.R));
        }
        return new WorldPoint(s.OriginX + (size * GridMath.HalfSqrt3 * a.Q), s.OriginY + (size * (a.R + (a.Q * 0.5))));
    }

    /// <summary>The cell whose hexagon contains a world position (fractional axial coordinates rounded to the nearest hex centre; exact ties round up).</summary>
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
        double px = (point.X - s.OriginX) / size;
        double py = (point.Y - s.OriginY) / size;
        double fq;
        double fr;
        if (Layout is HexLayout.PointyOddRow or HexLayout.PointyEvenRow)
        {
            fr = py / GridMath.HalfSqrt3;
            fq = px - (fr * 0.5);
        }
        else
        {
            fq = px / GridMath.HalfSqrt3;
            fr = py - (fq * 0.5);
        }
        const double Limit = 1073741824.0; // keeps the integer conversions below well inside int
        cell = default;
        if (!(Math.Abs(fq) < Limit && Math.Abs(fr) < Limit))
        {
            return false;
        }

        // Cube rounding: round each coordinate, then repair the one with the largest rounding error so q + r + s = 0.
        double fs = -fq - fr;
        double rq = Math.Floor(fq + 0.5);
        double rr = Math.Floor(fr + 0.5);
        double rs = Math.Floor(fs + 0.5);
        double dq = Math.Abs(rq - fq);
        double dr = Math.Abs(rr - fr);
        double ds = Math.Abs(rs - fs);
        if (dq > dr && dq > ds)
        {
            rq = -rr - rs;
        }
        else if (dr > ds)
        {
            rr = -rq - rs;
        }
        Cell c = new HexCoord((int)rq, (int)rr).ToOffset(Layout);
        if (!Contains(c.X, c.Y))
        {
            return false;
        }
        cell = c;
        return true;
    }

    /// <summary>Creates the search topology.</summary>
    /// <returns>A view of this grid.</returns>
    public HexTopology CreateTopology() => new(this);

    /// <summary>Finds a least-cost path between two cells.</summary>
    /// <param name="start">Start cell (column, row).</param>
    /// <param name="goal">Goal cell.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>. Only the heuristic, weight, step cost, blocked filter, recording and iteration budget apply; the agent size must be 1.</param>
    /// <returns>The result; check <see cref="GridPath.Status"/>.</returns>
    /// <exception cref="ArgumentException">The options ask for an agent size other than 1.</exception>
    public GridPath FindPath(Cell start, Cell goal, GridOptions? options = null) => FindPathCore(start, goal, options ?? DefaultOptions, false);

    /// <summary>Finds a least-cost path between two world positions; the result carries world-space cell centres.</summary>
    /// <param name="start">Start position.</param>
    /// <param name="goal">Goal position.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>The result. Positions outside the grid give <see cref="PathOutcome.StartInvalid"/> or <see cref="PathOutcome.GoalInvalid"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    /// <exception cref="ArgumentException">The options ask for an agent size other than 1.</exception>
    public GridPath FindPath(WorldPoint start, WorldPoint goal, GridOptions? options = null)
    {
        bool s = TryGetCell(start, out Cell sc);
        bool g = TryGetCell(goal, out Cell gc);
        if (!s)
        {
            return new GridPath(PathOutcome.StartInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        if (!g)
        {
            return new GridPath(PathOutcome.GoalInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        return FindPathCore(sc, gc, options ?? DefaultOptions, true);
    }

    /// <summary>Creates an incremental search that keeps the path up to date as the grid is edited and the start moves; see <see cref="Grid.CreateReplanner"/>.</summary>
    /// <param name="start">Start cell (column, row).</param>
    /// <param name="goal">Goal cell.</param>
    /// <param name="options">Query options; null means <see cref="GridOptions.Default"/>. Agent size must be 1, heuristic weight 1.</param>
    /// <returns>The replanner.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A cell is outside the grid.</exception>
    /// <exception cref="ArgumentException">The agent size is not 1 or the heuristic weight is not 1.</exception>
    public HexReplanner CreateReplanner(Cell start, Cell goal, GridOptions? options = null) => new(this, start, goal, options ?? DefaultOptions);

    private GridPath FindPathCore(Cell start, Cell goal, GridOptions o, bool world)
    {
        if (o.AgentSize != 1)
        {
            throw new ArgumentException("Hex grids support only agent size 1.", nameof(o));
        }
        if (!Contains(start.X, start.Y) || _cells[(start.Y * Width) + start.X] == AreaCosts.Wall || o.Blocked?.Invoke(start) == true)
        {
            return new GridPath(PathOutcome.StartInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        if (!Contains(goal.X, goal.Y) || _cells[(goal.Y * Width) + goal.X] == AreaCosts.Wall || o.Blocked?.Invoke(goal) == true)
        {
            return new GridPath(PathOutcome.GoalInvalid, double.PositiveInfinity, null, 0, null, null);
        }
        HexTopology topology = CreateTopology();
        bool zero = o.Heuristic == GridHeuristic.Zero;
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
                return zero
                    ? SearchRunner.Run(topology, cost, default(ZeroHeuristic), buffers, Width, a, b, o, centre)
                    : SearchRunner.Run(topology, cost, new HexHeuristic(Width, Layout, scale), buffers, Width, a, b, o, centre);
            }
            else
            {
                var cost = new SearchRunner.DelegateStepCost { Width = Width, Step = o.StepCost, Blocked = o.Blocked };
                return zero
                    ? SearchRunner.Run(topology, cost, default(ZeroHeuristic), buffers, Width, a, b, o, centre)
                    : SearchRunner.Run(topology, cost, new HexHeuristic(Width, Layout, scale), buffers, Width, a, b, o, centre);
            }
        }
        finally
        {
            _spare = buffers;
        }
    }

    /// <summary>Computes a flow field: the least cost from every cell to the nearest of the goal cells, and the step each cell takes.</summary>
    /// <param name="goals">Goal cells; at least one, all inside the grid. A goal the agent cannot stand on seeds nothing.</param>
    /// <param name="options">Options; null means <see cref="GridOptions.Default"/>. Heuristic, algorithm and recording do not apply.</param>
    /// <returns>The field, complete unless <see cref="GridOptions.IterationBudget"/> stopped it.</returns>
    /// <exception cref="ArgumentException">No goals, or the options ask for an agent size other than 1.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A goal is outside the grid.</exception>
    public GridFlowField CreateFlowField(ReadOnlySpan<Cell> goals, GridOptions? options = null) => new(this, goals, options ?? DefaultOptions);

    /// <summary>Computes a flow field toward one goal cell; see <see cref="CreateFlowField(ReadOnlySpan{Cell}, GridOptions?)"/>.</summary>
    /// <param name="goal">The goal cell, inside the grid.</param>
    /// <param name="options">Options; null means <see cref="GridOptions.Default"/>.</param>
    /// <returns>The field.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The goal is outside the grid.</exception>
    public GridFlowField CreateFlowField(Cell goal, GridOptions? options = null) => CreateFlowField([goal], options);

    private void CheckCell(int col, int row)
    {
        if (!Contains(col, row))
        {
            throw new ArgumentOutOfRangeException(col >= 0 && col < Width ? nameof(row) : nameof(col), $"Cell ({col}, {row}) is outside the {Width}x{Height} grid.");
        }
    }
}
