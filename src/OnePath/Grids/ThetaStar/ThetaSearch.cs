using System;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>Which any-angle algorithm a <see cref="ThetaSearch{THeuristic}"/> runs.</summary>
public enum ThetaVariant
{
    /// <summary>Theta* (Nash, Daniel, Koenig &amp; Felner 2007): a line-of-sight test for every neighbour generated.</summary>
    Theta = 0,

    /// <summary>
    /// Lazy Theta* (Nash, Koenig &amp; Tovey 2010): assumes the line of sight when generating a neighbour and tests it once,
    /// when the node is expanded, repairing the parent from the closed neighbours if the test fails.
    /// </summary>
    Lazy = 1,

    /// <summary>
    /// Lazy Theta* with one change to the paper: when the line-of-sight test fails and the parent is repaired, the node goes
    /// back into the open list under its corrected cost instead of being expanded at once under its optimistic one (the
    /// repaired pop still counts as an iteration). The paper's variant expands such nodes too early and in rare cases returns a
    /// path longer than the cell-by-cell A* path (about 3 queries in 10,000 on random grids, by up to 13%); this one did not on
    /// the same 10^4-grid checks, and costs a few more iterations. This is what the one-line API's
    /// <see cref="GridAlgorithm.LazyThetaStar"/> runs.
    /// </summary>
    LazyRequeue = 2,
}

/// <summary>
/// Any-angle search on an eight-connected square grid without corner cutting, agent size 1: Theta* or Lazy Theta*.
/// Nodes are cells; the path is a list of waypoints (cell centres) joined by straight segments that each pass the
/// exact test of <see cref="GridLineOfSight"/>. It runs over the legal grid moves, so it finds a path exactly when A* does.
/// <para>
/// Costs are <c>unitCost * sqrt(dx² + dy²)</c> per segment, added in path order from 0. Theta* and Lazy Theta* give short,
/// not always shortest, any-angle paths (the papers give no optimality guarantee). Use <see cref="EuclideanHeuristic"/> (admissible for any-angle movement) or <see cref="ZeroHeuristic"/>.
/// </para>
/// <para>
/// Open list: the grid core's <see cref="OpenSet"/> (smaller f, then larger g, then smaller cell index). Neighbours are generated in
/// the order north, east, south, west, north-east, south-east, south-west, north-west. A closed node is never reopened
/// (as in the papers); a neighbour is updated only when the new cost is strictly lower, so ties keep the first parent.
/// An iteration pops one node and expands it, so a sliced search returns what an unsliced one does.
/// </para>
/// <para>Not thread-safe; the grid must not be edited during a search. No allocation during a search once the buffers exist.</para>
/// </summary>
/// <typeparam name="THeuristic">The distance estimate, admissible for straight-line movement at <c>unitCost</c> per cell.</typeparam>
public sealed class ThetaSearch<THeuristic>
    where THeuristic : struct, IHeuristic
{
    private static readonly int[] Dx = [0, 1, 0, -1, 1, 1, -1, -1];
    private static readonly int[] Dy = [-1, 0, 1, 0, -1, 1, 1, -1];

    private readonly Grid _grid;
    private readonly ThetaVariant _variant;
    private readonly double _unit;
    private readonly double _diagonalUnit;
    private readonly int _width;
    private readonly int _height;
    private THeuristic _heuristic;
    private readonly GridSearchBuffers _buffers;
    private double _weight = 1.0;
    private int _goal = -1;
    private int _expandedCount;
    private int _iterations;
    private double _pathCost;
    private SearchStatus _status;

    /// <summary>Creates a search object.</summary>
    /// <param name="grid">The grid; read directly, so it must not change while a search runs.</param>
    /// <param name="heuristic">The estimate.</param>
    /// <param name="variant">Theta* or Lazy Theta*.</param>
    /// <param name="unitCost">The cost of one cell of straight movement (the common multiplier of every area); positive and finite.</param>
    /// <param name="buffers">Working memory to reuse, or null to allocate a private set.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="unitCost"/> is not positive and finite, or <paramref name="variant"/> is undefined.</exception>
    /// <exception cref="ArgumentException">The buffers are smaller than the grid.</exception>
    public ThetaSearch(Grid grid, THeuristic heuristic, ThetaVariant variant = ThetaVariant.Theta, double unitCost = 1.0, GridSearchBuffers? buffers = null)
    {
        ArgumentNullException.ThrowIfNull(grid);
        if (!(unitCost > 0.0) || double.IsPositiveInfinity(unitCost))
        {
            throw new ArgumentOutOfRangeException(nameof(unitCost), unitCost, "The step cost must be positive and finite.");
        }
        if (variant is not (ThetaVariant.Theta or ThetaVariant.Lazy or ThetaVariant.LazyRequeue))
        {
            throw new ArgumentOutOfRangeException(nameof(variant), variant, "Unknown any-angle variant.");
        }
        _grid = grid;
        _heuristic = heuristic;
        _variant = variant;
        _unit = unitCost;
        _diagonalUnit = GridMath.Sqrt2 * unitCost;
        _width = grid.Width;
        _height = grid.Height;
        _buffers = buffers ?? new GridSearchBuffers(grid.CellCount);
        if (_buffers.NodeCapacity < grid.CellCount)
        {
            throw new ArgumentException($"The search buffers hold {_buffers.NodeCapacity} nodes but the grid has {grid.CellCount}.", nameof(buffers));
        }
    }

    /// <summary>The state of the search.</summary>
    public SearchStatus Status => _status;

    /// <summary>Nodes expanded so far in this search.</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>Iterations performed so far.</summary>
    public int Iterations => _iterations;

    /// <summary>Path length once <see cref="Status"/> is <see cref="SearchStatus.Found"/>.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathCost : double.PositiveInfinity;

    /// <summary>The working memory.</summary>
    public GridSearchBuffers Buffers => _buffers;

    /// <summary>Multiplier on the estimate, 1 by default; above 1 the search is faster and paths may be longer. Finite and not negative.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is negative, infinite or NaN.</exception>
    public double HeuristicWeight
    {
        get => _weight;
        set
        {
            if (!(value >= 0.0) || double.IsPositiveInfinity(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "The heuristic weight must be finite and not negative.");
            }
            _weight = value;
        }
    }

    /// <summary>Starts a new search, discarding any previous one.</summary>
    /// <param name="start">Start cell index (row-major).</param>
    /// <param name="goal">Goal cell index.</param>
    /// <exception cref="ArgumentOutOfRangeException">A cell is outside the grid.</exception>
    /// <exception cref="ArgumentException">A cell is a wall.</exception>
    public void Begin(int start, int goal)
    {
        int n = _grid.CellCount;
        if ((uint)start >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(start), start, "Start cell is outside the grid.");
        }
        if ((uint)goal >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(goal), goal, "Goal cell is outside the grid.");
        }
        ReadOnlySpan<byte> cells = _grid.Cells;
        if (cells[start] == AreaCosts.Wall)
        {
            throw new ArgumentException("The start cell is not walkable.", nameof(start));
        }
        if (cells[goal] == AreaCosts.Wall)
        {
            throw new ArgumentException("The goal cell is not walkable.", nameof(goal));
        }
        _goal = goal;
        _expandedCount = 0;
        _iterations = 0;
        _pathCost = double.PositiveInfinity;
        _buffers.BeginSearch(n);
        _heuristic.SetGoal(goal);
        _buffers.G[start] = 0.0;
        _buffers.Parent[start] = -1;
        _buffers.Mark[start] = _buffers.Base;
        _buffers.Open.Push(start, _weight * _heuristic.Estimate(start), 0.0);
        _status = SearchStatus.Running;
    }

    /// <summary>Runs until the search finishes or <paramref name="maxIterations"/> more iterations have been done.</summary>
    /// <param name="maxIterations">Iteration budget for this call; at least 1.</param>
    /// <returns>The status afterwards.</returns>
    /// <exception cref="InvalidOperationException"><see cref="Begin"/> has not been called.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxIterations"/> is below 1.</exception>
    public SearchStatus Step(int maxIterations)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(maxIterations, 1);
        if (_status == SearchStatus.NotStarted)
        {
            throw new InvalidOperationException("Call Begin before Step.");
        }
        return Loop(maxIterations, default);
    }

    /// <summary>Runs the search to completion.</summary>
    /// <param name="cancellationToken">Checked every 256 iterations; cancelling throws and leaves the search resumable.</param>
    /// <returns><see cref="SearchStatus.Found"/> or <see cref="SearchStatus.NotFound"/>.</returns>
    /// <exception cref="InvalidOperationException"><see cref="Begin"/> has not been called.</exception>
    /// <exception cref="OperationCanceledException">Cancellation was requested.</exception>
    public SearchStatus Run(CancellationToken cancellationToken = default)
    {
        if (_status == SearchStatus.NotStarted)
        {
            throw new InvalidOperationException("Call Begin before Run.");
        }
        return Loop(int.MaxValue, cancellationToken);
    }

    /// <summary>Number of waypoints on the found path, start and goal included.</summary>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    public int PathLength()
    {
        RequireFound();
        int count = 0;
        for (int n = _goal; n >= 0; n = _buffers.Parent[n])
        {
            count++;
        }
        return count;
    }

    /// <summary>Writes the waypoints of the found path, start first, into <paramref name="destination"/>.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> elements.</param>
    /// <returns>The number of waypoints written.</returns>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int CopyPath(Span<int> destination)
    {
        int length = PathLength();
        if (destination.Length < length)
        {
            throw new ArgumentException($"The path has {length} waypoints but the destination holds {destination.Length}.", nameof(destination));
        }
        int i = length;
        for (int n = _goal; n >= 0; n = _buffers.Parent[n])
        {
            destination[--i] = n;
        }
        return length;
    }

    private void RequireFound()
    {
        if (_status != SearchStatus.Found)
        {
            throw new InvalidOperationException("The search has not found a path.");
        }
    }

    private SearchStatus Loop(int budget, CancellationToken token)
    {
        if (_status != SearchStatus.Running)
        {
            return _status;
        }
        GridSearchBuffers b = _buffers;
        OpenSet open = b.Open;
        bool record = b.RecordExpansions;
        bool cancellable = token.CanBeCanceled;
        bool lazy = _variant != ThetaVariant.Theta;
        bool requeue = _variant == ThetaVariant.LazyRequeue;
        while (budget > 0)
        {
            if (cancellable && (_iterations & 255) == 0)
            {
                token.ThrowIfCancellationRequested();
            }
            if (!open.Pop(out int node, out _, out _))
            {
                _status = SearchStatus.NotFound;
                return _status;
            }
            budget--;
            _iterations++;
            if (lazy && SetVertex(node) && requeue)
            {
                b.Open.Push(node, b.G[node] + (_weight * _heuristic.Estimate(node)), b.G[node]);
                continue;
            }
            b.Mark[node] = b.Base + 1;
            if (node == _goal)
            {
                _pathCost = b.G[node];
                _status = SearchStatus.Found;
                return _status;
            }
            _expandedCount++;
            if (record)
            {
                b.RecordExpanded(node);
            }
            Expand(node, lazy);
        }
        return _status;
    }

    // Nash, Koenig & Tovey 2010, Alg. 2 (SetVertex): the line of sight assumed when the node was generated is checked now;
    // if it fails, the parent becomes the closed grid neighbour giving the cheapest path (first in neighbour order on ties).
    private bool SetVertex(int s)
    {
        GridSearchBuffers b = _buffers;
        int p = b.Parent[s];
        int sy = s / _width;
        int sx = s - (sy * _width);
        if (p < 0 || Visible(p, sx, sy))
        {
            return false;
        }
        double best = double.PositiveInfinity;
        int bestNode = -1;
        for (int d = 0; d < 8; d++)
        {
            if (!TryMove(sx, sy, d, out int t) || b.Mark[t] != b.Base + 1)
            {
                continue;
            }
            double c = b.G[t] + StepCost(d);
            if (c < best)
            {
                best = c;
                bestNode = t;
            }
        }
        if (bestNode >= 0)
        {
            b.Parent[s] = bestNode;
            b.G[s] = best;
            return true;
        }
        return false;
    }

    private void Expand(int s, bool lazy)
    {
        GridSearchBuffers b = _buffers;
        int sy = s / _width;
        int sx = s - (sy * _width);
        int p = b.Parent[s];
        for (int d = 0; d < 8; d++)
        {
            if (!TryMove(sx, sy, d, out int t) || b.Mark[t] == b.Base + 1)
            {
                continue;
            }
            int parent;
            double g;
            if (p < 0)
            {
                parent = s;
                g = b.G[s] + StepCost(d);
            }
            else if (lazy || Visible(p, t % _width, t / _width))
            {
                // Path 2 of the papers: straight from the parent (Theta* tests it now, Lazy Theta* assumes it).
                parent = p;
                g = b.G[p] + SegmentCost(p, t);
            }
            else
            {
                // Path 1: the grid step from the expanded node.
                parent = s;
                g = b.G[s] + StepCost(d);
            }
            if (b.Mark[t] < b.Base)
            {
                b.Mark[t] = b.Base;
            }
            else if (!(g < b.G[t]))
            {
                continue;
            }
            b.G[t] = g;
            b.Parent[t] = parent;
            b.Open.Push(t, g + (_weight * _heuristic.Estimate(t)), g);
        }
    }

    private bool Visible(int from, int toX, int toY)
    {
        int fy = from / _width;
        return GridLineOfSight.Visible(_grid.Cells, _width, from - (fy * _width), fy, toX, toY);
    }

    // The neighbour of (x, y) in direction d if the move is legal (no corner cutting), else false.
    private bool TryMove(int x, int y, int d, out int target)
    {
        int nx = x + Dx[d];
        int ny = y + Dy[d];
        target = (ny * _width) + nx;
        if ((uint)nx >= (uint)_width || (uint)ny >= (uint)_height)
        {
            return false;
        }
        ReadOnlySpan<byte> cells = _grid.Cells;
        if (cells[target] == AreaCosts.Wall)
        {
            return false;
        }
        return d < 4 || (cells[(y * _width) + nx] != AreaCosts.Wall && cells[(ny * _width) + x] != AreaCosts.Wall);
    }

    private double StepCost(int d) => d < 4 ? _unit : _diagonalUnit;

    private double SegmentCost(int a, int b)
    {
        int ay = a / _width;
        int by = b / _width;
        long dx = (b - (by * _width)) - (a - (ay * _width));
        long dy = by - ay;
        return _unit * Math.Sqrt((double)((dx * dx) + (dy * dy)));
    }
}
