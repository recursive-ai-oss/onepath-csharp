using System;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Jump point search over an eight-connected square grid without corner cutting: A* whose successors are jump points
/// (Harabor &amp; Grastien 2011). With <see cref="OnlineJumper"/> it is JPS, with <see cref="TableJumper"/> JPS+.
/// <para>
/// All areas must cost the same (<c>unitCost</c> per straight step, <c>unitCost * √2</c> per diagonal step) and the heuristic must be
/// admissible and consistent for that cost (<see cref="OctileHeuristic"/> scaled by <c>unitCost</c>, or <see cref="ZeroHeuristic"/>); the path is then optimal. The open
/// list is the grid core's <see cref="OpenSet"/> (smaller f, then larger g, then smaller node index) and a cheaper route reopens a node, exactly as in
/// <see cref="AStar{TTopology, TCost, THeuristic}"/>; successors are generated in direction order 0..7.
/// An iteration pops one node and expands it, so a search sliced with <see cref="Step"/> returns what an unsliced one does.
/// </para>
/// <para>Not thread-safe. No allocation during a search once the buffers exist.</para>
/// </summary>
/// <typeparam name="TJumper">How jump points are found.</typeparam>
/// <typeparam name="THeuristic">The distance estimate.</typeparam>
public sealed partial class JumpSearch<TJumper, THeuristic>
    where TJumper : struct, IJumper
    where THeuristic : struct, IHeuristic
{
    private readonly TJumper _jumper;
    private THeuristic _heuristic;
    private readonly GridSearchBuffers _buffers;
    private readonly double _unit;
    private readonly double _diagonalUnit;
    private readonly int _width;
    private int _goal = -1;
    private int _expandedCount;
    private int _iterations;
    private double _pathCost;
    private SearchStatus _status;

    /// <summary>Creates a search object.</summary>
    /// <param name="jumper">The jump point finder (reads the grid, or the JPS+ tables).</param>
    /// <param name="heuristic">The estimate; admissible for <paramref name="unitCost"/>.</param>
    /// <param name="unitCost">The cost of a straight step (the common multiplier of every area); positive and finite.</param>
    /// <param name="buffers">Working memory to reuse, or null to allocate a private set.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="unitCost"/> is not positive and finite.</exception>
    /// <exception cref="ArgumentException">The buffers are smaller than the grid.</exception>
    public JumpSearch(TJumper jumper, THeuristic heuristic, double unitCost = 1.0, GridSearchBuffers? buffers = null)
    {
        if (!(unitCost > 0.0) || double.IsPositiveInfinity(unitCost))
        {
            throw new ArgumentOutOfRangeException(nameof(unitCost), unitCost, "The step cost must be positive and finite.");
        }
        _jumper = jumper;
        _heuristic = heuristic;
        _unit = unitCost;
        _diagonalUnit = GridMath.Sqrt2 * unitCost;
        _width = jumper.Width;
        int n = jumper.Width * jumper.Height;
        _buffers = buffers ?? new GridSearchBuffers(n);
        if (_buffers.NodeCapacity < n)
        {
            throw new ArgumentException($"The search buffers hold {_buffers.NodeCapacity} nodes but the grid has {n}.", nameof(buffers));
        }
    }

    /// <summary>The state of the search.</summary>
    public SearchStatus Status => _status;

    /// <summary>Nodes expanded so far in this search.</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>Iterations performed so far.</summary>
    public int Iterations => _iterations;

    /// <summary>Path cost once <see cref="Status"/> is <see cref="SearchStatus.Found"/>.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathCost : double.PositiveInfinity;

    /// <summary>The working memory.</summary>
    public GridSearchBuffers Buffers => _buffers;

    /// <summary>Starts a new search, discarding any previous one.</summary>
    /// <param name="start">Start cell index (row-major).</param>
    /// <param name="goal">Goal cell index.</param>
    /// <exception cref="ArgumentOutOfRangeException">A cell is outside the grid.</exception>
    /// <exception cref="ArgumentException">A cell is a wall.</exception>
    public void Begin(int start, int goal)
    {
        int n = _width * _jumper.Height;
        if ((uint)start >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(start), start, "Start cell is outside the grid.");
        }
        if ((uint)goal >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(goal), goal, "Goal cell is outside the grid.");
        }
        if (!_jumper.IsFree(start % _width, start / _width))
        {
            throw new ArgumentException("The start cell is not walkable.", nameof(start));
        }
        if (!_jumper.IsFree(goal % _width, goal / _width))
        {
            throw new ArgumentException("The goal cell is not walkable.", nameof(goal));
        }
        _goal = goal;
        _expandedCount = 0;
        _iterations = 0;
        _pathCost = double.PositiveInfinity;
        _jumper.BeginSearch();
        _buffers.BeginSearch(n);
        _heuristic.SetGoal(goal);
        _buffers.G[start] = 0.0;
        _buffers.Parent[start] = -1;
        _buffers.Mark[start] = _buffers.Base;
        _buffers.Open.Push(start, _heuristic.Estimate(start), 0.0);
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

    /// <summary>Number of cells on the found path, start and goal included (every cell, not only the jump points).</summary>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    public int PathLength()
    {
        RequireFound();
        int count = 1;
        for (int n = _goal; _buffers.Parent[n] >= 0; n = _buffers.Parent[n])
        {
            count += Steps(_buffers.Parent[n], n);
        }
        return count;
    }

    /// <summary>Writes the found path as consecutive cells, start first, into <paramref name="destination"/>.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> elements.</param>
    /// <returns>The number of cells written.</returns>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int CopyPath(Span<int> destination)
    {
        int length = PathLength();
        if (destination.Length < length)
        {
            throw new ArgumentException($"The path has {length} cells but the destination holds {destination.Length}.", nameof(destination));
        }
        int i = length;
        int to = _goal;
        destination[--i] = to;
        while (_buffers.Parent[to] >= 0)
        {
            int from = _buffers.Parent[to];
            int steps = Steps(from, to);
            int stride = Stride(from, to);
            for (int k = 1; k < steps; k++)
            {
                destination[--i] = to - (k * stride);
            }
            destination[--i] = from;
            to = from;
        }
        return length;
    }

    /// <summary>Writes only the jump points of the found path (start, turning points, goal), start first.</summary>
    /// <param name="destination">Large enough for the jump points (at most <see cref="PathLength"/>).</param>
    /// <returns>The number of cells written.</returns>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int CopyJumpPoints(Span<int> destination)
    {
        RequireFound();
        int count = 0;
        for (int n = _goal; n >= 0; n = _buffers.Parent[n])
        {
            count++;
        }
        if (destination.Length < count)
        {
            throw new ArgumentException($"The path has {count} jump points but the destination holds {destination.Length}.", nameof(destination));
        }
        int i = count;
        for (int n = _goal; n >= 0; n = _buffers.Parent[n])
        {
            destination[--i] = n;
        }
        return count;
    }

    private int Steps(int from, int to)
    {
        int fy = from / _width;
        int ty = to / _width;
        int dx = Math.Abs((to - (ty * _width)) - (from - (fy * _width)));
        int dy = Math.Abs(ty - fy);
        return dx > dy ? dx : dy;
    }

    private int Stride(int from, int to)
    {
        int fy = from / _width;
        int ty = to / _width;
        int sx = Math.Sign((to - (ty * _width)) - (from - (fy * _width)));
        return sx + (Math.Sign(ty - fy) * _width);
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
        int w = _width;
        int goal = _goal;
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
            b.Mark[node] = b.Base + 1;
            if (node == goal)
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
            double gNode = b.G[node];
            int y = node / w;
            int x = node - (y * w);
            int parent = b.Parent[node];
            int mask;
            if (parent < 0)
            {
                mask = 0xFF;
            }
            else
            {
                int py = parent / w;
                mask = JumpRules.Successors(_jumper, x, y, Math.Sign(x - (parent - (py * w))), Math.Sign(y - py));
            }
            for (int dir = 0; dir < 8; dir++)
            {
                if ((mask & (1 << dir)) == 0)
                {
                    continue;
                }
                int target = _jumper.Jump(node, dir, goal, out int steps);
                if (target < 0)
                {
                    continue;
                }
                Relax(node, gNode + (steps * (dir < 4 ? _unit : _diagonalUnit)), target);
            }
        }
        return _status;
    }

    // Take the jump point if this route is strictly cheaper; ties keep the first parent found (the earlier node pop, then the earlier direction).
    private void Relax(int from, double g, int to)
    {
        GridSearchBuffers b = _buffers;
        if (b.Mark[to] < b.Base)
        {
            b.Mark[to] = b.Base;
        }
        else if (!(g < b.G[to]))
        {
            return;
        }
        else if (b.Mark[to] != b.Base)
        {
            b.Mark[to] = b.Base; // reopen a closed node
        }
        b.G[to] = g;
        b.Parent[to] = from;
        b.Open.Push(to, g + _heuristic.Estimate(to), g);
    }
}
