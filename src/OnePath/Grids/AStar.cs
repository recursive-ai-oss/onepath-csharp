using System;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// A* search (Hart, Nilsson &amp; Raphael 1968) over any <see cref="IGridTopology"/>, with struct-generic cost and
/// heuristic hooks so the loop inlines them. With <see cref="ZeroHeuristic"/> it is Dijkstra's algorithm.
/// <para>
/// The search is resumable: <see cref="Step"/> performs at most the given number of iterations (one iteration
/// pops one node and expands it; never time), and <see cref="Run"/> is <c>Step(int.MaxValue)</c>, so a sliced search
/// returns exactly what an unsliced one does. Nodes pop in <see cref="OpenSet"/> order. A node whose cost improves
/// after it was closed is reopened, so results stay correct with an inconsistent heuristic. The goal test happens
/// when the goal is popped. No allocation happens during a search once the buffers exist.
/// </para>
/// <para>Not thread-safe. The topology must not change while a search is running.</para>
/// </summary>
/// <typeparam name="TTopology">The graph.</typeparam>
/// <typeparam name="TCost">The cost hook.</typeparam>
/// <typeparam name="THeuristic">The distance estimate.</typeparam>
public sealed class AStar<TTopology, TCost, THeuristic>
    where TTopology : struct, IGridTopology
    where TCost : struct, IStepCost
    where THeuristic : struct, IHeuristic
{
    private readonly TTopology _topology;
    private readonly TCost _cost;
    private THeuristic _heuristic;
    private readonly GridSearchBuffers _buffers;
    private double _weight = 1.0;
    private int _start = -1;
    private int _goal = -1;
    private int _expandedCount;
    private int _iterations;
    private double _pathCost;
    private SearchStatus _status;

    /// <summary>Creates a search object.</summary>
    /// <param name="topology">The graph.</param>
    /// <param name="cost">The cost hook (use <see cref="DefaultStepCost"/> for none).</param>
    /// <param name="heuristic">The estimate.</param>
    /// <param name="buffers">Working memory to reuse, or null to allocate a private set sized for the topology.</param>
    /// <exception cref="ArgumentException">The buffers are smaller than the topology.</exception>
    public AStar(TTopology topology, TCost cost, THeuristic heuristic, GridSearchBuffers? buffers = null)
    {
        _topology = topology;
        _cost = cost;
        _heuristic = heuristic;
        _buffers = buffers ?? new GridSearchBuffers(topology.NodeCount);
        if (_buffers.NodeCapacity < topology.NodeCount)
        {
            throw new ArgumentException($"The search buffers hold {_buffers.NodeCapacity} nodes but the graph has {topology.NodeCount}.", nameof(buffers));
        }
    }

    /// <summary>The state of the search.</summary>
    public SearchStatus Status => _status;

    /// <summary>Nodes expanded so far in this search.</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>Iterations performed so far in this search (equal to <see cref="ExpandedCount"/> plus one once the goal was popped).</summary>
    public int Iterations => _iterations;

    /// <summary>Path cost once <see cref="Status"/> is <see cref="SearchStatus.Found"/>.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathCost : double.PositiveInfinity;

    /// <summary>The working memory (for debug data such as <see cref="GridSearchBuffers.ExpandedNodes"/>).</summary>
    public GridSearchBuffers Buffers => _buffers;

    /// <summary>
    /// Multiplier on the estimate, 1 by default. Above 1 the search is faster and no longer optimal (weighted A*);
    /// below 1 it approaches Dijkstra. Must be finite and not negative.
    /// </summary>
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
    /// <param name="start">Start node.</param>
    /// <param name="goal">Goal node.</param>
    /// <exception cref="ArgumentOutOfRangeException">A node is outside the graph.</exception>
    /// <exception cref="ArgumentException">A node is not walkable.</exception>
    public void Begin(int start, int goal)
    {
        int n = _topology.NodeCount;
        if ((uint)start >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(start), start, "Start node is outside the graph.");
        }
        if ((uint)goal >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(goal), goal, "Goal node is outside the graph.");
        }
        if (!_topology.IsWalkable(start))
        {
            throw new ArgumentException("The start node is not walkable.", nameof(start));
        }
        if (!_topology.IsWalkable(goal))
        {
            throw new ArgumentException("The goal node is not walkable.", nameof(goal));
        }
        _start = start;
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

    /// <summary>Number of nodes on the found path, start and goal included.</summary>
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

    /// <summary>Writes the found path, start first, into <paramref name="destination"/>.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> elements.</param>
    /// <returns>The number of nodes written.</returns>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int CopyPath(Span<int> destination)
    {
        int length = PathLength();
        if (destination.Length < length)
        {
            throw new ArgumentException($"The path has {length} nodes but the destination holds {destination.Length}.", nameof(destination));
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
        var relaxer = new Relaxer { Owner = this };
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
            relaxer.From = node;
            relaxer.GFrom = b.G[node];
            _topology.VisitNeighbors(node, ref relaxer);
        }
        return _status;
    }

    // Relax the edge from the node being expanded: take the neighbour if this route is strictly cheaper
    // (ties keep the first parent found, which is the earlier neighbour in the topology's order).
    private void Relax(int from, double gFrom, int to, double baseCost)
    {
        double step = _cost.Cost(from, to, baseCost);
        if (!(step >= 0.0) || double.IsPositiveInfinity(step))
        {
            return;
        }
        GridSearchBuffers b = _buffers;
        double g = gFrom + step;
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
        b.Open.Push(to, g + (_weight * _heuristic.Estimate(to)), g);
    }

    private struct Relaxer : INeighborVisitor
    {
        public AStar<TTopology, TCost, THeuristic> Owner;
        public int From;
        public double GFrom;

        public readonly void Visit(int neighbor, double baseCost) => Owner.Relax(From, GFrom, neighbor, baseCost);
    }
}
