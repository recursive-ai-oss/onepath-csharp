using System;
using System.Threading;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Graphs;

/// <summary>
/// A* over a <see cref="CsrGraph"/> (Hart, Nilsson &amp; Raphael 1968); with <see cref="ZeroGraphHeuristic"/> it is Dijkstra's algorithm.
/// Same loop, tie-breaks and iteration counting as the grid search (see <c>docs/graphs.md</c>): the open set is the grid
/// core's <see cref="OpenSet"/> (smaller f, then larger g, then smaller node id), an iteration is one pop, a node whose
/// cost improves is reopened, and equal-cost alternatives keep the first parent found. Not thread-safe; use one per thread.
/// </summary>
/// <typeparam name="TCost">Edge pricing; a struct so the call inlines.</typeparam>
/// <typeparam name="THeuristic">The estimate; a struct so the call inlines.</typeparam>
public sealed class GraphAStar<TCost, THeuristic>
    where TCost : struct, IGraphCost
    where THeuristic : struct, IGraphHeuristic
{
    private readonly CsrGraph _graph;
    private readonly TCost _cost;
    private readonly THeuristic _heuristic;
    private readonly GridSearchBuffers _buffers;
    private int _goal;
    private SearchStatus _status;
    private int _iterations;
    private int _expandedCount;
    private double _pathCost = double.PositiveInfinity;

    /// <summary>Creates a search over <paramref name="graph"/>.</summary>
    /// <param name="graph">The graph.</param>
    /// <param name="cost">Edge pricing.</param>
    /// <param name="heuristic">The estimate to the goal.</param>
    /// <param name="buffers">Working memory with capacity for the graph's nodes; reuse it across searches.</param>
    /// <exception cref="ArgumentNullException">A reference argument is null.</exception>
    /// <exception cref="ArgumentException">The buffers are too small for the graph.</exception>
    public GraphAStar(CsrGraph graph, TCost cost, THeuristic heuristic, GridSearchBuffers buffers)
    {
        ArgumentNullException.ThrowIfNull(graph);
        ArgumentNullException.ThrowIfNull(buffers);
        if (buffers.NodeCapacity < graph.NodeCount)
        {
            throw new ArgumentException($"The search buffers hold {buffers.NodeCapacity} nodes but the graph has {graph.NodeCount}.", nameof(buffers));
        }
        _graph = graph;
        _cost = cost;
        _heuristic = heuristic;
        _buffers = buffers;
    }

    /// <summary>Where the search stands.</summary>
    public SearchStatus Status => _status;

    /// <summary>Nodes expanded so far (the goal's pop is not an expansion).</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>Iterations (pops) performed so far.</summary>
    public int Iterations => _iterations;

    /// <summary>Path cost once <see cref="Status"/> is <see cref="SearchStatus.Found"/>, otherwise +∞.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathCost : double.PositiveInfinity;

    /// <summary>The working memory.</summary>
    public GridSearchBuffers Buffers => _buffers;

    /// <summary>Starts a new search, discarding any previous one.</summary>
    /// <param name="start">Start node.</param>
    /// <param name="goal">Goal node.</param>
    /// <exception cref="ArgumentOutOfRangeException">A node is outside the graph.</exception>
    public void Begin(int start, int goal)
    {
        int n = _graph.NodeCount;
        if ((uint)start >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(start), start, "Start node is outside the graph.");
        }
        if ((uint)goal >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(goal), goal, "Goal node is outside the graph.");
        }
        _goal = goal;
        _iterations = 0;
        _expandedCount = 0;
        _pathCost = double.PositiveInfinity;
        _buffers.BeginSearch(n);
        _buffers.G[start] = 0.0;
        _buffers.Parent[start] = -1;
        _buffers.Mark[start] = _buffers.Base;
        double h = _heuristic.Estimate(start, goal);
        if (!double.IsPositiveInfinity(h))
        {
            _buffers.Open.Push(start, h, 0.0);
        }
        _status = SearchStatus.Running;
    }

    /// <summary>Runs until the search finishes or <paramref name="maxIterations"/> more iterations have been done.</summary>
    /// <param name="maxIterations">Iteration budget for this call; at least 1.</param>
    /// <returns>The status afterwards. Any sequence of budgets gives the same result as one unsliced run.</returns>
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
        double[] g = b.G;
        int[] parent = b.Parent;
        uint[] mark = b.Mark;
        uint reached = b.Base;
        bool record = b.RecordExpansions;
        bool cancellable = token.CanBeCanceled;
        int[] offsets = _graph.OffsetArray;
        int[] targets = _graph.TargetArray;
        double[] weights = _graph.WeightArray;
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
            mark[node] = reached + 1;
            if (node == _goal)
            {
                _pathCost = g[node];
                _status = SearchStatus.Found;
                return _status;
            }
            _expandedCount++;
            if (record)
            {
                b.RecordExpanded(node);
            }
            double gn = g[node];
            int end = offsets[node + 1];
            for (int e = offsets[node]; e < end; e++)
            {
                int v = targets[e];
                double c = _cost.Cost(node, v, weights[e]);
                // Forbidden edges: negative, NaN or infinite cost.
                if (!(c >= 0.0) || double.IsPositiveInfinity(c))
                {
                    continue;
                }
                double gv = gn + c;
                if (mark[v] < reached || gv < g[v])
                {
                    double h = _heuristic.Estimate(v, _goal);
                    if (double.IsPositiveInfinity(h))
                    {
                        continue; // the goal cannot be reached from v
                    }
                    g[v] = gv;
                    parent[v] = node;
                    mark[v] = reached;
                    open.Push(v, gv + h, gv);
                }
            }
        }
        return _status;
    }
}
