using System;
using System.Threading;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Graphs;

/// <summary>
/// Bidirectional search over a <see cref="CsrGraph"/>: one search forward from the start over the out-edges, one backward
/// from the goal over the in-edges, taking turns. With <see cref="ZeroGraphHeuristic"/> it is bidirectional Dijkstra
/// (Pohl 1969; Nicholson 1966). With a heuristic it is bidirectional A* in the consistent average-potential form of
/// Ikeda et al. 1994 (also Goldberg &amp; Harrelson 2005, §4): with <c>pf(v) = (h(v, goal) - h(start, v)) / 2</c> the forward
/// search orders nodes by <c>g + pf</c> and the backward search by <c>g - pf</c>, which makes both run on the same
/// non-negative reduced edge weights.
/// <para>
/// <b>Stopping rule.</b> Every edge relaxation that meets a node the other side has reached offers a path of cost
/// <c>g_forward + g_backward</c>; the best one found is <c>μ</c>. The search stops when the two open sets' best keys sum to
/// at least <c>μ</c> (no path through unsettled nodes can be cheaper), or when either open set is empty. Stopping on the
/// first meeting would be wrong. The result equals unidirectional A* for a consistent heuristic (up to rounding for
/// fractional weights). See <c>docs/graphs.md</c> for ordering and tie-breaks. Not thread-safe; use one per thread.
/// </para>
/// </summary>
/// <typeparam name="TCost">Edge pricing; a struct so the call inlines.</typeparam>
/// <typeparam name="THeuristic">The estimate, bounding distances in both directions; a struct so the call inlines.</typeparam>
public sealed class BidirectionalGraphSearch<TCost, THeuristic>
    where TCost : struct, IGraphCost
    where THeuristic : struct, IGraphHeuristic
{
    private readonly CsrGraph _graph;
    private readonly TCost _cost;
    private readonly THeuristic _heuristic;
    private readonly GridSearchBuffers _fwd;
    private readonly GridSearchBuffers _bwd;
    private int _start;
    private int _goal;
    private SearchStatus _status;
    private int _iterations;
    private int _expandedCount;
    private bool _forwardTurn;
    private double _mu = double.PositiveInfinity;
    private int _meet = -1;

    /// <summary>Creates a search over <paramref name="graph"/>.</summary>
    /// <param name="graph">The graph.</param>
    /// <param name="cost">Edge pricing.</param>
    /// <param name="heuristic">The estimate (use <see cref="ZeroGraphHeuristic"/> for bidirectional Dijkstra).</param>
    /// <param name="forward">Working memory of the forward side, with capacity for the graph's nodes.</param>
    /// <param name="backward">Working memory of the backward side; a different object from <paramref name="forward"/>.</param>
    /// <exception cref="ArgumentNullException">A reference argument is null.</exception>
    /// <exception cref="ArgumentException">The buffers are too small for the graph or the same object.</exception>
    public BidirectionalGraphSearch(CsrGraph graph, TCost cost, THeuristic heuristic, GridSearchBuffers forward, GridSearchBuffers backward)
    {
        ArgumentNullException.ThrowIfNull(graph);
        ArgumentNullException.ThrowIfNull(forward);
        ArgumentNullException.ThrowIfNull(backward);
        if (ReferenceEquals(forward, backward))
        {
            throw new ArgumentException("The forward and backward searches need separate buffers.", nameof(backward));
        }
        if (forward.NodeCapacity < graph.NodeCount || backward.NodeCapacity < graph.NodeCount)
        {
            throw new ArgumentException($"The search buffers must hold the graph's {graph.NodeCount} nodes.", nameof(forward));
        }
        _graph = graph;
        _cost = cost;
        _heuristic = heuristic;
        _fwd = forward;
        _bwd = backward;
    }

    /// <summary>Where the search stands.</summary>
    public SearchStatus Status => _status;

    /// <summary>Nodes expanded so far by both sides together.</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>Iterations performed so far: one per expansion of either side.</summary>
    public int Iterations => _iterations;

    /// <summary>Path cost once <see cref="Status"/> is <see cref="SearchStatus.Found"/>, otherwise +∞.</summary>
    public double PathCost => _status == SearchStatus.Found ? _mu : double.PositiveInfinity;

    /// <summary>Working memory of the forward side (its <see cref="GridSearchBuffers.ExpandedNodes"/> lists the forward expansions).</summary>
    public GridSearchBuffers ForwardBuffers => _fwd;

    /// <summary>Working memory of the backward side.</summary>
    public GridSearchBuffers BackwardBuffers => _bwd;

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
        _start = start;
        _goal = goal;
        _iterations = 0;
        _expandedCount = 0;
        _forwardTurn = true;
        _mu = double.PositiveInfinity;
        _meet = -1;
        _fwd.BeginSearch(n);
        _bwd.BeginSearch(n);
        _fwd.G[start] = 0.0;
        _fwd.Parent[start] = -1;
        _fwd.Mark[start] = _fwd.Base;
        _bwd.G[goal] = 0.0;
        _bwd.Parent[goal] = -1;
        _bwd.Mark[goal] = _bwd.Base;
        if (start == goal)
        {
            _mu = 0.0;
            _meet = start;
            _status = SearchStatus.Found;
            return;
        }
        if (Potential(start, out double ps))
        {
            _fwd.Open.Push(start, ps, 0.0);
        }
        if (Potential(goal, out double pg))
        {
            _bwd.Open.Push(goal, -pg, 0.0);
        }
        _status = SearchStatus.Running;
    }

    /// <summary>Runs until the search finishes or <paramref name="maxIterations"/> more expansions have been done.</summary>
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
        for (int n = _meet; n >= 0; n = _fwd.Parent[n])
        {
            count++;
        }
        for (int n = _bwd.Parent[_meet]; n >= 0; n = _bwd.Parent[n])
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
        int forwardCount = 0;
        for (int n = _meet; n >= 0; n = _fwd.Parent[n])
        {
            forwardCount++;
        }
        int i = forwardCount;
        for (int n = _meet; n >= 0; n = _fwd.Parent[n])
        {
            destination[--i] = n;
        }
        int j = forwardCount;
        for (int n = _bwd.Parent[_meet]; n >= 0; n = _bwd.Parent[n])
        {
            destination[j++] = n;
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

    // pf(v) = (h(v, goal) - h(start, v)) / 2. False when v cannot lie on any start-goal path.
    private bool Potential(int v, out double p)
    {
        double ht = _heuristic.Estimate(v, _goal);
        double hs = _heuristic.Estimate(_start, v);
        if (double.IsPositiveInfinity(ht) || double.IsPositiveInfinity(hs))
        {
            p = 0.0;
            return false;
        }
        p = 0.5 * (ht - hs);
        return true;
    }

    private SearchStatus Loop(int budget, CancellationToken token)
    {
        if (_status != SearchStatus.Running)
        {
            return _status;
        }
        bool cancellable = token.CanBeCanceled;
        while (true)
        {
            // Stop when no unsettled path can beat mu, or a side ran dry (every node it can reach has been settled).
            if (_fwd.Open.Count == 0 || _bwd.Open.Count == 0 || _fwd.Open.PeekF() + _bwd.Open.PeekF() >= _mu)
            {
                _status = double.IsPositiveInfinity(_mu) ? SearchStatus.NotFound : SearchStatus.Found;
                return _status;
            }
            if (budget == 0)
            {
                return _status;
            }
            if (cancellable && (_iterations & 255) == 0)
            {
                token.ThrowIfCancellationRequested();
            }
            budget--;
            _iterations++;
            bool forward = _forwardTurn;
            _forwardTurn = !_forwardTurn;
            Expand(forward);
        }
    }

    private void Expand(bool forward)
    {
        GridSearchBuffers me = forward ? _fwd : _bwd;
        GridSearchBuffers other = forward ? _bwd : _fwd;
        me.Open.Pop(out int node, out _, out _);
        me.Mark[node] = me.Base + 1;
        _expandedCount++;
        if (me.RecordExpansions)
        {
            me.RecordExpanded(node);
        }
        double gn = me.G[node];
        int[] offsets = forward ? _graph.OffsetArray : _graph.ReverseOffsetArray;
        int[] nodes = forward ? _graph.TargetArray : _graph.SourceArray;
        double[] weights = forward ? _graph.WeightArray : _graph.ReverseWeightArray;
        int end = offsets[node + 1];
        for (int e = offsets[node]; e < end; e++)
        {
            int v = nodes[e];
            // The hook always sees the edge in its true direction.
            double c = forward ? _cost.Cost(node, v, weights[e]) : _cost.Cost(v, node, weights[e]);
            if (!(c >= 0.0) || double.IsPositiveInfinity(c))
            {
                continue;
            }
            double gv = gn + c;
            if (me.Mark[v] >= me.Base && !(gv < me.G[v]))
            {
                continue;
            }
            if (!Potential(v, out double p))
            {
                continue;
            }
            me.G[v] = gv;
            me.Parent[v] = node;
            me.Mark[v] = me.Base;
            me.Open.Push(v, forward ? gv + p : gv - p, gv);
            if (other.Mark[v] >= other.Base)
            {
                double through = gv + other.G[v];
                if (through < _mu)
                {
                    _mu = through;
                    _meet = v;
                }
            }
        }
    }
}
