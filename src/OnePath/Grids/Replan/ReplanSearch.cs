using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>Which incremental search a <see cref="ReplanSearch"/> runs.</summary>
public enum ReplanMode
{
    /// <summary>
    /// Lifelong Planning A* (Koenig, Likhachev &amp; Furcy 2004): searches forward from a fixed start; the goal is the
    /// query. Cell edits are repaired without searching again from scratch.
    /// </summary>
    LpaStar = 0,

    /// <summary>
    /// D* Lite (Koenig &amp; Likhachev 2002, the version with the key modifier): searches backward from the goal so
    /// the start can move along the path while the map changes; moving the start never invalidates earlier work.
    /// </summary>
    DStarLite = 1,
}

/// <summary>
/// The non-generic surface of an incremental grid search (tier 3). Create one with
/// <see cref="ReplanSearch{TTopology, TCost, THeuristic}"/>, tell it which nodes' moves changed
/// (<see cref="MarkChanged"/>), then call <see cref="Replan"/>.
/// </summary>
public abstract class ReplanSearch
{
    /// <summary>The algorithm.</summary>
    public abstract ReplanMode Mode { get; }

    /// <summary>The start node (for <see cref="ReplanMode.DStarLite"/>, the agent's current position).</summary>
    public abstract int Start { get; }

    /// <summary>The goal node.</summary>
    public abstract int Goal { get; }

    /// <summary>Iterations (queue pops) made since the search was created or last restarted; the unit of the budget.</summary>
    public abstract long Iterations { get; }

    /// <summary>Nodes expanded since creation or the last restart: pops that changed a node's cost-to-come (re-queues of stale keys excluded).</summary>
    public abstract long ExpandedCount { get; }

    /// <summary>Nodes expanded by the most recent <see cref="Replan"/> call (all calls of one sliced replan add up).</summary>
    public abstract int LastExpanded { get; }

    /// <summary>When true, each <see cref="Replan"/> records the nodes it settled or reopened, in order, in <see cref="LastExpandedNodes"/>. Off by default.</summary>
    public abstract bool RecordExpansions { get; set; }

    /// <summary>Debug data: the nodes the last <see cref="Replan"/> expanded, in order; empty unless <see cref="RecordExpansions"/> was set before it.</summary>
    public abstract ReadOnlySpan<int> LastExpandedNodes { get; }

    internal abstract void WriteState(StateWriter w);

    internal abstract void ReadState(ref StateReader r);

    /// <summary>
    /// Records that the moves into and out of <paramref name="node"/> changed (it became a wall, changed area, was blocked...).
    /// With <paramref name="withNeighbors"/> the node's neighbours are re-examined too (needed when only the node itself is known to
    /// have changed, since its neighbours' moves into it changed with it). Nothing is recomputed until <see cref="Replan"/>.
    /// </summary>
    /// <param name="node">Node index.</param>
    /// <param name="withNeighbors">True to re-examine the neighbours as well.</param>
    /// <exception cref="ArgumentOutOfRangeException">The node is not in the graph.</exception>
    public abstract void MarkChanged(int node, bool withNeighbors);

    /// <summary>Forgets everything learned and searches from scratch on the next <see cref="Replan"/> (what a change of cost table needs).</summary>
    public abstract void Restart();

    /// <summary>Moves the start. D* Lite keeps all earlier work; LPA* restarts.</summary>
    /// <param name="node">The new start node.</param>
    /// <exception cref="ArgumentOutOfRangeException">The node is not in the graph.</exception>
    public abstract void MoveStart(int node);

    /// <summary>Changes the goal; the search restarts.</summary>
    /// <param name="node">The new goal node.</param>
    /// <exception cref="ArgumentOutOfRangeException">The node is not in the graph.</exception>
    public abstract void SetGoal(int node);

    /// <summary>
    /// Applies the recorded changes and repairs the search until the start's cost is exact or <paramref name="budget"/> iterations
    /// have been spent. A call that returns <see cref="PathOutcome.BudgetExhausted"/> can be continued by calling again; the result
    /// is the same as one unsliced call.
    /// </summary>
    /// <param name="budget">Most queue pops this call may make; at least 1.</param>
    /// <returns>Found, NoPath, StartInvalid, GoalInvalid or BudgetExhausted.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The budget is below 1.</exception>
    public abstract PathOutcome Replan(int budget = int.MaxValue);

    /// <summary>The cost of the best path after <see cref="Replan"/> returned Found (+∞ otherwise).</summary>
    public abstract double Cost { get; }

    /// <summary>
    /// The nodes of the best path from start to goal, or null when none is known (call after <see cref="Replan"/> returned Found).
    /// Among neighbours with equal total cost the first in the topology's neighbour order is taken, preferring nodes whose
    /// estimate is settled.
    /// </summary>
    /// <param name="cost">The path's cost, summed from the start in path order.</param>
    /// <returns>The nodes, start first.</returns>
    public abstract int[]? ExtractPath(out double cost);
}

/// <summary>
/// LPA* and D* Lite over any <see cref="IGridTopology"/> (tier 2): struct-generic topology, cost hook and heuristic, like
/// <see cref="AStar{TTopology, TCost, THeuristic}"/>.
/// <para>
/// One engine serves both: the search runs from a <i>root</i> (LPA*: the start; D* Lite: the goal) and stops when the
/// <i>focus</i> (LPA*: the goal; D* Lite: the start) has an exact cost. <c>g</c> is the cost from the root, <c>rhs</c> the
/// one-step lookahead over the neighbours; a node with g ≠ rhs is queued with key
/// <c>[min(g, rhs) + h(focus, node) + km, min(g, rhs)]</c> (<see cref="ReplanOpenSet"/> order). Following Koenig &amp; Likhachev the loop pops
/// while the top key is smaller than the focus's key or the focus is inconsistent. The paper's per-edge updates are done as a full
/// recomputation of <c>rhs</c> for every node an edit touches, which is equivalent and handles any number of changed edges.
/// </para>
/// <para>
/// The graph must be undirected in structure (a move a→b exists iff b→a does, as on the square and hex topologies); the cost
/// hook may differ by direction. The heuristic must be consistent for the costs used. Costs of +∞ or NaN remove a move.
/// </para>
/// </summary>
/// <typeparam name="TTopology">The graph.</typeparam>
/// <typeparam name="TCost">The cost hook.</typeparam>
/// <typeparam name="THeuristic">The distance estimate; it must be symmetric (distance between two nodes, not only to the goal).</typeparam>
public sealed partial class ReplanSearch<TTopology, TCost, THeuristic> : ReplanSearch
    where TTopology : struct, IGridTopology
    where TCost : struct, IStepCost
    where THeuristic : struct, IHeuristic
{
    private readonly TTopology _topology;
    private TCost _cost;
    private THeuristic _heuristic;
    private readonly ReplanMode _mode;
    private readonly int _nodeCount;
    private readonly double[] _g;
    private readonly double[] _rhs;
    private readonly ReplanOpenSet _queue;
    private readonly List<int> _pending = [];
    private bool _pendingOverflow;
    private int _start;
    private int _goal;
    private double _km;
    private bool _initialized;
    private long _iterations;
    private long _expanded;
    private int _lastExpanded;
    private bool _found;
    private bool _record;
    private int[] _lastNodes = [];
    private int _lastNodeCount;

    /// <summary>Creates the search; nothing is computed until <see cref="Replan"/>.</summary>
    /// <param name="topology">The graph.</param>
    /// <param name="cost">The cost hook.</param>
    /// <param name="heuristic">The estimate; its goal is set by the search.</param>
    /// <param name="mode">LPA* or D* Lite.</param>
    /// <param name="start">Start node.</param>
    /// <param name="goal">Goal node.</param>
    /// <exception cref="ArgumentOutOfRangeException">A node is not in the graph.</exception>
    public ReplanSearch(TTopology topology, TCost cost, THeuristic heuristic, ReplanMode mode, int start, int goal)
    {
        _topology = topology;
        _cost = cost;
        _heuristic = heuristic;
        _mode = mode;
        _nodeCount = topology.NodeCount;
        CheckNode(start);
        CheckNode(goal);
        _start = start;
        _goal = goal;
        _g = new double[_nodeCount];
        _rhs = new double[_nodeCount];
        _queue = new ReplanOpenSet(_nodeCount);
    }

    /// <inheritdoc/>
    public override ReplanMode Mode => _mode;

    /// <inheritdoc/>
    public override int Start => _start;

    /// <inheritdoc/>
    public override int Goal => _goal;

    /// <inheritdoc/>
    public override long Iterations => _iterations;

    /// <inheritdoc/>
    public override long ExpandedCount => _expanded;

    /// <inheritdoc/>
    public override int LastExpanded => _lastExpanded;

    /// <inheritdoc/>
    public override bool RecordExpansions
    {
        get => _record;
        set => _record = value;
    }

    /// <inheritdoc/>
    public override ReadOnlySpan<int> LastExpandedNodes => new(_lastNodes, 0, _lastNodeCount);

    private void RecordNode(int node)
    {
        if (_lastNodeCount == _lastNodes.Length)
        {
            Array.Resize(ref _lastNodes, Math.Max(64, _lastNodes.Length * 2));
        }
        _lastNodes[_lastNodeCount++] = node;
    }

    /// <summary>The cost from the root found so far for a node (+∞ if unknown). Exact for settled nodes.</summary>
    /// <param name="node">Node index.</param>
    /// <returns>g.</returns>
    public double G(int node) => _g[node];

    /// <summary>The one-step lookahead cost of a node.</summary>
    /// <param name="node">Node index.</param>
    /// <returns>rhs.</returns>
    public double Rhs(int node) => _rhs[node];

    // Costs are sums of doubles added in different orders by different repairs, so two costs that are equal on paper can differ in the last
    // bits. Without a tolerance a node whose key ties the focus's key on paper may look a hair larger, be left in the open list, and leave
    // a stale cost behind. Comparisons that decide "is this settled" therefore allow a relative 1e-11; stopping late is harmless, stopping early is not.
    private const double Relative = 1e-11;

    private static double Slack(double a) => double.IsInfinity(a) ? 0.0 : Relative * Math.Max(1.0, Math.Abs(a));

    private static bool Same(double a, double b) => a == b || (!double.IsInfinity(a) && !double.IsInfinity(b) && Math.Abs(a - b) <= Slack(a));

    private int Root => _mode == ReplanMode.LpaStar ? _start : _goal;

    private int Focus => _mode == ReplanMode.LpaStar ? _goal : _start;

    private bool Reverse => _mode == ReplanMode.DStarLite;

    private void CheckNode(int node)
    {
        if ((uint)node >= (uint)_nodeCount)
        {
            throw new ArgumentOutOfRangeException(nameof(node), node, $"Node {node} is not in the graph of {_nodeCount} nodes.");
        }
    }

    /// <inheritdoc/>
    public override void MarkChanged(int node, bool withNeighbors)
    {
        CheckNode(node);
        if (!_initialized || _pendingOverflow)
        {
            return;
        }
        if (_pending.Count >= _nodeCount)
        {
            // So many edits that repairing one by one would cost more than starting over.
            _pendingOverflow = true;
            _pending.Clear();
            return;
        }
        _pending.Add(withNeighbors ? ~node : node);
    }

    /// <inheritdoc/>
    public override void Restart()
    {
        _initialized = false;
        _pending.Clear();
        _pendingOverflow = false;
    }

    /// <inheritdoc/>
    public override void MoveStart(int node)
    {
        CheckNode(node);
        if (node == _start)
        {
            return;
        }
        if (_mode == ReplanMode.DStarLite && _initialized)
        {
            // Keys in the queue were computed against the old start; km raises every later key by the most the estimate
            // between the two starts can have changed, so queued keys stay valid lower bounds (D* Lite, Koenig & Likhachev 2002).
            _heuristic.SetGoal(node);
            _km += _heuristic.Estimate(_start);
            _start = node;
            return;
        }
        _start = node;
        Restart();
    }

    /// <inheritdoc/>
    public override void SetGoal(int node)
    {
        CheckNode(node);
        if (node == _goal)
        {
            return;
        }
        _goal = node;
        Restart();
    }

    private void Initialize()
    {
        Array.Fill(_g, double.PositiveInfinity);
        Array.Fill(_rhs, double.PositiveInfinity);
        _queue.Clear();
        _km = 0.0;
        _heuristic.SetGoal(Focus);
        int root = Root;
        _rhs[root] = 0.0;
        _queue.Set(root, _heuristic.Estimate(root), 0.0);
        _pending.Clear();
        _pendingOverflow = false;
        _initialized = true;
        _iterations = 0;
        _expanded = 0;
    }

    private (double, double) CalculateKey(int node)
    {
        double m = Math.Min(_g[node], _rhs[node]);
        return (m + _heuristic.Estimate(node) + _km, m);
    }

    private struct RhsVisitor : INeighborVisitor
    {
        public double[] G;
        public TCost Cost;
        public int Node;
        public bool Reverse;
        public double Best;

        public void Visit(int neighbor, double baseCost)
        {
            double gv = G[neighbor];
            if (double.IsPositiveInfinity(gv))
            {
                return;
            }
            double c = Reverse ? Cost.Cost(Node, neighbor, baseCost) : Cost.Cost(neighbor, Node, baseCost);
            if (!(c < double.PositiveInfinity))
            {
                return;
            }
            double total = gv + c;
            if (total < Best)
            {
                Best = total;
            }
        }
    }

    private struct NeighborUpdater : INeighborVisitor
    {
        public ReplanSearch<TTopology, TCost, THeuristic> Owner;

        public readonly void Visit(int neighbor, double baseCost) => Owner.UpdateVertex(neighbor);
    }

    private double ComputeRhs(int node)
    {
        if (!_topology.IsWalkable(node))
        {
            return double.PositiveInfinity;
        }
        var v = new RhsVisitor { G = _g, Cost = _cost, Node = node, Reverse = Reverse, Best = double.PositiveInfinity };
        _topology.VisitNeighbors(node, ref v);
        return v.Best;
    }

    // Koenig & Likhachev 2002, UpdateVertex: refresh rhs, then queue the node iff it is inconsistent.
    private void UpdateVertex(int node)
    {
        if (node != Root)
        {
            _rhs[node] = ComputeRhs(node);
        }
        if (!Same(_g[node], _rhs[node]))
        {
            (double k1, double k2) = CalculateKey(node);
            _queue.Set(node, k1, k2);
        }
        else
        {
            _queue.Remove(node);
        }
    }

    private void UpdateNeighbors(int node)
    {
        var u = new NeighborUpdater { Owner = this };
        _topology.VisitNeighbors(node, ref u);
    }

    private void ApplyPending()
    {
        if (_pendingOverflow)
        {
            Restart();
            return;
        }
        for (int i = 0; i < _pending.Count; i++)
        {
            int e = _pending[i];
            if (e < 0)
            {
                int node = ~e;
                UpdateVertex(node);
                UpdateNeighbors(node);
            }
            else
            {
                UpdateVertex(e);
            }
        }
        _pending.Clear();
    }

    /// <inheritdoc/>
    public override PathOutcome Replan(int budget = int.MaxValue)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(budget, 1);
        PathOutcome status = ReplanCore(budget);
        _found = status == PathOutcome.Found;
        return status;
    }

    private PathOutcome ReplanCore(int budget)
    {
        _lastExpanded = 0;
        _lastNodeCount = 0;
        if (!_initialized || _pendingOverflow)
        {
            Initialize();
        }
        else
        {
            ApplyPending();
        }
        if (!_topology.IsWalkable(_start))
        {
            return PathOutcome.StartInvalid;
        }
        if (!_topology.IsWalkable(_goal))
        {
            return PathOutcome.GoalInvalid;
        }
        int focus = Focus;
        int spent = 0;
        while (true)
        {
            (double f1, double f2) = CalculateKey(focus);
            double t1 = _queue.TopKey1;
            double slack = Slack(f1);
            bool topSmaller = t1 < f1 - slack || (t1 <= f1 + slack && _queue.TopKey2 < f2 - Slack(f2));
            if ((!topSmaller && Same(_rhs[focus], _g[focus])) || _queue.Count == 0)
            {
                break;
            }
            if (spent >= budget)
            {
                return PathOutcome.BudgetExhausted;
            }
            spent++;
            _iterations++;
            int u = _queue.Top;
            double k1Old = t1;
            double k2Old = _queue.TopKey2;
            (double k1, double k2) = CalculateKey(u);
            if (k1Old < k1 - Slack(k1) || (k1Old <= k1 + Slack(k1) && k2Old < k2 - Slack(k2)))
            {
                // The key went stale (the start moved); put the node back where it belongs.
                _queue.Set(u, k1, k2);
            }
            else if (_g[u] > _rhs[u])
            {
                // Overconsistent: the node got cheaper, settle it and let its neighbours see the new cost.
                _g[u] = _rhs[u];
                _queue.Remove(u);
                _expanded++;
                _lastExpanded++;
                if (_record)
                {
                    RecordNode(u);
                }
                UpdateNeighbors(u);
            }
            else
            {
                // Underconsistent: the node got dearer. Forget its cost and re-derive it and its neighbours.
                _g[u] = double.PositiveInfinity;
                UpdateVertex(u);
                _expanded++;
                _lastExpanded++;
                if (_record)
                {
                    RecordNode(u);
                }
                UpdateNeighbors(u);
            }
        }
        return double.IsPositiveInfinity(_g[focus]) ? PathOutcome.NoPath : PathOutcome.Found;
    }

    /// <inheritdoc/>
    public override double Cost => _found ? _g[Focus] : double.PositiveInfinity;

    private struct StepChooser : INeighborVisitor
    {
        public double[] G;
        public double[] Rhs;
        public TCost Cost;
        public int Node;
        public bool Reverse;
        public int BestSettled;
        public double BestSettledTotal;
        public double BestSettledCost;
        public int BestAny;
        public double BestAnyTotal;
        public double BestAnyCost;

        public void Visit(int neighbor, double baseCost)
        {
            double gv = G[neighbor];
            if (double.IsPositiveInfinity(gv))
            {
                return;
            }
            double c = Reverse ? Cost.Cost(Node, neighbor, baseCost) : Cost.Cost(neighbor, Node, baseCost);
            if (!(c < double.PositiveInfinity))
            {
                return;
            }
            double total = gv + c;
            if (total < BestAnyTotal)
            {
                BestAnyTotal = total;
                BestAny = neighbor;
                BestAnyCost = c;
            }
            if (Same(Rhs[neighbor], gv) && total < BestSettledTotal)
            {
                BestSettledTotal = total;
                BestSettled = neighbor;
                BestSettledCost = c;
            }
        }
    }

    /// <inheritdoc/>
    public override int[]? ExtractPath(out double cost)
    {
        cost = double.PositiveInfinity;
        int focus = Focus;
        int root = Root;
        if (!_found || double.IsPositiveInfinity(_g[focus]))
        {
            return null;
        }
        var nodes = new List<int> { focus };
        var costs = new List<double>();
        int cur = focus;
        while (cur != root)
        {
            if (nodes.Count > _nodeCount)
            {
                return null;
            }
            var s = new StepChooser
            {
                G = _g,
                Rhs = _rhs,
                Cost = _cost,
                Node = cur,
                Reverse = Reverse,
                BestSettled = -1,
                BestSettledTotal = double.PositiveInfinity,
                BestAny = -1,
                BestAnyTotal = double.PositiveInfinity,
            };
            _topology.VisitNeighbors(cur, ref s);
            // A neighbour that is still queued has a cost that may be out of date; settled ones are exact (see docs/grids.md).
            int next = s.BestSettled >= 0 ? s.BestSettled : s.BestAny;
            if (next < 0)
            {
                return null;
            }
            costs.Add(s.BestSettled >= 0 ? s.BestSettledCost : s.BestAnyCost);
            nodes.Add(next);
            cur = next;
        }
        // Order the nodes start first and add the move costs up in that order, the way A* adds them.
        if (!Reverse)
        {
            nodes.Reverse();
            costs.Reverse();
        }
        double sum = 0.0;
        for (int i = 0; i < costs.Count; i++)
        {
            sum += costs[i];
        }
        cost = sum;
        return nodes.ToArray();
    }
}
