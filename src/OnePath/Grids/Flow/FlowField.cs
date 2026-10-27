using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// A flow field for many agents heading to one goal set: the <b>integration field</b> holds the exact least cost from
/// every cell to the nearest goal (Dijkstra's algorithm run outwards from the goals, the integration step of
/// flow-field pathfinding as described by Emerson, "Crowd Pathfinding and Steering Using Flow Field Tiles", 2013),
/// and the <b>direction field</b> holds, per cell, the neighbour to step to.
/// <para>
/// Costs are the topology's move costs (area multipliers, agent clearance, diagonal rule) passed through the
/// <typeparamref name="TCost"/> hook; the hook is asked for the cost of the move <i>out of</i> the cell being
/// labelled, the direction an agent travels. Moves must cost the same in both directions between the same two
/// cells (all built-in topologies do), because the wave runs backwards from the goals along the neighbour lists.
/// </para>
/// <para>
/// Normative rules: goals cost 0. The cost of any other cell is <c>min over neighbours u of (cost[u] + move(cell, u))</c>,
/// computed in IEEE double, so equal inputs give equal bits. The next cell of a non-goal reachable cell is the first
/// neighbour, in the topology's neighbour order, that attains that minimum; goals and unreachable cells have none (−1).
/// The open list is <see cref="OpenSet"/> keyed by cost, ties on node index; because the cost of a cell is the
/// minimum over its neighbours' final costs, the result does not depend on the order cells are settled.
/// </para>
/// <para>
/// The field is resumable (<see cref="Step"/> counts cells settled, never time) and can be repaired after cells
/// change (<see cref="BeginRepair"/>): the repaired field equals a field computed from scratch, bit for bit.
/// Not thread-safe; use one instance per thread. Several instances may read the same grid concurrently.
/// </para>
/// </summary>
/// <typeparam name="TTopology">The graph.</typeparam>
/// <typeparam name="TCost">The cost hook (<see cref="DefaultStepCost"/> for none).</typeparam>
public sealed class FlowField<TTopology, TCost>
    where TTopology : struct, IGridTopology
    where TCost : struct, IStepCost
{
    private readonly TTopology _topology;
    private readonly TCost _cost;
    private readonly double[] _dist;
    private readonly int[] _next;
    private readonly OpenSet _open;
    private readonly List<int> _goals = [];
    private uint[]? _stamp; // repair scratch: marks nodes already invalidated or touched
    private uint _stampBase;
    private readonly List<int> _touched = [];
    private int _settled;
    private FlowFieldStatus _status;
    private bool _repairing;

    /// <summary>Creates a field for a graph; call <see cref="Begin"/> to compute it.</summary>
    /// <param name="topology">The graph.</param>
    /// <param name="cost">The cost hook.</param>
    public FlowField(TTopology topology, TCost cost)
    {
        _topology = topology;
        _cost = cost;
        int n = topology.NodeCount;
        _dist = new double[n];
        _next = new int[n];
        _open = new OpenSet(n);
        Array.Fill(_dist, double.PositiveInfinity);
        Array.Fill(_next, -1);
    }

    /// <summary>Number of nodes in the graph.</summary>
    public int NodeCount => _dist.Length;

    /// <summary>Where the computation stands.</summary>
    public FlowFieldStatus Status => _status;

    /// <summary>Cells settled so far in the current computation (the unit of the iteration budget).</summary>
    public int Settled => _settled;

    /// <summary>The goal nodes given to <see cref="Begin"/>, sorted and without duplicates (inactive goals included).</summary>
    public IReadOnlyList<int> Goals => _goals;

    /// <summary>
    /// The integration field: least cost from each node to the nearest goal, +∞ if unreachable or not walkable.
    /// Final once <see cref="Status"/> is <see cref="FlowFieldStatus.Complete"/>.
    /// </summary>
    public ReadOnlySpan<double> Costs => _dist;

    /// <summary>The direction field as node indices: the neighbour to step to, or −1 for goals and unreachable nodes. Final once complete.</summary>
    public ReadOnlySpan<int> Next => _next;

    /// <summary>The least cost from a node to the nearest goal.</summary>
    /// <param name="node">Node index.</param>
    /// <returns>The cost, +∞ if unreachable.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The node is outside the graph.</exception>
    public double Cost(int node)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(node);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(node, _dist.Length);
        return _dist[node];
    }

    /// <summary>Starts a new computation, discarding the previous field.</summary>
    /// <param name="goals">Goal nodes; at least one. Goals that are not walkable are kept but inactive (they seed nothing).</param>
    /// <exception cref="ArgumentException">The list is empty.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A goal is outside the graph.</exception>
    public void Begin(ReadOnlySpan<int> goals)
    {
        if (goals.IsEmpty)
        {
            throw new ArgumentException("At least one goal is required.", nameof(goals));
        }
        int n = _dist.Length;
        for (int i = 0; i < goals.Length; i++)
        {
            if ((uint)goals[i] >= (uint)n)
            {
                throw new ArgumentOutOfRangeException(nameof(goals), goals[i], $"Goal node {goals[i]} is outside the graph of {n} nodes.");
            }
        }
        _goals.Clear();
        _goals.AddRange(goals.ToArray());
        _goals.Sort();
        int w = 0;
        for (int i = 0; i < _goals.Count; i++)
        {
            if (i == 0 || _goals[i] != _goals[i - 1])
            {
                _goals[w++] = _goals[i];
            }
        }
        _goals.RemoveRange(w, _goals.Count - w);

        _open.Clear();
        Array.Fill(_dist, double.PositiveInfinity);
        Array.Fill(_next, -1);
        _settled = 0;
        _repairing = false;
        for (int i = 0; i < _goals.Count; i++)
        {
            int g = _goals[i];
            if (_topology.IsWalkable(g))
            {
                _dist[g] = 0.0;
                _open.Push(g, 0.0, 0.0);
            }
        }
        _status = FlowFieldStatus.Running;
        if (_open.Count == 0)
        {
            Finish();
        }
    }

    /// <summary>Settles at most <paramref name="maxIterations"/> more cells (one iteration pops the cheapest open cell and relaxes its neighbours).</summary>
    /// <param name="maxIterations">Budget for this call; at least 1.</param>
    /// <returns>The status afterwards. A sliced computation ends with exactly the field an unsliced one gives.</returns>
    /// <exception cref="InvalidOperationException"><see cref="Begin"/> has not been called.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxIterations"/> is below 1.</exception>
    public FlowFieldStatus Step(int maxIterations)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(maxIterations, 1);
        if (_status == FlowFieldStatus.NotStarted)
        {
            throw new InvalidOperationException("Call Begin before Step.");
        }
        return Loop(maxIterations, default);
    }

    /// <summary>Runs the computation to completion.</summary>
    /// <param name="cancellationToken">Checked every 256 iterations; cancelling throws and leaves the computation resumable.</param>
    /// <returns><see cref="FlowFieldStatus.Complete"/>.</returns>
    /// <exception cref="InvalidOperationException"><see cref="Begin"/> has not been called.</exception>
    /// <exception cref="OperationCanceledException">Cancelled.</exception>
    public FlowFieldStatus Run(CancellationToken cancellationToken = default)
    {
        if (_status == FlowFieldStatus.NotStarted)
        {
            throw new InvalidOperationException("Call Begin before Run.");
        }
        return Loop(int.MaxValue, cancellationToken);
    }

    private FlowFieldStatus Loop(int maxIterations, CancellationToken token)
    {
        if (_status == FlowFieldStatus.Complete)
        {
            return _status;
        }
        var relax = new Relax { Owner = this };
        int done = 0;
        while (done < maxIterations)
        {
            if ((done & 255) == 0)
            {
                token.ThrowIfCancellationRequested();
            }
            if (!_open.Pop(out int node, out _, out _))
            {
                Finish();
                return _status;
            }
            done++;
            _settled++;
            relax.From = node;
            relax.Base = _dist[node];
            _topology.VisitNeighbors(node, ref relax);
        }
        if (_open.Count == 0)
        {
            Finish();
        }
        return _status;
    }

    // Relaxes the neighbour of a settled node: a neighbour reaches the goals through the settled node at
    // cost[from] + (cost of the neighbour's move onto from).
    private struct Relax : INeighborVisitor
    {
        public FlowField<TTopology, TCost> Owner;
        public int From;
        public double Base;

        public void Visit(int neighbor, double baseCost)
        {
            double w = Owner._cost.Cost(neighbor, From, baseCost);
            if (!(w >= 0.0) || double.IsPositiveInfinity(w))
            {
                return; // NaN, negative or +∞: the move is forbidden
            }
            double nd = Base + w;
            if (nd < Owner._dist[neighbor])
            {
                Owner._dist[neighbor] = nd;
                Owner._open.Push(neighbor, nd, nd);
                Owner.NoteChanged(neighbor);
            }
        }
    }

    // Direction rule: the first neighbour in visit order attaining the minimum of cost[u] + move(node, u).
    private struct PickNext : INeighborVisitor
    {
        public FlowField<TTopology, TCost> Owner;
        public int Node;
        public double Best;
        public int Choice;

        public void Visit(int neighbor, double baseCost)
        {
            double du = Owner._dist[neighbor];
            if (double.IsPositiveInfinity(du))
            {
                return;
            }
            double w = Owner._cost.Cost(Node, neighbor, baseCost);
            if (!(w >= 0.0) || double.IsPositiveInfinity(w))
            {
                return;
            }
            double t = du + w;
            if (t < Best)
            {
                Best = t;
                Choice = neighbor;
            }
        }
    }

    private void ComputeNext(int node)
    {
        double d = _dist[node];
        if (double.IsPositiveInfinity(d) || (d == 0.0 && IsGoal(node)))
        {
            _next[node] = -1;
            return;
        }
        var pick = new PickNext { Owner = this, Node = node, Best = double.PositiveInfinity, Choice = -1 };
        _topology.VisitNeighbors(node, ref pick);
        _next[node] = pick.Choice;
    }

    private bool IsGoal(int node)
    {
        int lo = 0;
        int hi = _goals.Count - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >> 1;
            int g = _goals[mid];
            if (g == node)
            {
                return true;
            }
            if (g < node)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid - 1;
            }
        }
        return false;
    }

    private void Finish()
    {
        if (_repairing)
        {
            // Only cells whose cost changed, their neighbours and the dirty cells can have a new direction.
            int count = _touched.Count;
            for (int i = 0; i < count; i++)
            {
                ComputeNext(_touched[i]);
            }
            _repairing = false;
        }
        else
        {
            for (int i = 0; i < _dist.Length; i++)
            {
                if (!double.IsPositiveInfinity(_dist[i]))
                {
                    ComputeNext(i);
                }
            }
        }
        _status = FlowFieldStatus.Complete;
    }

    private struct Collect : INeighborVisitor
    {
        public FlowField<TTopology, TCost> Owner;
        public int Mode; // 0: invalidate descendants of Node, 1: seed Node from settled neighbours, 2: add neighbours to touched
        public int Node;
        public double Best;
        public Queue<int> Queue;

        public void Visit(int neighbor, double baseCost)
        {
            FlowField<TTopology, TCost> o = Owner;
            switch (Mode)
            {
                case 0:
                    if (o._next[neighbor] == Node && o.TryMark(neighbor))
                    {
                        Queue.Enqueue(neighbor);
                    }
                    break;
                case 1:
                    {
                        double du = o._dist[neighbor];
                        if (double.IsPositiveInfinity(du) || o.IsMarked(neighbor))
                        {
                            return;
                        }
                        double w = o._cost.Cost(Node, neighbor, baseCost);
                        if (!(w >= 0.0) || double.IsPositiveInfinity(w))
                        {
                            return;
                        }
                        double t = du + w;
                        if (t < Best)
                        {
                            Best = t;
                        }
                        break;
                    }
                default:
                    o.Touch(neighbor);
                    break;
            }
        }
    }

    private bool IsMarked(int node) => _stamp![node] == _stampBase;

    private bool TryMark(int node)
    {
        if (_stamp![node] == _stampBase)
        {
            return false;
        }
        _stamp[node] = _stampBase;
        return true;
    }

    private uint[]? _touchStamp;

    private void Touch(int node)
    {
        if (_touchStamp![node] != _stampBase)
        {
            _touchStamp[node] = _stampBase;
            _touched.Add(node);
        }
    }

    /// <summary>
    /// Starts repairing the field after cells changed, recomputing only what the change can affect; finish it with
    /// <see cref="Step"/> or <see cref="Run"/>. The result equals <see cref="Begin"/> on the new graph with the same goals, bit for bit.
    /// Invalidation of the affected region happens in this call (it is not sliced); the wave that refills it is.
    /// </summary>
    /// <param name="dirty">
    /// Every node with an incident move whose existence or cost changed (both endpoints of every changed move) and every node
    /// whose walkability changed. For a grid, the cells within one step of an edited cell (more for agents wider than one cell).
    /// Listing extra nodes is always safe.
    /// </param>
    /// <exception cref="InvalidOperationException">No field has been computed, or a computation is still running.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A node is outside the graph.</exception>
    public void BeginRepair(ReadOnlySpan<int> dirty)
    {
        if (_status != FlowFieldStatus.Complete)
        {
            throw new InvalidOperationException("Repair needs a completed field; call Begin and Run first.");
        }
        int n = _dist.Length;
        for (int i = 0; i < dirty.Length; i++)
        {
            if ((uint)dirty[i] >= (uint)n)
            {
                throw new ArgumentOutOfRangeException(nameof(dirty), dirty[i], $"Node {dirty[i]} is outside the graph of {n} nodes.");
            }
        }
        _stamp ??= new uint[n];
        _touchStamp ??= new uint[n];
        if (_stampBase >= uint.MaxValue - 2)
        {
            Array.Clear(_stamp);
            Array.Clear(_touchStamp);
            _stampBase = 0;
        }
        _stampBase++;
        _touched.Clear();
        _open.Clear();
        _settled = 0;

        // 1. Invalidate the dirty nodes and every node whose chosen route runs through an invalid node
        //    (Ramalingam & Reps 1996, dynamic shortest paths: the affected set of a changed edge).
        var invalid = new List<int>();
        var queue = new Queue<int>();
        for (int i = 0; i < dirty.Length; i++)
        {
            if (TryMark(dirty[i]))
            {
                queue.Enqueue(dirty[i]);
            }
        }
        var walk = new Collect { Owner = this, Mode = 0, Queue = queue };
        while (queue.Count > 0)
        {
            int x = queue.Dequeue();
            invalid.Add(x);
            walk.Node = x;
            _topology.VisitNeighbors(x, ref walk);
        }

        // 2. Reset them, remember which directions may change, then seed each from its still-valid neighbours.
        for (int i = 0; i < invalid.Count; i++)
        {
            int x = invalid[i];
            _dist[x] = double.PositiveInfinity;
            _next[x] = -1;
        }
        for (int i = 0; i < invalid.Count; i++)
        {
            int x = invalid[i];
            Touch(x);
            walk.Mode = 2;
            walk.Node = x;
            _topology.VisitNeighbors(x, ref walk);
        }
        for (int i = 0; i < invalid.Count; i++)
        {
            int x = invalid[i];
            if (!_topology.IsWalkable(x))
            {
                continue;
            }
            if (IsGoal(x))
            {
                _dist[x] = 0.0;
                _open.Push(x, 0.0, 0.0);
                continue;
            }
            walk.Mode = 1;
            walk.Node = x;
            walk.Best = double.PositiveInfinity;
            _topology.VisitNeighbors(x, ref walk);
            if (!double.IsPositiveInfinity(walk.Best))
            {
                _dist[x] = walk.Best;
                _open.Push(x, walk.Best, walk.Best);
            }
        }
        _repairing = true;
        _status = FlowFieldStatus.Running;
        if (_open.Count == 0)
        {
            Finish();
        }
    }

    // A node whose cost changed, and its neighbours, get their direction recomputed when a repair finishes.
    private void NoteChanged(int node)
    {
        if (_repairing)
        {
            Touch(node);
            var touch = new Collect { Owner = this, Mode = 2, Node = node };
            _topology.VisitNeighbors(node, ref touch);
        }
    }
}
