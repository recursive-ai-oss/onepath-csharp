using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// An HPA* query over an <see cref="HpaHierarchy"/> (Botea, Müller &amp; Schaeffer 2004): the start and goal are inserted into
/// the abstract graph (each is joined to every node of its chunk by an intra-chunk search), A* runs over the abstract graph
/// (nodes are cells, so ties break on cell index like everywhere else), and the abstract path is refined into cells by
/// searching inside each chunk it crosses. An optional smoothing pass then replaces stretches of the path by straight
/// horizontal, vertical or diagonal runs where that is cheaper (the paper's path smoothing).
/// <para>
/// The path is near optimal, not optimal, and it exists exactly when a path exists for A* with the same moves.
/// The search is resumable like <see cref="AStar{TTopology, TCost, THeuristic}"/>: <see cref="Step"/> performs at most the given
/// number of iterations, one iteration being one abstract node expansion (insertion of the start and goal happens in
/// <see cref="Begin"/> and refinement in the call that reaches the goal, neither counts); sliced and unsliced runs give the same path.
/// Not thread-safe; one search object per thread.
/// </para>
/// </summary>
public sealed class HpaSearch
{
    private static readonly (int Dx, int Dy)[] Directions =
    [
        (0, -1), (1, 0), (0, 1), (-1, 0), (1, -1), (1, 1), (-1, 1), (-1, -1),
    ];

    private readonly HpaHierarchy _hierarchy;
    private readonly GridSearchBuffers _buffers;
    private readonly ClusterSolver _solver;
    private readonly List<int> _path = [];
    private readonly int _width;
    private readonly int _height;
    private readonly double _scale;
    private readonly bool _eight;
    private int _start = -1;
    private int _goal = -1;
    private int _goalCluster;
    private int _goalX;
    private int _goalY;
    private double[] _startDist = [];
    private double _startToGoal;
    private double[] _goalDist = [];
    private int _expandedCount;
    private int _iterations;
    private double _pathCost;
    private SearchStatus _status;

    /// <summary>Creates a search object for a hierarchy.</summary>
    /// <param name="hierarchy">The hierarchy.</param>
    /// <param name="buffers">Working memory sized for the grid's cell count to reuse, or null to allocate.</param>
    /// <exception cref="ArgumentNullException"><paramref name="hierarchy"/> is null.</exception>
    /// <exception cref="ArgumentException">The buffers are smaller than the grid.</exception>
    public HpaSearch(HpaHierarchy hierarchy, GridSearchBuffers? buffers = null)
    {
        ArgumentNullException.ThrowIfNull(hierarchy);
        _hierarchy = hierarchy;
        _width = hierarchy.Grid.Width;
        _height = hierarchy.Grid.Height;
        _buffers = buffers ?? new GridSearchBuffers(hierarchy.Grid.CellCount);
        if (_buffers.NodeCapacity < hierarchy.Grid.CellCount)
        {
            throw new ArgumentException($"The search buffers hold {_buffers.NodeCapacity} nodes but the grid has {hierarchy.Grid.CellCount}.", nameof(buffers));
        }
        _solver = hierarchy.NewSolver();
        _scale = hierarchy.MinCost;
        _eight = hierarchy.Connectivity == Connectivity.Eight;
    }

    /// <summary>The hierarchy this search runs on.</summary>
    public HpaHierarchy Hierarchy => _hierarchy;

    /// <summary>Whether the refined path is smoothed (default true).</summary>
    public bool Smoothing { get; set; } = true;

    /// <summary>The state of the search.</summary>
    public SearchStatus Status => _status;

    /// <summary>Abstract nodes expanded so far.</summary>
    public int ExpandedCount => _expandedCount;

    /// <summary>The working memory (for <see cref="GridSearchBuffers.ExpandedNodes"/> when <see cref="GridSearchBuffers.RecordExpansions"/> is set).</summary>
    public GridSearchBuffers Buffers => _buffers;

    /// <summary>The cost of the final cell path (after refinement and smoothing) once found; +∞ otherwise.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathCost : double.PositiveInfinity;

    /// <summary>Number of cells of the found path, start and goal included.</summary>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    public int PathLength()
    {
        RequireFound();
        return _path.Count;
    }

    /// <summary>Writes the found path, start first, into <paramref name="destination"/>.</summary>
    /// <param name="destination">At least <see cref="PathLength"/> elements.</param>
    /// <returns>The number of cells written.</returns>
    /// <exception cref="InvalidOperationException">No path has been found.</exception>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int CopyPath(Span<int> destination)
    {
        RequireFound();
        if (destination.Length < _path.Count)
        {
            throw new ArgumentException($"The path has {_path.Count} cells but the destination holds {destination.Length}.", nameof(destination));
        }
        for (int i = 0; i < _path.Count; i++)
        {
            destination[i] = _path[i];
        }
        return _path.Count;
    }

    /// <summary>
    /// Starts a query: refreshes the hierarchy after edits, then inserts the start and goal. Discards any previous query.
    /// </summary>
    /// <param name="start">Start cell index.</param>
    /// <param name="goal">Goal cell index.</param>
    /// <exception cref="ArgumentOutOfRangeException">A cell is outside the grid.</exception>
    /// <exception cref="ArgumentException">A cell is not walkable for the hierarchy's agent size.</exception>
    public void Begin(int start, int goal)
    {
        int n = _width * _height;
        if ((uint)start >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(start), start, "Start cell is outside the grid.");
        }
        if ((uint)goal >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(goal), goal, "Goal cell is outside the grid.");
        }
        _hierarchy.Refresh();
        SquareTopology topology = _hierarchy.Topology;
        if (!topology.IsWalkable(start))
        {
            throw new ArgumentException("The start cell is not walkable.", nameof(start));
        }
        if (!topology.IsWalkable(goal))
        {
            throw new ArgumentException("The goal cell is not walkable.", nameof(goal));
        }
        _start = start;
        _goal = goal;
        _expandedCount = 0;
        _iterations = 0;
        _pathCost = double.PositiveInfinity;
        _path.Clear();
        _buffers.BeginSearch(n);
        if (start == goal)
        {
            _path.Add(start);
            _pathCost = 0.0;
            _status = SearchStatus.Found;
            return;
        }

        int startCluster = _hierarchy.ClusterOf(start);
        _goalCluster = _hierarchy.ClusterOf(goal);
        _goalY = goal / _width;
        _goalX = goal - (_goalY * _width);

        // Insert the start: distances to every node of its chunk (and to the goal when it shares the chunk).
        ReadOnlySpan<int> startNodes = _hierarchy.ClusterNodes(startCluster);
        _hierarchy.ClusterRect(startCluster, out int x0, out int y0, out int x1, out int y1);
        _solver.Run(x0, y0, x1, y1, start, startNodes, startCluster == _goalCluster ? goal : -1);
        if (_startDist.Length < startNodes.Length)
        {
            _startDist = new double[startNodes.Length];
        }
        for (int i = 0; i < startNodes.Length; i++)
        {
            _startDist[i] = _solver.Dist(startNodes[i]);
        }
        _startToGoal = startCluster == _goalCluster ? _solver.Dist(goal) : double.PositiveInfinity;

        // Insert the goal: distances from it to every node of its chunk (move costs are symmetric, so they serve as
        // the costs of the edges into the goal).
        ReadOnlySpan<int> goalNodes = _hierarchy.ClusterNodes(_goalCluster);
        _hierarchy.ClusterRect(_goalCluster, out x0, out y0, out x1, out y1);
        _solver.Run(x0, y0, x1, y1, goal, goalNodes);
        if (_goalDist.Length < goalNodes.Length)
        {
            _goalDist = new double[goalNodes.Length];
        }
        for (int i = 0; i < goalNodes.Length; i++)
        {
            _goalDist[i] = _solver.Dist(goalNodes[i]);
        }

        _buffers.G[start] = 0.0;
        _buffers.Parent[start] = -1;
        _buffers.Mark[start] = _buffers.Base;
        _buffers.Open.Push(start, Estimate(start), 0.0);
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

    private void RequireFound()
    {
        if (_status != SearchStatus.Found)
        {
            throw new InvalidOperationException("The search has not found a path.");
        }
    }

    private double Estimate(int cell)
    {
        int y = cell / _width;
        int x = cell - (y * _width);
        int dx = x > _goalX ? x - _goalX : _goalX - x;
        int dy = y > _goalY ? y - _goalY : _goalY - y;
        if (!_eight)
        {
            return _scale * (dx + dy);
        }
        int min = dx < dy ? dx : dy;
        int max = dx < dy ? dy : dx;
        return _scale * ((max - min) + (min * GridMath.Sqrt2));
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
                Refine();
                _status = SearchStatus.Found;
                return _status;
            }
            _expandedCount++;
            if (record)
            {
                b.RecordExpanded(node);
            }
            Expand(node);
        }
        return _status;
    }

    // Edges out of an abstract node, in a fixed order: edges to the nodes of its chunk (by node index), the goal, inter-edges.
    private void Expand(int u)
    {
        double g = _buffers.G[u];
        int cluster = _hierarchy.ClusterOf(u);
        ReadOnlySpan<int> nodes = _hierarchy.ClusterNodes(cluster);
        int index = _hierarchy.FindNode(cluster, u);
        if (u == _start)
        {
            for (int i = 0; i < nodes.Length; i++)
            {
                if (nodes[i] != u && _startDist[i] < double.PositiveInfinity)
                {
                    Relax(u, g, nodes[i], _startDist[i]);
                }
            }
            if (cluster == _goalCluster && _startToGoal < double.PositiveInfinity)
            {
                Relax(u, g, _goal, _startToGoal);
            }
        }
        else if (index >= 0)
        {
            HpaHierarchy.ClusterData d = _hierarchy.Cluster(cluster);
            int n = nodes.Length;
            for (int j = 0; j < n; j++)
            {
                double c = d.Intra[(index * n) + j];
                if (j != index && c < double.PositiveInfinity)
                {
                    Relax(u, g, nodes[j], c);
                }
            }
            if (cluster == _goalCluster && _goalDist[index] < double.PositiveInfinity)
            {
                Relax(u, g, _goal, _goalDist[index]);
            }
        }
        if (index >= 0)
        {
            HpaHierarchy.ClusterData d = _hierarchy.Cluster(cluster);
            for (int e = d.InterOffset[index]; e < d.InterOffset[index + 1]; e++)
            {
                Relax(u, g, d.InterTo[e], d.InterCost[e]);
            }
        }
    }

    private void Relax(int from, double gFrom, int to, double step)
    {
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
        b.Open.Push(to, g + Estimate(to), g);
    }

    // Turns the abstract path into cells: a move across a border is one step; an edge inside a chunk is searched again
    // inside that chunk (the same search that priced it, so the cells match its cost).
    private void Refine()
    {
        var abstractPath = new List<int>();
        for (int n = _goal; n >= 0; n = _buffers.Parent[n])
        {
            abstractPath.Add(n);
        }
        abstractPath.Reverse();
        _path.Add(abstractPath[0]);
        var target = new int[1];
        for (int i = 1; i < abstractPath.Count; i++)
        {
            int p = abstractPath[i - 1];
            int q = abstractPath[i];
            int c = _hierarchy.ClusterOf(p);
            if (c != _hierarchy.ClusterOf(q))
            {
                _path.Add(q);
                continue;
            }
            _hierarchy.ClusterRect(c, out int x0, out int y0, out int x1, out int y1);
            target[0] = q;
            _solver.Run(x0, y0, x1, y1, p, target);
            if (!_solver.AppendPath(q, _path))
            {
                throw new InvalidOperationException("The hierarchy is inconsistent with the grid: an intra-chunk edge could not be refined.");
            }
        }
        if (Smoothing)
        {
            Smooth();
        }
        _pathCost = Cost(_path);
    }

    private double Cost(List<int> cells)
    {
        double total = 0.0;
        for (int i = 1; i < cells.Count; i++)
        {
            total += _hierarchy.MoveCost(cells[i - 1], cells[i]);
        }
        return total;
    }

    // Path smoothing (Botea et al. 2004, section 5): from each cell of the path, walk straight in each of the allowed
    // directions; where the walk meets a later cell of the path and is cheaper than the path between them (by more than
    // 1e-9, so rounding noise never counts as an improvement) take the walk. Among the options the one reaching the
    // latest path position wins, then the cheaper walk, then the lower direction number.
    private void Smooth()
    {
        int count = _path.Count;
        if (count < 3)
        {
            return;
        }
        var prefix = new double[count];
        var index = new Dictionary<int, int>(count);
        index[_path[0]] = 0;
        for (int i = 1; i < count; i++)
        {
            prefix[i] = prefix[i - 1] + _hierarchy.MoveCost(_path[i - 1], _path[i]);
            index[_path[i]] = i; // a cell met twice maps to its last position
        }
        int dirs = _eight ? 8 : 4;
        var result = new List<int>(count) { _path[0] };
        int at = 0;
        while (at < count - 1)
        {
            int bestJ = -1;
            int bestDir = -1;
            int bestSteps = 0;
            double bestCost = 0.0;
            double remaining = prefix[count - 1] - prefix[at];
            for (int d = 0; d < dirs; d++)
            {
                int cur = _path[at];
                int cx = cur % _width;
                int cy = cur / _width;
                double acc = 0.0;
                for (int steps = 1; ; steps++)
                {
                    cx += Directions[d].Dx;
                    cy += Directions[d].Dy;
                    if ((uint)cx >= (uint)_width || (uint)cy >= (uint)_height)
                    {
                        break;
                    }
                    int next = (cy * _width) + cx;
                    double mc = _hierarchy.MoveCost(cur, next);
                    if (mc >= double.PositiveInfinity)
                    {
                        break;
                    }
                    acc += mc;
                    cur = next;
                    if (acc >= remaining)
                    {
                        break;
                    }
                    if (index.TryGetValue(cur, out int j) && j > at + 1 && acc + 1e-9 < prefix[j] - prefix[at]
                        && (j > bestJ || (j == bestJ && acc < bestCost)))
                    {
                        bestJ = j;
                        bestDir = d;
                        bestSteps = steps;
                        bestCost = acc;
                    }
                }
            }
            if (bestJ < 0)
            {
                result.Add(_path[at + 1]);
                at++;
                continue;
            }
            int px = _path[at] % _width;
            int py = _path[at] / _width;
            for (int s = 1; s <= bestSteps; s++)
            {
                result.Add(((py + (Directions[bestDir].Dy * s)) * _width) + px + (Directions[bestDir].Dx * s));
            }
            at = bestJ;
        }
        _path.Clear();
        _path.AddRange(result);
    }
}
