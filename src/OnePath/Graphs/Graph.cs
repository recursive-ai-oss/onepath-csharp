using System;
using System.Collections.Generic;
using System.Globalization;
using System.Threading;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Graphs;

namespace RecursiveAI.OnePath;

/// <summary>Which search a graph query runs.</summary>
public enum GraphAlgorithm
{
    /// <summary>A* when an estimate exists (node positions of a <see cref="PointGraph"/>, <see cref="GraphOptions.Landmarks"/> or <see cref="GraphOptions.Heuristic"/>), otherwise Dijkstra.</summary>
    Auto = 0,

    /// <summary>Dijkstra's algorithm (A* with a zero estimate).</summary>
    Dijkstra = 1,

    /// <summary>A* with the estimates that exist; Dijkstra when there are none.</summary>
    AStar = 2,

    /// <summary>Bidirectional Dijkstra with the exact stopping rule.</summary>
    BidirectionalDijkstra = 3,

    /// <summary>Bidirectional A* (consistent average potentials) with the estimates that exist; bidirectional Dijkstra when there are none.</summary>
    BidirectionalAStar = 4,
}

/// <summary>Plain options for one-line graph queries; every member is optional. No struct callbacks are needed.</summary>
public sealed record GraphOptions
{
    /// <summary>Plain defaults: the search picks itself (Dijkstra without estimates), no filters, unlimited.</summary>
    public static GraphOptions Default => new();

    /// <summary>Bidirectional A* (Dijkstra when there is no estimate), the fast choice for long queries on large graphs.</summary>
    public static GraphOptions Bidirectional => new() { Algorithm = GraphAlgorithm.BidirectionalAStar };

    /// <summary>A* with ALT landmark estimates (Goldberg &amp; Harrelson 2005).</summary>
    /// <param name="landmarks">Tables built for the graph that is queried.</param>
    /// <returns>A new options object.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="landmarks"/> is null.</exception>
    public static GraphOptions WithLandmarks(Landmarks landmarks)
    {
        ArgumentNullException.ThrowIfNull(landmarks);
        return new GraphOptions { Algorithm = GraphAlgorithm.AStar, Landmarks = landmarks };
    }

    /// <summary>The search to run (<see cref="GraphAlgorithm.Auto"/> by default).</summary>
    public GraphAlgorithm Algorithm { get; init; } = GraphAlgorithm.Auto;

    /// <summary>
    /// ALT landmarks used as an A* estimate (alone, or combined by maximum with the point positions and <see cref="Heuristic"/>).
    /// Must have been built for the graph that is queried, and for weights no larger than the edge costs the query uses.
    /// </summary>
    public Landmarks? Landmarks { get; init; }

    /// <summary>
    /// An extra A* estimate <c>(from, to) → lower bound on the distance</c>, combined by maximum with the others. It must be
    /// admissible and consistent for the optimality guarantee; bidirectional A* also evaluates it as <c>(start, v)</c>.
    /// </summary>
    public Func<int, int, double>? Heuristic { get; init; }

    /// <summary>Edge cost hook <c>(from, to, weight) → cost</c>; return +∞, a negative value or NaN to forbid an edge. Null uses the stored weight.</summary>
    public Func<int, int, double, double>? EdgeCost { get; init; }

    /// <summary>Node filter: nodes for which this returns true can not be entered (the start and goal included, which makes the query invalid).</summary>
    public Func<int, bool>? Blocked { get; init; }

    /// <summary>Maximum iterations (expansions); 0 means unlimited. A query that runs out ends with <see cref="PathStatus.BudgetExhausted"/>.</summary>
    public int IterationBudget { get; init => field = Require.That(value >= 0, value, nameof(IterationBudget), "0 (unlimited) or more"); }

    /// <summary>When true the result lists the expanded nodes in order (debug data; forward expansions first, then backward ones).</summary>
    public bool RecordExpansions { get; init; }
}

/// <summary>The result of a one-line graph query. Immutable.</summary>
public sealed class GraphPath
{
    private static readonly int[] NoNodes = [];
    private readonly int[] _nodes;
    private readonly int[] _expanded;
    private readonly Point3[] _positions;

    internal GraphPath(PathStatus status, double cost, int[]? nodes, int expandedCount, int[]? expanded, Point3[]? positions, GraphAlgorithm algorithm)
    {
        Status = status;
        Cost = cost;
        _nodes = nodes ?? NoNodes;
        ExpandedCount = expandedCount;
        _expanded = expanded ?? NoNodes;
        _positions = positions ?? [];
        Algorithm = algorithm;
    }

    /// <summary>How the query ended.</summary>
    public PathStatus Status { get; }

    /// <summary>True when a path to the goal was found.</summary>
    public bool Found => Status == PathStatus.Found;

    /// <summary>True when the result holds a usable path (found, or best effort toward the goal). Grid and graph searches never return a partial path.</summary>
    public bool HasPath => Status.HasPath();

    /// <summary>True for a best-effort path that stops short of the goal; always false here.</summary>
    public bool IsPartial => Status == PathStatus.Partial;

    /// <summary>The path cost (+∞ when no path was found).</summary>
    public double Cost { get; }

    /// <summary>The node ids of the path, start first; empty when no path was found.</summary>
    public ReadOnlySpan<int> Nodes => _nodes;

    /// <summary>The positions of the path's nodes for point graphs; empty otherwise.</summary>
    public ReadOnlySpan<Point3> Points => _positions;

    /// <summary>
    /// Length of the polyline through <see cref="Points"/>, in world units: 0 when there is no path, NaN when a path exists but the query
    /// produced no points (the cell or node overloads).
    /// </summary>
    public double Length
    {
        get
        {
            if (_nodes.Length == 0)
            {
                return 0;
            }
            if (_positions.Length == 0)
            {
                return double.NaN;
            }
            double l = 0;
            for (int i = 1; i < _positions.Length; i++)
            {
                l += Math.Sqrt(((_positions[i].X-_positions[i-1].X)*(_positions[i].X-_positions[i-1].X))+((_positions[i].Y-_positions[i-1].Y)*(_positions[i].Y-_positions[i-1].Y))+((_positions[i].Z-_positions[i-1].Z)*(_positions[i].Z-_positions[i-1].Z)));
            }
            return l;
        }
    }

    /// <summary>Number of nodes the search expanded.</summary>
    public int ExpandedCount { get; }

    /// <summary>Debug data: the expanded nodes in order; empty unless <see cref="GraphOptions.RecordExpansions"/> was set.</summary>
    public ReadOnlySpan<int> ExpandedNodes => _expanded;

    /// <summary>The algorithm that actually ran (it differs from <see cref="GraphOptions.Algorithm"/> when no estimate existed).</summary>
    public GraphAlgorithm Algorithm { get; }
}

/// <summary>
/// A mutable weighted graph: the tier-1 entry point (<c>Graph.From(edges).FindPath(a, b)</c>) and the owner of an edge list
/// from which the immutable <see cref="CsrGraph"/> is derived. Node ids are stable: nodes are never removed, and an edge keeps
/// its id (returned by <see cref="AddEdge"/>) for life, removed or not. Editing marks the compiled form stale; the next query (or
/// <see cref="Compile"/>) rebuilds it in O(nodes + edges), so batch edits before querying. Edits must not overlap queries;
/// concurrent queries on an unedited graph are safe.
/// </summary>
public sealed partial class Graph
{
    private readonly List<GraphEdge> _edges = [];
    private readonly List<bool> _alive = [];
    private readonly object _sync = new();
    private int _nodeCount;
    private int _liveEdges;
    private CsrGraph? _compiled;
    private SearchPair? _spare;

    /// <summary>Creates a graph with <paramref name="nodeCount"/> nodes and no edges.</summary>
    /// <param name="nodeCount">Initial node count, 0 to <see cref="CsrGraph.MaxNodes"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="nodeCount"/> is out of range.</exception>
    public Graph(int nodeCount = 0)
    {
        if ((uint)nodeCount > CsrGraph.MaxNodes)
        {
            throw new ArgumentOutOfRangeException(nameof(nodeCount), nodeCount, $"The node count must be between 0 and {CsrGraph.MaxNodes}.");
        }
        _nodeCount = nodeCount;
    }

    /// <summary>Number of nodes.</summary>
    public int NodeCount => _nodeCount;

    /// <summary>Number of edges that are not removed.</summary>
    public int EdgeCount => _liveEdges;

    /// <summary>
    /// Builds a graph from an edge list (the tier-1 entry point).
    /// </summary>
    /// <param name="edges">The edges; weights finite and not negative.</param>
    /// <param name="nodeCount">Number of nodes, or −1 (default) for one more than the largest id used.</param>
    /// <param name="directed">False (default): every edge is travelled both ways. True: edges are one-way.</param>
    /// <returns>The graph. Edge ids follow list order; for an undirected graph edge <c>i</c> has ids <c>2i</c> (forward) and <c>2i+1</c> (back).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="edges"/> is null.</exception>
    /// <exception cref="ArgumentException">An edge has a negative id or a bad weight; the message names the edge.</exception>
    public static Graph From(IEnumerable<GraphEdge> edges, int nodeCount = -1, bool directed = false)
    {
        ArgumentNullException.ThrowIfNull(edges);
        var list = new List<GraphEdge>(edges);
        int n = nodeCount;
        if (n < 0)
        {
            long max = -1;
            foreach (GraphEdge e in list)
            {
                max = Math.Max(max, Math.Max(e.From, e.To));
            }
            n = checked((int)(max + 1));
        }
        var g = new Graph(n);
        for (int i = 0; i < list.Count; i++)
        {
            GraphEdge e = list[i];
            try
            {
                if (directed)
                {
                    g.AddEdge(e.From, e.To, e.Weight);
                }
                else
                {
                    g.AddUndirectedEdge(e.From, e.To, e.Weight);
                }
            }
            catch (ArgumentException ex)
            {
                throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Edge {i}: {ex.Message}"), nameof(edges), ex);
            }
        }
        return g;
    }

    /// <summary>Builds a graph from <c>(from, to, weight)</c> tuples; see <see cref="From(IEnumerable{GraphEdge}, int, bool)"/>.</summary>
    /// <param name="edges">The edges.</param>
    /// <param name="nodeCount">Number of nodes, or −1 for one more than the largest id used.</param>
    /// <param name="directed">False (default): edges are travelled both ways.</param>
    /// <returns>The graph.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="edges"/> is null.</exception>
    /// <exception cref="ArgumentException">An edge has a negative id or a bad weight.</exception>
    public static Graph From(IEnumerable<(int From, int To, double Weight)> edges, int nodeCount = -1, bool directed = false)
    {
        ArgumentNullException.ThrowIfNull(edges);
        var list = new List<GraphEdge>();
        foreach ((int f, int t, double w) in edges)
        {
            list.Add(new GraphEdge(f, t, w));
        }
        return From(list, nodeCount, directed);
    }

    /// <summary>Adds a node.</summary>
    /// <returns>Its id (the previous node count).</returns>
    /// <exception cref="InvalidOperationException">The graph already has <see cref="CsrGraph.MaxNodes"/> nodes.</exception>
    public int AddNode()
    {
        if (_nodeCount >= CsrGraph.MaxNodes)
        {
            throw new InvalidOperationException($"A graph holds at most {CsrGraph.MaxNodes} nodes.");
        }
        Invalidate();
        return _nodeCount++;
    }

    /// <summary>Adds a directed edge.</summary>
    /// <param name="from">Source node.</param>
    /// <param name="to">Target node.</param>
    /// <param name="weight">Finite, not negative.</param>
    /// <returns>The edge's stable id.</returns>
    /// <exception cref="ArgumentException">An endpoint is outside the graph or the weight is negative, infinite or NaN.</exception>
    public int AddEdge(int from, int to, double weight)
    {
        if ((uint)from >= (uint)_nodeCount || (uint)to >= (uint)_nodeCount)
        {
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Edge {from} -> {to} has an endpoint outside the {_nodeCount} nodes of the graph."));
        }
        if (!(weight >= 0.0) || double.IsPositiveInfinity(weight))
        {
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Edge {from} -> {to} has weight {weight}; weights must be finite and not negative."));
        }
        Invalidate();
        _edges.Add(new GraphEdge(from, to, weight));
        _alive.Add(true);
        _liveEdges++;
        return _edges.Count - 1;
    }

    /// <summary>Adds the edge in both directions.</summary>
    /// <param name="a">One end.</param>
    /// <param name="b">The other end.</param>
    /// <param name="weight">Finite, not negative.</param>
    /// <returns>The id of <c>a → b</c>; <c>b → a</c> has the next id.</returns>
    /// <exception cref="ArgumentException">See <see cref="AddEdge"/>.</exception>
    public int AddUndirectedEdge(int a, int b, double weight)
    {
        int id = AddEdge(a, b, weight);
        AddEdge(b, a, weight);
        return id;
    }

    /// <summary>Removes an edge. Its id is never reused.</summary>
    /// <param name="edgeId">An id returned by <see cref="AddEdge"/>.</param>
    /// <returns>False if the edge was already removed.</returns>
    /// <exception cref="ArgumentOutOfRangeException">No such edge id.</exception>
    public bool RemoveEdge(int edgeId)
    {
        if ((uint)edgeId >= (uint)_edges.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(edgeId), edgeId, "No edge has this id.");
        }
        if (!_alive[edgeId])
        {
            return false;
        }
        Invalidate();
        _alive[edgeId] = false;
        _liveEdges--;
        return true;
    }

    /// <summary>Changes an edge's weight.</summary>
    /// <param name="edgeId">An id returned by <see cref="AddEdge"/> of an edge that is not removed.</param>
    /// <param name="weight">Finite, not negative.</param>
    /// <exception cref="ArgumentOutOfRangeException">No such edge id.</exception>
    /// <exception cref="ArgumentException">The edge is removed or the weight is bad.</exception>
    public void SetWeight(int edgeId, double weight)
    {
        if ((uint)edgeId >= (uint)_edges.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(edgeId), edgeId, "No edge has this id.");
        }
        if (!_alive[edgeId])
        {
            throw new ArgumentException($"Edge {edgeId} was removed.", nameof(edgeId));
        }
        if (!(weight >= 0.0) || double.IsPositiveInfinity(weight))
        {
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Weight {weight} is not valid; weights must be finite and not negative."), nameof(weight));
        }
        Invalidate();
        GraphEdge e = _edges[edgeId];
        _edges[edgeId] = e with { Weight = weight };
    }

    /// <summary>The compiled graph, rebuilt if edits happened since the last call. Edges appear in id order within each node.</summary>
    public CsrGraph Compile()
    {
        lock (_sync)
        {
            if (_compiled is null)
            {
                var live = new List<GraphEdge>(_liveEdges);
                for (int i = 0; i < _edges.Count; i++)
                {
                    if (_alive[i])
                    {
                        live.Add(_edges[i]);
                    }
                }
                _compiled = CsrGraph.Create(_nodeCount, live.ToArray());
            }
            return _compiled;
        }
    }

    /// <summary>Builds landmark tables for the current edges (see <see cref="Landmarks.Build"/>). Edits afterwards make the table stale.</summary>
    /// <param name="options">Count, selection and seed; null means <see cref="LandmarkOptions"/> defaults (16 landmarks, avoid, seed 1).</param>
    /// <returns>The table.</returns>
    /// <exception cref="ArgumentException">The graph has no nodes.</exception>
    public Landmarks BuildLandmarks(LandmarkOptions? options = null) => Landmarks.Build(Compile(), options?.Count ?? 16, options?.Selection ?? LandmarkSelection.Avoid, options?.Seed ?? 1);

    /// <summary>Finds the cheapest path from <paramref name="start"/> to <paramref name="goal"/>.</summary>
    /// <param name="start">Start node id.</param>
    /// <param name="goal">Goal node id.</param>
    /// <param name="options">Options; null means defaults (Dijkstra).</param>
    /// <param name="cancellationToken">Checked every 256 iterations.</param>
    /// <returns>The result. Ids outside the graph (or blocked by the filter) give <see cref="PathStatus.StartInvalid"/> or <see cref="PathStatus.GoalInvalid"/>.</returns>
    /// <exception cref="InvalidOperationException">The landmarks in <paramref name="options"/> were built for a different graph.</exception>
    /// <exception cref="OperationCanceledException">Cancellation was requested.</exception>
    public GraphPath FindPath(int start, int goal, GraphOptions? options = null, CancellationToken cancellationToken = default)
    {
        CsrGraph csr = Compile();
        SearchPair pair = Interlocked.Exchange(ref _spare, null) ?? new SearchPair(csr.NodeCount);
        try
        {
            return GraphRunner.Run(csr, null, pair, start, goal, options ?? new GraphOptions(), cancellationToken);
        }
        finally
        {
            _spare = pair;
        }
    }

    private void Invalidate()
    {
        lock (_sync)
        {
            _compiled = null;
            _spare = null;
        }
    }
}
