using System;
using System.Threading;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Graphs;

// Pooled working memory of one tier-1 query: forward buffers always, backward ones when a bidirectional search needs them.
internal sealed class SearchPair
{
    private readonly int _capacity;
    private GridSearchBuffers? _backward;

    internal SearchPair(int nodeCapacity)
    {
        _capacity = nodeCapacity;
        Forward = new GridSearchBuffers(nodeCapacity);
    }

    internal GridSearchBuffers Forward { get; }

    internal GridSearchBuffers Backward => _backward ??= new GridSearchBuffers(_capacity);
}

// The tier-1 plain options adapted onto the struct-generic searches: one adapter type for costs and one for estimates,
// so the happy path instantiates each search once instead of once per option combination.
internal struct OptionCost : IGraphCost
{
    public Func<int, int, double, double>? Edge;
    public Func<int, bool>? Blocked;

    public readonly double Cost(int from, int to, double weight)
    {
        if (Blocked is not null && Blocked(to))
        {
            return double.PositiveInfinity;
        }
        return Edge is null ? weight : Edge(from, to, weight);
    }
}

internal struct OptionHeuristic : IGraphHeuristic
{
    public Point3[]? Positions;
    public Landmarks? Landmarks;
    public Func<int, int, double>? Custom;

    internal readonly bool IsZero => Positions is null && Landmarks is null && Custom is null;

    public readonly double Estimate(int from, int to)
    {
        double best = 0.0;
        if (Positions is not null)
        {
            best = Positions[from].DistanceTo(Positions[to]);
        }
        if (Landmarks is not null)
        {
            double b = Landmarks.LowerBound(from, to);
            if (b > best)
            {
                best = b;
            }
        }
        if (Custom is not null)
        {
            double b = Custom(from, to);
            if (b > best)
            {
                best = b;
            }
        }
        return best;
    }
}

internal static class GraphRunner
{
    internal static GraphPath Run(CsrGraph graph, Point3[]? positions, SearchPair pair, int start, int goal, GraphOptions options, CancellationToken token)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(options.IterationBudget);
        if (options.Landmarks is not null && !options.Landmarks.Matches(graph))
        {
            throw new InvalidOperationException("The landmarks in the options were built for a different graph (or an earlier state of this one); rebuild them.");
        }
        var cost = new OptionCost { Edge = options.EdgeCost, Blocked = options.Blocked };
        if ((uint)start >= (uint)graph.NodeCount || (options.Blocked?.Invoke(start) ?? false))
        {
            return new GraphPath(PathStatus.StartInvalid, double.PositiveInfinity, null, 0, null, null, options.Algorithm);
        }
        if ((uint)goal >= (uint)graph.NodeCount || (options.Blocked?.Invoke(goal) ?? false))
        {
            return new GraphPath(PathStatus.GoalInvalid, double.PositiveInfinity, null, 0, null, null, options.Algorithm);
        }
        var heuristic = new OptionHeuristic { Positions = positions, Landmarks = options.Landmarks, Custom = options.Heuristic };
        GraphAlgorithm algorithm = options.Algorithm;
        if (algorithm == GraphAlgorithm.Auto)
        {
            algorithm = heuristic.IsZero ? GraphAlgorithm.Dijkstra : GraphAlgorithm.AStar;
        }
        if (heuristic.IsZero)
        {
            algorithm = algorithm == GraphAlgorithm.AStar ? GraphAlgorithm.Dijkstra
                : algorithm == GraphAlgorithm.BidirectionalAStar ? GraphAlgorithm.BidirectionalDijkstra : algorithm;
        }
        else if (algorithm == GraphAlgorithm.Dijkstra || algorithm == GraphAlgorithm.BidirectionalDijkstra)
        {
            heuristic = default;
        }
        pair.Forward.RecordExpansions = options.RecordExpansions;
        int budget = options.IterationBudget == 0 ? int.MaxValue : options.IterationBudget;
        bool bidirectional = algorithm == GraphAlgorithm.BidirectionalDijkstra || algorithm == GraphAlgorithm.BidirectionalAStar;
        SearchStatus status;
        int expandedCount;
        double pathCost;
        int[]? nodes = null;
        int[]? expanded = null;
        if (!bidirectional)
        {
            var search = new GraphAStar<OptionCost, OptionHeuristic>(graph, cost, heuristic, pair.Forward);
            search.Begin(start, goal);
            status = Drive(budget, search.Status, search.Step, token);
            expandedCount = search.ExpandedCount;
            pathCost = search.PathCost;
            if (status == SearchStatus.Found)
            {
                nodes = new int[search.PathLength()];
                search.CopyPath(nodes);
            }
            if (options.RecordExpansions)
            {
                expanded = pair.Forward.ExpandedNodes.ToArray();
            }
        }
        else
        {
            pair.Backward.RecordExpansions = options.RecordExpansions;
            var search = new BidirectionalGraphSearch<OptionCost, OptionHeuristic>(graph, cost, heuristic, pair.Forward, pair.Backward);
            search.Begin(start, goal);
            status = Drive(budget, search.Status, search.Step, token);
            expandedCount = search.ExpandedCount;
            pathCost = search.PathCost;
            if (status == SearchStatus.Found)
            {
                nodes = new int[search.PathLength()];
                search.CopyPath(nodes);
            }
            if (options.RecordExpansions)
            {
                ReadOnlySpan<int> f = pair.Forward.ExpandedNodes;
                ReadOnlySpan<int> b = pair.Backward.ExpandedNodes;
                expanded = new int[f.Length + b.Length];
                f.CopyTo(expanded);
                b.CopyTo(expanded.AsSpan(f.Length));
            }
        }
        if (status == SearchStatus.Running)
        {
            return new GraphPath(PathStatus.BudgetExhausted, double.PositiveInfinity, null, expandedCount, expanded, null, algorithm);
        }
        if (status == SearchStatus.NotFound)
        {
            return new GraphPath(PathStatus.NoPath, double.PositiveInfinity, null, expandedCount, expanded, null, algorithm);
        }
        Point3[]? world = null;
        if (positions is not null && nodes is not null)
        {
            world = new Point3[nodes.Length];
            for (int i = 0; i < nodes.Length; i++)
            {
                world[i] = positions[nodes[i]];
            }
        }
        return new GraphPath(PathStatus.Found, pathCost, nodes, expandedCount, expanded, world, algorithm);
    }

    // With a token the search runs in 256-iteration slices so cancellation is observed between them; the result is
    // identical to one unsliced run (any budget sequence is).
    private static SearchStatus Drive(int budget, SearchStatus current, Func<int, SearchStatus> step, CancellationToken token)
    {
        int remaining = budget;
        SearchStatus s = current;
        while (s == SearchStatus.Running && remaining > 0)
        {
            token.ThrowIfCancellationRequested();
            int slice = Math.Min(remaining, 256);
            s = step(slice);
            remaining -= slice;
        }
        return s;
    }
}
