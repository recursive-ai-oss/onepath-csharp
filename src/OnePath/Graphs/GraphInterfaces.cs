using System;

namespace RecursiveAI.OnePath.Graphs;

/// <summary>
/// Prices an edge during a graph search. Implement as a struct so the call inlines. Return the edge's cost
/// (usually <c>weight</c> scaled or offset), or +∞ (or a negative or NaN value) to forbid the edge.
/// A bidirectional search calls this with the edge's true direction (<c>source</c> → <c>target</c>).
/// </summary>
public interface IGraphCost
{
    /// <summary>The cost of travelling the edge.</summary>
    /// <param name="source">Source node.</param>
    /// <param name="target">Target node.</param>
    /// <param name="weight">The stored edge weight.</param>
    double Cost(int source, int target, double weight);
}

/// <summary>
/// A lower bound on the distance between two nodes, used by A*. For optimal paths it must be admissible
/// (never above the true distance) and consistent (<c>Estimate(u, t) ≤ cost(u, v) + Estimate(v, t)</c>);
/// bidirectional A* additionally evaluates <c>Estimate(start, v)</c>, so it must bound distances in both
/// directions. Return +∞ only when the target cannot be reached from the source.
/// </summary>
public interface IGraphHeuristic
{
    /// <summary>A lower bound on the distance from <paramref name="source"/> to <paramref name="target"/>.</summary>
    /// <param name="source">Source node.</param>
    /// <param name="target">Target node.</param>
    double Estimate(int source, int target);
}

/// <summary>The stored weight as the cost.</summary>
public readonly struct DefaultGraphCost : IGraphCost
{
    /// <inheritdoc />
    public readonly double Cost(int source, int target, double weight) => weight;
}

/// <summary>The zero estimate: A* with it is Dijkstra's algorithm.</summary>
public readonly struct ZeroGraphHeuristic : IGraphHeuristic
{
    /// <inheritdoc />
    public readonly double Estimate(int source, int target) => 0.0;
}

/// <summary>Straight-line distance between node positions: consistent when every edge weight is at least the distance between its endpoints.</summary>
public readonly struct EuclideanGraphHeuristic : IGraphHeuristic
{
    private readonly Point3[] _positions;

    /// <summary>Creates the heuristic over <paramref name="positions"/> (indexed by node id).</summary>
    /// <param name="positions">Node positions.</param>
    /// <exception cref="ArgumentNullException"><paramref name="positions"/> is null.</exception>
    public EuclideanGraphHeuristic(Point3[] positions)
    {
        ArgumentNullException.ThrowIfNull(positions);
        _positions = positions;
    }

    /// <inheritdoc />
    public readonly double Estimate(int source, int target) => _positions[source].DistanceTo(_positions[target]);
}

/// <summary>The ALT landmark bound (Goldberg &amp; Harrelson 2005) from a <see cref="Landmarks"/> table.</summary>
public readonly struct AltGraphHeuristic : IGraphHeuristic
{
    private readonly Landmarks _landmarks;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="landmarks">The table; it must have been built for the graph that is searched.</param>
    /// <exception cref="ArgumentNullException"><paramref name="landmarks"/> is null.</exception>
    public AltGraphHeuristic(Landmarks landmarks)
    {
        ArgumentNullException.ThrowIfNull(landmarks);
        _landmarks = landmarks;
    }

    /// <inheritdoc />
    public readonly double Estimate(int source, int target) => _landmarks.LowerBound(source, target);
}
