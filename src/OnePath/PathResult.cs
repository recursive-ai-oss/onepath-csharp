using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>Result of the one-call path query.</summary>
public sealed class PathResult
{
    internal PathResult(QueryStatus detail, StraightPathPoint[] points, PolyRef[] corridor, double cost, bool truncated)
    {
        Detail = detail;
        Points = points;
        Corridor = corridor;
        Cost = cost;
        Truncated = truncated;
    }

    /// <summary>Found, Partial (ends at the closest reachable point), or the reason the query failed.</summary>
    public PathStatus Status => Detail.ToPathStatus();

    /// <summary>The detailed query-level status behind <see cref="Status"/> (which reference was bad, and so on).</summary>
    public QueryStatus Detail { get; }

    /// <summary>True for Found and Partial: <see cref="Points"/> holds a path.</summary>
    public bool HasPath => Detail is QueryStatus.Success or QueryStatus.Partial;

    /// <summary>True when the path was found to the goal.</summary>
    public bool Found => Detail == QueryStatus.Success;

    /// <summary>True when the goal is unreachable and the path ends at the closest reachable point.</summary>
    public bool IsPartial => Detail == QueryStatus.Partial;

    /// <summary>The straight path, start to end (empty on failure).</summary>
    public IReadOnlyList<StraightPathPoint> Points { get; }

    /// <summary>The polygon corridor the straight path was pulled through.</summary>
    public IReadOnlyList<PolyRef> Corridor { get; }

    /// <summary>The search cost of the corridor under the filter's cost model.</summary>
    public double Cost { get; }

    /// <summary>True if the corridor or the point list was cut to fit its limit.</summary>
    public bool Truncated { get; }

    /// <summary>Total length of the straight path in world units (0 when there is no path).</summary>
    public double Length
    {
        get
        {
            double l = 0;
            for (int i = 1; i < Points.Count; i++)
            {
                l += Vector3D.Distance(Points[i - 1].Position, Points[i].Position);
            }
            return l;
        }
    }
}
