using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>
/// Detailed outcome of a polygon-level query (tier 2): which reference was bad, whether a sliced search is still running.
/// One-call results and crowd agents report the shorter <see cref="PathStatus"/>; <see cref="QueryStatusExtensions.ToPathStatus"/> is the mapping.
/// </summary>
public enum QueryStatus
{
    /// <summary>The end polygon was reached; the corridor is the cheapest under the cost model.</summary>
    Success = 0,

    /// <summary>The end polygon could not be reached (or the node limit was hit); the corridor ends at the visited polygon closest to the goal.</summary>
    Partial = 1,

    /// <summary>A sliced search has not finished.</summary>
    InProgress = 2,

    /// <summary>The start polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidStartPoly = 3,

    /// <summary>The end polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidEndPoly = 4,

    /// <summary>No acceptable polygon near the start position.</summary>
    NoStartNearby = 5,

    /// <summary>No acceptable polygon near the end position.</summary>
    NoEndNearby = 6,

    /// <summary>The corridor passed to a straight-path call is empty, holds invalid references or has polygons that do not touch.</summary>
    InvalidCorridor = 7,
}

/// <summary>Maps the query-level status onto the shared path vocabulary.</summary>
public static class QueryStatusExtensions
{
    /// <summary>
    /// Success is Found; Partial stays Partial; InProgress stays InProgress; a bad or missing start polygon is StartInvalid and a
    /// bad or missing end polygon is GoalInvalid; an unusable corridor is NoPath.
    /// </summary>
    /// <param name="status">The query-level status.</param>
    /// <returns>The shared status.</returns>
    public static PathStatus ToPathStatus(this QueryStatus status) => status switch
    {
        QueryStatus.Success => PathStatus.Found,
        QueryStatus.Partial => PathStatus.Partial,
        QueryStatus.InProgress => PathStatus.InProgress,
        QueryStatus.InvalidStartPoly or QueryStatus.NoStartNearby => PathStatus.StartInvalid,
        QueryStatus.InvalidEndPoly or QueryStatus.NoEndNearby => PathStatus.GoalInvalid,
        _ => PathStatus.NoPath,
    };
}

/// <summary>Result of a nearest-polygon query.</summary>
public readonly struct NearestPoly
{
    internal NearestPoly(PolyRef poly, Vector3D point, bool overPolygon, double distanceSquared)
    {
        Poly = poly;
        Point = point;
        OverPolygon = overPolygon;
        DistanceSquared = distanceSquared;
    }

    /// <summary>The polygon, or <see cref="PolyRef.Invalid"/> if none was found.</summary>
    public PolyRef Poly { get; }

    /// <summary>The closest point of the polygon surface to the query position (world coordinates).</summary>
    public Vector3D Point { get; }

    /// <summary>True if the query position lies inside the polygon's outline seen from above.</summary>
    public bool OverPolygon { get; }

    /// <summary>Squared 3D distance from the query position to <see cref="Point"/>.</summary>
    public double DistanceSquared { get; }
}

/// <summary>Properties of a straight-path point.</summary>
[Flags]
public enum StraightPathMarks : byte
{
    /// <summary>A corner of the path.</summary>
    None = 0,

    /// <summary>The first point.</summary>
    Start = 1,

    /// <summary>The last point.</summary>
    End = 2,

    /// <summary>The point where an off-mesh link is entered; the next point is where it is left.</summary>
    OffMeshConnection = 4,
}

/// <summary>One point of a straight (string-pulled) path.</summary>
public readonly struct StraightPathPoint : IEquatable<StraightPathPoint>
{
    internal StraightPathPoint(Vector3D position, StraightPathMarks flags, PolyRef poly)
    {
        Position = position;
        Flags = flags;
        Poly = poly;
    }

    /// <summary>World position.</summary>
    public Vector3D Position { get; }

    /// <summary>Start, end and off-mesh markers.</summary>
    public StraightPathMarks Flags { get; }

    /// <summary>The corridor polygon the path is in after this point (for the last point, the end polygon).</summary>
    public PolyRef Poly { get; }

    /// <inheritdoc />
    public bool Equals(StraightPathPoint other) => Position == other.Position && Flags == other.Flags && Poly == other.Poly;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is StraightPathPoint p && Equals(p);

    /// <inheritdoc />
    public override int GetHashCode() => (int)Poly.Value;

    /// <summary>Equality.</summary>
    public static bool operator ==(StraightPathPoint a, StraightPathPoint b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(StraightPathPoint a, StraightPathPoint b) => !a.Equals(b);
}

/// <summary>Outcome of a raycast along the surface.</summary>
public enum RaycastStatus
{
    /// <summary>The ray was traced (see <see cref="RaycastResult.Hit"/>).</summary>
    Ok = 0,

    /// <summary>The start polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidStartPoly = 1,

    /// <summary>The start position is not inside the start polygon (more than 1e-6 outside).</summary>
    StartOutsidePolygon = 2,
}

/// <summary>Result of <see cref="NavMeshQuery.Raycast{TFilter}"/>.</summary>
public readonly struct RaycastResult
{
    internal RaycastResult(RaycastStatus status, bool hit, double t, Vector3D normal, PolyRef hitPoly, int hitEdge, bool truncated)
    {
        Status = status;
        Hit = hit;
        T = t;
        Normal = normal;
        HitPoly = hitPoly;
        HitEdge = hitEdge;
        Truncated = truncated;
    }

    /// <summary>Whether the ray was traced at all.</summary>
    public RaycastStatus Status { get; }

    /// <summary>True if a wall (boundary edge, unusable neighbour) stopped the ray before its end.</summary>
    public bool Hit { get; }

    /// <summary>Fraction of the segment travelled: where the wall was hit, or 1 when nothing was.</summary>
    public double T { get; }

    /// <summary>Outward unit normal (in the X/Z plane, Y = 0) of the wall that was hit; zero if none.</summary>
    public Vector3D Normal { get; }

    /// <summary>The polygon whose edge was hit (the last polygon visited).</summary>
    public PolyRef HitPoly { get; }

    /// <summary>Index of the hit edge within <see cref="HitPoly"/>, or -1.</summary>
    public int HitEdge { get; }

    /// <summary>True if the visited-polygon buffer was too small (the trace itself is still complete).</summary>
    public bool Truncated { get; }
}

/// <summary>Options of the one-call path query (tier 1).</summary>
public sealed class PathOptions
{
    /// <summary>Which polygons may be used and what they cost; null means everything at cost 1.</summary>
    public QueryFilter? Filter { get; set; }

    /// <summary>Half extents of the box around the start and end positions that is searched for the nearest polygon. Default (2, 4, 2).</summary>
    public Vector3D SearchExtents { get; set; } = new(2, 4, 2);

    /// <summary>Largest corridor length returned; longer corridors are cut. Default 4096.</summary>
    public int MaxCorridor { get; set; } = 4096;

    /// <summary>Largest number of straight-path points. Default 4096.</summary>
    public int MaxPoints { get; set; } = 4096;
}

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

/// <summary>What one node of the polygon path search stands for.</summary>
public enum PathNodeModel
{
    /// <summary>One node per crossing (leaving a polygon through one edge or link). Exact for the midpoint-crossing cost model; about three times the nodes of <see cref="Polygon"/>, so slower.</summary>
    Crossing = 0,

    /// <summary>One node per polygon, placed at the edge midpoint where the cheapest route found so far enters it. The default: fewer nodes, so faster; the corridor may differ slightly from the crossing model's.</summary>
    Polygon = 1,
}
