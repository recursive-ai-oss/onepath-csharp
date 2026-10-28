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
    /// <summary>No query has produced a result: the value of <c>default(QueryStatus)</c>, so an unassigned status never reads as success.</summary>
    None = 0,

    /// <summary>The end polygon was reached; the corridor is the cheapest under the cost model.</summary>
    Success = 1,

    /// <summary>The end polygon could not be reached (or the node limit was hit); the corridor ends at the visited polygon closest to the goal.</summary>
    Partial = 2,

    /// <summary>A sliced search has not finished.</summary>
    InProgress = 3,

    /// <summary>The start polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidStartPoly = 4,

    /// <summary>The end polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidEndPoly = 5,

    /// <summary>No acceptable polygon near the start position.</summary>
    NoStartNearby = 6,

    /// <summary>No acceptable polygon near the end position.</summary>
    NoEndNearby = 7,

    /// <summary>The corridor passed to a straight-path call is empty, holds invalid references or has polygons that do not touch.</summary>
    InvalidCorridor = 8,
}

/// <summary>Maps the query-level status onto the shared path vocabulary.</summary>
public static class QueryStatusExtensions
{
    /// <summary>
    /// Success is Found; Partial stays Partial; InProgress stays InProgress; a bad or missing start polygon is StartInvalid and a
    /// bad or missing end polygon is GoalInvalid; an unusable corridor is NoPath; None stays None.
    /// </summary>
    /// <param name="status">The query-level status.</param>
    /// <returns>The shared status.</returns>
    public static PathStatus ToPathStatus(this QueryStatus status) => status switch
    {
        QueryStatus.None => PathStatus.None,
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

/// <summary>Outcome of a raycast along the surface.</summary>
public enum RaycastStatus
{
    /// <summary>No raycast has produced a result: the value of <c>default(RaycastStatus)</c>.</summary>
    None = 0,

    /// <summary>The ray was traced (see <see cref="RaycastResult.Hit"/>).</summary>
    Ok = 1,

    /// <summary>The start polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidStartPoly = 2,

    /// <summary>The start position is not inside the start polygon (more than 1e-6 outside).</summary>
    StartOutsidePolygon = 3,
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
