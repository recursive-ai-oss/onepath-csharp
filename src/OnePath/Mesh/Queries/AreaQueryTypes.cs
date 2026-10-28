using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>Outcome of a polygon-set query (circle, shape, local neighbourhood) or of beginning one.</summary>
public enum AreaQueryStatus
{
    /// <summary>No query has produced a result: the value of <c>default(AreaQueryStatus)</c>.</summary>
    None = 0,

    /// <summary>The search finished; the result lists every polygon reached.</summary>
    Success = 1,

    /// <summary>A sliced search has not finished; finishing it now lists the polygons expanded so far.</summary>
    InProgress = 2,

    /// <summary>The start polygon reference is invalid, stale or rejected by the filter.</summary>
    InvalidStartPoly = 3,

    /// <summary>The shape is not a convex polygon of 3 to 64 vertices with non-zero area (or has a vertex that is not finite).</summary>
    InvalidShape = 4,
}

/// <summary>Which polygons a polygon-set search keeps.</summary>
public enum AreaSearchKind
{
    /// <summary>Every polygon reachable through portals that touch the shape.</summary>
    Reachable = 0,

    /// <summary>
    /// Like <see cref="Reachable"/>, but a polygon whose outline overlaps (in X/Z, interiors intersecting) a polygon already
    /// kept is skipped, so stacked floors do not both appear: what a character standing at the centre can step onto locally.
    /// </summary>
    LocalNeighbourhood = 1,
}

/// <summary>Result of a random-point query.</summary>
public readonly struct RandomPointResult
{
    internal RandomPointResult(PolyRef poly, Vector3D point)
    {
        Poly = poly;
        Point = point;
    }

    /// <summary>True if a point was found.</summary>
    public bool Found => Poly.IsSet;

    /// <summary>The polygon the point is on, or <see cref="PolyRef.Invalid"/> if none was found.</summary>
    public PolyRef Poly { get; }

    /// <summary>The point on the polygon surface (world coordinates); default if none was found.</summary>
    public Vector3D Point { get; }
}

/// <summary>Result of a distance-to-wall query.</summary>
public readonly struct WallHit
{
    internal WallHit(bool found, double distance, Vector3D point, Vector3D normal, PolyRef poly, int edge)
    {
        Found = found;
        Distance = distance;
        Point = point;
        Normal = normal;
        Poly = poly;
        Edge = edge;
    }

    /// <summary>True if a wall lies within the radius.</summary>
    public bool Found { get; }

    /// <summary>Distance in the X/Z plane from the query position to <see cref="Point"/>; the search radius if nothing was found.</summary>
    public double Distance { get; }

    /// <summary>The nearest wall point (world coordinates, height interpolated along the edge); default if none.</summary>
    public Vector3D Point { get; }

    /// <summary>
    /// Unit vector (Y = 0) from the wall point toward the query position; if the position is on the wall, the inward normal
    /// of the wall's edge. Zero if none was found.
    /// </summary>
    public Vector3D Normal { get; }

    /// <summary>The polygon owning the wall edge, or <see cref="PolyRef.Invalid"/>.</summary>
    public PolyRef Poly { get; }

    /// <summary>Index of the wall edge within <see cref="Poly"/>, or -1.</summary>
    public int Edge { get; }
}

/// <summary>Result of moving along the surface.</summary>
public readonly struct MoveResult
{
    internal MoveResult(RaycastStatus status, Vector3D position, PolyRef poly, bool blocked, bool truncated)
    {
        Status = status;
        Position = position;
        Poly = poly;
        Blocked = blocked;
        Truncated = truncated;
    }

    /// <summary>Ok, or why the move could not start (invalid start polygon, start outside it).</summary>
    public RaycastStatus Status { get; }

    /// <summary>Where the mover ends up (world coordinates, on the surface of <see cref="Poly"/>); the start position on failure.</summary>
    public Vector3D Position { get; }

    /// <summary>The polygon containing <see cref="Position"/>.</summary>
    public PolyRef Poly { get; }

    /// <summary>True if a wall stopped the straight move and the result slid along it.</summary>
    public bool Blocked { get; }

    /// <summary>True if the visited-polygon buffer was too small.</summary>
    public bool Truncated { get; }
}
