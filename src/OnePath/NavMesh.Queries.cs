using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>Where a <see cref="NavMesh.Raycast"/> along the mesh surface stopped.</summary>
public readonly struct NavMeshHit
{
    internal NavMeshHit(Vector3D position, Vector3D normal, double distance)
    {
        Position = position;
        Normal = normal;
        Distance = distance;
    }

    /// <summary>The point where the ray met the mesh boundary (world coordinates). When the start was off the mesh, the start itself.</summary>
    public Vector3D Position { get; }

    /// <summary>Outward unit normal of the boundary edge in the X/Z plane (Y = 0); zero when the start was off the mesh.</summary>
    public Vector3D Normal { get; }

    /// <summary>Distance from the (snapped) start to <see cref="Position"/>.</summary>
    public double Distance { get; }
}

public sealed partial class NavMesh
{
    /// <summary>Half extents of the box searched for a polygon around a raycast start: (2, 4, 2), like <see cref="PathOptions.SearchExtents"/>.</summary>
    private static readonly Vector3D s_snapExtents = new(2, 4, 2);

    /// <summary>
    /// Finds the point of the mesh nearest to a position, within a distance: the usual "snap a click to the walkable area" call.
    /// Uses the same pooled query object as <see cref="FindPath(Vector3D, Vector3D, PathOptions?, System.Threading.CancellationToken)"/>.
    /// </summary>
    /// <param name="position">World position (double).</param>
    /// <param name="maxDistance">Largest 3D distance from <paramref name="position"/> to the returned point, 0 or more.</param>
    /// <param name="point">The nearest point on an accepted polygon; default when none is within range.</param>
    /// <param name="filter">Which polygons count; null accepts every polygon.</param>
    /// <returns>True if a point within <paramref name="maxDistance"/> exists.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is non-finite or beyond ±2^21, or <paramref name="maxDistance"/> is negative or not finite.</exception>
    public bool TrySamplePosition(Vector3D position, double maxDistance, out Vector3D point, QueryFilter? filter = null)
    {
        NavMeshQuery.CheckPos(position, nameof(position));
        if (!(maxDistance >= 0 && double.IsFinite(maxDistance)))
        {
            throw new ArgumentOutOfRangeException(nameof(maxDistance), maxDistance, "maxDistance must be finite and not negative.");
        }
        NavMeshQuery q = Tiles.RentQuery();
        try
        {
            NearestPoly n = q.FindNearestPoly(position, new Vector3D(maxDistance, maxDistance, maxDistance), filter);
            if (n.Poly.IsSet && n.DistanceSquared <= maxDistance * maxDistance)
            {
                point = n.Point;
                return true;
            }
        }
        finally
        {
            Tiles.ReturnQuery(q);
        }
        point = default;
        return false;
    }

    /// <summary>
    /// Traces a straight line along the mesh surface from a position toward another and reports where it leaves the walkable area
    /// (a boundary edge or an edge to a polygon the filter rejects). The start is first snapped to the nearest polygon within
    /// (2, 4, 2); a start with none nearby counts as blocked at the start. Heights follow the surface, so the line is
    /// traced in the X/Z plane.
    /// </summary>
    /// <param name="from">Start (world, double).</param>
    /// <param name="to">Target (world).</param>
    /// <param name="hit">Where the ray stopped; default when the way is clear.</param>
    /// <param name="filter">Which polygons may be crossed; null accepts every polygon.</param>
    /// <returns>True if the ray was blocked before reaching <paramref name="to"/> (<paramref name="hit"/> is set); false if the line is clear.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    public bool Raycast(Vector3D from, Vector3D to, out NavMeshHit hit, QueryFilter? filter = null)
    {
        NavMeshQuery.CheckPos(from, nameof(from));
        NavMeshQuery.CheckPos(to, nameof(to));
        NavMeshQuery q = Tiles.RentQuery();
        try
        {
            NearestPoly s = q.FindNearestPoly(from, s_snapExtents, filter);
            if (!s.Poly.IsSet)
            {
                hit = new NavMeshHit(from, default, 0);
                return true;
            }
            RaycastResult r = q.Raycast(s.Poly, s.Point, to, filter, default, out _);
            if (r.Status != RaycastStatus.Ok)
            {
                hit = new NavMeshHit(s.Point, default, 0);
                return true;
            }
            if (!r.Hit)
            {
                hit = default;
                return false;
            }
            var p = new Vector3D(
                s.Point.X + ((to.X - s.Point.X) * r.T),
                s.Point.Y + ((to.Y - s.Point.Y) * r.T),
                s.Point.Z + ((to.Z - s.Point.Z) * r.T));
            hit = new NavMeshHit(p, r.Normal, Vector3D.Distance(s.Point, p));
            return true;
        }
        finally
        {
            Tiles.ReturnQuery(q);
        }
    }

    /// <summary>
    /// A random point on the mesh, uniform by polygon area. The generator state is the only source of randomness, so the same
    /// state gives the same point on every platform; it advances by four draws when a point is found and not at all otherwise.
    /// </summary>
    /// <param name="rng">The seeded generator.</param>
    /// <param name="point">The point; default when no accepted polygon has area.</param>
    /// <param name="filter">Which polygons may be chosen; null accepts every polygon.</param>
    /// <returns>True if a point was found.</returns>
    public bool TryGetRandomPoint(ref SeededRandom rng, out Vector3D point, QueryFilter? filter = null)
    {
        NavMeshQuery q = Tiles.RentQuery();
        try
        {
            RandomPointResult r = q.RandomPoint(ref rng, filter);
            point = r.Found ? r.Point : default;
            return r.Found;
        }
        finally
        {
            Tiles.ReturnQuery(q);
        }
    }
}
