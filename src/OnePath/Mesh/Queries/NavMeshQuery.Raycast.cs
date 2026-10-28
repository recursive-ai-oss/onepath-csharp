using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    /// <summary>
    /// Casts a ray along the walkable surface from a point in a polygon toward an end position, in the X/Z plane (heights
    /// follow the surface and are not tested). It leaves each polygon through the edge where the segment exits (Cyrus and Beck
    /// clipping against the convex outline) and continues into the neighbour if there is one the filter accepts, through an
    /// in-tile edge or a tile-portal link that covers the crossing point; otherwise the ray hits a wall there. Off-mesh links
    /// are not crossed. When it passes through a vertex (or along collinear edges) it goes on through whichever candidate edge leads to an accepted polygon, else the lowest edge index is the wall.
    /// </summary>
    /// <param name="startPoly">The polygon containing <paramref name="start"/>.</param>
    /// <param name="start">Start of the segment (within 1e-6 of the polygon outline).</param>
    /// <param name="end">End of the segment.</param>
    /// <param name="filter">Which neighbours may be entered.</param>
    /// <param name="visited">Receives the polygons visited, start first (may be empty).</param>
    /// <param name="visitedCount">Number written to <paramref name="visited"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public RaycastResult Raycast<TFilter>(PolyRef startPoly, Vector3D start, Vector3D end, in TFilter filter, Span<PolyRef> visited, out int visitedCount)
        where TFilter : struct, IQueryFilter => RaycastCore(startPoly, start, end, in filter, visited, out visitedCount, out _);

    // Which edge the segment leaves the polygon through. Edges whose line the segment leaves at the same parameter (equal up to
    // rounding) are candidates: consecutive collinear edges (a vertex in the middle of a side) and the two edges at a vertex the
    // segment passes through. Of those, only the ones whose extent holds the crossing point count, and among them the first
    // that leads to a polygon the filter accepts wins, so a segment that slips through a vertex into the next polygon
    // continues; if none leads anywhere the lowest index is the wall that was hit.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private PolyRef PickExitEdge<TFilter>(PolyRef cur, NavTile t, int p, int b, int n, double sx, double sz, double dx, double dz, double tCross, double tmax, int first, in TFilter filter, out int edge)
        where TFilter : struct, IQueryFilter
    {
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        double px = sx + (dx * tCross), pz = sz + (dz * tCross);
        double tol = 1e-9 * (1.0 + Math.Abs(tmax));
        double minD = double.PositiveInfinity;
        for (int pass = 0; pass < 2; pass++)
        {
            for (int e = 0; e < n; e++)
            {
                int a = idx[b + e], c = idx[b + ((e + 1) % n)];
                double ax = v[3 * a], az = v[(3 * a) + 2];
                double ex = v[3 * c] - ax, ez = v[(3 * c) + 2] - az;
                double denom = (ez * dx) - (ex * dz);
                if (denom <= 0 || (((ez * (ax - sx)) - (ex * (az - sz))) / denom) > tmax + tol)
                {
                    continue;
                }
                double len2 = (ex * ex) + (ez * ez);
                double u = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0.0, 1.0) : 0.0;
                double qx = ax + (ex * u) - px, qz = az + (ez * u) - pz;
                double d = (qx * qx) + (qz * qz);
                if (pass == 0)
                {
                    minD = Math.Min(minD, d);
                }
                else if (d <= minD + 1e-18)
                {
                    PolyRef next = CrossEdge(cur, t, p, e, t.OriginX + px, t.OriginZ + pz, in filter);
                    if (next.IsSet)
                    {
                        edge = e;
                        return next;
                    }
                }
            }
        }
        // Nothing leads on: the wall hit is the first candidate edge in index order.
        edge = first;
        for (int e = 0; e < n; e++)
        {
            int a = idx[b + e], c = idx[b + ((e + 1) % n)];
            double ax = v[3 * a], az = v[(3 * a) + 2];
            double ex = v[3 * c] - ax, ez = v[(3 * c) + 2] - az;
            double denom = (ez * dx) - (ex * dz);
            if (denom <= 0 || (((ez * (ax - sx)) - (ex * (az - sz))) / denom) > tmax + tol)
            {
                continue;
            }
            double len2 = (ex * ex) + (ez * ez);
            double u = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0.0, 1.0) : 0.0;
            double qx = ax + (ex * u) - px, qz = az + (ez * u) - pz;
            if ((qx * qx) + (qz * qz) <= minD + 1e-18)
            {
                edge = e;
                break;
            }
        }
        return PolyRef.Invalid;
    }

    // The raycast proper; also reports the last polygon the ray was in (the visited buffer may be too short to tell).
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private RaycastResult RaycastCore<TFilter>(PolyRef startPoly, Vector3D start, Vector3D end, in TFilter filter, Span<PolyRef> visited, out int visitedCount, out PolyRef last)
        where TFilter : struct, IQueryFilter
    {
        last = startPoly;
        CheckPos(start, nameof(start));
        CheckPos(end, nameof(end));
        visitedCount = 0;
        if (!_mesh.TryGetPoly(startPoly, out NavTile t, out int p) || !filter.Passes(t.PolyFlags[p], t.PolyAreas[p]))
        {
            return new RaycastResult(RaycastStatus.InvalidStartPoly, false, 1.0, default, PolyRef.Invalid, -1, false);
        }
        if (!PolyGeometry.Contains(t, p, start.X - t.OriginX, start.Z - t.OriginZ, 1e-6))
        {
            return new RaycastResult(RaycastStatus.StartOutsidePolygon, false, 1.0, default, PolyRef.Invalid, -1, false);
        }
        double dx = end.X - start.X, dz = end.Z - start.Z;
        PolyRef cur = startPoly;
        double tmin = 0;
        bool truncated = false;
        // Every step moves into a different polygon along a straight segment, so the walk is bounded by the number of polygons
        // it can cross; the cap only guards against rounding cycles at vertices.
        int cap = 1 << 20;
        for (int step = 0; step < cap; step++)
        {
            last = cur;
            if (visitedCount < visited.Length)
            {
                visited[visitedCount++] = cur;
            }
            else
            {
                truncated = true;
            }
            ReadOnlySpan<float> v = t.Vertices;
            ReadOnlySpan<ushort> idx = t.PolyIndices;
            int n = t.PolyVertexCounts[p];
            int b = t.PolyStart[p];
            double sx = start.X - t.OriginX, sz = start.Z - t.OriginZ;
            double tmax = double.PositiveInfinity;
            int edge = -1;
            for (int e = 0; e < n; e++)
            {
                int a = idx[b + e], c = idx[b + ((e + 1) % n)];
                double ax = v[3 * a], az = v[(3 * a) + 2];
                double ex = v[3 * c] - ax, ez = v[(3 * c) + 2] - az;
                double nx = ez, nz = -ex; // outward normal of a counter-clockwise polygon
                double denom = (nx * dx) + (nz * dz);
                if (denom <= 0)
                {
                    continue;
                }
                double tt = ((nx * (ax - sx)) + (nz * (az - sz))) / denom;
                if (tt < tmax)
                {
                    tmax = tt;
                    edge = e;
                }
            }
            if (edge < 0 || tmax >= 1.0)
            {
                return new RaycastResult(RaycastStatus.Ok, false, 1.0, default, PolyRef.Invalid, -1, truncated);
            }
            if (tmax < tmin)
            {
                tmax = tmin;
            }
            PolyRef next = PickExitEdge(cur, t, p, b, n, sx, sz, dx, dz, tmax, tmax, edge, in filter, out edge);
            if (!next.IsSet)
            {
                int a = idx[b + edge], c = idx[b + ((edge + 1) % n)];
                double ex = v[3 * c] - (double)v[3 * a], ez = v[(3 * c) + 2] - (double)v[(3 * a) + 2];
                double len = Math.Sqrt((ex * ex) + (ez * ez));
                return new RaycastResult(RaycastStatus.Ok, true, tmax, new Vector3D(ez / len, 0, -ex / len), cur, edge, truncated);
            }
            cur = next;
            _mesh.TryGetPoly(cur, out t, out p);
            tmin = tmax;
        }
        return new RaycastResult(RaycastStatus.Ok, false, 1.0, default, PolyRef.Invalid, -1, truncated);
    }

    /// <summary>Raycast with the plain filter (null accepts everything).</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public RaycastResult Raycast(PolyRef startPoly, Vector3D start, Vector3D end, QueryFilter? filter, Span<PolyRef> visited, out int visitedCount)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return Raycast(startPoly, start, end, in f, visited, out visitedCount);
    }

    // The polygon on the other side of an edge at world (x, z), if the filter allows entering it.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private PolyRef CrossEdge<TFilter>(PolyRef cur, NavTile t, int p, int edge, double x, double z, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        int b = t.PolyStart[p];
        int nb = t.EdgeNeighbors[b + edge];
        if (nb != 0)
        {
            int q = nb - 1;
            return filter.Passes(t.PolyFlags[q], t.PolyAreas[q]) ? new PolyRef(cur.Salt, cur.TileSlot, q) : PolyRef.Invalid;
        }
        if (t.EdgePortalSides[b + edge] == NavMeshFormat.NoSide)
        {
            return PolyRef.Invalid;
        }
        int n = t.PolyVertexCounts[p];
        ReadOnlySpan<float> v = t.Vertices;
        int a = 3 * t.PolyIndices[b + edge], c = 3 * t.PolyIndices[b + ((edge + 1) % n)];
        double ex = v[c] - (double)v[a], ez = v[c + 2] - (double)v[a + 2];
        double len2 = (ex * ex) + (ez * ez);
        double u = len2 > 0 ? ((((x - t.OriginX) - v[a]) * ex) + (((z - t.OriginZ) - v[a + 2]) * ez)) / len2 : 0.0;
        double pos = u * 255.0;
        ReadOnlySpan<NavLink> links = _mesh.GetLinks(cur);
        for (int j = 0; j < links.Length; j++)
        {
            NavLink l = links[j];
            if (l.Kind == NavLinkKind.Portal && l.Index == edge && pos >= l.RangeMin - 1 && pos <= l.RangeMax + 1
                && _mesh.TryGetPoly(l.Target, out NavTile qt, out int qi) && filter.Passes(qt.PolyFlags[qi], qt.PolyAreas[qi]))
            {
                return l.Target;
            }
        }
        return PolyRef.Invalid;
    }
}
