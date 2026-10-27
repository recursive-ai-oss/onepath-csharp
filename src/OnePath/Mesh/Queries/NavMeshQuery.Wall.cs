using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    /// <summary>
    /// The nearest wall to a position among the polygons a circle search of the same radius reaches
    /// (<see cref="BeginAreaSearch{TFilter}(PolyRef, Vector3D, double, in TFilter, AreaSearchKind)"/>). A wall is a polygon edge (or the part of one) with no
    /// walkable polygon on the other side: no neighbour at all, a neighbour the filter rejects, or the stretch of a tile-border
    /// edge not covered by a link to an accepted polygon (link ranges are quantised to 1/255 of the edge). Distances are in the
    /// X/Z plane; only walls within the radius count. Ties go to the lowest (polygon reference, edge index, position along the edge).
    /// </summary>
    /// <param name="startPoly">The polygon the search starts in (normally the one containing <paramref name="center"/>).</param>
    /// <param name="center">The query position.</param>
    /// <param name="radius">Search radius: finite, 0 to 2^22.</param>
    /// <param name="filter">Which polygons count as walkable.</param>
    /// <param name="status">Success, or InvalidStartPoly.</param>
    /// <exception cref="ArgumentOutOfRangeException">The centre is non-finite or beyond ±2^21, or the radius is out of range.</exception>
    public WallHit DistanceToWall<TFilter>(PolyRef startPoly, Vector3D center, double radius, in TFilter filter, out AreaQueryStatus status)
        where TFilter : struct, IQueryFilter
    {
        status = BeginAreaSearch(startPoly, center, radius, in filter);
        if (status != AreaQueryStatus.InProgress)
        {
            return new WallHit(false, radius, default, default, PolyRef.Invalid, -1);
        }
        UpdateAreaSearch(int.MaxValue, in filter);
        status = AreaQueryStatus.Success;
        return FinishDistanceToWall(in filter);
    }

    /// <summary>Distance to the nearest wall with the plain filter (null accepts everything).</summary>
    public WallHit DistanceToWall(PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter, out AreaQueryStatus status)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return DistanceToWall(startPoly, center, radius, in f, out status);
    }

    /// <summary>
    /// Ends a sliced circle search (<see cref="BeginAreaSearch(PolyRef, Vector3D, double, QueryFilter?, AreaSearchKind)"/>) by finding the nearest wall among the
    /// polygons expanded so far. Pass the filter the search used.
    /// </summary>
    /// <exception cref="InvalidOperationException">No circle search has begun.</exception>
    public WallHit FinishDistanceToWall<TFilter>(in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        if (_aState == SearchState.None || !_aIsCircle)
        {
            throw new InvalidOperationException("No circle search has begun.");
        }
        bool found = false;
        double bestD2 = _aR2, bestX = 0, bestY = 0, bestZ = 0;
        ulong bestPoly = 0;
        int bestEdge = -1;
        double bestFrac = 0;
        double bestNx = 0, bestNz = 0;
        for (int i = 0; i < _aOrderCount; i++)
        {
            var poly = new PolyRef(_aPoly[_aOrder[i]]);
            if (!_mesh.TryGetPoly(poly, out NavTile t, out int pi))
            {
                continue;
            }
            ReadOnlySpan<float> v = t.Vertices;
            ReadOnlySpan<ushort> idx = t.PolyIndices;
            int n = t.PolyVertexCounts[pi];
            int b = t.PolyStart[pi];
            ReadOnlySpan<NavLink> links = _mesh.GetLinks(poly);
            for (int e = 0; e < n; e++)
            {
                int nb = t.EdgeNeighbors[b + e];
                if (nb != 0 && filter.Passes(t.PolyFlags[nb - 1], t.PolyAreas[nb - 1]))
                {
                    continue;
                }
                bool border = nb == 0 && t.EdgePortalSides[b + e] != NavMeshFormat.NoSide;
                int a = 3 * idx[b + e], c = 3 * idx[b + ((e + 1) % n)];
                double ax = t.OriginX + (double)v[a], az = t.OriginZ + (double)v[a + 2];
                double cx = t.OriginX + (double)v[c], cz = t.OriginZ + (double)v[c + 2];
                // Walk the wall pieces of the edge as fractions [f0, f1] of its length; a border edge is cut by its links.
                double f0 = 0;
                while (f0 < 1.0)
                {
                    double f1 = 1.0;
                    if (border)
                    {
                        // Skip covered ranges; the gap then runs to the next covered start.
                        bool moved = true;
                        while (moved && f0 < 1.0)
                        {
                            moved = false;
                            for (int j = 0; j < links.Length; j++)
                            {
                                NavLink l = links[j];
                                if (LinkCovers(l, e, in filter) && (l.RangeMin / 255.0) <= f0 && (l.RangeMax / 255.0) > f0)
                                {
                                    f0 = l.RangeMax / 255.0;
                                    moved = true;
                                }
                            }
                        }
                        if (f0 >= 1.0)
                        {
                            break;
                        }
                        for (int j = 0; j < links.Length; j++)
                        {
                            NavLink l = links[j];
                            double ls = l.RangeMin / 255.0;
                            if (LinkCovers(l, e, in filter) && ls > f0 && ls < f1)
                            {
                                f1 = ls;
                            }
                        }
                    }
                    Lerp(ax, az, cx, cz, f0, out double px, out double pz);
                    Lerp(ax, az, cx, cz, f1, out double rx, out double rz);
                    double ex = rx - px, ez = rz - pz;
                    double len2 = (ex * ex) + (ez * ez);
                    double u = len2 > 0 ? Math.Clamp((((_aCx - px) * ex) + ((_aCz - pz) * ez)) / len2, 0.0, 1.0) : 0.0;
                    double qx = px + (ex * u), qz = pz + (ez * u);
                    double d2 = ((_aCx - qx) * (_aCx - qx)) + ((_aCz - qz) * (_aCz - qz));
                    double frac = f0 + ((f1 - f0) * u);
                    bool take = d2 <= bestD2 && (!found || d2 < bestD2
                        || poly.Value < bestPoly
                        || (poly.Value == bestPoly && (e < bestEdge || (e == bestEdge && frac < bestFrac))));
                    if (take)
                    {
                        found = true;
                        bestD2 = d2;
                        bestPoly = poly.Value;
                        bestEdge = e;
                        bestFrac = frac;
                        bestX = qx;
                        bestZ = qz;
                        bestY = t.OriginY + v[a + 1] + ((v[c + 1] - (double)v[a + 1]) * frac);
                        double elx = cx - ax, elz = cz - az;
                        double el = Math.Sqrt((elx * elx) + (elz * elz));
                        bestNx = el > 0 ? -elz / el : 0; // inward normal of a counter-clockwise polygon
                        bestNz = el > 0 ? elx / el : 0;
                    }
                    f0 = f1;
                    if (!border)
                    {
                        break;
                    }
                }
            }
        }
        if (!found)
        {
            return new WallHit(false, _aRadius, default, default, PolyRef.Invalid, -1);
        }
        double dist = Math.Sqrt(bestD2);
        Vector3D normal = dist > 0 ? new Vector3D((_aCx - bestX) / dist, 0, (_aCz - bestZ) / dist) : new Vector3D(bestNx, 0, bestNz);
        return new WallHit(true, dist, new Vector3D(bestX, bestY, bestZ), normal, new PolyRef(bestPoly), bestEdge);
    }

    /// <summary>Ends a sliced circle search begun with a plain filter by finding the nearest wall.</summary>
    /// <exception cref="InvalidOperationException">No circle search has begun, or it was started with a struct filter.</exception>
    public WallHit FinishDistanceToWall()
    {
        if (!_aPlainStored)
        {
            throw new InvalidOperationException("The search was started with a struct filter; pass it to FinishDistanceToWall<TFilter>.");
        }
        PlainFilter f = _aPlain;
        return FinishDistanceToWall(in f);
    }

    // A tile-border link on this edge to a polygon the filter accepts.
    private bool LinkCovers<TFilter>(NavLink l, int edge, in TFilter filter)
        where TFilter : struct, IQueryFilter =>
        l.Kind == NavLinkKind.Portal && l.Index == edge
        && _mesh.TryGetPoly(l.Target, out NavTile qt, out int qi) && filter.Passes(qt.PolyFlags[qi], qt.PolyAreas[qi]);
}
