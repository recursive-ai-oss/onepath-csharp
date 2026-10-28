using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>A piece of the mesh boundary: a stretch of a polygon edge with no accepted walkable polygon beyond it.</summary>
public readonly struct WallSegment
{
    /// <summary>Creates a wall segment.</summary>
    public WallSegment(double ax, double ay, double az, double bx, double by, double bz, PolyRef poly, int edge)
    {
        AX = ax;
        AY = ay;
        AZ = az;
        BX = bx;
        BY = by;
        BZ = bz;
        Poly = poly;
        Edge = edge;
    }

    /// <summary>X of the first end (world).</summary>
    public double AX { get; }

    /// <summary>Height of the first end (world Y).</summary>
    public double AY { get; }

    /// <summary>Z of the first end (world).</summary>
    public double AZ { get; }

    /// <summary>X of the second end (world).</summary>
    public double BX { get; }

    /// <summary>Height of the second end (world Y).</summary>
    public double BY { get; }

    /// <summary>Z of the second end (world).</summary>
    public double BZ { get; }

    /// <summary>The polygon owning the edge.</summary>
    public PolyRef Poly { get; }

    /// <summary>Index of the edge in its polygon.</summary>
    public int Edge { get; }
}

public sealed partial class NavMeshQuery
{
    /// <summary>
    /// Lists the wall pieces within a radius of a position, among the polygons a circle search of the same radius reaches
    /// (see <see cref="DistanceToWall(PolyRef, Vector3D, double, QueryFilter?, out AreaQueryStatus)"/> for what a wall is). Pieces
    /// are ordered by (distance to the centre, polygon reference, edge, position along the edge) so the result is the same
    /// everywhere; when there are more than the buffer holds, the nearest are kept. Distances are in the X/Z plane.
    /// </summary>
    /// <param name="startPoly">The polygon the search starts in.</param>
    /// <param name="center">The query position.</param>
    /// <param name="radius">Search radius: finite, 0 to 2^22.</param>
    /// <param name="filter">Which polygons count as walkable.</param>
    /// <param name="walls">Receives the pieces, nearest first.</param>
    /// <param name="count">Number written.</param>
    /// <param name="truncated">True if more pieces were found than <paramref name="walls"/> holds.</param>
    /// <returns>Success, or InvalidStartPoly (nothing written).</returns>
    /// <exception cref="ArgumentOutOfRangeException">The centre is non-finite or beyond ±2^21, or the radius is out of range.</exception>
    public AreaQueryStatus FindWalls<TFilter>(PolyRef startPoly, Vector3D center, double radius, in TFilter filter, Span<WallSegment> walls, out int count, out bool truncated)
        where TFilter : struct, IQueryFilter
    {
        count = 0;
        truncated = false;
        AreaQueryStatus status = BeginAreaSearch(startPoly, center, radius, in filter);
        if (status != AreaQueryStatus.InProgress)
        {
            return status;
        }
        UpdateAreaSearch(int.MaxValue, in filter);
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
                // Wall pieces of the edge as fractions [f0, f1] of its length; a border edge is cut by its links.
                double f0 = 0;
                while (f0 < 1.0)
                {
                    double f1 = 1.0;
                    if (border)
                    {
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
                    double d2 = SegDist2(_aCx, _aCz, px, pz, rx, rz);
                    if (d2 <= _aR2)
                    {
                        double py = t.OriginY + v[a + 1] + ((v[c + 1] - (double)v[a + 1]) * f0);
                        double ry = t.OriginY + v[a + 1] + ((v[c + 1] - (double)v[a + 1]) * f1);
                        Insert(walls, _aCx, _aCz, ref count, ref truncated, d2, new WallSegment(px, py, pz, rx, ry, rz, poly, e));
                    }
                    f0 = f1;
                    if (!border)
                    {
                        break;
                    }
                }
            }
        }
        return AreaQueryStatus.Success;
    }

    /// <summary>Lists wall pieces with the plain filter (null accepts everything).</summary>
    public AreaQueryStatus FindWalls(PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter, Span<WallSegment> walls, out int count, out bool truncated)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return FindWalls(startPoly, center, radius, in f, walls, out count, out truncated);
    }

    // Keeps the buffer sorted by distance; a later piece at equal distance goes after the earlier ones (pieces are met in a
    // deterministic order). Distances of stored pieces are recomputed with the same expression, so they equal the keys.
    private static void Insert(Span<WallSegment> walls, double cx, double cz, ref int count, ref bool truncated, double d2, WallSegment w)
    {
        int pos = count;
        if (pos == walls.Length)
        {
            truncated = true;
            if (pos == 0 || !(d2 < Dist2(walls[pos - 1], cx, cz)))
            {
                return;
            }
            pos--;
        }
        else
        {
            count++;
        }
        while (pos > 0 && d2 < Dist2(walls[pos - 1], cx, cz))
        {
            walls[pos] = walls[pos - 1];
            pos--;
        }
        walls[pos] = w;
    }

    private static double Dist2(WallSegment w, double cx, double cz) => SegDist2(cx, cz, w.AX, w.AZ, w.BX, w.BZ);
}
