using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    private double[] _clipA = new double[64];
    private double[] _clipB = new double[64];

    /// <summary>Most attempts <see cref="RandomPointAround{TFilter}"/> makes before giving up.</summary>
    public const int MaxRandomAttempts = 64;

    /// <summary>
    /// A random point on the mesh, uniform by area over the polygons the filter accepts (area = outline area in the X/Z plane).
    /// Polygons are taken in (tile slot, polygon index) order; the generator is advanced by exactly four draws when a point is
    /// found (polygon, fan triangle, then the two barycentric draws) and not at all when no polygon qualifies.
    /// </summary>
    /// <param name="rng">The generator; its state is the only source of randomness, so the same state gives the same point everywhere.</param>
    /// <param name="filter">Which polygons may be chosen.</param>
    /// <returns>The point; not <see cref="RandomPointResult.Found"/> if no accepted polygon has positive area.</returns>
    public RandomPointResult RandomPoint<TFilter>(ref SeededRandom rng, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        double total = 0;
        for (int s = 0; s < _mesh.SlotCount; s++)
        {
            NavTile? t = _mesh.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                if (filter.Passes(t.PolyFlags[p], t.PolyAreas[p]))
                {
                    total += PolyAreaXZ(t, p);
                }
            }
        }
        if (!(total > 0))
        {
            return default;
        }
        double target = rng.NextDouble() * total;
        double acc = 0;
        NavTile? pick = null;
        int pickPoly = -1, pickSlot = -1;
        for (int s = 0; s < _mesh.SlotCount; s++)
        {
            NavTile? t = _mesh.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                if (!filter.Passes(t.PolyFlags[p], t.PolyAreas[p]))
                {
                    continue;
                }
                double a = PolyAreaXZ(t, p);
                if (!(a > 0))
                {
                    continue;
                }
                acc += a;
                pick = t;
                pickPoly = p;
                pickSlot = s;
                if (target < acc)
                {
                    goto chosen; // the last positive-area polygon stays chosen if rounding leaves target at the total
                }
            }
        }
    chosen:
        if (pick is null)
        {
            return default;
        }
        double ox = pick.OriginX, oz = pick.OriginZ;
        int n = pick.PolyVertexCounts[pickPoly];
        EnsureClip(n);
        FillWorldPoly(pick, pickPoly, _clipA);
        SamplePolygon(_clipA, n, ref rng, out double x, out double z);
        double y = pick.OriginY + PolyGeometry.HeightAt(pick, pickPoly, x - ox, z - oz);
        return new RandomPointResult(_mesh.GetSlotRef(pickSlot).Poly(pickPoly), new Vector3D(x, y, z));
    }

    /// <summary>Random point with the plain filter (null accepts everything).</summary>
    public RandomPointResult RandomPoint(ref SeededRandom rng, QueryFilter? filter = null)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return RandomPoint(ref rng, in f);
    }

    /// <summary>
    /// A random point within a radius of a position, on polygons reachable from the start polygon (the circle search of
    /// <see cref="BeginAreaSearch{TFilter}(PolyRef, Vector3D, double, in TFilter, AreaSearchKind)"/>). It is uniform by area over the part of those
    /// polygons inside the circle in the X/Z plane (stacked polygons each count): polygons are weighted by the area of their
    /// intersection with the circle's bounding square, a point is drawn uniformly in that part and kept if it lies inside the
    /// circle. Up to <see cref="MaxRandomAttempts"/> attempts of four draws each are made, in expansion order of the search;
    /// the generator advances by exactly the draws used.
    /// </summary>
    /// <returns>The point; not Found if the start polygon is invalid, the radius is 0, or every attempt missed the circle (rare).</returns>
    /// <exception cref="ArgumentOutOfRangeException">The centre is non-finite or beyond ±2^21, or the radius is out of range.</exception>
    public RandomPointResult RandomPointAround<TFilter>(ref SeededRandom rng, PolyRef startPoly, Vector3D center, double radius, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        AreaQueryStatus s = BeginAreaSearch(startPoly, center, radius, in filter);
        if (s != AreaQueryStatus.InProgress)
        {
            return default;
        }
        UpdateAreaSearch(int.MaxValue, in filter);
        return FinishRandomPointAround(ref rng);
    }

    /// <summary>Random point around a position with the plain filter.</summary>
    public RandomPointResult RandomPointAround(ref SeededRandom rng, PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter = null)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return RandomPointAround(ref rng, startPoly, center, radius, in f);
    }

    /// <summary>Ends a sliced circle search by drawing the random point from the polygons expanded so far.</summary>
    /// <exception cref="InvalidOperationException">No circle search has begun.</exception>
    public RandomPointResult FinishRandomPointAround(ref SeededRandom rng)
    {
        if (_aState == SearchState.None || !_aIsCircle)
        {
            throw new InvalidOperationException("No circle search has begun.");
        }
        double x0 = _aCx - _aRadius, x1 = _aCx + _aRadius, z0 = _aCz - _aRadius, z1 = _aCz + _aRadius;
        double total = 0;
        for (int i = 0; i < _aOrderCount; i++)
        {
            double a = 0;
            if (_mesh.TryGetPoly(new PolyRef(_aPoly[_aOrder[i]]), out NavTile t, out int p))
            {
                int m = ClipPolyToBox(t, p, x0, z0, x1, z1, out _);
                a = m >= 3 ? FanArea(_clipA, m) : 0.0;
            }
            _aArea[i] = a;
            total += a;
        }
        if (!(total > 0))
        {
            return default;
        }
        for (int attempt = 0; attempt < MaxRandomAttempts; attempt++)
        {
            double target = rng.NextDouble() * total;
            double acc = 0;
            int pick = -1;
            for (int i = 0; i < _aOrderCount; i++)
            {
                if (_aArea[i] > 0)
                {
                    acc += _aArea[i];
                    pick = i;
                    if (target < acc)
                    {
                        break;
                    }
                }
            }
            var pr = new PolyRef(_aPoly[_aOrder[pick]]);
            if (!_mesh.TryGetPoly(pr, out NavTile t, out int p))
            {
                continue;
            }
            int m = ClipPolyToBox(t, p, x0, z0, x1, z1, out double[] pts);
            SamplePolygon(pts, m, ref rng, out double x, out double z);
            if (((x - _aCx) * (x - _aCx)) + ((z - _aCz) * (z - _aCz)) <= _aR2)
            {
                double y = t.OriginY + PolyGeometry.HeightAt(t, p, x - t.OriginX, z - t.OriginZ);
                return new RandomPointResult(pr, new Vector3D(x, y, z));
            }
        }
        return default;
    }

    // ---- helpers ----

    private void EnsureClip(int n)
    {
        if (_clipA.Length < 2 * (n + 8))
        {
            _clipA = new double[2 * (n + 8)];
            _clipB = new double[2 * (n + 8)];
        }
    }

    private static double PolyAreaXZ(NavTile t, int p)
    {
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        int i0 = 3 * t.PolyIndices[b];
        double ax = t.Vertices[i0], az = t.Vertices[i0 + 2];
        double sum = 0;
        for (int i = 1; i + 1 < n; i++)
        {
            int i1 = 3 * t.PolyIndices[b + i], i2 = 3 * t.PolyIndices[b + i + 1];
            double ux = t.Vertices[i1] - ax, uz = t.Vertices[i1 + 2] - az;
            double vx = t.Vertices[i2] - ax, vz = t.Vertices[i2 + 2] - az;
            sum += 0.5 * Math.Abs((ux * vz) - (uz * vx));
        }
        return sum;
    }

    private static void FillWorldPoly(NavTile t, int p, double[] dst)
    {
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        for (int i = 0; i < n; i++)
        {
            int v = 3 * t.PolyIndices[b + i];
            dst[2 * i] = t.OriginX + (double)t.Vertices[v];
            dst[(2 * i) + 1] = t.OriginZ + (double)t.Vertices[v + 2];
        }
    }

    // Area of the fan (v0, vi, vi+1) of a convex polygon stored as x,z pairs.
    private static double FanArea(double[] pts, int n)
    {
        double sum = 0;
        for (int i = 1; i + 1 < n; i++)
        {
            sum += TriArea(pts, 0, i, i + 1);
        }
        return sum;
    }

    private static double TriArea(double[] pts, int a, int b, int c)
    {
        double ux = pts[2 * b] - pts[2 * a], uz = pts[(2 * b) + 1] - pts[(2 * a) + 1];
        double vx = pts[2 * c] - pts[2 * a], vz = pts[(2 * c) + 1] - pts[(2 * a) + 1];
        return 0.5 * Math.Abs((ux * vz) - (uz * vx));
    }

    // A uniform point in a convex polygon: a fan triangle by area, then Osada et al.'s square-root barycentric mapping
    // (only sqrt, so identical everywhere). Draws: triangle, r1, r2.
    private static void SamplePolygon(double[] pts, int n, ref SeededRandom rng, out double x, out double z)
    {
        double total = FanArea(pts, n);
        double target = rng.NextDouble() * total;
        double acc = 0;
        int tri = 1;
        for (int i = 1; i + 1 < n; i++)
        {
            double a = TriArea(pts, 0, i, i + 1);
            if (a > 0)
            {
                acc += a;
                tri = i;
                if (target < acc)
                {
                    break;
                }
            }
        }
        double r1 = Math.Sqrt(rng.NextDouble()), r2 = rng.NextDouble();
        double wa = 1 - r1, wb = r1 * (1 - r2), wc = r1 * r2;
        x = (wa * pts[0]) + (wb * pts[2 * tri]) + (wc * pts[2 * (tri + 1)]);
        z = (wa * pts[1]) + (wb * pts[(2 * tri) + 1]) + (wc * pts[(2 * (tri + 1)) + 1]);
    }

    // Clips a polygon (world X/Z) to an axis-aligned box (Sutherland and Hodgman 1974); the result is left in _clipA.
    private int ClipPolyToBox(NavTile t, int p, double x0, double z0, double x1, double z1, out double[] pts)
    {
        int n = t.PolyVertexCounts[p];
        EnsureClip(n);
        FillWorldPoly(t, p, _clipA);
        double[] src = _clipA, dst = _clipB;
        for (int plane = 0; plane < 4 && n >= 3; plane++)
        {
            int m = 0;
            for (int i = 0; i < n; i++)
            {
                int j = (i + 1) % n;
                double px = src[2 * i], pz = src[(2 * i) + 1], qx = src[2 * j], qz = src[(2 * j) + 1];
                double dp = PlaneDist(plane, px, pz, x0, z0, x1, z1), dq = PlaneDist(plane, qx, qz, x0, z0, x1, z1);
                if (dp >= 0)
                {
                    dst[2 * m] = px;
                    dst[(2 * m) + 1] = pz;
                    m++;
                }
                if ((dp >= 0) != (dq >= 0))
                {
                    double s = dp / (dp - dq);
                    double ix = px + ((qx - px) * s), iz = pz + ((qz - pz) * s);
                    // Put the crossing exactly on the clip line.
                    switch (plane)
                    {
                        case 0: ix = x0; break;
                        case 1: ix = x1; break;
                        case 2: iz = z0; break;
                        default: iz = z1; break;
                    }
                    dst[2 * m] = ix;
                    dst[(2 * m) + 1] = iz;
                    m++;
                }
            }
            n = m;
            (src, dst) = (dst, src);
        }
        pts = src;
        if (!ReferenceEquals(src, _clipA))
        {
            Array.Copy(src, _clipA, 2 * n);
            pts = _clipA;
        }
        return n;
    }

    private static double PlaneDist(int plane, double x, double z, double x0, double z0, double x1, double z1) => plane switch
    {
        0 => x - x0,
        1 => x1 - x,
        2 => z - z0,
        _ => z1 - z,
    };
}
