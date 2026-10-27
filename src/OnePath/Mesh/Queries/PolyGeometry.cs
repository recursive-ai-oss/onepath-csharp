using System;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>Geometry helpers on one polygon of a tile; all coordinates are tile-local doubles.</summary>
internal static class PolyGeometry
{
    /// <summary>
    /// Height of the polygon surface at (x, z). With detail, the first detail triangle containing the point; without,
    /// the fan of triangles (v0, vi, vi+1). A point outside every triangle (boundary rounding) takes the plane of the
    /// triangle it violates least.
    /// </summary>
    public static double HeightAt(NavTile t, int p, double x, double z)
    {
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        DetailMesh m = t.DetailMeshes[p];
        double bestMin = double.NegativeInfinity;
        double bestY = v[(3 * idx[b]) + 1];
        if (m.TriangleCount > 0)
        {
            ReadOnlySpan<byte> dt = t.DetailTriangles;
            for (int k = 0; k < m.TriangleCount; k++)
            {
                int o = 4 * (int)(m.TriangleBase + (uint)k);
                Corner(t, p, dt[o], out double ax, out double ay, out double az);
                Corner(t, p, dt[o + 1], out double bx, out double by, out double bz);
                Corner(t, p, dt[o + 2], out double cx, out double cy, out double cz);
                if (Tri(ax, ay, az, bx, by, bz, cx, cy, cz, x, z, ref bestMin, ref bestY))
                {
                    return bestY;
                }
            }
            return bestY;
        }
        int i0 = idx[b];
        double x0 = v[3 * i0], y0 = v[(3 * i0) + 1], z0 = v[(3 * i0) + 2];
        for (int i = 1; i + 1 < n; i++)
        {
            int i1 = idx[b + i], i2 = idx[b + i + 1];
            if (Tri(x0, y0, z0, v[3 * i1], v[(3 * i1) + 1], v[(3 * i1) + 2], v[3 * i2], v[(3 * i2) + 1], v[(3 * i2) + 2], x, z, ref bestMin, ref bestY))
            {
                return bestY;
            }
        }
        return bestY;
    }

    // Vertex i of a polygon's detail mesh: the polygon's own vertices first, then its extra detail vertices.
    private static void Corner(NavTile t, int p, int i, out double px, out double py, out double pz)
    {
        int n = t.PolyVertexCounts[p];
        if (i < n)
        {
            int vi = t.PolyIndices[t.PolyStart[p] + i];
            px = t.Vertices[3 * vi];
            py = t.Vertices[(3 * vi) + 1];
            pz = t.Vertices[(3 * vi) + 2];
        }
        else
        {
            int di = 3 * (int)(t.DetailMeshes[p].VertexBase + (uint)(i - n));
            px = t.DetailVertices[di];
            py = t.DetailVertices[di + 1];
            pz = t.DetailVertices[di + 2];
        }
    }

    // Barycentric test in X/Z. Returns true (with y) if the point is inside within a tiny tolerance; otherwise keeps the
    // least-violating triangle's plane height in bestY.
    private static bool Tri(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz, double x, double z, ref double bestMin, ref double bestY)
    {
        double d = ((bz - cz) * (ax - cx)) + ((cx - bx) * (az - cz));
        if (d == 0)
        {
            return false;
        }
        double u = (((bz - cz) * (x - cx)) + ((cx - bx) * (z - cz))) / d;
        double w = (((cz - az) * (x - cx)) + ((ax - cx) * (z - cz))) / d;
        double q = 1.0 - u - w;
        double mn = Math.Min(u, Math.Min(w, q));
        double y = (u * ay) + (w * by) + (q * cy);
        if (mn >= -1e-9)
        {
            bestY = y;
            return true;
        }
        if (mn > bestMin)
        {
            bestMin = mn;
            bestY = y;
        }
        return false;
    }

    /// <summary>True if (x, z) is inside or within <paramref name="tol"/> (distance) of the polygon outline.</summary>
    public static bool Contains(NavTile t, int p, double x, double z, double tol)
    {
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        for (int e = 0; e < n; e++)
        {
            int a = idx[b + e], c = idx[b + ((e + 1) % n)];
            double ex = v[3 * c] - (double)v[3 * a], ez = v[(3 * c) + 2] - (double)v[(3 * a) + 2];
            double cross = (ex * (z - v[(3 * a) + 2])) - (ez * (x - v[3 * a]));
            if (cross < -tol * Math.Sqrt((ex * ex) + (ez * ez)))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Closest point of the polygon to a position: inside the outline seen from above, the surface point under/over it;
    /// otherwise the nearest point of the outline in X/Z (ties to the lower edge) with height interpolated along that edge.
    /// </summary>
    public static void Closest(NavTile t, int p, double px, double py, double pz, out double cx, out double cy, out double cz, out bool over)
    {
        _ = py;
        if (Contains(t, p, px, pz, 0.0))
        {
            cx = px;
            cz = pz;
            cy = HeightAt(t, p, px, pz);
            over = true;
            return;
        }
        over = false;
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        double best = double.PositiveInfinity;
        cx = cy = cz = 0;
        for (int e = 0; e < n; e++)
        {
            int a = idx[b + e], c = idx[b + ((e + 1) % n)];
            double ax = v[3 * a], az = v[(3 * a) + 2], ay = v[(3 * a) + 1];
            double ex = v[3 * c] - ax, ez = v[(3 * c) + 2] - az;
            double len2 = (ex * ex) + (ez * ez);
            double s = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0.0, 1.0) : 0.0;
            double qx = ax + (ex * s), qz = az + (ez * s);
            double d2 = ((px - qx) * (px - qx)) + ((pz - qz) * (pz - qz));
            if (d2 < best)
            {
                best = d2;
                cx = qx;
                cz = qz;
                cy = ay + ((v[(3 * c) + 1] - ay) * s);
            }
        }
    }
}

/// <summary>
/// A bounding-volume hierarchy over a tile's polygons for box queries. Built once per tile by a deterministic median
/// split (sorted by centroid, ties by polygon index) and cached on the tile.
/// </summary>
internal sealed class TileBvh
{
    private const int LeafSize = 4;

    private readonly float[] _bounds; // 6 per node: min xyz, max xyz
    private readonly int[] _a;        // internal: left child; leaf: first slot in _order
    private readonly int[] _b;        // internal: right child; leaf: -(count)
    private readonly int[] _order;
    private float[] _pb = [];
    private int _count;

    private TileBvh(int polys)
    {
        int cap = (2 * polys) + 1;
        _bounds = new float[6 * cap];
        _a = new int[cap];
        _b = new int[cap];
        _order = new int[polys];
    }

    public static TileBvh Build(NavTile t)
    {
        int n = t.PolyCount;
        var bvh = new TileBvh(n);
        if (n == 0)
        {
            return bvh;
        }
        var pb = new float[6 * n];
        for (int p = 0; p < n; p++)
        {
            int cnt = t.PolyVertexCounts[p];
            int s = t.PolyStart[p];
            float x0 = float.MaxValue, y0 = float.MaxValue, z0 = float.MaxValue, x1 = float.MinValue, y1 = float.MinValue, z1 = float.MinValue;
            for (int i = 0; i < cnt; i++)
            {
                int v = 3 * t.PolyIndices[s + i];
                x0 = Math.Min(x0, t.Vertices[v]);
                y0 = Math.Min(y0, t.Vertices[v + 1]);
                z0 = Math.Min(z0, t.Vertices[v + 2]);
                x1 = Math.Max(x1, t.Vertices[v]);
                y1 = Math.Max(y1, t.Vertices[v + 1]);
                z1 = Math.Max(z1, t.Vertices[v + 2]);
            }
            pb[6 * p] = x0;
            pb[(6 * p) + 1] = y0;
            pb[(6 * p) + 2] = z0;
            pb[(6 * p) + 3] = x1;
            pb[(6 * p) + 4] = y1;
            pb[(6 * p) + 5] = z1;
        }
        for (int i = 0; i < n; i++)
        {
            bvh._order[i] = i;
        }
        bvh._pb = pb;
        bvh.BuildNode(pb, 0, n);
        return bvh;
    }

    private int BuildNode(float[] pb, int lo, int hi)
    {
        int node = _count++;
        float x0 = float.MaxValue, y0 = float.MaxValue, z0 = float.MaxValue, x1 = float.MinValue, y1 = float.MinValue, z1 = float.MinValue;
        for (int i = lo; i < hi; i++)
        {
            int p = 6 * _order[i];
            x0 = Math.Min(x0, pb[p]);
            y0 = Math.Min(y0, pb[p + 1]);
            z0 = Math.Min(z0, pb[p + 2]);
            x1 = Math.Max(x1, pb[p + 3]);
            y1 = Math.Max(y1, pb[p + 4]);
            z1 = Math.Max(z1, pb[p + 5]);
        }
        int o = 6 * node;
        _bounds[o] = x0;
        _bounds[o + 1] = y0;
        _bounds[o + 2] = z0;
        _bounds[o + 3] = x1;
        _bounds[o + 4] = y1;
        _bounds[o + 5] = z1;
        if (hi - lo <= LeafSize)
        {
            _a[node] = lo;
            _b[node] = -(hi - lo);
            return node;
        }
        // Split on the longest axis of the bounds at the median centroid; ties by polygon index keep the order total.
        int axis = 0;
        float ex = x1 - x0, ey = y1 - y0, ez = z1 - z0;
        if (ey > ex && ey >= ez)
        {
            axis = 1;
        }
        else if (ez > ex && ez > ey)
        {
            axis = 2;
        }
        Array.Sort(_order, lo, hi - lo, new CentroidComparer(pb, axis));
        int mid = lo + ((hi - lo) / 2);
        int left = BuildNode(pb, lo, mid);
        int right = BuildNode(pb, mid, hi);
        _a[node] = left;
        _b[node] = right;
        return node;
    }

    private sealed class CentroidComparer : System.Collections.Generic.IComparer<int>
    {
        private readonly float[] _pb;
        private readonly int _axis;

        public CentroidComparer(float[] pb, int axis)
        {
            _pb = pb;
            _axis = axis;
        }

        public int Compare(int x, int y)
        {
            float cx = _pb[(6 * x) + _axis] + _pb[(6 * x) + 3 + _axis];
            float cy = _pb[(6 * y) + _axis] + _pb[(6 * y) + 3 + _axis];
            int c = cx.CompareTo(cy);
            return c != 0 ? c : x.CompareTo(y);
        }
    }

    /// <summary>
    /// Collects polygons whose box overlaps the tile-local box into <paramref name="results"/>, advancing
    /// <paramref name="count"/> past what was written (stops writing when full but keeps counting).
    /// </summary>
    public void Query(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, int[] results, ref int count, int[] stack)
    {
        if (_count == 0)
        {
            return;
        }
        // Explicit caller-provided stack: depth is at most ~log2(polygons) + 1.
        int sp = 0;
        stack[sp++] = 0;
        while (sp > 0)
        {
            int node = stack[--sp];
            int o = 6 * node;
            if (_bounds[o] > maxX || _bounds[o + 3] < minX || _bounds[o + 1] > maxY || _bounds[o + 4] < minY || _bounds[o + 2] > maxZ || _bounds[o + 5] < minZ)
            {
                continue;
            }
            if (_b[node] <= 0)
            {
                int first = _a[node];
                int c = -_b[node];
                for (int i = 0; i < c; i++)
                {
                    int p = _order[first + i];
                    int q = 6 * p;
                    if (_pb[q] > maxX || _pb[q + 3] < minX || _pb[q + 1] > maxY || _pb[q + 4] < minY || _pb[q + 2] > maxZ || _pb[q + 5] < minZ)
                    {
                        continue;
                    }
                    if (count < results.Length)
                    {
                        results[count] = p;
                    }
                    count++;
                }
            }
            else
            {
                stack[sp++] = _b[node];
                stack[sp++] = _a[node];
            }
        }
    }
}
