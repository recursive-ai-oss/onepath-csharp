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
            ReadOnlySpan<float> dv = t.DetailVertices;
            int vb = (int)m.VertexBase;
            int tb = (int)m.TriangleBase;
            // Pass 1 tries only the triangles a lookup grid lists for the point's cell (in index order, every ill-shaped
            // triangle included), so the first triangle that contains the point is the one a scan of all of them would
            // find. Pass 2 (only when none contains the point) is the full scan that picks the least-violating plane.
            if (m.TriangleCount >= DetailHeightGrid.MinTriangles)
            {
                DetailHeightGrid g = t.GetHeightGrid();
                ReadOnlySpan<int> cand = g.Candidates(p, x, z);
                for (int ci = 0; ci < cand.Length; ci++)
                {
                    int o = 4 * (tb + cand[ci]);
                    DetailHeightGrid.Corners(v, dv, idx, b, n, vb, dt, o, out double ax, out double ay, out double az, out double bx, out double by, out double bz, out double cx, out double cy, out double cz);
                    double dummyMin = double.NegativeInfinity, dummyY = 0;
                    if (Tri(ax, ay, az, bx, by, bz, cx, cy, cz, x, z, ref dummyMin, ref dummyY))
                    {
                        return dummyY;
                    }
                }
            }
            for (int k = 0; k < m.TriangleCount; k++)
            {
                int o = 4 * (tb + k);
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
    internal static bool Tri(double ax, double ay, double az, double bx, double by, double bz, double cx, double cy, double cz, double x, double z, ref double bestMin, ref double bestY)
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

/// <summary>
/// A lookup grid over each polygon's detail triangles, so a height query tests the few triangles near the point and not
/// all of them. Candidate lists are in triangle order and include every triangle the containment test could accept at
/// a point in the cell, so the first accepted candidate is the first accepted triangle overall. Built once per tile,
/// deterministically, and cached on it.
/// </summary>
internal sealed class DetailHeightGrid
{
    /// <summary>Polygons with fewer detail triangles are scanned directly.</summary>
    public const int MinTriangles = 12;

    private readonly int[] _cellBase;   // per polygon: first cell index, or -1 when scanned directly
    private readonly int[] _dim;        // per polygon: cells per side
    private readonly double[] _box;     // per polygon: minX, minZ, 1/cellX, 1/cellZ
    private int[] _cellStart = [];      // CSR over cells (cell + 1 for the end)
    private int[] _items = [];

    private DetailHeightGrid(int polys)
    {
        _cellBase = new int[polys];
        _dim = new int[polys];
        _box = new double[4 * polys];
    }

    public static void Corners(ReadOnlySpan<float> v, ReadOnlySpan<float> dv, ReadOnlySpan<ushort> idx, int b, int n, int vb, ReadOnlySpan<byte> dt, int o,
        out double ax, out double ay, out double az, out double bx, out double by, out double bz, out double cx, out double cy, out double cz)
    {
        Corner(v, dv, idx, b, n, vb, dt[o], out ax, out ay, out az);
        Corner(v, dv, idx, b, n, vb, dt[o + 1], out bx, out by, out bz);
        Corner(v, dv, idx, b, n, vb, dt[o + 2], out cx, out cy, out cz);
    }

    private static void Corner(ReadOnlySpan<float> v, ReadOnlySpan<float> dv, ReadOnlySpan<ushort> idx, int b, int n, int vb, int i, out double x, out double y, out double z)
    {
        if (i < n)
        {
            int vi = 3 * idx[b + i];
            x = v[vi];
            y = v[vi + 1];
            z = v[vi + 2];
        }
        else
        {
            int di = 3 * (vb + i - n);
            x = dv[di];
            y = dv[di + 1];
            z = dv[di + 2];
        }
    }

    public ReadOnlySpan<int> Candidates(int p, double x, double z)
    {
        int cb = _cellBase[p];
        if (cb < 0)
        {
            return default;
        }
        int d = _dim[p];
        int cx = ClampCell((x - _box[4 * p]) * _box[(4 * p) + 2], d);
        int cz = ClampCell((z - _box[(4 * p) + 1]) * _box[(4 * p) + 3], d);
        int cell = cb + (cz * d) + cx;
        return _items.AsSpan(_cellStart[cell], _cellStart[cell + 1] - _cellStart[cell]);
    }

    private static int ClampCell(double f, int d) => !(f > 0) ? 0 : (f >= d ? d - 1 : (int)f);

    public static DetailHeightGrid Build(NavTile t)
    {
        int polys = t.PolyCount;
        var g = new DetailHeightGrid(polys);
        var starts = new System.Collections.Generic.List<int>();
        var items = new System.Collections.Generic.List<int>();
        int cells = 0;
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<float> dv = t.DetailVertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        ReadOnlySpan<byte> dt = t.DetailTriangles;
        for (int p = 0; p < polys; p++)
        {
            DetailMesh m = t.DetailMeshes[p];
            g._cellBase[p] = -1;
            if (m.TriangleCount < MinTriangles)
            {
                continue;
            }
            int n = t.PolyVertexCounts[p];
            int b = t.PolyStart[p];
            int vb = (int)m.VertexBase;
            int tb = (int)m.TriangleBase;
            int tc = m.TriangleCount;
            var lo = new double[2 * tc];
            var hi = new double[2 * tc];
            var bad = new bool[tc];
            double minX = double.PositiveInfinity, minZ = double.PositiveInfinity, maxX = double.NegativeInfinity, maxZ = double.NegativeInfinity;
            for (int k = 0; k < tc; k++)
            {
                Corners(v, dv, idx, b, n, vb, dt, 4 * (tb + k), out double ax, out _, out double az, out double bx, out _, out double bz, out double cx, out _, out double cz);
                double tx0 = Math.Min(ax, Math.Min(bx, cx)), tx1 = Math.Max(ax, Math.Max(bx, cx));
                double tz0 = Math.Min(az, Math.Min(bz, cz)), tz1 = Math.Max(az, Math.Max(bz, cz));
                double ext = Math.Max(tx1 - tx0, tz1 - tz0);
                double margin = (1e-6 * ext) + 1e-9;
                lo[2 * k] = tx0 - margin;
                lo[(2 * k) + 1] = tz0 - margin;
                hi[2 * k] = tx1 + margin;
                hi[(2 * k) + 1] = tz1 + margin;
                // Containment is a division by twice the signed area; when that is tiny next to the triangle's size the
                // quotients are unreliable, so such a triangle is a candidate everywhere.
                bad[k] = !(Math.Abs(((bz - cz) * (ax - cx)) + ((cx - bx) * (az - cz))) >= 1e-6 * ext * ext);
                minX = Math.Min(minX, tx0);
                minZ = Math.Min(minZ, tz0);
                maxX = Math.Max(maxX, tx1);
                maxZ = Math.Max(maxZ, tz1);
            }
            int dim = Math.Clamp((int)Math.Sqrt(tc / 2.0), 2, 32);
            double cw = Math.Max((maxX - minX) / dim, 1e-9), ch = Math.Max((maxZ - minZ) / dim, 1e-9);
            g._box[4 * p] = minX;
            g._box[(4 * p) + 1] = minZ;
            g._box[(4 * p) + 2] = 1.0 / cw;
            g._box[(4 * p) + 3] = 1.0 / ch;
            g._dim[p] = dim;
            g._cellBase[p] = cells;
            for (int cz = 0; cz < dim; cz++)
            {
                for (int cx = 0; cx < dim; cx++)
                {
                    starts.Add(items.Count);
                    // The cell's box, widened a little so rounding of the cell lookup never drops a triangle.
                    double x0 = minX + (cx * cw) - (1e-6 * cw), x1 = minX + ((cx + 1) * cw) + (1e-6 * cw);
                    double z0 = minZ + (cz * ch) - (1e-6 * ch), z1 = minZ + ((cz + 1) * ch) + (1e-6 * ch);
                    for (int k = 0; k < tc; k++)
                    {
                        if (bad[k] || (lo[2 * k] <= x1 && hi[2 * k] >= x0 && lo[(2 * k) + 1] <= z1 && hi[(2 * k) + 1] >= z0))
                        {
                            items.Add(k);
                        }
                    }
                }
            }
            cells += dim * dim;
        }
        starts.Add(items.Count);
        g._cellStart = starts.ToArray();
        g._items = items.ToArray();
        return g;
    }
}
