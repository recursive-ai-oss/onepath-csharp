using System;
using System.Collections.Generic;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>An incrementally built triangulation of one polygon's detail points: ear-clipped hull, point insertion, Lawson flips with the exact in-circle test.</summary>
internal sealed class DetailTriangulation
{
    private readonly List<double> _x = new();
    private readonly List<double> _z = new();
    private readonly List<double> _y = new();
    private readonly List<int> _tri = new();
    private readonly List<int> _nb = new();
    private readonly List<int> _touched = new();
    private readonly List<int> _stamp = new();
    private readonly Stack<(int Tri, int Edge)> _stack = new();
    private int _round = 1;

    public int VertexCount => _x.Count;

    public int TriangleCount => _tri.Count / 3;

    public int Failures { get; private set; }

    public IReadOnlyList<int> Touched => _touched;

    public double X(int v) => _x[v];

    public double Z(int v) => _z[v];

    public double Y(int v) => _y[v];

    public int Vertex(int tri, int k) => _tri[(3 * tri) + k];

    public int Neighbor(int tri, int k) => _nb[(3 * tri) + k];

    public int AddVertex(double x, double z, double y)
    {
        _x.Add(x);
        _z.Add(z);
        _y.Add(y);
        return _x.Count - 1;
    }

    public int Orient(int a, int b, int c) => ExactPredicates.Orient2D(_x[a], _z[a], _x[b], _z[b], _x[c], _z[c]);

    private int Orient(int a, int b, double qx, double qz) => ExactPredicates.Orient2D(_x[a], _z[a], _x[b], _z[b], qx, qz);

    /// <summary>Ear-clips the ring (counter-clockwise vertex ids) into triangles, then flips to Delaunay.</summary>
    public void Start(IReadOnlyList<int> ring)
    {
        int n = ring.Count;
        var prev = new int[n];
        var next = new int[n];
        for (int i = 0; i < n; i++)
        {
            prev[i] = (i + n - 1) % n;
            next[i] = (i + 1) % n;
        }
        int count = n;
        int head = 0;
        while (count > 3)
        {
            int found = -1;
            for (int mode = 0; mode < 3 && found < 0; mode++)
            {
                int i = head;
                for (int step = 0; step < count; step++, i = next[i])
                {
                    int o = Orient(ring[prev[i]], ring[i], ring[next[i]]);
                    if (mode == 2 || (mode == 0 ? o > 0 : o >= 0))
                    {
                        if (mode == 2 || EarIsEmpty(ring, next, prev[i], i, next[i], count))
                        {
                            found = i;
                            break;
                        }
                    }
                }
                if (found >= 0 && mode == 2)
                {
                    Failures++;
                }
            }
            int p = prev[found], q = next[found];
            if (Orient(ring[p], ring[found], ring[q]) > 0)
            {
                AddTriangle(ring[p], ring[found], ring[q]);
            }
            next[p] = q;
            prev[q] = p;
            head = p;
            count--;
        }
        int a = head, b = next[a], c = next[b];
        if (Orient(ring[a], ring[b], ring[c]) > 0)
        {
            AddTriangle(ring[a], ring[b], ring[c]);
        }
        LinkAll();
        _stack.Clear();
        for (int t = 0; t < TriangleCount; t++)
        {
            for (int e = 0; e < 3; e++)
            {
                _stack.Push((t, e));
            }
        }
        Legalize();
    }

    private bool EarIsEmpty(IReadOnlyList<int> ring, int[] next, int ip, int ii, int iq, int count)
    {
        int a = ring[ip], b = ring[ii], c = ring[iq];
        int k = next[iq];
        for (int s = 0; s < count - 3; s++, k = next[k])
        {
            int v = ring[k];
            if (Orient(a, b, v) >= 0 && Orient(b, c, v) >= 0 && Orient(c, a, v) >= 0)
            {
                return false;
            }
        }
        return true;
    }

    private void AddTriangle(int a, int b, int c)
    {
        _tri.Add(a);
        _tri.Add(b);
        _tri.Add(c);
        _nb.Add(-1);
        _nb.Add(-1);
        _nb.Add(-1);
        _stamp.Add(0);
    }

    private void LinkAll()
    {
        var owner = new Dictionary<(int, int), (int, int)>();
        for (int t = 0; t < TriangleCount; t++)
        {
            for (int e = 0; e < 3; e++)
            {
                owner[(_tri[(3 * t) + e], _tri[(3 * t) + ((e + 1) % 3)])] = (t, e);
            }
        }
        for (int t = 0; t < TriangleCount; t++)
        {
            for (int e = 0; e < 3; e++)
            {
                if (owner.TryGetValue((_tri[(3 * t) + ((e + 1) % 3)], _tri[(3 * t) + e]), out var o))
                {
                    _nb[(3 * t) + e] = o.Item1;
                }
            }
        }
    }

    private void Touch(int t)
    {
        if (_stamp[t] != _round)
        {
            _stamp[t] = _round;
            _touched.Add(t);
        }
    }

    /// <summary>Starts a new round of change tracking.</summary>
    public void ClearTouched()
    {
        _touched.Clear();
        _round++;
    }

    public bool IsTouched(int t) => _stamp[t] == _round;

    private void Set(int t, int a, int b, int c, int n0, int n1, int n2)
    {
        int i = 3 * t;
        _tri[i] = a;
        _tri[i + 1] = b;
        _tri[i + 2] = c;
        _nb[i] = n0;
        _nb[i + 1] = n1;
        _nb[i + 2] = n2;
        Touch(t);
    }

    // Points the edge (from, to) of triangle t at a new neighbour.
    private void Link(int t, int from, int to, int newNeighbor)
    {
        if (t < 0)
        {
            return;
        }
        for (int k = 0; k < 3; k++)
        {
            if (_tri[(3 * t) + k] == from && _tri[(3 * t) + ((k + 1) % 3)] == to)
            {
                _nb[(3 * t) + k] = newNeighbor;
                return;
            }
        }
    }

    /// <summary>Finds the triangle containing (qx, qz) among <paramref name="candidates"/> (boundary counts), or -1.</summary>
    public int Locate(double qx, double qz, IReadOnlyList<int> candidates)
    {
        for (int i = 0; i < candidates.Count; i++)
        {
            int t = candidates[i];
            int a = _tri[3 * t], b = _tri[(3 * t) + 1], c = _tri[(3 * t) + 2];
            if (Orient(a, b, qx, qz) >= 0 && Orient(b, c, qx, qz) >= 0 && Orient(c, a, qx, qz) >= 0)
            {
                return t;
            }
        }
        return -1;
    }

    /// <summary>Height of triangle <paramref name="t"/>'s plane at (qx, qz).</summary>
    public double HeightAt(int t, double qx, double qz)
    {
        int a = _tri[3 * t], b = _tri[(3 * t) + 1], c = _tri[(3 * t) + 2];
        double d = ((_x[b] - _x[a]) * (_z[c] - _z[a])) - ((_z[b] - _z[a]) * (_x[c] - _x[a]));
        double u = (((qx - _x[a]) * (_z[c] - _z[a])) - ((qz - _z[a]) * (_x[c] - _x[a]))) / d;
        double v = (((_x[b] - _x[a]) * (qz - _z[a])) - ((_z[b] - _z[a]) * (qx - _x[a]))) / d;
        return _y[a] + (u * (_y[b] - _y[a])) + (v * (_y[c] - _y[a]));
    }

    /// <summary>Inserts the vertex (already added) into triangle <paramref name="t"/>, which contains it. Returns false when it lies on a hull edge or on a vertex.</summary>
    public bool Insert(int p, int t)
    {
        int a = _tri[3 * t], b = _tri[(3 * t) + 1], c = _tri[(3 * t) + 2];
        double px = _x[p], pz = _z[p];
        int o0 = Orient(a, b, px, pz), o1 = Orient(b, c, px, pz), o2 = Orient(c, a, px, pz);
        int zeros = (o0 == 0 ? 1 : 0) + (o1 == 0 ? 1 : 0) + (o2 == 0 ? 1 : 0);
        if (zeros >= 2)
        {
            return false;
        }
        _stack.Clear();
        if (zeros == 0)
        {
            int nab = _nb[3 * t], nbc = _nb[(3 * t) + 1], nca = _nb[(3 * t) + 2];
            int t1 = TriangleCount;
            AddTriangle(0, 0, 0);
            int t2 = TriangleCount;
            AddTriangle(0, 0, 0);
            Set(t, a, b, p, nab, t1, t2);
            Set(t1, b, c, p, nbc, t2, t);
            Set(t2, c, a, p, nca, t, t1);
            Link(nbc, c, b, t1);
            Link(nca, a, c, t2);
            _stack.Push((t, 0));
            _stack.Push((t1, 0));
            _stack.Push((t2, 0));
        }
        else
        {
            // On an edge: rotate so that the edge is (a, b).
            int e = o0 == 0 ? 0 : o1 == 0 ? 1 : 2;
            a = _tri[(3 * t) + e];
            b = _tri[(3 * t) + ((e + 1) % 3)];
            c = _tri[(3 * t) + ((e + 2) % 3)];
            int u = _nb[(3 * t) + e];
            if (u < 0)
            {
                return false;
            }
            int nbc = _nb[(3 * t) + ((e + 1) % 3)], nca = _nb[(3 * t) + ((e + 2) % 3)];
            int f = 0;
            while (_nb[(3 * u) + f] != t)
            {
                f++;
            }
            int d = _tri[(3 * u) + ((f + 2) % 3)];
            int nad = _nb[(3 * u) + ((f + 1) % 3)], ndb = _nb[(3 * u) + ((f + 2) % 3)];
            int t1 = TriangleCount;
            AddTriangle(0, 0, 0);
            int u1 = TriangleCount;
            AddTriangle(0, 0, 0);
            Set(t, a, p, c, u1, t1, nca);
            Set(t1, p, b, c, u, nbc, t);
            Set(u, b, p, d, t1, u1, ndb);
            Set(u1, p, a, d, t, nad, u);
            Link(nbc, c, b, t1);
            Link(nad, d, a, u1);
            _stack.Push((t, 2));
            _stack.Push((t1, 1));
            _stack.Push((u, 2));
            _stack.Push((u1, 1));
        }
        Legalize();
        return true;
    }

    private void Legalize()
    {
        while (_stack.Count > 0)
        {
            (int t, int e) = _stack.Pop();
            int u = _nb[(3 * t) + e];
            if (u < 0)
            {
                continue;
            }
            int a = _tri[(3 * t) + e], b = _tri[(3 * t) + ((e + 1) % 3)], c = _tri[(3 * t) + ((e + 2) % 3)];
            int f = 0;
            while (_nb[(3 * u) + f] != t)
            {
                f++;
            }
            int d = _tri[(3 * u) + ((f + 2) % 3)];
            if (ExactPredicates.InCircle(_x[a], _z[a], _x[b], _z[b], _x[c], _z[c], _x[d], _z[d]) <= 0)
            {
                continue;
            }
            // The quad a, d, b, c must be strictly convex for the other diagonal to be inside it.
            if (Orient(c, a, d) <= 0 || Orient(d, b, c) <= 0)
            {
                continue;
            }
            int nbc = _nb[(3 * t) + ((e + 1) % 3)], nca = _nb[(3 * t) + ((e + 2) % 3)];
            int nad = _nb[(3 * u) + ((f + 1) % 3)], ndb = _nb[(3 * u) + ((f + 2) % 3)];
            Set(t, c, a, d, nca, nad, u);
            Set(u, d, b, c, ndb, nbc, t);
            Link(nad, d, a, t);
            Link(nbc, c, b, u);
            _stack.Push((t, 0));
            _stack.Push((t, 1));
            _stack.Push((u, 0));
            _stack.Push((u, 1));
        }
    }
}

/// <summary>Plain data returned by <see cref="DetailMeshBuilder"/>: one detail mesh per polygon, in the format's layout.</summary>
public sealed class DetailMeshSet
{
    internal DetailMeshSet(DetailMesh[] meshes, float[] vertices, byte[] triangles, int failures)
    {
        Meshes = meshes;
        Vertices = vertices;
        Triangles = triangles;
        TriangulationFailures = failures;
    }

    /// <summary>Per polygon, where its detail sits.</summary>
    public DetailMesh[] Meshes { get; }

    /// <summary>Extra detail vertices, three tile-local float32 each.</summary>
    public float[] Vertices { get; }

    /// <summary>Detail triangles, four bytes each: three vertex indices and the edge flags (bit k: edge k lies on the polygon boundary).</summary>
    public byte[] Triangles { get; }

    /// <summary>Hull triangulation steps that settled for a degenerate ear (0 for well-formed polygons).</summary>
    public int TriangulationFailures { get; }
}

/// <summary>Parameters of the height detail stage, in world units.</summary>
public sealed class DetailSettings
{
    /// <summary>Spacing of the height samples inside a polygon and along its edges. 0 (or any value at most 0.9 cells) disables detail: each polygon becomes triangles over its own vertices. Default 1.8.</summary>
    public double SampleDistance { get; set; } = 1.8;

    /// <summary>Largest vertical distance a sampled height may lie from the detail surface before a vertex is added. Default 0.2.</summary>
    public double MaxError { get; set; } = 0.2;

    /// <summary>Checks the values.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is negative, NaN or infinite.</exception>
    public void Validate()
    {
        if (!(SampleDistance >= 0) || double.IsInfinity(SampleDistance))
        {
            throw new ArgumentOutOfRangeException(nameof(SampleDistance), SampleDistance, "The sample distance must be finite and not negative.");
        }
        if (!(MaxError >= 0) || double.IsInfinity(MaxError))
        {
            throw new ArgumentOutOfRangeException(nameof(MaxError), MaxError, "The maximum error must be finite and not negative.");
        }
    }
}

/// <summary>
/// Builds the per-polygon height detail: edge points chosen by Douglas-Peucker on the sampled heights, then interior samples added one at a time
/// where the detail surface is farthest from the heightfield until the error is within bounds, keeping the triangulation Delaunay with the exact in-circle
/// test. Details and tie-breaks are in docs/build3d-polys.md.
/// </summary>
public static class DetailMeshBuilder
{
    private const int MaxVertices = 255;

    /// <summary>Builds the detail of every polygon.</summary>
    /// <param name="mesh">The polygons.</param>
    /// <param name="chf">The compact heightfield the polygons came from (with regions).</param>
    /// <param name="map">Maps grid positions to tile-local coordinates.</param>
    /// <param name="settings">Sampling parameters.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Invalid settings.</exception>
    public static DetailMeshSet Build(PolyMesh mesh, CompactHeightfield chf, TileCoordinates map, DetailSettings settings)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(map);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        double sd = settings.SampleDistance / map.CellSize;
        bool sampling = sd > 0.9;
        int np = mesh.PolygonCount;
        var meshes = new DetailMesh[np];
        var dverts = new List<float>();
        var dtris = new List<byte>();
        int failures = 0;
        ReadOnlySpan<int> v = mesh.Vertices;
        ReadOnlySpan<int> idx = mesh.PolygonIndices;
        ReadOnlySpan<int> counts = mesh.PolygonVertexCounts;
        int pos = 0;
        for (int pi = 0; pi < np; pi++)
        {
            int n = counts[pi];
            var tri = new DetailTriangulation();
            var cx = new double[n];
            var cz = new double[n];
            var cy = new int[n];
            for (int i = 0; i < n; i++)
            {
                int g = idx[pos + i];
                cx[i] = v[3 * g];
                cy[i] = v[(3 * g) + 1];
                cz[i] = v[(3 * g) + 2];
                tri.AddVertex(map.Local(cx[i]), map.Local(cz[i]), map.LocalY(cy[i]));
            }
            var ring = new List<int>(n);
            for (int i = 0; i < n; i++)
            {
                ring.Add(i);
            }
            if (sampling)
            {
                var extra = new List<(int After, double X, double Z, double Row)>();
                for (int i = 0; i < n; i++)
                {
                    EdgePoints(chf, map, settings, sd, cx[i], cz[i], cy[i], cx[(i + 1) % n], cz[(i + 1) % n], cy[(i + 1) % n], i, extra);
                }
                if (n + extra.Count <= MaxVertices)
                {
                    ring.Clear();
                    int ei = 0;
                    for (int i = 0; i < n; i++)
                    {
                        ring.Add(i);
                        while (ei < extra.Count && extra[ei].After == i)
                        {
                            ring.Add(tri.AddVertex(map.Local(extra[ei].X), map.Local(extra[ei].Z), map.LocalY(extra[ei].Row)));
                            ei++;
                        }
                    }
                }
            }
            tri.Start(ring);
            if (sampling)
            {
                Interior(mesh, chf, map, settings, sd, mesh.RegionArray[pi], cx, cz, cy, tri);
            }
            failures += tri.Failures;

            int vcount = tri.VertexCount - n;
            if (tri.TriangleCount > ushort.MaxValue)
            {
                throw new InvalidOperationException("A polygon's detail has more than 65535 triangles.");
            }
            meshes[pi] = new DetailMesh { VertexBase = (uint)(dverts.Count / 3), TriangleBase = (uint)(dtris.Count / 4), VertexCount = (ushort)vcount, TriangleCount = (ushort)tri.TriangleCount };
            for (int k = n; k < tri.VertexCount; k++)
            {
                dverts.Add((float)tri.X(k));
                dverts.Add((float)tri.Y(k));
                dverts.Add((float)tri.Z(k));
            }
            for (int t = 0; t < tri.TriangleCount; t++)
            {
                int flags = 0;
                for (int e = 0; e < 3; e++)
                {
                    if (tri.Neighbor(t, e) < 0)
                    {
                        flags |= 1 << e;
                    }
                }
                dtris.Add((byte)tri.Vertex(t, 0));
                dtris.Add((byte)tri.Vertex(t, 1));
                dtris.Add((byte)tri.Vertex(t, 2));
                dtris.Add((byte)flags);
            }
            pos += n;
        }
        return new DetailMeshSet(meshes, dverts.ToArray(), dtris.ToArray(), failures);
    }

    // The span floor row in the columns around (x, z) closest to the expected row, within the climb; -1 when none.
    // Looks at the column of the floor of the position and, on a grid line, at the column before it, so that the answer
    // is the same from both sides of a shared edge.
    private static int NearestRow(CompactHeightfield chf, double x, double z, double expected, int region)
    {
        HeightfieldFrame f = chf.Frame;
        int fx = (int)Math.Floor(x), fz = (int)Math.Floor(z);
        int best = -1;
        double bestD = double.MaxValue;
        int maxDelta = Math.Max(chf.WalkableClimb, 1) + 1;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> cnt = chf.CellCount;
        Span<CompactSpan> spans = chf.Spans;
        for (int dz = (z == fz ? -1 : 0); dz <= 0; dz++)
        {
            for (int dx = (x == fx ? -1 : 0); dx <= 0; dx++)
            {
                int ax = fx + dx, az = fz + dz;
                if (ax < 0 || az < 0 || ax >= f.Width || az >= f.Depth)
                {
                    continue;
                }
                int col = ax + (az * f.Width);
                for (int s = first[col]; s < first[col] + cnt[col]; s++)
                {
                    if (region >= 0 && spans[s].Region != region)
                    {
                        continue;
                    }
                    double d = Math.Abs(spans[s].Y - expected);
                    if (d <= maxDelta && d < bestD)
                    {
                        bestD = d;
                        best = spans[s].Y;
                    }
                }
            }
        }
        return best;
    }

    // Points along one polygon edge, in the polygon's direction, after Douglas-Peucker on the heights. The walk starts from the
    // lexicographically smaller end so that the two polygons sharing the edge generate the same points.
    private static void EdgePoints(CompactHeightfield chf, TileCoordinates map, DetailSettings s, double sd, double ax, double az, int ay, double bx, double bz, int by, int after, List<(int, double, double, double)> output)
    {
        bool swap = ax > bx || (ax == bx && az > bz);
        double x0 = swap ? bx : ax, z0 = swap ? bz : az, x1 = swap ? ax : bx, z1 = swap ? az : bz;
        double r0 = swap ? by : ay, r1 = swap ? ay : by;
        double dx = x1 - x0, dz = z1 - z0;
        double len = Math.Sqrt((dx * dx) + (dz * dz));
        if (!(len > sd))
        {
            return;
        }
        int ns = (int)Math.Ceiling(len / sd);
        if (ns > 4096)
        {
            return;
        }
        var px = new double[ns + 1];
        var pz = new double[ns + 1];
        var ph = new double[ns + 1];
        var lin = new double[ns + 1];
        for (int k = 0; k <= ns; k++)
        {
            double t = k / (double)ns;
            px[k] = x0 + (dx * t);
            pz[k] = z0 + (dz * t);
            lin[k] = r0 + ((r1 - r0) * t);
            ph[k] = lin[k];
            if (k > 0 && k < ns)
            {
                int row = NearestRow(chf, px[k], pz[k], lin[k], -1);
                if (row >= 0)
                {
                    ph[k] = row;
                }
            }
        }
        var keep = new bool[ns + 1];
        keep[0] = keep[ns] = true;
        var stack = new Stack<(int, int)>();
        stack.Push((0, ns));
        double maxRows = s.MaxError / map.CellHeight;
        while (stack.Count > 0)
        {
            (int i, int j) = stack.Pop();
            int worst = -1;
            double worstD = maxRows;
            for (int k = i + 1; k < j; k++)
            {
                double t = (k - i) / (double)(j - i);
                double d = Math.Abs(ph[k] - (ph[i] + ((ph[j] - ph[i]) * t)));
                if (d > worstD)
                {
                    worstD = d;
                    worst = k;
                }
            }
            if (worst >= 0)
            {
                keep[worst] = true;
                stack.Push((i, worst));
                stack.Push((worst, j));
            }
        }
        if (!swap)
        {
            for (int k = 1; k < ns; k++)
            {
                if (keep[k])
                {
                    output.Add((after, px[k], pz[k], ph[k]));
                }
            }
        }
        else
        {
            for (int k = ns - 1; k >= 1; k--)
            {
                if (keep[k])
                {
                    output.Add((after, px[k], pz[k], ph[k]));
                }
            }
        }
    }

    private static void Interior(PolyMesh mesh, CompactHeightfield chf, TileCoordinates map, DetailSettings settings, double sd, int region, double[] cx, double[] cz, int[] cy, DetailTriangulation tri)
    {
        int n = cx.Length;
        double minX = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxZ = double.MinValue;
        double meanRow = 0;
        for (int i = 0; i < n; i++)
        {
            minX = Math.Min(minX, cx[i]);
            maxX = Math.Max(maxX, cx[i]);
            minZ = Math.Min(minZ, cz[i]);
            maxZ = Math.Max(maxZ, cz[i]);
            meanRow += cy[i];
        }
        meanRow /= n;
        var sx = new List<double>();
        var sz = new List<double>();
        var sy = new List<double>();
        double half = sd * 0.5;
        for (double z = minZ + sd; z < maxZ; z += sd)
        {
            for (double x = minX + sd; x < maxX; x += sd)
            {
                bool inside = true;
                for (int i = 0; i < n && inside; i++)
                {
                    int j = (i + 1) % n;
                    double ex = cx[j] - cx[i], ez = cz[j] - cz[i];
                    double cr = (ex * (z - cz[i])) - (ez * (x - cx[i]));
                    double l = Math.Sqrt((ex * ex) + (ez * ez));
                    if (!(cr > half * l))
                    {
                        inside = false;
                    }
                }
                if (!inside)
                {
                    continue;
                }
                int row = NearestRow(chf, x, z, meanRow, region);
                if (row < 0)
                {
                    continue;
                }
                sx.Add(map.Local(x));
                sz.Add(map.Local(z));
                sy.Add(map.LocalY(row));
            }
        }
        int ns = sx.Count;
        if (ns == 0)
        {
            return;
        }
        var owner = new int[ns];
        var err = new double[ns];
        var live = new bool[ns];
        var all = new List<int>();
        for (int t = 0; t < tri.TriangleCount; t++)
        {
            all.Add(t);
        }
        for (int s = 0; s < ns; s++)
        {
            live[s] = true;
            owner[s] = tri.Locate(sx[s], sz[s], all);
            if (owner[s] < 0)
            {
                live[s] = false;
                continue;
            }
            err[s] = Math.Abs(sy[s] - tri.HeightAt(owner[s], sx[s], sz[s]));
        }
        while (tri.VertexCount < MaxVertices)
        {
            int best = -1;
            double bestErr = settings.MaxError;
            for (int s = 0; s < ns; s++)
            {
                if (live[s] && err[s] > bestErr)
                {
                    bestErr = err[s];
                    best = s;
                }
            }
            if (best < 0)
            {
                break;
            }
            live[best] = false;
            int v = tri.AddVertex(sx[best], sz[best], sy[best]);
            tri.ClearTouched();
            if (!tri.Insert(v, owner[best]))
            {
                continue;
            }
            IReadOnlyList<int> touched = tri.Touched;
            for (int s = 0; s < ns; s++)
            {
                if (!live[s] || !tri.IsTouched(owner[s]))
                {
                    continue;
                }
                int t = tri.Locate(sx[s], sz[s], touched);
                if (t < 0)
                {
                    live[s] = false;
                    continue;
                }
                owner[s] = t;
                err[s] = Math.Abs(sy[s] - tri.HeightAt(t, sx[s], sz[s]));
            }
        }
    }
}
