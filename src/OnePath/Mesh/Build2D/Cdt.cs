using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>
/// Constrained Delaunay triangulation of grid points with exact predicates.
/// </summary>
/// <remarks>
/// Points are inserted in canonical (x, y) order by Lawson's incremental algorithm inside a large finite
/// bounding triangle; constraints are then recovered by Sloan's edge-flipping method (Sloan 1993), and the
/// non-constrained edges are finally restored to the Delaunay property (Chew 1989). Every decision is an exact
/// orientation or in-circle sign. The constraints must be noded: they may meet only at shared endpoints.
/// </remarks>
internal sealed class Cdt
{
    private const long SuperExtent = 1L << 45;

    private readonly P[] _p;
    private readonly int _n; // real vertex count; vertices _n.._n+2 are the bounding triangle
    private int[] _v;        // 3 vertices per triangle, counter-clockwise
    private int[] _nb;       // neighbour across edge i = (v[i], v[i+1]); -1 on the outer border
    private bool[] _con;     // constrained flag per triangle edge
    private readonly int[] _vt; // one incident triangle per vertex
    private int _tc;
    private int _last;

    public int VertexCount => _n;

    public int TriangleCount => _tc;

    public int Vertex(int t, int i) => _v[(3 * t) + i];

    public int Neighbor(int t, int i) => _nb[(3 * t) + i];

    public bool IsConstrained(int t, int i) => _con[(3 * t) + i];

    public bool TouchesBoundingTriangle(int t) => _v[3 * t] >= _n || _v[(3 * t) + 1] >= _n || _v[(3 * t) + 2] >= _n;

    /// <summary>Triangulates <paramref name="points"/> (distinct, sorted) with the given constraint edges (vertex index pairs).</summary>
    public Cdt(P[] points, IReadOnlyList<(int A, int B)> constraints)
    {
        _n = points.Length;
        _p = new P[_n + 3];
        Array.Copy(points, _p, _n);
        _p[_n] = new P(-SuperExtent, -SuperExtent);
        _p[_n + 1] = new P(3 * SuperExtent, -SuperExtent);
        _p[_n + 2] = new P(-SuperExtent, 3 * SuperExtent);
        int cap = (2 * _n) + 8;
        _v = new int[3 * cap];
        _nb = new int[3 * cap];
        _con = new bool[3 * cap];
        _vt = new int[_n + 3];
        _v[0] = _n; _v[1] = _n + 1; _v[2] = _n + 2;
        _nb[0] = _nb[1] = _nb[2] = -1;
        _tc = 1;
        _vt[_n] = _vt[_n + 1] = _vt[_n + 2] = 0;

        for (int i = 0; i < _n; i++)
        {
            Insert(i);
        }
        foreach (var (a, b) in constraints)
        {
            InsertConstraint(a, b);
        }
        RestoreDelaunay();
    }

    private static int Next(int i) => i == 2 ? 0 : i + 1;

    private static int Prev(int i) => i == 0 ? 2 : i - 1;

    private int IndexOf(int t, int vertex)
    {
        int b = 3 * t;
        return _v[b] == vertex ? 0 : _v[b + 1] == vertex ? 1 : _v[b + 2] == vertex ? 2 : -1;
    }

    // Index of the edge of triangle t running from a to b, or -1.
    private int EdgeIndex(int t, int a, int b)
    {
        int i = IndexOf(t, a);
        return i >= 0 && _v[(3 * t) + Next(i)] == b ? i : -1;
    }

    private void SetNeighborAcross(int t, int a, int b, int newNeighbor)
    {
        if (t < 0)
        {
            return;
        }
        // The edge of t that runs b→a is the one shared with the triangle holding a→b.
        int i = EdgeIndex(t, b, a);
        if (i < 0)
        {
            throw new GeometryException("Triangulation adjacency is inconsistent.");
        }
        _nb[(3 * t) + i] = newNeighbor;
    }

    private void SetTri(int t, int a, int b, int c, int n0, int n1, int n2, bool c0, bool c1, bool c2)
    {
        int k = 3 * t;
        _v[k] = a; _v[k + 1] = b; _v[k + 2] = c;
        _nb[k] = n0; _nb[k + 1] = n1; _nb[k + 2] = n2;
        _con[k] = c0; _con[k + 1] = c1; _con[k + 2] = c2;
        _vt[a] = t; _vt[b] = t; _vt[c] = t;
    }

    // ---- point location and insertion ----

    private int Locate(P p, out int edge)
    {
        int t = _last;
        int steps = 0;
        int limit = (4 * _tc) + 16;
        while (true)
        {
            int k = 3 * t;
            int moved = -1;
            int zeroEdge = -1;
            int zeros = 0;
            for (int i = 0; i < 3; i++)
            {
                int s = Geo.Orient(_p[_v[k + i]], _p[_v[k + Next(i)]], p);
                if (s < 0)
                {
                    moved = _nb[k + i];
                    break;
                }
                if (s == 0)
                {
                    zeros++;
                    zeroEdge = i;
                }
            }
            if (moved >= 0)
            {
                t = moved;
                if (++steps > limit)
                {
                    return LocateSlow(p, out edge);
                }
                continue;
            }
            if (zeros >= 2)
            {
                throw new GeometryException("A point coincides with an existing vertex.");
            }
            edge = zeros == 1 ? zeroEdge : -1;
            return t;
        }
    }

    private int LocateSlow(P p, out int edge)
    {
        for (int t = 0; t < _tc; t++)
        {
            int k = 3 * t;
            int zeros = 0, zeroEdge = -1;
            bool outside = false;
            for (int i = 0; i < 3 && !outside; i++)
            {
                int s = Geo.Orient(_p[_v[k + i]], _p[_v[k + Next(i)]], p);
                if (s < 0) { outside = true; }
                else if (s == 0) { zeros++; zeroEdge = i; }
            }
            if (!outside)
            {
                edge = zeros == 1 ? zeroEdge : -1;
                return t;
            }
        }
        throw new GeometryException("Point lies outside the bounding triangle.");
    }

    private void Insert(int pi)
    {
        P p = _p[pi];
        int t = Locate(p, out int edge);
        var stack = new List<int>(16);
        if (edge < 0)
        {
            int k = 3 * t;
            int a = _v[k], b = _v[k + 1], c = _v[k + 2];
            int n0 = _nb[k], n1 = _nb[k + 1], n2 = _nb[k + 2];
            bool c0 = _con[k], c1 = _con[k + 1], c2 = _con[k + 2];
            int t1 = _tc++, t2 = _tc++;
            SetTri(t, a, b, pi, n0, t1, t2, c0, false, false);
            SetTri(t1, b, c, pi, n1, t2, t, c1, false, false);
            SetTri(t2, c, a, pi, n2, t, t1, c2, false, false);
            SetNeighborAcross(n1, b, c, t1);
            SetNeighborAcross(n2, c, a, t2);
            _vt[pi] = t;
            stack.Add(t);
            stack.Add(t1);
            stack.Add(t2);
        }
        else
        {
            // p lies on edge `edge` of t: split t and its neighbour u into four triangles.
            int k = 3 * t;
            int a = _v[k + edge], b = _v[k + Next(edge)], c = _v[k + Prev(edge)];
            int nAB = _nb[k + edge];
            int nBC = _nb[k + Next(edge)];
            int nCA = _nb[k + Prev(edge)];
            bool cBC = _con[k + Next(edge)], cCA = _con[k + Prev(edge)];
            int u = nAB;
            if (u < 0)
            {
                throw new GeometryException("Point on the outer border of the bounding triangle.");
            }
            int ku = 3 * u;
            int j = EdgeIndex(u, b, a);
            int d = _v[ku + Prev(j)];
            int nAD = _nb[ku + Next(j)];   // edge (a, d)
            int nDB = _nb[ku + Prev(j)];   // edge (d, b)
            bool cAD = _con[ku + Next(j)], cDB = _con[ku + Prev(j)];
            int t3 = _tc++, t4 = _tc++;
            // t = (a, p, c), t3 = (p, b, c), u = (b, p, d), t4 = (p, a, d)
            SetTri(t, a, pi, c, t4, t3, nCA, false, false, cCA);
            SetTri(t3, pi, b, c, u, nBC, t, false, cBC, false);
            SetTri(u, b, pi, d, t3, t4, nDB, false, false, cDB);
            SetTri(t4, pi, a, d, t, nAD, u, false, cAD, false);
            SetNeighborAcross(nBC, b, c, t3);
            SetNeighborAcross(nAD, a, d, t4);
            _vt[pi] = t;
            stack.Add(t);
            stack.Add(t3);
            stack.Add(u);
            stack.Add(t4);
        }
        _last = t;
        // Legalize the edges opposite pi (Lawson flips).
        while (stack.Count > 0)
        {
            int tt = stack[^1];
            stack.RemoveAt(stack.Count - 1);
            LegalizeEdge(tt, OppositeEdge(tt, pi), stack);
        }
    }

    // Edge of t opposite vertex v (the edge not incident to v).
    private int OppositeEdge(int t, int v) => Next(IndexOf(t, v));

    private void LegalizeEdge(int t, int e, List<int> stack)
    {
        int k = 3 * t;
        if (_con[k + e])
        {
            return;
        }
        int u = _nb[k + e];
        if (u < 0)
        {
            return;
        }
        int a = _v[k + e], b = _v[k + Next(e)], c = _v[k + Prev(e)];
        int j = EdgeIndex(u, b, a);
        int d = _v[(3 * u) + Prev(j)];
        if (Geo.InCircle(_p[a], _p[b], _p[c], _p[d]) > 0)
        {
            Flip(t, e);
            // After the flip, t = (a, d, c) and u = (d, b, c): both contain c == pi; check their outer edges.
            stack.Add(t);
            stack.Add(u);
        }
    }

    // Flips edge e of t with its neighbour. Afterwards t = (a, d, c) and u = (d, b, c).
    private void Flip(int t, int e)
    {
        int k = 3 * t;
        int a = _v[k + e], b = _v[k + Next(e)], c = _v[k + Prev(e)];
        int nBC = _nb[k + Next(e)], nCA = _nb[k + Prev(e)];
        bool cBC = _con[k + Next(e)], cCA = _con[k + Prev(e)];
        int u = _nb[k + e];
        int ku = 3 * u;
        int j = EdgeIndex(u, b, a);
        int d = _v[ku + Prev(j)];
        int nAD = _nb[ku + Next(j)], nDB = _nb[ku + Prev(j)];
        bool cAD = _con[ku + Next(j)], cDB = _con[ku + Prev(j)];
        SetTri(t, a, d, c, nAD, u, nCA, cAD, false, cCA);
        SetTri(u, d, b, c, nDB, nBC, t, cDB, cBC, false);
        SetNeighborAcross(nAD, a, d, t);
        SetNeighborAcross(nBC, b, c, u);
        _last = t;
    }

    // ---- constraints ----

    private (int T, int I) FindEdge(int p, int q)
    {
        // Rotate around p over its incident triangles.
        int start = _vt[p];
        int t = start;
        do
        {
            int i = EdgeIndex(t, p, q);
            if (i >= 0)
            {
                return (t, i);
            }
            int ip = IndexOf(t, p);
            t = _nb[(3 * t) + Prev(ip)]; // across edge (prev, p): next triangle around p
        }
        while (t >= 0 && t != start);
        // The edge may sit on the other side of a border (only for bounding-triangle vertices).
        for (int s = 0; s < _tc; s++)
        {
            int i = EdgeIndex(s, p, q);
            if (i >= 0)
            {
                return (s, i);
            }
        }
        return (-1, -1);
    }

    private void MarkConstrained(int a, int b)
    {
        (int t, int i) = FindEdge(a, b);
        if (t < 0)
        {
            throw new GeometryException("Constraint edge is missing after recovery.");
        }
        _con[(3 * t) + i] = true;
        int u = _nb[(3 * t) + i];
        if (u >= 0)
        {
            _con[(3 * u) + EdgeIndex(u, b, a)] = true;
        }
    }

    private void InsertConstraint(int a, int b)
    {
        if (a == b)
        {
            return;
        }
        if (FindEdge(a, b).T >= 0)
        {
            MarkConstrained(a, b);
            return;
        }
        P pa = _p[a], pb = _p[b];

        // Find the triangle around a that the segment leaves through.
        int start = _vt[a];
        int t = start;
        int foundT = -1;
        do
        {
            int ia = IndexOf(t, a);
            int u = _v[(3 * t) + Next(ia)], w = _v[(3 * t) + Prev(ia)];
            if (Geo.Orient(pa, _p[u], pb) > 0 && Geo.Orient(pa, _p[w], pb) < 0)
            {
                foundT = t;
                break;
            }
            t = _nb[(3 * t) + Prev(ia)];
        }
        while (t >= 0 && t != start);
        if (foundT < 0)
        {
            throw new GeometryException("Constraint is not noded: a vertex lies on a constraint or constraints cross.");
        }

        var crossed = new List<(int P, int Q)>();
        {
            int ia = IndexOf(foundT, a);
            int right = _v[(3 * foundT) + Next(ia)]; // right of a→b
            int left = _v[(3 * foundT) + Prev(ia)];  // left of a→b
            int cur = foundT;
            while (true)
            {
                int ci = EdgeIndex(cur, right, left);
                if (ci < 0)
                {
                    throw new GeometryException("Constraint walk lost its edge.");
                }
                if (_con[(3 * cur) + ci])
                {
                    throw new GeometryException("Constraint crosses another constraint.");
                }
                crossed.Add((right, left));
                int nxt = _nb[(3 * cur) + ci];
                if (nxt < 0)
                {
                    throw new GeometryException("Constraint walk left the triangulation.");
                }
                int ni = EdgeIndex(nxt, left, right);
                int x = _v[(3 * nxt) + Prev(ni)];
                if (x == b)
                {
                    break;
                }
                int s = Geo.Orient(pa, pb, _p[x]);
                if (s > 0)
                {
                    left = x;
                }
                else if (s < 0)
                {
                    right = x;
                }
                else
                {
                    throw new GeometryException("Constraint is not noded: a vertex lies on a constraint.");
                }
                cur = nxt;
            }
        }

        // Sloan's flip loop: flip crossed edges whose quadrilateral is strictly convex until none remain.
        var queue = new Queue<(int P, int Q)>(crossed);
        int guard = 0;
        int limit = 64 + (crossed.Count * crossed.Count * 4);
        while (queue.Count > 0)
        {
            if (++guard > limit + 100000)
            {
                throw new GeometryException("Constraint recovery did not terminate.");
            }
            (int p, int q) = queue.Dequeue();
            (int t1, int i1) = FindEdge(p, q);
            if (t1 < 0)
            {
                throw new GeometryException("Crossed edge vanished during recovery.");
            }
            int k1 = 3 * t1;
            int r = _v[k1 + Prev(i1)];
            int u = _nb[k1 + i1];
            int ju = EdgeIndex(u, q, p);
            int s2 = _v[(3 * u) + Prev(ju)];
            bool convex = Geo.Orient(_p[r], _p[s2], _p[p]) * Geo.Orient(_p[r], _p[s2], _p[q]) < 0;
            if (!convex)
            {
                queue.Enqueue((p, q));
                continue;
            }
            Flip(t1, i1);
            // New edge r–s2.
            if (Geo.Orient(pa, pb, _p[r]) * Geo.Orient(pa, pb, _p[s2]) < 0)
            {
                queue.Enqueue((r, s2));
            }
        }
        MarkConstrained(a, b);
    }

    private void RestoreDelaunay()
    {
        bool flipped = true;
        int passes = 0;
        while (flipped)
        {
            flipped = false;
            if (++passes > 10000)
            {
                throw new GeometryException("Delaunay restoration did not terminate.");
            }
            for (int t = 0; t < _tc; t++)
            {
                for (int e = 0; e < 3; e++)
                {
                    int k = 3 * t;
                    int u = _nb[k + e];
                    if (u < 0 || u < t || _con[k + e])
                    {
                        continue;
                    }
                    int a = _v[k + e], b = _v[k + Next(e)], c = _v[k + Prev(e)];
                    int j = EdgeIndex(u, b, a);
                    int d = _v[(3 * u) + Prev(j)];
                    if (Geo.InCircle(_p[a], _p[b], _p[c], _p[d]) > 0
                        && Geo.Orient(_p[c], _p[d], _p[a]) * Geo.Orient(_p[c], _p[d], _p[b]) < 0)
                    {
                        Flip(t, e);
                        flipped = true;
                    }
                }
            }
        }
    }

    /// <summary>
    /// Winding numbers per triangle: 0 on the unbounded face; crossing a constraint changes the count by the
    /// constraint's net delta. <paramref name="deltaOf"/> returns the net deltas of constraint (u, v) running u→v.
    /// </summary>
    public (int W0, int W1)[] Winding(Func<int, int, (int D0, int D1)> deltaOf)
    {
        var w = new (int, int)[_tc];
        var seen = new bool[_tc];
        var queue = new Queue<int>();
        // The first triangle created touches the bounding triangle's corners: it lies on the unbounded face.
        int root = -1;
        for (int t = 0; t < _tc; t++)
        {
            if (TouchesBoundingTriangle(t))
            {
                root = t;
                break;
            }
        }
        seen[root] = true;
        queue.Enqueue(root);
        while (queue.Count > 0)
        {
            int t = queue.Dequeue();
            for (int e = 0; e < 3; e++)
            {
                int u = _nb[(3 * t) + e];
                if (u < 0 || seen[u])
                {
                    continue;
                }
                var (w0, w1) = w[t];
                if (_con[(3 * t) + e])
                {
                    // Edge runs v[e]→v[e+1] with t on its left; u is on the right: w(left) = w(right) + d.
                    var (d0, d1) = deltaOf(_v[(3 * t) + e], _v[(3 * t) + Next(e)]);
                    w0 -= d0;
                    w1 -= d1;
                }
                w[u] = (w0, w1);
                seen[u] = true;
                queue.Enqueue(u);
            }
        }
        return w;
    }
}
