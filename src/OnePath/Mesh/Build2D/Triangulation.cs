using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>
/// A triangulation of a <see cref="Region"/>: counter-clockwise triangles with adjacency. Vertices are sorted by
/// (x, y); each triangle starts at its smallest vertex index and triangles are sorted by their index triple, so the
/// arrays depend only on the region.
/// </summary>
public sealed class Triangulation
{
    private readonly Vector2D[] _vertices;
    private readonly int[] _indices;
    private readonly int[] _neighbors;

    private Triangulation(Vector2D[] vertices, int[] indices, int[] neighbors)
    {
        _vertices = vertices;
        _indices = indices;
        _neighbors = neighbors;
    }

    /// <summary>Vertex positions (exact on the snap grid).</summary>
    public ReadOnlySpan<Vector2D> Vertices => _vertices;

    /// <summary>Three vertex indices per triangle, counter-clockwise.</summary>
    public ReadOnlySpan<int> Indices => _indices;

    /// <summary>
    /// Three entries per triangle: entry <c>3t + i</c> is the triangle across the edge from vertex <c>i</c> to vertex
    /// <c>(i + 1) % 3</c> of triangle <c>t</c>, or -1 when that edge lies on the region boundary.
    /// </summary>
    public ReadOnlySpan<int> Neighbors => _neighbors;

    /// <summary>Number of triangles.</summary>
    public int TriangleCount => _indices.Length / 3;

    internal static Triangulation CreateEmpty() => new(Array.Empty<Vector2D>(), Array.Empty<int>(), Array.Empty<int>());

    internal static Triangulation FromCdt(Cdt cdt, (int, int)[] winding, P[] verts, int bits)
    {
        var tris = new List<(int A, int B, int C)>();
        for (int t = 0; t < cdt.TriangleCount; t++)
        {
            if (winding[t].Item1 <= 0 || cdt.TouchesBoundingTriangle(t))
            {
                continue;
            }
            int a = cdt.Vertex(t, 0), b = cdt.Vertex(t, 1), c = cdt.Vertex(t, 2);
            if (b < a && b < c)
            {
                (a, b, c) = (b, c, a);
            }
            else if (c < a && c < b)
            {
                (a, b, c) = (c, a, b);
            }
            tris.Add((a, b, c));
        }
        tris.Sort(static (x, y) =>
        {
            int r = x.A.CompareTo(y.A);
            if (r != 0) { return r; }
            r = x.B.CompareTo(y.B);
            return r != 0 ? r : x.C.CompareTo(y.C);
        });
        var indices = new int[tris.Count * 3];
        var edge = new Dictionary<long, int>(tris.Count * 3);
        for (int t = 0; t < tris.Count; t++)
        {
            indices[3 * t] = tris[t].A;
            indices[(3 * t) + 1] = tris[t].B;
            indices[(3 * t) + 2] = tris[t].C;
            for (int i = 0; i < 3; i++)
            {
                edge[((long)indices[(3 * t) + i] << 32) | (uint)indices[(3 * t) + ((i + 1) % 3)]] = t;
            }
        }
        var nb = new int[indices.Length];
        for (int t = 0; t < tris.Count; t++)
        {
            for (int i = 0; i < 3; i++)
            {
                long rev = ((long)indices[(3 * t) + ((i + 1) % 3)] << 32) | (uint)indices[(3 * t) + i];
                nb[(3 * t) + i] = edge.TryGetValue(rev, out int u) ? u : -1;
            }
        }
        return new Triangulation(Region.ToPublic(verts, bits), indices, nb);
    }
}

/// <summary>
/// A partition of a region into convex polygons with adjacency: the plain, documented hand-off between the 2D
/// geometry stages and the navigation mesh. Polygons are counter-clockwise and weakly convex (a vertex on a straight
/// edge is kept where a neighbouring polygon needs it). Each polygon starts at its smallest vertex index, and
/// polygons are sorted by their index lists; the result depends only on the region and the vertex limit.
/// </summary>
public sealed class ConvexPartition
{
    private readonly Vector2D[] _vertices;
    private readonly int[] _start;
    private readonly int[] _indices;
    private readonly int[] _neighbors;

    private ConvexPartition(Vector2D[] vertices, int[] start, int[] indices, int[] neighbors)
    {
        _vertices = vertices;
        _start = start;
        _indices = indices;
        _neighbors = neighbors;
    }

    /// <summary>Vertex positions (exact on the snap grid), sorted by (x, y).</summary>
    public ReadOnlySpan<Vector2D> Vertices => _vertices;

    /// <summary>Number of polygons.</summary>
    public int PolygonCount => _start.Length - 1;

    /// <summary>The vertex indices of polygon <paramref name="polygon"/>, counter-clockwise.</summary>
    public ReadOnlySpan<int> GetIndices(int polygon) => _indices.AsSpan(_start[polygon], _start[polygon + 1] - _start[polygon]);

    /// <summary>
    /// For each edge <c>i</c> of the polygon (from vertex <c>i</c> to <c>(i + 1) % n</c>): the polygon across it, or -1
    /// on the region boundary. When several polygons border the edge's line only one shares the whole edge, so an
    /// edge has at most one neighbour.
    /// </summary>
    public ReadOnlySpan<int> GetNeighbors(int polygon) => _neighbors.AsSpan(_start[polygon], _start[polygon + 1] - _start[polygon]);

    internal static ConvexPartition Create(Triangulation tri, int maxVertices)
    {
        if (maxVertices < 3 || maxVertices > 16)
        {
            throw new ArgumentOutOfRangeException(nameof(maxVertices), maxVertices, "The polygon vertex limit must be between 3 and 16.");
        }
        ReadOnlySpan<Vector2D> vv = tri.Vertices;
        ReadOnlySpan<int> idx = tri.Indices;
        ReadOnlySpan<int> nbr = tri.Neighbors;
        int nt = tri.TriangleCount;
        var polys = new List<int>?[nt]; // polygon vertex lists, null once merged into another
        var owner = new int[nt];
        for (int t = 0; t < nt; t++)
        {
            polys[t] = new List<int> { idx[3 * t], idx[(3 * t) + 1], idx[(3 * t) + 2] };
            owner[t] = t;
        }
        // Candidate diagonals: each interior edge once, longest first (Hertel & Mehlhorn 1983, greedy order), ties by index.
        var cand = new List<(double Len, int A, int B)>();
        for (int t = 0; t < nt; t++)
        {
            for (int i = 0; i < 3; i++)
            {
                int u = nbr[(3 * t) + i];
                if (u > t)
                {
                    int a = idx[(3 * t) + i], b = idx[(3 * t) + ((i + 1) % 3)];
                    double dx = vv[b].X - vv[a].X, dy = vv[b].Y - vv[a].Y;
                    cand.Add(((dx * dx) + (dy * dy), a, b));
                }
            }
        }
        cand.Sort(static (x, y) =>
        {
            int c = y.Len.CompareTo(x.Len);
            if (c != 0) { return c; }
            c = x.A.CompareTo(y.A);
            return c != 0 ? c : x.B.CompareTo(y.B);
        });
        var find = new int[nt];
        for (int t = 0; t < nt; t++) { find[t] = t; }
        int Root(int x)
        {
            while (find[x] != x)
            {
                find[x] = find[find[x]];
                x = find[x];
            }
            return x;
        }
        // Edge → the triangle on its left, to locate both polygons of a diagonal.
        var leftOf = new Dictionary<long, int>(nt * 3);
        for (int t = 0; t < nt; t++)
        {
            for (int i = 0; i < 3; i++)
            {
                leftOf[((long)idx[(3 * t) + i] << 32) | (uint)idx[(3 * t) + ((i + 1) % 3)]] = t;
            }
        }
        foreach (var c in cand)
        {
            int pa = Root(leftOf[((long)c.A << 32) | (uint)c.B]);
            int pb = Root(leftOf[((long)c.B << 32) | (uint)c.A]);
            if (pa == pb)
            {
                continue;
            }
            List<int> P = polys[pa]!, Q = polys[pb]!;
            if (P.Count + Q.Count - 2 > maxVertices)
            {
                continue;
            }
            // Polygon P runs a→b somewhere, Q runs b→a.
            int a = c.A, b = c.B;
            int ia = IndexOfEdge(P, a, b);
            int jb = IndexOfEdge(Q, b, a);
            if (ia < 0 || jb < 0)
            {
                throw new GeometryException("Diagonal is missing from a polygon.");
            }
            int pn = P.Count, qn = Q.Count;
            if (SharesOtherVertex(P, Q, a, b))
            {
                continue; // the polygons touch elsewhere: merging would repeat a vertex
            }
            // Corner at a: P's vertex before a, a, Q's vertex after a (in Q: b, a, next).
            int pPrevA = P[(ia + pn - 1) % pn];
            int qNextA = Q[(jb + 2) % qn];
            // Corner at b: Q's vertex before b, b, P's vertex after b.
            int qPrevB = Q[(jb + qn - 1) % qn];
            int pNextB = P[(ia + 2) % pn];
            if (CornerOrient(vv, pPrevA, a, qNextA) < 0 || CornerOrient(vv, qPrevB, b, pNextB) < 0)
            {
                continue;
            }
            var merged = new List<int>(pn + qn - 2);
            // From b around P back to a, then Q from after a to before b.
            for (int k = 1; k <= pn; k++) { merged.Add(P[(ia + k) % pn]); }
            for (int k = 2; k < qn; k++) { merged.Add(Q[(jb + k) % qn]); }
            // merged now starts at b (P[ia+1]) ... a (P[ia]) then Q's interior vertices: drop the duplicate join.
            polys[pa] = merged;
            polys[pb] = null;
            find[pb] = pa;
        }

        // Collect, rotate each polygon to start at its smallest index, sort.
        var list = new List<int[]>();
        for (int t = 0; t < nt; t++)
        {
            List<int>? p = polys[t];
            if (p == null || Root(t) != t)
            {
                continue;
            }
            int s = 0;
            for (int k = 1; k < p.Count; k++)
            {
                if (p[k] < p[s]) { s = k; }
            }
            var r = new int[p.Count];
            for (int k = 0; k < r.Length; k++) { r[k] = p[(s + k) % r.Length]; }
            list.Add(r);
        }
        list.Sort(static (x, y) =>
        {
            int n = Math.Min(x.Length, y.Length);
            for (int k = 0; k < n; k++)
            {
                int c = x[k].CompareTo(y[k]);
                if (c != 0) { return c; }
            }
            return x.Length.CompareTo(y.Length);
        });
        var start = new int[list.Count + 1];
        var indices = new List<int>();
        for (int i = 0; i < list.Count; i++)
        {
            start[i] = indices.Count;
            indices.AddRange(list[i]);
        }
        start[list.Count] = indices.Count;
        var edgeOwner = new Dictionary<long, int>(indices.Count);
        for (int i = 0; i < list.Count; i++)
        {
            int[] r = list[i];
            for (int k = 0; k < r.Length; k++)
            {
                edgeOwner[((long)r[k] << 32) | (uint)r[(k + 1) % r.Length]] = i;
            }
        }
        var neighbors = new int[indices.Count];
        for (int i = 0; i < list.Count; i++)
        {
            int[] r = list[i];
            for (int k = 0; k < r.Length; k++)
            {
                long rev = ((long)r[(k + 1) % r.Length] << 32) | (uint)r[k];
                neighbors[start[i] + k] = edgeOwner.TryGetValue(rev, out int o) ? o : -1;
            }
        }
        return new ConvexPartition(vv.ToArray(), start, indices.ToArray(), neighbors);
    }

    private static int IndexOfEdge(List<int> poly, int a, int b)
    {
        int n = poly.Count;
        for (int i = 0; i < n; i++)
        {
            if (poly[i] == a && poly[(i + 1) % n] == b)
            {
                return i;
            }
        }
        return -1;
    }

    // Sign of the turn p→v→q using the exact predicate on the doubles (grid multiples, exact in double).
    private static int CornerOrient(ReadOnlySpan<Vector2D> v, int p, int c, int q) =>
        OneMath.Predicates.ExactPredicates.Orient2D(v[p].X, v[p].Y, v[c].X, v[c].Y, v[q].X, v[q].Y);

    private static bool SharesOtherVertex(List<int> p, List<int> q, int a, int b)
    {
        foreach (int x in q)
        {
            if (x != a && x != b && p.Contains(x))
            {
                return true;
            }
        }
        return false;
    }
}
