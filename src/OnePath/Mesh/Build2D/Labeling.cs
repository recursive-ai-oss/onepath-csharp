using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>How the winding numbers of the operands decide whether a face is inside the result.</summary>
internal enum BoolOp
{
    /// <summary>Operand 0 winding is positive.</summary>
    Positive,

    /// <summary>Operand 0 winding is odd (even-odd fill; any ring orientation).</summary>
    EvenOdd,

    /// <summary>Either operand winding is positive.</summary>
    Union,

    /// <summary>Both operand windings are positive.</summary>
    Intersection,

    /// <summary>Operand 0 positive and operand 1 not.</summary>
    Difference,

    /// <summary>Exactly one operand positive.</summary>
    Xor,
}

/// <summary>Labelling of an arrangement's faces and extraction of the boundary and its rings.</summary>
internal static class Labeling
{
    public static bool Inside(BoolOp op, int w0, int w1) => op switch
    {
        BoolOp.Positive => w0 > 0,
        BoolOp.EvenOdd => (w0 & 1) != 0,
        BoolOp.Union => w0 > 0 || w1 > 0,
        BoolOp.Intersection => w0 > 0 && w1 > 0,
        BoolOp.Difference => w0 > 0 && w1 <= 0,
        _ => (w0 > 0) != (w1 > 0),
    };

    /// <summary>
    /// Triangulates the arrangement's constraints, derives each face's winding by flooding from the unbounded
    /// face, and returns the segments separating inside from outside, directed with the inside on their left.
    /// </summary>
    public static List<(P A, P B)> Boundary(Arrangement arr, BoolOp op)
    {
        var result = new List<(P, P)>();
        if (arr.SegA.Length == 0)
        {
            return result;
        }
        var constraints = new List<(int, int)>(arr.SegA.Length);
        var index = new Dictionary<long, int>(arr.SegA.Length);
        for (int i = 0; i < arr.SegA.Length; i++)
        {
            constraints.Add((arr.SegA[i], arr.SegB[i]));
            index[((long)arr.SegA[i] << 32) | (uint)arr.SegB[i]] = i;
        }
        var cdt = new Cdt(arr.Verts, constraints);
        (int, int)[] w = cdt.Winding((u, v) =>
        {
            if (u < v)
            {
                int s = index[((long)u << 32) | (uint)v];
                return (arr.D0[s], arr.D1[s]);
            }
            int s2 = index[((long)v << 32) | (uint)u];
            return (-arr.D0[s2], -arr.D1[s2]);
        });
        for (int t = 0; t < cdt.TriangleCount; t++)
        {
            for (int e = 0; e < 3; e++)
            {
                if (!cdt.IsConstrained(t, e))
                {
                    continue;
                }
                int u = cdt.Vertex(t, e), v = cdt.Vertex(t, (e + 1) % 3);
                if (u > v)
                {
                    continue; // each constraint is seen from both sides; take the u < v view
                }
                int nb = cdt.Neighbor(t, e);
                bool left = Inside(op, w[t].Item1, w[t].Item2);
                bool right = Inside(op, w[nb].Item1, w[nb].Item2);
                if (left != right)
                {
                    result.Add(left ? (arr.Verts[u], arr.Verts[v]) : (arr.Verts[v], arr.Verts[u]));
                }
            }
        }
        return result;
    }

    /// <summary>
    /// Chains directed boundary edges into rings. At a vertex where several rings touch, each ring takes the
    /// outgoing edge first met turning clockwise from the way back along its incoming edge, so rings are the
    /// minimal wedges of the region and never cross. Collinear degree-2 vertices are dropped. Rings start at their
    /// smallest edge, and the rings are ordered by it: the result depends only on the edge set.
    /// </summary>
    public static List<P[]> TraceRings(List<(P A, P B)> edges)
    {
        var rings = new List<P[]>();
        int m = edges.Count;
        if (m == 0)
        {
            return rings;
        }
        var sorted = edges.ToArray();
        Array.Sort(sorted, static (x, y) =>
        {
            int c = x.A.CompareTo(y.A);
            return c != 0 ? c : x.B.CompareTo(y.B);
        });
        // first[i]: index of the first edge leaving vertex-group i; edges with equal A are contiguous.
        var used = new bool[m];
        for (int s = 0; s < m; s++)
        {
            if (used[s])
            {
                continue;
            }
            var ring = new List<P>();
            int cur = s;
            while (!used[cur])
            {
                used[cur] = true;
                ring.Add(sorted[cur].A);
                cur = Successor(sorted, cur);
            }
            if (cur != s)
            {
                throw new GeometryException("Boundary edges do not form closed rings.");
            }
            rings.Add(Simplify(ring, sorted));
        }
        rings.RemoveAll(static r => r.Length < 3);
        return rings;
    }

    private static int FirstEdgeFrom(P[] _, (P A, P B)[] sorted, P v)
    {
        int lo = 0, hi = sorted.Length;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            if (sorted[mid].A.CompareTo(v) < 0) { lo = mid + 1; } else { hi = mid; }
        }
        return lo;
    }

    private static int Successor((P A, P B)[] sorted, int e)
    {
        P a = sorted[e].A, v = sorted[e].B;
        int first = FirstEdgeFrom(Array.Empty<P>(), sorted, v);
        int count = 0;
        while (first + count < sorted.Length && sorted[first + count].A == v)
        {
            count++;
        }
        if (count == 0)
        {
            throw new GeometryException("Boundary edge ends at a vertex with no outgoing edge.");
        }
        if (count == 1)
        {
            return first;
        }
        long rx = a.X - v.X, ry = a.Y - v.Y;
        int best = first;
        for (int k = first + 1; k < first + count; k++)
        {
            if (ClockwiseBefore(rx, ry, sorted[k].B.X - v.X, sorted[k].B.Y - v.Y, sorted[best].B.X - v.X, sorted[best].B.Y - v.Y))
            {
                best = k;
            }
        }
        return best;
    }

    // Is direction c1 met before c2 when turning clockwise from reference r?
    private static bool ClockwiseBefore(long rx, long ry, long c1x, long c1y, long c2x, long c2y)
    {
        int g1 = Group(rx, ry, c1x, c1y), g2 = Group(rx, ry, c2x, c2y);
        if (g1 != g2)
        {
            return g1 < g2;
        }
        Int128 cr = ((Int128)c1x * c2y) - ((Int128)c1y * c2x);
        return cr < 0;
    }

    private static int Group(long rx, long ry, long cx, long cy)
    {
        Int128 cr = ((Int128)rx * cy) - ((Int128)ry * cx);
        return cr < 0 ? 0 : cr == 0 ? 1 : 2;
    }

    private static P[] Simplify(List<P> ring, (P A, P B)[] sorted)
    {
        int n = ring.Count;
        var keep = new List<P>(n);
        for (int i = 0; i < n; i++)
        {
            P prev = ring[(i + n - 1) % n], v = ring[i], next = ring[(i + 1) % n];
            int first = FirstEdgeFrom(Array.Empty<P>(), sorted, v);
            bool single = first + 1 >= sorted.Length || sorted[first + 1].A != v;
            if (single && Geo.Orient(prev, v, next) == 0 && Geo.Dot(v, prev, next) < 0)
            {
                continue; // straight pass-through: prev→v and v→next point the same way
            }
            keep.Add(v);
        }
        return keep.ToArray();
    }
}
