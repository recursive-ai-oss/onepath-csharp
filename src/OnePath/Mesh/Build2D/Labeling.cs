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
    /// Returns the segments separating inside from outside, directed with the inside on their left. The winding
    /// numbers come from a plane sweep over the non-crossing segments (de Berg et al., "Computational Geometry", ch. 2,
    /// without the intersection events, which noding already removed): the status is the segments cut by the sweep line,
    /// bottom to top, and the winding just above a segment is the winding just below it plus its net delta. The unbounded
    /// face has winding zero. The sweep visits vertices in (x, y) order, which treats vertical segments as if the line were
    /// tilted by an infinitesimal angle; every decision is an exact orientation test.
    /// </summary>
    public static List<(P A, P B)> Boundary(Arrangement arr, BoolOp op)
    {
        var result = new List<(P, P)>();
        int m = arr.SegA.Length;
        if (m == 0)
        {
            return result;
        }
        P[] verts = arr.Verts;
        int[] segA = arr.SegA, segB = arr.SegB;
        // Segments are sorted by (A, B); first[v] .. first[v + 1] are those starting at vertex v.
        var first = new int[verts.Length + 1];
        for (int s = 0; s < m; s++)
        {
            first[segA[s] + 1]++;
        }
        for (int v = 0; v < verts.Length; v++)
        {
            first[v + 1] += first[v];
        }
        var status = new List<int>();
        var up0 = new int[m];
        var up1 = new int[m];
        var starting = new List<int>();
        for (int vi = 0; vi < verts.Length; vi++)
        {
            P v = verts[vi];
            // Segments ending at v are consecutive in the status: below them v is above the segment, on them v is its
            // end, above them v is below the segment.
            int lo = Partition(status, arr, verts, v, strict: false);
            int hi = Partition(status, arr, verts, v, strict: true);
            for (int k = lo; k < hi; k++)
            {
                if (segB[status[k]] != vi)
                {
                    throw new GeometryException("Segment passes through a vertex: the arrangement is not noded.");
                }
            }
            int w0 = 0, w1 = 0;
            if (lo > 0)
            {
                w0 = up0[status[lo - 1]];
                w1 = up1[status[lo - 1]];
            }
            status.RemoveRange(lo, hi - lo);
            starting.Clear();
            for (int s = first[vi]; s < first[vi + 1]; s++)
            {
                starting.Add(s);
            }
            if (starting.Count > 1)
            {
                // Bottom to top: a segment whose direction is counter-clockwise of another's lies above it.
                starting.Sort((x, y) =>
                {
                    Int128 c = Geo.Cross(v, verts[segB[x]], verts[segB[y]]);
                    return c > 0 ? -1 : c < 0 ? 1 : x.CompareTo(y);
                });
            }
            int pos = lo;
            foreach (int s in starting)
            {
                bool leftInside = false;
                int a0 = w0 + arr.D0[s], a1 = w1 + arr.D1[s];
                up0[s] = a0;
                up1[s] = a1;
                leftInside = Inside(op, a0, a1);
                if (leftInside != Inside(op, w0, w1))
                {
                    result.Add(leftInside ? (verts[segA[s]], verts[segB[s]]) : (verts[segB[s]], verts[segA[s]]));
                }
                w0 = a0;
                w1 = a1;
                status.Insert(pos++, s);
            }
        }
        return result;
    }

    // First index of the status whose segment is not below v (strict: not below-or-ending at v), by binary search.
    private static int Partition(List<int> status, Arrangement arr, P[] verts, P v, bool strict)
    {
        int lo = 0, hi = status.Count;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            int s = status[mid];
            int side = Geo.Orient(verts[arr.SegA[s]], verts[arr.SegB[s]], v);
            bool below = strict ? side >= 0 : side > 0;
            if (below)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }
        return lo;
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
