using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>Thrown when an internal invariant of the geometry pipeline fails; this indicates a defect, never bad input.</summary>
public sealed class GeometryException : InvalidOperationException
{
    /// <summary>Creates the exception with a message naming the failed invariant.</summary>
    public GeometryException(string message)
        : base(message)
    {
    }
}

/// <summary>A directed input edge belonging to operand 0 or 1.</summary>
internal readonly struct InEdge
{
    public readonly P A;
    public readonly P B;
    public readonly int Op;

    public InEdge(P a, P b, int op)
    {
        A = a;
        B = b;
        Op = op;
    }
}

/// <summary>
/// A planar arrangement of directed edges: the input split into non-crossing segments between grid points.
/// Each unique segment (vertex indices A &lt; B) carries the net signed count of input edges per operand running A→B.
/// </summary>
/// <remarks>
/// Noding is snap rounding (Hobby 1999; Guibas &amp; Marimont 1998): every vertex and every rounded crossing is a
/// "hot pixel", and each segment is rerouted through the hot pixels its closed unit square-neighbourhood meets. A
/// verification loop then repairs anything the rerouting could still leave (a crossing, or a vertex lying exactly
/// inside a segment) until the result is provably non-crossing.
/// </remarks>
internal sealed class Arrangement
{
    private const int MaxRepairRounds = 256;

    public P[] Verts = Array.Empty<P>();
    public int[] SegA = Array.Empty<int>();
    public int[] SegB = Array.Empty<int>();
    public int[] D0 = Array.Empty<int>();
    public int[] D1 = Array.Empty<int>();

    private struct Frag
    {
        public P A;
        public P B;
        public int Op;
    }

    public static Arrangement Build(List<InEdge> input)
    {
        var frags = new List<Frag>(input.Count);
        foreach (InEdge e in input)
        {
            if (e.A != e.B)
            {
                frags.Add(new Frag { A = e.A, B = e.B, Op = e.Op });
            }
        }
        var result = new Arrangement();
        if (frags.Count == 0)
        {
            return result;
        }

        // Hot pixels: every endpoint, plus the rounded point of every proper crossing among the original edges.
        var hot = new List<P>(frags.Count * 2);
        foreach (Frag f in frags)
        {
            hot.Add(f.A);
            hot.Add(f.B);
        }
        foreach (P c in FindCrossings(frags, null))
        {
            hot.Add(c);
        }
        P[] hotSorted = SortUnique(hot);
        frags = RerouteAll(frags, hotSorted);

        // Repair loop (normally a single verification pass that finds nothing).
        for (int round = 0; ; round++)
        {
            var marked = new bool[frags.Count];
            List<P> crossings = FindCrossings(frags, marked);
            bool changed = crossings.Count > 0;
            if (changed)
            {
                hot.AddRange(crossings);
                hotSorted = SortUnique(hot);
            }
            var next = new List<Frag>(frags.Count + 8);
            for (int i = 0; i < frags.Count; i++)
            {
                Frag f = frags[i];
                if (marked[i])
                {
                    Reroute(f, hotSorted, touching: true, next);
                }
                else if (Reroute(f, hotSorted, touching: false, next))
                {
                    changed = true;
                }
            }
            frags = next;
            if (!changed)
            {
                break;
            }
            if (round >= MaxRepairRounds)
            {
                throw new GeometryException("Snap rounding did not converge after " + MaxRepairRounds + " repair rounds.");
            }
        }

        BuildSegments(frags, result);
        return result;
    }

    private static P[] SortUnique(List<P> pts)
    {
        P[] a = pts.ToArray();
        Array.Sort(a, static (x, y) => x.CompareTo(y));
        int n = 0;
        for (int i = 0; i < a.Length; i++)
        {
            if (i == 0 || a[i] != a[n - 1])
            {
                a[n++] = a[i];
            }
        }
        Array.Resize(ref a, n);
        return a;
    }

    /// <summary>All rounded intersection points of proper crossings. Marks the fragments involved when <paramref name="marked"/> is given.</summary>
    private static List<P> FindCrossings(List<Frag> f, bool[]? marked)
    {
        var result = new List<P>();
        int n = f.Count;
        var order = new int[n];
        for (int i = 0; i < n; i++)
        {
            order[i] = i;
        }
        // Sweep in increasing min-x with an active list; a stable key (min x, then index) fixes the order.
        Array.Sort(order, (i, j) =>
        {
            long a = Math.Min(f[i].A.X, f[i].B.X), b = Math.Min(f[j].A.X, f[j].B.X);
            return a < b ? -1 : a > b ? 1 : i.CompareTo(j);
        });
        var active = new List<int>();
        foreach (int i in order)
        {
            Frag s = f[i];
            long minX = Math.Min(s.A.X, s.B.X), maxX = Math.Max(s.A.X, s.B.X);
            long minY = Math.Min(s.A.Y, s.B.Y), maxY = Math.Max(s.A.Y, s.B.Y);
            int keep = 0;
            for (int k = 0; k < active.Count; k++)
            {
                int j = active[k];
                Frag t = f[j];
                if (Math.Max(t.A.X, t.B.X) < minX)
                {
                    continue; // expired: ends left of everything still to come
                }
                active[keep++] = j;
                if (Math.Max(t.A.Y, t.B.Y) < minY || Math.Min(t.A.Y, t.B.Y) > maxY)
                {
                    continue;
                }
                if (Geo.ProperCrossing(s.A, s.B, t.A, t.B))
                {
                    P? x = Geo.RoundedIntersection(s.A, s.B, t.A, t.B);
                    if (x is P px)
                    {
                        result.Add(px);
                        if (marked != null)
                        {
                            marked[i] = true;
                            marked[j] = true;
                        }
                    }
                }
            }
            active.RemoveRange(keep, active.Count - keep);
            active.Add(i);
            _ = maxX;
        }
        return result;
    }

    private static List<Frag> RerouteAll(List<Frag> frags, P[] hot)
    {
        var next = new List<Frag>(frags.Count + 16);
        foreach (Frag f in frags)
        {
            Reroute(f, hot, touching: true, next);
        }
        return next;
    }

    // Index range [lo, hi) of hot pixels whose x lies in [minX, maxX].
    private static void XRange(P[] hot, long minX, long maxX, out int lo, out int hi)
    {
        int l = 0, h = hot.Length;
        while (l < h)
        {
            int m = (l + h) >> 1;
            if (hot[m].X < minX) { l = m + 1; } else { h = m; }
        }
        lo = l;
        h = hot.Length;
        while (l < h)
        {
            int m = (l + h) >> 1;
            if (hot[m].X <= maxX) { l = m + 1; } else { h = m; }
        }
        hi = l;
    }

    /// <summary>
    /// Appends the polyline replacing <paramref name="f"/> to <paramref name="output"/>. With <paramref name="touching"/>
    /// the path visits every hot pixel whose closed square the segment meets (snap rounding); otherwise only hot
    /// vertices lying exactly inside the segment (a pure split, geometry unchanged). Returns true if it was split.
    /// </summary>
    private static bool Reroute(Frag f, P[] hot, bool touching, List<Frag> output)
    {
        long minX = Math.Min(f.A.X, f.B.X), maxX = Math.Max(f.A.X, f.B.X);
        long minY = Math.Min(f.A.Y, f.B.Y), maxY = Math.Max(f.A.Y, f.B.Y);
        int pad = touching ? 1 : 0;
        XRange(hot, minX - pad, maxX + pad, out int lo, out int hi);
        List<(P Pt, Int128 T, Int128 Raw)>? via = null;
        Int128 len2 = Geo.LengthSquared(f.A, f.B);
        for (int k = lo; k < hi; k++)
        {
            P c = hot[k];
            if (c.Y < minY - pad || c.Y > maxY + pad || c == f.A || c == f.B)
            {
                continue;
            }
            bool hit = touching ? Geo.SegmentTouchesPixel(f.A, f.B, c) : Geo.StrictlyInside(f.A, f.B, c);
            if (!hit)
            {
                continue;
            }
            Int128 raw = Geo.Dot(f.A, c, f.B);
            Int128 t = raw <= 0 ? 0 : raw >= len2 ? len2 : raw;
            (via ??= new List<(P, Int128, Int128)>()).Add((c, t, raw));
        }
        if (via == null)
        {
            output.Add(f);
            return false;
        }
        via.Sort(static (x, y) =>
        {
            int c = x.T.CompareTo(y.T);
            if (c != 0)
            {
                return c;
            }
            c = x.Raw.CompareTo(y.Raw);
            return c != 0 ? c : x.Pt.CompareTo(y.Pt);
        });
        P prev = f.A;
        foreach (var (pt, _, _) in via)
        {
            if (pt != prev)
            {
                output.Add(new Frag { A = prev, B = pt, Op = f.Op });
                prev = pt;
            }
        }
        if (prev != f.B)
        {
            output.Add(new Frag { A = prev, B = f.B, Op = f.Op });
        }
        return true;
    }

    private static void BuildSegments(List<Frag> frags, Arrangement r)
    {
        var pts = new List<P>(frags.Count * 2);
        foreach (Frag f in frags)
        {
            pts.Add(f.A);
            pts.Add(f.B);
        }
        P[] verts = SortUnique(pts);
        int n = frags.Count;
        var keys = new (long Key, int D0, int D1)[n];
        for (int i = 0; i < n; i++)
        {
            Frag f = frags[i];
            int a = Array.BinarySearch(verts, f.A, Comparer<P>.Create(static (x, y) => x.CompareTo(y)));
            int b = Array.BinarySearch(verts, f.B, Comparer<P>.Create(static (x, y) => x.CompareTo(y)));
            int sign = 1;
            if (a > b)
            {
                (a, b) = (b, a);
                sign = -1;
            }
            keys[i] = (((long)a << 32) | (uint)b, f.Op == 0 ? sign : 0, f.Op == 1 ? sign : 0);
        }
        Array.Sort(keys, static (x, y) => x.Key.CompareTo(y.Key));
        var a2 = new List<int>();
        var b2 = new List<int>();
        var d0 = new List<int>();
        var d1 = new List<int>();
        for (int i = 0; i < n;)
        {
            long key = keys[i].Key;
            int s0 = 0, s1 = 0;
            while (i < n && keys[i].Key == key)
            {
                s0 += keys[i].D0;
                s1 += keys[i].D1;
                i++;
            }
            if (s0 != 0 || s1 != 0)
            {
                a2.Add((int)(key >> 32));
                b2.Add((int)(key & 0xFFFFFFFFL));
                d0.Add(s0);
                d1.Add(s1);
            }
        }
        // Renumber so only vertices still carrying a segment remain (order preserved).
        var used = new bool[verts.Length];
        foreach (int a in a2) { used[a] = true; }
        foreach (int b in b2) { used[b] = true; }
        var map = new int[verts.Length];
        var kept = new List<P>();
        for (int i = 0; i < verts.Length; i++)
        {
            if (used[i])
            {
                map[i] = kept.Count;
                kept.Add(verts[i]);
            }
        }
        r.Verts = kept.ToArray();
        r.SegA = new int[a2.Count];
        r.SegB = new int[a2.Count];
        for (int i = 0; i < a2.Count; i++)
        {
            r.SegA[i] = map[a2[i]];
            r.SegB[i] = map[b2[i]];
        }
        r.D0 = d0.ToArray();
        r.D1 = d1.ToArray();
    }
}
