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

/// <summary>A directed input edge belonging to operand 0, operand 1, or (Op 2) both at once.</summary>
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
        var hg0 = HotGrid.Build(hotSorted, MeanSpan(frags));
        frags = RerouteAll(frags, hg0);

        // Repair loop (normally a single verification pass that finds nothing).
        HotGrid lastGrid = hg0;
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
            HotGrid grid = HotGrid.Build(hotSorted, MeanSpan(frags));
            lastGrid = grid;
            for (int i = 0; i < frags.Count; i++)
            {
                Frag f = frags[i];
                if (marked[i])
                {
                    Reroute(f, grid, touching: true, next);
                }
                else if (Reroute(f, grid, touching: false, next))
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

        BuildSegments(frags, hotSorted, lastGrid, result);
        return result;
    }

    private static P[] SortUnique(List<P> pts)
    {
        int cnt = pts.Count;
        long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
        foreach (P p in pts)
        {
            minX = Math.Min(minX, p.X);
            maxX = Math.Max(maxX, p.X);
            minY = Math.Min(minY, p.Y);
            maxY = Math.Max(maxY, p.Y);
        }
        int yBits = 64 - System.Numerics.BitOperations.LeadingZeroCount((ulong)Math.Max(1, maxY - minY));
        int xBits = 64 - System.Numerics.BitOperations.LeadingZeroCount((ulong)Math.Max(1, maxX - minX));
        P[] a;
        if (cnt > 0 && xBits + yBits <= 63)
        {
            // Both coordinates packed into one unsigned key whose order is the (x, y) order; sorting primitives is faster.
            var keys = new ulong[cnt];
            for (int i = 0; i < cnt; i++)
            {
                P p = pts[i];
                keys[i] = ((ulong)(p.X - minX) << yBits) | (ulong)(p.Y - minY);
            }
            Array.Sort(keys);
            ulong mask = (1UL << yBits) - 1;
            a = new P[cnt];
            int u = 0;
            for (int i = 0; i < cnt; i++)
            {
                if (i == 0 || keys[i] != keys[i - 1])
                {
                    a[u++] = new P(minX + (long)(keys[i] >> yBits), minY + (long)(keys[i] & mask));
                }
            }
            Array.Resize(ref a, u);
            return a;
        }
        a = pts.ToArray();
        Array.Sort(a);
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

    // Mean of max(|dx|, |dy|) over the fragments: the typical cell size for the bucket grids.
    private static long MeanSpan(List<Frag> f)
    {
        if (f.Count == 0)
        {
            return 2;
        }
        Int128 sum = 0;
        foreach (Frag s in f)
        {
            sum += Math.Max(Math.Abs(s.B.X - s.A.X), Math.Abs(s.B.Y - s.A.Y));
        }
        return (long)(sum / f.Count);
    }

    /// <summary>
    /// All rounded intersection points of proper crossings. Marks the fragments involved when <paramref name="marked"/> is given.
    /// Fragments are bucketed into a uniform grid and only fragments sharing a cell are tested; a pair sharing several
    /// cells is tested more than once, which only repeats a point already in the result.
    /// </summary>
    private static List<P> FindCrossings(List<Frag> f, bool[]? marked)
    {
        var result = new List<P>();
        int n = f.Count;
        if (n < 2)
        {
            return result;
        }
        var bx0 = new long[n];
        var bx1 = new long[n];
        var by0 = new long[n];
        var by1 = new long[n];
        long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
        for (int i = 0; i < n; i++)
        {
            Frag s = f[i];
            bx0[i] = Math.Min(s.A.X, s.B.X);
            bx1[i] = Math.Max(s.A.X, s.B.X);
            by0[i] = Math.Min(s.A.Y, s.B.Y);
            by1[i] = Math.Max(s.A.Y, s.B.Y);
            minX = Math.Min(minX, bx0[i]);
            maxX = Math.Max(maxX, bx1[i]);
            minY = Math.Min(minY, by0[i]);
            maxY = Math.Max(maxY, by1[i]);
        }
        CellGrid g = CellGrid.Create(minX, minY, maxX, maxY, MeanSpan(f), 2L * n);
        var lo = new int[g.Cols + 1];
        var hi = new int[g.Cols + 1];
        var start = new int[g.CellCount + 1];
        for (int pass = 0; pass < 2; pass++)
        {
            if (pass == 1)
            {
                // The pairs a cell tests grow with the square of its count. Clustered input (a few huge segments stretching
                // the bounding box over many tiny ones) piles everything into a few cells; the sweep copes with that better.
                long pairs = 0;
                for (int c = 0; c < start.Length - 1; c++)
                {
                    pairs += (long)start[c] * start[c];
                }
                if (pairs > UniformLimit * n)
                {
                    return FindCrossingsSweep(f, marked);
                }
                // Prefix sums turn the counts into start offsets; the second pass fills each cell's list.
                int sum = 0;
                for (int c = 0; c < start.Length - 1; c++)
                {
                    int cnt = start[c];
                    start[c] = sum;
                    sum += cnt;
                }
                start[^1] = sum;
            }
            int[]? fill = pass == 1 ? (int[])start.Clone() : null;
            int[]? ids = pass == 1 ? new int[start[^1]] : null;
            for (int i = 0; i < n; i++)
            {
                if (g.BoxCells(bx0[i], by0[i], bx1[i], by1[i], 4, out int k0, out int k1, out int q0, out int q1))
                {
                    // A short fragment: the cells of its bounding box include every cell it passes through, and there are few.
                    for (int r = q0; r <= q1; r++)
                    {
                        for (int c = k0; c <= k1; c++)
                        {
                            int cell = (r * g.Cols) + c;
                            if (pass == 0)
                            {
                                start[cell]++;
                            }
                            else
                            {
                                ids![fill![cell]++] = i;
                            }
                        }
                    }
                    continue;
                }
                g.Traverse(f[i].A, f[i].B, lo, hi, out int c0, out int c1);
                for (int c = c0; c <= c1; c++)
                {
                    for (int r = lo[c - c0]; r <= hi[c - c0]; r++)
                    {
                        int cell = (r * g.Cols) + c;
                        if (pass == 0)
                        {
                            start[cell]++;
                        }
                        else
                        {
                            ids![fill![cell]++] = i;
                        }
                    }
                }
            }
            if (pass == 1)
            {
                for (int cell = 0; cell < g.CellCount; cell++)
                {
                    for (int a = start[cell]; a < start[cell + 1]; a++)
                    {
                        int i = ids![a];
                        for (int b = a + 1; b < start[cell + 1]; b++)
                        {
                            int j = ids[b];
                            if (bx1[j] < bx0[i] || bx0[j] > bx1[i] || by1[j] < by0[i] || by0[j] > by1[i])
                            {
                                continue;
                            }
                            Frag s = f[i], t = f[j];
                            if (s.A == t.A || s.A == t.B || s.B == t.A || s.B == t.B)
                            {
                                continue; // segments sharing an endpoint cannot cross in their interiors
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
                    }
                }
            }
        }
        return result;
    }

    // Largest mean number of same-cell partners per item for which the bucket grids are used.
    private const long UniformLimit = 32;

    // Sweep in increasing min-x with an active list; a stable key (min x, then index) fixes the order. Quadratic in the
    // worst case but indifferent to how the input is spread over the plane.
    private static List<P> FindCrossingsSweep(List<Frag> f, bool[]? marked)
    {
        var result = new List<P>();
        int n = f.Count;
        var order = new int[n];
        for (int i = 0; i < n; i++)
        {
            order[i] = i;
        }
        Array.Sort(order, (i, j) =>
        {
            long a = Math.Min(f[i].A.X, f[i].B.X), b = Math.Min(f[j].A.X, f[j].B.X);
            return a < b ? -1 : a > b ? 1 : i.CompareTo(j);
        });
        var active = new List<int>();
        foreach (int i in order)
        {
            Frag s = f[i];
            long minX = Math.Min(s.A.X, s.B.X);
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
        }
        return result;
    }

    private static List<Frag> RerouteAll(List<Frag> frags, HotGrid hot)
    {
        var next = new List<Frag>(frags.Count + 16);
        foreach (Frag f in frags)
        {
            Reroute(f, hot, touching: true, next);
        }
        return next;
    }

    /// <summary>The hot pixels bucketed by cell, for finding those near a segment.</summary>
    private sealed class HotGrid
    {
        public CellGrid Grid = null!;
        public int[] Start = Array.Empty<int>();
        public P[] Pts = Array.Empty<P>();
        public int[] Ids = Array.Empty<int>();
        public int[] Lo = Array.Empty<int>();
        public int[] Hi = Array.Empty<int>();

        /// <summary>The hot pixels in (x, y) order, for the fallback search.</summary>
        public P[] Sorted = Array.Empty<P>();

        /// <summary>False when the points crowd into a few cells, so searching by x range is the better way.</summary>
        public bool Usable;

        public static HotGrid Build(P[] hot, long meanSpan)
        {
            var h = new HotGrid { Sorted = hot };
            long minX = long.MaxValue, minY = long.MaxValue, maxX = long.MinValue, maxY = long.MinValue;
            foreach (P p in hot)
            {
                minX = Math.Min(minX, p.X);
                maxX = Math.Max(maxX, p.X);
                minY = Math.Min(minY, p.Y);
                maxY = Math.Max(maxY, p.Y);
            }
            h.Grid = CellGrid.Create(minX, minY, maxX, maxY, meanSpan, 2L * hot.Length);
            h.Lo = new int[h.Grid.Cols + 1];
            h.Hi = new int[h.Grid.Cols + 1];
            h.Start = new int[h.Grid.CellCount + 1];
            foreach (P p in hot)
            {
                h.Start[Cell(h.Grid, p)]++;
            }
            long pairs = 0;
            for (int c = 0; c < h.Start.Length - 1; c++)
            {
                pairs += (long)h.Start[c] * h.Start[c];
            }
            h.Usable = pairs <= UniformLimit * hot.Length;
            if (!h.Usable)
            {
                return h;
            }
            int sum = 0;
            for (int c = 0; c < h.Start.Length - 1; c++)
            {
                int cnt = h.Start[c];
                h.Start[c] = sum;
                sum += cnt;
            }
            h.Start[^1] = sum;
            var fill = (int[])h.Start.Clone();
            h.Pts = new P[hot.Length];
            h.Ids = new int[hot.Length];
            for (int i = 0; i < hot.Length; i++)
            {
                int pos = fill[Cell(h.Grid, hot[i])]++;
                h.Pts[pos] = hot[i];
                h.Ids[pos] = i;
            }
            return h;
        }

        /// <summary>The index of a hot pixel in the sorted list (it must be one of them).</summary>
        public int IndexOf(P p)
        {
            if (Usable)
            {
                int cell = Cell(Grid, p);
                for (int k = Start[cell]; k < Start[cell + 1]; k++)
                {
                    if (Pts[k] == p)
                    {
                        return Ids[k];
                    }
                }
            }
            return Array.BinarySearch(Sorted, p);
        }

        private static int Cell(CellGrid g, P p) => (g.RowOf(p.Y) * g.Cols) + g.ColOf(p.X);
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
    private static bool Reroute(Frag f, HotGrid hg, bool touching, List<Frag> output)
    {
        long minX = Math.Min(f.A.X, f.B.X), maxX = Math.Max(f.A.X, f.B.X);
        long minY = Math.Min(f.A.Y, f.B.Y), maxY = Math.Max(f.A.Y, f.B.Y);
        int pad = touching ? 1 : 0;
        List<(P Pt, Int128 T, Int128 Raw)>? via = null;
        Int128 len2 = Geo.LengthSquared(f.A, f.B);

        void Consider(P h)
        {
            if (h.X < minX - pad || h.X > maxX + pad || h.Y < minY - pad || h.Y > maxY + pad || h == f.A || h == f.B)
            {
                return;
            }
            bool hit = touching ? Geo.SegmentTouchesPixel(f.A, f.B, h) : Geo.StrictlyInside(f.A, f.B, h);
            if (!hit)
            {
                return;
            }
            Int128 raw = Geo.Dot(f.A, h, f.B);
            Int128 t = raw <= 0 ? 0 : raw >= len2 ? len2 : raw;
            (via ??= new List<(P, Int128, Int128)>()).Add((h, t, raw));
        }

        if (!hg.Usable)
        {
            XRange(hg.Sorted, minX - pad, maxX + pad, out int lo, out int hi);
            for (int k = lo; k < hi; k++)
            {
                Consider(hg.Sorted[k]);
            }
        }
        else if (hg.Grid.BoxCells(minX - pad, minY - pad, maxX + pad, maxY + pad, 6, out int bc0, out int bc1, out int br0, out int br1))
        {
            // A short segment: every pixel Consider can accept lies inside its bounding box grown by pad, so the cells of
            // that box are enough, and there are only a few of them.
            CellGrid g = hg.Grid;
            for (int r = br0; r <= br1; r++)
            {
                for (int c = bc0; c <= bc1; c++)
                {
                    int cell = (r * g.Cols) + c;
                    for (int k = hg.Start[cell]; k < hg.Start[cell + 1]; k++)
                    {
                        Consider(hg.Pts[k]);
                    }
                }
            }
        }
        else
        {
            CellGrid g = hg.Grid;
            g.Traverse(f.A, f.B, hg.Lo, hg.Hi, out int c0, out int c1);
            // Candidate cells: the cells the segment visits and their neighbours (a hot pixel within one unit of the
            // segment lies in a neighbouring cell because cells are at least two units wide).
            // A vertex exactly on the segment (the touching = false case) lies in a cell the segment visits, so no neighbours.
            int nb = touching ? 1 : 0;
            for (int c = Math.Max(0, c0 - nb); c <= Math.Min(g.Cols - 1, c1 + nb); c++)
            {
                int rlo = int.MaxValue, rhi = int.MinValue;
                for (int k = Math.Max(c0, c - nb); k <= Math.Min(c1, c + nb); k++)
                {
                    rlo = Math.Min(rlo, hg.Lo[k - c0]);
                    rhi = Math.Max(rhi, hg.Hi[k - c0]);
                }
                rlo = Math.Max(0, rlo - nb);
                rhi = Math.Min(g.Rows - 1, rhi + nb);
                for (int r = rlo; r <= rhi; r++)
                {
                    int cell = (r * g.Cols) + c;
                    for (int k = hg.Start[cell]; k < hg.Start[cell + 1]; k++)
                    {
                        Consider(hg.Pts[k]);
                    }
                }
            }
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

    // Every fragment end is a hot pixel, so the sorted hot pixels serve as the vertex list; those no segment uses are
    // dropped by the renumbering below.
    private static void BuildSegments(List<Frag> frags, P[] verts, HotGrid grid, Arrangement r)
    {
        int n = frags.Count;
        var dd0 = new int[n];
        var dd1 = new int[n];
        long[] keys = new long[n];
        int[]? order = null;
        ulong[]? packed = null;
        // Vertex and fragment numbers that fit the packed form: vertex a (20 bits), vertex b (20 bits), fragment (24 bits).
        bool pack = verts.Length < (1 << 20) && n < (1 << 24);
        if (pack)
        {
            packed = new ulong[n];
        }
        else
        {
            order = new int[n];
        }
        for (int i = 0; i < n; i++)
        {
            Frag f = frags[i];
            int a = grid.IndexOf(f.A);
            int b = grid.IndexOf(f.B);
            int sign = 1;
            if (a > b)
            {
                (a, b) = (b, a);
                sign = -1;
            }
            if (pack)
            {
                packed![i] = ((ulong)a << 44) | ((ulong)b << 24) | (uint)i;
            }
            else
            {
                keys[i] = ((long)a << 32) | (uint)b;
                order![i] = i;
            }
            dd0[i] = f.Op != 1 ? sign : 0;
            dd1[i] = f.Op != 0 ? sign : 0;
        }
        if (pack)
        {
            Array.Sort(packed!);
            for (int i = 0; i < n; i++)
            {
                ulong v = packed![i];
                keys[i] = (long)(((v >> 44) << 32) | ((v >> 24) & 0xFFFFF));
            }
            order = new int[n];
            for (int i = 0; i < n; i++)
            {
                order[i] = (int)(packed![i] & 0xFFFFFF);
            }
        }
        else
        {
            Array.Sort(keys, order!);
        }
        // Equal keys are summed, so their relative order does not matter.
        var a2 = new List<int>();
        var b2 = new List<int>();
        var d0 = new List<int>();
        var d1 = new List<int>();
        for (int i = 0; i < n;)
        {
            long key = keys[i];
            int s0 = 0, s1 = 0;
            while (i < n && keys[i] == key)
            {
                s0 += dd0[order![i]];
                s1 += dd1[order![i]];
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
