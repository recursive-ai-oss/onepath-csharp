using System;
using System.Collections.Generic;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>A point on the integer snap grid (coordinate = value × 2^-SnapBits).</summary>
internal readonly struct P : IComparable<P>, IEquatable<P>
{
    public readonly long X;
    public readonly long Y;

    public P(long x, long y)
    {
        X = x;
        Y = y;
    }

    /// <summary>Lexicographic order (x, then y): the canonical vertex order.</summary>
    public int CompareTo(P o) => X < o.X ? -1 : X > o.X ? 1 : Y < o.Y ? -1 : Y > o.Y ? 1 : 0;

    public bool Equals(P o) => X == o.X && Y == o.Y;

    public override bool Equals(object? obj) => obj is P p && Equals(p);

    public override int GetHashCode() => throw new NotSupportedException("Grid points are never hashed.");

    public static bool operator ==(P a, P b) => a.X == b.X && a.Y == b.Y;

    public static bool operator !=(P a, P b) => !(a == b);

    public override string ToString() => $"({X}, {Y})";
}

/// <summary>Exact helpers on grid points. Signs come from the shared exact predicates; constructions use 128-bit integers.</summary>
internal static class Geo
{
    /// <summary>Largest absolute grid coordinate any stage produces; keeps every 128-bit product below 2^127.</summary>
    public const long MaxGrid = 1L << 39;

    public static int Orient(P a, P b, P c)
    {
        // Grid points are small in practice: with every coordinate below 2^29 the determinant fits a long exactly, which
        // is the same sign the exact predicate returns without its floating-point filter.
        const long Bound = 1L << 29;
        if ((ulong)(a.X + Bound) < (2UL * Bound) && (ulong)(a.Y + Bound) < (2UL * Bound)
            && (ulong)(b.X + Bound) < (2UL * Bound) && (ulong)(b.Y + Bound) < (2UL * Bound)
            && (ulong)(c.X + Bound) < (2UL * Bound) && (ulong)(c.Y + Bound) < (2UL * Bound))
        {
            long det = ((b.X - a.X) * (c.Y - a.Y)) - ((b.Y - a.Y) * (c.X - a.X));
            return det > 0 ? 1 : det < 0 ? -1 : 0;
        }
        return ExactPredicates.Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y);
    }

    /// <summary>+1 when d is inside the circle through counter-clockwise a, b, c.</summary>
    public static int InCircle(P a, P b, P c, P d)
    {
        // With every coordinate below 2^29 the translated terms fit a long (lifts below 2^61, cross terms below 2^61) and
        // the determinant fits 128 bits, so the sign is computed exactly in integers. Cocircular grid points, common in
        // tile maps, are decided here instead of falling through to the expansion arithmetic.
        const long Bound = 1L << 29;
        if ((ulong)(a.X + Bound) < (2UL * Bound) && (ulong)(a.Y + Bound) < (2UL * Bound)
            && (ulong)(b.X + Bound) < (2UL * Bound) && (ulong)(b.Y + Bound) < (2UL * Bound)
            && (ulong)(c.X + Bound) < (2UL * Bound) && (ulong)(c.Y + Bound) < (2UL * Bound)
            && (ulong)(d.X + Bound) < (2UL * Bound) && (ulong)(d.Y + Bound) < (2UL * Bound))
        {
            long adx = a.X - d.X, ady = a.Y - d.Y, bdx = b.X - d.X, bdy = b.Y - d.Y, cdx = c.X - d.X, cdy = c.Y - d.Y;
            long alift = (adx * adx) + (ady * ady), blift = (bdx * bdx) + (bdy * bdy), clift = (cdx * cdx) + (cdy * cdy);
            long bc = (bdx * cdy) - (cdx * bdy), ca = (cdx * ady) - (adx * cdy), ab = (adx * bdy) - (bdx * ady);
            Int128 det = ((Int128)alift * bc) + ((Int128)blift * ca) + ((Int128)clift * ab);
            return det > 0 ? 1 : det < 0 ? -1 : 0;
        }
        return ExactPredicates.InCircle(a.X, a.Y, b.X, b.Y, c.X, c.Y, d.X, d.Y);
    }

    public static Int128 Cross(P o, P a, P b) =>
        ((Int128)(a.X - o.X) * (b.Y - o.Y)) - ((Int128)(a.Y - o.Y) * (b.X - o.X));

    public static Int128 Dot(P o, P a, P b) =>
        ((Int128)(a.X - o.X) * (b.X - o.X)) + ((Int128)(a.Y - o.Y) * (b.Y - o.Y));

    public static Int128 LengthSquared(P a, P b) => Dot(a, b, b);

    /// <summary>floor(a / b) for b &gt; 0.</summary>
    public static Int128 FloorDiv(Int128 a, Int128 b)
    {
        Int128 q = a / b;
        if ((a % b) < 0)
        {
            q--;
        }
        return q;
    }

    /// <summary>
    /// The grid point nearest to the intersection of lines p1p2 and p3p4 (ties round up), or null when they are parallel.
    /// Computed exactly in integers, then rounded once.
    /// </summary>
    public static P? RoundedIntersection(P p1, P p2, P p3, P p4)
    {
        long d1x = p2.X - p1.X, d1y = p2.Y - p1.Y;
        long d2x = p4.X - p3.X, d2y = p4.Y - p3.Y;
        Int128 den = ((Int128)d1x * d2y) - ((Int128)d1y * d2x);
        if (den == 0)
        {
            return null;
        }
        Int128 tn = ((Int128)(p3.X - p1.X) * d2y) - ((Int128)(p3.Y - p1.Y) * d2x);
        if (den < 0)
        {
            den = -den;
            tn = -tn;
        }
        // x = p1.x + d1x * tn / den, rounded to nearest: floor((2 * num + den) / (2 * den)).
        return new P(p1.X + RoundedQuotient(d1x, tn, den), p1.Y + RoundedQuotient(d1y, tn, den));
    }

    // floor((2 * d * tn + den) / (2 * den)) for den > 0: d * tn / den rounded to nearest, ties up. A double estimate
    // is corrected with exact 128-bit arithmetic, so the result is the exact integer whatever the estimate was; this
    // avoids 128-bit division, the slow part of the crossing computation.
    private static long RoundedQuotient(long d, Int128 tn, Int128 den)
    {
        if (den < (1L << 62) && tn < (Int128)(1L << 62) && tn > -(Int128)(1L << 62) && Math.Abs(d) < (1L << 31))
        {
            double est = Math.Floor((((2.0 * d) * (double)tn) + (double)den) / (2.0 * (double)den));
            long k = (long)est;
            Int128 twoDen = 2 * den;
            Int128 rem = (2 * d * tn) + den - (k * twoDen);
            while (rem < 0)
            {
                k--;
                rem += twoDen;
            }
            while (rem >= twoDen)
            {
                k++;
                rem -= twoDen;
            }
            return k;
        }
        return (long)FloorDiv((2 * d * tn) + den, 2 * den);
    }

    /// <summary>
    /// True when the closed segment ab meets the half-open unit square [c - 1/2, c + 1/2) x [c - 1/2, c + 1/2) around grid
    /// point c (the set of points that round to c, ties up). Exact: a separating-axis test on the bounding boxes and on
    /// the line through a, b, in doubled coordinates so the square's corners are integers, with the open upper sides
    /// handled by a symbolic shrink (x by e, y by e^2) that decides every zero orientation.
    /// </summary>
    public static bool SegmentTouchesPixel(P a, P b, P c)
    {
        long ax = 2 * a.X, ay = 2 * a.Y, bx = 2 * b.X, by = 2 * b.Y;
        long loX = (2 * c.X) - 1, hiX = (2 * c.X) + 1, loY = (2 * c.Y) - 1, hiY = (2 * c.Y) + 1;
        if (Math.Max(ax, bx) < loX || Math.Min(ax, bx) >= hiX || Math.Max(ay, by) < loY || Math.Min(ay, by) >= hiY)
        {
            return false;
        }
        var pa = new P(ax, ay);
        var pb = new P(bx, by);
        int dxs = Math.Sign(bx - ax), dys = Math.Sign(by - ay);
        int s0 = Orient(pa, pb, new P(loX, loY));
        int s1 = Orient(pa, pb, new P(hiX, loY));
        if (s1 == 0) { s1 = dys; }
        int s2 = Orient(pa, pb, new P(hiX, hiY));
        if (s2 == 0) { s2 = dys != 0 ? dys : -dxs; }
        int s3 = Orient(pa, pb, new P(loX, hiY));
        if (s3 == 0) { s3 = -dxs; }
        bool allPositive = s0 > 0 && s1 > 0 && s2 > 0 && s3 > 0;
        bool allNegative = s0 < 0 && s1 < 0 && s2 < 0 && s3 < 0;
        return !(allPositive || allNegative);
    }

    /// <summary>True when c lies on segment ab strictly between its endpoints (exact).</summary>
    public static bool StrictlyInside(P a, P b, P c)
    {
        if (c == a || c == b || Orient(a, b, c) != 0)
        {
            return false;
        }
        return Math.Min(a.X, b.X) <= c.X && c.X <= Math.Max(a.X, b.X) && Math.Min(a.Y, b.Y) <= c.Y && c.Y <= Math.Max(a.Y, b.Y);
    }

    /// <summary>True when open segments ab and cd cross at a single point interior to both.</summary>
    public static bool ProperCrossing(P a, P b, P c, P d)
    {
        int o1 = Orient(a, b, c);
        int o2 = Orient(a, b, d);
        if (o1 * o2 >= 0)
        {
            return false;
        }
        int o3 = Orient(c, d, a);
        int o4 = Orient(c, d, b);
        return o3 * o4 < 0;
    }
}

/// <summary>
/// Hashing for dictionaries keyed by a pair of vertex indices packed into a long (high half, low half). The default
/// hash of a long xors the halves, which sends neighbouring index pairs to a handful of buckets; this mixes them. The
/// dictionaries are only ever looked up, never enumerated, so the hash cannot change a result.
/// </summary>
internal sealed class EdgeKeyComparer : IEqualityComparer<long>
{
    public static readonly EdgeKeyComparer Instance = new();

    public bool Equals(long x, long y) => x == y;

    public int GetHashCode(long key)
    {
        ulong z = (ulong)key * 0x9E3779B97F4A7C15UL;
        return (int)(z >> 32);
    }
}
