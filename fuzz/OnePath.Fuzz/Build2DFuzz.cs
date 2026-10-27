using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for the 2D geometry pipeline: bytes decode to outline sets (small grids, fine grids, one-ulp
/// neighbours, values at and beyond the coordinate bound, NaN and infinity), which run through union, difference,
/// offset, triangulation and convex partition. Argument exceptions (bad input) are the expected rejection; anything
/// else, or a result that fails the cheap exact checks below, is a finding.
/// </summary>
public static class Build2DFuzz
{
    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _i;
        public double Last;

        public Reader(byte[] d) => _d = d;

        public bool More => _i < _d.Length;

        public int Byte() => _i < _d.Length ? _d[_i++] : 0;

        public int SByte() => (sbyte)Byte();

        public double Coord()
        {
            int b = Byte();
            double v;
            switch (b >> 6)
            {
                case 0:
                    v = SByte() / 4.0;
                    break;
                case 1:
                    v = (short)((Byte() << 8) | Byte()) / 64.0;
                    break;
                case 2:
                    // Mostly valid special values; the invalid ones (NaN, infinity, beyond the bound) need both top bits set.
                    v = (b & 0x30) == 0x30
                        ? (b & 7) switch
                        {
                            0 => 2097152.5,
                            1 => double.NaN,
                            2 => double.PositiveInfinity,
                            3 => 1e300,
                            4 => 1e-300,
                            5 => -2097152.5,
                            _ => double.NegativeInfinity,
                        }
                        : (b & 7) switch
                        {
                            0 => 0.0,
                            1 => 1e-5,
                            2 => -1e-5,
                            3 => Math.BitIncrement(Last),
                            4 => Math.BitDecrement(Last),
                            5 => 2097152.0,
                            6 => -2097152.0,
                            _ => Last,
                        };
                    break;
                default:
                    v = Last + (SByte() * 7.62939453125e-6); // multiples of 2^-17
                    break;
            }
            Last = v;
            return v;
        }
    }

    private static Region? Build(Reader r, System.Text.StringBuilder? log = null)
    {
        int outlines = 1 + (r.Byte() % 3);
        var list = new List<Outline>();
        for (int o = 0; o < outlines; o++)
        {
            int rings = 1 + (r.Byte() % 2);
            var rs = new List<IReadOnlyList<Vector2D>>();
            for (int k = 0; k < rings; k++)
            {
                var pts = new Vector2D[3 + (r.Byte() % 8)];
                for (int i = 0; i < pts.Length; i++)
                {
                    double x = r.Coord();
                    double y = r.Coord();
                    pts[i] = new Vector2D(x, y);
                }
                rs.Add(pts);
            }
            list.Add(new Outline(rs));
            if (log != null)
            {
                foreach (IReadOnlyList<Vector2D> ring in rs)
                {
                    log.Append("ring:");
                    foreach (Vector2D p in ring) { log.Append(" (").Append(p.X.ToString("R", System.Globalization.CultureInfo.InvariantCulture)).Append(',').Append(p.Y.ToString("R", System.Globalization.CultureInfo.InvariantCulture)).Append(')'); }
                    log.AppendLine();
                }
                log.AppendLine("--");
            }
        }
        return Region.FromOutlines(list);
    }

    private static Int128 TwiceArea(Vector2D[] ring)
    {
        Int128 t = 0;
        for (int i = 0; i < ring.Length; i++)
        {
            Vector2D a = ring[i], b = ring[(i + 1) % ring.Length];
            long ax = (long)(a.X * 65536.0), ay = (long)(a.Y * 65536.0), bx = (long)(b.X * 65536.0), by = (long)(b.Y * 65536.0);
            t += ((Int128)ax * by) - ((Int128)ay * bx);
        }
        return t;
    }

    private static void Check(Region region)
    {
        Int128 twice = 0;
        for (int i = 0; i < region.RingCount; i++)
        {
            twice += TwiceArea(region.GetRing(i));
        }
        Triangulation t = region.Triangulate();
        ReadOnlySpan<Vector2D> v = t.Vertices;
        ReadOnlySpan<int> idx = t.Indices;
        Int128 tri = 0;
        for (int k = 0; k < t.TriangleCount; k++)
        {
            tri += TwiceArea(new[] { v[idx[3 * k]], v[idx[(3 * k) + 1]], v[idx[(3 * k) + 2]] });
        }
        if (tri != twice)
        {
            throw new InvalidOperationException("Triangle areas do not add up to the region area.");
        }
        ConvexPartition p = region.PartitionConvex(3 + (int)((twice & 7) % 6));
        Int128 poly = 0;
        ReadOnlySpan<Vector2D> pv = p.Vertices;
        for (int k = 0; k < p.PolygonCount; k++)
        {
            ReadOnlySpan<int> ids = p.GetIndices(k);
            var ring = new Vector2D[ids.Length];
            for (int i = 0; i < ring.Length; i++) { ring[i] = pv[ids[i]]; }
            poly += TwiceArea(ring);
        }
        if (poly != twice)
        {
            throw new InvalidOperationException("Polygon areas do not add up to the region area.");
        }
    }

    /// <summary>Runs one decoded input through the pipeline.</summary>
    public static void Run(byte[] data)
    {
        var r = new Reader(data);
        try
        {
            Region? a = Build(r);
            Region? b = Build(r);
            if (a == null || b == null)
            {
                return;
            }
            Check(a);
            Region u = a.Union(b);
            Check(u);
            Check(a.Difference(b));
            Check(a.Intersection(b));
            Check(a.SymmetricDifference(b));
            double d = (r.Byte() - 128) / 16.0;
            Check(u.Offset(d, (r.Byte() & 1) == 0 ? JoinStyle.Round : JoinStyle.Square, (r.Byte() & 3) * 0.01));
        }
        catch (ArgumentException)
        {
            // Rejected input: the documented outcome for NaN, infinity and out-of-range coordinates.
        }
    }

    /// <summary>Writes the outlines a decoded input produces (for turning a finding into a regression test).</summary>
    public static void Describe(byte[] data)
    {
        var log = new System.Text.StringBuilder();
        var r = new Reader(data);
        try
        {
            Build(r, log);
            log.AppendLine("==== second set");
            Build(r, log);
        }
        catch (ArgumentException)
        {
        }
        Console.WriteLine(log.ToString());
    }

    /// <summary>A built-in random driver: treats <paramref name="seedBytes"/> as a seed and runs many generated inputs.</summary>
    public static void Smoke(byte[] seedBytes)
    {
        ulong s = 1469598103934665603UL;
        foreach (byte b in seedBytes) { s = (s ^ b) * 1099511628211UL; }
        for (int n = 0; n < 2000; n++)
        {
            var data = new byte[24 + (int)(Next(ref s) % 300)];
            for (int i = 0; i < data.Length; i++) { data[i] = (byte)Next(ref s); }
            try
            {
                Run(data);
            }
            catch (Exception)
            {
                // Print the failing input so it can be replayed and minimized.
                Console.Error.WriteLine("FAILING INPUT: " + Convert.ToHexString(data));
                throw;
            }
        }
    }

    private static ulong Next(ref ulong s)
    {
        ulong z = s += 0x9E3779B97F4A7C15UL;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }
}
