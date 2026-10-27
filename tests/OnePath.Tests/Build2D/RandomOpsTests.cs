using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

public class RandomOpsTests
{
    private static long Count(long dflt)
    {
        string? v = Environment.GetEnvironmentVariable("ONEPATH_GEOMETRY_CASES");
        return v != null && long.TryParse(v, out long n) ? n : dflt;
    }

    private static List<Vector2D[][]> Prepare(List<Outline> set)
    {
        var l = new List<Vector2D[][]>();
        foreach (Outline o in set)
        {
            var rings = new Vector2D[o.RingCount][];
            for (int i = 0; i < rings.Length; i++) { rings[i] = o.GetRing(i); }
            l.Add(rings);
        }
        return l;
    }

    private static bool Member(List<Vector2D[][]> set, double px, double py)
    {
        foreach (Vector2D[][] o in set)
        {
            if (GeoOracle.EvenOdd(o, px, py)) { return true; }
        }
        return false;
    }

    private static double NearestEdge(List<Vector2D[][]> a, List<Vector2D[][]> b, double px, double py)
    {
        double best = double.MaxValue;
        foreach (var set in new[] { a, b })
        {
            foreach (Vector2D[][] o in set)
            {
                best = Math.Min(best, GeoOracle.MinEdgeDistance(o, px, py));
            }
        }
        return best;
    }

    [Theory]
    [InlineData("union", 1)]
    [InlineData("intersection", 2)]
    [InlineData("difference", 3)]
    [InlineData("xor", 4)]
    public void Operations_match_point_membership(string op, int seed)
    {
        long total = Count(1500);
        const int Chunk = 100;
        long chunks = (total + Chunk - 1) / Chunk;
        long samples = 0;
        string? failure = null;
        Parallel.For(0, chunks, new ParallelOptions { MaxDegreeOfParallelism = Math.Min(Environment.ProcessorCount, 6) }, (c, state) =>
        {
            long local = 0;
            long end = Math.Min(total, (c + 1) * Chunk);
            for (long i = c * Chunk; i < end; i++)
            {
                var rng = new Random(unchecked((int)(seed * 1_000_003L + i)));
                // Mostly an 8-unit square with eighth-unit coordinates (many collinear and touching cases);
                // some cases use coarse integer coordinates, some fine ones.
                int denom = rng.Next(3) == 0 ? 1 : rng.Next(2) == 0 ? 8 : 1024;
                int extent = denom == 1024 ? 1 : 8;
                // Every fifth case sits at the edge of the supported range; every third nudges vertices by an ulp.
                double ox = i % 5 == 0 ? 2097152.0 - extent - 1 : i % 7 == 0 ? -2097151.0 : 0.0;
                double oy = i % 5 == 0 ? -2097151.0 : 0.0;
                bool nudge = i % 3 == 0;
                List<Outline> a = RegionTests.RandomOutlines(rng, 1 + rng.Next(3), extent, denom, ox, oy, nudge);
                List<Outline> b = RegionTests.RandomOutlines(rng, 1 + rng.Next(3), extent, denom, ox, oy, nudge);
                try
                {
                    var pa = Prepare(a);
                    var pb = Prepare(b);
                    Region ra = Region.FromOutlines(a), rb = Region.FromOutlines(b);
                    Region r = op switch
                    {
                        "union" => ra.Union(rb),
                        "intersection" => ra.Intersection(rb),
                        "difference" => ra.Difference(rb),
                        _ => ra.SymmetricDifference(rb),
                    };
                    GeoOracle.ValidateRegion(ra);
                    GeoOracle.ValidateRegion(r);
                    if (i % 4 == 0)
                    {
                        GeoOracle.ValidateTriangulation(r, r.Triangulate());
                        GeoOracle.ValidatePartition(r, r.PartitionConvex(3 + (int)(i % 6)));
                    }
                    var rr = new List<Vector2D[]>(Rings(r)).ToArray();
                    var rra = new List<Vector2D[]>(Rings(r)).ToArray();
                    for (int s = 0; s < 30; s++)
                    {
                        double px = ox + (rng.NextDouble() * extent), py = oy + (rng.NextDouble() * extent);
                        if (NearestEdge(pa, pb, px, py) < 3e-4 * extent || GeoOracle.MinEdgeDistance(rr, px, py) < 3e-4 * extent)
                        {
                            continue;
                        }
                        bool ina = Member(pa, px, py), inb = Member(pb, px, py);
                        bool want = op switch
                        {
                            "union" => ina || inb,
                            "intersection" => ina && inb,
                            "difference" => ina && !inb,
                            _ => ina != inb,
                        };
                        bool got = GeoOracle.EvenOdd(rra, px, py);
                        if (want != got)
                        {
                            throw new InvalidOperationException($"case {i}: sample ({px},{py}) want {want} got {got}");
                        }
                        local++;
                    }
                }
                catch (Exception e)
                {
                    Interlocked.CompareExchange(ref failure, $"{op} case {i} (seed {seed}): {e.GetType().Name}: {e.Message}", null);
                    state.Stop();
                    return;
                }
            }
            Interlocked.Add(ref samples, local);
        });
        Assert.True(failure == null, failure);
        Assert.True(samples > total * 5, "too few samples were checked");
    }

    private static IEnumerable<Vector2D[]> Rings(Region r)
    {
        for (int i = 0; i < r.RingCount; i++) { yield return r.GetRing(i); }
    }

    [Fact]
    public void Offset_matches_distance_to_boundary()
    {
        long total = Count(1500) / 5 + 20;
        for (long i = 0; i < total; i++)
        {
            var rng = new Random(unchecked((int)(7_000_001L + i)));
            var o = RegionTests.RandomOutlines(rng, 1 + rng.Next(2), 8, 4);
            Region r = Region.FromOutlines(o);
            double d = (rng.Next(6) + 1) * 0.25;
            bool shrink = rng.Next(2) == 0;
            JoinStyle js = rng.Next(2) == 0 ? JoinStyle.Round : JoinStyle.Square;
            Region off = r.Offset(shrink ? -d : d, js);
            GeoOracle.ValidateRegion(off);
            GeoOracle.ValidateTriangulation(off, off.Triangulate());
            var ringsOfR = new List<Vector2D[]>(Rings(r));
            for (int s = 0; s < 30; s++)
            {
                double px = rng.NextDouble() * 8, py = rng.NextDouble() * 8;
                double dist = GeoOracle.MinEdgeDistance(ringsOfR, px, py);
                // Chebyshev distance for the square join: take the exact Minkowski bound instead of Euclid.
                double slack = 0.02 + (js == JoinStyle.Round ? d / 64 : 0);
                bool inside = GeoOracle.InRegion(r, px, py);
                bool got = GeoOracle.InRegion(off, px, py);
                if (js == JoinStyle.Round)
                {
                    if (shrink && inside && dist > d + slack) { Assert.True(got, $"case {i}: deep inside point lost"); }
                    if (shrink && inside && dist < d - slack) { Assert.False(got, $"case {i}: near-boundary point kept"); }
                    if (shrink && !inside) { Assert.False(got); }
                    if (!shrink && inside) { Assert.True(got); }
                    if (!shrink && !inside && dist < d - slack) { Assert.True(got, $"case {i}: near point not grown over"); }
                    if (!shrink && !inside && dist > d * 1.05 + slack) { Assert.False(got, $"case {i}: far point covered"); }
                }
                else
                {
                    // Square join covers at least the Euclidean ball and at most the ball scaled by sqrt(2).
                    if (!shrink && !inside && dist < d - slack) { Assert.True(got); }
                    if (!shrink && !inside && dist > d * 1.4143 + slack) { Assert.False(got); }
                    if (shrink && inside && dist > d * 1.4143 + slack) { Assert.True(got); }
                    if (shrink && inside && dist < d - slack) { Assert.False(got); }
                }
            }
        }
    }
}
