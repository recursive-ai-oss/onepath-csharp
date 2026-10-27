using System;
using System.Collections.Generic;
using System.Numerics;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;
using RecursiveAI.OnePath.NavMesh.Build2D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

/// <summary>Independent checks of regions, triangulations and partitions, written without the library's own code paths.</summary>
internal static class GeoOracle
{
    public static (long X, long Y)[] Grid(Vector2D[] ring, int bits)
    {
        double s = 1;
        for (int i = 0; i < bits; i++) { s *= 2; }
        var r = new (long, long)[ring.Length];
        for (int i = 0; i < r.Length; i++)
        {
            double x = ring[i].X * s, y = ring[i].Y * s;
            Assert.Equal(Math.Floor(x), x); // exact on the grid
            Assert.Equal(Math.Floor(y), y);
            r[i] = ((long)x, (long)y);
        }
        return r;
    }

    public static BigInteger TwiceArea((long X, long Y)[] r)
    {
        BigInteger a = 0;
        for (int i = 0; i < r.Length; i++)
        {
            var p = r[i];
            var q = r[(i + 1) % r.Length];
            a += (BigInteger)p.X * q.Y - (BigInteger)p.Y * q.X;
        }
        return a;
    }

    /// <summary>Rings never cross or overlap, touch only at shared vertices; returns twice the exact area.</summary>
    public static BigInteger ValidateRegion(Region region)
    {
        var edges = new List<((long, long) A, (long, long) B)>();
        BigInteger twice = 0;
        for (int i = 0; i < region.RingCount; i++)
        {
            var ring = Grid(region.GetRing(i), region.SnapBits);
            Assert.True(ring.Length >= 3);
            for (int k = 0; k < ring.Length; k++)
            {
                edges.Add((ring[k], ring[(k + 1) % ring.Length]));
            }
            twice += TwiceArea(ring);
        }
        for (int i = 0; i < edges.Count; i++)
        {
            Assert.NotEqual(edges[i].A, edges[i].B);
            for (int j = i + 1; j < edges.Count; j++)
            {
                var (a, b) = edges[i];
                var (c, d) = edges[j];
                var rel = SegmentPredicates.Classify(a.Item1, a.Item2, b.Item1, b.Item2, c.Item1, c.Item2, d.Item1, d.Item2);
                switch (rel)
                {
                    case SegmentRelation.Disjoint:
                        break;
                    case SegmentRelation.Touching:
                        Assert.True(a == c || a == d || b == c || b == d, $"T-junction between ring edges {a}-{b} and {c}-{d}");
                        break;
                    default:
                        Assert.Fail($"ring edges {a}-{b} and {c}-{d} are {rel}");
                        break;
                }
            }
        }
        return twice;
    }

    public static double Dist(double px, double py, Vector2D a, Vector2D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y;
        double len2 = dx * dx + dy * dy;
        double t = len2 == 0 ? 0 : ((px - a.X) * dx + (py - a.Y) * dy) / len2;
        t = Math.Max(0, Math.Min(1, t));
        double qx = a.X + t * dx - px, qy = a.Y + t * dy - py;
        return Math.Sqrt(qx * qx + qy * qy);
    }

    /// <summary>Even-odd membership by ray casting (sample points are never on an edge: callers keep clear of edges).</summary>
    public static bool EvenOdd(IEnumerable<Vector2D[]> rings, double px, double py)
    {
        bool inside = false;
        foreach (var r in rings)
        {
            for (int i = 0, j = r.Length - 1; i < r.Length; j = i++)
            {
                if ((r[i].Y > py) != (r[j].Y > py))
                {
                    double x = r[j].X + (py - r[j].Y) / (r[i].Y - r[j].Y) * (r[i].X - r[j].X);
                    if (px < x)
                    {
                        inside = !inside;
                    }
                }
            }
        }
        return inside;
    }

    public static bool InRegion(Region region, double px, double py)
    {
        var rings = new List<Vector2D[]>();
        for (int i = 0; i < region.RingCount; i++) { rings.Add(region.GetRing(i)); }
        return EvenOdd(rings, px, py);
    }

    public static double MinEdgeDistance(IEnumerable<Vector2D[]> rings, double px, double py)
    {
        double best = double.MaxValue;
        foreach (var r in rings)
        {
            for (int i = 0; i < r.Length; i++)
            {
                best = Math.Min(best, Dist(px, py, r[i], r[(i + 1) % r.Length]));
            }
        }
        return best;
    }

    public static bool InTriangle(Vector2D a, Vector2D b, Vector2D c, double px, double py)
    {
        double d1 = (b.X - a.X) * (py - a.Y) - (b.Y - a.Y) * (px - a.X);
        double d2 = (c.X - b.X) * (py - b.Y) - (c.Y - b.Y) * (px - b.X);
        double d3 = (a.X - c.X) * (py - c.Y) - (a.Y - c.Y) * (px - c.X);
        return d1 > 0 && d2 > 0 && d3 > 0;
    }

    /// <summary>Triangles are counter-clockwise, adjacency is symmetric, and their exact total area equals the region's.</summary>
    public static void ValidateTriangulation(Region region, Triangulation tri)
    {
        var v = tri.Vertices.ToArray();
        var idx = tri.Indices.ToArray();
        var nb = tri.Neighbors.ToArray();
        BigInteger twice = 0;
        for (int t = 0; t < tri.TriangleCount; t++)
        {
            var g = new (long, long)[3];
            for (int i = 0; i < 3; i++)
            {
                g[i] = Grid(new[] { v[idx[3 * t + i]] }, region.SnapBits)[0];
            }
            BigInteger a = TwiceArea(g);
            Assert.True(a > 0, "triangle not counter-clockwise or degenerate");
            twice += a;
            for (int i = 0; i < 3; i++)
            {
                int u = nb[3 * t + i];
                if (u < 0) { continue; }
                bool found = false;
                for (int k = 0; k < 3; k++)
                {
                    if (idx[3 * u + k] == idx[3 * t + (i + 1) % 3] && idx[3 * u + (k + 1) % 3] == idx[3 * t + i] && nb[3 * u + k] == t)
                    {
                        found = true;
                    }
                }
                Assert.True(found, "adjacency is not symmetric");
            }
        }
        Assert.Equal(ValidateRegion(region), twice);
    }

    public static void ValidatePartition(Region region, ConvexPartition cp)
    {
        var v = cp.Vertices.ToArray();
        BigInteger twice = 0;
        for (int p = 0; p < cp.PolygonCount; p++)
        {
            var ids = cp.GetIndices(p).ToArray();
            var nbs = cp.GetNeighbors(p).ToArray();
            Assert.True(ids.Length >= 3);
            var g = new (long, long)[ids.Length];
            for (int i = 0; i < ids.Length; i++) { g[i] = Grid(new[] { v[ids[i]] }, region.SnapBits)[0]; }
            BigInteger a = TwiceArea(g);
            Assert.True(a > 0);
            twice += a;
            for (int i = 0; i < ids.Length; i++)
            {
                var p0 = v[ids[i]];
                var p1 = v[ids[(i + 1) % ids.Length]];
                var p2 = v[ids[(i + 2) % ids.Length]];
                Assert.True(ExactPredicates.Orient2D(p0.X, p0.Y, p1.X, p1.Y, p2.X, p2.Y) >= 0, "polygon not convex");
                int u = nbs[i];
                if (u < 0) { continue; }
                var other = cp.GetIndices(u).ToArray();
                var on = cp.GetNeighbors(u).ToArray();
                bool found = false;
                for (int k = 0; k < other.Length; k++)
                {
                    if (other[k] == ids[(i + 1) % ids.Length] && other[(k + 1) % other.Length] == ids[i] && on[k] == p)
                    {
                        found = true;
                    }
                }
                Assert.True(found, "polygon adjacency is not symmetric");
            }
        }
        Assert.Equal(ValidateRegion(region), twice);
    }
}
