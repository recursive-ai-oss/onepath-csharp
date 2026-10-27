using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// 2D geometry scenarios: fixed pseudo-random outline sets (coarse and fine grids, at the edge of the coordinate
/// range, with one-ulp nudges) run through union, difference, offset, triangulation and convex partition. Cases run
/// on <see cref="ScenarioContext.Threads"/> workers and are written in case order.
/// </summary>
internal static class Build2DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build2d/boolean", ctx => Run(ctx, 0, 300));
        Scenarios.Register("build2d/offset", ctx => Run(ctx, 1, 120));
        Scenarios.Register("build2d/mesh", ctx => Run(ctx, 2, 200));
    }

    private static void Run(ScenarioContext ctx, int kind, int cases)
    {
        var results = new List<long>[cases];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, cases, options, i => results[i] = Case(kind, i));
        foreach (List<long> r in results)
        {
            ctx.Write(r.Count);
            foreach (long v in r)
            {
                ctx.Write(v);
            }
        }
    }

    private struct Rng
    {
        public ulong S;

        public int Next(int n)
        {
            ulong z = S += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return (int)((z ^ (z >> 31)) % (ulong)n);
        }
    }

    private static List<Outline> Outlines(ref Rng rng, int count, int extent, int denom, double ox, double oy, bool nudge)
    {
        var list = new List<Outline>();
        for (int o = 0; o < count; o++)
        {
            int rings = 1 + (rng.Next(3) == 0 ? 1 : 0);
            var rs = new List<IReadOnlyList<Vector2D>>();
            for (int r = 0; r < rings; r++)
            {
                var pts = new Vector2D[3 + rng.Next(5)];
                for (int i = 0; i < pts.Length; i++)
                {
                    double x = ox + (rng.Next((extent * denom) + 1) / (double)denom);
                    double y = oy + (rng.Next((extent * denom) + 1) / (double)denom);
                    if (nudge && rng.Next(4) == 0)
                    {
                        x = rng.Next(2) == 0 ? Math.BitIncrement(x) : Math.BitDecrement(x);
                    }
                    pts[i] = new Vector2D(x, y);
                }
                rs.Add(pts);
            }
            list.Add(new Outline(rs));
        }
        return list;
    }

    private static List<long> Case(int kind, int i)
    {
        var rng = new Rng { S = (ulong)((kind * 1_000_003L) + i + 1) };
        int denom = rng.Next(3) == 0 ? 1 : rng.Next(2) == 0 ? 8 : 1024;
        int extent = denom == 1024 ? 1 : 8;
        double ox = i % 5 == 0 ? 2097152.0 - extent - 1 : 0.0;
        double oy = i % 5 == 0 ? -2097151.0 : 0.0;
        bool nudge = i % 3 == 0;
        Region a = Region.FromOutlines(Outlines(ref rng, 1 + rng.Next(3), extent, denom, ox, oy, nudge));
        var res = new List<long>();
        switch (kind)
        {
            case 0:
                {
                    Region b = Region.FromOutlines(Outlines(ref rng, 1 + rng.Next(3), extent, denom, ox, oy, nudge));
                    AddRegion(res, a.Union(b));
                    AddRegion(res, a.Intersection(b));
                    AddRegion(res, a.Difference(b));
                    AddRegion(res, a.SymmetricDifference(b));
                    break;
                }
            case 1:
                {
                    double d = (1 + rng.Next(8)) * (denom == 1024 ? 0.0625 : 0.25);
                    AddRegion(res, a.Offset(d));
                    AddRegion(res, a.Offset(-d));
                    AddRegion(res, a.Offset(d, JoinStyle.Square));
                    AddRegion(res, a.Offset(-d, JoinStyle.Square, 0));
                    break;
                }
            default:
                {
                    Triangulation t = a.Triangulate();
                    foreach (int v in t.Indices) { res.Add(v); }
                    foreach (int v in t.Neighbors) { res.Add(v); }
                    ConvexPartition p = a.PartitionConvex(3 + rng.Next(6));
                    for (int k = 0; k < p.PolygonCount; k++)
                    {
                        res.Add(-1);
                        foreach (int v in p.GetIndices(k)) { res.Add(v); }
                        foreach (int v in p.GetNeighbors(k)) { res.Add(v); }
                    }
                    break;
                }
        }
        return res;
    }

    private static void AddRegion(List<long> res, Region r)
    {
        res.Add(r.RingCount);
        for (int i = 0; i < r.RingCount; i++)
        {
            Vector2D[] ring = r.GetRing(i);
            res.Add(ring.Length);
            foreach (Vector2D p in ring)
            {
                res.Add(BitConverter.DoubleToInt64Bits(p.X));
                res.Add(BitConverter.DoubleToInt64Bits(p.Y));
            }
        }
    }
}
