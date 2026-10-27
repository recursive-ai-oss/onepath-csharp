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
        Scenarios.Register("build2d/tiles", ctx => Run(ctx, 3, 120));
        Scenarios.Register("build2d/pipeline", ctx => Run(ctx, 4, 80));
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
        if (kind == 3)
        {
            return TileCase(ref rng, i);
        }
        if (kind == 4)
        {
            return PipelineCase(ref rng, i);
        }
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

    // Outlines on the 2^-10 grid cut into tiles of several sizes (tile lines on and off the outlines' vertices), built into
    // a stitched mesh whose serialized bytes are hashed.
    private static List<long> TileCase(ref Rng rng, int i)
    {
        int denom = new[] { 1, 8, 1024 }[rng.Next(3)];
        double ox = i % 4 == 0 ? 2097152.0 - 70 : 0.0;
        double oz = i % 4 == 0 ? -2097151.0 : 0.0;
        Region region = Region.FromOutlines(Outlines(ref rng, 1 + rng.Next(3), 24, denom, ox, oz, false), new Geometry2DOptions { SnapBits = 10 });
        double ts = new[] { 2.0, 4.0, 8.0, 16.0 }[rng.Next(4)];
        var p = new RecursiveAI.OnePath.Mesh.NavMeshParams { TileSize = ts, CellSize = 0.25, OriginX = ox - (rng.Next(4) * 0.5), OriginZ = oz - (rng.Next(4) * 0.25), MaxVertsPerPoly = 3 + rng.Next(6) };
        var data = new List<RecursiveAI.OnePath.Mesh.NavTileData>();
        foreach (TilePartition t in TileSlicer.Slice(region, p))
        {
            data.Add(Mesh2DPipeline.ToTileData(t, p, 0.5, 0, 1));
        }
        byte[] bytes = RecursiveAI.OnePath.Mesh.NavMeshSerializer.WriteMesh(Mesh2DPipeline.Assemble(p, data));
        var res = new List<long>(bytes.Length / 8) { bytes.Length };
        for (int k = 0; k < bytes.Length; k += 8)
        {
            long v = 0;
            for (int b = 0; b < 8 && k + b < bytes.Length; b++) { v |= (long)bytes[k + b] << (8 * b); }
            res.Add(v);
        }
        return res;
    }

    // The whole Build2D pipeline (shrink by the agent radius, tile cut, convex partition, assembly) on a random tile map
    // or random outlines, at several radii, solo and tiled; the serialized mesh bytes are hashed.
    private static List<long> PipelineCase(ref Rng rng, int i)
    {
        double[] radii = { 0.25, 0.5, 0.75, 1.0 };
        bool tiled = rng.Next(2) == 0;
        var settings = new RecursiveAI.OnePath.NavSettings
        {
            CellSize = 0.25,
            AgentRadius = (float)radii[i % radii.Length],
            TileSize = tiled ? 4.0 : 64.0,
            MaxVertsPerPoly = 3 + rng.Next(6),
        };
        Region region;
        if (i % 2 == 0)
        {
            int w = 4 + rng.Next(10), h = 4 + rng.Next(10);
            var cells = new bool[w * h];
            for (int k = 0; k < cells.Length; k++) { cells[k] = rng.Next(100) < 65; }
            region = Region.FromTileMap(cells, w, h, 2.0, new Vector2D(0, 0), Mesh2DPipeline.GeometryOptions(settings));
        }
        else
        {
            region = Region.FromOutlines(Outlines(ref rng, 1 + rng.Next(3), 12, 8, 0.0, 0.0, false), Mesh2DPipeline.GeometryOptions(settings));
        }
        byte[] bytes = RecursiveAI.OnePath.Mesh.NavMeshSerializer.WriteMesh(Mesh2DPipeline.Build(region, settings));
        var res = new List<long>(bytes.Length / 8) { bytes.Length };
        for (int k = 0; k < bytes.Length; k += 8)
        {
            long v = 0;
            for (int b = 0; b < 8 && k + b < bytes.Length; b++) { v |= (long)bytes[k + b] << (8 * b); }
            res.Add(v);
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
