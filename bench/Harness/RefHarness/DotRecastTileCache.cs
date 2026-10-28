// Black-box driver for DotRecast's tile cache (NuGet 2026.3.1, package DotRecast.Detour.TileCache): the same scenario and rows as the
// native tilecache command in recast_harness.cpp (see the comment there). Written against the library's public surface by compiling
// against the package; no source or metadata viewed.
//   dotrecast tilecache --obj f.obj --points f.points [--obstacles 16] [--obstacle-radius 2] [--obstacle-height 3] [--reps N] [settings...]
using System;
using System.Collections.Generic;
using System.Diagnostics;
using DotRecast.Core;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.TileCache;
using DotRecast.Detour.TileCache.Io.Compress;
using DotRecast.Recast;

namespace RecursiveAI.OnePath.RefHarness;

internal static class DotRecastTileCache
{
    private const string Side = "dotrecast-2026.3.1";

    /// <summary>Stores layers uncompressed: the cost of compression is not what the rows measure.</summary>
    private sealed class PassThrough : IRcCompressor
    {
        public byte[] Compress(byte[] buf) => (byte[])buf.Clone();

        public byte[] Decompress(byte[] buf) => (byte[])buf.Clone();

        public byte[] Decompress(byte[] buf, int offset, int len, int outputLen)
        {
            var r = new byte[len];
            Array.Copy(buf, offset, r, 0, len);
            return r;
        }
    }

    private sealed class PassThroughFactory : IDtTileCacheCompressorFactory
    {
        public IRcCompressor Create(int version) => new PassThrough();
    }

    /// <summary>One ground area and one walkable flag, as every other driver sets on its polygons.</summary>
    private sealed class MeshProc : IDtTileCacheMeshProcess
    {
        public void Process(DtNavMeshCreateParams param)
        {
            for (int i = 0; i < param.polyCount; i++)
            {
                param.polyAreas[i] = 0;
                param.polyFlags[i] = 1;
            }
        }
    }

    private sealed class Cache
    {
        public DtNavMesh Nav;
        public DtTileCache Tc;
        public int Layers;
    }

    private static Cache BuildCache(DotRecastDriver.Settings s, DotRecastDriver.Geom geom)
    {
        var bmin = geom.GetMeshBoundsMin();
        var bmax = geom.GetMeshBoundsMax();
        RcRecast.CalcGridSize(bmin, bmax, s.Cs, out int gw, out int gh);
        int tw = (gw + s.TileSize - 1) / s.TileSize, th = (gh + s.TileSize - 1) / s.TileSize;
        var sp = new DtTileCacheStorageParams(RcByteOrder.LITTLE_ENDIAN, false);
        var factory = new PassThroughFactory();
        var cfg = DotRecastDriver.MakeConfig(s, true);
        var builder = new DtTileCacheLayerBuilder(factory);

        var tp = new DtTileCacheParams();
        tp.orig = bmin;
        tp.cs = s.Cs;
        tp.ch = s.Ch;
        tp.width = s.TileSize;
        tp.height = s.TileSize;
        tp.walkableHeight = s.AgentHeight;
        tp.walkableRadius = s.AgentRadius;
        tp.walkableClimb = s.Climb;
        tp.maxSimplificationError = s.EdgeErr;
        int bits = 1;
        while ((1 << bits) < tw * th * 4)
        {
            bits++;
        }

        bits = Math.Min(bits, 14);
        tp.maxTiles = 1 << bits;
        tp.maxObstacles = 256;

        var np = new DtNavMeshParams();
        np.orig = bmin;
        np.tileWidth = np.tileHeight = s.TileSize * s.Cs;
        np.maxTiles = 1 << bits;
        np.maxPolys = 1 << (22 - bits);
        var nav = new DtNavMesh();
        nav.Init(np, s.Nvp);
        var tc = new DtTileCache(tp, sp, nav, factory.Create(0), new MeshProc());
        var c = new Cache { Nav = nav, Tc = tc };

        var tileRefs = new List<long>();
        var built = builder.Build(geom, cfg, sp, 1, tw, th);
        foreach (var r in built)
        {
            foreach (var data in r.layers)
            {
                tileRefs.Add(tc.AddTile(data, 0));
                c.Layers++;
            }
        }

        foreach (long tileRef in tileRefs)
        {
            tc.BuildNavMeshTile(tileRef);
        }

        return c;
    }

    private static (long Polys, double Area, int Tiles) Measure(DtNavMesh nav)
    {
        long polys = 0;
        double area = 0;
        int tiles = 0;
        for (int i = 0; i < nav.GetMaxTiles(); i++)
        {
            var t = nav.GetTile(i);
            if (t == null || t.data == null || t.data.header == null)
            {
                continue;
            }

            tiles++;
            for (int p = 0; p < t.data.header.polyCount; p++)
            {
                var poly = t.data.polys[p];
                polys++;
                double a = 0;
                for (int j = 0, k = poly.vertCount - 1; j < poly.vertCount; k = j++)
                {
                    int vj = poly.verts[j] * 3, vk = poly.verts[k] * 3;
                    a += (double)t.data.verts[vk] * t.data.verts[vj + 2] - (double)t.data.verts[vj] * t.data.verts[vk + 2];
                }

                area += Math.Abs(a) * 0.5;
            }
        }

        return (polys, area, tiles);
    }

    public static int Run(Args a)
    {
        if (!Input.LoadObj(a.Str("obj"), out var verts, out var tris))
        {
            Console.Error.WriteLine("cannot read --obj");
            return 2;
        }

        var pts = Input.LoadPoints(a.Str("points"));
        var s = new DotRecastDriver.Settings(a);
        var geom = new DotRecastDriver.Geom(verts, tris);
        int reps = a.Int("reps", 3), count = a.Int("obstacles", 16);
        float radius = (float)a.Num("obstacle-radius", 2.0), height = (float)a.Num("obstacle-height", 3.0);
        string scen = a.Str("scenario", a.Str("obj") + "#tilecache");
        int npts = pts.Length / 3;
        if (npts < count)
        {
            Console.Error.WriteLine("need " + count + " points");
            return 2;
        }

        // Obstacle k: a cylinder whose base sits half a unit under the k-th stride's point.
        var pos = new List<RcVec3f>();
        for (int k = 0; k < count; k++)
        {
            int i = 3 * (k * (npts / count));
            pos.Add(new RcVec3f(pts[i], pts[i + 1] - 0.5f, pts[i + 2]));
        }

        var buildTimes = new List<double>();
        var cycleTimes = new List<double>();
        Cache cache = null;
        for (int rep = 0; rep < reps; rep++)
        {
            var sw = Stopwatch.StartNew();
            cache = BuildCache(s, geom);
            buildTimes.Add(sw.Elapsed.TotalMilliseconds);
        }

        var initial = Measure(cache.Nav);
        (long Polys, double Area, int Tiles) stacked = default, restored = default;
        for (int rep = 0; rep < reps; rep++)
        {
            var refs = new long[count];
            var sw = Stopwatch.StartNew();
            for (int k = 0; k < count; k++)
            {
                refs[k] = cache.Tc.AddObstacle(pos[k], radius, height);
                Settle(cache);
            }

            double mid = sw.Elapsed.TotalMilliseconds;
            if (rep == 0)
            {
                stacked = Measure(cache.Nav);
            }

            sw.Restart();
            for (int k = 0; k < count; k++)
            {
                cache.Tc.RemoveObstacle(refs[k]);
                Settle(cache);
            }

            cycleTimes.Add(mid + sw.Elapsed.TotalMilliseconds);
            if (rep == 0)
            {
                restored = Measure(cache.Nav);
            }
        }

        Out.Row("tilecache", scen, Side, "time_best", Out.Best(cycleTimes), "ms");
        Out.Row("tilecache", scen, Side, "time_median", Out.Median(cycleTimes), "ms");
        Out.Row("tilecache", scen, Side, "cache_build_time", Out.Best(buildTimes), "ms");
        Out.Row("tilecache", scen, Side, "obstacles", count, "count");
        Out.Row("tilecache", scen, Side, "polys_initial", initial.Polys, "count");
        Out.Row("tilecache", scen, Side, "polys", stacked.Polys, "count");
        Out.Row("tilecache", scen, Side, "polys_restored", restored.Polys, "count");
        Out.Row("tilecache", scen, Side, "coverage_area_initial", initial.Area, "world_units2");
        Out.Row("tilecache", scen, Side, "coverage_area", stacked.Area, "world_units2");
        Out.Row("tilecache", scen, Side, "coverage_area_restored", restored.Area, "world_units2");
        Out.Row("tilecache", scen, Side, "tiles", initial.Tiles, "count");
        Out.Row("tilecache", scen, Side, "layers", cache.Layers, "count");
        Out.Row("tilecache", scen, Side, "input_tris", tris.Length / 3, "count");
        Out.Row("tilecache", scen, Side, "peak_rss", Out.PeakRssMb(), "MB");
        return 0;
    }

    private static void Settle(Cache c)
    {
        // Update returns whether the cache is up to date; a call handles only some of the pending requests.
        for (int guard = 0; guard < 100000 && !c.Tc.Update(); guard++)
        {
        }
    }
}
