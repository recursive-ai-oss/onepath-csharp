// Black-box driver for DotRecast (NuGet 2026.3.1). Same subcommands and rows as the native Recast driver.
// Written against the library's public surface by compiling against the package; no source or metadata viewed.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using DotRecast.Core;
using DotRecast.Core.Collections;
using DotRecast.Core.Numerics;
using DotRecast.Detour;
using DotRecast.Detour.Crowd;
using DotRecast.Recast;

namespace RecursiveAI.OnePath.RefHarness;

internal static class DotRecastDriver
{
    private const string Side = "dotrecast-2026.3.1";

    internal sealed class Settings
    {
        public float Cs = 0.3f, Ch = 0.2f, AgentHeight = 2.0f, AgentRadius = 0.6f, Climb = 0.9f, Slope = 45.0f;
        public float RegionMin = 8, RegionMerge = 20, EdgeLen = 12, EdgeErr = 1.3f, DetailDist = 6, DetailErr = 1;
        public int Nvp = 6, TileSize = 64;
        public bool Tiled;

        public Settings(Args a)
        {
            Cs = (float)a.Num("cell", Cs); Ch = (float)a.Num("cellheight", Ch);
            AgentHeight = (float)a.Num("agent-height", AgentHeight); AgentRadius = (float)a.Num("agent-radius", AgentRadius);
            Climb = (float)a.Num("climb", Climb); Slope = (float)a.Num("slope", Slope);
            RegionMin = (float)a.Num("region-min", RegionMin); RegionMerge = (float)a.Num("region-merge", RegionMerge);
            EdgeLen = (float)a.Num("edge-len", EdgeLen); EdgeErr = (float)a.Num("edge-err", EdgeErr);
            DetailDist = (float)a.Num("detail-dist", DetailDist); DetailErr = (float)a.Num("detail-err", DetailErr);
            Nvp = a.Int("verts-per-poly", Nvp); TileSize = a.Int("tile-size", TileSize);
            Tiled = a.Str("mode", "solo") == "tiled";
        }
    }

    public static int Run(Args a)
    {
        if (a.Pos.Count < 2)
        {
            Console.Error.WriteLine("usage: dotrecast build|query|crowd|tilecache --obj f.obj ...");
            return 2;
        }

        switch (a.Pos[1])
        {
            case "build": return Build(a);
            case "query": return Query(a);
            case "crowd": return Crowd(a);
            case "tilecache": return DotRecastTileCache.Run(a);
            default:
                Console.Error.WriteLine("unknown subcommand " + a.Pos[1]);
                return 2;
        }
    }

    /// <summary>A plain triangle soup as the library's input-geometry interface.</summary>
    internal sealed class Geom : DotRecast.Recast.Geom.IRcInputGeomProvider
    {
        private readonly DotRecast.Recast.Geom.RcTriMesh mesh;
        private readonly RcVec3f bmin, bmax;
        private readonly List<RcConvexVolume> volumes = new();
        private readonly List<DotRecast.Recast.Geom.RcOffMeshConnection> links = new();

        public Geom(float[] verts, int[] tris)
        {
            mesh = new DotRecast.Recast.Geom.RcTriMesh(verts, tris);
            RcVec3f lo = new RcVec3f(float.MaxValue, float.MaxValue, float.MaxValue), hi = new RcVec3f(float.MinValue, float.MinValue, float.MinValue);
            for (int i = 0; i < verts.Length; i += 3)
            {
                lo = new RcVec3f(Math.Min(lo.X, verts[i]), Math.Min(lo.Y, verts[i + 1]), Math.Min(lo.Z, verts[i + 2]));
                hi = new RcVec3f(Math.Max(hi.X, verts[i]), Math.Max(hi.Y, verts[i + 1]), Math.Max(hi.Z, verts[i + 2]));
            }

            bmin = lo;
            bmax = hi;
        }

        public RcVec3f GetMeshBoundsMin() => bmin;
        public RcVec3f GetMeshBoundsMax() => bmax;
        public DotRecast.Recast.Geom.RcTriMesh GetMesh() => mesh;
        public IEnumerable<DotRecast.Recast.Geom.RcTriMesh> Meshes() { yield return mesh; }
        public IList<RcConvexVolume> ConvexVolumes() => volumes;
        public void AddConvexVolume(RcConvexVolume vol) => volumes.Add(vol);
        public List<DotRecast.Recast.Geom.RcOffMeshConnection> GetOffMeshConnections() => links;
        public void AddOffMeshConnection(RcVec3f start, RcVec3f end, float radius, bool bidir, int area, int flags) { }
        public void RemoveOffMeshConnections(Predicate<DotRecast.Recast.Geom.RcOffMeshConnection> filter) { }
    }

    private sealed class BuildResult
    {
        public List<DtMeshData> Tiles = new();
        public long Polys, Verts;
        public double AreaXZ;
        public RcVec3f Bmin, Bmax;
        public int Tw = 1, Th = 1;
    }

    internal static RcConfig MakeConfig(Settings s, bool tiled) =>
        new RcConfig(tiled, s.TileSize, s.TileSize, tiled ? (int)Math.Ceiling(s.AgentRadius / s.Cs) + 3 : 0, RcPartition.WATERSHED, s.Cs, s.Ch, s.Slope, s.AgentHeight, s.AgentRadius, s.Climb,
            (int)s.RegionMin, (int)s.RegionMerge, s.EdgeLen, s.EdgeErr, s.Nvp, s.DetailDist, s.DetailErr, true, true, true,
            new RcAreaModification(63), true);

    private static void AddTile(Settings s, RcBuilderResult res, int tx, int tz, BuildResult o)
    {
        var pm = res.Mesh;
        if (Environment.GetEnvironmentVariable("REFHARNESS_DEBUG") != null)
        {
            Console.Error.WriteLine($"tile {tx},{tz}: mesh={(pm == null ? "null" : pm.npolys.ToString())} detail={(res.MeshDetail == null ? "null" : res.MeshDetail.nmeshes.ToString())} contours={(res.ContourSet == null ? "null" : "set")} chf={(res.CompactHeightfield == null ? "null" : res.CompactHeightfield.spanCount.ToString())} hf={(res.SolidHeightfiled == null ? "null" : "set")}");
        }

        if (pm == null || pm.npolys == 0)
        {
            return;
        }

        for (int i = 0; i < pm.npolys; i++)
        {
            pm.areas[i] = 0; // one ground area
            pm.flags[i] = 1; // one walkable flag
            double area = 0;
            int n = 0;
            while (n < pm.nvp && pm.polys[i * pm.nvp * 2 + n] != 0xffff)
            {
                n++;
            }

            for (int j = 0, k = n - 1; j < n; k = j++)
            {
                int vj = pm.polys[i * pm.nvp * 2 + j] * 3, vk = pm.polys[i * pm.nvp * 2 + k] * 3;
                area += (double)pm.verts[vk] * pm.verts[vj + 2] - (double)pm.verts[vj] * pm.verts[vk + 2];
            }

            o.AreaXZ += Math.Abs(area) * 0.5 * pm.cs * pm.cs;
        }

        o.Polys += pm.npolys;
        o.Verts += pm.nverts;
        var dm = res.MeshDetail;
        var param = new DtNavMeshCreateParams();
        param.verts = pm.verts; param.vertCount = pm.nverts;
        param.polys = pm.polys; param.polyAreas = pm.areas; param.polyFlags = pm.flags; param.polyCount = pm.npolys;
        param.nvp = pm.nvp;
        param.detailMeshes = dm.meshes; param.detailVerts = dm.verts; param.detailVertsCount = dm.nverts;
        param.detailTris = dm.tris; param.detailTriCount = dm.ntris;
        param.walkableHeight = s.AgentHeight; param.walkableRadius = s.AgentRadius; param.walkableClimb = s.Climb;
        param.tileX = tx; param.tileZ = tz; param.tileLayer = 0;
        param.bmin = pm.bmin; param.bmax = pm.bmax;
        param.cs = pm.cs; param.ch = pm.ch; param.buildBvTree = true;
        o.Tiles.Add(DtNavMeshBuilder.CreateNavMeshData(param));
    }

    private static BuildResult BuildAll(Settings s, Geom geom)
    {
        var o = new BuildResult { Bmin = geom.GetMeshBoundsMin(), Bmax = geom.GetMeshBoundsMax() };
        var builder = new RcBuilder();
        var cfg = MakeConfig(s, s.Tiled);
        if (!s.Tiled)
        {
            AddTile(s, builder.Build(geom, new RcBuilderConfig(cfg, o.Bmin, o.Bmax), Environment.GetEnvironmentVariable("REFHARNESS_DEBUG") != null), 0, 0, o);
            return o;
        }

        RcRecast.CalcGridSize(o.Bmin, o.Bmax, s.Cs, out int gw, out int gh);
        if (Environment.GetEnvironmentVariable("REFHARNESS_DEBUG") != null)
        {
            Console.Error.WriteLine($"grid {gw}x{gh} bounds {o.Bmin} .. {o.Bmax} cs {s.Cs} tile {s.TileSize}");
        }

        o.Tw = (gw + s.TileSize - 1) / s.TileSize;
        o.Th = (gh + s.TileSize - 1) / s.TileSize;
        for (int y = 0; y < o.Th; y++)
        {
            for (int x = 0; x < o.Tw; x++)
            {
                // The tile overload takes the bounds of the whole input and the tile's grid position: it offsets the box itself.
                // (Passing the tile's own box offsets it twice, so only the lower-left quarter of the level gets built.)
                AddTile(s, builder.Build(geom, new RcBuilderConfig(cfg, o.Bmin, o.Bmax, x, y), false), x, y, o);
            }
        }

        return o;
    }

    private static int Build(Args a)
    {
        if (!Input.LoadObj(a.Str("obj"), out var verts, out var tris))
        {
            Console.Error.WriteLine("cannot read --obj");
            return 2;
        }

        var s = new Settings(a);
        var geom = new Geom(verts, tris);
        int reps = a.Int("reps", 3);
        string scen = a.Str("scenario", a.Str("obj") + (s.Tiled ? "#tiled" : "#solo"));
        var times = new List<double>();
        BuildResult last = null;
        for (int i = 0; i < reps; i++)
        {
            var sw = Stopwatch.StartNew();
            last = BuildAll(s, geom);
            times.Add(sw.Elapsed.TotalMilliseconds);
        }

        Out.Row("build", scen, Side, "time_best", Out.Best(times), "ms");
        Out.Row("build", scen, Side, "time_median", Out.Median(times), "ms");
        Out.Row("build", scen, Side, "polys", last.Polys, "count");
        Out.Row("build", scen, Side, "verts", last.Verts, "count");
        Out.Row("build", scen, Side, "coverage_area", last.AreaXZ, "world_units2");
        Out.Row("build", scen, Side, "tiles", last.Tiles.Count, "count");
        Out.Row("build", scen, Side, "input_tris", tris.Length / 3, "count");
        Out.Row("build", scen, Side, "peak_rss", Out.PeakRssMb(), "MB");
        return 0;
    }

    private static DtNavMesh MakeNavMesh(Settings s, BuildResult r)
    {
        var nav = new DtNavMesh();
        if (!s.Tiled)
        {
            nav.Init(r.Tiles[0], s.Nvp, 0);
            return nav;
        }

        var p = new DtNavMeshParams();
        p.orig = r.Bmin;
        p.tileWidth = p.tileHeight = s.TileSize * s.Cs;
        int bits = 1;
        while ((1 << bits) < r.Tw * r.Th)
        {
            bits++;
        }

        bits = Math.Min(bits, 14);
        p.maxTiles = 1 << bits;
        p.maxPolys = 1 << (22 - bits);
        nav.Init(p, s.Nvp);
        foreach (var t in r.Tiles)
        {
            nav.AddTile(t, 0, 0, out long _);
        }

        return nav;
    }

    private static readonly RcVec3f Extents = new RcVec3f(2, 4, 2);

    private static List<(long Ref, RcVec3f Pos)> Snap(DtNavMeshQuery q, float[] pts)
    {
        var filter = new DtQueryDefaultFilter();
        var res = new List<(long, RcVec3f)>();
        for (int i = 0; i + 2 < pts.Length; i += 3)
        {
            q.FindNearestPoly(new RcVec3f(pts[i], pts[i + 1], pts[i + 2]), Extents, filter, out long r, out RcVec3f np, out bool _);
            res.Add((r, np));
        }

        return res;
    }

    private static int Query(Args a)
    {
        if (!Input.LoadObj(a.Str("obj"), out var verts, out var tris))
        {
            Console.Error.WriteLine("cannot read --obj");
            return 2;
        }

        var pts = Input.LoadPoints(a.Str("points"));
        var s = new Settings(a);
        int reps = a.Int("reps", 5);
        string scen = a.Str("scenario", a.Str("obj") + (s.Tiled ? "#tiled" : "#solo"));
        var r = BuildAll(s, new Geom(verts, tris));
        var nav = MakeNavMesh(s, r);
        var q = new DtNavMeshQuery(nav);
        var filter = new DtQueryDefaultFilter();
        var sp = Snap(q, pts);
        var pairs = new List<(int, int)>();
        for (int i = 0; i + 1 < sp.Count; i += 2)
        {
            if (sp[i].Ref != 0 && sp[i + 1].Ref != 0)
            {
                pairs.Add((i, i + 1));
            }
        }

        Out.Row("query", scen, Side, "points_snapped", sp.FindAll(x => x.Ref != 0).Count, "count");
        Out.Row("query", scen, Side, "pairs", pairs.Count, "count");
        var path = new long[4096];
        var straight = new DtStraightPath[4096];
        double lenSum = 0;
        long ok = 0, partial = 0, hits = 0, rayFailed = 0;
        var tPath = new List<double>();
        var tRay = new List<double>();
        var tNear = new List<double>();
        for (int rep = 0; rep < reps; rep++)
        {
            lenSum = 0; ok = 0; partial = 0;
            var sw = Stopwatch.StartNew();
            foreach (var (ia, ib) in pairs)
            {
                var A = sp[ia]; var B = sp[ib];
                q.FindPath(A.Ref, B.Ref, A.Pos, B.Pos, filter, path, out int pc, path.Length);
                if (pc <= 0)
                {
                    continue;
                }

                var endPos = B.Pos;
                if (path[pc - 1] != B.Ref)
                {
                    partial++;
                    q.ClosestPointOnPoly(path[pc - 1], B.Pos, out endPos, out bool _);
                }

                q.FindStraightPath(A.Pos, endPos, path, pc, straight, out int sc, straight.Length, 0);
                double len = 0;
                for (int i = 1; i < sc; i++)
                {
                    len += RcVec3f.Distance(straight[i - 1].pos, straight[i].pos);
                }

                lenSum += len; ok++;
            }

            tPath.Add(sw.Elapsed.TotalMilliseconds);
        }

        for (int rep = 0; rep < reps; rep++)
        {
            hits = 0; rayFailed = 0;
            var sw = Stopwatch.StartNew();
            foreach (var (ia, ib) in pairs)
            {
                var A = sp[ia]; var B = sp[ib];
                var hit = new DtRaycastHit();
                var st = q.Raycast(A.Ref, A.Pos, B.Pos, filter, 0, ref hit, 0L);
                if (st.Failed())
                {
                    rayFailed++;
                }
                else if (hit.t < 1.0f)
                {
                    hits++;
                }
            }

            tRay.Add(sw.Elapsed.TotalMilliseconds);
        }

        for (int rep = 0; rep < reps; rep++)
        {
            var sw = Stopwatch.StartNew();
            for (int i = 0; i + 2 < pts.Length; i += 3)
            {
                q.FindNearestPoly(new RcVec3f(pts[i], pts[i + 1], pts[i + 2]), Extents, filter, out long _, out RcVec3f _, out bool _);
            }

            tNear.Add(sw.Elapsed.TotalMilliseconds);
        }

        Out.Row("query", scen, Side, "path_time_best", Out.Best(tPath), "ms");
        Out.Row("query", scen, Side, "path_per_sec", pairs.Count / (Out.Best(tPath) / 1000.0), "1/s");
        Out.Row("query", scen, Side, "path_mean_length", ok > 0 ? lenSum / ok : 0, "world_units");
        Out.Row("query", scen, Side, "path_found", ok, "count");
        Out.Row("query", scen, Side, "path_partial", partial, "count");
        Out.Row("query", scen, Side, "raycast_time_best", Out.Best(tRay), "ms");
        Out.Row("query", scen, Side, "raycast_per_sec", pairs.Count / (Out.Best(tRay) / 1000.0), "1/s");
        Out.Row("query", scen, Side, "raycast_hits", hits, "count");
        Out.Row("query", scen, Side, "raycast_failed", rayFailed, "count");
        Out.Row("query", scen, Side, "nearest_time_best", Out.Best(tNear), "ms");
        Out.Row("query", scen, Side, "nearest_per_sec", (pts.Length / 3) / (Out.Best(tNear) / 1000.0), "1/s");
        Out.Row("query", scen, Side, "peak_rss", Out.PeakRssMb(), "MB");
        return 0;
    }

    private static int Crowd(Args a)
    {
        if (!Input.LoadObj(a.Str("obj"), out var verts, out var tris))
        {
            Console.Error.WriteLine("cannot read --obj");
            return 2;
        }

        var pts = Input.LoadPoints(a.Str("points"));
        var s = new Settings(a);
        int n = a.Int("agents", 100), steps = a.Int("steps", 100), warm = a.Int("warmup", 20);
        float dt = (float)a.Num("dt", 0.05);
        string scen = a.Str("scenario", a.Str("obj") + "#crowd" + n);
        var r = BuildAll(s, new Geom(verts, tris));
        var nav = MakeNavMesh(s, r);
        var q = new DtNavMeshQuery(nav);
        var sp = Snap(q, pts).FindAll(x => x.Ref != 0);
        if (sp.Count < 2)
        {
            Console.Error.WriteLine("not enough usable points");
            return 1;
        }

        var crowd = new DtCrowd(new DtCrowdConfig(s.AgentRadius), nav);
        var ap = new DtCrowdAgentParams();
        ap.radius = s.AgentRadius; ap.height = s.AgentHeight; ap.maxAcceleration = 8.0f; ap.maxSpeed = 3.5f;
        ap.collisionQueryRange = ap.radius * 12.0f; ap.pathOptimizationRange = ap.radius * 30.0f;
        ap.updateFlags = DtCrowdAgentUpdateFlags.DT_CROWD_ANTICIPATE_TURNS | DtCrowdAgentUpdateFlags.DT_CROWD_OPTIMIZE_VIS | DtCrowdAgentUpdateFlags.DT_CROWD_OPTIMIZE_TOPO | DtCrowdAgentUpdateFlags.DT_CROWD_OBSTACLE_AVOIDANCE | DtCrowdAgentUpdateFlags.DT_CROWD_SEPARATION;
        ap.obstacleAvoidanceType = 0; ap.separationWeight = 2.0f;
        int added = 0;
        for (int i = 0; i < n; i++)
        {
            var st = sp[i % sp.Count];
            var goal = sp[(int)(((long)i * 7 + sp.Count / 2) % sp.Count)];
            var ag = crowd.AddAgent(st.Pos, ap);
            if (ag == null)
            {
                continue;
            }

            added++;
            crowd.RequestMoveTarget(ag, goal.Ref, goal.Pos);
        }

        for (int i = 0; i < warm; i++)
        {
            crowd.Update(dt, null);
        }

        var sw = Stopwatch.StartNew();
        for (int i = 0; i < steps; i++)
        {
            crowd.Update(dt, null);
        }

        double ms = sw.Elapsed.TotalMilliseconds;
        int moving = 0;
        foreach (var ag in crowd.GetActiveAgents())
        {
            if (ag.vel.Length() > 0.1f)
            {
                moving++;
            }
        }

        Out.Row("crowd", scen, Side, "agents", added, "count");
        Out.Row("crowd", scen, Side, "steps", steps, "count");
        Out.Row("crowd", scen, Side, "time", ms, "ms");
        Out.Row("crowd", scen, Side, "agents_per_ms", (double)added * steps / ms, "agent-steps/ms");
        Out.Row("crowd", scen, Side, "moving_at_end", moving, "count");
        Out.Row("crowd", scen, Side, "peak_rss", Out.PeakRssMb(), "MB");
        return 0;
    }
}
