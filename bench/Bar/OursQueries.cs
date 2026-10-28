using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the navmesh query rows: the same measurements, metric names and query-point handling as the reference
/// driver's <c>query</c> command (points paired as (0,1), (2,3), ...; every point snapped with extents (2, 4, 2); a path per
/// pair as a corridor plus a straight path, ending at the closest point of the last polygon when partial; a raycast per pair;
/// a nearest-polygon query per point). With an <c>--obj</c> level the mesh is our 3D builder's on the reference settings and the points are used as they are; without one, the mesh is the format test
/// generator's (8 by 8 tiles, one layer, about 64 by 64 units) and the points file is mapped onto it by scaling its X/Z bounding box;
/// the <c>mesh_source</c> row says so. Rows are therefore comparable in method, not yet in geometry.
/// </summary>
internal static class OursQueries
{
    public static List<Row> Run(SuiteRow row, Paths paths)
    {
        string[] t = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        string? pointsFile = null;
        string? objFile = OursBuild.Arg(t, "--obj");
        bool tiled = OursBuild.Arg(t, "--mode") == "tiled";
        int tileCells = int.Parse(OursBuild.Arg(t, "--tile-size") ?? "64", CultureInfo.InvariantCulture);
        int reps = 5;
        for (int i = 0; i + 1 < t.Length; i++)
        {
            if (t[i] == "--points")
            {
                pointsFile = t[i + 1];
            }
            else if (t[i] == "--reps")
            {
                reps = int.Parse(t[i + 1], CultureInfo.InvariantCulture);
            }
        }
        if (pointsFile is null || !File.Exists(pointsFile))
        {
            return new List<Row> { new(row.Part, row.Id, OurScenarios.Side, "error", "missing --points file", "text") };
        }
        var raw = new List<Vector3D>();
        foreach (string line in File.ReadLines(pointsFile))
        {
            if (line.StartsWith('#'))
            {
                continue; // the generator's header comment
            }
            string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (f.Length >= 3 && f[0][0] != (char)35)
            {
                raw.Add(new Vector3D(double.Parse(f[0], CultureInfo.InvariantCulture), double.Parse(f[1], CultureInfo.InvariantCulture), double.Parse(f[2], CultureInfo.InvariantCulture)));
            }
        }
        double x0 = raw.Min(p => p.X), x1 = raw.Max(p => p.X), z0 = raw.Min(p => p.Z), z1 = raw.Max(p => p.Z);
        TiledNavMesh mesh;
        List<Vector3D> pts;
        string meshSource;
        int objIndex = Array.IndexOf(t, "--obj");
        if (objIndex >= 0 && objIndex + 1 < t.Length && t[objIndex + 1].Contains("tilemap", StringComparison.Ordinal))
        {
            // The tile-map level: the mesh is ours built from the same tiles, the query points are the level's own.
            bool[] cells = RecursiveAI.OnePath.Bench.TileMapWorld.FromObj(t[objIndex + 1], out int tiles, out _);
            NavSettings settings = OursBuild2D.SettingsFor(t, tiles * RecursiveAI.OnePath.Bench.TileMapWorld.TileUnits);
            mesh = NavMesh.Build2D(cells, tiles, tiles, RecursiveAI.OnePath.Bench.TileMapWorld.TileUnits, new Vector2D(0, 0), settings).Tiles;
            pts = raw;
            meshSource = "NavMesh.Build2D from the level's tiles; the level's own query points";
        }
        else if (objFile is not null && File.Exists(objFile))
        {
            // The level's own mesh, built with the reference settings, and the points as they are.
            (double[] ov, int[] oi) = OursBuild.LoadObj(objFile);
            mesh = OursBuild.Build(ov, oi, tiled, tileCells).Tiles;
            pts = raw;
            meshSource = "3D builder, " + OursBuild.Describe(tiled, tileCells);
        }
        else
        {
            mesh = NavMeshWorlds.RandomMesh(7, -4, -4, 8, 8, 1, detail: false, offMesh: 0);
            pts = raw.Select(p => new Vector3D(-47 + ((p.X - x0) / Math.Max(1e-9, x1 - x0) * 62), 0.2, -47 + ((p.Z - z0) / Math.Max(1e-9, z1 - z0) * 62))).ToList();
            meshSource = "format test generator (8x8 tiles); points scaled onto it";
        }

        // Search node model: --node-model in the row, else the ONEPATH_NODE_MODEL environment variable (polygon or crossing), else the query default.
        string? model = OursBuild.Arg(t, "--node-model") ?? Environment.GetEnvironmentVariable("ONEPATH_NODE_MODEL");
        NavMeshQuery q = string.Equals(model, "polygon", StringComparison.OrdinalIgnoreCase) ? new NavMeshQuery(mesh, 65536, PathNodeModel.Polygon)
            : string.Equals(model, "crossing", StringComparison.OrdinalIgnoreCase) ? new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing) : new NavMeshQuery(mesh);
        var filter = new PlainFilterStruct();
        var ext = new Vector3D(2, 4, 2);
        var snapped = new NearestPoly[pts.Count];
        for (int i = 0; i < pts.Count; i++)
        {
            snapped[i] = q.FindNearestPoly(pts[i], ext, in filter);
        }
        var pairs = new List<(int, int)>();
        for (int i = 0; i + 1 < snapped.Length; i += 2)
        {
            if (snapped[i].Poly.IsSet && snapped[i + 1].Poly.IsSet)
            {
                pairs.Add((i, i + 1));
            }
        }
        var corridor = new PolyRef[4096];
        var straight = new StraightPathPoint[4096];
        var visited = new PolyRef[4096];
        double lenSum = 0;
        long ok = 0, partial = 0, hits = 0;
        var tPath = new List<double>();
        var tRay = new List<double>();
        var tNear = new List<double>();
        for (int rep = 0; rep < reps; rep++)
        {
            lenSum = 0;
            ok = 0;
            partial = 0;
            long start = Stopwatch.GetTimestamp();
            foreach (var (a, b) in pairs)
            {
                NearestPoly A = snapped[a], B = snapped[b];
                PathStatus s = q.FindPath(A.Poly, B.Poly, A.Point, B.Point, in filter, corridor, out int n);
                if (n <= 0)
                {
                    continue;
                }
                Vector3D end = B.Point;
                if (s == PathStatus.Partial)
                {
                    partial++;
                    q.ClosestPointOnPoly(corridor[n - 1], B.Point, out end, out _);
                }
                q.FindStraightPath(A.Point, end, corridor.AsSpan(0, n), straight, out int sc, out _);
                for (int i = 1; i < sc; i++)
                {
                    lenSum += Vector3D.Distance(straight[i - 1].Position, straight[i].Position);
                }
                ok++;
            }
            tPath.Add(Ms(start));
        }
        for (int rep = 0; rep < reps; rep++)
        {
            hits = 0;
            long start = Stopwatch.GetTimestamp();
            foreach (var (a, b) in pairs)
            {
                RaycastResult r = q.Raycast(snapped[a].Poly, snapped[a].Point, snapped[b].Point, in filter, visited, out _);
                if (r.Hit)
                {
                    hits++;
                }
            }
            tRay.Add(Ms(start));
        }
        for (int rep = 0; rep < reps; rep++)
        {
            long start = Stopwatch.GetTimestamp();
            foreach (Vector3D p in pts)
            {
                _ = q.FindNearestPoly(p, ext, in filter);
            }
            tNear.Add(Ms(start));
        }
        string id = row.Id;
        string side = OurScenarios.Side;
        Row R(string metric, double v, string unit) => new(row.Part, id, side, metric, v.ToString("R", CultureInfo.InvariantCulture), unit);
        double np = pairs.Count;
        return new List<Row>
        {
            R("points_snapped", snapped.Count(s => s.Poly.IsSet), "count"),
            R("pairs", np, "count"),
            R("path_time_best", tPath.Min(), "ms"),
            R("path_per_sec", np / (tPath.Min() / 1000.0), "1/s"),
            R("path_mean_length", ok > 0 ? lenSum / ok : 0, "world_units"),
            R("path_found", ok, "count"),
            R("path_partial", partial, "count"),
            R("raycast_time_best", tRay.Min(), "ms"),
            R("raycast_per_sec", np / (tRay.Min() / 1000.0), "1/s"),
            R("raycast_hits", hits, "count"),
            R("nearest_time_best", tNear.Min(), "ms"),
            R("nearest_per_sec", pts.Count / (tNear.Min() / 1000.0), "1/s"),
            R("peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
            new Row(row.Part, id, side, "mesh_source", meshSource, "text"),
            new Row(row.Part, id, side, "path_node_model", q.NodeModel.ToString(), "text"),
        };
    }

    private static double Ms(long start) => (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;

    private struct PlainFilterStruct : IQueryFilter
    {
        public readonly bool Passes(ushort flags, byte area) => true;

        public readonly double AreaCost(byte area) => 1.0;

        public readonly double MinAreaCost => 1.0;
    }
}
