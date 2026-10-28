using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the run-time rebuild rows (<c>tilecache</c>): the reference drivers build a tiled mesh with a tile cache, then add
/// <c>--obstacles</c> cylinders one at a time (each followed by the rebuild of the tiles it touches), then remove them the same way.
/// Obstacle k stands on the k-th of every (points / obstacles) query points, base half a unit under the point. We do the same with a
/// <see cref="TileCache"/>: <c>AddObstacle</c> then <c>UpdateAll</c>. Metrics have the reference's names: <c>time_best</c> is one
/// whole add-and-remove cycle, <c>cache_build_time</c> the build of the level with the cache, and polygon counts and walkable area
/// are taken with no obstacle, with all in force and after all are removed.
/// </summary>
internal static class OursTileCache
{
    public static bool Handles(SuiteRow row) => row.Part == "tilecache";

    public static List<Row> Run(SuiteRow row, Paths paths)
    {
        string[] t = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        string obj = OursBuild.Arg(t, "--obj") ?? "";
        string pointsFile = OursBuild.Arg(t, "--points") ?? "";
        int tileCells = int.Parse(OursBuild.Arg(t, "--tile-size") ?? "64", CultureInfo.InvariantCulture);
        int reps = int.Parse(OursBuild.Arg(t, "--reps") ?? "3", CultureInfo.InvariantCulture);
        int count = int.Parse(OursBuild.Arg(t, "--obstacles") ?? "16", CultureInfo.InvariantCulture);
        double radius = double.Parse(OursBuild.Arg(t, "--obstacle-radius") ?? "2", CultureInfo.InvariantCulture);
        double height = double.Parse(OursBuild.Arg(t, "--obstacle-height") ?? "3", CultureInfo.InvariantCulture);
        string side = OurScenarios.Side;
        if (!File.Exists(obj) || !File.Exists(pointsFile))
        {
            return [new Row(row.Part, row.Id, side, "error", "missing --obj or --points file", "text")];
        }

        (double[] verts, int[] idx) = OursBuild.LoadObj(obj);
        var points = new List<(double X, double Y, double Z)>();
        foreach (string line in File.ReadLines(pointsFile))
        {
            string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (f.Length >= 3 && f[0][0] != '#')
            {
                points.Add((double.Parse(f[0], CultureInfo.InvariantCulture), double.Parse(f[1], CultureInfo.InvariantCulture), double.Parse(f[2], CultureInfo.InvariantCulture)));
            }
        }

        var obstacles = new List<TileObstacle>();
        for (int k = 0; k < count; k++)
        {
            var p = points[k * (points.Count / count)];
            obstacles.Add(TileObstacle.Cylinder((double)(float)p.X, (double)(float)(p.Y - 0.5f), (double)(float)p.Z, radius, height));
        }

        var pipeline = new Mesh3DPipeline(OursBuild.SettingsFor(verts, true, tileCells));
        var buildTimes = new List<double>();
        var cycleTimes = new List<double>();
        TileCache? cache = null;
        for (int rep = 0; rep < reps; rep++)
        {
            long start = Stopwatch.GetTimestamp();
            cache = pipeline.BuildSoupCached(verts, idx);
            buildTimes.Add(Ms(start));
        }

        (int Polys, double Area) Measure()
        {
            int polys = 0;
            for (int s = 0; s < cache!.Mesh.SlotCount; s++)
            {
                polys += cache.Mesh.GetTileBySlot(s)?.PolyCount ?? 0;
            }

            return (polys, OursBuild.Coverage(cache.Mesh));
        }
        var initial = Measure();
        (int Polys, double Area) stacked = default, restored = default;
        for (int rep = 0; rep < reps; rep++)
        {
            var ids = new int[count];
            long start = Stopwatch.GetTimestamp();
            for (int k = 0; k < count; k++)
            {
                ids[k] = cache!.AddObstacle(obstacles[k]);
                cache.UpdateAll();
            }

            double mid = Ms(start);
            if (rep == 0)
            {
                stacked = Measure();
            }

            start = Stopwatch.GetTimestamp();
            for (int k = 0; k < count; k++)
            {
                cache!.RemoveObstacle(ids[k]);
                cache.UpdateAll();
            }

            cycleTimes.Add(mid + Ms(start));
            if (rep == 0)
            {
                restored = Measure();
            }
        }

        buildTimes.Sort();
        cycleTimes.Sort();
        int tiles = 0;
        for (int s = 0; s < cache!.Mesh.SlotCount; s++)
        {
            tiles += cache.Mesh.GetTileBySlot(s) is null ? 0 : 1;
        }

        Row R(string metric, double v, string unit) => new(row.Part, row.Id, side, metric, v.ToString("R", CultureInfo.InvariantCulture), unit);
        return
        [
            R("time_best", cycleTimes[0], "ms"),
            R("time_median", cycleTimes[cycleTimes.Count / 2], "ms"),
            R("cache_build_time", buildTimes[0], "ms"),
            R("obstacles", count, "count"),
            R("polys_initial", initial.Polys, "count"),
            R("polys", stacked.Polys, "count"),
            R("polys_restored", restored.Polys, "count"),
            R("coverage_area_initial", initial.Area, "world_units2"),
            R("coverage_area", stacked.Area, "world_units2"),
            R("coverage_area_restored", restored.Area, "world_units2"),
            R("tiles", tiles, "count"),
            R("layers", cache.CachedTileCount, "count"),
            R("input_tris", idx.Length / 3, "count"),
            R("peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
        ];
    }

    private static double Ms(long start) => (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
}
