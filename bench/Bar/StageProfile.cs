using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Per-stage timing of the 3D builder, switched on with <c>ONEPATH_PROFILE=1</c>. Every stage of <see cref="Mesh3DPipeline"/> is a replaceable
/// delegate, so each is wrapped with a stopwatch that adds up the time over all tiles; what no delegate covers (triangle binning, voxelizing,
/// tile data, stitching) is reported as <c>stage_other</c>. The wrappers are the library's public tier-2 hooks, not a patched build: the
/// overhead is two clock reads per stage per tile. Rows are <c>stage_*_ms</c> of the best of several builds (the build with the lowest total).
/// </summary>
internal sealed class StageProfile
{
    private static readonly string[] Names =
    [
        "low_obstacle_filter", "ledge_filter", "low_height_filter", "compact", "erode", "regions", "border_cut_and_contours", "polygons", "detail",
    ];

    private readonly double[] _ticks = new double[Names.Length];

    /// <summary>Wraps every stage of <paramref name="p"/> so its time is added to this profile.</summary>
    public void Attach(Mesh3DPipeline p)
    {
        SpanPipeline sp = p.Regions.Spans;
        Action<SolidHeightfield, SpanSettings>? lo = sp.LowObstacleFilter, le = sp.LedgeFilter, lh = sp.LowHeightFilter;
        Func<SolidHeightfield, SpanSettings, CompactHeightfield> compact = sp.Compact;
        Action<CompactHeightfield, SpanSettings>? erode = sp.Erode;
        Action<CompactHeightfield, RegionSettings> part = p.Regions.Partition;
        Func<CompactHeightfield, ContourSettings, ContourSet> trace = p.Regions.Trace;
        var poly = p.Polygonize;
        var detail = p.BuildDetail;
        if (lo is not null) { sp.LowObstacleFilter = (h, s) => { long t = Stopwatch.GetTimestamp(); lo(h, s); _ticks[0] += Stopwatch.GetTimestamp() - t; }; }
        if (le is not null) { sp.LedgeFilter = (h, s) => { long t = Stopwatch.GetTimestamp(); le(h, s); _ticks[1] += Stopwatch.GetTimestamp() - t; }; }
        if (lh is not null) { sp.LowHeightFilter = (h, s) => { long t = Stopwatch.GetTimestamp(); lh(h, s); _ticks[2] += Stopwatch.GetTimestamp() - t; }; }
        sp.Compact = (h, s) => { long t = Stopwatch.GetTimestamp(); var r = compact(h, s); _ticks[3] += Stopwatch.GetTimestamp() - t; return r; };
        if (erode is not null) { sp.Erode = (c, s) => { long t = Stopwatch.GetTimestamp(); erode(c, s); _ticks[4] += Stopwatch.GetTimestamp() - t; }; }
        p.Regions.Partition = (c, s) => { long t = Stopwatch.GetTimestamp(); part(c, s); _ticks[5] += Stopwatch.GetTimestamp() - t; };
        p.Regions.Trace = (c, s) => { long t = Stopwatch.GetTimestamp(); var r = trace(c, s); _ticks[6] += Stopwatch.GetTimestamp() - t; return r; };
        p.Polygonize = (c, n, b, m) => { long t = Stopwatch.GetTimestamp(); var r = poly(c, n, b, m); _ticks[7] += Stopwatch.GetTimestamp() - t; return r; };
        p.BuildDetail = (m, c, tc, d) => { long t = Stopwatch.GetTimestamp(); var r = detail(m, c, tc, d); _ticks[8] += Stopwatch.GetTimestamp() - t; return r; };
    }

    public void Reset() => Array.Clear(_ticks);

    /// <summary>Stage times in ms in <see cref="Names"/> order.</summary>
    public double[] Ms()
    {
        var r = new double[Names.Length];
        for (int i = 0; i < r.Length; i++)
        {
            r[i] = _ticks[i] * 1000.0 / Stopwatch.Frequency;
        }

        return r;
    }

    /// <summary>Profiles a whole-level build (<paramref name="cycle"/> false) or the obstacle cycle (true) and returns the rows.</summary>
    public static List<Row> Run(SuiteRow row, double[] verts, int[] idx, NavSettings settings)
    {
        var prof = new StageProfile();
        var pipe = new Mesh3DPipeline(settings);
        prof.Attach(pipe);
        OursBuild.WarmUp(() => pipe.BuildSoup(verts, idx));
        double[]? best = null;
        double bestTotal = double.MaxValue;
        for (int i = 0; i < 5; i++)
        {
            prof.Reset();
            long start = Stopwatch.GetTimestamp();
            _ = pipe.BuildSoup(verts, idx);
            double total = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
            if (total < bestTotal)
            {
                bestTotal = total;
                best = prof.Ms();
                best = [.. best, total];
            }
        }

        List<Row> rows = Rows(row, best!);
        rows.Add(new Row(row.Part, row.Id, OurScenarios.Side, "stage_voxelize_standalone_ms", Voxelize(pipe, settings, verts, idx).ToString("F3", CultureInfo.InvariantCulture), "ms"));
        return rows;
    }

    /// <summary>
    /// Time of the voxelize step alone (classify and rasterize the whole input into one heightfield of the first tile's size with the
    /// pipeline's border), best of 5 after warm-up. It is part of <c>stage_other_ms</c>, listed apart because no hook sits on it; for a
    /// solo build it is the whole step, for a tiled build it is a full-level raster into a tile-sized grid and so only an upper bound
    /// of the per-triangle cost.
    /// </summary>
    private static double Voxelize(Mesh3DPipeline pipe, NavSettings settings, double[] verts, int[] idx)
    {
        double minY = double.MaxValue, maxY = double.MinValue, minX = double.MaxValue, minZ = double.MaxValue;
        for (int i = 0; i < verts.Length; i += 3)
        {
            minX = Math.Min(minX, verts[i]);
            minY = Math.Min(minY, verts[i + 1]);
            maxY = Math.Max(maxY, verts[i + 1]);
            minZ = Math.Min(minZ, verts[i + 2]);
        }

        double originY = minY - settings.CellHeight;
        int rows = (int)Math.Ceiling((maxY - originY) / settings.CellHeight) + pipe.SpanOptions.WalkableHeight + 2;
        int side = pipe.TileCells + (2 * pipe.Border);
        var frame = new HeightfieldFrame(minX - (pipe.Border * pipe.CellSize), originY, minZ - (pipe.Border * pipe.CellSize), (float)pipe.CellSize, (float)settings.CellHeight, side, side, rows);
        var areas = new byte[idx.Length / 3];
        double best = double.MaxValue;
        for (int i = 0; i < 25; i++)
        {
            long start = Stopwatch.GetTimestamp();
            var solid = new SolidHeightfield(frame);
            Voxelizer.ClassifyTriangles(frame, verts, idx, pipe.SpanOptions.WalkableSlopeCosine, areas);
            Voxelizer.Rasterize(solid, verts, idx, areas, pipe.SpanOptions.EffectiveMergeThreshold);
            if (i >= 5)
            {
                best = Math.Min(best, (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency);
            }
        }

        return best;
    }

    internal static List<Row> Rows(SuiteRow row, double[] ms)
    {
        var rows = new List<Row>();
        double sum = 0;
        for (int i = 0; i < Names.Length; i++)
        {
            sum += ms[i];
            rows.Add(new Row(row.Part, row.Id, OurScenarios.Side, "stage_" + Names[i] + "_ms", ms[i].ToString("F3", CultureInfo.InvariantCulture), "ms"));
        }

        double total = ms[Names.Length];
        rows.Add(new Row(row.Part, row.Id, OurScenarios.Side, "stage_other_ms", (total - sum).ToString("F3", CultureInfo.InvariantCulture), "ms"));
        rows.Add(new Row(row.Part, row.Id, OurScenarios.Side, "stage_total_ms", total.ToString("F3", CultureInfo.InvariantCulture), "ms"));
        return rows;
    }

    /// <summary>Same profile for the tile cache: the build with the cache, then one add-and-remove cycle of the obstacles (only the obstacle-dependent stages run again).</summary>
    public static List<Row> RunCycle(SuiteRow row, double[] verts, int[] idx, NavSettings settings, List<TileObstacle> obstacles)
    {
        var prof = new StageProfile();
        var pipe = new Mesh3DPipeline(settings);
        prof.Attach(pipe);
        TileCache cache = pipe.BuildSoupCached(verts, idx);
        double[]? best = null;
        double bestTotal = double.MaxValue;
        for (int i = 0; i < 8; i++)
        {
            prof.Reset();
            var ids = new int[obstacles.Count];
            long start = Stopwatch.GetTimestamp();
            for (int k = 0; k < obstacles.Count; k++)
            {
                ids[k] = cache.AddObstacle(obstacles[k]);
                cache.UpdateAll();
            }

            for (int k = 0; k < obstacles.Count; k++)
            {
                cache.RemoveObstacle(ids[k]);
                cache.UpdateAll();
            }

            double total = (Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency;
            if (i >= 3 && total < bestTotal)
            {
                bestTotal = total;
                best = [.. prof.Ms(), total];
            }
        }

        return Rows(row, best!);
    }
}
