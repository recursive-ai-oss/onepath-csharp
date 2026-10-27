using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Bench;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the tile-map build rows: the reference builds a navmesh from the level's flat triangles; we build the
/// same footprint from its tiles with <see cref="NavMesh.Build2D(ReadOnlySpan{bool}, int, int, double, Vector2D, NavSettings?)"/>.
/// Settings are read from the same command line the reference gets (<c>--cell</c>, <c>--agent-radius</c>, <c>--tile-size</c> in
/// cells, <c>--mode</c>, <c>--verts-per-poly</c>, <c>--reps</c>), so both sides run the same numbers. Metric names match the
/// reference driver's <c>build</c> command; coverage is the sum of polygon areas on the ground plane.
/// </summary>
internal static class OursBuild2D
{
    public static bool Handles(SuiteRow row) => row.Part == "build" && row.Args.Contains("tilemap", StringComparison.Ordinal);

    /// <summary>The settings a row's command line asks for.</summary>
    public static NavSettings SettingsFor(string[] t, double extent)
    {
        double cell = Num(t, "--cell", 0.3);
        double radius = Num(t, "--agent-radius", 0.6);
        int tileCells = (int)Num(t, "--tile-size", 64);
        bool tiled = Str(t, "--mode", "solo") == "tiled";
        return NavSettings.Humanoid with
        {
            CellSize = cell,
            // The reference's floors end one cell short of every edge before it erodes by the radius (its coverage on this
            // level matches ours at radius + one cell and is 45% below ours at the nominal radius), so ours takes the same
            // effective radius; the settings row says so.
            AgentRadius = (float)(radius + cell),
            AgentHeight = (float)Num(t, "--agent-height", 2.0),
            MaxVertsPerPoly = (int)Num(t, "--verts-per-poly", 6),
            // One tile spanning the level stands in for the reference's solo mode.
            TileSize = tiled ? tileCells * cell : Math.Ceiling(extent) + 1,
        };
    }

    public static List<Row> Run(SuiteRow row, Paths paths)
    {
        string[] t = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        string obj = Str(t, "--obj", "");
        int reps = (int)Num(t, "--reps", 5);
        bool[] cells = TileMapWorld.FromObj(obj, out int tiles, out int triangles);
        double extent = tiles * TileMapWorld.TileUnits;
        NavSettings settings = SettingsFor(t, extent);
        var times = new List<double>();
        NavMesh? mesh = null;
        for (int rep = 0; rep < reps; rep++)
        {
            long start = Stopwatch.GetTimestamp();
            mesh = NavMesh.Build2D(cells, tiles, tiles, TileMapWorld.TileUnits, new Vector2D(0, 0), settings);
            times.Add((Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency);
        }
        (int polys, int verts, int tileCount, double area) = Measure(mesh!);
        times.Sort();
        string side = OurScenarios.Side;
        Row R(string metric, double v, string unit) => new(row.Part, row.Id, side, metric, v.ToString("R", CultureInfo.InvariantCulture), unit);
        return new List<Row>
        {
            R("time_best", times[0], "ms"),
            R("time_median", times[times.Count / 2], "ms"),
            R("polys", polys, "count"),
            R("verts", verts, "count"),
            R("coverage_area", area, "world_units2"),
            R("tiles", tileCount, "count"),
            R("input_tris", triangles, "count"),
            R("peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
            new Row(row.Part, row.Id, side, "settings", $"cell={settings.CellSize:R} r={settings.AgentRadius:R} (nominal + one cell) tile={settings.TileSize:R} nvp={settings.MaxVertsPerPoly}; tile map {tiles}x{tiles}", "text"),
        };
    }

    /// <summary>Polygon, vertex and tile counts and the walkable area (sum of polygon areas projected on the ground plane).</summary>
    public static (int Polys, int Verts, int Tiles, double Area) Measure(NavMesh mesh)
    {
        int polys = 0, verts = 0, tiles = 0;
        double area = 0;
        for (int s = 0; s < mesh.Tiles.SlotCount; s++)
        {
            NavTile? tile = mesh.Tiles.GetTileBySlot(s);
            if (tile is null)
            {
                continue;
            }
            tiles++;
            polys += tile.PolyCount;
            verts += tile.VertexCount;
            ReadOnlySpan<float> v = tile.Vertices;
            ReadOnlySpan<ushort> idx = tile.PolyIndices;
            ReadOnlySpan<byte> counts = tile.PolyVertexCounts;
            for (int p = 0; p < tile.PolyCount; p++)
            {
                int first = tile.PolyFirstIndex(p), n = counts[p];
                double twice = 0;
                for (int i = 0; i < n; i++)
                {
                    int a = idx[first + i], b = idx[first + ((i + 1) % n)];
                    twice += ((double)v[3 * a] * v[(3 * b) + 2]) - ((double)v[3 * b] * v[(3 * a) + 2]);
                }
                area += Math.Abs(twice) * 0.5;
            }
        }
        return (polys, verts, tiles, area);
    }

    private static double Num(string[] t, string key, double fallback)
    {
        for (int i = 0; i + 1 < t.Length; i++)
        {
            if (t[i] == key)
            {
                return double.Parse(t[i + 1], CultureInfo.InvariantCulture);
            }
        }
        return fallback;
    }

    private static string Str(string[] t, string key, string fallback)
    {
        for (int i = 0; i + 1 < t.Length; i++)
        {
            if (t[i] == key)
            {
                return t[i + 1];
            }
        }
        return fallback;
    }
}
