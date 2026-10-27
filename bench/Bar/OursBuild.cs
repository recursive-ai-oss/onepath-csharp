using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of the navmesh build rows, and the mesh the query rows use: the OBJ level is built with our 3D builder on the settings the reference
/// driver prints for the same row (cell 0.3, cell height 0.2, agent 0.6 / 2 / 0.9 / 45 degrees, region sizes 8 and 20, edge 12 cells, error 1.3, 6 vertices,
/// detail distance 6 cells and error 1 row). Solo rows build the whole level in one tile; tiled rows use tiles of <c>--tile-size</c> cells
/// from the level's minimum corner. The metrics have the reference's names; coverage is the sum of polygon areas projected on the ground plane.
/// </summary>
internal static class OursBuild
{
    public static List<Row>? Run(SuiteRow row, Paths paths)
    {
        string[] t = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        if (t.Length == 0 || t[0] != "build")
        {
            return null;
        }
        string? obj = Arg(t, "--obj");
        if (obj is null || !File.Exists(obj))
        {
            return new List<Row> { new(row.Part, row.Id, OurScenarios.Side, "error", "missing --obj file", "text") };
        }
        bool tiled = Arg(t, "--mode") == "tiled";
        int tileCells = int.Parse(Arg(t, "--tile-size") ?? "64", CultureInfo.InvariantCulture);
        int reps = int.Parse(Arg(t, "--reps") ?? "5", CultureInfo.InvariantCulture);
        (double[] verts, int[] idx) = LoadObj(obj);
        var times = new List<double>();
        NavMesh? last = null;
        for (int i = 0; i < reps; i++)
        {
            long start = Stopwatch.GetTimestamp();
            last = Build(verts, idx, tiled, tileCells);
            times.Add((Stopwatch.GetTimestamp() - start) * 1000.0 / Stopwatch.Frequency);
        }
        times.Sort();
        long polys = 0, nv = 0;
        int tiles = 0;
        for (int s = 0; s < last!.Tiles.SlotCount; s++)
        {
            NavTile? tile = last.Tiles.GetTileBySlot(s);
            if (tile is null)
            {
                continue;
            }
            tiles++;
            polys += tile.PolyVertexCounts.Length;
            nv += tile.Vertices.Length / 3;
        }
        string side = OurScenarios.Side;
        Row R(string metric, double v, string unit) => new(row.Part, row.Id, side, metric, v.ToString("R", CultureInfo.InvariantCulture), unit);
        return new List<Row>
        {
            R("time_best", times[0], "ms"),
            R("time_median", times[times.Count / 2], "ms"),
            R("polys", polys, "count"),
            R("verts", nv, "count"),
            R("coverage_area", Coverage(last.Tiles), "world_units2"),
            R("tiles", tiles, "count"),
            R("input_tris", idx.Length / 3, "count"),
            R("peak_rss", Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0), "MB"),
            new Row(row.Part, row.Id, side, "settings", Describe(tiled, tileCells), "text"),
        };
    }

    public static string Describe(bool tiled, int tileCells) =>
        "cs=0.3 ch=0.2 h=2 r=0.6 climb=0.9 slope=45 regmin=8 regmerge=20 edge=12 err=1.3 nvp=6 dd=6 de=1" + (tiled ? " tile=" + tileCells.ToString(CultureInfo.InvariantCulture) : string.Empty);

    /// <summary>Builds the level on the reference settings.</summary>
    public static NavMesh Build(double[] verts, int[] idx, bool tiled, int tileCells)
    {
        double minX = double.MaxValue, minZ = double.MaxValue;
        for (int i = 0; i < verts.Length; i += 3)
        {
            minX = Math.Min(minX, verts[i]);
            minZ = Math.Min(minZ, verts[i + 2]);
        }
        var s = new NavSettings
        {
            CellSize = (double)0.3f,
            CellHeight = (double)0.2f,
            AgentHeight = 2f,
            AgentRadius = 0.6f,
            WalkableClimb = 0.9f,
            AgentMaxSlopeDegrees = 45f,
            MaxVertsPerPoly = 6,
            MinRegionArea = 64,
            MergeRegionArea = 400,
            MaxEdgeLength = 12 * (double)0.3f,
            ContourMaxError = 1.3f,
            DetailSampleDistance = 6 * (double)0.3f,
            DetailMaxError = 1 * (double)0.2f,
            OriginX = minX,
            OriginZ = minZ,
            // A tile of tileCells cells; the solo build is one tile large enough for any level in the suite.
            TileSize = tiled ? (double)(tileCells * 0.3f) : 512,
        };
        return NavMesh.Build(verts, idx, s);
    }

    public static double Coverage(TiledNavMesh m)
    {
        double sum = 0;
        for (int slot = 0; slot < m.SlotCount; slot++)
        {
            NavTile? t = m.GetTileBySlot(slot);
            if (t is null)
            {
                continue;
            }
            ReadOnlySpan<float> v = t.Vertices;
            int pos = 0;
            foreach (byte n in t.PolyVertexCounts)
            {
                double a2 = 0;
                for (int e = 0; e < n; e++)
                {
                    int i0 = t.PolyIndices[pos + e], i1 = t.PolyIndices[pos + ((e + 1) % n)];
                    a2 += ((double)v[3 * i0] * v[(3 * i1) + 2]) - ((double)v[3 * i1] * v[(3 * i0) + 2]);
                }
                sum += Math.Abs(a2) / 2;
                pos += n;
            }
        }
        return sum;
    }

    public static string? Arg(string[] t, string name)
    {
        for (int i = 0; i + 1 < t.Length; i++)
        {
            if (t[i] == name)
            {
                return t[i + 1];
            }
        }
        return null;
    }

    /// <summary>Reads vertices and faces of a Wavefront OBJ (faces fan-triangulated, negative indices relative).</summary>
    public static (double[] Vertices, int[] Indices) LoadObj(string path)
    {
        var v = new List<double>();
        var ix = new List<int>();
        foreach (string line in File.ReadLines(path))
        {
            if (line.StartsWith("v ", StringComparison.Ordinal))
            {
                string[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                v.Add(double.Parse(f[1], CultureInfo.InvariantCulture));
                v.Add(double.Parse(f[2], CultureInfo.InvariantCulture));
                v.Add(double.Parse(f[3], CultureInfo.InvariantCulture));
            }
            else if (line.StartsWith("f ", StringComparison.Ordinal))
            {
                int[] f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries).Skip(1).Select(x =>
                {
                    int k = int.Parse(x.Split('/')[0], CultureInfo.InvariantCulture);
                    return k > 0 ? k - 1 : (v.Count / 3) + k;
                }).ToArray();
                for (int i = 1; i + 1 < f.Length; i++)
                {
                    ix.Add(f[0]);
                    ix.Add(f[i]);
                    ix.Add(f[i + 1]);
                }
            }
        }
        return (v.ToArray(), ix.ToArray());
    }
}
