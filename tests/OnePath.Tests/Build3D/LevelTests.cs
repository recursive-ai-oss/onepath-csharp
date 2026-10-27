using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using RecursiveAI.OnePath.NavMesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>
/// Builds the generated benchmark levels (tools/geom/gen_levels.py writes them) through the span stage. Skipped unless
/// ONEPATH_LEVELS names a folder holding the generated .obj files.
/// </summary>
public class LevelTests
{
    private static (double[] Vertices, int[] Indices) LoadObj(string path)
    {
        var v = new List<double>();
        var f = new List<int>();
        foreach (string line in File.ReadLines(path))
        {
            string[] p = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (p.Length == 4 && p[0] == "v")
            {
                for (int i = 1; i < 4; i++)
                {
                    v.Add(double.Parse(p[i], CultureInfo.InvariantCulture));
                }
            }
            else if (p.Length == 4 && p[0] == "f")
            {
                for (int i = 1; i < 4; i++)
                {
                    f.Add(int.Parse(p[i], CultureInfo.InvariantCulture) - 1);
                }
            }
        }
        return (v.ToArray(), f.ToArray());
    }

    [Fact]
    public void GeneratedLevelsBuild()
    {
        string? dir = Environment.GetEnvironmentVariable("ONEPATH_LEVELS");
        Assert.SkipUnless(!string.IsNullOrEmpty(dir) && Directory.Exists(dir), "ONEPATH_LEVELS is not set");
        var lines = new List<string>();
        foreach (string file in Directory.GetFiles(dir!, "*.obj"))
        {
            var (verts, idx) = LoadObj(file);
            double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue;
            double maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
            for (int i = 0; i < verts.Length; i += 3)
            {
                minX = Math.Min(minX, verts[i]);
                maxX = Math.Max(maxX, verts[i]);
                minY = Math.Min(minY, verts[i + 1]);
                maxY = Math.Max(maxY, verts[i + 1]);
                minZ = Math.Min(minZ, verts[i + 2]);
                maxZ = Math.Max(maxZ, verts[i + 2]);
            }
            var frame = HeightfieldFrame.FromBounds(minX, minY - 1, minZ, maxX, maxY + 4, maxZ, 0.3f, 0.2f);
            var settings = SpanSettings.Humanoid;
            var solid = new SolidHeightfield(frame);
            var stats = Voxelizer.Rasterize(solid, verts, idx, settings);
            int solidSpans = solid.SpanCount;
            var pipe = new SpanPipeline(settings);
            var chf = pipe.Run(solid);
            int walkable = 0;
            for (int i = 0; i < chf.SpanCount; i++)
            {
                walkable += chf.Spans[i].Area != 0 ? 1 : 0;
            }
            lines.Add(string.Create(CultureInfo.InvariantCulture, $"{Path.GetFileNameWithoutExtension(file)}: {idx.Length / 3} tris ({stats.Degenerate} degenerate), grid {frame.Width}x{frame.Depth}x{frame.Height} = {frame.ColumnCount} columns, {solidSpans} solid spans, {chf.SpanCount} walkable before erosion, {walkable} after"));
            Assert.True(chf.SpanCount > 0, file);
        }
        File.WriteAllLines(Path.Combine(dir!, "span-stage-report.txt"), lines);
        Assert.NotEmpty(lines);
    }
}
