using System;
using System.Collections.Generic;
using System.Globalization;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Scope;

/// <summary>One drawable piece of a stage's output.</summary>
internal readonly record struct StagePrim(byte Kind, Vector3 A, Vector3 B, Vector3 C, Color32 Color);

/// <summary>What one build stage produced: its hash, a one-line summary, and what to draw.</summary>
internal sealed class StageResult(string name, ulong hash, string summary, List<StagePrim> prims, int total)
{
    public string Name { get; } = name;

    public ulong Hash { get; } = hash;

    public string Summary { get; } = summary;

    public List<StagePrim> Prims { get; } = prims;

    /// <summary>How many elements the stage produced; only <see cref="Prims"/> of them are drawn when this is above <see cref="BuildStages.DrawCap"/>.</summary>
    public int Total { get; } = total;
}

/// <summary>
/// Runs the library's public build stages on a whole input as one tile, in order, keeping each stage's output hash and drawing.
/// The stages are the ones <see cref="Mesh3DPipeline"/> runs per tile (rasterize, the three span filters, compaction and erosion,
/// region partition, contour trace, polygonize, detail), called one after another with the pipeline's own settings.
/// </summary>
internal sealed class BuildStages
{
    internal const int DrawCap = 200_000;
    internal const byte PrimLine = 0, PrimBox = 1, PrimTriangle = 2;

    private readonly Lazy<StageResult[]> _stages;

    public BuildStages(BuildInput.Parsed input)
    {
        Input = input;
        _stages = new Lazy<StageResult[]>(Run);
    }

    public BuildInput.Parsed Input { get; }

    public StageResult[] All
    {
        get
        {
            try
            {
                return _stages.Value;
            }
            catch (ArgumentException e)
            {
                throw new ScopeFormatException("build input", "expected geometry and settings the library's build stages accept: " + e.Message);
            }
        }
    }

    private StageResult[] Run()
    {
        NavSettings st = Input.Settings;
        var pipe = new Mesh3DPipeline(st);
        float cs = (float)st.CellSize, ch = (float)st.CellHeight;
        double[] v = Input.Vertices;
        double minX = double.MaxValue, minY = double.MaxValue, minZ = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue, maxZ = double.MinValue;
        foreach (int i in Input.Indices)
        {
            minX = Math.Min(minX, v[i * 3]);
            maxX = Math.Max(maxX, v[i * 3]);
            minY = Math.Min(minY, v[(i * 3) + 1]);
            maxY = Math.Max(maxY, v[(i * 3) + 1]);
            minZ = Math.Min(minZ, v[(i * 3) + 2]);
            maxZ = Math.Max(maxZ, v[(i * 3) + 2]);
        }

        if (Input.Indices.Length == 0)
        {
            throw new ScopeFormatException("build input", "expected at least one triangle to build from.");
        }

        // Room above the highest vertex for the agent's height, and a flat input still gets a positive extent.
        maxY += (pipe.SpanOptions.WalkableHeight + 1) * (double)ch;
        minY -= ch;
        maxX = Math.Max(maxX, minX + cs);
        maxZ = Math.Max(maxZ, minZ + cs);
        HeightfieldFrame frame;
        try
        {
            frame = HeightfieldFrame.FromBounds(minX, minY, minZ, maxX, maxY, maxZ, cs, ch);
        }
        catch (ArgumentException e)
        {
            throw new ScopeFormatException("build input", "expected an input whose voxel grid fits the library's limits: " + e.Message);
        }

        if ((long)frame.Width * frame.Depth > BuildInput.MaxColumns)
        {
            throw new ScopeFormatException("build input", $"expected at most {BuildInput.MaxColumns} voxel columns, this input needs {frame.Width} x {frame.Depth}; use a larger cell size.");
        }

        SpanSettings so = pipe.SpanOptions;
        SpanPipeline spans = pipe.Regions.Spans;
        var results = new StageResult[ScopeInfo.StageNames.Length];

        var solid = new SolidHeightfield(frame);
        Voxelizer.Rasterize(solid, v, Input.Indices, so);
        results[0] = SolidStage(0, solid, "after rasterizing");
        spans.LowObstacleFilter?.Invoke(solid, so);
        spans.LedgeFilter?.Invoke(solid, so);
        spans.LowHeightFilter?.Invoke(solid, so);
        results[1] = SolidStage(1, solid, "after the low-obstacle, ledge and low-height filters");

        CompactHeightfield chf = spans.Compact?.Invoke(solid, so) ?? throw new InvalidOperationException("The Compact step is missing.");
        spans.Erode?.Invoke(chf, so);
        results[2] = CompactStage(2, chf, regions: false);

        pipe.Regions.Partition.Invoke(chf, pipe.Regions.RegionOptions);
        results[3] = CompactStage(3, chf, regions: true);

        ContourSet contours = pipe.Regions.Trace.Invoke(chf, pipe.Regions.ContourOptions);
        results[4] = ContourStage(4, contours, frame);

        int tileCells = Math.Max(frame.Width, frame.Depth);
        var map = new TileCoordinates(0, tileCells, cs, (float)(tileCells * (double)cs), ch, 0);
        PolyMesh poly = pipe.Polygonize(contours, st.MaxVertsPerPoly, 0, map);
        results[5] = PolyStage(5, poly, frame);

        DetailMeshSet detail = pipe.BuildDetail(poly, chf, map, pipe.Detail);
        results[6] = DetailStage(6, poly, detail, frame, map);
        return results;
    }

    private static StageResult SolidStage(int index, SolidHeightfield hf, string what)
    {
        HeightfieldFrame f = hf.Frame;
        var prims = new List<StagePrim>();
        int total = 0, walkable = 0;
        ReadOnlySpan<int> heads = hf.Heads;
        for (int col = 0; col < heads.Length; col++)
        {
            for (int s = heads[col]; s >= 0;)
            {
                ref SolidSpan sp = ref hf.Span(s);
                total++;
                if (sp.Area != 0)
                {
                    walkable++;
                }

                if (prims.Count < DrawCap)
                {
                    int x = col % f.Width, z = col / f.Width;
                    float y0 = (float)f.WorldY(sp.Min), y1 = (float)f.WorldY(sp.Max);
                    prims.Add(new StagePrim(PrimBox, new Vector3((float)f.CenterX(x), (y0 + y1) / 2, (float)f.CenterZ(z)), new Vector3(f.CellSize / 2, Math.Max((y1 - y0) / 2, f.CellHeight / 4), f.CellSize / 2), default, sp.Area != 0 ? new Color32(90, 190, 110) : new Color32(95, 95, 105)));
                }

                s = sp.Next;
            }
        }

        return new StageResult(ScopeInfo.StageNames[index], HashSolid(hf), $"{total} spans, {walkable} walkable, {what}; grid {f.Width} x {f.Depth} x {f.Height}", prims, total);
    }

    private static StageResult CompactStage(int index, CompactHeightfield chf, bool regions)
    {
        HeightfieldFrame f = chf.Frame;
        ReadOnlySpan<int> first = chf.CellFirst, count = chf.CellCount;
        var prims = new List<StagePrim>();
        int walkable = 0;
        Span<CompactSpan> sp = chf.Spans;
        for (int i = 0; i < sp.Length; i++)
        {
            if (sp[i].Area != 0)
            {
                walkable++;
            }
        }

        for (int c = 0; c < first.Length && prims.Count < DrawCap; c++)
        {
            int x = c % f.Width, z = c / f.Width;
            for (int k = 0; k < count[c] && prims.Count < DrawCap; k++)
            {
                CompactSpan s = sp[first[c] + k];
                Color32 col = s.Area == 0 ? new Color32(190, 70, 60) : regions ? RegionColor(s.Region) : new Color32(90, 190, 110);
                float y = (float)f.WorldY(s.Y);
                prims.Add(new StagePrim(PrimBox, new Vector3((float)f.CenterX(x), y, (float)f.CenterZ(z)), new Vector3(f.CellSize / 2, f.CellHeight / 3, f.CellSize / 2), default, col));
            }
        }

        string what = regions ? $"{chf.RegionCount} regions over {walkable} walkable spans" : $"{sp.Length} spans, {walkable} walkable after erosion (red = eroded away)";
        return new StageResult(ScopeInfo.StageNames[index], HashCompact(chf), what, prims, sp.Length);
    }

    private static StageResult ContourStage(int index, ContourSet set, HeightfieldFrame f)
    {
        var prims = new List<StagePrim>();
        int vertices = 0;
        foreach (Contour c in set.Contours)
        {
            Color32 col = RegionColor(c.Region);
            ContourVertex[] vs = c.Vertices;
            for (int i = 0; i < vs.Length; i++)
            {
                vertices++;
                ContourVertex b = vs[(i + 1) % vs.Length];
                if (prims.Count < DrawCap)
                {
                    prims.Add(new StagePrim(PrimLine, Cell(f, vs[i].X, vs[i].Y, vs[i].Z), Cell(f, b.X, b.Y, b.Z), default, col));
                }
            }
        }

        return new StageResult(ScopeInfo.StageNames[index], HashContours(set), $"{set.Contours.Count} contours, {vertices} vertices", prims, vertices);
    }

    private static StageResult PolyStage(int index, PolyMesh m, HeightfieldFrame f)
    {
        return new StageResult(ScopeInfo.StageNames[index], HashPoly(m), $"{m.PolygonCount} polygons over {m.VertexCount} vertices", PolyPrims(m, f), m.PolygonCount);
    }

    private static List<StagePrim> PolyPrims(PolyMesh m, HeightfieldFrame f)
    {
        var prims = new List<StagePrim>();
        ReadOnlySpan<int> verts = m.Vertices, counts = m.PolygonVertexCounts, idx = m.PolygonIndices, regions = m.Regions;
        int at = 0;
        for (int p = 0; p < counts.Length && prims.Count < DrawCap; p++)
        {
            int n = counts[p];
            Color32 col = RegionColor(regions[p]);
            Color32 edge = new(235, 235, 235);
            Vector3 first = default;
            Vector3 prev = default;
            for (int k = 0; k < n; k++)
            {
                int vi = idx[at + k] * 3;
                Vector3 a = Cell(f, verts[vi], verts[vi + 1], verts[vi + 2]);
                if (k == 0)
                {
                    first = a;
                }
                else if (k >= 2)
                {
                    prims.Add(new StagePrim(PrimTriangle, first, prev, a, col));
                }

                int wi = idx[at + ((k + 1) % n)] * 3;
                prims.Add(new StagePrim(PrimLine, a, Cell(f, verts[wi], verts[wi + 1], verts[wi + 2]), default, edge));
                prev = a;
            }

            at += n;
        }

        return prims;
    }

    private static StageResult DetailStage(int index, PolyMesh m, DetailMeshSet d, HeightfieldFrame f, TileCoordinates map)
    {
        var prims = new List<StagePrim>();
        int[] verts = m.Vertices.ToArray(), counts = m.PolygonVertexCounts.ToArray(), idx = m.PolygonIndices.ToArray();
        int at = 0, triangles = 0;
        for (int p = 0; p < counts.Length; p++)
        {
            int n = counts[p];
            DetailMesh dm = d.Meshes[p];
            Vector3 Vertex(int k)
            {
                if (k < n)
                {
                    int vi = idx[at + k] * 3;
                    return new Vector3((float)f.OriginX + map.Local(verts[vi]), (float)f.OriginY + map.LocalY(verts[vi + 1]), (float)f.OriginZ + map.Local(verts[vi + 2]));
                }

                int e = ((int)dm.VertexBase + k - n) * 3;
                return new Vector3((float)f.OriginX + d.Vertices[e], (float)f.OriginY + d.Vertices[e + 1], (float)f.OriginZ + d.Vertices[e + 2]);
            }

            Color32 col = RegionColor(m.Regions[p]);
            for (int t = 0; t < dm.TriangleCount; t++)
            {
                int b = ((int)dm.TriangleBase + t) * 4;
                triangles++;
                if (prims.Count < DrawCap)
                {
                    prims.Add(new StagePrim(PrimTriangle, Vertex(d.Triangles[b]), Vertex(d.Triangles[b + 1]), Vertex(d.Triangles[b + 2]), col));
                }
            }

            at += n;
        }

        return new StageResult(ScopeInfo.StageNames[index], HashDetail(d), $"{triangles} detail triangles, {d.Vertices.Length / 3} extra vertices", prims, triangles);
    }

    // The stage hashes: FNV-1a over the integer contents of what the library's stage returned, in the order the library stores it.
    internal static ulong HashSolid(SolidHeightfield hf)
    {
        var h = new Fnv();
        h.Add(hf.Frame.Width);
        h.Add(hf.Frame.Depth);
        h.Add(hf.Frame.Height);
        ReadOnlySpan<int> heads = hf.Heads;
        for (int col = 0; col < heads.Length; col++)
        {
            for (int s = heads[col]; s >= 0;)
            {
                ref SolidSpan sp = ref hf.Span(s);
                h.Add(col);
                h.Add(sp.Min);
                h.Add(sp.Max);
                h.Add(sp.Area);
                s = sp.Next;
            }
        }

        return h.Value;
    }

    internal static ulong HashCompact(CompactHeightfield chf)
    {
        var h = new Fnv();
        h.Add(chf.SpanCount);
        h.Add(chf.RegionCount);
        ReadOnlySpan<int> first = chf.CellFirst, count = chf.CellCount;
        for (int c = 0; c < first.Length; c++)
        {
            h.Add(first[c]);
            h.Add(count[c]);
        }

        foreach (CompactSpan s in chf.Spans)
        {
            h.Add(s.Y);
            h.Add(s.Height);
            h.Add(s.Area);
            h.Add(s.Connections);
            h.Add(s.Region);
        }

        return h.Value;
    }

    internal static ulong HashContours(ContourSet set)
    {
        var h = new Fnv();
        h.Add(set.RegionCount);
        foreach (Contour c in set.Contours)
        {
            h.Add(c.Region);
            h.Add(c.Area);
            foreach (ContourVertex v in c.Vertices)
            {
                h.Add(v.X);
                h.Add(v.Y);
                h.Add(v.Z);
                h.Add(v.Neighbor);
            }
        }

        return h.Value;
    }

    internal static ulong HashPoly(PolyMesh m)
    {
        var h = new Fnv();
        h.Add(m.MaxVertsPerPoly);
        foreach (int x in m.Vertices)
        {
            h.Add(x);
        }

        foreach (int x in m.PolygonVertexCounts)
        {
            h.Add(x);
        }

        foreach (int x in m.PolygonIndices)
        {
            h.Add(x);
        }

        foreach (byte x in m.Areas)
        {
            h.Add(x);
        }

        foreach (int x in m.Regions)
        {
            h.Add(x);
        }

        return h.Value;
    }

    internal static ulong HashDetail(DetailMeshSet d)
    {
        var h = new Fnv();
        foreach (DetailMesh dm in d.Meshes)
        {
            h.Add((int)dm.VertexBase);
            h.Add((int)dm.TriangleBase);
            h.Add(dm.VertexCount);
            h.Add(dm.TriangleCount);
        }

        foreach (float x in d.Vertices)
        {
            h.Add(BitConverter.SingleToInt32Bits(x));
        }

        foreach (byte x in d.Triangles)
        {
            h.Add(x);
        }

        return h.Value;
    }

    private static Vector3 Cell(HeightfieldFrame f, int x, int y, int z) =>
        new((float)(f.OriginX + (x * (double)f.CellSize)), (float)f.WorldY(y), (float)(f.OriginZ + (z * (double)f.CellSize)));

    internal static Color32 RegionColor(int region)
    {
        if (region <= 0)
        {
            return new Color32(120, 120, 120);
        }

        uint x = (uint)region * 2654435761u;
        return new Color32((byte)(70 + ((x >> 8) & 0x7F)), (byte)(70 + ((x >> 16) & 0x7F)), (byte)(70 + ((x >> 24) & 0x7F)));
    }

    /// <summary>FNV-1a (64-bit) over 32-bit values, little-endian.</summary>
    internal struct Fnv
    {
        private ulong _h;
        private bool _started;

        public readonly ulong Value => _started ? _h : 0xCBF29CE484222325UL;

        public void Add(int value)
        {
            if (!_started)
            {
                _h = 0xCBF29CE484222325UL;
                _started = true;
            }

            for (int k = 0; k < 4; k++)
            {
                _h = (_h ^ (byte)(value >> (8 * k))) * 0x100000001B3UL;
            }
        }
    }

    public static string Hex(ulong v) => v.ToString("x16", CultureInfo.InvariantCulture);
}
