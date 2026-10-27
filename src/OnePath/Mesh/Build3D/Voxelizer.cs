using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>Counts from one rasterization call.</summary>
/// <param name="Triangles">Triangles in the input.</param>
/// <param name="Degenerate">Triangles skipped because they have no area (collinear or coincident vertices).</param>
/// <param name="Touching">Triangles that put at least one span into the grid (the others lie outside it or only graze its boundary).</param>
public readonly record struct VoxelizeStats(int Triangles, int Degenerate, int Touching);

/// <summary>
/// Turns triangle soup, height grids and voxel chunks into a <see cref="SolidHeightfield"/>. The rasterization
/// rule (which cells a triangle touches, the span rows it covers, the tie cases) is written down in
/// docs/build3d-spans.md; the short version: a cell is touched when the triangle covers part of its footprint with
/// positive area (vertical triangles: positive length), a span covers rows floor(lowest y) up to ceil(highest y)
/// of the clipped piece, and exact ties resolve toward the side the surface faces away from.
/// </summary>
public static class Voxelizer
{
    /// <summary>
    /// Gives every triangle its area id: <see cref="SpanSettings.WalkableArea"/> when the unit normal's y exceeds
    /// <paramref name="walkableSlopeCosine"/>, else 0. Triangles are wound counter-clockwise seen from the side they face
    /// (a floor seen from above); degenerate triangles get 0.
    /// </summary>
    /// <param name="frame">The grid; vertices are made tile-local float32 relative to its origin first.</param>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle.</param>
    /// <param name="walkableSlopeCosine">See <see cref="SpanSettings.WalkableSlopeCosine"/>.</param>
    /// <param name="areas">Receives one area id per triangle; its length must equal the triangle count.</param>
    /// <exception cref="ArgumentException">Lengths are not multiples of three or do not match, or a referenced coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range or a referenced coordinate is beyond ±2^21.</exception>
    public static void ClassifyTriangles(HeightfieldFrame frame, ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, float walkableSlopeCosine, Span<byte> areas)
    {
        ArgumentNullException.ThrowIfNull(frame);
        Validate(vertices, indices);
        int tris = indices.Length / 3;
        if (areas.Length != tris)
        {
            throw new ArgumentException($"areas has {areas.Length} entries for {tris} triangles.", nameof(areas));
        }
        for (int t = 0; t < tris; t++)
        {
            Load(frame, vertices, indices, t, out float x0, out float y0, out float z0, out float x1, out float y1, out float z1, out float x2, out float y2, out float z2);
            Normal(x0, y0, z0, x1, y1, z1, x2, y2, z2, out _, out float ny, out _, out float len);
            areas[t] = len > 0f && !float.IsInfinity(len) && ny > walkableSlopeCosine * len ? SpanSettings.WalkableArea : (byte)0;
        }
    }

    /// <summary>Classifies by slope with <paramref name="settings"/> and rasterizes in one call.</summary>
    /// <inheritdoc cref="Rasterize(SolidHeightfield, ReadOnlySpan{double}, ReadOnlySpan{int}, ReadOnlySpan{byte}, int)"/>
    public static VoxelizeStats Rasterize(SolidHeightfield target, ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, SpanSettings settings)
    {
        ArgumentNullException.ThrowIfNull(target);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        Validate(vertices, indices);
        byte[] areas = new byte[indices.Length / 3];
        ClassifyTriangles(target.Frame, vertices, indices, settings.WalkableSlopeCosine, areas);
        return Rasterize(target, vertices, indices, areas, settings.EffectiveMergeThreshold);
    }

    /// <summary>
    /// Adds the triangles to <paramref name="target"/> in index order. The result depends on the triangle order only
    /// through the area rule of span merging (see <see cref="SolidHeightfield.AddSpan"/>), never on thread count or machine.
    /// Degenerate triangles are skipped and counted; triangles outside the grid contribute nothing.
    /// </summary>
    /// <param name="target">Heightfield to add to; may already hold spans.</param>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle.</param>
    /// <param name="areas">Area id of each triangle (0 = not walkable).</param>
    /// <param name="mergeThreshold">Rows within which the larger area wins when spans merge.</param>
    /// <exception cref="ArgumentException">Lengths are not multiples of three or do not match, or a referenced coordinate is NaN or infinite. Nothing is added then.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range, a referenced coordinate is beyond ±2^21, or the threshold is negative. Nothing is added then.</exception>
    public static VoxelizeStats Rasterize(SolidHeightfield target, ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, ReadOnlySpan<byte> areas, int mergeThreshold)
    {
        ArgumentNullException.ThrowIfNull(target);
        Validate(vertices, indices);
        int tris = indices.Length / 3;
        if (areas.Length != tris)
        {
            throw new ArgumentException($"areas has {areas.Length} entries for {tris} triangles.", nameof(areas));
        }
        if (mergeThreshold < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(mergeThreshold), mergeThreshold, "The merge threshold must not be negative.");
        }
        var r = new TriangleRasterizer(target, mergeThreshold);
        int degenerate = 0;
        int touching = 0;
        for (int t = 0; t < tris; t++)
        {
            Load(target.Frame, vertices, indices, t, out float x0, out float y0, out float z0, out float x1, out float y1, out float z1, out float x2, out float y2, out float z2);
            Normal(x0, y0, z0, x1, y1, z1, x2, y2, z2, out float nx, out float ny, out float nz, out float len);
            if (!(len > 0f) || float.IsInfinity(len))
            {
                degenerate++;
                continue;
            }
            if (r.Triangle(x0, y0, z0, x1, y1, z1, x2, y2, z2, nx, ny, nz, areas[t]))
            {
                touching++;
            }
        }
        return new VoxelizeStats(tris, degenerate, touching);
    }

    /// <summary>
    /// Rasterizes a regular height grid (one height per sample, x varying fastest) as two triangles per square,
    /// each classified by slope. Sample (i, j) sits at (<paramref name="originX"/> + i * spacing, heights[i + j * countX], <paramref name="originZ"/> + j * spacing).
    /// </summary>
    /// <exception cref="ArgumentException">Fewer than 2 samples per side, a length mismatch, or a NaN or infinite height.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21 or the spacing is not positive.</exception>
    public static VoxelizeStats RasterizeHeightGrid(SolidHeightfield target, ReadOnlySpan<double> heights, int countX, int countZ, double originX, double originZ, double spacing, SpanSettings settings)
    {
        ArgumentNullException.ThrowIfNull(target);
        if (countX < 2 || countZ < 2 || (long)countX * countZ > int.MaxValue / 3)
        {
            throw new ArgumentException($"A height grid needs 2..{int.MaxValue / 3} samples in total and at least 2 per side, got {countX} x {countZ}.");
        }
        if (heights.Length != countX * countZ)
        {
            throw new ArgumentException($"heights has {heights.Length} entries for a {countX} x {countZ} grid.", nameof(heights));
        }
        if (!(spacing > 0.0) || double.IsInfinity(spacing))
        {
            throw new ArgumentOutOfRangeException(nameof(spacing), spacing, "The spacing must be positive and finite.");
        }
        double[] verts = new double[heights.Length * 3];
        for (int j = 0; j < countZ; j++)
        {
            for (int i = 0; i < countX; i++)
            {
                int k = i + (j * countX);
                verts[3 * k] = originX + (i * spacing);
                verts[(3 * k) + 1] = heights[k];
                verts[(3 * k) + 2] = originZ + (j * spacing);
            }
        }
        int[] idx = new int[(countX - 1) * (countZ - 1) * 6];
        int n = 0;
        for (int j = 0; j < countZ - 1; j++)
        {
            for (int i = 0; i < countX - 1; i++)
            {
                int a = i + (j * countX);
                int b = a + 1;
                int c = a + countX;
                int d = c + 1;
                idx[n++] = a;
                idx[n++] = c;
                idx[n++] = d;
                idx[n++] = a;
                idx[n++] = d;
                idx[n++] = b;
            }
        }
        return Rasterize(target, verts, idx, settings);
    }

    /// <summary>
    /// Adds a voxel chunk straight into the span representation, no triangles involved. Each run becomes the span
    /// [start, start + length) whose top surface gets the area of the run's material; touching runs merge exactly
    /// (merge threshold 0), so a stack of different materials keeps the top material's area.
    /// </summary>
    /// <param name="target">Heightfield to add to.</param>
    /// <param name="chunk">The chunk.</param>
    /// <param name="offsetX">Column of the target grid where the chunk's column 0 sits (cells outside the grid are dropped).</param>
    /// <param name="offsetY">Row of the target grid where the chunk's row 0 sits.</param>
    /// <param name="offsetZ">Row (z) of the target grid where the chunk's depth 0 sits.</param>
    /// <param name="materialAreas">Area id per material id (256 entries), or empty for "every material is walkable (63)". Material 0 never occurs in a run.</param>
    /// <returns>The number of spans added.</returns>
    /// <exception cref="ArgumentException">materialAreas has a length other than 0 or 256.</exception>
    public static int AddChunk(SolidHeightfield target, VoxelChunk chunk, int offsetX, int offsetY, int offsetZ, ReadOnlySpan<byte> materialAreas)
    {
        ArgumentNullException.ThrowIfNull(target);
        ArgumentNullException.ThrowIfNull(chunk);
        if (materialAreas.Length != 0 && materialAreas.Length != 256)
        {
            throw new ArgumentException("materialAreas must be empty or have 256 entries.", nameof(materialAreas));
        }
        HeightfieldFrame f = target.Frame;
        int added = 0;
        ReadOnlySpan<int> starts = chunk.ColumnStarts;
        ReadOnlySpan<VoxelRun> runs = chunk.Runs;
        for (int cz = 0; cz < chunk.Depth; cz++)
        {
            long tz = (long)cz + offsetZ;
            if (tz < 0 || tz >= f.Depth)
            {
                continue;
            }
            for (int cx = 0; cx < chunk.Width; cx++)
            {
                long tx = (long)cx + offsetX;
                if (tx < 0 || tx >= f.Width)
                {
                    continue;
                }
                int col = cx + (cz * chunk.Width);
                int column = f.Column((int)tx, (int)tz);
                for (int r = starts[col]; r < starts[col + 1]; r++)
                {
                    VoxelRun run = runs[r];
                    long lo = Math.Max(0L, (long)run.Start + offsetY);
                    long hi = Math.Min(f.Height, (long)run.Start + run.Length + offsetY);
                    if (lo >= hi)
                    {
                        continue;
                    }
                    byte area = materialAreas.Length == 0 ? SpanSettings.WalkableArea : materialAreas[run.Material];
                    target.AddSpan(column, (int)lo, (int)hi, area, 0);
                    added++;
                }
            }
        }
        return added;
    }

    private static void Validate(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices)
    {
        if (vertices.Length % 3 != 0)
        {
            throw new ArgumentException($"The vertex array length {vertices.Length} is not a multiple of 3.", nameof(vertices));
        }
        if (indices.Length % 3 != 0)
        {
            throw new ArgumentException($"The index array length {indices.Length} is not a multiple of 3.", nameof(indices));
        }
        int nv = vertices.Length / 3;
        for (int i = 0; i < indices.Length; i++)
        {
            int v = indices[i];
            if ((uint)v >= (uint)nv)
            {
                throw new ArgumentOutOfRangeException(nameof(indices), v, $"Index {i} (triangle {i / 3}) refers to vertex {v}, but there are only {nv} vertices.");
            }
            for (int c = 0; c < 3; c++)
            {
                double value = vertices[(3 * v) + c];
                if (double.IsNaN(value) || double.IsInfinity(value))
                {
                    throw new ArgumentException($"Vertex {v} (used by triangle {i / 3}) has a non-finite coordinate: {value}.", nameof(vertices));
                }
                if (Math.Abs(value) > HeightfieldFrame.MaxCoordinate)
                {
                    throw new ArgumentOutOfRangeException(nameof(vertices), value, $"Vertex {v} (used by triangle {i / 3}) has coordinate {value}, beyond the supported ±{HeightfieldFrame.MaxCoordinate} (2^21).");
                }
            }
        }
    }

    private static void Load(HeightfieldFrame f, ReadOnlySpan<double> v, ReadOnlySpan<int> idx, int t,
        out float x0, out float y0, out float z0, out float x1, out float y1, out float z1, out float x2, out float y2, out float z2)
    {
        int a = idx[3 * t] * 3;
        int b = idx[(3 * t) + 1] * 3;
        int c = idx[(3 * t) + 2] * 3;
        x0 = (float)(v[a] - f.OriginX);
        y0 = (float)(v[a + 1] - f.OriginY);
        z0 = (float)(v[a + 2] - f.OriginZ);
        x1 = (float)(v[b] - f.OriginX);
        y1 = (float)(v[b + 1] - f.OriginY);
        z1 = (float)(v[b + 2] - f.OriginZ);
        x2 = (float)(v[c] - f.OriginX);
        y2 = (float)(v[c + 1] - f.OriginY);
        z2 = (float)(v[c + 2] - f.OriginZ);
    }

    // (v1 - v0) x (v2 - v0): points to the side the triangle faces when it is wound counter-clockwise seen from there.
    private static void Normal(float x0, float y0, float z0, float x1, float y1, float z1, float x2, float y2, float z2,
        out float nx, out float ny, out float nz, out float len)
    {
        float e1x = x1 - x0;
        float e1y = y1 - y0;
        float e1z = z1 - z0;
        float e2x = x2 - x0;
        float e2y = y2 - y0;
        float e2z = z2 - z0;
        nx = (e1y * e2z) - (e1z * e2y);
        ny = (e1z * e2x) - (e1x * e2z);
        nz = (e1x * e2y) - (e1y * e2x);
        len = SqrtF((nx * nx) + (ny * ny) + (nz * nz));
    }

    private static float SqrtF(float v) => (float)Math.Sqrt(v);
}
