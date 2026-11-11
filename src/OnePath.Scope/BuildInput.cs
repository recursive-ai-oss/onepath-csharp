using System;
using System.Buffers;
using System.Buffers.Binary;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Scope;

/// <summary>
/// The file a host writes so the viewer can step a navigation-mesh build along its stages: the input triangles and the build settings
/// (<c>OPBI</c>, see docs/replay.md). The viewer runs the library's public build stages on the whole input as one tile, one stage at a time.
/// </summary>
public static class BuildInput
{
    /// <summary>The file magic, <c>OPBI</c>.</summary>
    public static ReadOnlySpan<byte> Magic => "OPBI"u8;

    /// <summary>Most vertices a build input may hold.</summary>
    public const int MaxVertices = 4_000_000;

    /// <summary>Most triangles a build input may hold.</summary>
    public const int MaxTriangles = 8_000_000;

    /// <summary>The most voxel columns (cells across by cells deep) the viewer builds; a bigger input is refused.</summary>
    public const long MaxColumns = 4_000_000;

    private const int Fixed = 4 + 4 + (10 * 8) + (3 * 4) + 4 + 4 + 4;

    /// <summary>Appends the file for a triangle soup and settings. Off-mesh links, tile size, origin and area/flag settings are not part of the file.</summary>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle, counter-clockwise seen from the side they face.</param>
    /// <param name="settings">The build settings; null for <see cref="NavSettings.Humanoid"/>.</param>
    /// <param name="into">Receives the bytes.</param>
    /// <exception cref="ArgumentException">A length is not a multiple of three, a count is over the limit, or an index is out of range.</exception>
    public static void Write(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, NavSettings? settings, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        settings ??= NavSettings.Humanoid;
        if (vertices.Length % 3 != 0 || indices.Length % 3 != 0 || vertices.Length / 3 > MaxVertices || indices.Length / 3 > MaxTriangles)
        {
            throw new ArgumentException($"Expected whole vertices and triangles, at most {MaxVertices} vertices and {MaxTriangles} triangles.");
        }

        foreach (int i in indices)
        {
            if ((uint)i >= (uint)(vertices.Length / 3))
            {
                throw new ArgumentException($"Index {i} names a vertex beyond the {vertices.Length / 3} given.", nameof(indices));
            }
        }

        int size = Fixed + (vertices.Length * 8) + (indices.Length * 4);
        Span<byte> d = into.GetSpan(size);
        Magic.CopyTo(d);
        d[4] = 1;
        d[5] = 0;
        d[6] = 0;
        d[7] = 0;
        int n = 8;
        foreach (double v in (ReadOnlySpan<double>)[settings.AgentRadius, settings.AgentHeight, settings.WalkableClimb, settings.AgentMaxSlopeDegrees, settings.CellSize, settings.CellHeight, settings.ContourMaxError, settings.MaxEdgeLength, settings.DetailSampleDistance, settings.DetailMaxError])
        {
            BinaryPrimitives.WriteInt64LittleEndian(d[n..], BitConverter.DoubleToInt64Bits(v));
            n += 8;
        }

        BinaryPrimitives.WriteInt32LittleEndian(d[n..], settings.MinRegionArea);
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 4)..], settings.MergeRegionArea);
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 8)..], settings.MaxVertsPerPoly);
        n += 12;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], (int)settings.Partition);
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 4)..], vertices.Length / 3);
        n += 8;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], indices.Length / 3);
        n += 4;
        foreach (double v in vertices)
        {
            BinaryPrimitives.WriteInt64LittleEndian(d[n..], BitConverter.DoubleToInt64Bits(v));
            n += 8;
        }

        foreach (int i in indices)
        {
            BinaryPrimitives.WriteInt32LittleEndian(d[n..], i);
            n += 4;
        }

        into.Advance(n);
    }

    internal sealed record Parsed(NavSettings Settings, double[] Vertices, int[] Indices);

    /// <summary>Reads and checks a build input. Every count is checked against the bytes present before anything is allocated.</summary>
    internal static Parsed Read(ReadOnlySpan<byte> bytes, string name)
    {
        if (bytes.Length < Fixed)
        {
            throw new OneScope.Sdk.ScopeFormatException(name, $"expected at least {Fixed} bytes of build input, got {bytes.Length}.");
        }

        if (!bytes.StartsWith(Magic) || bytes[4] != 1 || bytes[5] != 0 || bytes[6] != 0 || bytes[7] != 0)
        {
            throw new OneScope.Sdk.ScopeFormatException(name, "expected build input version 1 (bytes 'OPBI', 1, 0, 0, 0 at the start).");
        }

        int n = 8;
        double[] f = new double[10];
        for (int i = 0; i < f.Length; i++)
        {
            f[i] = BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(bytes[n..]));
            n += 8;
        }

        int minRegion = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
        int mergeRegion = BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 4)..]);
        int maxVerts = BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 8)..]);
        int partition = BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 12)..]);
        int vertexCount = BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 16)..]);
        int triangleCount = BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 20)..]);
        n += 24;
        if (vertexCount < 0 || vertexCount > MaxVertices || triangleCount < 0 || triangleCount > MaxTriangles)
        {
            throw new OneScope.Sdk.ScopeFormatException(name, $"expected 0 to {MaxVertices} vertices and 0 to {MaxTriangles} triangles, got {vertexCount} and {triangleCount} at byte {n - 8}.");
        }

        long need = (checked((long)vertexCount * 24)) + (checked((long)triangleCount * 12));
        if (bytes.Length - n != need)
        {
            throw new OneScope.Sdk.ScopeFormatException(name, $"expected {need} bytes of geometry after byte {n}, found {bytes.Length - n}.");
        }

        NavSettings settings;
        try
        {
            settings = new NavSettings
            {
                AgentRadius = f[0],
                AgentHeight = f[1],
                WalkableClimb = f[2],
                AgentMaxSlopeDegrees = f[3],
                CellSize = f[4],
                CellHeight = f[5],
                ContourMaxError = f[6],
                MaxEdgeLength = f[7],
                DetailSampleDistance = f[8],
                DetailMaxError = f[9],
                MinRegionArea = minRegion,
                MergeRegionArea = mergeRegion,
                MaxVertsPerPoly = maxVerts,
                Partition = (PartitionType)partition,
            };
            _ = new Mesh3DPipeline(settings);
        }
        catch (ArgumentException e)
        {
            throw new OneScope.Sdk.ScopeFormatException(name, "expected valid build settings: " + e.Message);
        }

        double[] vertices = new double[vertexCount * 3];
        for (int i = 0; i < vertices.Length; i++)
        {
            double v = BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(bytes[n..]));
            n += 8;
            if (!(Math.Abs(v) < HeightfieldFrame.MaxCoordinate))
            {
                throw new OneScope.Sdk.ScopeFormatException(name, $"expected vertex coordinates that are finite and within ±{HeightfieldFrame.MaxCoordinate:R}, got {v} in value {i} of the vertices.");
            }

            vertices[i] = v;
        }

        int[] indices = new int[triangleCount * 3];
        for (int i = 0; i < indices.Length; i++)
        {
            int v = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
            n += 4;
            if ((uint)v >= (uint)vertexCount)
            {
                throw new OneScope.Sdk.ScopeFormatException(name, $"expected triangle indices below {vertexCount}, got {v} at index {i}.");
            }

            indices[i] = v;
        }

        return new Parsed(settings, vertices, indices);
    }
}
