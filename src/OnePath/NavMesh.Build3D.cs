using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath;

/// <summary>A triangle of level geometry, wound counter-clockwise seen from the side it faces (a floor seen from above).</summary>
/// <param name="A">First corner (world, double).</param>
/// <param name="B">Second corner.</param>
/// <param name="C">Third corner.</param>
public readonly record struct Triangle3D(Vector3D A, Vector3D B, Vector3D C);

public sealed partial class NavMesh
{
    /// <summary>
    /// Builds a mesh from level geometry: the triangles are voxelized, filtered by the agent's slope, step and clearance, eroded by its radius, split into regions,
    /// outlined, cut into convex polygons with a height detail mesh, and tiled; see <see cref="Mesh3DPipeline"/> for the stages and
    /// <c>docs/build3d-polys.md</c> for the rules. Y is up. Multiple floors are fine; <see cref="NavSettings.Partition"/> = layers splits them into tile layers.
    /// </summary>
    /// <param name="triangles">The level geometry.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentException">A coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21 (the message names the vertex and the value), a setting is out of range, or the geometry needs more tiles than the format holds.</exception>
    public static NavMesh Build(IEnumerable<Triangle3D> triangles, NavSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(triangles);
        var v = new List<double>();
        var ix = new List<int>();
        foreach (Triangle3D t in triangles)
        {
            int b = v.Count / 3;
            v.Add(t.A.X);
            v.Add(t.A.Y);
            v.Add(t.A.Z);
            v.Add(t.B.X);
            v.Add(t.B.Y);
            v.Add(t.B.Z);
            v.Add(t.C.X);
            v.Add(t.C.Y);
            v.Add(t.C.Z);
            ix.Add(b);
            ix.Add(b + 1);
            ix.Add(b + 2);
        }
        return Build(System.Runtime.InteropServices.CollectionsMarshal.AsSpan(v), System.Runtime.InteropServices.CollectionsMarshal.AsSpan(ix), settings);
    }

    /// <summary>
    /// Builds a mesh from an indexed triangle soup of single-precision coordinates, as engines and <see cref="System.Numerics.Vector3"/> arrays hold them.
    /// Each float widens to double exactly, so the result is byte-equal to the double overload given the same values.
    /// To convert a single <see cref="System.Numerics.Vector3"/> use OneMath's <c>Vector3D</c> conversions to and from <c>System.Numerics</c>.
    /// </summary>
    /// <param name="vertices">x, y, z world coordinates, three floats per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentException">A length is not a multiple of three, or a referenced coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range, a coordinate is beyond ±2^21, or a setting is out of range.</exception>
    public static NavMesh Build(ReadOnlySpan<float> vertices, ReadOnlySpan<int> indices, NavSettings? settings = null)
    {
        var wide = new double[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            wide[i] = vertices[i];
        }
        return Build(wide, indices, settings);
    }

    /// <summary>Builds a mesh from an indexed triangle soup; see <see cref="Build(IEnumerable{Triangle3D}, NavSettings?)"/>.</summary>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentException">A length is not a multiple of three, or a referenced coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range, a coordinate is beyond ±2^21, or a setting is out of range.</exception>
    public static NavMesh Build(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, NavSettings? settings = null)
    {
        settings ??= NavSettings.Humanoid;
        return new NavMesh(new Mesh3DPipeline(settings).BuildSoup(vertices, indices), settings);
    }

    /// <summary>
    /// Builds a mesh from a solid heightfield (columns of spans). Cell (i, k) sits at <see cref="NavSettings.OriginX"/> + i cells, <see cref="NavSettings.OriginZ"/> + k cells;
    /// the heightfield's own cell sizes are ignored, see <see cref="Mesh3DPipeline.BuildHeightfield"/>.
    /// </summary>
    /// <param name="heightfield">The spans.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static NavMesh Build(SolidHeightfield heightfield, NavSettings? settings = null)
    {
        settings ??= NavSettings.Humanoid;
        return new NavMesh(new Mesh3DPipeline(settings).BuildHeightfield(heightfield), settings);
    }

    /// <summary>
    /// Builds a mesh straight from voxels, skipping triangles: the chunk enters at the span stage. Voxel size and placement are described at <see cref="Mesh3DPipeline.BuildChunk"/>.
    /// </summary>
    /// <param name="chunk">The voxels.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <param name="materialAreas">Area id per material id (256 entries), or empty for "every material is walkable".</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static NavMesh Build(VoxelChunk chunk, NavSettings? settings = null, ReadOnlySpan<byte> materialAreas = default)
    {
        settings ??= NavSettings.Humanoid;
        return new NavMesh(new Mesh3DPipeline(settings).BuildChunk(chunk, materialAreas), settings);
    }
}
