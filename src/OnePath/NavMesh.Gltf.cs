using System;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath;

public sealed partial class NavMesh
{
    /// <summary>Builds a mesh from a glTF 2.0 file (<c>.glb</c> or <c>.gltf</c>): <c>NavMesh.BuildGltf(File.ReadAllBytes(path))</c>. See <see cref="GltfReader"/> for what is read.</summary>
    /// <param name="file">The file's bytes.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <param name="options">Reader options (external buffers, limits); null means the defaults.</param>
    /// <exception cref="GltfException">The file is malformed, unsupported or too large.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21 or a setting is out of range.</exception>
    public static NavMesh BuildGltf(ReadOnlySpan<byte> file, NavSettings? settings = null, GltfOptions? options = null) =>
        Build(GltfReader.Read(file, options), settings);

    /// <summary>Builds a mesh from level geometry already read from glTF.</summary>
    /// <param name="scene">The geometry.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static NavMesh Build(GltfScene scene, NavSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(scene);
        return Build(scene.Vertices, scene.Indices, settings);
    }
}
