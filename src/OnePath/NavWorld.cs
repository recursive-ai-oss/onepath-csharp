using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath;

/// <summary>
/// One world for several agent sizes: a navigation mesh per agent class, all built from the same input, each with its own
/// <see cref="NavSettings"/> (radius, height, climb, slope, cell size). Class <c>i</c>'s mesh is exactly what
/// <c>NavMesh.Build(geometry, classes[i])</c> returns on its own; the world adds the choice of class for an agent and a crowd front
/// that sends every agent to the crowd of its class.
/// </summary>
public sealed class NavWorld
{
    private readonly NavMesh[] _meshes;

    private NavWorld(NavMesh[] meshes) => _meshes = meshes;

    /// <summary>The classes' meshes, in the order the settings were given.</summary>
    public IReadOnlyList<NavMesh> Meshes => _meshes;

    /// <summary>The number of agent classes.</summary>
    public int ClassCount => _meshes.Length;

    /// <summary>The mesh of agent class <paramref name="classIndex"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The index is not a class.</exception>
    public NavMesh this[int classIndex] =>
        (uint)classIndex < (uint)_meshes.Length ? _meshes[classIndex] : throw new ArgumentOutOfRangeException(nameof(classIndex), classIndex, $"The world has {_meshes.Length} agent classes.");

    /// <summary>Builds one mesh per class from level geometry (Y up); see <see cref="NavMesh.Build(IEnumerable{Triangle3D}, NavSettings?)"/>.</summary>
    /// <param name="triangles">The level geometry, read once.</param>
    /// <param name="classes">One settings object per agent class, at least one.</param>
    /// <exception cref="ArgumentNullException">An argument or a class is null.</exception>
    /// <exception cref="ArgumentException">No class is given, or a coordinate is NaN or infinite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21 or a setting is out of range.</exception>
    public static NavWorld Build(IEnumerable<Triangle3D> triangles, params NavSettings[] classes)
    {
        ArgumentNullException.ThrowIfNull(triangles);
        CheckClasses(classes);
        var v = new List<double>();
        var ix = new List<int>();
        foreach (Triangle3D t in triangles)
        {
            int b = v.Count / 3;
            v.AddRange([t.A.X, t.A.Y, t.A.Z, t.B.X, t.B.Y, t.B.Z, t.C.X, t.C.Y, t.C.Z]);
            ix.AddRange([b, b + 1, b + 2]);
        }
        return Build(v.ToArray(), ix.ToArray(), classes);
    }

    /// <summary>Builds one mesh per class from an indexed triangle soup; see <see cref="NavMesh.Build(ReadOnlySpan{double}, ReadOnlySpan{int}, NavSettings?)"/>.</summary>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle.</param>
    /// <param name="classes">One settings object per agent class, at least one.</param>
    /// <exception cref="ArgumentNullException">A class is null.</exception>
    /// <exception cref="ArgumentException">No class is given, or the input is malformed.</exception>
    /// <exception cref="ArgumentOutOfRangeException">As for <see cref="NavMesh.Build(ReadOnlySpan{double}, ReadOnlySpan{int}, NavSettings?)"/>.</exception>
    public static NavWorld Build(ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices, params NavSettings[] classes)
    {
        CheckClasses(classes);
        var meshes = new NavMesh[classes.Length];
        for (int i = 0; i < classes.Length; i++)
        {
            meshes[i] = NavMesh.Build(vertices, indices, classes[i]);
        }
        return new NavWorld(meshes);
    }

    /// <summary>Builds one mesh per class from a glTF file; see <see cref="NavMesh.BuildGltf"/>.</summary>
    /// <param name="file">The file's bytes (<c>.glb</c> or <c>.gltf</c>).</param>
    /// <param name="options">Reader options; null means the defaults.</param>
    /// <param name="classes">One settings object per agent class, at least one.</param>
    /// <exception cref="Mesh.Build3D.GltfException">The file is malformed, unsupported or too large.</exception>
    public static NavWorld BuildGltf(ReadOnlySpan<byte> file, Mesh.Build3D.GltfOptions? options, params NavSettings[] classes)
    {
        CheckClasses(classes);
        Mesh.Build3D.GltfScene scene = Mesh.Build3D.GltfReader.Read(file, options);
        return Build(scene.Vertices, scene.Indices, classes);
    }

    /// <summary>Builds one mesh per class from walkable outlines; see <see cref="NavMesh.Build2D(IEnumerable{Outline}, NavSettings?)"/>.</summary>
    /// <param name="walkable">The outlines, read once.</param>
    /// <param name="classes">One settings object per agent class, at least one.</param>
    /// <exception cref="ArgumentNullException">An argument or a class is null.</exception>
    /// <exception cref="ArgumentException">No class is given.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate or a setting is out of range.</exception>
    public static NavWorld Build2D(IEnumerable<Outline> walkable, params NavSettings[] classes)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        CheckClasses(classes);
        var list = new List<Outline>(walkable);
        var meshes = new NavMesh[classes.Length];
        for (int i = 0; i < classes.Length; i++)
        {
            meshes[i] = NavMesh.Build2D(list, classes[i]);
        }
        return new NavWorld(meshes);
    }

    private static void CheckClasses(NavSettings[] classes)
    {
        ArgumentNullException.ThrowIfNull(classes);
        if (classes.Length == 0)
        {
            throw new ArgumentException("A world needs at least one agent class.", nameof(classes));
        }
        for (int i = 0; i < classes.Length; i++)
        {
            if (classes[i] is null)
            {
                throw new ArgumentNullException(nameof(classes), $"classes[{i}] is null.");
            }
        }
    }

    /// <summary>
    /// The class an agent of this radius uses: the one with the smallest <see cref="NavSettings.AgentRadius"/> that is at least
    /// <paramref name="radius"/> (the lowest index among equals), or the class with the largest radius when the agent is bigger than all.
    /// </summary>
    /// <param name="radius">The agent's radius, not negative.</param>
    /// <exception cref="ArgumentOutOfRangeException">The radius is negative or NaN.</exception>
    public int ClassFor(double radius)
    {
        if (!(radius >= 0))
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must not be negative.");
        }
        int fit = -1, biggest = 0;
        for (int i = 0; i < _meshes.Length; i++)
        {
            double r = (float)_meshes[i].Settings.AgentRadius;
            if (r > (float)_meshes[biggest].Settings.AgentRadius)
            {
                biggest = i;
            }
            if (r >= radius && (fit < 0 || r < (float)_meshes[fit].Settings.AgentRadius))
            {
                fit = i;
            }
        }
        return fit >= 0 ? fit : biggest;
    }

    /// <summary>The mesh of <see cref="ClassFor"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The radius is negative or NaN.</exception>
    public NavMesh MeshFor(double radius) => _meshes[ClassFor(radius)];

    /// <summary>Finds a path for an agent of the given radius, on the mesh of its class.</summary>
    /// <param name="start">Start (world).</param>
    /// <param name="end">End (world).</param>
    /// <param name="radius">The agent's radius.</param>
    /// <param name="options">Path options; null means the defaults.</param>
    /// <param name="cancellationToken">Checked every 256 expansions; see <see cref="NavMesh.FindPath(Vector3D, Vector3D, PathOptions?, System.Threading.CancellationToken)"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or beyond ±2^21, or the radius is negative.</exception>
    /// <exception cref="OperationCanceledException">Cancellation was requested.</exception>
    public PathResult FindPath(Vector3D start, Vector3D end, double radius, PathOptions? options = null, System.Threading.CancellationToken cancellationToken = default) =>
        MeshFor(radius).FindPath(start, end, options, cancellationToken);

    /// <summary>Creates a crowd front with one crowd per class; see <see cref="WorldCrowd"/>.</summary>
    /// <param name="options">Crowd settings for every class; null means <see cref="CrowdOptions.Default"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range.</exception>
    public WorldCrowd CreateCrowd(CrowdOptions? options = null)
    {
        var crowds = new Crowd[_meshes.Length];
        for (int i = 0; i < crowds.Length; i++)
        {
            crowds[i] = _meshes[i].CreateCrowd(options);
        }
        return new WorldCrowd(this, crowds);
    }
}

/// <summary>
/// The crowds of a <see cref="NavWorld"/>, one per agent class. <see cref="Add"/> puts an agent in the crowd of its class and
/// <see cref="Step"/> steps the crowds in class order. Agents of different classes walk different meshes and do not avoid one another.
/// </summary>
public sealed class WorldCrowd
{
    private readonly NavWorld _world;
    private readonly Crowd[] _crowds;

    internal WorldCrowd(NavWorld world, Crowd[] crowds)
    {
        _world = world;
        _crowds = crowds;
    }

    /// <summary>The crowd of each class.</summary>
    public IReadOnlyList<Crowd> Crowds => _crowds;

    /// <summary>Agents over all classes.</summary>
    public int Count
    {
        get
        {
            int n = 0;
            foreach (Crowd c in _crowds)
            {
                n += c.Count;
            }
            return n;
        }
    }

    /// <summary>Adds an agent to the crowd of the class that fits its radius (<see cref="NavWorld.ClassFor"/>) or of <paramref name="classIndex"/>.</summary>
    /// <param name="position">World position.</param>
    /// <param name="options">Agent settings; null means <see cref="Agent.Humanoid"/>.</param>
    /// <param name="classIndex">The class to use; -1 (default) picks by radius.</param>
    /// <exception cref="ArgumentOutOfRangeException">As for <see cref="Crowd.Add"/>, or the class is not -1 or a class index.</exception>
    /// <exception cref="ArgumentException">No walkable polygon of that class is near the position.</exception>
    public Agent Add(Vector3D position, AgentOptions? options = null, int classIndex = -1)
    {
        options ??= Agent.Humanoid;
        int c = classIndex == -1 ? _world.ClassFor(options.Radius) : classIndex;
        if ((uint)c >= (uint)_crowds.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(classIndex), classIndex, $"The world has {_crowds.Length} agent classes.");
        }
        return _crowds[c].Add(position, options);
    }

    /// <summary>Steps every class's crowd by <paramref name="dt"/> seconds, class 0 first.</summary>
    /// <exception cref="ArgumentOutOfRangeException">dt is not positive and finite.</exception>
    public void Step(double dt)
    {
        foreach (Crowd c in _crowds)
        {
            c.Step(dt);
        }
    }
}
