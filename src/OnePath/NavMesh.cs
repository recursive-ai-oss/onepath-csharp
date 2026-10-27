using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath;

/// <summary>
/// A navigation mesh for one agent class: the tiled mesh and the settings it was built with. Build it from outlines or
/// a tile map with <see cref="Build2D(IEnumerable{Outline}, NavSettings?)"/>. The type is partial so that the
/// query layer adds its members in a file of its own.
/// </summary>
public sealed partial class NavMesh
{
    internal NavMesh(TiledNavMesh tiles, NavSettings settings)
    {
        Tiles = tiles;
        Settings = settings;
    }

    /// <summary>The tiled mesh (tier 2 and 3 access).</summary>
    public TiledNavMesh Tiles { get; }

    /// <summary>The settings the mesh was built with.</summary>
    public NavSettings Settings { get; }

    /// <summary>
    /// Builds a mesh from walkable outlines: the union of the outlines (each the even-odd region of its rings) is the
    /// walkable area, which is shrunk by the agent radius and cut into tiles. Plane x is world X, plane y is world Z;
    /// the floor is at <see cref="NavSettings.Height"/>.
    /// </summary>
    /// <param name="walkable">The walkable outlines.</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or beyond ±2^21 (the message names it), or a setting is out of range.</exception>
    public static NavMesh Build2D(IEnumerable<Outline> walkable, NavSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        settings ??= NavSettings.Humanoid;
        Region r = Region.FromOutlines(walkable, Mesh2DPipeline.GeometryOptions(settings));
        return Build2D(r, settings);
    }

    /// <summary>
    /// Builds a mesh from a tile map: cell (x, y) of <paramref name="cells"/> (row-major) is walkable when true and spans
    /// <c>origin + (x, y) * cellSize</c> to <c>origin + (x + 1, y + 1) * cellSize</c>.
    /// </summary>
    /// <exception cref="ArgumentException">The span length is not <c>width * height</c>, or a size is not positive.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A grid line is beyond ±2^21, or a setting is out of range.</exception>
    public static NavMesh Build2D(ReadOnlySpan<bool> cells, int width, int height, double cellSize, Vector2D origin, NavSettings? settings = null)
    {
        settings ??= NavSettings.Humanoid;
        Region r = Region.FromTileMap(cells, width, height, cellSize, origin, Mesh2DPipeline.GeometryOptions(settings));
        return Build2D(r, settings);
    }

    /// <summary>Builds a mesh from an already resolved walkable region (for example after boolean operations with obstacles).</summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static NavMesh Build2D(Region walkable, NavSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        settings ??= NavSettings.Humanoid;
        return new NavMesh(Mesh2DPipeline.Build(walkable, settings), settings);
    }
}
