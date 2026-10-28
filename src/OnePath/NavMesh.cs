using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath;

/// <summary>
/// A navigation mesh for one agent class: the tiled mesh and the settings it was built with. Build it from outlines or
/// a tile map with <see cref="Build2D(IEnumerable{Outline}, NavSettings?)"/>.
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
    /// the floor is at <see cref="NavSettings.FloorY"/>.
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

    /// <summary>
    /// Builds a mesh from a square <see cref="Grid"/>: every cell that is not a wall is walkable. The mesh equals
    /// <see cref="Build2D(ReadOnlySpan{bool}, int, int, double, Vector2D, NavSettings?)"/> called with the grid's walkability, the cell size
    /// <see cref="GridSpace.CellSize"/> and the origin (<see cref="GridSpace.OriginX"/>, <see cref="GridSpace.OriginY"/>) of <see cref="Grid.Space"/>:
    /// grid column x is world X, grid row y is world Z, and the floor is at <see cref="NavSettings.FloorY"/>.
    /// <para>
    /// Clearance: the walkable cells are shrunk by <see cref="NavSettings.AgentRadius"/> like every 2D build, so an agent's centre may
    /// stand where its disc fits inside the walkable cells; the grid's own agent-size clearance is not used.
    /// Areas: the 2D builder gives every polygon of a mesh one area (<see cref="NavSettings.Area"/>), so the grid's area ids and
    /// <see cref="Grid.Areas"/> multipliers are not carried over; only wall versus not-wall is. Use a query filter for per-area costs.
    /// A grid with no walkable cell gives a mesh with no polygons (paths fail with a status, they do not throw).
    /// </para>
    /// </summary>
    /// <param name="grid">The grid; <see cref="Grid.Space"/> gives cell size and origin (default: unit cells at the origin).</param>
    /// <param name="settings">Build settings; null means <see cref="NavSettings.Humanoid"/>.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A grid line is beyond ±2^21, or a setting is out of range.</exception>
    public static NavMesh Build2D(Grid grid, NavSettings? settings = null)
    {
        ArgumentNullException.ThrowIfNull(grid);
        ReadOnlySpan<byte> areas = grid.Cells;
        var walkable = new bool[areas.Length];
        for (int i = 0; i < walkable.Length; i++)
        {
            walkable[i] = areas[i] != AreaCosts.Wall;
        }
        GridSpace space = grid.Space;
        return Build2D(walkable, grid.Width, grid.Height, space.CellSize, new Vector2D(space.OriginX, space.OriginY), settings);
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
