using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>
/// The exact 2D builder as separate stages, each a plain static method over plain types so any can be replaced:
/// walkable <see cref="Region"/> → <see cref="Shrink"/> by the agent radius → <see cref="TileSlicer.Slice"/> →
/// <see cref="ToTileData"/> per tile → <see cref="Assemble"/> into a <see cref="TiledNavMesh"/>. <see cref="Build"/>
/// runs them in that order. The plane maps to the mesh as x → X, y → Z, with a constant world Y.
/// </summary>
public static class Mesh2DPipeline
{
    /// <summary>The geometry options for the settings' snap grid (null SnapBits: the finest the cell size allows).</summary>
    public static Geometry2DOptions GeometryOptions(NavSettings settings)
    {
        ArgumentNullException.ThrowIfNull(settings);
        return new Geometry2DOptions { SnapBits = settings.SnapBits ?? TileSlicer.FinestSnapBits(settings.CellSize) };
    }

    /// <summary>
    /// The arc tolerance the build passes to <see cref="Shrink"/>: a quarter cell, raised to the octagon's tolerance
    /// (0.0825 of the radius) so that mid-sized radii use 8 vertices per circle, but never above half a cell, so the
    /// polygonal arcs stay within a cell of the true circle at any radius.
    /// </summary>
    internal static double ArcTolerance(NavSettings settings) =>
        Math.Max(settings.CellSize / 4, Math.Min(0.0825 * settings.AgentRadius, settings.CellSize / 2));

    /// <summary>
    /// Shrinks the walkable area by the agent radius (round joins); a radius of zero returns the region. The polygonal
    /// arcs stay within <paramref name="arcTolerance"/> of the true circle (default: a sixteenth of the agent radius,
    /// far below a navigation cell); only the pieces that can change the result are built.
    /// </summary>
    public static Region Shrink(Region walkable, double agentRadius, double arcTolerance = 0)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        return agentRadius > 0 ? walkable.ShrinkOneSided(agentRadius, arcTolerance > 0 ? arcTolerance : agentRadius / 16.0) : walkable;
    }

    /// <summary>
    /// Turns one tile's partition into tile data: world vertices (x, <paramref name="height"/>, y), the polygons, one
    /// <paramref name="area"/> and <paramref name="flags"/> for all, in-tile adjacency and tile-line portals. The
    /// detail mesh is empty.
    /// </summary>
    public static NavTileData ToTileData(TilePartition tile, NavMeshParams p, double height, byte area, ushort flags)
    {
        ArgumentNullException.ThrowIfNull(tile);
        ConvexPartition part = tile.Partition;
        ReadOnlySpan<Vector2D> pv = part.Vertices;
        var verts = new double[pv.Length * 3];
        for (int i = 0; i < pv.Length; i++)
        {
            verts[3 * i] = pv[i].X;
            verts[(3 * i) + 1] = height;
            verts[(3 * i) + 2] = pv[i].Y;
        }
        int np = part.PolygonCount;
        var counts = new int[np];
        var indices = new List<int>();
        for (int i = 0; i < np; i++)
        {
            ReadOnlySpan<int> ix = part.GetIndices(i);
            counts[i] = ix.Length;
            foreach (int v in ix)
            {
                indices.Add(v);
            }
        }
        var areas = new byte[np];
        var fl = new ushort[np];
        Array.Fill(areas, area);
        Array.Fill(fl, flags);
        return NavTileAssembler.FromPolygons(p, tile.TileX, tile.TileZ, 0, height, verts, counts, indices.ToArray(), areas, fl);
    }

    /// <summary>Validates every tile and adds it to a new mesh, stitching neighbours.</summary>
    public static TiledNavMesh Assemble(NavMeshParams p, IEnumerable<NavTileData> tiles)
    {
        ArgumentNullException.ThrowIfNull(p);
        ArgumentNullException.ThrowIfNull(tiles);
        var mesh = new TiledNavMesh(p);
        foreach (NavTileData d in tiles)
        {
            mesh.AddTile(NavTile.Create(d, p));
        }
        return mesh;
    }

    /// <summary>Runs all stages on an already resolved walkable region.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range or the geometry leaves ±2^21.</exception>
    public static TiledNavMesh Build(Region walkable, NavSettings settings)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        ArgumentNullException.ThrowIfNull(settings);
        NavMeshParams p = settings.ToParams();
        NavMeshParams.CheckWorld(settings.Height, nameof(settings.Height));
        OffMeshLinkBaker.Validate(p, settings.OffMeshLinks);
        Region shrunk = Shrink(walkable, settings.AgentRadius, ArcTolerance(settings));
        var tiles = new List<NavTileData>();
        foreach (TilePartition t in TileSlicer.Slice(shrunk, p))
        {
            tiles.Add(ToTileData(t, p, settings.Height, settings.Area, settings.Flags));
        }
        OffMeshLinkBaker.Attach(p, settings.OffMeshLinks, tiles, 0);
        return Assemble(p, tiles);
    }
}
