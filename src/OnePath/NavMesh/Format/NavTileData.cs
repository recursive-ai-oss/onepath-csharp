using System;

namespace RecursiveAI.OnePath.NavMesh;

/// <summary>
/// The plain, mutable description of one tile that builders fill and <see cref="NavTile.Create"/> validates and
/// copies. Null arrays count as empty. See <c>docs/navmesh-format.md</c> for the meaning of every field.
/// </summary>
public sealed class NavTileData
{
    /// <summary>Tile column.</summary>
    public int TileX { get; set; }

    /// <summary>Tile row.</summary>
    public int TileZ { get; set; }

    /// <summary>Layer within the tile column/row (overlapping floors), 0 to 1023.</summary>
    public int Layer { get; set; }

    /// <summary>World Y of the tile-local Y origin (normally the tile's lowest point).</summary>
    public double OriginY { get; set; }

    /// <summary>Vertices, three float32 per vertex (x, y, z), tile-local.</summary>
    public float[]? Vertices { get; set; }

    /// <summary>Vertex count of each polygon.</summary>
    public byte[]? PolyVertexCounts { get; set; }

    /// <summary>Vertex indices of all polygons concatenated; counter-clockwise in X (right) / Z (up).</summary>
    public ushort[]? PolyIndices { get; set; }

    /// <summary>Per polygon edge (parallel to <see cref="PolyIndices"/>; edge i runs from vertex i to i+1): 0 for a wall or portal, otherwise 1 + the index of the polygon across the edge inside this tile.</summary>
    public int[]? EdgeNeighbors { get; set; }

    /// <summary>Per polygon edge: the tile side (0 = +X, 1 = +Z, 2 = -X, 3 = -Z) if the edge is a tile portal, otherwise 255.</summary>
    public byte[]? EdgePortalSides { get; set; }

    /// <summary>Area id of each polygon, 0 to 63.</summary>
    public byte[]? PolyAreas { get; set; }

    /// <summary>Flags of each polygon, for query filters.</summary>
    public ushort[]? PolyFlags { get; set; }

    /// <summary>Per-polygon detail mesh ranges; empty (no detail anywhere) or one per polygon.</summary>
    public DetailMesh[]? DetailMeshes { get; set; }

    /// <summary>Extra detail vertices, three float32 each, tile-local.</summary>
    public float[]? DetailVertices { get; set; }

    /// <summary>Detail triangles, four bytes each: three vertex indices (polygon vertices first, then the polygon's extra detail vertices) and an edge-flag byte.</summary>
    public byte[]? DetailTriangles { get; set; }

    /// <summary>Off-mesh link slots owned by this tile.</summary>
    public OffMeshLink[]? OffMeshLinks { get; set; }
}
