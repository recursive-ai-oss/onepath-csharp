using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// Tiles a host already holds, found by content id, so a snapshot taken with <see cref="SnapshotMode.Referenced"/>
/// need not carry their bytes. Lookup only: results never depend on insertion order.
/// </summary>
public sealed class TileLibrary
{
    private readonly SortedDictionary<TileContentId, NavTile> _tiles = new();

    /// <summary>Number of distinct tiles held.</summary>
    public int Count => _tiles.Count;

    /// <summary>Adds a tile (a tile with the same content is kept once).</summary>
    /// <returns>The tile's content id.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="tile"/> is null.</exception>
    public TileContentId Add(NavTile tile)
    {
        ArgumentNullException.ThrowIfNull(tile);
        TileContentId id = tile.ContentId;
        // Kept as built: a tile taken from a running mesh may carry runtime edits that a snapshot stores separately.
        if (!_tiles.ContainsKey(id))
        {
            _tiles.Add(id, tile.PristineCopy());
        }
        return id;
    }

    /// <summary>Adds every tile currently in <paramref name="mesh"/>.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    public void AddAll(TiledNavMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            if (mesh.GetTileBySlot(s) is { } t)
            {
                Add(t);
            }
        }
    }

    /// <summary>Finds a tile by content id.</summary>
    public bool TryGet(TileContentId id, out NavTile tile)
    {
        if (_tiles.TryGetValue(id, out NavTile? t))
        {
            tile = t;
            return true;
        }
        tile = null!;
        return false;
    }
}
