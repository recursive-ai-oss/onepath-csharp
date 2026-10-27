using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>How a snapshot stores the tiles in the slots.</summary>
public enum SnapshotMode
{
    /// <summary>Every tile's bytes are inside the snapshot: self-contained, restorable anywhere.</summary>
    Full = 0,

    /// <summary>
    /// Every tile is named by its <see cref="TileContentId"/>; restore needs a <see cref="TileLibrary"/> holding those
    /// tiles. Small and fast for a host that already keeps the tile data (a rollback host snapshots every frame).
    /// </summary>
    Referenced = 1,
}

/// <summary>
/// Byte-exact snapshot and restore of the run-time state of a <see cref="TiledNavMesh"/> (format in
/// <c>docs/navmesh-snapshot.md</c>): which tile sits in which slot with which generation, runtime polygon area and flag
/// edits, and switched-off off-mesh links. The same state always produces the same bytes, and a restored mesh
/// snapshots back to the same bytes, answers every query the same and treats stale polygon references the same.
/// </summary>
public static class NavMeshSnapshot
{
    private const uint Magic = 0x5350_4E4F; // "ONPS" little-endian
    private const ushort SectionSlots = 1;

    private const byte SlotFree = 0;
    private const byte SlotInline = 1;
    private const byte SlotReferenced = 2;

    /// <summary>Captures the run-time state of <paramref name="mesh"/>.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="mode"/> is not a defined value.</exception>
    public static byte[] Take(TiledNavMesh mesh, SnapshotMode mode = SnapshotMode.Full)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        if (mode != SnapshotMode.Full && mode != SnapshotMode.Referenced)
        {
            throw new ArgumentOutOfRangeException(nameof(mode), mode, "Unknown snapshot mode.");
        }
        var body = new NavMeshSerializer.Writer(256);
        body.U32((uint)mesh.SlotCount);
        Span<byte> id = stackalloc byte[32];
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            body.U32(mesh.GetSlotSalt(s));
            if (mesh.GetTileBySlot(s) is not { } t)
            {
                body.U8(SlotFree);
                continue;
            }
            if (mode == SnapshotMode.Full)
            {
                byte[] tile = NavMeshSerializer.WriteTile(t, pristine: true);
                body.U8(SlotInline);
                body.U32((uint)tile.Length);
                body.Bytes(tile);
            }
            else
            {
                t.ContentId.CopyTo(id);
                body.U8(SlotReferenced);
                body.Bytes(id);
            }

            // Runtime edits: polygons whose area or flags differ from the as-built values, in polygon order.
            int edits = 0;
            for (int p = 0; p < t.PolyCount; p++)
            {
                edits += IsEdited(t, p) ? 1 : 0;
            }
            body.U32((uint)edits);
            for (int p = 0; p < t.PolyCount; p++)
            {
                if (IsEdited(t, p))
                {
                    body.U32((uint)p);
                    body.U8(t.PolyAreas[p]);
                    body.U16(t.PolyFlags[p]);
                }
            }
            int disabled = 0;
            for (int l = 0; l < t.OffMeshLinks.Length; l++)
            {
                disabled += t.IsOffMeshDisabled(l) ? 1 : 0;
            }
            body.U32((uint)disabled);
            for (int l = 0; l < t.OffMeshLinks.Length; l++)
            {
                if (t.IsOffMeshDisabled(l))
                {
                    body.U32((uint)l);
                }
            }
        }
        byte[] slots = body.ToArray();

        var w = new NavMeshSerializer.Writer(slots.Length + 128);
        w.U32(Magic);
        w.U16(NavMeshFormat.Version);
        w.U16(NavMeshSerializer.HeaderFlags);
        NavMeshSerializer.WriteParams(w, mesh.Params);
        w.U32(1); // section count
        w.U16(SectionSlots);
        w.U16(0);
        w.U32((uint)slots.Length);
        w.Bytes(slots);
        return w.ToArray();
    }

    /// <summary>Builds a new mesh from a snapshot.</summary>
    /// <param name="snapshot">Bytes written by <see cref="Take"/>.</param>
    /// <param name="library">Tiles for referenced slots; may be null for a self-contained snapshot.</param>
    /// <exception cref="NavMeshFormatException">The bytes are malformed or truncated, a referenced tile is not in the library, or a tile does not fit the snapshot's parameters.</exception>
    public static TiledNavMesh Restore(ReadOnlySpan<byte> snapshot, TileLibrary? library = null) => Parse(snapshot, library, null);

    /// <summary>
    /// Replaces the run-time state of <paramref name="mesh"/> with a snapshot, keeping the object (and anything holding it,
    /// such as query objects). All or nothing: on any error the mesh is untouched.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    /// <exception cref="NavMeshFormatException">As for <see cref="Restore"/>, or the snapshot was taken from a mesh with different parameters.</exception>
    public static void RestoreInto(TiledNavMesh mesh, ReadOnlySpan<byte> snapshot, TileLibrary? library = null)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        Parse(snapshot, library, mesh);
    }

    private static bool IsEdited(NavTile t, int p) => t.PolyAreas[p] != t.BaseAreas[p] || t.PolyFlags[p] != t.BaseFlags[p];

    private static TiledNavMesh Parse(ReadOnlySpan<byte> bytes, TileLibrary? library, TiledNavMesh? target)
    {
        var r = new NavMeshSerializer.Reader(bytes);
        if (r.U32("magic") != Magic)
        {
            throw new NavMeshFormatException("Not a navigation mesh snapshot (bad magic).");
        }
        NavMeshSerializer.CheckVersion(ref r);
        NavMeshParams p = NavMeshSerializer.ReadParams(ref r);
        if (target is not null && !target.Params.SameAs(p))
        {
            throw new NavMeshFormatException("The snapshot was taken from a mesh with different parameters than the mesh being restored.");
        }
        uint sections = r.U32("section count");
        if (sections > r.Remaining / 8)
        {
            throw new NavMeshFormatException($"Section count {sections} is not possible for {r.Remaining} remaining bytes.");
        }
        List<TiledNavMesh.SlotRestore>? slots = null;
        int lastId = 0;
        for (uint i = 0; i < sections; i++)
        {
            ushort id = r.U16("section id");
            if (r.U16("section reserved") != 0)
            {
                throw new NavMeshFormatException($"Section {id}: reserved field is not zero.");
            }
            uint len = r.U32("section length");
            ReadOnlySpan<byte> body = r.Take((int)Math.Min(len, int.MaxValue), $"section {id} body");
            // Sections come in ascending id order, once each. An id this version does not know would be state it
            // cannot restore, so it is an error rather than something to skip.
            if (id <= lastId)
            {
                throw new NavMeshFormatException($"Section {id} is repeated or out of order.");
            }
            lastId = id;
            if (id != SectionSlots)
            {
                throw new NavMeshFormatException($"Unknown snapshot section {id}; this build reads section {SectionSlots} only.");
            }
            slots = ReadSlots(body, p, library);
        }
        r.ExpectEnd();
        if (slots is null)
        {
            throw new NavMeshFormatException("The snapshot has no slot section.");
        }
        var staging = new TiledNavMesh(p);
        staging.RestoreSlots(slots);
        if (target is null)
        {
            return staging;
        }
        target.AdoptState(staging);
        return target;
    }

    private static List<TiledNavMesh.SlotRestore> ReadSlots(ReadOnlySpan<byte> body, NavMeshParams p, TileLibrary? library)
    {
        var r = new NavMeshSerializer.Reader(body);
        uint count = r.U32("slot count");
        if (count > NavMeshFormat.MaxTileSlots || count > r.Remaining / 5)
        {
            throw new NavMeshFormatException($"Slot count {count} is not possible for {r.Remaining} remaining bytes.");
        }
        var slots = new List<TiledNavMesh.SlotRestore>((int)count);
        for (uint s = 0; s < count; s++)
        {
            uint salt = r.U32("slot generation");
            if (salt == 0 || salt > 0xFFFFFF)
            {
                throw new NavMeshFormatException($"Slot {s} has generation {salt}; it must be 1 to 16777215.");
            }
            byte kind = r.U8("slot kind");
            NavTile tile;
            if (kind == SlotFree)
            {
                slots.Add(new TiledNavMesh.SlotRestore(salt, null, [], []));
                continue;
            }
            if (kind == SlotInline)
            {
                uint len = r.U32("tile length");
                ReadOnlySpan<byte> tb = r.Take((int)Math.Min(len, int.MaxValue), "tile body");
                try
                {
                    tile = NavMeshSerializer.ReadTile(tb, p);
                }
                catch (NavMeshFormatException e)
                {
                    throw new NavMeshFormatException($"Slot {s}: {e.Message}", e);
                }
            }
            else if (kind == SlotReferenced)
            {
                var id = TileContentId.FromBytes(r.Take(32, "tile content id"));
                if (library is null || !library.TryGet(id, out NavTile? found))
                {
                    throw new NavMeshFormatException($"Slot {s}: the snapshot refers to tile {id}, which is not in the tile library.");
                }
                if (!found.Params.SameAs(p))
                {
                    throw new NavMeshFormatException($"Slot {s}: library tile {id} was built with different mesh parameters than the snapshot's.");
                }
                tile = found;
            }
            else
            {
                throw new NavMeshFormatException($"Slot {s} has unknown kind {kind}.");
            }

            uint nEdits = r.U32("edit count");
            if (nEdits > tile.PolyCount || (ulong)nEdits * 7 > (ulong)r.Remaining)
            {
                throw new NavMeshFormatException($"Slot {s}: {nEdits} polygon edits are not possible for a tile of {tile.PolyCount} polygons.");
            }
            var edits = new TiledNavMesh.PolyEdit[nEdits];
            long prev = -1;
            for (int i = 0; i < edits.Length; i++)
            {
                uint poly = r.U32("edit polygon");
                byte area = r.U8("edit area");
                ushort flags = r.U16("edit flags");
                if (poly >= tile.PolyCount || poly <= prev)
                {
                    throw new NavMeshFormatException($"Slot {s}: edit {i} names polygon {poly}, which is out of range or not in ascending order.");
                }
                if (area > NavMeshFormat.MaxArea)
                {
                    throw new NavMeshFormatException($"Slot {s}: edit {i} has area {area}; the largest area id is {NavMeshFormat.MaxArea}.");
                }
                // Only real differences are stored, so one state has one encoding.
                if (area == tile.BaseAreas[(int)poly] && flags == tile.BaseFlags[(int)poly])
                {
                    throw new NavMeshFormatException($"Slot {s}: edit {i} on polygon {poly} equals the as-built values.");
                }
                prev = poly;
                edits[i] = new TiledNavMesh.PolyEdit((int)poly, area, flags);
            }
            uint nDisabled = r.U32("disabled link count");
            if (nDisabled > tile.OffMeshLinks.Length || (ulong)nDisabled * 4 > (ulong)r.Remaining)
            {
                throw new NavMeshFormatException($"Slot {s}: {nDisabled} disabled links are not possible for a tile with {tile.OffMeshLinks.Length} off-mesh links.");
            }
            var disabled = new int[nDisabled];
            prev = -1;
            for (int i = 0; i < disabled.Length; i++)
            {
                uint l = r.U32("disabled link");
                if (l >= tile.OffMeshLinks.Length || l <= prev)
                {
                    throw new NavMeshFormatException($"Slot {s}: disabled link {l} is out of range or not in ascending order.");
                }
                prev = l;
                disabled[i] = (int)l;
            }
            slots.Add(new TiledNavMesh.SlotRestore(salt, tile, edits, disabled));
        }
        r.ExpectEnd();
        return slots;
    }
}
