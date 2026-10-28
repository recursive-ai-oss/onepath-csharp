using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// A tiled navigation mesh: tiles keyed by (column, row, layer) in numbered slots, stable polygon references and
/// the stitched links between tiles. Adding, removing and replacing tiles is deterministic: slots are
/// reused lowest-first, and the links of a tile set are a pure function of the set (the same tiles give the same
/// links whatever order they were added in).
/// <para>
/// Not thread-safe for modification. Read-only use from several threads is safe while nobody modifies the mesh.
/// </para>
/// </summary>
public sealed class TiledNavMesh
{
    private static readonly TileLinks s_noLinks = new([], [0]);

    private readonly List<NavTile?> _tiles = new();
    private readonly List<uint> _salts = new();
    private readonly List<TileLinks> _links = new();
    private readonly Dictionary<long, List<int>> _columns = new();
    private int _tileCount;
    private int _offMeshTileCount;

    /// <summary>Creates an empty mesh.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="p"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A parameter is out of range.</exception>
    public TiledNavMesh(NavMeshParams p)
    {
        ArgumentNullException.ThrowIfNull(p);
        p.Validate();
        Params = p;
    }

    /// <summary>The grid and agent parameters.</summary>
    public NavMeshParams Params { get; }

    /// <summary>Number of tiles present.</summary>
    public int TileCount => _tileCount;

    /// <summary>
    /// Counts the changes made to the mesh through this object (tiles added, replaced or removed, polygon flags and areas set, links
    /// switched, state restored). It only grows within a process; it is not saved, so compare it for "changed since", never for equality across runs.
    /// </summary>
    public long EditCount => _editCount;

    private long _editCount;

    /// <summary>Number of slots ever used (present or free); slot indices are 0 to SlotCount - 1.</summary>
    public int SlotCount => _tiles.Count;

    /// <summary>The tile in <paramref name="slot"/>, or null if the slot is free or out of range.</summary>
    public NavTile? GetTileBySlot(int slot) => (uint)slot < (uint)_tiles.Count ? _tiles[slot] : null;

    /// <summary>The current generation of <paramref name="slot"/> (0 if the slot does not exist).</summary>
    public uint GetSlotSalt(int slot) => (uint)slot < (uint)_salts.Count ? _salts[slot] : 0;

    /// <summary>The reference of the tile in <paramref name="slot"/>, or <see cref="TileRef.Invalid"/> when the slot is free or out of range.</summary>
    public TileRef GetSlotRef(int slot) => (uint)slot < (uint)_tiles.Count && _tiles[slot] is not null ? new TileRef(_salts[slot], slot) : TileRef.Invalid;

    /// <summary>Adds a tile and stitches it to its neighbours.</summary>
    /// <returns>A reference to the new tile.</returns>
    /// <exception cref="ArgumentException">The tile was built with different parameters.</exception>
    /// <exception cref="InvalidOperationException">A tile already exists at the tile's (column, row, layer), or no slot is left.</exception>
    public TileRef AddTile(NavTile tile)
    {
        _editCount++;
        CheckTile(tile);
        if (FindSlot(tile.TileX, tile.TileZ, tile.Layer) >= 0)
        {
            throw new InvalidOperationException($"A tile already exists at ({tile.TileX}, {tile.TileZ}, layer {tile.Layer}); use ReplaceTile.");
        }
        int slot = AllocateSlot();
        Place(slot, tile);
        Restitch(tile.TileX, tile.TileZ, tile.OffMeshLinks.Length > 0);
        return new TileRef(_salts[slot], slot);
    }

    /// <summary>
    /// Replaces the tile at the same (column, row, layer), or adds it when none exists. The slot's generation
    /// changes, so every polygon reference into the old tile becomes invalid.
    /// </summary>
    /// <exception cref="ArgumentException">The tile was built with different parameters.</exception>
    public TileRef ReplaceTile(NavTile tile)
    {
        _editCount++;
        CheckTile(tile);
        int slot = FindSlot(tile.TileX, tile.TileZ, tile.Layer);
        if (slot < 0)
        {
            return AddTile(tile);
        }
        bool hadOffMesh = _tiles[slot]!.OffMeshLinks.Length > 0;
        Unplace(slot);
        Place(slot, tile);
        Restitch(tile.TileX, tile.TileZ, hadOffMesh || tile.OffMeshLinks.Length > 0);
        return new TileRef(_salts[slot], slot);
    }

    /// <summary>Removes a tile; its polygon references become invalid.</summary>
    /// <returns>False if the reference was not valid (nothing changed).</returns>
    public bool RemoveTile(TileRef tile)
    {
        _editCount++;
        if (!IsValid(tile))
        {
            return false;
        }
        NavTile t = _tiles[tile.Slot]!;
        bool hadOffMesh = t.OffMeshLinks.Length > 0;
        Unplace(tile.Slot);
        Restitch(t.TileX, t.TileZ, hadOffMesh);
        return true;
    }

    /// <summary>Removes the tile at the given position, if any.</summary>
    public bool RemoveTile(int tileX, int tileZ, int layer) => RemoveTile(GetTileRef(tileX, tileZ, layer));

    /// <summary>True if <paramref name="tile"/> still refers to the tile it was issued for.</summary>
    public bool IsValid(TileRef tile) =>
        tile.IsSet && (uint)tile.Slot < (uint)_tiles.Count && _salts[tile.Slot] == tile.Salt && _tiles[tile.Slot] is not null;

    /// <summary>The reference of the tile at (column, row, layer), or <see cref="TileRef.Invalid"/>.</summary>
    public TileRef GetTileRef(int tileX, int tileZ, int layer)
    {
        int slot = FindSlot(tileX, tileZ, layer);
        return slot < 0 ? TileRef.Invalid : new TileRef(_salts[slot], slot);
    }

    /// <summary>The tile behind a reference, or null if the reference is stale.</summary>
    public NavTile? GetTile(TileRef tile) => IsValid(tile) ? _tiles[tile.Slot] : null;

    /// <summary>
    /// Writes the references of all layers present at (column, row), lowest layer first, into <paramref name="destination"/>.
    /// </summary>
    /// <returns>How many tiles exist there (may exceed the destination length).</returns>
    public int GetTilesAt(int tileX, int tileZ, Span<TileRef> destination)
    {
        if (!_columns.TryGetValue(ColumnKey(tileX, tileZ), out var list))
        {
            return 0;
        }
        for (int i = 0; i < list.Count && i < destination.Length; i++)
        {
            destination[i] = new TileRef(_salts[list[i]], list[i]);
        }
        return list.Count;
    }

    /// <summary>True if <paramref name="poly"/> refers to an existing polygon of a tile still in its slot.</summary>
    public bool IsValid(PolyRef poly) => TryGetPoly(poly, out _, out _);

    /// <summary>Resolves a polygon reference.</summary>
    /// <returns>False for a stale, forged or invalid reference.</returns>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public bool TryGetPoly(PolyRef poly, out NavTile tile, out int polyIndex)
    {
        int slot = poly.TileSlot;
        if (poly.IsSet && (uint)slot < (uint)_tiles.Count && _salts[slot] == poly.Salt && _tiles[slot] is { } t && poly.PolyIndex < t.PolyCount)
        {
            tile = t;
            polyIndex = poly.PolyIndex;
            return true;
        }
        tile = null!;
        polyIndex = -1;
        return false;
    }

    /// <summary>The stitched links leaving <paramref name="poly"/> through tile portals and off-mesh links (empty for an invalid reference). In-tile neighbours are in <see cref="NavTile.EdgeNeighbors"/>.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public ReadOnlySpan<NavLink> GetLinks(PolyRef poly)
    {
        if (!TryGetPoly(poly, out _, out int pi))
        {
            return default;
        }
        TileLinks tl = _links[poly.TileSlot];
        int a = tl.PolyStart[pi];
        int b = tl.PolyStart[pi + 1];
        return tl.Links.AsSpan(a, b - a);
    }

    // Links of a polygon already resolved by TryGetPoly (slot and index known valid), skipping the second validation.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    internal ReadOnlySpan<NavLink> LinksOfResolved(int slot, int polyIndex)
    {
        TileLinks tl = _links[slot];
        int a = tl.PolyStart[polyIndex];
        return tl.Links.AsSpan(a, tl.PolyStart[polyIndex + 1] - a);
    }

    /// <summary>The reference for polygon <paramref name="polyIndex"/> of a valid tile reference, or <see cref="PolyRef.Invalid"/>.</summary>
    public PolyRef GetPolyRef(TileRef tile, int polyIndex)
    {
        NavTile? t = GetTile(tile);
        return t is not null && (uint)polyIndex < (uint)t.PolyCount ? tile.Poly(polyIndex) : PolyRef.Invalid;
    }

    // --- slot management ---

    private void CheckTile(NavTile tile)
    {
        ArgumentNullException.ThrowIfNull(tile);
        if (!tile.Params.SameAs(Params))
        {
            throw new ArgumentException("The tile was built with different mesh parameters.", nameof(tile));
        }
    }

    private static long ColumnKey(int x, int z) => ((long)x << 32) | (uint)z;

    private int FindSlot(int x, int z, int layer)
    {
        if (_columns.TryGetValue(ColumnKey(x, z), out var list))
        {
            foreach (int s in list)
            {
                if (_tiles[s]!.Layer == layer)
                {
                    return s;
                }
            }
        }
        return -1;
    }

    private int AllocateSlot()
    {
        for (int i = 0; i < _tiles.Count; i++)
        {
            if (_tiles[i] is null)
            {
                return i;
            }
        }
        if (_tiles.Count >= NavMeshFormat.MaxTileSlots)
        {
            throw new InvalidOperationException("The mesh has no free tile slot.");
        }
        _tiles.Add(null);
        _salts.Add(1);
        _links.Add(s_noLinks);
        return _tiles.Count - 1;
    }

    private void Place(int slot, NavTile tile)
    {
        // The mesh keeps its own copy so runtime edits never reach a tile object the caller or another mesh still holds.
        tile = tile.OwnedCopy();
        _tiles[slot] = tile;
        _tileCount++;
        if (tile.OffMeshLinks.Length > 0)
        {
            _offMeshTileCount++;
        }
        long key = ColumnKey(tile.TileX, tile.TileZ);
        if (!_columns.TryGetValue(key, out var list))
        {
            list = new List<int>();
            _columns[key] = list;
        }
        int at = 0;
        while (at < list.Count && _tiles[list[at]]!.Layer < tile.Layer)
        {
            at++;
        }
        list.Insert(at, slot);
    }

    private void Unplace(int slot)
    {
        NavTile t = _tiles[slot]!;
        long key = ColumnKey(t.TileX, t.TileZ);
        var list = _columns[key];
        list.Remove(slot);
        if (list.Count == 0)
        {
            _columns.Remove(key);
        }
        if (t.OffMeshLinks.Length > 0)
        {
            _offMeshTileCount--;
        }
        _tiles[slot] = null;
        _links[slot] = s_noLinks;
        _tileCount--;
        uint salt = (_salts[slot] + 1) & 0xFFFFFF;
        _salts[slot] = salt == 0 ? 1 : salt;
    }

    // Used by the deserializer and by snapshot restore: slots are restored exactly (including free slots and their
    // generations), then runtime edits are applied before the links are built.
    internal void RestoreSlots(IReadOnlyList<(uint Salt, NavTile? Tile)> slots)
    {
        _editCount++;
        var list = new List<SlotRestore>(slots.Count);
        foreach ((uint salt, NavTile? tile) in slots)
        {
            list.Add(new SlotRestore(salt, tile, [], []));
        }
        RestoreSlots(list);
    }

    internal readonly record struct PolyEdit(int Poly, byte Area, ushort Flags);

    internal readonly record struct SlotRestore(uint Salt, NavTile? Tile, PolyEdit[] Edits, int[] DisabledLinks);

    internal void RestoreSlots(IReadOnlyList<SlotRestore> slots)
    {
        _editCount++;
        for (int i = 0; i < slots.Count; i++)
        {
            _tiles.Add(null);
            _salts.Add(slots[i].Salt);
            _links.Add(s_noLinks);
            if (slots[i].Tile is { } t)
            {
                CheckTile(t);
                if (FindSlot(t.TileX, t.TileZ, t.Layer) >= 0)
                {
                    throw new NavMeshFormatException($"Slots hold two tiles at ({t.TileX}, {t.TileZ}, layer {t.Layer}).");
                }
                Place(i, t);
                NavTile placed = _tiles[i]!;
                foreach (PolyEdit e in slots[i].Edits)
                {
                    placed.SetPolyArea(e.Poly, e.Area);
                    placed.SetPolyFlags(e.Poly, e.Flags);
                }
                foreach (int l in slots[i].DisabledLinks)
                {
                    placed.SetOffMeshDisabled(l, true);
                }
            }
        }
        for (int i = 0; i < _tiles.Count; i++)
        {
            _links[i] = _tiles[i] is null ? s_noLinks : BuildLinks(i);
        }
    }

    // Moves the whole state of a freshly restored staging mesh (same parameters) into this one. Cannot fail, so a
    // restore either replaces everything or nothing.
    internal void AdoptState(TiledNavMesh o)
    {
        _editCount++;
        _tiles.Clear();
        _tiles.AddRange(o._tiles);
        _salts.Clear();
        _salts.AddRange(o._salts);
        _links.Clear();
        _links.AddRange(o._links);
        _columns.Clear();
        foreach (KeyValuePair<long, List<int>> kv in o._columns)
        {
            _columns.Add(kv.Key, kv.Value);
        }
        _tileCount = o._tileCount;
        _offMeshTileCount = o._offMeshTileCount;
    }

    // --- runtime state: polygon flags and areas, off-mesh link switches ---

    /// <summary>
    /// Changes the flags of a polygon at run time (a door that opens, a zone that closes). Takes effect for every
    /// later query; polygon references stay valid. Replacing the tile resets its edits to the new tile's values.
    /// </summary>
    /// <returns>False if the reference is stale or invalid (nothing changed).</returns>
    public bool SetPolyFlags(PolyRef poly, ushort flags)
    {
        _editCount++;
        if (!TryGetPoly(poly, out NavTile t, out int pi))
        {
            return false;
        }
        t.SetPolyFlags(pi, flags);
        return true;
    }

    /// <summary>Changes the area id of a polygon at run time (see <see cref="SetPolyFlags"/>).</summary>
    /// <returns>False if the reference is stale or invalid (nothing changed).</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="area"/> is above <see cref="NavMeshFormat.MaxArea"/>.</exception>
    public bool SetPolyArea(PolyRef poly, byte area)
    {
        _editCount++;
        if (area > NavMeshFormat.MaxArea)
        {
            throw new ArgumentOutOfRangeException(nameof(area), area, $"The largest area id is {NavMeshFormat.MaxArea}.");
        }
        if (!TryGetPoly(poly, out NavTile t, out int pi))
        {
            return false;
        }
        t.SetPolyArea(pi, area);
        return true;
    }

    /// <summary>
    /// Switches an off-mesh link of a tile on or off (all links start on). A disabled link has no stitched links, so
    /// no query uses it; the mesh is re-stitched immediately.
    /// </summary>
    /// <returns>False if the tile reference is stale or the link index does not exist (nothing changed).</returns>
    public bool SetOffMeshLinkEnabled(TileRef tile, int link, bool enabled)
    {
        _editCount++;
        NavTile? t = GetTile(tile);
        if (t is null || (uint)link >= (uint)t.OffMeshLinks.Length)
        {
            return false;
        }
        if (t.IsOffMeshDisabled(link) == !enabled)
        {
            return true;
        }
        t.SetOffMeshDisabled(link, !enabled);
        Restitch(t.TileX, t.TileZ, true);
        return true;
    }

    /// <summary>True if the off-mesh link exists and is switched on.</summary>
    public bool IsOffMeshLinkEnabled(TileRef tile, int link) =>
        GetTile(tile) is { } t && (uint)link < (uint)t.OffMeshLinks.Length && !t.IsOffMeshDisabled(link);

    /// <summary>
    /// Captures the run-time state of the mesh (which tiles sit in which slots with which generations, runtime polygon
    /// edits, switched-off off-mesh links) in the format of <c>docs/navmesh-snapshot.md</c>.
    /// </summary>
    public byte[] Snapshot(SnapshotMode mode = SnapshotMode.Full) => NavMeshSnapshot.Take(this, mode);

    /// <summary>
    /// Replaces the run-time state with a snapshot taken from a mesh with the same parameters. All or nothing: on a
    /// malformed snapshot the mesh is left unchanged.
    /// </summary>
    /// <exception cref="NavMeshFormatException">The bytes are malformed, a referenced tile is missing from <paramref name="library"/>, or the snapshot belongs to a mesh with different parameters.</exception>
    public void Restore(ReadOnlySpan<byte> snapshot, TileLibrary? library = null) => NavMeshSnapshot.RestoreInto(this, snapshot, library);

    // --- stitching ---

    private void Restitch(int tileX, int tileZ, bool offMeshInvolved)
    {
        if (offMeshInvolved || _offMeshTileCount > 0)
        {
            // Off-mesh endpoints can reach any tile: rebuild every tile's links, in slot order.
            for (int s = 0; s < _tiles.Count; s++)
            {
                _links[s] = _tiles[s] is null ? s_noLinks : BuildLinks(s);
            }
            return;
        }
        // Portals only join side-adjacent columns: rebuild the changed column and its four neighbours.
        for (int d = -1; d < NavMeshFormat.SideCount; d++)
        {
            int x = tileX + (d < 0 ? 0 : NavMeshFormat.SideDx(d));
            int z = tileZ + (d < 0 ? 0 : NavMeshFormat.SideDz(d));
            if (_columns.TryGetValue(ColumnKey(x, z), out var list))
            {
                foreach (int s in list)
                {
                    _links[s] = BuildLinks(s);
                }
            }
        }
    }

    private TileLinks BuildLinks(int slot)
    {
        NavTile t = _tiles[slot]!;
        var pending = new List<(long Key, NavLink Link)>();
        int seq = 0;

        // 1. Portals, in (side, polygon, edge) order of the portal lists.
        for (int side = 0; side < NavMeshFormat.SideCount; side++)
        {
            if (t.PortalCount(side) == 0 || !_columns.TryGetValue(ColumnKey(t.TileX + NavMeshFormat.SideDx(side), t.TileZ + NavMeshFormat.SideDz(side)), out var column))
            {
                continue;
            }
            int opp = NavMeshFormat.OppositeSide(side);
            for (int i = 0; i < t.PortalCount(side); i++)
            {
                int poly = t.PortalPoly(side, i);
                int ge = t.PortalEdge(side, i);
                foreach (int ns in column)
                {
                    NavTile nt = _tiles[ns]!;
                    for (int j = 0; j < nt.PortalCount(opp); j++)
                    {
                        if (MatchPortal(t, ge, poly, side, nt, nt.PortalEdge(opp, j), out byte rmin, out byte rmax))
                        {
                            int edge = ge - t.PolyStart[poly];
                            var link = new NavLink(new PolyRef(_salts[ns], ns, nt.PortalPoly(opp, j)), NavLinkKind.Portal, (byte)side, rmin, rmax, edge, -1);
                            pending.Add((((long)poly << 32) | (uint)seq++, link));
                        }
                    }
                }
            }
        }

        // 2. Forward off-mesh links that start in this tile.
        if (_offMeshTileCount > 0)
        {
            for (int i = 0; i < t.OffMeshLinks.Length; i++)
            {
                if (ResolveEnds(slot, i, out int startPoly, out int endSlot, out int endPoly))
                {
                    var link = new NavLink(new PolyRef(_salts[endSlot], endSlot, endPoly), NavLinkKind.OffMesh, NavMeshFormat.NoSide, 0, 0, i, slot);
                    pending.Add((((long)startPoly << 32) | (uint)seq++, link));
                }
            }

            // 3. Reverse links of bidirectional off-mesh links (from any tile, in slot order) that end in this tile.
            for (int os = 0; os < _tiles.Count; os++)
            {
                NavTile? ot = _tiles[os];
                if (ot is null)
                {
                    continue;
                }
                for (int i = 0; i < ot.OffMeshLinks.Length; i++)
                {
                    if (ot.OffMeshLinks[i].Bidirectional && ResolveEnds(os, i, out int startPoly, out int endSlot, out int endPoly) && endSlot == slot)
                    {
                        var link = new NavLink(new PolyRef(_salts[os], os, startPoly), NavLinkKind.OffMeshReverse, NavMeshFormat.NoSide, 0, 0, i, os);
                        pending.Add((((long)endPoly << 32) | (uint)seq++, link));
                    }
                }
            }
        }

        if (pending.Count == 0)
        {
            return new TileLinks([], new int[t.PolyCount + 1]);
        }
        // Group by source polygon, keeping generation order inside a polygon (keys are unique).
        var keys = new long[pending.Count];
        for (int i = 0; i < keys.Length; i++)
        {
            keys[i] = pending[i].Key;
        }
        var order = new int[keys.Length];
        for (int i = 0; i < order.Length; i++)
        {
            order[i] = i;
        }
        Array.Sort(keys, order);
        var links = new NavLink[keys.Length];
        var polyStart = new int[t.PolyCount + 1];
        for (int i = 0; i < keys.Length; i++)
        {
            links[i] = pending[order[i]].Link;
            polyStart[(int)(keys[i] >> 32) + 1]++;
        }
        for (int i = 0; i < t.PolyCount; i++)
        {
            polyStart[i + 1] += polyStart[i];
        }
        return new TileLinks(links, polyStart);
    }

    // Two opposite-side portal edges connect when they overlap along the boundary and their heights at both ends of
    // the overlap differ by at most the walkable climb. The test is symmetric, so links come in pairs.
    private bool MatchPortal(NavTile a, int ga, int pa, int side, NavTile b, int gb, out byte rmin, out byte rmax)
    {
        rmin = 0;
        rmax = 0;
        int axis = (side & 1) == 0 ? 2 : 0; // coordinate along the shared boundary
        ReadOnlySpan<float> va = a.Vertices;
        ReadOnlySpan<float> vb = b.Vertices;
        int na = a.PolyVertexCounts[pa];
        int fa = a.PolyStart[pa];
        int eaI = ga - fa;
        int a0 = a.PolyIndices[ga];
        int a1 = a.PolyIndices[fa + ((eaI + 1) % na)];
        int pb = PolyOfEdge(b, gb);
        int nb = b.PolyVertexCounts[pb];
        int fb = b.PolyStart[pb];
        int b0 = b.PolyIndices[gb];
        int b1 = b.PolyIndices[fb + (((gb - fb) + 1) % nb)];

        double ua0 = va[(3 * a0) + axis], ua1 = va[(3 * a1) + axis];
        double ub0 = vb[(3 * b0) + axis], ub1 = vb[(3 * b1) + axis];
        double lo = Math.Max(Math.Min(ua0, ua1), Math.Min(ub0, ub1));
        double hi = Math.Min(Math.Max(ua0, ua1), Math.Max(ub0, ub1));
        if (!(hi > lo))
        {
            return false;
        }
        double climb = Params.WalkableClimb;
        double ya0 = a.OriginY + va[(3 * a0) + 1], ya1 = a.OriginY + va[(3 * a1) + 1];
        double yb0 = b.OriginY + vb[(3 * b0) + 1], yb1 = b.OriginY + vb[(3 * b1) + 1];
        double ta0 = (lo - ua0) / (ua1 - ua0), ta1 = (hi - ua0) / (ua1 - ua0);
        double tb0 = (lo - ub0) / (ub1 - ub0), tb1 = (hi - ub0) / (ub1 - ub0);
        double dLo = Math.Abs((ya0 + ((ya1 - ya0) * ta0)) - (yb0 + ((yb1 - yb0) * tb0)));
        double dHi = Math.Abs((ya0 + ((ya1 - ya0) * ta1)) - (yb0 + ((yb1 - yb0) * tb1)));
        if (dLo > climb || dHi > climb)
        {
            return false;
        }
        double tmin = Math.Min(ta0, ta1);
        double tmax = Math.Max(ta0, ta1);
        rmin = (byte)Math.Clamp(Math.Floor(tmin * 255.0), 0, 255);
        rmax = (byte)Math.Clamp(Math.Ceiling(tmax * 255.0), 0, 255);
        return true;
    }

    private static int PolyOfEdge(NavTile t, int globalEdge)
    {
        // Binary search over the prefix sums: the last polygon whose start is <= globalEdge.
        int[] st = t.PolyStart;
        int lo = 0, hi = t.PolyCount - 1;
        while (lo < hi)
        {
            int mid = (lo + hi + 1) >> 1;
            if (st[mid] <= globalEdge)
            {
                lo = mid;
            }
            else
            {
                hi = mid - 1;
            }
        }
        return lo;
    }

    // Finds the polygons an off-mesh link joins: the start polygon in the owning tile, the end polygon in the tile
    // column under the end point (any layer). Unresolvable links simply stay unconnected.
    private bool ResolveEnds(int ownerSlot, int linkIndex, out int startPoly, out int endSlot, out int endPoly)
    {
        NavTile t = _tiles[ownerSlot]!;
        OffMeshLink l = t.OffMeshLinks[linkIndex];
        endSlot = -1;
        endPoly = -1;
        startPoly = -1;
        if (t.IsOffMeshDisabled(linkIndex))
        {
            return false;
        }
        double radius = l.Radius;
        if (!NearestPoly(t, t.OriginX + l.StartX, t.OriginY + l.StartY, t.OriginZ + l.StartZ, radius, out startPoly, out _))
        {
            return false;
        }
        double ex = t.OriginX + l.EndX, ey = t.OriginY + l.EndY, ez = t.OriginZ + l.EndZ;
        if (!(Math.Abs(ex) < NavMeshParams.MaxWorld && Math.Abs(ez) < NavMeshParams.MaxWorld && Math.Abs(ey) < NavMeshParams.MaxWorld))
        {
            return false;
        }
        Params.WorldToTile(ex, ez, out int tx, out int tz);
        if (!_columns.TryGetValue(ColumnKey(tx, tz), out var column))
        {
            return false;
        }
        double best = double.PositiveInfinity;
        foreach (int s in column)
        {
            if (NearestPoly(_tiles[s]!, ex, ey, ez, radius, out int poly, out double score) && score < best)
            {
                best = score;
                endSlot = s;
                endPoly = poly;
            }
        }
        return endSlot >= 0;
    }

    // Nearest polygon of a tile to a world point: horizontal distance to the polygon (0 inside) must be within
    // the radius and the height outside the polygon's vertex range within the climb. Score is the squared sum; ties go to the lower index.
    private bool NearestPoly(NavTile t, double wx, double wy, double wz, double radius, out int bestPoly, out double bestScore)
    {
        bestPoly = -1;
        bestScore = double.PositiveInfinity;
        double px = wx - t.OriginX, py = wy - t.OriginY, pz = wz - t.OriginZ;
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        double r2 = radius * radius;
        for (int p = 0; p < t.PolyCount; p++)
        {
            int n = t.PolyVertexCounts[p];
            int b = t.PolyStart[p];
            bool inside = true;
            double d2 = double.PositiveInfinity;
            double ymin = double.PositiveInfinity, ymax = double.NegativeInfinity;
            for (int e = 0; e < n; e++)
            {
                int i0 = idx[b + e], i1 = idx[b + ((e + 1) % n)];
                double x0 = v[3 * i0], z0 = v[(3 * i0) + 2], x1 = v[3 * i1], z1 = v[(3 * i1) + 2];
                double y0 = v[(3 * i0) + 1];
                ymin = Math.Min(ymin, y0);
                ymax = Math.Max(ymax, y0);
                double ex = x1 - x0, ez = z1 - z0;
                if ((ex * (pz - z0)) - (ez * (px - x0)) < 0)
                {
                    inside = false;
                }
                double len2 = (ex * ex) + (ez * ez);
                double s = len2 > 0 ? Math.Clamp((((px - x0) * ex) + ((pz - z0) * ez)) / len2, 0.0, 1.0) : 0.0;
                double dx = px - (x0 + (ex * s)), dz = pz - (z0 + (ez * s));
                d2 = Math.Min(d2, (dx * dx) + (dz * dz));
            }
            if (inside)
            {
                d2 = 0;
            }
            double dy = py < ymin ? ymin - py : py > ymax ? py - ymax : 0.0;
            if (d2 <= r2 && dy <= Params.WalkableClimb)
            {
                double score = d2 + (dy * dy);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestPoly = p;
                }
            }
        }
        return bestPoly >= 0;
    }

    private sealed class TileLinks
    {
        public TileLinks(NavLink[] links, int[] polyStart)
        {
            Links = links;
            PolyStart = polyStart;
        }

        public NavLink[] Links { get; }

        public int[] PolyStart { get; }
    }
}
