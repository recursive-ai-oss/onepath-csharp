using System;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// An off-mesh link slot: a one-way or two-way connection between two points that are not joined by the
/// polygon surface (a jump, a ladder, a door). Positions are tile-local float32 relative to the owning tile's origin.
/// The start must lie in the owning tile; the end may lie in any tile.
/// </summary>
public struct OffMeshLink : IEquatable<OffMeshLink>
{
    /// <summary>Start X, float32 world units from the owning tile's origin; must lie inside the tile.</summary>
    public float StartX { get; set; }

    /// <summary>Start height (Y is up), float32 world units above the tile's OriginY.</summary>
    public float StartY { get; set; }

    /// <summary>Start Z, float32 world units from the owning tile's origin; must lie inside the tile.</summary>
    public float StartZ { get; set; }

    /// <summary>End X (tile-local to the owning tile; may be outside it).</summary>
    public float EndX { get; set; }

    /// <summary>End height (Y is up), float32 world units above the owning tile's OriginY.</summary>
    public float EndY { get; set; }

    /// <summary>End Z (tile-local to the owning tile; may be outside it).</summary>
    public float EndZ { get; set; }

    /// <summary>How far, in world units, from a polygon an endpoint may be and still connect to it. Positive, finite and at most the mesh's local extent.</summary>
    public float Radius { get; set; }

    /// <summary>Area id of the link, 0 to 63.</summary>
    public byte Area { get; set; }

    /// <summary>Flags of the link, for query filters.</summary>
    public ushort PolyFlags { get; set; }

    /// <summary>True if the link can be crossed from the end to the start as well.</summary>
    public bool Bidirectional { get; set; }

    /// <summary>Caller-defined id, carried unchanged through serialization, snapshots and queries (0 is a valid id).</summary>
    public uint UserId { get; set; }

    /// <inheritdoc />
    public readonly bool Equals(OffMeshLink o) =>
        BitConverter.SingleToInt32Bits(StartX) == BitConverter.SingleToInt32Bits(o.StartX)
        && BitConverter.SingleToInt32Bits(StartY) == BitConverter.SingleToInt32Bits(o.StartY)
        && BitConverter.SingleToInt32Bits(StartZ) == BitConverter.SingleToInt32Bits(o.StartZ)
        && BitConverter.SingleToInt32Bits(EndX) == BitConverter.SingleToInt32Bits(o.EndX)
        && BitConverter.SingleToInt32Bits(EndY) == BitConverter.SingleToInt32Bits(o.EndY)
        && BitConverter.SingleToInt32Bits(EndZ) == BitConverter.SingleToInt32Bits(o.EndZ)
        && BitConverter.SingleToInt32Bits(Radius) == BitConverter.SingleToInt32Bits(o.Radius)
        && Area == o.Area && PolyFlags == o.PolyFlags && Bidirectional == o.Bidirectional && UserId == o.UserId;

    /// <inheritdoc />
    public override readonly bool Equals(object? obj) => obj is OffMeshLink o && Equals(o);

    /// <inheritdoc />
    public override readonly int GetHashCode() => (int)UserId;

    /// <summary>Bitwise equality.</summary>
    public static bool operator ==(OffMeshLink a, OffMeshLink b) => a.Equals(b);

    /// <summary>Bitwise inequality.</summary>
    public static bool operator !=(OffMeshLink a, OffMeshLink b) => !a.Equals(b);
}

/// <summary>Where a polygon's height-detail mesh sits in the tile's detail arrays. All zeros means "no detail" (use the polygon's own plane).</summary>
public struct DetailMesh : IEquatable<DetailMesh>
{
    /// <summary>First extra detail vertex of this polygon.</summary>
    public uint VertexBase { get; set; }

    /// <summary>First detail triangle of this polygon.</summary>
    public uint TriangleBase { get; set; }

    /// <summary>Number of extra detail vertices (in addition to the polygon's own vertices).</summary>
    public ushort VertexCount { get; set; }

    /// <summary>Number of detail triangles.</summary>
    public ushort TriangleCount { get; set; }

    /// <inheritdoc />
    public readonly bool Equals(DetailMesh o) =>
        VertexBase == o.VertexBase && TriangleBase == o.TriangleBase && VertexCount == o.VertexCount && TriangleCount == o.TriangleCount;

    /// <inheritdoc />
    public override readonly bool Equals(object? obj) => obj is DetailMesh o && Equals(o);

    /// <inheritdoc />
    public override readonly int GetHashCode() => (int)VertexBase;

    /// <summary>Equality.</summary>
    public static bool operator ==(DetailMesh a, DetailMesh b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(DetailMesh a, DetailMesh b) => !a.Equals(b);
}

/// <summary>The kind of a stitched <see cref="NavLink"/>.</summary>
public enum NavLinkKind : byte
{
    /// <summary>Across a tile boundary edge to a polygon of the neighbouring tile (any layer).</summary>
    Portal = 0,

    /// <summary>Through an off-mesh link from its start polygon to its end polygon.</summary>
    OffMesh = 1,

    /// <summary>Through a bidirectional off-mesh link from its end polygon back to its start polygon.</summary>
    OffMeshReverse = 2,
}

/// <summary>
/// A stitched connection that leaves a polygon other than through an in-tile shared edge. Links are derived from
/// the tile set (not serialized) and are the same for the same set of tiles.
/// </summary>
public readonly struct NavLink
{
    internal NavLink(PolyRef target, NavLinkKind kind, byte side, byte rangeMin, byte rangeMax, int index, int ownerSlot)
    {
        Target = target;
        Kind = kind;
        Side = side;
        RangeMin = rangeMin;
        RangeMax = rangeMax;
        Index = index;
        OwnerSlot = ownerSlot;
    }

    /// <summary>The polygon this link leads to.</summary>
    public PolyRef Target { get; }

    /// <summary>Portal, forward or reverse off-mesh.</summary>
    public NavLinkKind Kind { get; }

    /// <summary>For a portal: the tile side (0 = +X, 1 = +Z, 2 = -X, 3 = -Z) of the source edge; otherwise <see cref="NavMeshFormat.NoSide"/>.</summary>
    public byte Side { get; }

    /// <summary>For a portal: start of the shared part of the source edge, in 1/255ths of the edge from its first vertex.</summary>
    public byte RangeMin { get; }

    /// <summary>For a portal: end of the shared part, in 1/255ths of the edge from its first vertex.</summary>
    public byte RangeMax { get; }

    /// <summary>For a portal: the source edge index within the source polygon. For off-mesh: the link index within the owning tile.</summary>
    public int Index { get; }

    /// <summary>For off-mesh links: the slot of the tile that owns the link; otherwise -1.</summary>
    public int OwnerSlot { get; }
}
