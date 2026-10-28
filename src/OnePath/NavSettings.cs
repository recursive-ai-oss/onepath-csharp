using System;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath;

/// <summary>
/// The tuning of a navigation mesh build: one agent class and the tiling. Every field has a default; the presets
/// (<see cref="Humanoid"/>, <see cref="Small"/>, <see cref="Large"/>) are starting points, and a record <c>with</c>
/// expression changes any field. Lengths are world units; the world must stay within ±2^21. Every quantity is a <c>double</c>; the agent
/// quantities (radius, height, climb, slope, contour error) are rounded to float32 once when a build starts, which is the precision
/// the mesh format stores them in. Each value is checked when it is set (<see cref="ArgumentOutOfRangeException"/> naming the property
/// and its range); constraints between values (tile size against cell size) are checked when the settings are used.
/// The 2D builder reads <see cref="FloorY"/>, <see cref="Area"/>, <see cref="Flags"/> and <see cref="SnapBits"/>; the 3D builder reads the partition, region,
/// contour and detail values and <see cref="CellHeight"/>.
/// </summary>
public sealed record NavSettings
{
    /// <summary>A person: radius 0.6, height 2, climb 0.9, slope 45°, cell 0.3, 64-unit tiles.</summary>
    public static NavSettings Humanoid { get; } = new();

    /// <summary>A small creature: radius 0.3, height 1, climb 0.4, slope 45°, cell 0.15, 32-unit tiles.</summary>
    public static NavSettings Small { get; } = new() { AgentRadius = 0.3, AgentHeight = 1, WalkableClimb = 0.4, CellSize = 0.15, CellHeight = 0.1, TileSize = 32, DetailSampleDistance = 0.9, DetailMaxError = 0.1 };

    /// <summary>A large creature or vehicle: radius 1.5, height 4, climb 1.2, slope 40°, cell 0.5, 128-unit tiles.</summary>
    public static NavSettings Large { get; } = new() { AgentRadius = 1.5, AgentHeight = 4, WalkableClimb = 1.2, AgentMaxSlopeDegrees = 40, CellSize = 0.5, CellHeight = 0.25, TileSize = 128, DetailSampleDistance = 3, DetailMaxError = 0.25 };

    /// <summary>Agent radius: the walkable area is shrunk by this much. Default 0.6.</summary>
    public double AgentRadius { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(AgentRadius), "0 or more and finite"); } = 0.6;

    /// <summary>Agent height. Default 2.</summary>
    public double AgentHeight { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(AgentHeight), "0 or more and finite"); } = 2;

    /// <summary>Largest step up or down an agent climbs. Default 0.9.</summary>
    public double WalkableClimb { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(WalkableClimb), "0 or more and finite"); } = 0.9;

    /// <summary>Steepest walkable slope in degrees, 0 to 90. Default 45.</summary>
    public double AgentMaxSlopeDegrees { get; init => field = Require.That(value >= 0 && value <= 90, value, nameof(AgentMaxSlopeDegrees), "0 to 90"); } = 45;

    /// <summary>Horizontal precision unit; tiles keep float32 resolution finer than a 512th of it. Default 0.3.</summary>
    public double CellSize { get; init => field = Require.That(value > 0 && double.IsFinite(value), value, nameof(CellSize), "above 0 and finite"); } = 0.3;

    /// <summary>Tile edge length, exactly representable as float32 and at most 8192 cells. Default 64.</summary>
    public double TileSize { get; init => field = Require.That(value > 0 && double.IsFinite(value) && (double)(float)value == value, value, nameof(TileSize), "above 0, finite and exactly representable as float32"); } = 64;

    /// <summary>Height of one voxel row in the 3D builder. Default 0.2.</summary>
    public double CellHeight { get; init => field = Require.That(value > 0 && double.IsFinite(value), value, nameof(CellHeight), "above 0 and finite"); } = 0.2;

    /// <summary>World X of tile column 0's origin. Default 0.</summary>
    public double OriginX { get; init => field = Require.That(double.IsFinite(value), value, nameof(OriginX), "finite"); }

    /// <summary>World Z of tile row 0's origin. Default 0.</summary>
    public double OriginZ { get; init => field = Require.That(double.IsFinite(value), value, nameof(OriginZ), "finite"); }

    /// <summary>
    /// Authored off-mesh links (jumps, ladders, doors) baked into every tile the build produces, 2D and 3D alike, and into rebuilt tiles. Default none.
    /// The list is copied when set, so later changes to the array you passed do not show through.
    /// See <see cref="OffMeshLinkDefinition"/>.
    /// </summary>
    public System.Collections.Generic.IReadOnlyList<OffMeshLinkDefinition> OffMeshLinks { get; init => field = FrozenList<OffMeshLinkDefinition>.Of(value); } = FrozenList<OffMeshLinkDefinition>.Empty;

    /// <summary>Largest vertex count of a polygon, 3 to 16. Default 6.</summary>
    public int MaxVertsPerPoly { get; init => field = Require.That(value is >= 3 and <= 16, value, nameof(MaxVertsPerPoly), "3 to 16"); } = 6;

    /// <summary>2D builder: world Y of the flat floor the mesh lies on (not the agent's height, which is <see cref="AgentHeight"/>). Default 0.</summary>
    public double FloorY { get; init => field = Require.That(double.IsFinite(value), value, nameof(FloorY), "finite"); }

    /// <summary>Area id given to every polygon of a 2D build, 0 to 63. Default 0.</summary>
    public byte Area { get; init => field = Require.That(value <= 63, value, nameof(Area), "0 to 63"); }

    /// <summary>Flags given to every polygon of a 2D build. Default 1 (walk).</summary>
    public ushort Flags { get; init; } = 1;

    /// <summary>
    /// The 2D builder's snap grid exponent (grid unit 2^-n), 0 to 16; null (the default) picks the finest grid the cell
    /// size allows, so features thinner than cell/256 are dropped and every remaining vertex survives as float32.
    /// </summary>
    public int? SnapBits { get; init; }

    /// <summary>How the 3D builder splits walkable surfaces into regions. Default watershed; <see cref="PartitionType.Layers"/> also splits overlapping floors into separate tile layers.</summary>
    public PartitionType Partition { get; init; } = PartitionType.Watershed;

    /// <summary>3D builder: regions with fewer cells than this that touch no other region are removed. Default 64. 0 disables.</summary>
    public int MinRegionArea { get; init => field = Require.That(value >= 0, value, nameof(MinRegionArea), "0 or more"); } = 64;

    /// <summary>3D builder: regions with fewer cells than this merge into a neighbour. Default 400. 0 disables.</summary>
    public int MergeRegionArea { get; init => field = Require.That(value >= 0, value, nameof(MergeRegionArea), "0 or more"); } = 400;

    /// <summary>3D builder: largest distance, in cells, the traced outline of a region may stray from the simplified one. Default 1.3.</summary>
    public double ContourMaxError { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(ContourMaxError), "0 or more and finite"); } = 1.3;

    /// <summary>3D builder: wall edges longer than this (world units) are split. Default 3.6. 0 disables.</summary>
    public double MaxEdgeLength { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(MaxEdgeLength), "0 or more and finite"); } = 3.6;

    /// <summary>3D builder: spacing of height samples for the detail mesh (world units); 0 turns the height detail off. Default 1.8.</summary>
    public double DetailSampleDistance { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(DetailSampleDistance), "0 or more and finite"); } = 1.8;

    /// <summary>3D builder: largest vertical distance between the heightfield and the detail surface before a vertex is added (world units). Default 0.2.</summary>
    public double DetailMaxError { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(DetailMaxError), "0 or more and finite"); } = 0.2;

    // Digest of the settings that change tile content and are not mesh parameters, so tiles built with other values are
    // refused by a mesh. Canonical encoding (little-endian): tag "ONSD", f64 CellHeight, i32 SnapBits (-1 = automatic),
    // i32 Partition, i32 MinRegionArea, i32 MergeRegionArea, f32 ContourMaxError, f64 MaxEdgeLength, f64 DetailSampleDistance,
    // f64 DetailMaxError, f64 FloorY, u8 Area, u16 Flags, u32 link count, then per link f64 start xyz, end xyz, radius,
    // u8 bidirectional, u8 area, u16 flags, u32 user id. The digest is the first 8 bytes of SHA-256, read little-endian.
    private ulong SettingsDigest()
    {
        var w = new NavMeshSerializer.Writer(128);
        w.U32(0x4453_4E4F);
        w.F64(CellHeight);
        w.I32(SnapBits ?? -1);
        w.I32((int)Partition);
        w.I32(MinRegionArea);
        w.I32(MergeRegionArea);
        w.F32((float)ContourMaxError);
        w.F64(MaxEdgeLength);
        w.F64(DetailSampleDistance);
        w.F64(DetailMaxError);
        w.F64(FloorY);
        w.U8(Area);
        w.U16(Flags);
        System.Collections.Generic.IReadOnlyList<OffMeshLinkDefinition> links = OffMeshLinks ?? System.Array.Empty<OffMeshLinkDefinition>();
        w.U32((uint)links.Count);
        for (int i = 0; i < links.Count; i++)
        {
            OffMeshLinkDefinition l = links[i];
            w.F64(l.Start.X);
            w.F64(l.Start.Y);
            w.F64(l.Start.Z);
            w.F64(l.End.X);
            w.F64(l.End.Y);
            w.F64(l.End.Z);
            w.F64(l.Radius);
            w.U8(l.Bidirectional ? (byte)1 : (byte)0);
            w.U8(l.Area);
            w.U16(l.Flags);
            w.U32(l.UserId);
        }
        System.Span<byte> h = stackalloc byte[32];
        Sha256.Hash(w.ToArray(), h);
        return System.Buffers.Binary.BinaryPrimitives.ReadUInt64LittleEndian(h);
    }

    /// <summary>The mesh parameters these settings describe.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range; the message names it.</exception>
    public NavMeshParams ToParams()
    {
        var p = new NavMeshParams
        {
            OriginX = OriginX,
            OriginZ = OriginZ,
            TileSize = TileSize,
            CellSize = CellSize,
            MaxVertsPerPoly = MaxVertsPerPoly,
            WalkableClimb = (float)WalkableClimb,
            AgentRadius = (float)AgentRadius,
            AgentHeight = (float)AgentHeight,
            AgentMaxSlopeDegrees = (float)AgentMaxSlopeDegrees,
            SettingsDigest = SettingsDigest(),
        };
        p.Validate();
        if (!(Area <= NavMeshFormat.MaxArea))
        {
            throw new ArgumentOutOfRangeException(nameof(Area), Area, "Area must be 0 to 63.");
        }
        return p;
    }
}
