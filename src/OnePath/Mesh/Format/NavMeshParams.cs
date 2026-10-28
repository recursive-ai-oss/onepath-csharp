using System;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// The grid and agent parameters of one mesh. A mesh serves one agent class (radius, height, climb, slope): meshes
/// for several agent sizes are several <see cref="TiledNavMesh"/> instances over the same tile grid.
/// <para>
/// Coordinate rule: world positions are double and must lie within ±2^21 (exact for a Q31.32 fixed-point host).
/// Tile coordinates are float32 relative to the tile's double origin. The tile size is at most
/// <see cref="MaxTileCells"/> cells so that float32 resolution at the largest local coordinate is finer than a
/// 1/<see cref="ResolutionMargin"/> of a cell.
/// </para>
/// </summary>
public sealed class NavMeshParams
{
    /// <summary>Largest tile size in cells.</summary>
    public const int MaxTileCells = 8192;

    /// <summary>Tile-local coordinates are bounded by this many cells; float32 resolution there is cell / <see cref="ResolutionMargin"/> or finer.</summary>
    public const int MaxLocalCells = 16384;

    /// <summary>Guaranteed ratio of cell size to the float32 spacing at the largest local coordinate.</summary>
    public const int ResolutionMargin = 512;

    /// <summary>Largest absolute world coordinate.</summary>
    public const double MaxWorld = 2097152.0;

    /// <summary>World X of tile column 0's origin. Default 0.</summary>
    public double OriginX { get; init; }

    /// <summary>World Y (up) the tile origins are measured from by default. Default 0.</summary>
    public double OriginY { get; init; }

    /// <summary>World Z of tile row 0's origin. Default 0.</summary>
    public double OriginZ { get; init; }

    /// <summary>Edge length of a tile along X and Z, in world units. Must be exactly representable as float32. Default 64.</summary>
    public double TileSize { get; init; } = 64;

    /// <summary>The builder's horizontal cell size; it sets the required precision. Default 0.3.</summary>
    public double CellSize { get; init; } = 0.3;

    /// <summary>Largest vertex count of a polygon, 3 to 16. Default 6.</summary>
    public int MaxVertsPerPoly { get; init; } = 6;

    /// <summary>Largest height difference two polygons across a tile boundary may have and still connect. Default 0.9.</summary>
    public float WalkableClimb { get; init; } = 0.9f;

    /// <summary>Radius of the agent class this mesh was built for. Default 0.6.</summary>
    public float AgentRadius { get; init; } = 0.6f;

    /// <summary>Height of the agent class. Default 2.</summary>
    public float AgentHeight { get; init; } = 2f;

    /// <summary>Steepest walkable slope of the agent class, in degrees. Default 45.</summary>
    public float AgentMaxSlopeDegrees { get; init; } = 45f;

    /// <summary>
    /// A 64-bit digest of the builder settings that change tile content but are not fields of this class (cell height, partition,
    /// region, contour and detail settings, authored links and so on). <see cref="NavSettings.ToParams"/> fills it; hand-built
    /// parameters leave it 0. It is part of <see cref="Fingerprint"/>.
    /// </summary>
    public ulong SettingsDigest { get; init; }

    private NavMeshFingerprint? _fingerprint;

    /// <summary>
    /// The fingerprint of these parameters: the first 16 bytes of SHA-256 over the ASCII tag <c>ONFP</c>, the format version (u16) and the
    /// parameter block exactly as the mesh header stores it. Identical on every platform.
    /// </summary>
    public NavMeshFingerprint Fingerprint => _fingerprint ??= ComputeFingerprint();

    private NavMeshFingerprint ComputeFingerprint()
    {
        var w = new NavMeshSerializer.Writer(96);
        w.U32(0x5046_4E4F); // "ONFP"
        w.U16(NavMeshFormat.Version);
        NavMeshSerializer.WriteParams(w, this);
        Span<byte> h = stackalloc byte[32];
        Sha256.Hash(w.ToArray(), h);
        return NavMeshFingerprint.From(h);
    }

    /// <summary>Largest absolute tile-local coordinate (world units).</summary>
    public double MaxLocalExtent => CellSize * MaxLocalCells;

    /// <summary>World X of the origin of tile column <paramref name="tileX"/>.</summary>
    public double TileOriginX(int tileX) => OriginX + (tileX * TileSize);

    /// <summary>World Z of the origin of tile row <paramref name="tileZ"/>.</summary>
    public double TileOriginZ(int tileZ) => OriginZ + (tileZ * TileSize);

    /// <summary>Checks every field.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is non-finite or outside its bound; the message names it.</exception>
    public void Validate()
    {
        CheckFinite(OriginX, nameof(OriginX));
        CheckFinite(OriginY, nameof(OriginY));
        CheckFinite(OriginZ, nameof(OriginZ));
        CheckWorld(OriginX, nameof(OriginX));
        CheckWorld(OriginY, nameof(OriginY));
        CheckWorld(OriginZ, nameof(OriginZ));
        if (!(TileSize > 0) || double.IsInfinity(TileSize) || (double)(float)TileSize != TileSize)
        {
            throw new ArgumentOutOfRangeException(nameof(TileSize), TileSize, "TileSize must be positive and exactly representable as float32.");
        }
        if (!(CellSize > 0) || double.IsInfinity(CellSize))
        {
            throw new ArgumentOutOfRangeException(nameof(CellSize), CellSize, "CellSize must be positive and finite.");
        }
        if (TileSize > CellSize * MaxTileCells)
        {
            throw new ArgumentOutOfRangeException(nameof(TileSize), TileSize,
                $"TileSize may be at most {MaxTileCells} cells ({CellSize * MaxTileCells}) so tile-local float32 resolution stays finer than a cell/{ResolutionMargin}.");
        }
        if (MaxVertsPerPoly < 3 || MaxVertsPerPoly > 16)
        {
            throw new ArgumentOutOfRangeException(nameof(MaxVertsPerPoly), MaxVertsPerPoly, "MaxVertsPerPoly must be 3 to 16.");
        }
        CheckNonNegative(WalkableClimb, nameof(WalkableClimb));
        CheckNonNegative(AgentRadius, nameof(AgentRadius));
        CheckNonNegative(AgentHeight, nameof(AgentHeight));
        if (!(AgentMaxSlopeDegrees >= 0 && AgentMaxSlopeDegrees <= 90))
        {
            throw new ArgumentOutOfRangeException(nameof(AgentMaxSlopeDegrees), AgentMaxSlopeDegrees, "AgentMaxSlopeDegrees must be 0 to 90.");
        }
    }

    /// <summary>
    /// Maps a world position to the tile column and row containing it.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is non-finite or beyond ±2^21.</exception>
    public void WorldToTile(double x, double z, out int tileX, out int tileZ)
    {
        CheckWorld(x, nameof(x));
        CheckWorld(z, nameof(z));
        double fx = Math.Floor((x - OriginX) / TileSize);
        double fz = Math.Floor((z - OriginZ) / TileSize);
        // Tiles are at least 1/8192 of the largest world extent apart in index terms; indices stay far inside int range.
        if (!(Math.Abs(fx) < 1e9 && Math.Abs(fz) < 1e9))
        {
            throw new ArgumentOutOfRangeException(nameof(x), $"Position ({x:R}, {z:R}) maps to a tile index beyond ±1e9; the tile size {TileSize:R} is too small for this world.");
        }
        tileX = (int)fx;
        tileZ = (int)fz;
    }

    internal static void CheckWorld(double v, string name)
    {
        if (!(v > -MaxWorld && v < MaxWorld))
        {
            throw new ArgumentOutOfRangeException(name, v, $"World coordinate {v:R} is outside the supported range of ±{MaxWorld:R} (2^21).");
        }
    }

    private static void CheckFinite(double v, string name)
    {
        if (!double.IsFinite(v))
        {
            throw new ArgumentOutOfRangeException(name, v, $"{name} must be finite.");
        }
    }

    private static void CheckNonNegative(float v, string name)
    {
        if (!(v >= 0) || float.IsInfinity(v))
        {
            throw new ArgumentOutOfRangeException(name, v, $"{name} must be finite and not negative.");
        }
    }

    internal bool SameAs(NavMeshParams o) =>
        BitConverter.DoubleToInt64Bits(OriginX) == BitConverter.DoubleToInt64Bits(o.OriginX)
        && BitConverter.DoubleToInt64Bits(OriginY) == BitConverter.DoubleToInt64Bits(o.OriginY)
        && BitConverter.DoubleToInt64Bits(OriginZ) == BitConverter.DoubleToInt64Bits(o.OriginZ)
        && BitConverter.DoubleToInt64Bits(TileSize) == BitConverter.DoubleToInt64Bits(o.TileSize)
        && BitConverter.DoubleToInt64Bits(CellSize) == BitConverter.DoubleToInt64Bits(o.CellSize)
        && MaxVertsPerPoly == o.MaxVertsPerPoly
        && BitConverter.SingleToInt32Bits(WalkableClimb) == BitConverter.SingleToInt32Bits(o.WalkableClimb)
        && BitConverter.SingleToInt32Bits(AgentRadius) == BitConverter.SingleToInt32Bits(o.AgentRadius)
        && BitConverter.SingleToInt32Bits(AgentHeight) == BitConverter.SingleToInt32Bits(o.AgentHeight)
        && BitConverter.SingleToInt32Bits(AgentMaxSlopeDegrees) == BitConverter.SingleToInt32Bits(o.AgentMaxSlopeDegrees)
        && SettingsDigest == o.SettingsDigest;
}
