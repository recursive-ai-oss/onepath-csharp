using System;

namespace RecursiveAI.OnePath;

/// <summary>A voxel of a <see cref="VoxelGrid"/>: zero-based integer coordinates along x, y and z.</summary>
/// <param name="X">Position along x (0 at the low edge).</param>
/// <param name="Y">Position along y.</param>
/// <param name="Z">Position along z.</param>
public readonly record struct VoxelCell(int X, int Y, int Z);

/// <summary>A position in world space, in double precision. Coordinates must lie within ±2^21 (see <see cref="GridSpace"/>).</summary>
/// <param name="X">World x.</param>
/// <param name="Y">World y.</param>
/// <param name="Z">World z.</param>
public readonly record struct VoxelPoint(double X, double Y, double Z);

/// <summary>
/// Maps voxel coordinates to world space: voxel (i, j, k) occupies the cube starting at the origin plus
/// <c>(i, j, k) * CellSize</c>. The origin and cell size are doubles within ±2^21 so every conversion
/// is exact for a deterministic host's fixed-point values.
/// </summary>
public readonly struct VoxelSpace : IEquatable<VoxelSpace>
{
    /// <summary>Creates a mapping.</summary>
    /// <param name="originX">World x of the grid's low corner.</param>
    /// <param name="originY">World y of the grid's low corner.</param>
    /// <param name="originZ">World z of the grid's low corner.</param>
    /// <param name="cellSize">Edge length of a voxel; positive, at most 2^21.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is NaN or beyond ±2^21, or the cell size is not positive.</exception>
    public VoxelSpace(double originX, double originY, double originZ, double cellSize)
    {
        GridSpace.CheckCoordinate(originX, nameof(originX));
        GridSpace.CheckCoordinate(originY, nameof(originY));
        GridSpace.CheckCoordinate(originZ, nameof(originZ));
        if (!(cellSize > 0.0) || cellSize > GridSpace.CoordinateLimit)
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "The cell size must be positive and at most 2^21 (2097152).");
        }
        OriginX = originX;
        OriginY = originY;
        OriginZ = originZ;
        CellSize = cellSize;
    }

    /// <summary>Unit voxels with the origin at (0, 0, 0).</summary>
    public static VoxelSpace Unit => new(0.0, 0.0, 0.0, 1.0);

    /// <summary>World x of the low corner.</summary>
    public double OriginX { get; }

    /// <summary>World y of the low corner.</summary>
    public double OriginY { get; }

    /// <summary>World z of the low corner.</summary>
    public double OriginZ { get; }

    /// <summary>Voxel edge length. Zero only for <c>default</c>, which behaves as <see cref="Unit"/>.</summary>
    public double CellSize { get; }

    internal double SizeOrOne => CellSize > 0.0 ? CellSize : 1.0;

    /// <inheritdoc/>
    public bool Equals(VoxelSpace other) => OriginX == other.OriginX && OriginY == other.OriginY && OriginZ == other.OriginZ && CellSize == other.CellSize;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is VoxelSpace s && Equals(s);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        long h = BitConverter.DoubleToInt64Bits(OriginX) ^ (BitConverter.DoubleToInt64Bits(OriginY) * 31) ^ (BitConverter.DoubleToInt64Bits(OriginZ) * 17) ^ (BitConverter.DoubleToInt64Bits(CellSize) * 13);
        return (int)h ^ (int)(h >> 32);
    }

    /// <summary>Equality.</summary>
    public static bool operator ==(VoxelSpace left, VoxelSpace right) => left.Equals(right);

    /// <summary>Inequality.</summary>
    public static bool operator !=(VoxelSpace left, VoxelSpace right) => !left.Equals(right);
}
