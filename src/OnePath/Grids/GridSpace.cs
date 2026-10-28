using System;
using System.Globalization;

namespace RecursiveAI.OnePath;

/// <summary>
/// Maps cells to world space: cell (x, y) of a square grid has its centre at
/// <c>origin + ((x + 0.5) * cellSize, (y + 0.5) * cellSize)</c>. World coordinates cross the public
/// boundary as double and must lie within ±2^21 (exact for a Q31.32 fixed-point host);
/// anything beyond, or NaN, is rejected with <see cref="ArgumentOutOfRangeException"/>.
/// </summary>
public readonly struct GridSpace : IEquatable<GridSpace>
{
    /// <summary>The largest absolute world coordinate accepted: 2^21 = 2,097,152.</summary>
    public const double CoordinateLimit = 2097152.0;

    /// <summary>Creates a mapping.</summary>
    /// <param name="originX">World x of the grid's left edge.</param>
    /// <param name="originY">World y of the grid's top edge.</param>
    /// <param name="cellSize">Edge length of a cell (centre spacing on hex grids); positive and finite.</param>
    /// <exception cref="ArgumentOutOfRangeException">A value is beyond ±2^21, NaN, or the cell size is not positive.</exception>
    public GridSpace(double originX, double originY, double cellSize)
    {
        CheckCoordinate(originX, nameof(originX));
        CheckCoordinate(originY, nameof(originY));
        if (!(cellSize > 0.0) || cellSize > CoordinateLimit)
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "The cell size must be positive and at most 2^21 (2097152).");
        }
        OriginX = originX;
        OriginY = originY;
        CellSize = cellSize;
    }

    /// <summary>Unit cells with the origin at (0, 0).</summary>
    public static GridSpace Unit => new(0.0, 0.0, 1.0);

    /// <summary>World x of the grid's left edge.</summary>
    public double OriginX { get; }

    /// <summary>World y of the grid's top edge.</summary>
    public double OriginY { get; }

    /// <summary>Cell edge length (centre spacing on hex grids). Zero only for <c>default</c>, which behaves as <see cref="Unit"/>.</summary>
    public double CellSize { get; }

    internal double SizeOrOne => CellSize > 0.0 ? CellSize : 1.0;

    // Throws unless the value is a number within ±2^21 (the coordinate rule).
    internal static void CheckCoordinate(double value, string paramName)
    {
        if (!(value >= -CoordinateLimit && value <= CoordinateLimit))
        {
            throw new ArgumentOutOfRangeException(
                paramName,
                value,
                string.Create(CultureInfo.InvariantCulture, $"World coordinate {value:R} is outside the supported range of ±2^21 (±2097152)."));
        }
    }

    /// <inheritdoc/>
    public bool Equals(GridSpace other) => OriginX == other.OriginX && OriginY == other.OriginY && CellSize == other.CellSize;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is GridSpace g && Equals(g);

    /// <inheritdoc/>
    public override int GetHashCode()
    {
        long h = BitConverter.DoubleToInt64Bits(OriginX) ^ (BitConverter.DoubleToInt64Bits(OriginY) * 31) ^ (BitConverter.DoubleToInt64Bits(CellSize) * 17);
        return (int)h ^ (int)(h >> 32);
    }

    /// <summary>Equality.</summary>
    public static bool operator ==(GridSpace left, GridSpace right) => left.Equals(right);

    /// <summary>Inequality.</summary>
    public static bool operator !=(GridSpace left, GridSpace right) => !left.Equals(right);
}
