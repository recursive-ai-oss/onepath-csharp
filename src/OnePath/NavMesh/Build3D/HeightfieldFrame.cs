using System;

namespace RecursiveAI.OnePath.NavMesh.Build3D;

/// <summary>
/// The voxel grid a tile is built on: a double world origin, the cell size (x and z) and cell height (y)
/// as float32, and the grid dimensions in cells. Everything inside the build is expressed relative to
/// <see cref="OriginX"/>, <see cref="OriginY"/>, <see cref="OriginZ"/> in float32, so a tile can sit anywhere
/// within the public coordinate range and still resolve a cell to better than 1/256 of its size.
/// </summary>
public sealed class HeightfieldFrame
{
    /// <summary>World coordinates are accepted within plus or minus this bound (2^21), the range a Q31.32 fixed-point value converts to double exactly.</summary>
    public const double MaxCoordinate = 2097152.0;

    /// <summary>Smallest accepted cell size or height (2^-20). Keeps cell-unit coordinates far from float32 overflow.</summary>
    public const float MinCell = 1f / 1048576f;

    /// <summary>Largest accepted cell size or height (2^20).</summary>
    public const float MaxCell = 1048576f;

    /// <summary>Largest grid size in x or z, in cells. Keeps tile-local float32 coordinates within 2^16 cells, where a float resolves a cell to 1/256.</summary>
    public const int MaxSide = 1 << 16;

    /// <summary>Largest number of columns (width times depth).</summary>
    public const int MaxColumns = 1 << 24;

    /// <summary>Largest grid height in cells.</summary>
    public const int MaxHeight = 65535;

    /// <summary>World x of the grid's minimum corner.</summary>
    public double OriginX { get; }

    /// <summary>World y of the grid's minimum corner.</summary>
    public double OriginY { get; }

    /// <summary>World z of the grid's minimum corner.</summary>
    public double OriginZ { get; }

    /// <summary>Cell size along x and z, in world units, within [<see cref="MinCell"/>, <see cref="MaxCell"/>].</summary>
    public float CellSize { get; }

    /// <summary>Cell height along y, in world units, within [<see cref="MinCell"/>, <see cref="MaxCell"/>].</summary>
    public float CellHeight { get; }

    /// <summary>Number of cells along x (at least 1).</summary>
    public int Width { get; }

    /// <summary>Number of cells along z (at least 1).</summary>
    public int Depth { get; }

    /// <summary>Number of cells along y (at least 1).</summary>
    public int Height { get; }

    /// <summary>Creates a frame from explicit dimensions.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The origin is beyond ±2^21, a dimension is outside its limit, or the cell size or height is not positive and finite.</exception>
    public HeightfieldFrame(double originX, double originY, double originZ, float cellSize, float cellHeight, int width, int depth, int height)
    {
        CheckCoordinate(originX, nameof(originX));
        CheckCoordinate(originY, nameof(originY));
        CheckCoordinate(originZ, nameof(originZ));
        if (!(cellSize >= MinCell && cellSize <= MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, $"The cell size must be in [{MinCell}, {MaxCell}].");
        }
        if (!(cellHeight >= MinCell && cellHeight <= MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(cellHeight), cellHeight, $"The cell height must be in [{MinCell}, {MaxCell}].");
        }
        if (width < 1 || width > MaxSide)
        {
            throw new ArgumentOutOfRangeException(nameof(width), width, $"The width must be in 1..{MaxSide}.");
        }
        if (depth < 1 || depth > MaxSide)
        {
            throw new ArgumentOutOfRangeException(nameof(depth), depth, $"The depth must be in 1..{MaxSide}.");
        }
        if (height < 1 || height > MaxHeight)
        {
            throw new ArgumentOutOfRangeException(nameof(height), height, $"The height must be in 1..{MaxHeight}.");
        }
        if ((long)width * depth > MaxColumns)
        {
            throw new ArgumentOutOfRangeException(nameof(width), width, $"Width times depth must not exceed {MaxColumns} columns.");
        }
        OriginX = originX;
        OriginY = originY;
        OriginZ = originZ;
        CellSize = cellSize;
        CellHeight = cellHeight;
        Width = width;
        Depth = depth;
        Height = height;
    }

    /// <summary>
    /// Creates the frame covering an axis-aligned box: the origin is the minimum corner and each dimension is
    /// the box extent divided by the cell size, rounded up.
    /// </summary>
    /// <exception cref="ArgumentException">A bound is NaN or infinite, or a maximum is not above its minimum.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A bound is beyond ±2^21, or the resulting grid exceeds the limits.</exception>
    public static HeightfieldFrame FromBounds(double minX, double minY, double minZ, double maxX, double maxY, double maxZ, float cellSize, float cellHeight)
    {
        CheckCoordinate(minX, nameof(minX));
        CheckCoordinate(minY, nameof(minY));
        CheckCoordinate(minZ, nameof(minZ));
        CheckCoordinate(maxX, nameof(maxX));
        CheckCoordinate(maxY, nameof(maxY));
        CheckCoordinate(maxZ, nameof(maxZ));
        if (!(maxX > minX) || !(maxY > minY) || !(maxZ > minZ))
        {
            throw new ArgumentException($"The bounds are empty or inverted: min ({minX}, {minY}, {minZ}), max ({maxX}, {maxY}, {maxZ}).");
        }
        if (!(cellSize >= MinCell && cellSize <= MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, $"The cell size must be in [{MinCell}, {MaxCell}].");
        }
        if (!(cellHeight >= MinCell && cellHeight <= MaxCell))
        {
            throw new ArgumentOutOfRangeException(nameof(cellHeight), cellHeight, $"The cell height must be in [{MinCell}, {MaxCell}].");
        }
        double w = Math.Ceiling((maxX - minX) / cellSize);
        double d = Math.Ceiling((maxZ - minZ) / cellSize);
        double h = Math.Ceiling((maxY - minY) / cellHeight);
        if (w > MaxSide || d > MaxSide || h > MaxHeight)
        {
            throw new ArgumentOutOfRangeException(nameof(maxX), $"The bounds need {w} x {d} x {h} cells; the limits are {MaxSide} x {MaxSide} x {MaxHeight}. Use a larger cell size or a smaller tile.");
        }
        return new HeightfieldFrame(minX, minY, minZ, cellSize, cellHeight, (int)w, (int)d, (int)h);
    }

    /// <summary>Number of columns, <see cref="Width"/> times <see cref="Depth"/>.</summary>
    public int ColumnCount => Width * Depth;

    /// <summary>Column index of cell (x, z): <c>x + z * Width</c>.</summary>
    public int Column(int x, int z) => x + (z * Width);

    /// <summary>World y of the bottom of voxel row <paramref name="y"/>.</summary>
    public double WorldY(int y) => OriginY + (y * (double)CellHeight);

    /// <summary>World x of the centre of cell column <paramref name="x"/>.</summary>
    public double CenterX(int x) => OriginX + ((x + 0.5) * CellSize);

    /// <summary>World z of the centre of cell row <paramref name="z"/>.</summary>
    public double CenterZ(int z) => OriginZ + ((z + 0.5) * CellSize);

    /// <summary>True when both frames describe the same grid.</summary>
    public bool SameGrid(HeightfieldFrame other)
    {
        ArgumentNullException.ThrowIfNull(other);
        return OriginX == other.OriginX && OriginY == other.OriginY && OriginZ == other.OriginZ
            && CellSize == other.CellSize && CellHeight == other.CellHeight
            && Width == other.Width && Depth == other.Depth && Height == other.Height;
    }

    internal static void CheckCoordinate(double value, string name)
    {
        if (double.IsNaN(value))
        {
            throw new ArgumentException($"{name} is NaN.", name);
        }
        if (!(Math.Abs(value) <= MaxCoordinate))
        {
            throw new ArgumentOutOfRangeException(name, value, $"The value {value} is outside the supported coordinate range of ±{MaxCoordinate} (2^21).");
        }
    }
}
