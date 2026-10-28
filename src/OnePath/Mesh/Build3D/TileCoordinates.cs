using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// Maps positions on a tile's build grid to tile-local float32 coordinates. The grid holds the tile's cells plus a border
/// of <see cref="Border"/> cells on every side; tile-local zero is the first cell after the border and the tile size is the
/// <see cref="TileCells"/>th cell corner. The two tile boundaries map to exactly 0 and exactly the float32 tile size so that
/// neighbouring tiles meet bit for bit; everything else is one double multiplication rounded once to float32.
/// </summary>
public sealed class TileCoordinates
{
    /// <summary>Creates the mapping.</summary>
    /// <param name="border">Border cells on each side of the tile.</param>
    /// <param name="tileCells">Cells along one side of the tile (at least 1).</param>
    /// <param name="cellSize">World size of a cell: the tile size divided by <paramref name="tileCells"/>.</param>
    /// <param name="tileSize">The tile size, exactly representable as float32.</param>
    /// <param name="cellHeight">World height of a row.</param>
    /// <param name="minRow">Row that is tile-local Y zero.</param>
    public TileCoordinates(int border, int tileCells, double cellSize, double tileSize, double cellHeight, int minRow)
    {
        Border = border;
        TileCells = tileCells;
        CellSize = cellSize;
        TileSize = (float)tileSize;
        CellHeight = cellHeight;
        MinRow = minRow;
        // Grid coordinates run from 0 to the cells across the grid (the tile's cells and a border on both sides).
        long span = tileCells + (2L * border);
        SafeCross = ((span * span) >> 19) + 1;
    }

    /// <summary>
    /// A turn of at least this many cell areas (twice the signed triangle area in cells) has the same sign at the float32 positions of its three corners:
    /// each position is off by at most 2^-24 of the grid's extent W, which moves the determinant by less than 2^-21 W^2, and this is that bound plus a factor of four.
    /// </summary>
    internal long SafeCross { get; }

    /// <summary>Border cells on each side.</summary>
    public int Border { get; }

    /// <summary>Cells along one side of the tile.</summary>
    public int TileCells { get; }

    /// <summary>World size of a cell.</summary>
    public double CellSize { get; }

    /// <summary>The tile size as float32.</summary>
    public float TileSize { get; }

    /// <summary>World height of a row.</summary>
    public double CellHeight { get; }

    /// <summary>The row that is tile-local Y zero.</summary>
    public int MinRow { get; }

    /// <summary>Tile-local X (or Z) of a grid coordinate in cells.</summary>
    public float Local(double cell)
    {
        double c = cell - Border;
        if (c == 0)
        {
            return 0f;
        }
        return c == TileCells ? TileSize : (float)(c * CellSize);
    }

    /// <summary>Tile-local Y of a row.</summary>
    public float LocalY(double row) => (float)((row - MinRow) * CellHeight);
}
