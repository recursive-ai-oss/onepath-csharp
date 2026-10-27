using System;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>Constants of the tiled navigation mesh format (see <c>docs/navmesh-format.md</c>).</summary>
public static class NavMeshFormat
{
    /// <summary>The format version this build reads and writes.</summary>
    public const ushort Version = 1;

    /// <summary>
    /// True while the format may still change incompatibly. Files written in that state carry
    /// <see cref="FlagUnstable"/>; a stable release of the format clears the flag and bumps <see cref="Version"/>,
    /// so unstable files are never mistaken for stable ones.
    /// </summary>
    public const bool IsUnstable = true;

    /// <summary>Header flag bit 0: the file was written by a draft (unstable) format.</summary>
    public const ushort FlagUnstable = 1;

    /// <summary>Largest polygon area id.</summary>
    public const int MaxArea = 63;

    /// <summary>Largest number of tile slots in one mesh (20 bits of a polygon reference).</summary>
    public const int MaxTileSlots = (1 << 20) - 1;

    /// <summary>Largest number of polygons in one tile (20 bits of a polygon reference).</summary>
    public const int MaxPolygonsPerTile = (1 << 20) - 1;

    /// <summary>Largest layer index.</summary>
    public const int MaxLayer = 1023;

    /// <summary>Edge side code meaning "not a tile portal".</summary>
    public const byte NoSide = 0xFF;

    /// <summary>Tile sides: 0 = +X, 1 = +Z, 2 = -X, 3 = -Z.</summary>
    public const int SideCount = 4;

    /// <summary>The side facing <paramref name="side"/> across a tile boundary.</summary>
    public static int OppositeSide(int side) => (side + 2) & 3;

    /// <summary>Tile index step for a side: +1, 0, -1, 0 along X.</summary>
    internal static int SideDx(int side) => side == 0 ? 1 : side == 2 ? -1 : 0;

    /// <summary>Tile index step for a side along Z.</summary>
    internal static int SideDz(int side) => side == 1 ? 1 : side == 3 ? -1 : 0;
}

/// <summary>
/// Thrown when mesh or tile data is malformed: a validation failure of hand-built data, or a byte stream that
/// is truncated, inconsistent or hostile. The message names what was wrong and where.
/// </summary>
public sealed class NavMeshFormatException : Exception
{
    /// <summary>Creates the exception with a message.</summary>
    public NavMeshFormatException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and an inner exception.</summary>
    public NavMeshFormatException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    /// <summary>Creates an exception with a default message.</summary>
    public NavMeshFormatException()
        : base("The navigation mesh data is malformed.")
    {
    }
}
