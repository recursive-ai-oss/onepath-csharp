using System;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath;

/// <summary>
/// The tuning of a navigation mesh build: one agent class and the tiling. Every field has a default; the presets
/// (<see cref="Humanoid"/>, <see cref="Small"/>, <see cref="Large"/>) are starting points, and a record <c>with</c>
/// expression changes any field. Lengths are world units; the world must stay within ±2^21.
/// </summary>
public sealed record NavSettings
{
    /// <summary>A person: radius 0.6, height 2, climb 0.9, slope 45°, cell 0.3, 64-unit tiles.</summary>
    public static NavSettings Humanoid { get; } = new();

    /// <summary>A small creature: radius 0.3, height 1, climb 0.4, slope 45°, cell 0.15, 32-unit tiles.</summary>
    public static NavSettings Small { get; } = new() { AgentRadius = 0.3f, AgentHeight = 1f, WalkableClimb = 0.4f, CellSize = 0.15, TileSize = 32 };

    /// <summary>A large creature or vehicle: radius 1.5, height 4, climb 1.2, slope 40°, cell 0.5, 128-unit tiles.</summary>
    public static NavSettings Large { get; } = new() { AgentRadius = 1.5f, AgentHeight = 4f, WalkableClimb = 1.2f, AgentMaxSlopeDegrees = 40f, CellSize = 0.5, TileSize = 128 };

    /// <summary>Agent radius: the walkable area is shrunk by this much. Default 0.6.</summary>
    public float AgentRadius { get; init; } = 0.6f;

    /// <summary>Agent height. Default 2.</summary>
    public float AgentHeight { get; init; } = 2f;

    /// <summary>Largest step up or down an agent climbs. Default 0.9.</summary>
    public float WalkableClimb { get; init; } = 0.9f;

    /// <summary>Steepest walkable slope in degrees, 0 to 90. Default 45.</summary>
    public float AgentMaxSlopeDegrees { get; init; } = 45f;

    /// <summary>Horizontal precision unit; tiles keep float32 resolution finer than a 512th of it. Default 0.3.</summary>
    public double CellSize { get; init; } = 0.3;

    /// <summary>Tile edge length, exactly representable as float32 and at most 8192 cells. Default 64.</summary>
    public double TileSize { get; init; } = 64;

    /// <summary>World X of tile column 0's origin. Default 0.</summary>
    public double OriginX { get; init; }

    /// <summary>World Z of tile row 0's origin. Default 0.</summary>
    public double OriginZ { get; init; }

    /// <summary>Largest vertex count of a polygon, 3 to 16. Default 6.</summary>
    public int MaxVertsPerPoly { get; init; } = 6;

    /// <summary>World Y of the flat floor a 2D build produces. Default 0.</summary>
    public double Height { get; init; }

    /// <summary>Area id given to every polygon of a 2D build, 0 to 63. Default 0.</summary>
    public byte Area { get; init; }

    /// <summary>Flags given to every polygon of a 2D build. Default 1 (walk).</summary>
    public ushort Flags { get; init; } = 1;

    /// <summary>
    /// The 2D builder's snap grid exponent (grid unit 2^-n), 0 to 16; null (the default) picks the finest grid the cell
    /// size allows, so features thinner than cell/256 are dropped and every remaining vertex survives as float32.
    /// </summary>
    public int? SnapBits { get; init; }

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
            WalkableClimb = WalkableClimb,
            AgentRadius = AgentRadius,
            AgentHeight = AgentHeight,
            AgentMaxSlopeDegrees = AgentMaxSlopeDegrees,
        };
        p.Validate();
        if (!(Area <= NavMeshFormat.MaxArea))
        {
            throw new ArgumentOutOfRangeException(nameof(Area), Area, "Area must be 0 to 63.");
        }
        return p;
    }
}
