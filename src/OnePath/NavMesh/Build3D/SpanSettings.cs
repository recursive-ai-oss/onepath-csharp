using System;

namespace RecursiveAI.OnePath.NavMesh.Build3D;

/// <summary>
/// Agent-dependent parameters of the span stage. All lengths are in cells (voxels): convert world sizes
/// by dividing by the frame's cell size (radius) or cell height (height, climb) and rounding as you see fit.
/// </summary>
public sealed class SpanSettings
{
    /// <summary>The area id the voxelizer gives to walkable surfaces (63). Area 0 means not walkable.</summary>
    public const byte WalkableArea = 63;

    /// <summary>
    /// Minimum y component of a unit surface normal for a triangle to count as walkable: the cosine of the
    /// steepest walkable slope. Default 0.70710677f (45 degrees). Taken as a cosine so the span stage needs no
    /// transcendental function. A value of 1 or more accepts nothing; a value of 0 or less accepts walls.
    /// </summary>
    public float WalkableSlopeCosine { get; set; } = 0.70710677f;

    /// <summary>Clearance an agent needs above a floor, in voxel rows (default 10: 2.0 units at a cell height of 0.2). At least 1.</summary>
    public int WalkableHeight { get; set; } = 10;

    /// <summary>Largest step an agent climbs, in voxel rows (default 4). 0 or more.</summary>
    public int WalkableClimb { get; set; } = 4;

    /// <summary>Agent radius in cells (default 2). Walkable spans closer than this to an edge are removed by erosion. 0 disables erosion.</summary>
    public int WalkableRadius { get; set; } = 2;

    /// <summary>
    /// When a triangle's span merges with an existing span whose top differs by at most this many rows, the
    /// merged span keeps the larger area id, so a walkable surface is not lost under a thin obstacle. Defaults to
    /// -1, meaning "use <see cref="WalkableClimb"/>".
    /// </summary>
    public int MergeThreshold { get; set; } = -1;

    /// <summary>The merge threshold in effect.</summary>
    public int EffectiveMergeThreshold => MergeThreshold >= 0 ? MergeThreshold : WalkableClimb;

    /// <summary>A person-sized agent at the default cell size 0.3 and cell height 0.2: radius 0.6, height 2.0, climb 0.9, slope 45 degrees.</summary>
    public static SpanSettings Humanoid => new();

    /// <summary>Checks the values.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A length is negative, the height is below 1, or the slope cosine is NaN.</exception>
    public void Validate()
    {
        if (WalkableHeight < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(WalkableHeight), WalkableHeight, "The walkable height must be at least 1 row.");
        }
        if (WalkableClimb < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(WalkableClimb), WalkableClimb, "The walkable climb must not be negative.");
        }
        if (WalkableRadius < 0 || WalkableRadius > 1 << 20)
        {
            throw new ArgumentOutOfRangeException(nameof(WalkableRadius), WalkableRadius, "The walkable radius must be in 0..2^20 cells.");
        }
        if (float.IsNaN(WalkableSlopeCosine))
        {
            throw new ArgumentOutOfRangeException(nameof(WalkableSlopeCosine), WalkableSlopeCosine, "The slope cosine must not be NaN.");
        }
    }
}
