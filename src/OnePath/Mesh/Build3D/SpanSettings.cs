using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// Agent-dependent parameters of the span stage. All lengths are in cells (voxels): convert world sizes
/// by dividing by the frame's cell size (radius) or cell height (height, climb) and rounding as you see fit.
/// </summary>
public sealed class SpanSettings
{
    /// <summary>The area id the voxelizer gives to walkable surfaces (63). Area 0 means not walkable.</summary>
    public const byte WalkableArea = 63;

    private float _slopeCosine = 0.70710677f;
    private float _slopeDegrees = 45f;

    /// <summary>
    /// Minimum y component of a unit surface normal for a triangle to count as walkable: the cosine of the
    /// steepest walkable slope. Default 0.70710677f (45 degrees). A value of 1 or more accepts nothing; a value of 0
    /// or less accepts walls. This is the value the span stage uses; <see cref="WalkableSlopeDegrees"/> is a
    /// convenient way to set it. Whichever of the two was set last wins, and setting the cosine makes
    /// <see cref="WalkableSlopeDegrees"/> read NaN, since no angle was given.
    /// </summary>
    public float WalkableSlopeCosine
    {
        get => _slopeCosine;
        set
        {
            _slopeCosine = value;
            _slopeDegrees = float.NaN;
        }
    }

    /// <summary>
    /// The steepest walkable slope in degrees, 0 to 90 (default 45). Setting it stores the correctly rounded cosine of
    /// the angle (computed with OneMath, so every platform gets the same bits) in <see cref="WalkableSlopeCosine"/>;
    /// 90 gives exactly 0 and 45 gives exactly 0.70710677f. Reads NaN after <see cref="WalkableSlopeCosine"/> was set directly.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is NaN, infinite or outside 0 to 90.</exception>
    public float WalkableSlopeDegrees
    {
        get => _slopeDegrees;
        set
        {
            if (!(value >= 0f && value <= 90f))
            {
                throw new ArgumentOutOfRangeException(nameof(WalkableSlopeDegrees), value, "The walkable slope must be 0 to 90 degrees.");
            }
            _slopeCosine = value == 90f ? 0f : (float)Math64.Cos(value * (Math.PI / 180.0));
            _slopeDegrees = value;
        }
    }

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
