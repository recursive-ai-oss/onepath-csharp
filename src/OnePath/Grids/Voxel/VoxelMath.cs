using System;

namespace RecursiveAI.OnePath.Grids.Voxel;

/// <summary>Constants of the voxel grid code.</summary>
public static class VoxelMath
{
    /// <summary>
    /// √3 as the double nearest the true value (0x3FFBB67AE8584CAA, 1.7320508075688772): the length of a corner
    /// move. Correctly rounded square root gives the same bits; the literal avoids relying on it.
    /// </summary>
    public const double Sqrt3 = 1.7320508075688772;
}
