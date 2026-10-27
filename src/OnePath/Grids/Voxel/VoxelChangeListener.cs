using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Grids.Voxel;

/// <summary>Told when a voxel of a <see cref="VoxelGrid"/> changes; caches and incremental searches subscribe to this.</summary>
public interface IVoxelChangeListener
{
    /// <summary>Called after voxel (<paramref name="x"/>, <paramref name="y"/>, <paramref name="z"/>) changed area (including becoming or ceasing to be solid).</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    void OnVoxelChanged(int x, int y, int z);
}
