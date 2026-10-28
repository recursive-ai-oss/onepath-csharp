using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath;

public sealed partial class VoxelGrid
{
    private const int StateChunkVolume = ChunkVolume;

    /// <summary>
    /// Writes the grid's mutable state — which chunks are allocated and their voxels, area costs, <see cref="Version"/> and
    /// <see cref="Space"/> — to bytes for a host that rolls back. Little-endian, in the order documented in docs/grids-state.md.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.VoxelGrid);
        w.I32(SizeX);
        w.I32(SizeY);
        w.I32(SizeZ);
        w.U8(_fill);
        w.I32(Version);
        VoxelSpace s = Space;
        w.F64(s.OriginX);
        w.F64(s.OriginY);
        w.F64(s.OriginZ);
        w.F64(s.CellSize);
        GridStateIO.WriteAreas(w, Areas);
        for (int i = 0; i < _chunks.Length; i++)
        {
            byte[]? c = _chunks[i];
            if (c is null)
            {
                w.U8(0);
            }
            else
            {
                w.U8(1);
                w.Bytes(c);
            }
        }
        return w.ToArray();
    }

    /// <summary>Creates a grid from bytes written by <see cref="SaveState"/>.</summary>
    /// <param name="state">The bytes.</param>
    /// <returns>A grid equal to the saved one, with the same version.</returns>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or hold impossible values.</exception>
    public static VoxelGrid FromState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.VoxelGrid);
        int x = r.I32();
        int y = r.I32();
        int z = r.I32();
        byte fill = r.U8();
        if (x < 1 || y < 1 || z < 1 || (long)x * y * z > MaxVoxels)
        {
            throw r.Fail($"the size {x}x{y}x{z} is not a valid voxel grid size.");
        }
        var grid = new VoxelGrid(x, y, z, fill);
        grid.RestoreState(state);
        return grid;
    }

    /// <summary>
    /// Rolls the grid back to a state written by <see cref="SaveState"/> (same size and default area). Voxels, chunk allocation,
    /// area costs, <see cref="Version"/> and <see cref="Space"/> become exactly the saved ones; listeners see the voxels that
    /// differ as ordinary edits. On a bad state nothing is changed. Must not overlap a running query.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid, or hold impossible values.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.VoxelGrid);
        r.Expect(SizeX, "the X size");
        r.Expect(SizeY, "the Y size");
        r.Expect(SizeZ, "the Z size");
        byte fill = r.U8();
        if (fill != _fill)
        {
            throw r.Fail($"the default area is {fill} in the state but {_fill} in the grid being restored.");
        }
        int version = r.I32();
        if (version < 0)
        {
            throw r.Fail($"the grid version is {version}.");
        }
        double ox = r.F64();
        double oy = r.F64();
        double oz = r.F64();
        double size = r.F64();
        VoxelSpace space;
        if (size == 0.0 && ox == 0.0 && oy == 0.0 && oz == 0.0)
        {
            space = default;
        }
        else
        {
            try
            {
                space = new VoxelSpace(ox, oy, oz, size);
            }
            catch (ArgumentOutOfRangeException e)
            {
                throw r.Fail("the world mapping is out of range: " + e.Message);
            }
        }
        var multipliers = new double[AreaCosts.Count];
        int areasVersion = GridStateIO.ReadAreas(ref r, multipliers);
        var offsets = new int[_chunks.Length];
        for (int i = 0; i < offsets.Length; i++)
        {
            byte flag = r.U8();
            if (flag > 1)
            {
                throw r.Fail($"chunk {i} has flag {flag}; it must be 0 or 1.");
            }
            if (flag == 1)
            {
                offsets[i] = r.Position;
                _ = r.Bytes(StateChunkVolume);
            }
            else
            {
                offsets[i] = -1;
            }
        }
        r.End();

        // Everything validated: apply. Listeners are told of each voxel whose area differs.
        bool tell = _listeners.Count > 0;
        var changed = tell ? new List<int>() : null;
        int allocated = 0;
        for (int i = 0; i < offsets.Length; i++)
        {
            byte[]? old = _chunks[i];
            if (offsets[i] < 0)
            {
                if (tell && old is not null)
                {
                    for (int k = 0; k < StateChunkVolume; k++)
                    {
                        if (old[k] != _fill)
                        {
                            changed!.Add(ChunkVoxelNode(i, k));
                        }
                    }
                }
                _chunks[i] = null;
                continue;
            }
            ReadOnlySpan<byte> incoming = state.Slice(offsets[i], StateChunkVolume);
            if (tell)
            {
                for (int k = 0; k < StateChunkVolume; k++)
                {
                    byte before = old is null ? _fill : old[k];
                    if (before != incoming[k])
                    {
                        changed!.Add(ChunkVoxelNode(i, k));
                    }
                }
            }
            byte[] target = old ?? new byte[StateChunkVolume];
            incoming.CopyTo(target);
            _chunks[i] = target;
            allocated++;
        }
        _allocated = allocated;
        Version = version;
        Space = space;
        Areas.Restore(multipliers, areasVersion);
        if (changed is not null)
        {
            changed.RemoveAll(static n => n == int.MaxValue); // padding voxels past the grid edge
            changed.Sort(); // ascending node index: a fixed order, not the chunk order
            IVoxelChangeListener[] copy = _listeners.ToArray();
            foreach (int node in changed)
            {
                VoxelCell c = Decode(node, SizeX, SizeY);
                foreach (IVoxelChangeListener l in copy)
                {
                    l.OnVoxelChanged(c.X, c.Y, c.Z);
                }
            }
        }
    }

    // Node index of voxel k of chunk i, or int.MaxValue when the voxel lies outside the grid (the chunk's padding), which cannot be edited.
    private int ChunkVoxelNode(int chunk, int k)
    {
        int cx = chunk % ChunksX;
        int cy = (chunk / ChunksX) % ChunksY;
        int cz = chunk / (ChunksX * ChunksY);
        int x = (cx << Shift) | (k & Mask);
        int y = (cy << Shift) | ((k >> Shift) & Mask);
        int z = (cz << Shift) | ((k >> (2 * Shift)) & Mask);
        return x < SizeX && y < SizeY && z < SizeZ ? x + (SizeX * (y + (SizeY * z))) : int.MaxValue;
    }
}
