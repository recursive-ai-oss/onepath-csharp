using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>Fuzz targets for the voxel grid code: the Moving AI voxel parsers and the query inputs. A target throws only when it finds a real problem.</summary>
internal static class VoxelTargets
{
    internal static void MapParser(byte[] data)
    {
        try
        {
            VoxelMap map = VoxelMap.Parse(data);
            if (map.SizeX < 1 || map.SizeY < 1 || map.SizeZ < 1 || (long)map.SizeX * map.SizeY * map.SizeZ > VoxelGrid.MaxVoxels)
            {
                throw new InvalidOperationException("parsed map has an impossible size");
            }
            // Building the grid of a large declared size is fine: chunks are allocated only for listed voxels.
            VoxelGrid g = map.ToGrid();
            if (g.SizeX != map.SizeX || g.SizeY != map.SizeY || g.SizeZ != map.SizeZ)
            {
                throw new InvalidOperationException("grid size differs from map size");
            }
        }
        catch (GridFormatException)
        {
        }
    }

    internal static void ScenarioParser(byte[] data)
    {
        try
        {
            foreach (VoxelScenarioEntry e in VoxelScenarios.Parse(data))
            {
                if (!double.IsFinite(e.OptimalLength) || !double.IsFinite(e.Ratio) || e.StartX < 0 || e.GoalZ < 0)
                {
                    throw new InvalidOperationException("scenario entry holds an impossible value");
                }
            }
        }
        catch (GridFormatException)
        {
        }
    }

    // A small random block, options and endpoints decoded from the bytes. Checked: A* and Dijkstra agree, found
    // paths are legal steps that sum to the reported cost, out-of-range endpoints are reported, not thrown, and an
    // edit followed by a query equals a query on a grid rebuilt from scratch.
    internal static void Query(byte[] data)
    {
        if (data.Length < 16)
        {
            return;
        }
        int sx = 1 + (data[0] % 10);
        int sy = 1 + (data[1] % 10);
        int sz = 1 + (data[2] % 10);
        byte flags = data[3];
        var options = new VoxelOptions
        {
            Connectivity = (flags & 3) switch { 0 => VoxelConnectivity.Six, 1 => VoxelConnectivity.Eighteen, _ => VoxelConnectivity.TwentySix },
            Corners = (VoxelCornerRule)((flags >> 2) % 3),
        };
        var start = new VoxelCell((data[4] % (sx + 2)) - 1, (data[5] % (sy + 2)) - 1, (data[6] % (sz + 2)) - 1);
        var goal = new VoxelCell((data[7] % (sx + 2)) - 1, (data[8] % (sy + 2)) - 1, (data[9] % (sz + 2)) - 1);
        var g = new VoxelGrid(sx, sy, sz);
        g.Areas[1] = 0.5 + (data[10] % 8);
        int i = 0;
        for (int z = 0; z < sz; z++)
        {
            for (int y = 0; y < sy; y++)
            {
                for (int x = 0; x < sx; x++)
                {
                    byte b = data[11 + (i++ % (data.Length - 11))];
                    g.SetArea(x, y, z, (b & 7) switch { 0 or 1 => AreaCosts.Wall, 2 => 1, _ => 0 });
                }
            }
        }

        VoxelPath fast = g.FindPath(start, goal, options);
        var dij = new VoxelOptions { Connectivity = options.Connectivity, Corners = options.Corners, Heuristic = GridHeuristic.Zero };
        VoxelPath slow = g.FindPath(start, goal, dij);
        if (fast.Status != slow.Status)
        {
            throw new InvalidOperationException($"A* says {fast.Status}, Dijkstra says {slow.Status}");
        }
        if (!fast.Found)
        {
            return;
        }
        if (Math.Abs(fast.Cost - slow.Cost) > 1e-9 * Math.Max(1.0, slow.Cost))
        {
            throw new InvalidOperationException($"A* cost {fast.Cost} differs from Dijkstra {slow.Cost}");
        }
        ReadOnlySpan<VoxelCell> cells = fast.Cells;
        if (cells[0] != start || cells[^1] != goal)
        {
            throw new InvalidOperationException("path does not run from start to goal");
        }
        int kinds = (int)options.Connectivity == 6 ? 1 : (int)options.Connectivity == 18 ? 2 : 3;
        double sum = 0;
        for (int k = 1; k < cells.Length; k++)
        {
            VoxelCell a = cells[k - 1];
            VoxelCell c = cells[k];
            int dx = Math.Abs(c.X - a.X);
            int dy = Math.Abs(c.Y - a.Y);
            int dz = Math.Abs(c.Z - a.Z);
            int n = dx + dy + dz;
            if (dx > 1 || dy > 1 || dz > 1 || n < 1 || n > kinds || !g.IsWalkable(c.X, c.Y, c.Z))
            {
                throw new InvalidOperationException("path takes an illegal step");
            }
            double len = n == 1 ? 1.0 : n == 2 ? GridMath.Sqrt2 : VoxelMath.Sqrt3;
            sum += len * ((g.Areas[g.GetArea(a.X, a.Y, a.Z)] + g.Areas[g.GetArea(c.X, c.Y, c.Z)]) * 0.5);
        }
        if (Math.Abs(sum - fast.Cost) > 1e-9 * Math.Max(1.0, sum))
        {
            throw new InvalidOperationException("path cost does not match its steps");
        }

        // Edit one voxel on the path, then compare with a grid built directly.
        VoxelCell e = cells[cells.Length / 2];
        g.SetWalkable(e.X, e.Y, e.Z, false);
        var fresh = new VoxelGrid(sx, sy, sz);
        fresh.Areas.CopyFrom(g.Areas);
        for (int z = 0; z < sz; z++)
        {
            for (int y = 0; y < sy; y++)
            {
                for (int x = 0; x < sx; x++)
                {
                    fresh.SetArea(x, y, z, g.GetArea(x, y, z));
                }
            }
        }
        VoxelPath p1 = g.FindPath(start, goal, options);
        VoxelPath p2 = fresh.FindPath(start, goal, options);
        if (p1.Status != p2.Status || p1.Cost != p2.Cost || !p1.Cells.SequenceEqual(p2.Cells))
        {
            throw new InvalidOperationException("edited grid differs from a rebuilt grid");
        }
    }
}
