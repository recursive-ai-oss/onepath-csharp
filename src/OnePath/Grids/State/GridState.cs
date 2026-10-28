using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Implemented by objects that keep derived state of a grid and cannot repair it from an edit list when the grid is
/// rolled back with <c>RestoreState</c>: they drop what they derived and rebuild on next use.
/// </summary>
internal interface IGridRestoreListener
{
    void OnGridRestored();
}

// Shared pieces of the grid snapshots (docs/grids-state.md).
internal static class GridStateIO
{
    internal static void WriteSpace(StateWriter w, GridSpace s)
    {
        w.F64(s.OriginX);
        w.F64(s.OriginY);
        w.F64(s.CellSize);
    }

    internal static GridSpace ReadSpace(ref StateReader r)
    {
        double x = r.F64();
        double y = r.F64();
        double size = r.F64();
        if (size == 0.0 && x == 0.0 && y == 0.0)
        {
            return default; // the all-zero value a host may have assigned; it behaves as unit cells
        }
        try
        {
            return new GridSpace(x, y, size);
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw r.Fail("the world mapping is out of range: " + e.Message);
        }
    }

    internal static void WriteAreas(StateWriter w, AreaCosts areas)
    {
        w.I32(areas.Version);
        double[] m = areas.Multipliers;
        for (int i = 0; i < AreaCosts.Count; i++)
        {
            w.F64(m[i]);
        }
    }

    internal static int ReadAreas(ref StateReader r, double[] into)
    {
        int version = r.I32();
        if (version < 0)
        {
            throw r.Fail($"the area table version is {version}.");
        }
        for (int i = 0; i < AreaCosts.Count; i++)
        {
            double v = r.F64();
            if (!(v > 0.0) || double.IsPositiveInfinity(v))
            {
                throw r.Fail(string.Create(CultureInfo.InvariantCulture, $"area {i} has multiplier {v:R}; it must be positive and finite."));
            }
            into[i] = v;
        }
        return version;
    }

    // Reports a rollback: objects that cannot repair themselves are told to rebuild, everything else sees the cells that
    // differ as ordinary edits.
    internal static void NotifyRestored(List<IGridChangeListener> listeners, List<int> changed, int width, int bx0, int by0, int bx1, int by1)
    {
        IGridChangeListener[] copy = listeners.ToArray(); // a listener may unsubscribe while being told
        foreach (IGridChangeListener l in copy)
        {
            if (l is IGridRestoreListener rl)
            {
                rl.OnGridRestored();
            }
            else if (changed.Count == 0)
            {
                continue;
            }
            else if (l is IGridBatchListener batch)
            {
                batch.OnRectChanged(bx0, by0, bx1 - bx0 + 1, by1 - by0 + 1);
            }
            else
            {
                for (int k = 0; k < changed.Count; k++)
                {
                    int idx = changed[k];
                    int cy = idx / width;
                    l.OnCellChanged(idx - (cy * width), cy);
                }
            }
        }
    }

    // Copies the new cells in and lists the indices that differ, with their bounding box.
    internal static void Diff(byte[] cells, ReadOnlySpan<byte> incoming, int width, List<int> changed, out int bx0, out int by0, out int bx1, out int by1)
    {
        changed.Clear();
        bx0 = int.MaxValue;
        by0 = int.MaxValue;
        bx1 = -1;
        by1 = -1;
        for (int i = 0; i < cells.Length; i++)
        {
            if (cells[i] != incoming[i])
            {
                changed.Add(i);
                int y = i / width;
                int x = i - (y * width);
                bx0 = Math.Min(bx0, x);
                bx1 = Math.Max(bx1, x);
                by0 = Math.Min(by0, y);
                by1 = Math.Max(by1, y);
            }
        }
        incoming.CopyTo(cells);
    }
}
