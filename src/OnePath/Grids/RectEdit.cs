using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// A listener that wants one call for a whole rectangle of edits (a tile) instead of one call per cell.
/// Grids call <see cref="OnRectChanged"/> in place of <see cref="IGridChangeListener.OnCellChanged"/> for listeners that implement this.
/// </summary>
public interface IGridBatchListener : IGridChangeListener
{
    /// <summary>
    /// Called once after a rectangle edit, with the smallest rectangle containing every cell that changed.
    /// Cells inside it may be unchanged. The grid already holds the final state.
    /// </summary>
    /// <param name="x">Left column of the rectangle.</param>
    /// <param name="y">Top row of the rectangle.</param>
    /// <param name="width">Columns, at least 1.</param>
    /// <param name="height">Rows, at least 1.</param>
    void OnRectChanged(int x, int y, int width, int height);
}

// Shared by Grid and HexGrid: validates a rectangle, writes it, and reports what changed.
internal static class RectEdit
{
    /// <summary>
    /// Applies <paramref name="fill"/> (or the row-major <paramref name="source"/> when its length is not zero) to the rectangle.
    /// Returns false when no cell changed. <paramref name="changed"/> receives the indices of the changed cells in row-major order.
    /// </summary>
    internal static bool Apply(byte[] cells, int gridWidth, int gridHeight, int x, int y, int width, int height, byte fill, ReadOnlySpan<byte> source, bool useSource, List<int> changed, out int bx0, out int by0, out int bx1, out int by1)
    {
        if (x < 0 || y < 0 || width < 0 || height < 0 || (long)x + width > gridWidth || (long)y + height > gridHeight)
        {
            throw new ArgumentOutOfRangeException(nameof(x), $"The rectangle at ({x}, {y}) of {width}x{height} cells does not lie inside the {gridWidth}x{gridHeight} grid.");
        }
        if (useSource && source.Length != width * height)
        {
            throw new ArgumentException($"Expected {width * height} cells for a {width}x{height} rectangle but got {source.Length}.", nameof(source));
        }
        changed.Clear();
        bx0 = int.MaxValue;
        by0 = int.MaxValue;
        bx1 = -1;
        by1 = -1;
        for (int j = 0; j < height; j++)
        {
            int row = (y + j) * gridWidth;
            for (int i = 0; i < width; i++)
            {
                byte v = useSource ? source[(j * width) + i] : fill;
                int idx = row + x + i;
                if (cells[idx] == v)
                {
                    continue;
                }
                cells[idx] = v;
                changed.Add(idx);
                bx0 = Math.Min(bx0, x + i);
                bx1 = Math.Max(bx1, x + i);
                by0 = Math.Min(by0, y + j);
                by1 = Math.Max(by1, y + j);
            }
        }
        return changed.Count > 0;
    }

    internal static void Notify(List<IGridChangeListener> listeners, List<int> changed, int gridWidth, int bx0, int by0, int bx1, int by1)
    {
        for (int l = 0; l < listeners.Count; l++)
        {
            IGridChangeListener listener = listeners[l];
            if (listener is IGridBatchListener batch)
            {
                batch.OnRectChanged(bx0, by0, bx1 - bx0 + 1, by1 - by0 + 1);
                continue;
            }
            for (int k = 0; k < changed.Count; k++)
            {
                int idx = changed[k];
                int cy = idx / gridWidth;
                listener.OnCellChanged(idx - (cy * gridWidth), cy);
            }
        }
    }
}
