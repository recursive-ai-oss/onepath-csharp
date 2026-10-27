using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

/// <summary>Told when a cell of a grid changes; incremental algorithms (replanning, caches) subscribe to this.</summary>
public interface IGridChangeListener
{
    /// <summary>Called after the cell (<paramref name="x"/>, <paramref name="y"/>) changed area (including becoming or ceasing to be a wall).</summary>
    /// <param name="x">Column of the cell.</param>
    /// <param name="y">Row of the cell.</param>
    void OnCellChanged(int x, int y);
}
