using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

public sealed partial class AreaCosts
{
    internal void Restore(double[] multipliers, int version)
    {
        Array.Copy(multipliers, _multipliers, Count);
        _multipliers[Wall] = double.PositiveInfinity;
        double min = double.PositiveInfinity;
        for (int i = 0; i < Count; i++)
        {
            if (_multipliers[i] < min)
            {
                min = _multipliers[i];
            }
        }
        _min = min;
        Version = version;
    }
}

public sealed partial class Grid
{
    /// <summary>
    /// Writes the grid's mutable state — cells, area costs, <see cref="Version"/> and <see cref="Space"/> — to bytes for a host
    /// that rolls back. Little-endian, in the order documented in docs/grids-state.md; equal states give equal bytes on every platform.
    /// Derived data (clearance, jump point indexes, HPA* hierarchies) is not stored: it is rebuilt after a restore.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.Grid);
        w.I32(Width);
        w.I32(Height);
        w.I32(Version);
        GridStateIO.WriteSpace(w, Space);
        GridStateIO.WriteAreas(w, Areas);
        w.Bytes(_cells);
        return w.ToArray();
    }

    /// <summary>
    /// Creates a grid from bytes written by <see cref="SaveState"/>.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <returns>A grid equal to the saved one, with the same version.</returns>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or hold impossible values.</exception>
    public static Grid FromState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.Grid);
        int w = r.I32();
        int h = r.I32();
        if (w < 1 || h < 1 || (long)w * h > MaxCells)
        {
            throw r.Fail($"the size {w}x{h} is not a valid grid size.");
        }
        var grid = new Grid(w, h);
        grid.RestoreState(state);
        return grid;
    }

    /// <summary>
    /// Rolls the grid back to a state written by <see cref="SaveState"/>. The state must come from a grid of the same size.
    /// Cells, area costs, <see cref="Version"/> and <see cref="Space"/> become exactly the saved ones and every later query
    /// returns what a grid that was never changed would return. On a bad state nothing is changed.
    /// <para>
    /// Objects built on the grid are told: flow fields, replanners and HPA* hierarchies rebuild on next use, other listeners see the
    /// cells that differ as ordinary edits. To roll those back too, restore the grid first and then their own saved states.
    /// Must not overlap a running query.
    /// </para>
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid size, or hold impossible values.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.Grid);
        r.Expect(Width, "the width");
        r.Expect(Height, "the height");
        int version = r.I32();
        if (version < 0)
        {
            throw r.Fail($"the grid version is {version}.");
        }
        GridSpace space = GridStateIO.ReadSpace(ref r);
        var multipliers = new double[AreaCosts.Count];
        int areasVersion = GridStateIO.ReadAreas(ref r, multipliers);
        ReadOnlySpan<byte> cells = r.Bytes(_cells.Length);
        r.End();

        lock (_sync)
        {
            GridStateIO.Diff(_cells, cells, Width, _changedScratch, out int bx0, out int by0, out int bx1, out int by1);
            Version = version;
            _hashValid = false;
            Space = space;
            Areas.Restore(multipliers, areasVersion);
            // Derived caches are rebuilt on demand: versions can repeat after a rollback, so none of them may be trusted.
            // The clearance array is shared with live topologies, so it is recomputed in place now (when it had been computed at all).
            bool hadClearance = _clearanceVersion >= 0;
            _clearanceVersion = -1;
            _dirty.Clear();
            if (hadClearance)
            {
                ComputeClearance();
                _clearanceVersion = Version;
            }
            _jpsPlus = null;
            DropOctileCaches();
            RestoreCount++;
            _blockMap?.Dispose();
            _blockMap = null;
            foreach (HpaHierarchy h in _hpa)
            {
                h.Dispose();
            }
            _hpa.Clear();
            GridStateIO.NotifyRestored(_listeners, _changedScratch, Width, bx0, by0, bx1, by1);
        }
    }
}

public sealed partial class HexGrid
{
    /// <summary>
    /// Writes the grid's mutable state — cells, area costs, <see cref="Version"/> and <see cref="Space"/> — to bytes for a host
    /// that rolls back; see <see cref="Grid.SaveState"/> and docs/grids-state.md.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.HexGrid);
        w.I32(Width);
        w.I32(Height);
        w.I32((int)Layout);
        w.I32(Version);
        GridStateIO.WriteSpace(w, Space);
        GridStateIO.WriteAreas(w, Areas);
        w.Bytes(_cells);
        return w.ToArray();
    }

    /// <summary>Creates a hex grid from bytes written by <see cref="SaveState"/>.</summary>
    /// <param name="state">The bytes.</param>
    /// <returns>A grid equal to the saved one, with the same version.</returns>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or hold impossible values.</exception>
    public static HexGrid FromState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.HexGrid);
        int w = r.I32();
        int h = r.I32();
        int layout = r.I32();
        if (w < 1 || h < 1 || (long)w * h > Grid.MaxCells)
        {
            throw r.Fail($"the size {w}x{h} is not a valid grid size.");
        }
        if (layout is < (int)HexLayout.PointyOddRow or > (int)HexLayout.FlatEvenColumn)
        {
            throw r.Fail($"hex layout {layout} is not defined.");
        }
        var grid = new HexGrid(w, h, (HexLayout)layout);
        grid.RestoreState(state);
        return grid;
    }

    /// <summary>
    /// Rolls the grid back to a state written by <see cref="SaveState"/> (same size and layout); see <see cref="Grid.RestoreState"/>.
    /// On a bad state nothing is changed.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid, or hold impossible values.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.HexGrid);
        r.Expect(Width, "the width");
        r.Expect(Height, "the height");
        r.Expect((int)Layout, "the layout");
        int version = r.I32();
        if (version < 0)
        {
            throw r.Fail($"the grid version is {version}.");
        }
        GridSpace space = GridStateIO.ReadSpace(ref r);
        var multipliers = new double[AreaCosts.Count];
        int areasVersion = GridStateIO.ReadAreas(ref r, multipliers);
        ReadOnlySpan<byte> cells = r.Bytes(_cells.Length);
        r.End();

        GridStateIO.Diff(_cells, cells, Width, _changedScratch, out int bx0, out int by0, out int bx1, out int by1);
        Version = version;
        Space = space;
        Areas.Restore(multipliers, areasVersion);
        GridStateIO.NotifyRestored(_listeners, _changedScratch, Width, bx0, by0, bx1, by1);
    }
}
