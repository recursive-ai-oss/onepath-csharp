using System;
using System.Globalization;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

public sealed partial class GridReplanner : IGridRestoreListener
{
    /// <summary>
    /// Writes the replanner's search state — start, goal, g and rhs values, the queue, km, pending edits and counters — to bytes for a
    /// host that rolls back. Little-endian, in the order documented in docs/grids-state.md. Restoring it into a replanner
    /// built over an equal grid with the same options makes every later <see cref="Replan()"/> return what the original would.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.GridReplanner);
        w.I32(_grid.Width);
        w.I32(_grid.Height);
        w.U8((byte)_algorithm);
        w.I32(_areaVersion);
        w.I32(_search.Start);
        w.I32(_search.Goal);
        _search.WriteState(w);
        return w.ToArray();
    }

    /// <summary>
    /// Restores a state written by <see cref="SaveState"/>. Restore the grid first (<see cref="Grid.RestoreState"/>), then this;
    /// the replanner must have been created with the same algorithm and options (agent size, connectivity, hooks) as the saved one.
    /// On a bad state the replanner is unchanged.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid or algorithm, or hold impossible values.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        var r = new StateReader(state, StateKind.GridReplanner);
        r.Expect(_grid.Width, "the width");
        r.Expect(_grid.Height, "the height");
        byte algorithm = r.U8();
        if (algorithm != (byte)_algorithm)
        {
            throw r.Fail(string.Create(CultureInfo.InvariantCulture, $"the state is for algorithm {algorithm} but this replanner runs {(byte)_algorithm} ({_algorithm})."));
        }
        int areaVersion = r.I32();
        int count = _grid.CellCount;
        int start = r.Node(count, "start");
        int goal = r.Node(count, "goal");
        int previousAreaVersion = _areaVersion;
        try
        {
            // Built over the grid as it is now, so the heuristic scale and costs match.
            ReplanSearch search = Build(SearchRunner.ToCell(start, _grid.Width), SearchRunner.ToCell(goal, _grid.Width));
            search.ReadState(ref r);
            r.End();
            _search = search;
            _areaVersion = areaVersion;
        }
        catch
        {
            _areaVersion = previousAreaVersion;
            throw;
        }
    }

    void IGridRestoreListener.OnGridRestored()
    {
        _search.Restart();
        _areaVersion = -1; // never equals a real version: the next Replan rebuilds the search over the restored area table
    }
}

public sealed partial class HexReplanner : IGridRestoreListener
{
    /// <summary>
    /// Writes the replanner's search state to bytes for a host that rolls back; see <see cref="GridReplanner.SaveState"/>.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.HexReplanner);
        w.I32(_grid.Width);
        w.I32(_grid.Height);
        w.U8((byte)_algorithm);
        w.I32(_areaVersion);
        w.I32(_search.Start);
        w.I32(_search.Goal);
        _search.WriteState(w);
        return w.ToArray();
    }

    /// <summary>Restores a state written by <see cref="SaveState"/>; see <see cref="GridReplanner.RestoreState"/>.</summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid or algorithm, or hold impossible values.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        var r = new StateReader(state, StateKind.HexReplanner);
        r.Expect(_grid.Width, "the width");
        r.Expect(_grid.Height, "the height");
        byte algorithm = r.U8();
        if (algorithm != (byte)_algorithm)
        {
            throw r.Fail(string.Create(CultureInfo.InvariantCulture, $"the state is for algorithm {algorithm} but this replanner runs {(byte)_algorithm} ({_algorithm})."));
        }
        int areaVersion = r.I32();
        int count = _grid.CellCount;
        int start = r.Node(count, "start");
        int goal = r.Node(count, "goal");
        int previousAreaVersion = _areaVersion;
        try
        {
            ReplanSearch search = Build(SearchRunner.ToCell(start, _grid.Width), SearchRunner.ToCell(goal, _grid.Width));
            search.ReadState(ref r);
            r.End();
            _search = search;
            _areaVersion = areaVersion;
        }
        catch
        {
            _areaVersion = previousAreaVersion;
            throw;
        }
    }

    void IGridRestoreListener.OnGridRestored()
    {
        _search.Restart();
        _areaVersion = -1;
    }
}
