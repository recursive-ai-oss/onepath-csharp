using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath;

public sealed partial class GridFlowField : IGridRestoreListener
{
    /// <summary>
    /// Writes the flow field's state — costs, directions, the open list of an unfinished (sliced) run or repair, goals and
    /// pending edits — to bytes for a host that rolls back. Little-endian, in the order documented in docs/grids-state.md.
    /// Restoring it into a field built over an equal grid with the same options and goals makes every later result identical.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.FlowField);
        w.I32(_width);
        w.I32(_height);
        w.I32(_areasVersion);
        w.I32(_goals.Length);
        for (int i = 0; i < _goals.Length; i++)
        {
            w.I32(_goals[i]);
        }
        w.I32(_pending.Count);
        for (int i = 0; i < _pending.Count; i++)
        {
            w.I32(_pending[i]);
        }
        _driver.WriteState(w);
        return w.ToArray();
    }

    /// <summary>
    /// Restores a state written by <see cref="SaveState"/>. Restore the grid first, then this; the field must have been created with the
    /// same goals and options (agent size, connectivity, hooks) as the saved one.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, are for a different grid or goals, or hold impossible values; the field must then be restored again from good bytes or recomputed with <see cref="Recompute"/>.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        var r = new StateReader(state, StateKind.FlowField);
        r.Expect(_width, "the width");
        r.Expect(_height, "the height");
        int areasVersion = r.I32();
        r.Expect(_goals.Length, "the goal count");
        for (int i = 0; i < _goals.Length; i++)
        {
            int g = r.I32();
            if (g != _goals[i])
            {
                throw r.Fail($"goal {i} is node {g} in the state but node {_goals[i]} in this field.");
            }
        }
        int cells = _width * _height;
        int pending = r.Count(int.MaxValue, 4, "the pending edit count");
        var edits = new List<int>(pending);
        for (int i = 0; i < pending; i++)
        {
            edits.Add(r.Node(cells, "pending edit"));
        }
        _driver.ReadState(ref r);
        r.End();
        _pending.Clear();
        _pending.AddRange(edits);
        _areasVersion = areasVersion;
    }

    internal int Width => _width;

    internal int Height => _height;

    internal Cell[] GoalCells()
    {
        var cells = new Cell[_goals.Length];
        for (int i = 0; i < cells.Length; i++)
        {
            int y = _goals[i] / _width;
            cells[i] = new Cell(_goals[i] - (y * _width), y);
        }
        return cells;
    }

    void IGridRestoreListener.OnGridRestored()
    {
        _pending.Clear();
        _areasVersion = -1; // never equals a real version: the next Refresh recomputes the field
    }
}
