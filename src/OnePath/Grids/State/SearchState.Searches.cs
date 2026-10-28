using System;
using System.Globalization;

namespace RecursiveAI.OnePath.Grids;

// Save/restore of a time-sliced search in progress (Begin/Step form): A*, jump point search, Theta*/Lazy Theta* and graph A*.
// The object is built the usual way over the same grid or graph and with the same options and then restored in place;
// options that are not state (cost and heuristic hooks, weight) are the caller's to match, and the weight is checked.
// HPA* and bidirectional searches are not covered (docs/grids-state.md).

internal static class SearchStateIO
{
    internal static void Write(StateWriter w, SearchStatus status, int nodeCount, double weight, int start, int goal, int expanded, int iterations, double pathCost, GridSearchBuffers buffers)
    {
        w.I32(nodeCount);
        w.U8((byte)status);
        w.F64(weight);
        w.I32(start);
        w.I32(goal);
        w.I32(expanded);
        w.I32(iterations);
        w.F64(pathCost);
        if (status != SearchStatus.NotStarted)
        {
            buffers.WriteState(w, nodeCount);
        }
    }

    internal static void Read(ref StateReader r, int nodeCount, double expectedWeight, GridSearchBuffers buffers, out SearchStatus status, out int start, out int goal, out int expanded, out int iterations, out double pathCost)
    {
        r.Expect(nodeCount, "the node count");
        byte s = r.U8();
        if (s > (byte)SearchStatus.NotFound)
        {
            throw r.Fail($"search status {s} is not defined.");
        }
        status = (SearchStatus)s;
        double weight = r.F64();
        if (BitConverter.DoubleToInt64Bits(weight) != BitConverter.DoubleToInt64Bits(expectedWeight))
        {
            throw r.Fail(string.Create(CultureInfo.InvariantCulture, $"the heuristic weight is {weight:R} in the state but {expectedWeight:R} on the search being restored; configure it the same first."));
        }
        start = r.I32();
        goal = r.I32();
        expanded = r.I32();
        iterations = r.I32();
        pathCost = r.F64();
        bool started = status != SearchStatus.NotStarted;
        if (start < -1 || start >= nodeCount || goal < -1 || goal >= nodeCount || (started && goal < 0))
        {
            throw r.Fail($"start {start} or goal {goal} is outside the {nodeCount} nodes.");
        }
        if (expanded < 0 || iterations < 0 || double.IsNaN(pathCost))
        {
            throw r.Fail("a counter or the path cost is negative or NaN.");
        }
        if (started)
        {
            buffers.ReadState(ref r, nodeCount);
            if (!buffers.ParentLinksAcyclic(nodeCount))
            {
                throw r.Fail("a visited node's parent is not a visited node that costs no more, or the parent links form a cycle.");
            }
            // Path extraction starts at the goal and follows parent links; a found search whose goal was never closed would follow
            // links left over from an earlier search.
            if (status == SearchStatus.Found && buffers.Mark[goal] != buffers.Base + 1)
            {
                throw r.Fail("a found search must have its goal closed.");
            }
        }
        r.End();
    }
}

public sealed partial class AStar<TTopology, TCost, THeuristic>
{
    /// <summary>
    /// Writes the search as it stands (before <see cref="Begin"/>, mid-run, or finished) to bytes. The format is little-endian
    /// and written in a documented order (docs/grids-state.md), so equal states give equal bytes on every platform.
    /// </summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.AStar);
        SearchStateIO.Write(w, _status, _topology.NodeCount, _weight, _start, _goal, _expandedCount, _iterations, _pathCost, _buffers);
        return w.ToArray();
    }

    /// <summary>
    /// Restores a state written by <see cref="SaveState"/> into this search, which must be built over the same grid and
    /// options (same node count and <see cref="HeuristicWeight"/>). Later steps give exactly what the saved search would have.
    /// </summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or inconsistent with this search; the search is then unusable until restored again or <see cref="Begin"/> is called.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.AStar);
        _status = SearchStatus.NotStarted;
        SearchStateIO.Read(ref r, _topology.NodeCount, _weight, _buffers, out SearchStatus status, out _start, out _goal, out _expandedCount, out _iterations, out _pathCost);
        if (status != SearchStatus.NotStarted)
        {
            _heuristic.SetGoal(_goal);
        }
        _status = status;
    }
}

public sealed partial class JumpSearch<TJumper, THeuristic>
{
    /// <summary>Writes the search as it stands to bytes; see <see cref="AStar{TTopology, TCost, THeuristic}.SaveState"/>.</summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.JumpSearch);
        SearchStateIO.Write(w, _status, _width * _jumper.Height, 1.0, -1, _goal, _expandedCount, _iterations, _pathCost, _buffers);
        return w.ToArray();
    }

    /// <summary>Restores a state written by <see cref="SaveState"/> into a search built over the same grid.</summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or inconsistent with this search.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.JumpSearch);
        _status = SearchStatus.NotStarted;
        SearchStateIO.Read(ref r, _width * _jumper.Height, 1.0, _buffers, out SearchStatus status, out _, out _goal, out _expandedCount, out _iterations, out _pathCost);
        if (status != SearchStatus.NotStarted)
        {
            _jumper.BeginSearch();
            _heuristic.SetGoal(_goal);
        }
        _status = status;
    }
}

public sealed partial class ThetaSearch<THeuristic>
{
    /// <summary>Writes the search as it stands to bytes; see <see cref="AStar{TTopology, TCost, THeuristic}.SaveState"/>.</summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.ThetaSearch);
        SearchStateIO.Write(w, _status, _grid.CellCount, _weight, -1, _goal, _expandedCount, _iterations, _pathCost, _buffers);
        return w.ToArray();
    }

    /// <summary>Restores a state written by <see cref="SaveState"/> into a search built over the same grid, variant and weight.</summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GridStateException">The bytes are truncated, of another kind or version, or inconsistent with this search.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.ThetaSearch);
        _status = SearchStatus.NotStarted;
        SearchStateIO.Read(ref r, _grid.CellCount, _weight, _buffers, out SearchStatus status, out _, out _goal, out _expandedCount, out _iterations, out _pathCost);
        if (status != SearchStatus.NotStarted)
        {
            _heuristic.SetGoal(_goal);
        }
        _status = status;
    }
}
