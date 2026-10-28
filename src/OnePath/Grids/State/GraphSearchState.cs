using System;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Graphs;

public sealed partial class GraphAStar<TCost, THeuristic>
{
    /// <summary>Writes the search as it stands to bytes; see <see cref="AStar{TTopology, TCost, THeuristic}.SaveState"/>.</summary>
    /// <returns>The state bytes.</returns>
    public byte[] SaveState()
    {
        var w = new StateWriter(StateKind.GraphAStar);
        SearchStateIO.Write(w, _status, _graph.NodeCount, 1.0, -1, _goal, _expandedCount, _iterations, _pathCost, _buffers);
        return w.ToArray();
    }

    /// <summary>Restores a state written by <see cref="SaveState"/> into a search built over an equal graph.</summary>
    /// <param name="state">The bytes.</param>
    /// <exception cref="GraphStateException">The bytes are truncated, of another kind or version, or inconsistent with this search.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        var r = new StateReader(state, StateKind.GraphAStar);
        _status = SearchStatus.NotStarted;
        SearchStateIO.Read(ref r, _graph.NodeCount, 1.0, _buffers, out SearchStatus status, out _, out _goal, out _expandedCount, out _iterations, out _pathCost);
        _status = status;
    }
}
