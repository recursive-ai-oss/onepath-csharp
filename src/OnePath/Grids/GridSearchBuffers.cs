using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Working memory of a grid search: per-node cost, parent and visit marks, plus the open set. Create one per
/// grid size and reuse it for any number of searches; a search never clears the arrays (marks carry a
/// generation number), so a query costs time proportional to what it touches. Not thread-safe: use one per thread.
/// </summary>
public sealed class GridSearchBuffers
{
    private uint _base;
    private int[] _expanded = Array.Empty<int>();
    private int _expandedCount;

    /// <summary>Creates buffers for graphs of up to <paramref name="nodeCapacity"/> nodes.</summary>
    /// <param name="nodeCapacity">Largest node count that will be searched; not negative.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="nodeCapacity"/> is negative.</exception>
    public GridSearchBuffers(int nodeCapacity)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(nodeCapacity);
        G = new double[nodeCapacity];
        Parent = new int[nodeCapacity];
        Mark = new uint[nodeCapacity];
        Open = new OpenSet(nodeCapacity);
    }

    /// <summary>Largest node count these buffers can search.</summary>
    public int NodeCapacity => G.Length;

    /// <summary>When true, searches record the nodes they expand, in order (see <see cref="ExpandedNodes"/>).</summary>
    public bool RecordExpansions { get; set; }

    /// <summary>The open set of the last search.</summary>
    public OpenSet Open { get; }

    /// <summary>The nodes expanded by the last search, in expansion order; empty unless <see cref="RecordExpansions"/> was set.</summary>
    public ReadOnlySpan<int> ExpandedNodes => new(_expanded, 0, _expandedCount);

    internal double[] G { get; }

    internal int[] Parent { get; }

    internal uint[] Mark { get; }

    /// <summary>First mark value of the current search: <c>Base</c> = open or reached, <c>Base + 1</c> = closed.</summary>
    internal uint Base => _base;

    internal void BeginSearch(int nodeCount)
    {
        if (nodeCount > G.Length)
        {
            throw new ArgumentException($"The search buffers hold {G.Length} nodes but the graph has {nodeCount}.", nameof(nodeCount));
        }
        if (_base >= uint.MaxValue - 4)
        {
            Array.Clear(Mark);
            _base = 0;
        }
        _base += 2;
        Open.Clear();
        _expandedCount = 0;
    }

    internal void RecordExpanded(int node)
    {
        if (_expandedCount == _expanded.Length)
        {
            Array.Resize(ref _expanded, Math.Max(256, _expanded.Length * 2));
        }
        _expanded[_expandedCount++] = node;
    }
}

/// <summary>Where a (possibly sliced) search stands.</summary>
public enum SearchStatus
{
    /// <summary>No search has begun.</summary>
    NotStarted = 0,

    /// <summary>The search has work left; call <c>Step</c> again.</summary>
    Running = 1,

    /// <summary>The goal was reached; the path and cost are available.</summary>
    Found = 2,

    /// <summary>The open set emptied without reaching the goal.</summary>
    NotFound = 3,
}
