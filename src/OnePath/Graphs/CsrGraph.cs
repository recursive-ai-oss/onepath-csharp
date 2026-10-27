using System;
using System.Globalization;

namespace RecursiveAI.OnePath.Graphs;

/// <summary>A directed edge of a weighted graph.</summary>
/// <param name="From">Source node id.</param>
/// <param name="To">Target node id.</param>
/// <param name="Weight">Cost of the edge: finite and not negative.</param>
public readonly record struct GraphEdge(int From, int To, double Weight);

/// <summary>A position in 3D (or 2D with <c>Z = 0</c>) for point graphs. Coordinates must be finite and within ±2^21.</summary>
public readonly record struct Point3(double X, double Y, double Z)
{
    /// <summary>Creates a point in the plane (<c>Z = 0</c>).</summary>
    public Point3(double x, double y) : this(x, y, 0.0)
    {
    }

    /// <summary>The straight-line distance to <paramref name="other"/>: <c>sqrt(dx*dx + dy*dy + dz*dz)</c> summed in that order, IEEE-exact on every platform.</summary>
    /// <param name="other">The other point.</param>
    public double DistanceTo(Point3 other)
    {
        double dx = X - other.X;
        double dy = Y - other.Y;
        double dz = Z - other.Z;
        return Math.Sqrt(((dx * dx) + (dy * dy)) + (dz * dz));
    }
}

/// <summary>
/// An immutable directed weighted graph in compressed sparse row form, with the reverse adjacency kept beside it
/// (backward searches and landmark tables need it). Node ids are <c>0..NodeCount-1</c>. The edges out of a node are
/// stored in the order they appeared in the input edge list (a stable counting sort), and the edges into a node
/// likewise, so every search enumerates neighbours in the same order on every platform. Safe to read from several threads.
/// </summary>
public sealed class CsrGraph
{
    /// <summary>The largest node count (2^26); bounds memory for hostile input.</summary>
    public const int MaxNodes = 1 << 26;

    private readonly int[] _offsets;
    private readonly int[] _targets;
    private readonly double[] _weights;
    private readonly int[] _reverseOffsets;
    private readonly int[] _sources;
    private readonly double[] _reverseWeights;
    private ulong _fingerprint;
    private bool _hasFingerprint;

    private CsrGraph(int[] offsets, int[] targets, double[] weights, int[] reverseOffsets, int[] sources, double[] reverseWeights)
    {
        _offsets = offsets;
        _targets = targets;
        _weights = weights;
        _reverseOffsets = reverseOffsets;
        _sources = sources;
        _reverseWeights = reverseWeights;
    }

    /// <summary>Number of nodes.</summary>
    public int NodeCount => _offsets.Length - 1;

    /// <summary>Number of directed edges.</summary>
    public int EdgeCount => _targets.Length;

    /// <summary>For node <c>v</c>, its out-edges are slots <c>Offsets[v] .. Offsets[v+1]-1</c> of <see cref="Targets"/> and <see cref="Weights"/>.</summary>
    public ReadOnlySpan<int> Offsets => _offsets;

    /// <summary>Target node of each out-edge slot.</summary>
    public ReadOnlySpan<int> Targets => _targets;

    /// <summary>Weight of each out-edge slot.</summary>
    public ReadOnlySpan<double> Weights => _weights;

    /// <summary>For node <c>v</c>, its in-edges are slots <c>ReverseOffsets[v] .. ReverseOffsets[v+1]-1</c> of <see cref="Sources"/> and <see cref="ReverseWeights"/>.</summary>
    public ReadOnlySpan<int> ReverseOffsets => _reverseOffsets;

    /// <summary>Source node of each in-edge slot.</summary>
    public ReadOnlySpan<int> Sources => _sources;

    /// <summary>Weight of each in-edge slot.</summary>
    public ReadOnlySpan<double> ReverseWeights => _reverseWeights;

    internal int[] OffsetArray => _offsets;

    internal int[] TargetArray => _targets;

    internal double[] WeightArray => _weights;

    internal int[] ReverseOffsetArray => _reverseOffsets;

    internal int[] SourceArray => _sources;

    internal double[] ReverseWeightArray => _reverseWeights;

    /// <summary>Builds the graph from an edge list in O(nodes + edges).</summary>
    /// <param name="nodeCount">Number of nodes, 0 to <see cref="MaxNodes"/>.</param>
    /// <param name="edges">The edges; endpoints in range, weights finite and not negative. Parallel edges and self-loops are kept.</param>
    /// <returns>The graph.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="nodeCount"/> is outside 0..<see cref="MaxNodes"/>.</exception>
    /// <exception cref="ArgumentException">An edge has an endpoint outside the graph or a weight that is negative, infinite or NaN; the message names the edge index.</exception>
    public static CsrGraph Create(int nodeCount, ReadOnlySpan<GraphEdge> edges)
    {
        if ((uint)nodeCount > MaxNodes)
        {
            throw new ArgumentOutOfRangeException(nameof(nodeCount), nodeCount, $"The node count must be between 0 and {MaxNodes}.");
        }
        var offsets = new int[nodeCount + 1];
        var reverseOffsets = new int[nodeCount + 1];
        for (int i = 0; i < edges.Length; i++)
        {
            GraphEdge e = edges[i];
            if ((uint)e.From >= (uint)nodeCount || (uint)e.To >= (uint)nodeCount)
            {
                throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Edge {i} ({e.From} -> {e.To}) has an endpoint outside the {nodeCount} nodes of the graph."), nameof(edges));
            }
            if (!(e.Weight >= 0.0) || double.IsPositiveInfinity(e.Weight))
            {
                throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Edge {i} ({e.From} -> {e.To}) has weight {e.Weight}; weights must be finite and not negative."), nameof(edges));
            }
            offsets[e.From + 1]++;
            reverseOffsets[e.To + 1]++;
        }
        for (int v = 0; v < nodeCount; v++)
        {
            offsets[v + 1] += offsets[v];
            reverseOffsets[v + 1] += reverseOffsets[v];
        }
        var targets = new int[edges.Length];
        var weights = new double[edges.Length];
        var sources = new int[edges.Length];
        var reverseWeights = new double[edges.Length];
        var next = new int[nodeCount];
        var reverseNext = new int[nodeCount];
        Array.Copy(offsets, next, nodeCount);
        Array.Copy(reverseOffsets, reverseNext, nodeCount);
        // Stable placement: input order is preserved within each node's slice.
        for (int i = 0; i < edges.Length; i++)
        {
            GraphEdge e = edges[i];
            int s = next[e.From]++;
            targets[s] = e.To;
            weights[s] = e.Weight;
            int r = reverseNext[e.To]++;
            sources[r] = e.From;
            reverseWeights[r] = e.Weight;
        }
        return new CsrGraph(offsets, targets, weights, reverseOffsets, sources, reverseWeights);
    }

    /// <summary>
    /// A 64-bit FNV-1a digest of the node count, offsets, targets and weight bits. Landmark tables carry it so they
    /// can be refused for a graph they were not built for. Equal graphs (same edges in the same order) have equal digests.
    /// </summary>
    public ulong Fingerprint()
    {
        if (_hasFingerprint)
        {
            return _fingerprint;
        }
        ulong h = 14695981039346656037UL;
        void Mix(ulong v)
        {
            // Little-endian bytes, low byte first.
            for (int i = 0; i < 8; i++)
            {
                h = (h ^ ((v >> (8 * i)) & 0xFF)) * 1099511628211UL;
            }
        }
        Mix((ulong)NodeCount);
        foreach (int o in _offsets)
        {
            Mix((ulong)o);
        }
        for (int i = 0; i < _targets.Length; i++)
        {
            Mix((ulong)_targets[i]);
            Mix(BitConverter.DoubleToUInt64Bits(_weights[i]));
        }
        _fingerprint = h;
        _hasFingerprint = true;
        return h;
    }
}
