using System;
using System.Buffers.Binary;
using System.Globalization;
using RecursiveAI.OnePath.Graphs;

namespace RecursiveAI.OnePath;

/// <summary>How <see cref="Landmarks.Build"/> picks landmark nodes.</summary>
public enum LandmarkSelection
{
    /// <summary>
    /// Goldberg &amp; Harrelson's "avoid": grow a shortest-path tree from a seeded random root, weight each node by how badly the
    /// current landmarks estimate its distance from the root, and put the next landmark at a leaf of the heaviest subtree that
    /// holds no landmark yet. Usually the better bounds on road-like graphs. The default.
    /// </summary>
    Avoid = 0,

    /// <summary>Goldberg &amp; Harrelson's "farthest": the first landmark is the node farthest from a seeded random root, each next one the node farthest from all landmarks so far.</summary>
    Farthest = 1,
}

/// <summary>Options for <see cref="Graph.BuildLandmarks"/> and <see cref="PointGraph.BuildLandmarks"/>.</summary>
public sealed record LandmarkOptions
{
    /// <summary>Requested number of landmarks, at least 1; capped at the node count. Default 16, a good value for road-like graphs.</summary>
    public int Count { get; init => field = Require.That(value >= 1, value, nameof(Count), "at least 1"); } = 16;

    /// <summary>How landmarks are chosen. Default <see cref="LandmarkSelection.Avoid"/>.</summary>
    public LandmarkSelection Selection { get; init; } = LandmarkSelection.Avoid;

    /// <summary>Seed of the generator that picks the first root; any value. Default 1.</summary>
    public ulong Seed { get; init; } = 1;
}

/// <summary>
/// ALT landmark tables (A*, Landmarks, Triangle inequality; Goldberg &amp; Harrelson, SODA 2005). For each landmark <c>L</c> the
/// table stores <c>d(L, v)</c> and <c>d(v, L)</c> for every node. The triangle inequality then gives the lower bound
/// <c>d(u, v) ≥ max over L of ( d(L, v) − d(L, u), d(u, L) − d(v, L) )</c>, which is admissible and consistent (a maximum of
/// consistent potentials), so A* and bidirectional A* using it stay optimal. Works on directed graphs and with unreachable nodes
/// (an infinite bound means "cannot be reached"). Selection and tie-breaks are deterministic for a given graph, count, method and seed.
/// A table belongs to the graph it was built for; a search refuses a table built for a different graph.
/// </summary>
public sealed class Landmarks
{
    private const int FormatVersion = 1;
    private static readonly byte[] Magic = [(byte)'O', (byte)'P', (byte)'L', (byte)'M'];

    private readonly int _nodeCount;
    private readonly int[] _nodes;
    private readonly double[] _from; // _from[l * n + v] = d(landmark l, v)
    private readonly double[] _to;   // _to[l * n + v] = d(v, landmark l)
    private readonly ulong _fingerprint;
    private readonly CsrGraph? _source;

    private Landmarks(int nodeCount, int[] nodes, double[] from, double[] to, ulong fingerprint, CsrGraph? source)
    {
        _nodeCount = nodeCount;
        _nodes = nodes;
        _from = from;
        _to = to;
        _fingerprint = fingerprint;
        _source = source;
    }

    /// <summary>Number of landmarks.</summary>
    public int Count => _nodes.Length;

    /// <summary>Number of nodes of the graph the table was built for.</summary>
    public int NodeCount => _nodeCount;

    /// <summary>The landmark nodes, in selection order.</summary>
    public ReadOnlySpan<int> Nodes => _nodes;

    /// <summary>
    /// Builds a table: selects <paramref name="count"/> landmarks and runs two Dijkstra searches (forward and over the reverse edges) from each.
    /// Cost O(count · E log V) time and 16 · count · V bytes.
    /// </summary>
    /// <param name="graph">The graph; edge weights as stored (a search with an edge-cost hook that lowers weights invalidates the bounds).</param>
    /// <param name="count">Requested number of landmarks, at least 1; capped at the node count. 16 is a good default for road-like graphs.</param>
    /// <param name="selection">How landmarks are chosen.</param>
    /// <param name="seed">Seed of the generator that picks roots (any value).</param>
    /// <returns>The table.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="graph"/> is null.</exception>
    /// <exception cref="ArgumentException">The graph has no nodes, or <paramref name="count"/> is below 1.</exception>
    public static Landmarks Build(CsrGraph graph, int count = 16, LandmarkSelection selection = LandmarkSelection.Avoid, ulong seed = 1)
    {
        ArgumentNullException.ThrowIfNull(graph);
        int n = graph.NodeCount;
        if (n == 0)
        {
            throw new ArgumentException("The graph has no nodes.", nameof(graph));
        }
        ArgumentOutOfRangeException.ThrowIfLessThan(count, 1);
        count = Math.Min(count, n);
        var rng = new SeededRandom(seed);
        var chosen = new int[count];
        var isLandmark = new bool[n];
        var from = new double[checked(count * n)];
        var to = new double[checked(count * n)];
        double[] dist = new double[n];
        int[] parent = new int[n];
        int[] order = new int[n];
        var nearest = new double[n]; // farthest selection: distance to the closest landmark so far
        Array.Fill(nearest, double.PositiveInfinity);
        int made = 0;
        while (made < count)
        {
            int root = rng.NextInt(n);
            int pick;
            if (selection == LandmarkSelection.Farthest)
            {
                pick = made == 0 ? FarthestFromRoot(graph, root, dist, parent, order) : ArgMaxFinite(nearest, isLandmark);
            }
            else
            {
                pick = AvoidPick(graph, root, made, from, to, isLandmark, dist, parent, order);
            }
            if (pick < 0)
            {
                pick = FirstUnused(isLandmark);
            }
            chosen[made] = pick;
            isLandmark[pick] = true;
            ShortestPathTree.Compute(graph, pick, false, from.AsSpan(made * n, n), parent, order);
            ShortestPathTree.Compute(graph, pick, true, to.AsSpan(made * n, n), parent, order);
            for (int v = 0; v < n; v++)
            {
                double d = from[(made * n) + v];
                if (d < nearest[v])
                {
                    nearest[v] = d;
                }
            }
            made++;
        }
        return new Landmarks(n, chosen, from, to, graph.Fingerprint(), graph);
    }

    /// <summary>A lower bound on the distance from <paramref name="from"/> to <paramref name="to"/>: 0 or more, +∞ when <paramref name="to"/> cannot be reached from <paramref name="from"/>.</summary>
    /// <param name="from">Source node.</param>
    /// <param name="to">Target node.</param>
    /// <exception cref="ArgumentOutOfRangeException">A node is outside the table.</exception>
    public double LowerBound(int from, int to)
    {
        int n = _nodeCount;
        if ((uint)from >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(from), from, "Node is outside the landmark table.");
        }
        if ((uint)to >= (uint)n)
        {
            throw new ArgumentOutOfRangeException(nameof(to), to, "Node is outside the landmark table.");
        }
        double best = 0.0;
        double[] f = _from;
        double[] t = _to;
        for (int l = 0, o = 0; l < _nodes.Length; l++, o += n)
        {
            // d(L, to) <= d(L, from) + d(from, to)
            double lf = f[o + from];
            if (!double.IsPositiveInfinity(lf))
            {
                double b = f[o + to] - lf;
                if (b > best)
                {
                    best = b;
                }
            }
            // d(from, L) <= d(from, to) + d(to, L)
            double tt = t[o + to];
            if (!double.IsPositiveInfinity(tt))
            {
                double b = t[o + from] - tt;
                if (b > best)
                {
                    best = b;
                }
            }
        }
        return best;
    }

    /// <summary>True if this table was built for <paramref name="graph"/> (the same object, or one with the same fingerprint).</summary>
    /// <param name="graph">The graph a search is about to use.</param>
    public bool Matches(CsrGraph graph)
    {
        ArgumentNullException.ThrowIfNull(graph);
        return ReferenceEquals(graph, _source) || (graph.NodeCount == _nodeCount && graph.Fingerprint() == _fingerprint);
    }

    /// <summary>
    /// Serializes the table. Layout (little-endian): the bytes "OPLM"; u32 format version (1); i32 node count; i32 landmark count;
    /// u64 graph fingerprint; i32 landmark node ids; the forward table (count · nodes f64, landmark-major); the backward table likewise.
    /// The same table writes the same bytes on every platform.
    /// </summary>
    /// <returns>The bytes.</returns>
    public byte[] Serialize()
    {
        int n = _nodeCount;
        int k = _nodes.Length;
        var bytes = new byte[4 + 4 + 4 + 4 + 8 + (4 * k) + (16 * k * n)];
        Span<byte> s = bytes;
        Magic.CopyTo(s);
        BinaryPrimitives.WriteUInt32LittleEndian(s[4..], FormatVersion);
        BinaryPrimitives.WriteInt32LittleEndian(s[8..], n);
        BinaryPrimitives.WriteInt32LittleEndian(s[12..], k);
        BinaryPrimitives.WriteUInt64LittleEndian(s[16..], _fingerprint);
        int p = 24;
        foreach (int node in _nodes)
        {
            BinaryPrimitives.WriteInt32LittleEndian(s[p..], node);
            p += 4;
        }
        foreach (double d in _from)
        {
            BinaryPrimitives.WriteUInt64LittleEndian(s[p..], BitConverter.DoubleToUInt64Bits(d));
            p += 8;
        }
        foreach (double d in _to)
        {
            BinaryPrimitives.WriteUInt64LittleEndian(s[p..], BitConverter.DoubleToUInt64Bits(d));
            p += 8;
        }
        return bytes;
    }

    /// <summary>Reads a table written by <see cref="Serialize"/> and checks it against <paramref name="graph"/>.</summary>
    /// <param name="data">The bytes.</param>
    /// <param name="graph">The graph the table must belong to.</param>
    /// <returns>The table.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="graph"/> is null.</exception>
    /// <exception cref="GraphStateException">The data is truncated, has a wrong magic, version or size, holds an out-of-range node id or an invalid distance, or was built for another graph.</exception>
    public static Landmarks Deserialize(ReadOnlySpan<byte> data, CsrGraph graph)
    {
        ArgumentNullException.ThrowIfNull(graph);
        if (data.Length < 24)
        {
            throw new GraphStateException($"Landmark data is {data.Length} bytes; the header alone needs 24.");
        }
        if (!data[..4].SequenceEqual(Magic))
        {
            throw new GraphStateException("Landmark data does not start with the OPLM magic.");
        }
        uint version = BinaryPrimitives.ReadUInt32LittleEndian(data[4..]);
        if (version != FormatVersion)
        {
            throw new GraphStateException($"Landmark data has format version {version}; this build reads version {FormatVersion}.");
        }
        int n = BinaryPrimitives.ReadInt32LittleEndian(data[8..]);
        int k = BinaryPrimitives.ReadInt32LittleEndian(data[12..]);
        ulong fingerprint = BinaryPrimitives.ReadUInt64LittleEndian(data[16..]);
        if (n != graph.NodeCount)
        {
            throw new GraphStateException($"Landmark data is for {n} nodes but the graph has {graph.NodeCount}.");
        }
        if (k < 1 || k > n)
        {
            throw new GraphStateException($"Landmark count {k} is outside 1..{n}.");
        }
        long expected = 24L + (4L * k) + (16L * k * n);
        if (data.Length != expected)
        {
            throw new GraphStateException($"Landmark data is {data.Length} bytes; {k} landmarks over {n} nodes need exactly {expected}.");
        }
        if (fingerprint != graph.Fingerprint())
        {
            throw new GraphStateException("Landmark data was built for a different graph (fingerprint mismatch).");
        }
        var nodes = new int[k];
        int p = 24;
        for (int i = 0; i < k; i++, p += 4)
        {
            int node = BinaryPrimitives.ReadInt32LittleEndian(data[p..]);
            if ((uint)node >= (uint)n)
            {
                throw new GraphStateException($"Landmark {i} is node {node}, outside 0..{n - 1}.");
            }
            nodes[i] = node;
        }
        var from = new double[k * n];
        var to = new double[k * n];
        ReadTable(data, ref p, from, "forward");
        ReadTable(data, ref p, to, "backward");
        for (int i = 0; i < k; i++)
        {
            if (from[(i * n) + nodes[i]] != 0.0 || to[(i * n) + nodes[i]] != 0.0)
            {
                throw new GraphStateException($"Landmark {i} (node {nodes[i]}) has a nonzero distance to itself.");
            }
        }
        return new Landmarks(n, nodes, from, to, fingerprint, graph);
    }

    private static void ReadTable(ReadOnlySpan<byte> data, ref int p, double[] table, string name)
    {
        for (int i = 0; i < table.Length; i++, p += 8)
        {
            double d = BitConverter.UInt64BitsToDouble(BinaryPrimitives.ReadUInt64LittleEndian(data[p..]));
            if (!(d >= 0.0))
            {
                throw new GraphStateException(string.Create(CultureInfo.InvariantCulture, $"Landmark {name} table entry {i} is {d}; distances must be 0 or more."));
            }
            table[i] = d;
        }
    }

    private static int FirstUnused(bool[] used)
    {
        for (int i = 0; i < used.Length; i++)
        {
            if (!used[i])
            {
                return i;
            }
        }
        return -1;
    }

    private static int ArgMaxFinite(double[] key, bool[] excluded)
    {
        int best = -1;
        double bestKey = -1.0;
        for (int v = 0; v < key.Length; v++)
        {
            double d = key[v];
            if (!excluded[v] && !double.IsPositiveInfinity(d) && d > bestKey)
            {
                bestKey = d;
                best = v;
            }
        }
        return best;
    }

    private static int FarthestFromRoot(CsrGraph g, int root, double[] dist, int[] parent, int[] order)
    {
        ShortestPathTree.Compute(g, root, false, dist, parent, order);
        return ArgMaxFinite(dist, new bool[g.NodeCount]);
    }

    // Goldberg & Harrelson 2005, §5 "avoid". Weight of v = d(root, v) minus the current lower bound on it; the size of a
    // subtree is the sum of its weights, or 0 if it already contains a landmark; descend from the heaviest to a leaf.
    private static int AvoidPick(CsrGraph g, int root, int made, double[] from, double[] to, bool[] isLandmark, double[] dist, int[] parent, int[] order)
    {
        int n = g.NodeCount;
        int settled = ShortestPathTree.Compute(g, root, false, dist, parent, order);
        var size = new double[n];
        var blocked = new bool[n];
        var bestChild = new int[n];
        Array.Fill(bestChild, -1);
        for (int i = settled - 1; i >= 0; i--)
        {
            int v = order[i];
            if (isLandmark[v])
            {
                blocked[v] = true;
            }
            if (!blocked[v])
            {
                double bound = 0.0;
                for (int l = 0; l < made; l++)
                {
                    int o = l * n;
                    double lf = from[o + root];
                    if (!double.IsPositiveInfinity(lf))
                    {
                        bound = Math.Max(bound, from[o + v] - lf);
                    }
                    double tv = to[o + v];
                    if (!double.IsPositiveInfinity(tv))
                    {
                        bound = Math.Max(bound, to[o + root] - tv);
                    }
                }
                size[v] += dist[v] - bound;
            }
            int p = parent[v];
            if (p >= 0)
            {
                if (blocked[v])
                {
                    blocked[p] = true;
                }
                else
                {
                    size[p] += size[v];
                    int bc = bestChild[p];
                    if (bc < 0 || size[v] > size[bc] || (size[v] == size[bc] && v < bc))
                    {
                        bestChild[p] = v;
                    }
                }
            }
        }
        int best = -1;
        double bestSize = 0.0;
        for (int i = 0; i < settled; i++)
        {
            int v = order[i];
            if (!blocked[v] && (best < 0 || size[v] > bestSize || (size[v] == bestSize && v < best)))
            {
                best = v;
                bestSize = size[v];
            }
        }
        if (best < 0 || !(bestSize > 0.0))
        {
            return -1;
        }
        // blocked children never become bestChild, so the descent stays inside landmark-free subtrees.
        while (bestChild[best] >= 0)
        {
            best = bestChild[best];
        }
        return isLandmark[best] ? -1 : best;
    }
}
