using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// The abstract graph of HPA* (Botea, Müller &amp; Schaeffer 2004, "Near Optimal Hierarchical Path-Finding") over a square
/// <see cref="Grid"/>: the grid is cut into square chunks (clusters); along the border of two neighbouring chunks every
/// maximal run of cells walkable on both sides is an entrance, which gets one transition in its middle (narrow entrance)
/// or one at each end (width at least <see cref="EntranceThreshold"/>). A transition is a pair of adjacent cells, one per
/// chunk, joined by an inter-edge; the cells are the abstract nodes. Inside a chunk, every pair of nodes is joined by an
/// intra-edge priced by a search restricted to the chunk. One level only (no multi-level hierarchy).
/// <para>
/// The graph is a pure function of the grid's cells, the area costs and the options: every list is sorted by cell index,
/// so a hierarchy updated after cell edits is identical to one built from scratch (<see cref="ContentEquals"/>, <see cref="GetContentHash"/>).
/// The hierarchy registers as a <see cref="IGridChangeListener"/>; edits only mark chunks, and the next query (or
/// <see cref="Refresh"/>) rebuilds the marked chunks and the chunks bordering them. Edits to the area cost table trigger a full rebuild.
/// </para>
/// <para>
/// Agent size: the hierarchy is built over the cells walkable for <see cref="AgentSize"/> (true clearance, see
/// <see cref="Grid.GetClearance"/>), so one hierarchy serves one size; keep one per size (the paper's annotated variant, one
/// annotation per size, is not used).
/// </para>
/// <para>Not thread-safe against edits; concurrent searches are fine while nobody edits the grid or calls <see cref="Refresh"/> after an edit.</para>
/// </summary>
public sealed class HpaHierarchy : IGridChangeListener, IDisposable
{
    private readonly Grid _grid;
    private readonly SquareTopology _topology;
    private readonly int _width;
    private readonly int _height;
    private readonly ClusterData[] _clusters;
    private readonly bool[] _dirty;
    private readonly ClusterSolver _solver;
    private readonly List<long> _pairs = [];
    private readonly bool _eight;
    private int _dirtyCount;
    private int _areasVersion;
    private int _nodeCount;
    private bool _disposed;
    private HpaSearch? _spareSearch;

    /// <summary>Builds the hierarchy for the grid's current contents and starts following its edits.</summary>
    /// <param name="grid">The grid.</param>
    /// <param name="options">Settings; null means the defaults.</param>
    /// <exception cref="ArgumentNullException"><paramref name="grid"/> is null.</exception>
    /// <exception cref="ArgumentException">Eight-connected moves with <see cref="DiagonalRule.AllowAll"/> were requested.</exception>
    public HpaHierarchy(Grid grid, HpaOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(grid);
        options ??= new HpaOptions();
        if (options.Connectivity == Connectivity.Eight && options.Diagonals == DiagonalRule.AllowAll)
        {
            throw new ArgumentException("HPA* does not support DiagonalRule.AllowAll on eight-connected grids.", nameof(options));
        }
        _grid = grid;
        ChunkSize = options.ChunkSize;
        EntranceThreshold = options.EntranceThreshold;
        AgentSize = options.AgentSize;
        Connectivity = options.Connectivity;
        Diagonals = options.Diagonals;
        _eight = options.Connectivity == Connectivity.Eight;
        _width = grid.Width;
        _height = grid.Height;
        ClustersX = (_width + ChunkSize - 1) / ChunkSize;
        ClustersY = (_height + ChunkSize - 1) / ChunkSize;
        _topology = grid.CreateTopology(AgentSize, Connectivity, Diagonals);
        _clusters = new ClusterData[ClustersX * ClustersY];
        _dirty = new bool[_clusters.Length];
        _solver = new ClusterSolver(_topology, ChunkSize * ChunkSize);
        _areasVersion = grid.Areas.Version;
        for (int c = 0; c < _clusters.Length; c++)
        {
            _dirty[c] = true;
        }
        _dirtyCount = _clusters.Length;
        Refresh();
        grid.AddListener(this);
    }

    /// <summary>Chunk side in cells.</summary>
    public int ChunkSize { get; }

    /// <summary>See <see cref="HpaOptions.EntranceThreshold"/>.</summary>
    public int EntranceThreshold { get; }

    /// <summary>The agent size the hierarchy is built for.</summary>
    public int AgentSize { get; }

    /// <summary>The moves the hierarchy is built for.</summary>
    public Connectivity Connectivity { get; }

    /// <summary>The diagonal rule the hierarchy is built for.</summary>
    public DiagonalRule Diagonals { get; }

    /// <summary>The grid this hierarchy describes.</summary>
    public Grid Grid => _grid;

    /// <summary>Chunks per row.</summary>
    public int ClustersX { get; }

    /// <summary>Chunks per column.</summary>
    public int ClustersY { get; }

    /// <summary>Number of chunks (row-major: <c>cy * ClustersX + cx</c>).</summary>
    public int ClusterCount => _clusters.Length;

    /// <summary>Total number of abstract nodes (transition cells).</summary>
    public int NodeCount => _nodeCount;

    internal SquareTopology Topology => _topology;

    internal double MinCost => _grid.Areas.MinCost;

    /// <summary>The chunk containing a cell.</summary>
    /// <param name="cell">Cell index (<c>y * Width + x</c>).</param>
    /// <returns>The chunk index.</returns>
    public int ClusterOf(int cell)
    {
        int y = cell / _width;
        return ((y / ChunkSize) * ClustersX) + ((cell - (y * _width)) / ChunkSize);
    }

    /// <summary>The cells of the nodes of a chunk in ascending order.</summary>
    /// <param name="cluster">Chunk index.</param>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside the hierarchy.</exception>
    public ReadOnlySpan<int> ClusterNodes(int cluster)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)cluster, (uint)_clusters.Length);
        return _clusters[cluster].Nodes;
    }

    /// <summary>The cost of the intra-edge between two nodes of a chunk (+∞ when the chunk does not connect them).</summary>
    /// <param name="cluster">Chunk index.</param>
    /// <param name="from">Index into <see cref="ClusterNodes"/>.</param>
    /// <param name="to">Index into <see cref="ClusterNodes"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range.</exception>
    public double IntraCost(int cluster, int from, int to)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)cluster, (uint)_clusters.Length);
        ClusterData d = _clusters[cluster];
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)from, (uint)d.Nodes.Length);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)to, (uint)d.Nodes.Length);
        return d.Intra[(from * d.Nodes.Length) + to];
    }

    /// <summary>The inter-edges leaving one node of a chunk: the cells they lead to (in the neighbouring chunk) and their costs, sorted by target cell.</summary>
    /// <param name="cluster">Chunk index.</param>
    /// <param name="node">Index into <see cref="ClusterNodes"/>.</param>
    /// <param name="targets">The target cells.</param>
    /// <param name="costs">The move costs.</param>
    /// <exception cref="ArgumentOutOfRangeException">An index is out of range.</exception>
    public void GetInterEdges(int cluster, int node, out ReadOnlySpan<int> targets, out ReadOnlySpan<double> costs)
    {
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)cluster, (uint)_clusters.Length);
        ClusterData d = _clusters[cluster];
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual((uint)node, (uint)d.Nodes.Length);
        int a = d.InterOffset[node];
        int n = d.InterOffset[node + 1] - a;
        targets = new ReadOnlySpan<int>(d.InterTo, a, n);
        costs = new ReadOnlySpan<double>(d.InterCost, a, n);
    }

    /// <summary>Marks every chunk for rebuilding at the next <see cref="Refresh"/> (what an area-cost edit does).</summary>
    public void InvalidateAll()
    {
        for (int c = 0; c < _dirty.Length; c++)
        {
            _dirty[c] = true;
        }
        _dirtyCount = _dirty.Length;
    }

    /// <inheritdoc/>
    public void OnCellChanged(int x, int y)
    {
        // A cell edit changes the clearance of the cells up and to the left within AgentSize-1, and entrances along a
        // chunk border involve the cell next to it: mark every chunk touching that rectangle grown by one cell.
        int reach = AgentSize - 1;
        int x0 = Math.Max(0, x - reach - 1);
        int y0 = Math.Max(0, y - reach - 1);
        int x1 = Math.Min(_width - 1, x + 1);
        int y1 = Math.Min(_height - 1, y + 1);
        for (int cy = y0 / ChunkSize; cy <= y1 / ChunkSize; cy++)
        {
            for (int cx = x0 / ChunkSize; cx <= x1 / ChunkSize; cx++)
            {
                int c = (cy * ClustersX) + cx;
                if (!_dirty[c])
                {
                    _dirty[c] = true;
                    _dirtyCount++;
                }
            }
        }
    }

    /// <summary>Rebuilds the chunks marked by edits since the last call (cheap when nothing changed). Searches call this themselves.</summary>
    public void Refresh()
    {
        if (_areasVersion != _grid.Areas.Version)
        {
            _areasVersion = _grid.Areas.Version;
            InvalidateAll();
        }
        if (_dirtyCount == 0)
        {
            return;
        }
        _ = _grid.ClearanceArray; // recompute clearance after edits before reading it
        for (int c = 0; c < _clusters.Length; c++)
        {
            if (_dirty[c])
            {
                BuildCluster(c);
                _dirty[c] = false;
            }
        }
        _dirtyCount = 0;
        int total = 0;
        for (int c = 0; c < _clusters.Length; c++)
        {
            total += _clusters[c].Nodes.Length;
        }
        _nodeCount = total;
    }

    /// <summary>Discards the hierarchy's cached state and stops following the grid's edits.</summary>
    public void Dispose()
    {
        if (!_disposed)
        {
            _disposed = true;
            _grid.RemoveListener(this);
        }
    }

    /// <summary>True when both hierarchies have exactly the same nodes, intra-edge costs and inter-edges (bit for bit).</summary>
    /// <param name="other">The other hierarchy.</param>
    /// <returns>Whether the abstract graphs are identical.</returns>
    public bool ContentEquals(HpaHierarchy other)
    {
        ArgumentNullException.ThrowIfNull(other);
        if (ChunkSize != other.ChunkSize || _clusters.Length != other._clusters.Length)
        {
            return false;
        }
        for (int c = 0; c < _clusters.Length; c++)
        {
            if (!_clusters[c].SameAs(other._clusters[c]))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>A 64-bit FNV-1a digest of the abstract graph (nodes, intra costs as IEEE bits, inter-edges), equal on every platform.</summary>
    /// <returns>The digest.</returns>
    public ulong GetContentHash()
    {
        ulong h = 14695981039346656037UL;
        for (int c = 0; c < _clusters.Length; c++)
        {
            _clusters[c].Hash(ref h);
        }
        return h;
    }

    internal int FindNode(int cluster, int cell)
    {
        int[] nodes = _clusters[cluster].Nodes;
        int lo = 0;
        int hi = nodes.Length - 1;
        while (lo <= hi)
        {
            int mid = (lo + hi) >> 1;
            if (nodes[mid] < cell)
            {
                lo = mid + 1;
            }
            else if (nodes[mid] > cell)
            {
                hi = mid - 1;
            }
            else
            {
                return mid;
            }
        }
        return -1;
    }

    internal ClusterData Cluster(int c) => _clusters[c];

    internal void ClusterRect(int c, out int x0, out int y0, out int x1, out int y1)
    {
        int cx = c % ClustersX;
        int cy = c / ClustersX;
        x0 = cx * ChunkSize;
        y0 = cy * ChunkSize;
        x1 = Math.Min(_width, x0 + ChunkSize) - 1;
        y1 = Math.Min(_height, y0 + ChunkSize) - 1;
    }

    internal ClusterSolver NewSolver() => new(_topology, ChunkSize * ChunkSize);

    internal HpaSearch? TakeSpareSearch() => System.Threading.Interlocked.Exchange(ref _spareSearch, null);

    internal void ReturnSpareSearch(HpaSearch search) => _spareSearch = search;

    // The cost of the single move a -> b, +inf when the topology does not allow it.
    internal double MoveCost(int a, int b)
    {
        var finder = new CostFinder { Target = b, Cost = double.PositiveInfinity };
        _topology.VisitNeighbors(a, ref finder);
        return finder.Cost;
    }

    private struct CostFinder : INeighborVisitor
    {
        public int Target;
        public double Cost;

        public void Visit(int neighbor, double baseCost)
        {
            if (neighbor == Target)
            {
                Cost = baseCost;
            }
        }
    }

    private void BuildCluster(int c)
    {
        int cx = c % ClustersX;
        int cy = c / ClustersX;
        _pairs.Clear();
        // Entrances on each of the four borders: the pair is (cell in this chunk, cell across the border).
        if (cx > 0)
        {
            AddBorder(cx - 1, cy, cx, cy, ownIsFirst: false);
        }
        if (cx + 1 < ClustersX)
        {
            AddBorder(cx, cy, cx + 1, cy, ownIsFirst: true);
        }
        if (cy > 0)
        {
            AddBorder(cx, cy - 1, cx, cy, ownIsFirst: false);
        }
        if (cy + 1 < ClustersY)
        {
            AddBorder(cx, cy, cx, cy + 1, ownIsFirst: true);
        }
        _pairs.Sort();

        var nodes = new List<int>();
        for (int i = 0; i < _pairs.Count; i++)
        {
            int own = (int)(_pairs[i] >> 32);
            if (nodes.Count == 0 || nodes[^1] != own)
            {
                nodes.Add(own);
            }
        }
        int n = nodes.Count;
        var data = new ClusterData
        {
            Nodes = nodes.ToArray(),
            Intra = new double[n * n],
            InterOffset = new int[n + 1],
            InterTo = new int[_pairs.Count],
            InterCost = new double[_pairs.Count],
        };
        double[] m = _grid.Areas.Multipliers;
        byte[] cells = _grid.CellArray;
        int e = 0;
        for (int i = 0; i < n; i++)
        {
            data.InterOffset[i] = e;
            while (e < _pairs.Count && (int)(_pairs[e] >> 32) == data.Nodes[i])
            {
                int other = (int)(_pairs[e] & 0xFFFFFFFF);
                data.InterTo[e] = other;
                // Same formula as SquareTopology for a straight move.
                data.InterCost[e] = (m[cells[data.Nodes[i]]] + m[cells[other]]) * 0.5;
                e++;
            }
        }
        data.InterOffset[n] = e;

        ClusterRect(c, out int x0, out int y0, out int x1, out int y1);
        for (int i = 0; i < n; i++)
        {
            _solver.Run(x0, y0, x1, y1, data.Nodes[i], data.Nodes);
            for (int j = 0; j < n; j++)
            {
                data.Intra[(i * n) + j] = _solver.Dist(data.Nodes[j]);
            }
        }
        _clusters[c] = data;
    }

    // Entrances between chunk A (left or above) and chunk B (right or below): maximal runs of border positions where the
    // two cells facing each other are both walkable for the agent (Botea et al. 2004, section 3.1). Diagonal crossings
    // need no entrances of their own: under the supported diagonal rules a diagonal move across a border implies a
    // walkable straight crossing next to it, and cells of one run are connected inside both chunks.
    private void AddBorder(int ax, int ay, int bx, int by, bool ownIsFirst)
    {
        bool horizontal = bx == ax + 1;
        int start = horizontal ? ay * ChunkSize : ax * ChunkSize;
        int end = Math.Min(horizontal ? _height : _width, start + ChunkSize);
        int run = -1;
        for (int p = start; p <= end; p++)
        {
            bool open = p < end && IsOpen(horizontal, ax, ay, bx, by, p);
            if (open)
            {
                if (run < 0)
                {
                    run = p;
                }
            }
            else if (run >= 0)
            {
                int hi = p - 1;
                int len = hi - run + 1;
                if (len >= EntranceThreshold && hi != run)
                {
                    AddTransition(horizontal, ax, ay, bx, by, run, ownIsFirst);
                    AddTransition(horizontal, ax, ay, bx, by, hi, ownIsFirst);
                }
                else
                {
                    AddTransition(horizontal, ax, ay, bx, by, run + ((len - 1) / 2), ownIsFirst);
                }
                run = -1;
            }
        }
    }

    private bool IsOpen(bool horizontal, int ax, int ay, int bx, int by, int p)
    {
        GetPair(horizontal, ax, ay, bx, by, p, out int u, out int v);
        return _topology.IsWalkable(u) && _topology.IsWalkable(v);
    }

    private void GetPair(bool horizontal, int ax, int ay, int bx, int by, int p, out int u, out int v)
    {
        if (horizontal)
        {
            u = (p * _width) + (ax * ChunkSize) + ChunkSize - 1;
            v = (p * _width) + (bx * ChunkSize);
        }
        else
        {
            u = ((ay * ChunkSize) + ChunkSize - 1) * _width + p;
            v = (by * ChunkSize * _width) + p;
        }
    }

    private void AddTransition(bool horizontal, int ax, int ay, int bx, int by, int p, bool ownIsFirst)
    {
        GetPair(horizontal, ax, ay, bx, by, p, out int u, out int v);
        _pairs.Add(ownIsFirst ? ((long)u << 32) | (uint)v : ((long)v << 32) | (uint)u);
    }

    internal sealed class ClusterData
    {
        public int[] Nodes = [];
        public double[] Intra = [];
        public int[] InterOffset = [0];
        public int[] InterTo = [];
        public double[] InterCost = [];

        public bool SameAs(ClusterData o)
        {
            if (!Nodes.AsSpan().SequenceEqual(o.Nodes) || !InterOffset.AsSpan().SequenceEqual(o.InterOffset) || !InterTo.AsSpan().SequenceEqual(o.InterTo))
            {
                return false;
            }
            for (int i = 0; i < Intra.Length; i++)
            {
                if (BitConverter.DoubleToInt64Bits(Intra[i]) != BitConverter.DoubleToInt64Bits(o.Intra[i]))
                {
                    return false;
                }
            }
            for (int i = 0; i < InterCost.Length; i++)
            {
                if (BitConverter.DoubleToInt64Bits(InterCost[i]) != BitConverter.DoubleToInt64Bits(o.InterCost[i]))
                {
                    return false;
                }
            }
            return true;
        }

        public void Hash(ref ulong h)
        {
            Mix(ref h, (ulong)Nodes.Length);
            foreach (int v in Nodes)
            {
                Mix(ref h, (uint)v);
            }
            foreach (double v in Intra)
            {
                Mix(ref h, (ulong)BitConverter.DoubleToInt64Bits(v));
            }
            foreach (int v in InterOffset)
            {
                Mix(ref h, (uint)v);
            }
            foreach (int v in InterTo)
            {
                Mix(ref h, (uint)v);
            }
            foreach (double v in InterCost)
            {
                Mix(ref h, (ulong)BitConverter.DoubleToInt64Bits(v));
            }
        }

        private static void Mix(ref ulong h, ulong v)
        {
            for (int i = 0; i < 8; i++)
            {
                h ^= (byte)(v >> (i * 8));
                h *= 1099511628211UL;
            }
        }
    }
}
