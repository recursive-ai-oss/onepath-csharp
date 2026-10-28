using System;
using System.Threading;

namespace RecursiveAI.OnePath.Grids.Voxel;

// Receives the moves out of one voxel with the target's coordinates, so the search never decodes a node index.
internal interface IVoxelVisitor
{
    void Visit(int x, int y, int z, double baseCost);
}

/// <summary>
/// Working memory of a voxel A* whose size follows the search, not the grid. Cost, parent, mark and open-set position are
/// kept in pages of 8 × 8 × 8 voxels, allocated the first time the search reaches a voxel of the block; a dense table with
/// one int per block finds the page. A 5 × 10^8 voxel block costs a 4 MB table plus
/// 10 KB for each page the search touches, instead of 10 GB. Pages are pooled between searches.
/// <para>
/// The search is the same algorithm as <see cref="AStar{TTopology, TCost, THeuristic}"/>: the same open-list order
/// (smaller f, larger g, smaller linear node index), the same strict-improvement relaxation with reopening, the same
/// neighbour order, so costs, paths, expansion order and iteration counts are equal; only where the numbers live differs.
/// No hash collection is involved, so nothing depends on enumeration order.
/// </para>
/// </summary>
internal sealed class VoxelSearchState
{
    // A page is an Edge x Edge x Edge block of voxels; its slot number is (page << PageShift) | in-page index.
    private const int EdgeBits = 3;
    private const int Edge = 1 << EdgeBits;
    private const int EdgeMask = Edge - 1;
    private const int PageShift = 3 * EdgeBits;
    private const int PageSize = 1 << PageShift;
    private const int PageMask = PageSize - 1;

    private struct Entry
    {
        public double F;
        public double G;
        public int Node; // linear node index: the final tie-break
        public int Slot; // (page << 12) | in-chunk index
    }

    private readonly int _sizeX;
    private readonly int _sizeY;
    private readonly int _chunksX;
    private readonly int _chunksY;
    private readonly int[] _pageOf; // chunk index -> page, or -1
    private double[][] _g = new double[4][];
    private int[][] _parent = new int[4][]; // parent slot, or -1 for the start
    private uint[][] _mark = new uint[4][];
    private int[][] _pos = new int[4][]; // heap position, or -1
    private int[] _pageChunk = new int[4];
    private int[] _pageX = new int[4]; // voxel coordinates of the page's corner
    private int[] _pageY = new int[4];
    private int[] _pageZ = new int[4];
    private int _pages; // pages in use by the current search
    private int _allocated; // pages ever allocated (pooled)
    private uint _base;
    private Entry[] _heap = new Entry[64];
    private int _count;
    private int[] _expanded = [];
    private int _expandedCount;

    internal VoxelSearchState(int sizeX, int sizeY, int sizeZ)
    {
        _sizeX = sizeX;
        _sizeY = sizeY;
        _chunksX = (sizeX + EdgeMask) >> EdgeBits;
        _chunksY = (sizeY + EdgeMask) >> EdgeBits;
        _pageOf = new int[_chunksX * _chunksY * ((sizeZ + EdgeMask) >> EdgeBits)];
        Array.Fill(_pageOf, -1);
    }

    /// <summary>Pages allocated so far (each holds 20 bytes for each of 4096 voxels).</summary>
    internal int PooledPages => _allocated;

    internal bool Fits(VoxelGrid grid) => grid.SizeX == _sizeX && grid.SizeY == _sizeY && _pageOf.Length == _chunksX * _chunksY * ((grid.SizeZ + EdgeMask) >> EdgeBits);

    internal ReadOnlySpan<int> ExpandedSlots => new(_expanded, 0, _expandedCount);

    internal VoxelCell CellOfSlot(int slot)
    {
        int p = slot >> PageShift;
        return new VoxelCell(_pageX[p] + (slot & EdgeMask), _pageY[p] + ((slot >> EdgeBits) & EdgeMask), _pageZ[p] + ((slot >> (2 * EdgeBits)) & EdgeMask));
    }

    internal int ParentOf(int slot) => _parent[slot >> PageShift][slot & PageMask];

    private void Begin()
    {
        for (int i = 0; i < _count; i++)
        {
            int s = _heap[i].Slot;
            _pos[s >> PageShift][s & PageMask] = -1;
        }
        _count = 0;
        for (int p = 0; p < _pages; p++)
        {
            _pageOf[_pageChunk[p]] = -1;
        }
        _pages = 0;
        if (_base >= uint.MaxValue - 4)
        {
            for (int p = 0; p < _allocated; p++)
            {
                Array.Clear(_mark[p]);
            }
            _base = 0;
        }
        _base += 2;
        _expandedCount = 0;
    }

    private int SlotOf(int x, int y, int z)
    {
        int cx = x >> EdgeBits;
        int cy = y >> EdgeBits;
        int cz = z >> EdgeBits;
        int ci = cx + (_chunksX * (cy + (_chunksY * cz)));
        int p = _pageOf[ci];
        if (p < 0)
        {
            p = NewPage(ci, cx << EdgeBits, cy << EdgeBits, cz << EdgeBits);
        }
        return (p << PageShift) | (x & EdgeMask) | ((y & EdgeMask) << EdgeBits) | ((z & EdgeMask) << (2 * EdgeBits));
    }

    private int NewPage(int chunk, int x0, int y0, int z0)
    {
        int p = _pages++;
        if (p == _allocated)
        {
            if (_allocated == _g.Length)
            {
                int n = _allocated * 2;
                Array.Resize(ref _g, n);
                Array.Resize(ref _parent, n);
                Array.Resize(ref _mark, n);
                Array.Resize(ref _pos, n);
                Array.Resize(ref _pageChunk, n);
                Array.Resize(ref _pageX, n);
                Array.Resize(ref _pageY, n);
                Array.Resize(ref _pageZ, n);
            }
            _g[p] = new double[PageSize];
            _parent[p] = new int[PageSize];
            _mark[p] = new uint[PageSize];
            int[] pos = new int[PageSize];
            Array.Fill(pos, -1);
            _pos[p] = pos;
            _allocated++;
        }
        _pageChunk[p] = chunk;
        _pageX[p] = x0;
        _pageY[p] = y0;
        _pageZ[p] = z0;
        _pageOf[chunk] = p;
        return p;
    }

    /// <summary>
    /// Runs the search for at most <paramref name="budget"/> iterations (one iteration pops and expands one voxel).
    /// Returns Found with <paramref name="goalSlot"/> set, NotFound, or Running when the budget ran out.
    /// </summary>
    internal SearchStatus Run<TCost>(
        VoxelTopology topology,
        TCost cost,
        VoxelHeuristic heuristic,
        VoxelCell start,
        VoxelCell goal,
        double weight,
        int budget,
        bool record,
        out int goalSlot,
        out double pathCost,
        out int expandedCount,
        CancellationToken token = default)
        where TCost : struct, IStepCost
    {
        Begin();
        heuristic.SetGoal(goal.X + (_sizeX * (goal.Y + (_sizeY * goal.Z))));
        int goalNode = goal.X + (_sizeX * (goal.Y + (_sizeY * goal.Z)));
        int startNode = start.X + (_sizeX * (start.Y + (_sizeY * start.Z)));
        int startSlot = SlotOf(start.X, start.Y, start.Z);
        _g[startSlot >> PageShift][startSlot & PageMask] = 0.0;
        _parent[startSlot >> PageShift][startSlot & PageMask] = -1;
        _mark[startSlot >> PageShift][startSlot & PageMask] = _base;
        Push(startSlot, startNode, weight * heuristic.EstimateAt(start.X, start.Y, start.Z), 0.0);

        var relaxer = new Relaxer<TCost> { Owner = this, Cost = cost, Heuristic = heuristic, Weight = weight };
        int expanded = 0;
        goalSlot = -1;
        pathCost = double.PositiveInfinity;
        bool cancellable = token.CanBeCanceled;
        while (budget > 0)
        {
            if (cancellable && (expanded & 255) == 0)
            {
                token.ThrowIfCancellationRequested();
            }
            if (_count == 0)
            {
                expandedCount = expanded;
                return SearchStatus.NotFound;
            }
            Entry top = _heap[0];
            Pop();
            budget--;
            int page = top.Slot >> PageShift;
            int local = top.Slot & PageMask;
            _mark[page][local] = _base + 1;
            if (top.Node == goalNode)
            {
                goalSlot = top.Slot;
                pathCost = _g[page][local];
                expandedCount = expanded;
                return SearchStatus.Found;
            }
            expanded++;
            if (record)
            {
                if (_expandedCount == _expanded.Length)
                {
                    Array.Resize(ref _expanded, Math.Max(256, _expanded.Length * 2));
                }
                _expanded[_expandedCount++] = top.Slot;
            }
            relaxer.FromSlot = top.Slot;
            relaxer.FromNode = top.Node;
            relaxer.GFrom = _g[page][local];
            topology.VisitNeighborsAt(_pageX[page] + (local & EdgeMask), _pageY[page] + ((local >> EdgeBits) & EdgeMask), _pageZ[page] + ((local >> (2 * EdgeBits)) & EdgeMask), ref relaxer);
        }
        expandedCount = expanded;
        return SearchStatus.Running;
    }

    // Take the neighbour if this route is strictly cheaper; ties keep the first parent found (the earlier move in the topology's order).
    private void Relax<TCost>(int fromSlot, int fromNode, double gFrom, int x, int y, int z, double baseCost, ref TCost cost, ref VoxelHeuristic heuristic, double weight)
        where TCost : struct, IStepCost
    {
        int toNode = x + (_sizeX * (y + (_sizeY * z)));
        double step = cost.Cost(fromNode, toNode, baseCost);
        if (!(step >= 0.0) || double.IsPositiveInfinity(step))
        {
            return;
        }
        double g = gFrom + step;
        int slot = SlotOf(x, y, z);
        int page = slot >> PageShift;
        int local = slot & PageMask;
        uint[] mark = _mark[page];
        if (mark[local] < _base)
        {
            mark[local] = _base;
        }
        else if (!(g < _g[page][local]))
        {
            return;
        }
        else if (mark[local] != _base)
        {
            mark[local] = _base; // reopen a closed voxel
        }
        _g[page][local] = g;
        _parent[page][local] = fromSlot;
        Push(slot, toNode, g + (weight * heuristic.EstimateAt(x, y, z)), g);
    }

    private struct Relaxer<TCost> : IVoxelVisitor
        where TCost : struct, IStepCost
    {
        public VoxelSearchState Owner;
        public TCost Cost;
        public VoxelHeuristic Heuristic;
        public double Weight;
        public int FromSlot;
        public int FromNode;
        public double GFrom;

        public void Visit(int x, int y, int z, double baseCost) => Owner.Relax(FromSlot, FromNode, GFrom, x, y, z, baseCost, ref Cost, ref Heuristic, Weight);
    }

    // The open list: the same indexed 4-ary heap and order as OpenSet, with positions kept in the pages.
    private static bool Less(in Entry a, in Entry b)
    {
        if (a.F != b.F)
        {
            return a.F < b.F;
        }
        if (a.G != b.G)
        {
            return a.G > b.G;
        }
        return a.Node < b.Node;
    }

    private void SetPos(int slot, int heapSlot) => _pos[slot >> PageShift][slot & PageMask] = heapSlot;

    private void Push(int slot, int node, double f, double g)
    {
        if (double.IsNaN(f) || double.IsNaN(g))
        {
            throw new ArgumentException("Open set keys must not be NaN.", nameof(f));
        }
        var e = new Entry { F = f, G = g, Node = node, Slot = slot };
        int at = _pos[slot >> PageShift][slot & PageMask];
        if (at < 0)
        {
            if (_count == _heap.Length)
            {
                Array.Resize(ref _heap, _heap.Length * 2);
            }
            at = _count++;
            SiftUp(at, e);
            return;
        }
        if (Less(in e, in _heap[at]))
        {
            SiftUp(at, e);
        }
        else
        {
            SiftDown(at, e);
        }
    }

    private void Pop()
    {
        SetPos(_heap[0].Slot, -1);
        _count--;
        if (_count > 0)
        {
            SiftDown(0, _heap[_count]);
        }
    }

    private void SiftUp(int at, Entry e)
    {
        while (at > 0)
        {
            int parent = (at - 1) >> 2;
            if (!Less(in e, in _heap[parent]))
            {
                break;
            }
            _heap[at] = _heap[parent];
            SetPos(_heap[at].Slot, at);
            at = parent;
        }
        _heap[at] = e;
        SetPos(e.Slot, at);
    }

    private void SiftDown(int at, Entry e)
    {
        int count = _count;
        while (true)
        {
            int first = (at << 2) + 1;
            if (first >= count)
            {
                break;
            }
            int best = first;
            int last = Math.Min(first + 4, count);
            for (int c = first + 1; c < last; c++)
            {
                if (Less(in _heap[c], in _heap[best]))
                {
                    best = c;
                }
            }
            if (!Less(in _heap[best], in e))
            {
                break;
            }
            _heap[at] = _heap[best];
            SetPos(_heap[at].Slot, at);
            at = best;
        }
        _heap[at] = e;
        SetPos(e.Slot, at);
    }
}
