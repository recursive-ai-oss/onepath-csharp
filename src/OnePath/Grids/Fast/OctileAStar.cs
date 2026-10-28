using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// A* (Hart, Nilsson &amp; Raphael 1968) specialised for the common grid query: eight moves, no corner cutting, agent size 1, one
/// cost for every walkable cell, an exact octile (or zero) estimate. It finds the same optimal costs as
/// <see cref="AStar{TTopology, TCost, THeuristic}"/>; what differs is speed (padded byte map without bounds tests, 16-byte
/// node records, a heap without decrease-key) and the tie-break, which is exact here (see <see cref="SearchNode"/>): on equal
/// <c>f</c> the node with the larger cost so far pops first, then the smaller node index. The goal test happens when the goal pops.
/// Costs are computed for unit steps; callers multiply by the cell cost.
/// <para>Not thread-safe. The grid must not change while a search runs; edits between searches are picked up.</para>
/// </summary>
internal sealed class OctileAStar
{
    private const double Sqrt2 = GridMath.Sqrt2;

    // A wall is a record whose tag is "closed" in every search, so relaxing it does nothing and the map needs no second array.
    private const uint WallTag = uint.MaxValue;

    private readonly Grid _grid;
    private readonly int _w;
    private readonly int _h;
    private readonly int _stride; // padded row length
    private readonly AStarNode[] _nodes;
    private readonly IndexedHeap _open;
    private readonly List<int> _expanded = [];
    private int _version = -1;
    private uint _generation;

    private int _goal;
    private int _startInner;
    private int _gx;
    private int _gy;
    private bool _useEstimate;
    private int _expandedCount;
    private int _iterations;
    private SearchStatus _status;
    private int _pathA;
    private int _pathB;

    public OctileAStar(Grid grid)
    {
        _grid = grid;
        _w = grid.Width;
        _h = grid.Height;
        _stride = _w + 2;
        long cells = (long)_stride * (_h + 2);
        if (cells > int.MaxValue)
        {
            throw new ArgumentOutOfRangeException(nameof(grid), "The grid is too large for this search.");
        }
        _nodes = new AStarNode[cells];
        Array.Fill(_nodes, new AStarNode { Tag = WallTag }); // the padding all round stays a wall
        _open = new IndexedHeap(_nodes);
    }

    public SearchStatus Status => _status;

    public int ExpandedCount => _expandedCount;

    public int Iterations => _iterations;

    /// <summary>The expanded cells in order (cell indexes), when recording was requested.</summary>
    public List<int> Expanded => _expanded;

    /// <summary>Cost of the found path for unit steps.</summary>
    public double PathCost => _status == SearchStatus.Found ? _pathA + (_pathB * Sqrt2) : double.PositiveInfinity;

    private void Refresh()
    {
        if (_version == _grid.Version)
        {
            return;
        }
        byte[] cells = _grid.CellArray;
        AStarNode[] nodes = _nodes;
        for (int y = 0; y < _h; y++)
        {
            int src = y * _w;
            int dst = ((y + 1) * _stride) + 1;
            for (int x = 0; x < _w; x++)
            {
                if (cells[src + x] == AreaCosts.Wall)
                {
                    nodes[dst + x].Tag = WallTag;
                }
                else if (nodes[dst + x].Tag == WallTag)
                {
                    nodes[dst + x].Tag = 0;
                }
            }
        }
        _version = _grid.Version;
    }

    private int ToInner(int cell)
    {
        int y = cell / _w;
        return ((y + 1) * _stride) + 1 + (cell - (y * _w));
    }

    private int ToCell(int inner)
    {
        int py = inner / _stride;
        return ((py - 1) * _w) + (inner - (py * _stride)) - 1;
    }

    /// <summary>Searches from <paramref name="start"/> to <paramref name="goal"/> (cell indexes, both walkable).</summary>
    /// <param name="start">Start cell.</param>
    /// <param name="goal">Goal cell.</param>
    /// <param name="zeroEstimate">True for Dijkstra's algorithm.</param>
    /// <param name="budget">Most iterations (pops of live nodes); the status is Running when it runs out.</param>
    /// <param name="record">Record expanded cells in <see cref="Expanded"/>.</param>
    public SearchStatus Run(int start, int goal, bool zeroEstimate, int budget, bool record)
    {
        if (_generation >= (uint.MaxValue >> 4) - 2)
        {
            Array.Fill(_nodes, new AStarNode { Tag = WallTag });
            _generation = 0;
            _version = -1; // the walls are marked again below
        }
        Refresh();
        _generation++;
        _expanded.Clear();
        _open.Clear();
        _expandedCount = 0;
        _iterations = 0;
        _useEstimate = !zeroEstimate;
        int s = ToInner(start);
        int t = ToInner(goal);
        _goal = t;
        _startInner = s;
        _gy = t / _stride;
        _gx = t - (_gy * _stride);
        _nodes[s] = new AStarNode { A = 0, B = 0, Tag = _generation << 4 };
        _open.Insert(Estimate(0, 0, s), 0f, s);
        _status = SearchStatus.Running;
        return Loop(budget, record);
    }

    private double Estimate(int a, int b, int node)
    {
        if (!_useEstimate)
        {
            return a + (b * Sqrt2);
        }
        int y = node / _stride;
        int x = node - (y * _stride);
        return Key(a, b, x, y);
    }

    // f = (a + b√2) + octile((x, y) → goal) with the √2 terms gathered so that equal pairs give equal doubles.
    private double Key(int a, int b, int x, int y)
    {
        if (!_useEstimate)
        {
            return a + (b * Sqrt2);
        }
        int dx = x - _gx;
        if (dx < 0)
        {
            dx = -dx;
        }
        int dy = y - _gy;
        if (dy < 0)
        {
            dy = -dy;
        }
        int min = dx < dy ? dx : dy;
        int max = dx < dy ? dy : dx;
        return (a + (max - min)) + ((b + min) * Sqrt2);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private SearchStatus Loop(int budget, bool record)
    {
        AStarNode[] nodes = _nodes;
        IndexedHeap open = _open;
        uint closedTag = (_generation << 4) | 1u;
        uint openBase = _generation << 4;
        int stride = _stride;
        while (budget > 0)
        {
            if (!open.Pop(out HeapItem item))
            {
                _status = SearchStatus.NotFound;
                return _status;
            }
            int node = item.Node;
            budget--;
            _iterations++;
            nodes[node].Tag = closedTag;
            int a = nodes[node].A;
            int b = nodes[node].B;
            if (node == _goal)
            {
                _pathA = a;
                _pathB = b;
                _status = SearchStatus.Found;
                return _status;
            }
            _expandedCount++;
            if (record)
            {
                _expanded.Add(ToCell(node));
            }
            int y = node / stride;
            int x = node - (y * stride);
            ref AStarNode rn = ref nodes[node - stride];
            ref AStarNode re = ref nodes[node + 1];
            ref AStarNode rs = ref nodes[node + stride];
            ref AStarNode rw = ref nodes[node - 1];
            bool n = rn.Tag != WallTag;
            bool e = re.Tag != WallTag;
            bool s = rs.Tag != WallTag;
            bool w = rw.Tag != WallTag;
            if (n)
            {
                Relax(ref rn, open, openBase, 0, node - stride, a + 1, b, x, y - 1);
            }
            if (e)
            {
                Relax(ref re, open, openBase, 1, node + 1, a + 1, b, x + 1, y);
            }
            if (s)
            {
                Relax(ref rs, open, openBase, 2, node + stride, a + 1, b, x, y + 1);
            }
            if (w)
            {
                Relax(ref rw, open, openBase, 3, node - 1, a + 1, b, x - 1, y);
            }
            // Diagonal moves need both cells beside the move and the target free (no corner cutting).
            if (n && e)
            {
                ref AStarNode r = ref nodes[node - stride + 1];
                if (r.Tag != WallTag)
                {
                    Relax(ref r, open, openBase, 4, node - stride + 1, a, b + 1, x + 1, y - 1);
                }
            }
            if (s && e)
            {
                ref AStarNode r = ref nodes[node + stride + 1];
                if (r.Tag != WallTag)
                {
                    Relax(ref r, open, openBase, 5, node + stride + 1, a, b + 1, x + 1, y + 1);
                }
            }
            if (s && w)
            {
                ref AStarNode r = ref nodes[node + stride - 1];
                if (r.Tag != WallTag)
                {
                    Relax(ref r, open, openBase, 6, node + stride - 1, a, b + 1, x - 1, y + 1);
                }
            }
            if (n && w)
            {
                ref AStarNode r = ref nodes[node - stride - 1];
                if (r.Tag != WallTag)
                {
                    Relax(ref r, open, openBase, 7, node - stride - 1, a, b + 1, x - 1, y - 1);
                }
            }
        }
        return _status;
    }

    // Take the target if this route is strictly cheaper than what it has (ties keep the first parent found).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Relax(ref AStarNode r, IndexedHeap open, uint openBase, int dir, int to, int a, int b, int x, int y)
    {
        double g = a + (b * Sqrt2);
        if (r.Tag < openBase)
        {
            r.A = a;
            r.B = b;
            r.Tag = openBase | (uint)(dir << 1);
            open.Insert(Key(a, b, x, y), (float)g, to);
            return;
        }
        if ((r.Tag & 1u) != 0 || !(g < r.A + (r.B * Sqrt2)))
        {
            return; // closed (the estimate is consistent, so a closed node is final) or not better
        }
        r.A = a;
        r.B = b;
        r.Tag = openBase | (uint)(dir << 1);
        open.Decrease(r.Pos, Key(a, b, x, y), (float)g, to);
    }

    private int Parent(int n)
    {
        int stride = _stride;
        switch ((_nodes[n].Tag >> 1) & 7)
        {
            case 0: return n + stride;
            case 1: return n - 1;
            case 2: return n - stride;
            case 3: return n + 1;
            case 4: return n + stride - 1;
            case 5: return n - stride - 1;
            case 6: return n - stride + 1;
            default: return n + stride + 1;
        }
    }

    /// <summary>Number of cells on the found path, start and goal included.</summary>
    public int PathLength()
    {
        int count = 1;
        for (int n = _goal; n != _startInner; n = Parent(n))
        {
            count++;
        }
        return count;
    }

    /// <summary>Writes the found path as cell indexes, start first; <paramref name="destination"/> holds at least <see cref="PathLength"/> entries.</summary>
    public void CopyPath(Span<int> destination)
    {
        int i = PathLength();
        for (int n = _goal; ; n = Parent(n))
        {
            destination[--i] = ToCell(n);
            if (n == _startInner)
            {
                break;
            }
        }
    }
}
