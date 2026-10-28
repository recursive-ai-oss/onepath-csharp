using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.Grids;

/// <summary>The part of a jump point search that finders share: node records, the open list, the goal, and offering a successor.</summary>
internal sealed class JumpCore
{
    private const double Sqrt2 = GridMath.Sqrt2;

    public readonly SearchNode[] Nodes;
    public readonly FastHeap Heap = new();
    public readonly int Width;
    public uint OpenBase;
    public int Goal;
    public int GoalX;
    public int GoalY;
    public bool UseEstimate;

    public JumpCore(int width, int height)
    {
        Width = width;
        Nodes = new SearchNode[(long)width * height];
    }

    // f = (a + b√2) + octile((x, y) → goal), gathered as (a + max - min) + (b + min)√2 so equal pairs give equal doubles.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public double Key(int a, int b, int x, int y)
    {
        int dx = x - GoalX;
        if (dx < 0)
        {
            dx = -dx;
        }
        int dy = y - GoalY;
        if (dy < 0)
        {
            dy = -dy;
        }
        int min = dx < dy ? dx : dy;
        int max = dx < dy ? dy : dx;
        if (!UseEstimate)
        {
            min = 0;
            max = 0;
        }
        return (a + (max - min)) + ((b + min) * Sqrt2);
    }

    /// <summary>
    /// Offers a successor reached from <paramref name="parent"/> at cost pair (<paramref name="a"/>, <paramref name="b"/>); it is taken if the cell is
    /// new or this route is strictly cheaper (ties keep the first parent). <paramref name="dir"/> is the direction of the last move.
    /// A closed node stays closed: the estimate is consistent, so its cost is final.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Offer(int to, int a, int b, int x, int y, int dir, int parent)
    {
        ref SearchNode r = ref Nodes[to];
        double g = a + (b * Sqrt2);
        if (r.Tag >= OpenBase && ((r.Tag & 1u) != 0 || !(g < r.A + (r.B * Sqrt2))))
        {
            return;
        }
        r.A = a;
        r.B = b;
        r.Parent = parent;
        r.Tag = OpenBase | (uint)(dir << 1);
        Heap.Push(Key(a, b, x, y), (float)g, to);
    }
}

/// <summary>Finds the successors of an expanded node (jump points) and offers them to the <see cref="JumpCore"/>.</summary>
internal interface IJumpFinder
{
    /// <param name="core">Receives the successors.</param>
    /// <param name="node">The expanded cell.</param>
    /// <param name="x">Its column.</param>
    /// <param name="y">Its row.</param>
    /// <param name="a">Straight steps of its cost.</param>
    /// <param name="b">Diagonal steps of its cost.</param>
    /// <param name="arrival">Direction of the move that reached it, or -1 for the start.</param>
    void Expand(JumpCore core, int node, int x, int y, int a, int b, int arrival);
}

/// <summary>
/// Jump point search (Harabor &amp; Grastien 2011) on an eight-connected grid without corner cutting, over any <see cref="IJumpFinder"/>:
/// <see cref="BitFinder"/> scans 64 cells at a time (Harabor &amp; Grastien 2014), <see cref="TableFinder"/> reads the JPS+ tables.
/// Costs are exact pairs (see <see cref="SearchNode"/>); the open list orders by smaller <c>f</c>, larger cost so far, smaller node index.
/// Costs are for unit steps; callers multiply by the cell cost. Not thread-safe.
/// </summary>
internal sealed partial class OctileJump<TFinder> : IOctileJumpEngine
    where TFinder : struct, IJumpFinder
{
    private const double Sqrt2 = GridMath.Sqrt2;

    private readonly JumpCore _core;
    private readonly TFinder _finder;
    private readonly int _w;
    private readonly List<int> _expanded = [];
    private uint _generation;
    private int _expandedCount;
    private int _iterations;
    private SearchStatus _status;
    private int _pathA;
    private int _pathB;
    private int _start = -1;

    public OctileJump(TFinder finder, int width, int height)
    {
        _finder = finder;
        _w = width;
        _core = new JumpCore(width, height);
    }

    public SearchStatus Status => _status;

    public int ExpandedCount => _expandedCount;

    public int Iterations => _iterations;

    public List<int> Expanded => _expanded;

    public double PathCost => _status == SearchStatus.Found ? _pathA + (_pathB * Sqrt2) : double.PositiveInfinity;

    public SearchStatus Run(int start, int goal, bool zeroEstimate, int budget, bool record)
    {
        Begin(start, goal, zeroEstimate);
        return Loop(budget, record);
    }

    /// <summary>Starts a search that <see cref="Step"/> then advances; <see cref="Run"/> is both in one call.</summary>
    public void Begin(int start, int goal, bool zeroEstimate)
    {
        JumpCore core = _core;
        NextGeneration();
        core.Heap.Clear();
        _expanded.Clear();
        _expandedCount = 0;
        _iterations = 0;
        _pathA = 0;
        _pathB = 0;
        core.UseEstimate = !zeroEstimate;
        core.Goal = goal;
        core.GoalY = goal / _w;
        core.GoalX = goal - (core.GoalY * _w);
        _start = start;
        core.Nodes[start] = new SearchNode { A = 0, B = 0, Parent = -1, Tag = core.OpenBase };
        int sy = start / _w;
        core.Heap.Push(core.Key(0, 0, start - (sy * _w), sy), 0f, start);
        _status = SearchStatus.Running;
    }

    /// <summary>Continues by at most <paramref name="budget"/> pops of live nodes; a finished search stays as it is.</summary>
    public SearchStatus Step(int budget) => _status == SearchStatus.Running ? Loop(budget, false) : _status;

    // A new generation makes every node record of the earlier search stale without clearing the array.
    private void NextGeneration()
    {
        if (_generation >= (uint.MaxValue >> 4) - 2)
        {
            Array.Clear(_core.Nodes);
            _generation = 0;
        }
        _generation++;
        _core.OpenBase = _generation << 4;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private SearchStatus Loop(int budget, bool record)
    {
        JumpCore core = _core;
        SearchNode[] nodes = core.Nodes;
        FastHeap heap = core.Heap;
        TFinder finder = _finder;
        int w = _w;
        ulong recip = w == 1 ? 0 : (ulong.MaxValue / (ulong)w) + 1; // 2^64 / 1 does not fit; a one-column grid takes the row from the index
        while (budget > 0)
        {
            if (!heap.Pop(out HeapItem item))
            {
                _status = SearchStatus.NotFound;
                return _status;
            }
            int node = item.Node;
            ref SearchNode rec = ref nodes[node];
            if ((rec.Tag & 1u) != 0)
            {
                continue; // an older, worse entry of a node that already closed
            }
            budget--;
            _iterations++;
            rec.Tag |= 1u;
            int a = rec.A;
            int b = rec.B;
            if (node == core.Goal)
            {
                _pathA = a;
                _pathB = b;
                _status = SearchStatus.Found;
                return _status;
            }
            _expandedCount++;
            if (record)
            {
                _expanded.Add(node);
            }
            int y = w == 1 ? node : (int)Math.BigMul((ulong)node, recip, out _); // node / w without a division: recip = ceil(2^64 / w), exact for node < 2^32
            int x = node - (y * w);
            int arrival = rec.Parent < 0 ? -1 : (int)((rec.Tag >> 1) & 7u);
            finder.Expand(core, node, x, y, a, b, arrival);
        }
        return _status;
    }

    /// <summary>Number of cells on the found path (every cell, not only the jump points), start and goal included.</summary>
    public int PathLength()
    {
        int count = 1;
        int to = _core.Goal;
        while (_core.Nodes[to].Parent >= 0)
        {
            int from = _core.Nodes[to].Parent;
            count += Segment(from, to, out _, out _, out _);
            to = from;
        }
        return count;
    }

    /// <summary>Writes the path, start first. A hop from a jump point to its successor is walked diagonal steps first, then straight.</summary>
    public void CopyPath(Span<int> destination)
    {
        int i = PathLength();
        int to = _core.Goal;
        destination[--i] = to;
        while (_core.Nodes[to].Parent >= 0)
        {
            int from = _core.Nodes[to].Parent;
            int steps = Segment(from, to, out int diagonal, out int diagonalDelta, out int straightDelta);
            // Walk back from `to`: the straight part comes last on the way out, so it is written first here.
            int cell = to;
            for (int k = 0; k < steps; k++)
            {
                int straightSteps = steps - diagonal;
                cell -= k < straightSteps ? straightDelta : diagonalDelta;
                destination[--i] = cell;
            }
            to = from;
        }
    }

    // Steps of the hop between two cells of the octile route "diagonal first": min(|dx|, |dy|) diagonal steps, then the rest straight.
    private int Segment(int from, int to, out int diagonal, out int diagonalDelta, out int straightDelta)
    {
        int fy = from / _w;
        int ty = to / _w;
        int dx = (to - (ty * _w)) - (from - (fy * _w));
        int dy = ty - fy;
        int ax = dx < 0 ? -dx : dx;
        int ay = dy < 0 ? -dy : dy;
        int sx = Math.Sign(dx);
        int sy = Math.Sign(dy);
        diagonal = ax < ay ? ax : ay;
        diagonalDelta = sx + (sy * _w);
        straightDelta = ax > ay ? sx : sy * _w;
        return ax > ay ? ax : ay;
    }
}

/// <summary>Jump points found by scanning bit rows and columns, one 64-cell word at a time (JPS with block-based symmetry breaking).</summary>
internal readonly struct BitFinder : IJumpFinder
{
    private readonly OctileMap _map;
    private readonly ulong[] _rows;
    private readonly ulong[] _cols;
    private readonly ulong[] _rowFwd;
    private readonly ulong[] _rowBwd;
    private readonly ulong[] _colFwd;
    private readonly ulong[] _colBwd;
    private readonly byte[] _nb;
    private readonly int _rs;
    private readonly int _cs;
    private readonly int _w;
    private readonly bool _improved;

    /// <param name="map">The walkability forms.</param>
    /// <param name="improvedPruning">Prune intermediate diagonal jump points (Harabor &amp; Grastien 2014): a diagonal run offers every straight jump point it passes instead of stopping at the cells in between.</param>
    public BitFinder(OctileMap map, bool improvedPruning)
    {
        _map = map;
        _rows = map.Rows;
        _cols = map.Cols;
        _rowFwd = map.RowStopForward;
        _rowBwd = map.RowStopBackward;
        _colFwd = map.ColStopForward;
        _colBwd = map.ColStopBackward;
        _nb = map.Neighbors;
        _rs = map.RowStride;
        _cs = map.ColStride;
        _w = map.Width;
        _improved = improvedPruning;
    }

    public OctileMap Map => _map;

    // Steps to the first jump point (or the goal) going east/west along row y from column x, 0 if none.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Horizontal(JumpCore core, int x, int y, int dx)
    {
        int goalPos = core.GoalY == y ? core.GoalX + 64 : -1;
        int self = (y + 1) * _rs;
        return dx > 0
            ? OctileMap.ScanForward(_rows, _rowFwd, self, x + 64, goalPos)
            : OctileMap.ScanBackward(_rows, _rowBwd, self, x + 64, goalPos);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Vertical(JumpCore core, int x, int y, int dy)
    {
        int goalPos = core.GoalX == x ? core.GoalY + 64 : -1;
        int self = (x + 1) * _cs;
        return dy > 0
            ? OctileMap.ScanForward(_cols, _colFwd, self, y + 64, goalPos)
            : OctileMap.ScanBackward(_cols, _colBwd, self, y + 64, goalPos);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Expand(JumpCore core, int node, int x, int y, int a, int b, int arrival)
    {
        int nb = _nb[node];
        int mask = arrival < 0 ? nb : OctileMap.Successors[(arrival << 8) | nb];
        int w = _w;
        while (mask != 0)
        {
            int d = BitOperations.TrailingZeroCount(mask);
            mask &= mask - 1;
            switch (d)
            {
                case 0:
                {
                    int s = Vertical(core, x, y, -1);
                    if (s > 0)
                    {
                        core.Offer(node - (s * w), a + s, b, x, y - s, 0, node);
                    }
                    break;
                }
                case 1:
                {
                    int s = Horizontal(core, x, y, 1);
                    if (s > 0)
                    {
                        core.Offer(node + s, a + s, b, x + s, y, 1, node);
                    }
                    break;
                }
                case 2:
                {
                    int s = Vertical(core, x, y, 1);
                    if (s > 0)
                    {
                        core.Offer(node + (s * w), a + s, b, x, y + s, 2, node);
                    }
                    break;
                }
                case 3:
                {
                    int s = Horizontal(core, x, y, -1);
                    if (s > 0)
                    {
                        core.Offer(node - s, a + s, b, x - s, y, 3, node);
                    }
                    break;
                }
                default:
                    if (_improved)
                    {
                        DiagonalImproved(core, node, x, y, a, b, d);
                    }
                    else
                    {
                        Diagonal(core, node, x, y, a, b, d);
                    }
                    break;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void Diagonal(JumpCore core, int node, int x, int y, int a, int b, int d)
    {
        int dx = OctileMap.Dx[d];
        int dy = OctileMap.Dy[d];
        int need = OctileMap.DiagonalNeed[d];
        int delta = dx + (dy * _w);
        int c = node;
        int n = 0;
        while (true)
        {
            // No corner cutting: both cells beside the move and the target must be free.
            if ((_nb[c] & need) != need)
            {
                return;
            }
            c += delta;
            x += dx;
            y += dy;
            n++;
            if (c == core.Goal)
            {
                core.Offer(c, a, b + n, x, y, d, node);
                return;
            }
            if (Horizontal(core, x, y, dx) > 0 || Vertical(core, x, y, dy) > 0)
            {
                core.Offer(c, a, b + n, x, y, d, node);
                return;
            }
        }
    }

    // Improved pruning (Harabor & Grastien 2014): a cell of the diagonal run is only a turning point; offer the straight jump points it leads to
    // and keep going. Same scans as Diagonal, with the line indices and stop arrays kept incrementally instead of recomputed per cell.
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void DiagonalImproved(JumpCore core, int node, int x, int y, int a, int b, int d)
    {
        int dx = OctileMap.Dx[d];
        int dy = OctileMap.Dy[d];
        int need = OctileMap.DiagonalNeed[d];
        int delta = dx + (dy * _w);
        int hDir = dx > 0 ? 1 : 3;
        int vDir = dy > 0 ? 2 : 0;
        ulong[] rowStops = dx > 0 ? _rowFwd : _rowBwd;
        ulong[] colStops = dy > 0 ? _colFwd : _colBwd;
        ulong[] rows = _rows;
        ulong[] cols = _cols;
        byte[] nbs = _nb;
        int goalX = core.GoalX;
        int goalY = core.GoalY;
        int rowSelf = (y + 1) * _rs;
        int colSelf = (x + 1) * _cs;
        int rowStep = dy * _rs;
        int colStep = dx * _cs;
        int c = node;
        int n = 0;
        while (true)
        {
            // No corner cutting: both cells beside the move and the target must be free.
            if ((nbs[c] & need) != need)
            {
                return;
            }
            c += delta;
            x += dx;
            y += dy;
            rowSelf += rowStep;
            colSelf += colStep;
            n++;
            if (c == core.Goal)
            {
                core.Offer(c, a, b + n, x, y, d, node);
                return;
            }
            int h = dx > 0
                ? OctileMap.ScanForward(rows, rowStops, rowSelf, x + 64, goalY == y ? goalX + 64 : -1)
                : OctileMap.ScanBackward(rows, rowStops, rowSelf, x + 64, goalY == y ? goalX + 64 : -1);
            if (h > 0)
            {
                core.Offer(c + (dx * h), a + h, b + n, x + (dx * h), y, hDir, node);
            }
            int v = dy > 0
                ? OctileMap.ScanForward(cols, colStops, colSelf, y + 64, goalX == x ? goalY + 64 : -1)
                : OctileMap.ScanBackward(cols, colStops, colSelf, y + 64, goalX == x ? goalY + 64 : -1);
            if (v > 0)
            {
                core.Offer(c + (dy * v * _w), a + v, b + n, x, y + (dy * v), vDir, node);
            }
        }
    }
}

/// <summary>Jump points read from the JPS+ tables (Game AI Pro 2, "JPS+"): one array read per jump.</summary>
internal readonly struct TableFinder : IJumpFinder
{
    private readonly short[] _t;
    private readonly byte[] _nb;
    private readonly int _w;
    private readonly bool _improved;

    /// <param name="index">The tables (cell-major, see <see cref="JpsPlusIndex"/>).</param>
    /// <param name="map">Neighbour bytes of the same grid.</param>
    /// <param name="improvedPruning">As for <see cref="BitFinder"/>.</param>
    public TableFinder(JpsPlusIndex index, OctileMap map, bool improvedPruning)
    {
        _t = index.Table;
        _nb = map.Neighbors;
        _w = map.Width;
        _improved = improvedPruning;
    }

    // Steps to the straight jump point or to the goal on the line, 0 if none, for the cell c at (x, y).
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int Straight(JumpCore core, int c, int x, int y, int d)
    {
        int jd = _t[(c << 3) + d];
        int len = jd > 0 ? jd : -jd;
        int k;
        switch (d)
        {
            case 0:
                k = core.GoalX == x ? y - core.GoalY : 0;
                break;
            case 1:
                k = core.GoalY == y ? core.GoalX - x : 0;
                break;
            case 2:
                k = core.GoalX == x ? core.GoalY - y : 0;
                break;
            default:
                k = core.GoalY == y ? x - core.GoalX : 0;
                break;
        }
        // The goal on the line ahead within the stored distance is a jump point of its own.
        if (k > 0 && k <= len)
        {
            return k;
        }
        return jd > 0 ? jd : 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Expand(JumpCore core, int node, int x, int y, int a, int b, int arrival)
    {
        int nb = _nb[node];
        int mask = arrival < 0 ? nb : OctileMap.Successors[(arrival << 8) | nb];
        int w = _w;
        while (mask != 0)
        {
            int d = BitOperations.TrailingZeroCount(mask);
            mask &= mask - 1;
            if (d < 4)
            {
                int s = Straight(core, node, x, y, d);
                if (s > 0)
                {
                    int tx = x + (OctileMap.Dx[d] * s);
                    int ty = y + (OctileMap.Dy[d] * s);
                    core.Offer(node + (OctileMap.Dx[d] * s) + (OctileMap.Dy[d] * s * w), a + s, b, tx, ty, d, node);
                }
            }
            else if (_improved)
            {
                DiagonalImproved(core, node, x, y, a, b, d);
            }
            else
            {
                int dx = OctileMap.Dx[d];
                int dy = OctileMap.Dy[d];
                int jd = _t[(node << 3) + d];
                int len = jd > 0 ? jd : -jd;
                int gdx = (core.GoalX - x) * dx;
                int gdy = (core.GoalY - y) * dy;
                int s = 0;
                // A goal in the run's quadrant stops the run at the cell level with it, when the run gets that far.
                if (gdx > 0 && gdy > 0)
                {
                    int m = gdx < gdy ? gdx : gdy;
                    if (m <= len)
                    {
                        s = m;
                    }
                }
                if (s == 0 && jd > 0)
                {
                    s = jd;
                }
                if (s > 0)
                {
                    core.Offer(node + (s * (dx + (dy * w))), a, b + s, x + (dx * s), y + (dy * s), d, node);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void DiagonalImproved(JumpCore core, int node, int x, int y, int a, int b, int d)
    {
        int dx = OctileMap.Dx[d];
        int dy = OctileMap.Dy[d];
        int delta = dx + (dy * _w);
        int hDir = dx > 0 ? 1 : 3;
        int vDir = dy > 0 ? 2 : 0;
        int c = node;
        int acc = 0;
        while (true)
        {
            int jd = _t[(c << 3) + d];
            int len = jd > 0 ? jd : -jd;
            int gdx = (core.GoalX - x) * dx;
            int gdy = (core.GoalY - y) * dy;
            int step = 0;
            if (gdx > 0 && gdy > 0)
            {
                int m = gdx < gdy ? gdx : gdy;
                if (m <= len)
                {
                    step = m;
                }
            }
            if (step == 0)
            {
                if (jd <= 0)
                {
                    return; // a wall ends the run
                }
                step = jd;
            }
            c += step * delta;
            x += step * dx;
            y += step * dy;
            acc += step;
            if (c == core.Goal)
            {
                core.Offer(c, a, b + acc, x, y, d, node);
                return;
            }
            int h = Straight(core, c, x, y, hDir);
            if (h > 0)
            {
                core.Offer(c + (dx * h), a + h, b + acc, x + (dx * h), y, hDir, node);
            }
            int v = Straight(core, c, x, y, vDir);
            if (v > 0)
            {
                core.Offer(c + (dy * v * _w), a + v, b + acc, x, y + (dy * v), vDir, node);
            }
        }
    }
}
