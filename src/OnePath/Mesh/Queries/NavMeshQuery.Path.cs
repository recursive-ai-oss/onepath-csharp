using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    // Search state. A node is a crossing: leaving polygon `from` through exit `exit` into polygon `arrival`, at the
    // crossing position. The start node is the start position in the start polygon; one terminal node stands for the end.
    private struct Node
    {
        public ulong Arrival;
        public double G;
        public double X;
        public double Y;
        public double Z;
        public double H; // straight distance from the node position to the end position
        public int Parent;
        public bool Closed;
    }

    private struct HashSlot
    {
        public ulong KeyA;
        public int KeyB;
        public int Node;
        public int Stamp;
    }

    private struct HeapEntry
    {
        public double F;
        public int Seq;
        public int Node;
    }

    private Node[] _nodes = [];
    private int _nodeCount;
    private HashSlot[] _hash = [];
    private int _hMask;
    private int _stamp;

    private HeapEntry[] _heap = new HeapEntry[1024];
    private int _heapCount;
    private int _seq;

    private readonly bool _polygonNodes;

    private enum SearchState { None, Active, Done }

    private SearchState _state;
    private PathStatus _beginStatus;
    private PolyRef _startPoly;
    private PolyRef _endPoly;
    private Vector3D _endPos;
    private int _terminal;
    private int _best;
    private double _bestH;
    private bool _nodeLimit;
    private int _iterations;
    private PlainFilter _plain;
    private bool _plainStored;
    private int[] _chain = new int[256];

    private void InitPathBuffers()
    {
        int n = _maxNodes + 2;
        _nodes = new Node[n];
        int h = 16;
        while (h < 2 * n)
        {
            h <<= 1;
        }
        _hash = new HashSlot[h];
        _hMask = h - 1;
    }

    /// <summary>Search iterations (node expansions) done by the current or last path search.</summary>
    public int Iterations => _iterations;

    /// <summary>True if the last search needed more nodes than the query's limit.</summary>
    public bool NodeLimitReached => _nodeLimit;

    /// <summary>
    /// Starts a path search between two polygons with the plain filter (null accepts everything), keeping the filter for
    /// <see cref="UpdatePath(int)"/>. The positions must lie on (or near) their polygons; they are not snapped.
    /// </summary>
    /// <returns><see cref="PathStatus.InProgress"/> when the search can run; otherwise the reason it cannot.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    public PathStatus BeginPath(PolyRef startPoly, PolyRef endPoly, Vector3D startPos, Vector3D endPos, QueryFilter? filter = null)
    {
        _plain = new PlainFilter(filter ?? s_defaultFilter);
        _plainStored = true;
        return BeginPath(startPoly, endPoly, startPos, endPos, in _plain);
    }

    /// <summary>Starts a path search with a struct filter. Pass the same filter to every <see cref="UpdatePath{TFilter}"/> call.</summary>
    public PathStatus BeginPath<TFilter>(PolyRef startPoly, PolyRef endPoly, Vector3D startPos, Vector3D endPos, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        CheckPos(startPos, nameof(startPos));
        CheckPos(endPos, nameof(endPos));
        _state = SearchState.None;
        _nodeLimit = false;
        _iterations = 0;
        _terminal = -1;
        if (typeof(TFilter) != typeof(PlainFilter))
        {
            _plainStored = false;
        }
        if (!_mesh.TryGetPoly(startPoly, out NavTile st, out int sp) || !filter.Passes(st.PolyFlags[sp], st.PolyAreas[sp]))
        {
            return _beginStatus = PathStatus.InvalidStartPoly;
        }
        if (!_mesh.TryGetPoly(endPoly, out NavTile et, out int ep) || !filter.Passes(et.PolyFlags[ep], et.PolyAreas[ep]))
        {
            return _beginStatus = PathStatus.InvalidEndPoly;
        }
        _stamp++;
        if (_stamp == int.MaxValue)
        {
            Array.Clear(_hash);
            _stamp = 1;
        }
        _nodeCount = 0;
        _heapCount = 0;
        _seq = 0;
        _startPoly = startPoly;
        _endPoly = endPoly;
        _endPos = endPos;
        _terminalNodeIndex = -1;
        int root = _polygonNodes ? FindOrAdd(startPoly.Value, 0, startPoly.Value, out _) : FindOrAdd(0, -1, startPoly.Value, out _);
        _nodes[root].Parent = -1;
        _nodes[root].G = 0.0;
        _nodes[root].X = startPos.X;
        _nodes[root].Y = startPos.Y;
        _nodes[root].Z = startPos.Z;
        _best = root;
        _bestH = Vector3D.Distance(startPos, endPos);
        double h0 = EndDistance(startPos.X, startPos.Y, startPos.Z);
        _nodes[root].H = h0;
        Push(filter.MinAreaCost * h0, root);
        _state = SearchState.Active;
        return _beginStatus = PathStatus.InProgress;
    }

    /// <summary>Runs up to <paramref name="maxIterations"/> expansions of the search started with the plain-filter <see cref="BeginPath(PolyRef, PolyRef, Vector3D, Vector3D, QueryFilter?)"/>.</summary>
    /// <returns>InProgress while unfinished, then Success or Partial; or the failure from Begin.</returns>
    /// <exception cref="InvalidOperationException">The search was started with a struct filter; use the generic overload.</exception>
    public PathStatus UpdatePath(int maxIterations)
    {
        if (!_plainStored)
        {
            throw new InvalidOperationException("The search was started with a struct filter; pass it to UpdatePath<TFilter>.");
        }
        PlainFilter f = _plain;
        return UpdatePath(maxIterations, in f);
    }

    /// <summary>
    /// Runs up to <paramref name="maxIterations"/> expansions (one pop of the open list is one iteration). The result
    /// after any sequence of calls adding up to the same work equals the result of one unlimited call.
    /// </summary>
    public PathStatus UpdatePath<TFilter>(int maxIterations, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        if (_state == SearchState.None)
        {
            return _beginStatus;
        }
        if (_state == SearchState.Done)
        {
            return _terminal >= 0 ? PathStatus.Success : PathStatus.Partial;
        }
        int budget = maxIterations;
        while (budget > 0 && _heapCount > 0)
        {
            int node = PopMin();
            if (_nodes[node].Closed)
            {
                continue; // stale entry from an earlier, more expensive route
            }
            _nodes[node].Closed = true;
            budget--;
            _iterations++;
            if (node == _terminalNodeIndex)
            {
                _terminal = node;
                _state = SearchState.Done;
                return PathStatus.Success;
            }
            Expand(node, in filter);
        }
        if (_heapCount == 0)
        {
            _state = SearchState.Done;
            return PathStatus.Partial;
        }
        return PathStatus.InProgress;
    }

    private int _terminalNodeIndex = -1;

    /// <summary>
    /// Ends the search and writes the corridor (start polygon first). Loops where the cheapest route left a polygon and
    /// came back to it are cut out. If the search is unfinished the corridor leads to the polygon closest to the goal so far.
    /// </summary>
    /// <param name="path">Receives the corridor; extra polygons beyond its length are dropped.</param>
    /// <param name="count">Number of polygons written.</param>
    /// <param name="truncated">True if the corridor was longer than <paramref name="path"/>.</param>
    /// <param name="cost">The search cost of the full route.</param>
    public PathStatus FinishPath(Span<PolyRef> path, out int count, out bool truncated, out double cost)
    {
        count = 0;
        truncated = false;
        cost = 0;
        if (_state == SearchState.None)
        {
            return _beginStatus;
        }
        bool done = _state == SearchState.Done;
        int target = _terminal >= 0 ? _terminal : _best;
        PathStatus status = _terminal >= 0 ? PathStatus.Success : (done ? PathStatus.Partial : PathStatus.InProgress);
        cost = _nodes[target].G;
        int len = 0;
        int cur = target;
        if (cur == _terminalNodeIndex)
        {
            cur = _nodes[cur].Parent;
        }
        while (cur >= 0)
        {
            if (len == _chain.Length)
            {
                Array.Resize(ref _chain, len * 2);
            }
            _chain[len++] = cur;
            cur = _nodes[cur].Parent;
        }
        int w = 0;
        // Chain is leaf to root; walk root to leaf, removing loops (a polygon met twice cuts everything between).
        for (int i = len - 1; i >= 0; i--)
        {
            ulong poly = _nodes[_chain[i]].Arrival;
            // Search the output so far (stored in _outTmp) for this polygon.
            int seen = FindInTmp(poly, w);
            if (seen >= 0)
            {
                w = seen + 1;
            }
            else
            {
                SetTmp(w++, poly);
            }
        }
        for (int i = 0; i < w; i++)
        {
            if (i < path.Length)
            {
                path[i] = new PolyRef(_tmp[i]);
            }
            else
            {
                truncated = true;
            }
        }
        count = Math.Min(w, path.Length);
        return status;
    }

    private ulong[] _tmp = new ulong[256];

    private int FindInTmp(ulong poly, int w)
    {
        for (int j = 0; j < w; j++)
        {
            if (_tmp[j] == poly)
            {
                return j;
            }
        }
        return -1;
    }

    private void SetTmp(int i, ulong v)
    {
        if (i == _tmp.Length)
        {
            Array.Resize(ref _tmp, i * 2);
        }
        _tmp[i] = v;
    }

    /// <summary>Polygons expanded by the current or last search, in expansion order (debug geometry; allocates).</summary>
    public PolyRef[] GetExpandedPolygons()
    {
        var list = new System.Collections.Generic.List<PolyRef>();
        for (int i = 0; i < _nodeCount; i++)
        {
            if (_nodes[i].Closed)
            {
                list.Add(new PolyRef(_nodes[i].Arrival));
            }
        }
        return list.ToArray();
    }

    /// <summary>
    /// Finds the cheapest corridor between two polygons in one call (begin, run to completion, finish).
    /// </summary>
    public PathStatus FindPath<TFilter>(PolyRef startPoly, PolyRef endPoly, Vector3D startPos, Vector3D endPos, in TFilter filter, Span<PolyRef> path, out int count)
        where TFilter : struct, IQueryFilter
    {
        PathStatus s = BeginPath(startPoly, endPoly, startPos, endPos, in filter);
        if (s != PathStatus.InProgress)
        {
            count = 0;
            return s;
        }
        UpdatePath(int.MaxValue, in filter);
        return FinishPath(path, out count, out _, out _);
    }

    // --- search internals ---

    private static ulong Mix(ulong x)
    {
        x ^= x >> 33;
        x *= 0xFF51AFD7ED558CCDUL;
        x ^= x >> 33;
        x *= 0xC4CEB9FE1A85EC53UL;
        return x ^ (x >> 33);
    }

    private double EndDistance(double x, double y, double z)
    {
        double dx = x - _endPos.X, dy = y - _endPos.Y, dz = z - _endPos.Z;
        return Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
    }

    // False when the node a move would reach already costs no more than the cost before the move (a leg never costs less than
    // zero), so the move cannot improve it and its position, leg and heuristic need not be computed.
    private bool MayImprove(ulong from, int exit, ulong arrival, double g)
    {
        HashSlot[] hash = _hash;
        ulong key = _polygonNodes ? arrival : from;
        int ex = _polygonNodes ? 0 : exit;
        int h = (int)(Mix(key ^ ((ulong)(uint)ex * 0x9E3779B97F4A7C15UL)) & (ulong)_hMask);
        while (hash[h].Stamp == _stamp)
        {
            if (hash[h].KeyA == key && hash[h].KeyB == ex)
            {
                return g < _nodes[hash[h].Node].G;
            }
            h = (h + 1) & _hMask;
        }
        return true;
    }

    // Finds the node for (from, exit) or creates it; returns -1 when the node limit stops creation.
    private int FindOrAdd(ulong from, int exit, ulong arrival, out bool created)
    {
        HashSlot[] hash = _hash;
        int h = (int)(Mix(from ^ ((ulong)(uint)exit * 0x9E3779B97F4A7C15UL)) & (ulong)_hMask);
        while (hash[h].Stamp == _stamp)
        {
            if (hash[h].KeyA == from && hash[h].KeyB == exit)
            {
                created = false;
                return hash[h].Node;
            }
            h = (h + 1) & _hMask;
        }
        created = true;
        if (_nodeCount >= _maxNodes)
        {
            _nodeLimit = true;
            return -1;
        }
        int n = _nodeCount++;
        hash[h] = new HashSlot { Stamp = _stamp, KeyA = from, KeyB = exit, Node = n };
        _nodes[n] = new Node { Arrival = arrival, G = double.PositiveInfinity, Parent = -1 };
        return n;
    }

    // Relax the crossing (from, exit) -> arrival reached with cost g at (x, y, z) from `parent`.
    private void Relax<TFilter>(int parent, ulong from, int exit, ulong arrival, double g, double x, double y, double z, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        bool created;
        int n;
        if (_polygonNodes)
        {
            // One node per polygon, positioned where the best route found so far enters it. A closed node whose route
            // improves is reopened: entry-point costs are not consistent, so a later route can be cheaper.
            n = FindOrAdd(arrival, 0, arrival, out created);
            if (n < 0 || !(g < _nodes[n].G))
            {
                return;
            }
            _nodes[n].Closed = false;
            created = true;
        }
        else
        {
            n = FindOrAdd(from, exit, arrival, out created);
            if (n < 0 || _nodes[n].Closed || !(g < _nodes[n].G))
            {
                return;
            }
        }
        _nodes[n].G = g;
        _nodes[n].Parent = parent;
        if (created)
        {
            _nodes[n].X = x;
            _nodes[n].Y = y;
            _nodes[n].Z = z;
        }
        double h = EndDistance(x, y, z);
        _nodes[n].H = h;
        Push(g + (filter.MinAreaCost * h), n);
    }

    private void Expand<TFilter>(int node, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        var poly = new PolyRef(_nodes[node].Arrival);
        if (!_mesh.TryGetPoly(poly, out NavTile t, out int pi))
        {
            return;
        }
        double cx = _nodes[node].X, cy = _nodes[node].Y, cz = _nodes[node].Z;
        double g = _nodes[node].G;
        double cost = filter.AreaCost(t.PolyAreas[pi]);
        double hHere = _nodes[node].H;
        if (hHere < _bestH)
        {
            _bestH = hHere;
            _best = node;
        }
        if (poly == _endPoly)
        {
            // The route may end here: walk to the end position inside the polygon.
            int term = FindOrAdd(_endPoly.Value, -2, _endPoly.Value, out bool created);
            if (term >= 0)
            {
                _terminalNodeIndex = term;
                double tg = g + (hHere * cost);
                if (!_nodes[term].Closed && tg < _nodes[term].G)
                {
                    _nodes[term].G = tg;
                    _nodes[term].Parent = node;
                    if (created)
                    {
                        _nodes[term].X = _endPos.X;
                        _nodes[term].Y = _endPos.Y;
                        _nodes[term].Z = _endPos.Z;
                    }
                    Push(tg, term);
                }
            }
        }
        // Polygon nodes only: stepping straight back into the polygon this node was reached from is never cheaper than the
        // parent's own entry, so the move is not generated. (Crossing nodes keep it: it creates distinct nodes whose ties
        // are part of that model's specified order.)
        int parentNode = _nodes[node].Parent;
        ulong back = _polygonNodes && parentNode >= 0 && parentNode != _terminalNodeIndex ? _nodes[parentNode].Arrival : 0UL;
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        int n = t.PolyVertexCounts[pi];
        int b = t.PolyStart[pi];
        for (int e = 0; e < n; e++)
        {
            int nb = t.EdgeNeighbors[b + e];
            if (nb == 0)
            {
                continue;
            }
            int q = nb - 1;
            ulong target = new PolyRef(poly.Salt, poly.TileSlot, q).Value;
            if (target == back || !filter.Passes(t.PolyFlags[q], t.PolyAreas[q]) || (_polygonNodes && !MayImprove(poly.Value, e, target, g)))
            {
                continue;
            }
            int a = 3 * idx[b + e], c = 3 * idx[b + ((e + 1) % n)];
            double mx, my, mz;
            if (_polygonNodes)
            {
                // Enter through the point of the edge that the straight line from here to the goal crosses (clamped to the
                // edge): any point on the edge gives a real route, and this one follows the string-pulled path closely.
                double ax = t.OriginX + v[a], az = t.OriginZ + v[a + 2];
                double dx = ((double)v[c] - v[a]), dz = ((double)v[c + 2] - v[a + 2]);
                double wx = _endPos.X - cx, wz = _endPos.Z - cz;
                double den = (dx * wz) - (dz * wx);
                double s = 0.5;
                if (den != 0)
                {
                    s = (((cx - ax) * wz) - ((cz - az) * wx)) / den;
                    s = s < 0 ? 0 : (s > 1 ? 1 : s);
                }
                mx = ax + (dx * s);
                my = t.OriginY + v[a + 1] + (((double)v[c + 1] - v[a + 1]) * s);
                mz = az + (dz * s);
            }
            else
            {
                mx = t.OriginX + (((double)v[a] + v[c]) * 0.5);
                my = t.OriginY + (((double)v[a + 1] + v[c + 1]) * 0.5);
                mz = t.OriginZ + (((double)v[a + 2] + v[c + 2]) * 0.5);
            }
            double leg = Dist(cx, cy, cz, mx, my, mz) * cost;
            Relax(node, poly.Value, e, target, g + leg, mx, my, mz, in filter);
        }
        ReadOnlySpan<NavLink> links = _mesh.LinksOfResolved(poly.TileSlot, pi);
        for (int j = 0; j < links.Length; j++)
        {
            NavLink l = links[j];
            if (l.Target.Value == back || !_mesh.TryGetPoly(l.Target, out NavTile qt, out int qi) || !filter.Passes(qt.PolyFlags[qi], qt.PolyAreas[qi]))
            {
                continue;
            }
            int exit = n + j;
            if (l.Kind == NavLinkKind.Portal)
            {
                int a = 3 * idx[b + l.Index], c = 3 * idx[b + ((l.Index + 1) % n)];
                double s = ((l.RangeMin + l.RangeMax) * 0.5) / 255.0;
                if (_polygonNodes)
                {
                    double ax = t.OriginX + v[a], az = t.OriginZ + v[a + 2];
                    double dx = ((double)v[c] - v[a]), dz = ((double)v[c + 2] - v[a + 2]);
                    double wx = _endPos.X - cx, wz = _endPos.Z - cz;
                    double den = (dx * wz) - (dz * wx);
                    if (den != 0)
                    {
                        double lo = l.RangeMin / 255.0, hi = l.RangeMax / 255.0;
                        s = (((cx - ax) * wz) - ((cz - az) * wx)) / den;
                        s = s < lo ? lo : (s > hi ? hi : s);
                    }
                }
                double mx = t.OriginX + v[a] + ((v[c] - (double)v[a]) * s);
                double my = t.OriginY + v[a + 1] + ((v[c + 1] - (double)v[a + 1]) * s);
                double mz = t.OriginZ + v[a + 2] + ((v[c + 2] - (double)v[a + 2]) * s);
                Relax(node, poly.Value, exit, l.Target.Value, g + (Dist(cx, cy, cz, mx, my, mz) * cost), mx, my, mz, in filter);
            }
            else
            {
                NavTile owner = _mesh.GetTileBySlot(l.OwnerSlot)!;
                OffMeshLink o = owner.OffMeshLinks[l.Index];
                if (!filter.Passes(o.PolyFlags, o.Area))
                {
                    continue;
                }
                bool fwd = l.Kind == NavLinkKind.OffMesh;
                double sx = owner.OriginX + (fwd ? o.StartX : o.EndX), sy = owner.OriginY + (fwd ? o.StartY : o.EndY), sz = owner.OriginZ + (fwd ? o.StartZ : o.EndZ);
                double ex = owner.OriginX + (fwd ? o.EndX : o.StartX), ey = owner.OriginY + (fwd ? o.EndY : o.StartY), ez = owner.OriginZ + (fwd ? o.EndZ : o.StartZ);
                double total = (Dist(cx, cy, cz, sx, sy, sz) * cost) + (Dist(sx, sy, sz, ex, ey, ez) * filter.AreaCost(o.Area));
                Relax(node, poly.Value, exit, l.Target.Value, g + total, ex, ey, ez, in filter);
            }
        }
    }

    private static double Dist(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        double dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
        return Math.Sqrt((dx * dx) + (dy * dy) + (dz * dz));
    }

    // --- open list: binary min-heap on (f, insertion sequence) ---

    private void Push(double f, int node)
    {
        if (_heapCount == _heap.Length)
        {
            Array.Resize(ref _heap, _heapCount * 2);
        }
        HeapEntry[] heap = _heap;
        int i = _heapCount++;
        int seq = _seq++;
        while (i > 0)
        {
            int parent = (i - 1) >> 1;
            if (heap[parent].F < f || (heap[parent].F == f && heap[parent].Seq < seq))
            {
                break;
            }
            heap[i] = heap[parent];
            i = parent;
        }
        heap[i] = new HeapEntry { F = f, Seq = seq, Node = node };
    }

    private int PopMin()
    {
        HeapEntry[] heap = _heap;
        int result = heap[0].Node;
        int last = --_heapCount;
        if (last > 0)
        {
            HeapEntry x = heap[last];
            int i = 0;
            while (true)
            {
                int c = (2 * i) + 1;
                if (c >= last)
                {
                    break;
                }
                int r = c + 1;
                if (r < last && (heap[r].F < heap[c].F || (heap[r].F == heap[c].F && heap[r].Seq < heap[c].Seq)))
                {
                    c = r;
                }
                if (x.F < heap[c].F || (x.F == heap[c].F && x.Seq < heap[c].Seq))
                {
                    break;
                }
                heap[i] = heap[c];
                i = c;
            }
            heap[i] = x;
        }
        return result;
    }
}
