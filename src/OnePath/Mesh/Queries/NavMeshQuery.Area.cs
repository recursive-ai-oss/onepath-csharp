using System;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    // Polygon-set search state. Nodes are polygons (one node per polygon reached); the open list is a binary heap on
    // (key, insertion sequence), so the expansion order is fully specified.
    private ulong[] _aPoly = [];
    private int[] _aParent = [];
    private double[] _aKey = [];
    private bool[] _aClosed = [];
    private double[] _aArea = [];
    private int[] _aOrder = [];
    private int _aCount;
    private int _aOrderCount;

    private int[] _ahStamp = [];
    private int[] _ahNode = [];
    private int _ahMask;
    private int _aStamp;

    private double[] _apF = new double[256];
    private int[] _apSeq = new int[256];
    private int[] _apNode = new int[256];
    private int _apCount;
    private int _aSeq;

    private SearchState _aState;
    private AreaSearchKind _aKind;
    private bool _aIsCircle;
    private double _aCx;
    private double _aCz;
    private double _aR2;
    private double _aRadius;
    private readonly double[] _aShape = new double[2 * MaxShapeVertices];
    private int _aShapeN;
    private bool _aNodeLimit;
    private int _aIterations;
    private PlainFilter _aPlain;
    private bool _aPlainStored;

    /// <summary>Most vertices a convex query shape may have.</summary>
    public const int MaxShapeVertices = 64;

    /// <summary>Expansions done by the current or last polygon-set search.</summary>
    public int AreaIterations => _aIterations;

    /// <summary>True if the last polygon-set search needed more polygons than the query's node limit and dropped some.</summary>
    public bool AreaNodeLimitReached => _aNodeLimit;

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void EnsureAreaBuffers()
    {
        if (_aPoly.Length != 0)
        {
            return;
        }
        int n = _maxNodes;
        _aPoly = new ulong[n];
        _aParent = new int[n];
        _aKey = new double[n];
        _aClosed = new bool[n];
        _aArea = new double[n];
        _aOrder = new int[n];
        int h = 16;
        while (h < 2 * n)
        {
            h <<= 1;
        }
        _ahStamp = new int[h];
        _ahNode = new int[h];
        _ahMask = h - 1;
    }

    // ---- begin ----

    /// <summary>
    /// Starts a sliced search for the polygons reachable from a start polygon through portals that touch a circle in the X/Z
    /// plane (see <c>docs/navmesh-queries.md</c>), keeping the plain filter (null accepts everything) for <see cref="UpdateAreaSearch(int)"/>.
    /// </summary>
    /// <param name="startPoly">Where the search starts; always part of the result.</param>
    /// <param name="center">Circle centre (the start polygon need not contain it).</param>
    /// <param name="radius">Circle radius: finite, 0 to 2^22.</param>
    /// <param name="filter">Which polygons may be entered.</param>
    /// <param name="kind">Whether stacked polygons are all kept or only the first met.</param>
    /// <returns><see cref="AreaQueryStatus.InProgress"/> when the search can run, else why not.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The centre is non-finite or beyond ±2^21, or the radius is out of range.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus BeginAreaSearch(PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter = null, AreaSearchKind kind = AreaSearchKind.Reachable)
    {
        _aPlain = new PlainFilter(filter ?? s_defaultFilter);
        _aPlainStored = true;
        return BeginAreaSearch(startPoly, center, radius, in _aPlain, kind);
    }

    /// <summary>Starts a sliced circle search with a struct filter; pass the same filter to every <see cref="UpdateAreaSearch{TFilter}"/> call.</summary>
    /// <inheritdoc cref="BeginAreaSearch(PolyRef, Vector3D, double, QueryFilter?, AreaSearchKind)"/>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus BeginAreaSearch<TFilter>(PolyRef startPoly, Vector3D center, double radius, in TFilter filter, AreaSearchKind kind = AreaSearchKind.Reachable)
        where TFilter : struct, IQueryFilter
    {
        CheckPos(center, nameof(center));
        if (!(radius >= 0 && radius <= 4194304.0))
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must be finite, from 0 to 2^22.");
        }
        ClearPlainMark<TFilter>();
        _aIsCircle = true;
        _aCx = center.X;
        _aCz = center.Z;
        _aRadius = radius;
        _aR2 = radius * radius;
        return BeginArea(startPoly, kind, in filter);
    }

    /// <summary>
    /// Starts a sliced search for the polygons reachable through portals that touch a convex shape (X/Z of the given points,
    /// either winding, 3 to <see cref="MaxShapeVertices"/> points; Y is ignored), keeping the plain filter.
    /// </summary>
    /// <returns><see cref="AreaQueryStatus.InProgress"/> when the search can run, <see cref="AreaQueryStatus.InvalidShape"/> for a bad shape, or <see cref="AreaQueryStatus.InvalidStartPoly"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A shape point is non-finite or beyond ±2^21.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus BeginAreaSearch(PolyRef startPoly, ReadOnlySpan<Vector3D> shape, QueryFilter? filter = null, AreaSearchKind kind = AreaSearchKind.Reachable)
    {
        _aPlain = new PlainFilter(filter ?? s_defaultFilter);
        _aPlainStored = true;
        return BeginAreaSearch(startPoly, shape, in _aPlain, kind);
    }

    /// <summary>Starts a sliced convex-shape search with a struct filter.</summary>
    /// <inheritdoc cref="BeginAreaSearch(PolyRef, ReadOnlySpan{Vector3D}, QueryFilter?, AreaSearchKind)"/>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus BeginAreaSearch<TFilter>(PolyRef startPoly, ReadOnlySpan<Vector3D> shape, in TFilter filter, AreaSearchKind kind = AreaSearchKind.Reachable)
        where TFilter : struct, IQueryFilter
    {
        for (int i = 0; i < shape.Length; i++)
        {
            CheckPos(shape[i], nameof(shape));
        }
        ClearPlainMark<TFilter>();
        _aState = SearchState.None;
        if (!SetConvexShape(shape))
        {
            return AreaQueryStatus.InvalidShape;
        }
        _aIsCircle = false;
        return BeginArea(startPoly, kind, in filter);
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void ClearPlainMark<TFilter>()
    {
        if (typeof(TFilter) != typeof(PlainFilter))
        {
            _aPlainStored = false;
        }
    }

    // Validates and stores a convex shape, counter-clockwise, with its reference point (vertex mean) in (_aCx, _aCz).
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private bool SetConvexShape(ReadOnlySpan<Vector3D> shape)
    {
        int n = shape.Length;
        if (n < 3 || n > MaxShapeVertices)
        {
            return false;
        }
        double area2 = 0;
        for (int i = 0; i < n; i++)
        {
            Vector3D a = shape[i], b = shape[(i + 1) % n];
            area2 += (a.X * b.Z) - (b.X * a.Z);
        }
        if (!(area2 != 0))
        {
            return false;
        }
        bool reverse = area2 < 0;
        double mx = 0, mz = 0;
        for (int i = 0; i < n; i++)
        {
            Vector3D v = shape[reverse ? n - 1 - i : i];
            _aShape[2 * i] = v.X;
            _aShape[(2 * i) + 1] = v.Z;
            mx += v.X;
            mz += v.Z;
        }
        // Convex means every vertex lies on or left of every edge; this also rejects star shapes that wind twice.
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            for (int k = 0; k < n; k++)
            {
                if (Side(_aShape[2 * i], _aShape[(2 * i) + 1], _aShape[2 * j], _aShape[(2 * j) + 1], _aShape[2 * k], _aShape[(2 * k) + 1]) < 0)
                {
                    return false;
                }
            }
        }
        _aShapeN = n;
        _aCx = mx / n;
        _aCz = mz / n;
        return true;
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private AreaQueryStatus BeginArea<TFilter>(PolyRef startPoly, AreaSearchKind kind, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        _aState = SearchState.None;
        _aNodeLimit = false;
        _aIterations = 0;
        _aCount = 0;
        _aOrderCount = 0;
        _apCount = 0;
        _aSeq = 0;
        _aKind = kind;
        if (!_mesh.TryGetPoly(startPoly, out NavTile t, out int p) || !filter.Passes(t.PolyFlags[p], t.PolyAreas[p]))
        {
            return AreaQueryStatus.InvalidStartPoly;
        }
        EnsureAreaBuffers();
        _aStamp++;
        if (_aStamp == int.MaxValue)
        {
            Array.Clear(_ahStamp);
            _aStamp = 1;
        }
        int root = AreaFind(startPoly.Value, out int slot);
        _ = root;
        int n = _aCount++;
        _ahStamp[slot] = _aStamp;
        _ahNode[slot] = n;
        _aPoly[n] = startPoly.Value;
        _aParent[n] = -1;
        _aKey[n] = 0.0;
        _aClosed[n] = false;
        AreaPush(0.0, n);
        _aState = SearchState.Active;
        return AreaQueryStatus.InProgress;
    }

    // Looks the polygon up; returns the node index, or -1 with the free hash slot in `slot`.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private int AreaFind(ulong poly, out int slot)
    {
        int h = (int)(Mix(poly) & (ulong)_ahMask);
        while (_ahStamp[h] == _aStamp)
        {
            if (_aPoly[_ahNode[h]] == poly)
            {
                slot = h;
                return _ahNode[h];
            }
            h = (h + 1) & _ahMask;
        }
        slot = h;
        return -1;
    }

    // ---- update ----

    /// <summary>Runs up to <paramref name="maxIterations"/> expansions of the search begun with a plain filter.</summary>
    /// <exception cref="InvalidOperationException">The search was started with a struct filter; use the generic overload.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus UpdateAreaSearch(int maxIterations)
    {
        if (!_aPlainStored)
        {
            throw new InvalidOperationException("The search was started with a struct filter; pass it to UpdateAreaSearch<TFilter>.");
        }
        PlainFilter f = _aPlain;
        return UpdateAreaSearch(maxIterations, in f);
    }

    /// <summary>
    /// Runs up to <paramref name="maxIterations"/> expansions (one pop of the open list is one iteration; stale entries are free).
    /// Any split of the same total work ends with the result of one unlimited call.
    /// </summary>
    /// <returns>InProgress while unfinished, then Success; or the failure from Begin.</returns>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus UpdateAreaSearch<TFilter>(int maxIterations, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        if (_aState == SearchState.None)
        {
            return AreaQueryStatus.InvalidStartPoly;
        }
        int budget = maxIterations;
        while (_aState == SearchState.Active && budget > 0 && _apCount > 0)
        {
            int node = AreaPop();
            if (_aClosed[node])
            {
                continue;
            }
            _aClosed[node] = true;
            _aOrder[_aOrderCount++] = node;
            budget--;
            _aIterations++;
            ExpandArea(node, in filter);
        }
        if (_apCount == 0)
        {
            _aState = SearchState.Done;
        }
        return _aState == SearchState.Done ? AreaQueryStatus.Success : AreaQueryStatus.InProgress;
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void ExpandArea<TFilter>(int node, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        var poly = new PolyRef(_aPoly[node]);
        if (!_mesh.TryGetPoly(poly, out NavTile t, out int pi))
        {
            return;
        }
        ReadOnlySpan<float> v = t.Vertices;
        ReadOnlySpan<ushort> idx = t.PolyIndices;
        int n = t.PolyVertexCounts[pi];
        int b = t.PolyStart[pi];
        ReadOnlySpan<NavLink> links = _mesh.GetLinks(poly);
        for (int e = 0; e < n; e++)
        {
            int a = 3 * idx[b + e], c = 3 * idx[b + ((e + 1) % n)];
            double ax = t.OriginX + (double)v[a], az = t.OriginZ + (double)v[a + 2];
            double cx = t.OriginX + (double)v[c], cz = t.OriginZ + (double)v[c + 2];
            int nb = t.EdgeNeighbors[b + e];
            if (nb != 0)
            {
                int q = nb - 1;
                if (filter.Passes(t.PolyFlags[q], t.PolyAreas[q]) && Touches(ax, az, cx, cz, out double key))
                {
                    AreaRelax(node, new PolyRef(poly.Salt, poly.TileSlot, q), key);
                }
                continue;
            }
            if (t.EdgePortalSides[b + e] == NavMeshFormat.NoSide)
            {
                continue;
            }
            for (int j = 0; j < links.Length; j++)
            {
                NavLink l = links[j];
                if (l.Kind != NavLinkKind.Portal || l.Index != e
                    || !_mesh.TryGetPoly(l.Target, out NavTile qt, out int qi) || !filter.Passes(qt.PolyFlags[qi], qt.PolyAreas[qi]))
                {
                    continue;
                }
                Lerp(ax, az, cx, cz, l.RangeMin / 255.0, out double px, out double pz);
                Lerp(ax, az, cx, cz, l.RangeMax / 255.0, out double rx, out double rz);
                if (Touches(px, pz, rx, rz, out double key))
                {
                    AreaRelax(node, l.Target, key);
                }
            }
        }
    }

    // Point at fraction s along a→c; the ends are returned exactly.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private static void Lerp(double ax, double az, double cx, double cz, double s, out double x, out double z)
    {
        if (s <= 0)
        {
            x = ax;
            z = az;
        }
        else if (s >= 1)
        {
            x = cx;
            z = cz;
        }
        else
        {
            x = ax + ((cx - ax) * s);
            z = az + ((cz - az) * s);
        }
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void AreaRelax(int parent, PolyRef target, double key)
    {
        int n = AreaFind(target.Value, out int slot);
        if (n >= 0)
        {
            if (!_aClosed[n] && key < _aKey[n])
            {
                _aKey[n] = key;
                _aParent[n] = parent;
                AreaPush(key, n);
            }
            return;
        }
        if (_aKind == AreaSearchKind.LocalNeighbourhood && OverlapsKept(target))
        {
            return;
        }
        if (_aCount >= _maxNodes)
        {
            _aNodeLimit = true;
            return;
        }
        n = _aCount++;
        _ahStamp[slot] = _aStamp;
        _ahNode[slot] = n;
        _aPoly[n] = target.Value;
        _aParent[n] = parent;
        _aKey[n] = key;
        _aClosed[n] = false;
        AreaPush(key, n);
    }

    // True if the polygon's X/Z outline overlaps (interiors intersect) a polygon already kept.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private bool OverlapsKept(PolyRef target)
    {
        if (!_mesh.TryGetPoly(target, out NavTile t, out int p))
        {
            return false;
        }
        for (int i = 0; i < _aCount; i++)
        {
            if (_mesh.TryGetPoly(new PolyRef(_aPoly[i]), out NavTile u, out int q) && PolysOverlap(t, p, u, q))
            {
                return true;
            }
        }
        return false;
    }

    // Convex polygons (counter-clockwise) have overlapping interiors unless some edge of one has the whole other on its
    // outer closed side (separating axis theorem; exact predicates, so touching polygons never count as overlapping).
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    internal static bool PolysOverlap(NavTile t1, int p1, NavTile t2, int p2) =>
        !SeparatedBy(t1, p1, t2, p2) && !SeparatedBy(t2, p2, t1, p1);

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private static bool SeparatedBy(NavTile a, int pa, NavTile b, int pb)
    {
        int na = a.PolyVertexCounts[pa], nb = b.PolyVertexCounts[pb];
        int sa = a.PolyStart[pa], sb = b.PolyStart[pb];
        for (int e = 0; e < na; e++)
        {
            int i0 = 3 * a.PolyIndices[sa + e], i1 = 3 * a.PolyIndices[sa + ((e + 1) % na)];
            double x0 = a.OriginX + (double)a.Vertices[i0], z0 = a.OriginZ + (double)a.Vertices[i0 + 2];
            double x1 = a.OriginX + (double)a.Vertices[i1], z1 = a.OriginZ + (double)a.Vertices[i1 + 2];
            bool allOutside = true;
            for (int k = 0; k < nb && allOutside; k++)
            {
                int ik = 3 * b.PolyIndices[sb + k];
                allOutside = Side(x0, z0, x1, z1, b.OriginX + (double)b.Vertices[ik], b.OriginZ + (double)b.Vertices[ik + 2]) <= 0;
            }
            if (allOutside)
            {
                return true;
            }
        }
        return false;
    }

    // ---- finish ----

    /// <summary>
    /// Writes the polygons reached, in expansion order (the start polygon first). If the search is unfinished, the polygons expanded so far.
    /// </summary>
    /// <param name="polys">Receives the polygons; extra ones beyond its length are dropped.</param>
    /// <param name="parents">Receives, per polygon, the polygon it was reached from (<see cref="PolyRef.Invalid"/> for the start); may be empty or shorter.</param>
    /// <param name="distancesSquared">Receives, per polygon, the squared X/Z distance from the shape's reference point (circle centre, or mean of the shape's points) to the portal it was entered through (0 for the start); may be empty or shorter.</param>
    /// <param name="count">Number of polygons written.</param>
    /// <param name="truncated">True if <paramref name="polys"/> was too short.</param>
    /// <returns>Success when the search is done, InProgress otherwise, InvalidStartPoly if it never began.</returns>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FinishAreaSearch(Span<PolyRef> polys, Span<PolyRef> parents, Span<double> distancesSquared, out int count, out bool truncated)
    {
        count = 0;
        truncated = false;
        if (_aState == SearchState.None)
        {
            return AreaQueryStatus.InvalidStartPoly;
        }
        count = Math.Min(polys.Length, _aOrderCount);
        truncated = _aOrderCount > polys.Length;
        for (int i = 0; i < count; i++)
        {
            int nd = _aOrder[i];
            polys[i] = new PolyRef(_aPoly[nd]);
            if (i < parents.Length)
            {
                parents[i] = _aParent[nd] < 0 ? PolyRef.Invalid : new PolyRef(_aPoly[_aParent[nd]]);
            }
            if (i < distancesSquared.Length)
            {
                distancesSquared[i] = _aKey[nd];
            }
        }
        return _aState == SearchState.Done ? AreaQueryStatus.Success : AreaQueryStatus.InProgress;
    }

    // ---- one-call forms ----

    /// <summary>
    /// The polygons reachable from <paramref name="startPoly"/> through portals (shared edges, tile-border links) that touch
    /// a circle in the X/Z plane: a portal touches when the closest point of its segment to <paramref name="center"/> is within
    /// <paramref name="radius"/>. Off-mesh links are not followed, heights are ignored. The start polygon is always included.
    /// </summary>
    /// <param name="startPoly">Where the search starts; always part of the result.</param>
    /// <param name="center">Circle centre (the start polygon need not contain it).</param>
    /// <param name="radius">Circle radius: finite, 0 to 2^22.</param>
    /// <param name="filter">Which polygons may be entered.</param>
    /// <param name="polys">Receives the polygons in expansion order, start first.</param>
    /// <param name="parents">Receives the polygon each was reached from (may be empty).</param>
    /// <param name="distancesSquared">Receives each polygon's squared entry distance (may be empty).</param>
    /// <param name="count">Number of polygons written.</param>
    /// <param name="truncated">True if <paramref name="polys"/> was too short.</param>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindPolysInCircle<TFilter>(PolyRef startPoly, Vector3D center, double radius, in TFilter filter, Span<PolyRef> polys, Span<PolyRef> parents, Span<double> distancesSquared, out int count, out bool truncated)
        where TFilter : struct, IQueryFilter
    {
        AreaQueryStatus s = BeginAreaSearch(startPoly, center, radius, in filter);
        return RunAndFinish(s, in filter, polys, parents, distancesSquared, out count, out truncated);
    }

    /// <summary>Circle query with the plain filter (null accepts everything).</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindPolysInCircle(PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter, Span<PolyRef> polys, Span<PolyRef> parents, Span<double> distancesSquared, out int count, out bool truncated)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return FindPolysInCircle(startPoly, center, radius, in f, polys, parents, distancesSquared, out count, out truncated);
    }

    /// <summary>As <see cref="FindPolysInCircle{TFilter}"/>, for a convex shape given by its X/Z points (see <see cref="BeginAreaSearch(PolyRef, ReadOnlySpan{Vector3D}, QueryFilter?, AreaSearchKind)"/>).</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindPolysInShape<TFilter>(PolyRef startPoly, ReadOnlySpan<Vector3D> shape, in TFilter filter, Span<PolyRef> polys, Span<PolyRef> parents, Span<double> distancesSquared, out int count, out bool truncated)
        where TFilter : struct, IQueryFilter
    {
        AreaQueryStatus s = BeginAreaSearch(startPoly, shape, in filter);
        return RunAndFinish(s, in filter, polys, parents, distancesSquared, out count, out truncated);
    }

    /// <summary>Shape query with the plain filter.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindPolysInShape(PolyRef startPoly, ReadOnlySpan<Vector3D> shape, QueryFilter? filter, Span<PolyRef> polys, Span<PolyRef> parents, Span<double> distancesSquared, out int count, out bool truncated)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return FindPolysInShape(startPoly, shape, in f, polys, parents, distancesSquared, out count, out truncated);
    }

    /// <summary>
    /// The local neighbourhood of a position: the circle query, but a polygon whose outline overlaps one already kept is
    /// skipped (and not expanded through), so stacked floors appear once. "Already kept" follows the search's creation order.
    /// </summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindLocalNeighbourhood<TFilter>(PolyRef startPoly, Vector3D center, double radius, in TFilter filter, Span<PolyRef> polys, Span<PolyRef> parents, out int count, out bool truncated)
        where TFilter : struct, IQueryFilter
    {
        AreaQueryStatus s = BeginAreaSearch(startPoly, center, radius, in filter, AreaSearchKind.LocalNeighbourhood);
        return RunAndFinish(s, in filter, polys, parents, default, out count, out truncated);
    }

    /// <summary>Local neighbourhood with the plain filter.</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public AreaQueryStatus FindLocalNeighbourhood(PolyRef startPoly, Vector3D center, double radius, QueryFilter? filter, Span<PolyRef> polys, Span<PolyRef> parents, out int count, out bool truncated)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return FindLocalNeighbourhood(startPoly, center, radius, in f, polys, parents, out count, out truncated);
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private AreaQueryStatus RunAndFinish<TFilter>(AreaQueryStatus begin, in TFilter filter, Span<PolyRef> polys, Span<PolyRef> parents, Span<double> dist, out int count, out bool truncated)
        where TFilter : struct, IQueryFilter
    {
        if (begin != AreaQueryStatus.InProgress)
        {
            count = 0;
            truncated = false;
            return begin;
        }
        UpdateAreaSearch(int.MaxValue, in filter);
        return FinishAreaSearch(polys, parents, dist, out count, out truncated);
    }

    // ---- shape tests ----

    // Orientation of c relative to a→b in the X/Z plane, exact. Magnitudes below 2^-100 are treated as zero so the
    // predicate's input range is never exceeded.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    internal static int Side(double ax, double az, double bx, double bz, double cx, double cz) =>
        ExactPredicates.Orient2D(Tiny(ax), Tiny(az), Tiny(bx), Tiny(bz), Tiny(cx), Tiny(cz));

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private static double Tiny(double v) => Math.Abs(v) < 7.888609052210118E-31 ? 0.0 : v;

    // Does the segment touch the query shape? If so, key = squared distance from the shape's reference point to the segment.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private bool Touches(double ax, double az, double bx, double bz, out double key)
    {
        key = SegDist2(_aCx, _aCz, ax, az, bx, bz);
        if (_aIsCircle)
        {
            return key <= _aR2;
        }
        if (InsideShape(ax, az) || InsideShape(bx, bz))
        {
            return true;
        }
        int n = _aShapeN;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            if (SegmentsMeet(ax, az, bx, bz, _aShape[2 * i], _aShape[(2 * i) + 1], _aShape[2 * j], _aShape[(2 * j) + 1]))
            {
                return true;
            }
        }
        return false;
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private bool InsideShape(double x, double z)
    {
        int n = _aShapeN;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            if (Side(_aShape[2 * i], _aShape[(2 * i) + 1], _aShape[2 * j], _aShape[(2 * j) + 1], x, z) < 0)
            {
                return false;
            }
        }
        return true;
    }

    // Closed segments a-b and c-d share a point.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private static bool SegmentsMeet(double ax, double az, double bx, double bz, double cx, double cz, double dx, double dz)
    {
        int o1 = Side(ax, az, bx, bz, cx, cz), o2 = Side(ax, az, bx, bz, dx, dz);
        int o3 = Side(cx, cz, dx, dz, ax, az), o4 = Side(cx, cz, dx, dz, bx, bz);
        if (o1 * o2 < 0 && o3 * o4 < 0)
        {
            return true;
        }
        return (o1 == 0 && OnBox(ax, az, bx, bz, cx, cz)) || (o2 == 0 && OnBox(ax, az, bx, bz, dx, dz))
            || (o3 == 0 && OnBox(cx, cz, dx, dz, ax, az)) || (o4 == 0 && OnBox(cx, cz, dx, dz, bx, bz));
    }

    // For a point known to be collinear with a-b: is it between the ends?
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private static bool OnBox(double ax, double az, double bx, double bz, double px, double pz) =>
        px >= Math.Min(ax, bx) && px <= Math.Max(ax, bx) && pz >= Math.Min(az, bz) && pz <= Math.Max(az, bz);

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    internal static double SegDist2(double px, double pz, double ax, double az, double bx, double bz)
    {
        double ex = bx - ax, ez = bz - az;
        double len2 = (ex * ex) + (ez * ez);
        double s = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0.0, 1.0) : 0.0;
        double qx = ax + (ex * s), qz = az + (ez * s);
        return ((px - qx) * (px - qx)) + ((pz - qz) * (pz - qz));
    }

    // ---- open list: binary min-heap on (key, insertion sequence) ----

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void AreaPush(double f, int node)
    {
        if (_apCount == _apF.Length)
        {
            Array.Resize(ref _apF, _apCount * 2);
            Array.Resize(ref _apSeq, _apCount * 2);
            Array.Resize(ref _apNode, _apCount * 2);
        }
        int i = _apCount++;
        int seq = _aSeq++;
        while (i > 0)
        {
            int parent = (i - 1) >> 1;
            if (_apF[parent] < f || (_apF[parent] == f && _apSeq[parent] < seq))
            {
                break;
            }
            _apF[i] = _apF[parent];
            _apSeq[i] = _apSeq[parent];
            _apNode[i] = _apNode[parent];
            i = parent;
        }
        _apF[i] = f;
        _apSeq[i] = seq;
        _apNode[i] = node;
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private int AreaPop()
    {
        int result = _apNode[0];
        int last = --_apCount;
        if (last > 0)
        {
            double f = _apF[last];
            int seq = _apSeq[last];
            int node = _apNode[last];
            int i = 0;
            while (true)
            {
                int c = (2 * i) + 1;
                if (c >= last)
                {
                    break;
                }
                int r = c + 1;
                if (r < last && (_apF[r] < _apF[c] || (_apF[r] == _apF[c] && _apSeq[r] < _apSeq[c])))
                {
                    c = r;
                }
                if (f < _apF[c] || (f == _apF[c] && seq < _apSeq[c]))
                {
                    break;
                }
                _apF[i] = _apF[c];
                _apSeq[i] = _apSeq[c];
                _apNode[i] = _apNode[c];
                i = c;
            }
            _apF[i] = f;
            _apSeq[i] = seq;
            _apNode[i] = node;
        }
        return result;
    }
}
