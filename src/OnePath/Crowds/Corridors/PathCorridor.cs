using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Crowds.Corridors;

/// <summary>What <see cref="PathCorridor.Validate"/> found.</summary>
public enum CorridorStatus
{
    /// <summary>Every polygon checked is valid, accepted by the filter and connected to the previous one.</summary>
    Valid = 0,

    /// <summary>The corridor was cut after its last good polygon; it no longer reaches the target and needs a replan.</summary>
    Trimmed = 1,

    /// <summary>The first polygon itself is gone or rejected: the agent must be relocated onto the mesh.</summary>
    Invalid = 2,
}

/// <summary>
/// The polygon path of one agent from where it stands to where it is going, kept valid while the agent moves, the target
/// moves and the mesh changes. The agent's position is always inside the first polygon; polygons the agent has passed are
/// trimmed away, so the corridor is the part of the route still ahead. Corners are found by string pulling
/// (<c>NavMeshQuery.FindStraightPath</c>). The idea is the path corridor of Mononen's public talks and writing; the
/// exact rules for merging, validating and repairing are in <c>docs/crowds.md</c>.
/// <para>
/// All state is plain data (<see cref="Write"/> and <see cref="Read"/> give a byte-exact form). A corridor is not thread-safe
/// and works with the <see cref="NavMeshQuery"/> and filter passed to each call (they are not stored).
/// </para>
/// </summary>
public sealed class PathCorridor
{
    /// <summary>Largest corridor capacity.</summary>
    public const int MaxCapacity = 4096;

    private const byte FormatVersion = 1;
    private const int FlagComplete = 1, FlagNeedsReplan = 2, FlagFromStart = 4;

    private static readonly QueryFilter s_defaultFilter = new();

    private readonly PolyRef[] _path;
    private readonly PolyRef[] _scratch;
    private readonly StraightPathPoint[] _pullScratch = new StraightPathPoint[64];
    private int _count;
    private Vector3D _position;
    private Vector3D _target;
    private PolyRef _targetPoly;
    private bool _complete;
    private bool _needsReplan;
    private bool _fromStart;
    private CorridorReplan? _session;

    /// <summary>Creates an empty corridor (no polygons) that can hold <paramref name="capacity"/> polygons.</summary>
    /// <param name="capacity">Most polygons the corridor keeps, 2 to <see cref="MaxCapacity"/>. Default 256. A longer route is cut and finished by a later replan.</param>
    /// <exception cref="ArgumentOutOfRangeException">The capacity is out of range.</exception>
    public PathCorridor(int capacity = 256)
    {
        if (capacity < 2 || capacity > MaxCapacity)
        {
            throw new ArgumentOutOfRangeException(nameof(capacity), capacity, $"Capacity must be 2 to {MaxCapacity}.");
        }
        _path = new PolyRef[capacity];
        _scratch = new PolyRef[capacity];
    }

    /// <summary>Most polygons the corridor keeps.</summary>
    public int Capacity => _path.Length;

    /// <summary>Number of polygons now in the corridor (0 until <see cref="Reset"/>).</summary>
    public int Count => _count;

    /// <summary>The polygons, the one holding <see cref="Position"/> first.</summary>
    public ReadOnlySpan<PolyRef> Path => _path.AsSpan(0, _count);

    /// <summary>The first polygon (<see cref="PolyRef.Invalid"/> while empty).</summary>
    public PolyRef FirstPoly => _count > 0 ? _path[0] : PolyRef.Invalid;

    /// <summary>The last polygon (<see cref="PolyRef.Invalid"/> while empty).</summary>
    public PolyRef LastPoly => _count > 0 ? _path[_count - 1] : PolyRef.Invalid;

    /// <summary>The agent's position on the surface of the first polygon (world coordinates).</summary>
    public Vector3D Position => _position;

    /// <summary>Where the corridor leads (world coordinates); the last corner is this point, or the nearest point of the last polygon when the corridor is not <see cref="IsComplete"/>.</summary>
    public Vector3D Target => _target;

    /// <summary>The polygon holding <see cref="Target"/>, or <see cref="PolyRef.Invalid"/> if it is not known yet.</summary>
    public PolyRef TargetPoly => _targetPoly;

    /// <summary>True when the last polygon is the target's polygon, so the corridor reaches the target.</summary>
    public bool IsComplete => _complete;

    /// <summary>True when the corridor was cut, was never finished or the target moved: <see cref="Replan"/> should run.</summary>
    public bool NeedsReplan => _needsReplan;

    internal bool ReplanFromStart => _fromStart;

    /// <summary>
    /// Empties the corridor and puts the agent at <paramref name="position"/> in <paramref name="poly"/>; the target becomes that
    /// position (the corridor is complete).
    /// </summary>
    /// <param name="poly">The polygon holding the position (not checked here; the first query that uses it reports a bad one).</param>
    /// <param name="position">World position, finite and within ±2^21.</param>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond ±2^21 (the message names it).</exception>
    public void Reset(PolyRef poly, Vector3D position)
    {
        NavMeshQuery.CheckPos(position, nameof(position));
        _path[0] = poly;
        _count = 1;
        _position = position;
        _target = position;
        _targetPoly = poly;
        _complete = true;
        _needsReplan = false;
        _fromStart = false;
    }

    /// <summary>
    /// Sets a new destination. The corridor keeps its polygons (the agent can go on walking them) but is no longer complete,
    /// and the next <see cref="Replan"/> searches from the agent's polygon.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The target is not finite or is beyond ±2^21.</exception>
    public void SetTarget(Vector3D target)
    {
        NavMeshQuery.CheckPos(target, nameof(target));
        _target = target;
        _targetPoly = PolyRef.Invalid;
        _complete = false;
        _needsReplan = true;
        _fromStart = true;
    }

    /// <summary>Marks the corridor as needing a replan from the agent's polygon (for example after the host stopped trusting it).</summary>
    public void RequestReplan()
    {
        _needsReplan = true;
        _fromStart = true;
        _complete = false;
    }

    /// <summary>
    /// Replaces the corridor with a route found elsewhere (a path query result). The path must start at the polygon holding
    /// <see cref="Position"/> for the corridor to be usable.
    /// </summary>
    /// <param name="target">World position the path leads to.</param>
    /// <param name="targetPoly">The polygon holding <paramref name="target"/>.</param>
    /// <param name="path">Connected polygons, start first; at most <see cref="Capacity"/> are kept.</param>
    /// <param name="complete">True if the last polygon is <paramref name="targetPoly"/>. A path cut to the capacity is never complete.</param>
    /// <exception cref="ArgumentException">The path is empty.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The target is not finite or is beyond ±2^21.</exception>
    public void SetPath(Vector3D target, PolyRef targetPoly, ReadOnlySpan<PolyRef> path, bool complete)
    {
        NavMeshQuery.CheckPos(target, nameof(target));
        if (path.Length == 0)
        {
            throw new ArgumentException("The path is empty.", nameof(path));
        }
        int n = Math.Min(path.Length, _path.Length);
        path[..n].CopyTo(_path);
        _count = n;
        _target = target;
        _targetPoly = targetPoly;
        _complete = complete && n == path.Length;
        _needsReplan = !_complete && n < path.Length;
        _fromStart = false;
    }

    // --- following the agent ---

    /// <summary>
    /// Moves the agent toward <paramref name="newPosition"/> along the surface (sliding along a wall, as
    /// <see cref="NavMeshQuery.MoveAlongSurface(PolyRef, Vector3D, Vector3D, QueryFilter?, Span{PolyRef}, out int)"/> does) and
    /// updates the corridor: the polygons walked are merged in and the ones left behind are trimmed, so the first polygon holds
    /// the new <see cref="Position"/>. If the agent ends in a polygon outside the corridor, the corridor leads back through the
    /// polygons walked.
    /// </summary>
    /// <returns>False, changing nothing, if the first polygon is invalid or does not hold the position, or the move crossed more polygons than the capacity.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond ±2^21.</exception>
    public bool MovePosition(NavMeshQuery query, Vector3D newPosition, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        NavMeshQuery.CheckPos(newPosition, nameof(newPosition));
        if (_count == 0)
        {
            return false;
        }
        MoveResult r = query.MoveAlongSurface(_path[0], _position, newPosition, filter, _scratch, out int n);
        if (r.Status != RaycastStatus.Ok || r.Truncated || n == 0)
        {
            return false;
        }
        _position = r.Position;
        MergeFront(_scratch.AsSpan(0, n), truncate: true);
        return true;
    }

    /// <summary>
    /// Moves the agent to a point that lies inside the first <paramref name="polygonLimit"/> polygons of the corridor, such as a
    /// point on the way to the next corner (a corner's leg runs inside the corridor by construction). The corridor is trimmed
    /// to start at the polygon holding the point: the one with the surface closest in height, the furthest along on a tie.
    /// This follows the corridor itself rather than tracing a ray, so a leg that grazes polygon vertices or runs along a wall
    /// cannot be blocked by rounding at a vertex.
    /// </summary>
    /// <param name="query">The query object (only its mesh is used).</param>
    /// <param name="to">The new position (world); only its horizontal part must lie in a polygon, its height picks between overlapping floors.</param>
    /// <param name="polygonLimit">How many polygons from the start may hold the point, at least 1.</param>
    /// <returns>False, changing nothing, if no polygon within the limit holds the point (within 1e-7 horizontally) or one is invalid: use <see cref="MovePosition"/> instead.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond ±2^21.</exception>
    public bool MoveAlong(NavMeshQuery query, Vector3D to, int polygonLimit = int.MaxValue)
    {
        ArgumentNullException.ThrowIfNull(query);
        NavMeshQuery.CheckPos(to, nameof(to));
        int n = Math.Min(_count, polygonLimit);
        int best = -1;
        double bestDy = double.PositiveInfinity;
        Vector3D bestP = default;
        for (int i = 0; i < n; i++)
        {
            if (!query.ClosestPointOnPoly(_path[i], to, out Vector3D cp, out bool over))
            {
                return false;
            }
            double dx = cp.X - to.X, dz = cp.Z - to.Z;
            if (!over && (dx * dx) + (dz * dz) > 1e-14)
            {
                continue;
            }
            double dy = Math.Abs(cp.Y - to.Y);
            if (best < 0 || dy <= bestDy + 1e-9)
            {
                bestDy = Math.Min(bestDy, dy);
                best = i;
                bestP = cp;
            }
        }
        if (best < 0)
        {
            return false;
        }
        if (best > 0)
        {
            Array.Copy(_path, best, _path, 0, _count - best);
            _count -= best;
        }
        _position = bestP;
        return true;
    }

    /// <summary>
    /// Moves the target along the surface from its current place to <paramref name="newTarget"/>, merging the polygons walked
    /// into the end of the corridor. If the corridor is not complete, the target is only recorded and a replan is requested.
    /// </summary>
    /// <returns>False, changing nothing, if the last polygon is invalid or does not hold the target, or the move crossed more polygons than the capacity.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond ±2^21.</exception>
    public bool MoveTarget(NavMeshQuery query, Vector3D newTarget, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        NavMeshQuery.CheckPos(newTarget, nameof(newTarget));
        if (_count == 0)
        {
            return false;
        }
        if (!_complete)
        {
            _target = newTarget;
            _targetPoly = PolyRef.Invalid;
            _needsReplan = true;
            return true;
        }
        MoveResult r = query.MoveAlongSurface(_path[_count - 1], _target, newTarget, filter, _scratch, out int n);
        if (r.Status != RaycastStatus.Ok || r.Truncated || n == 0)
        {
            return false;
        }
        // The end of the corridor is the front of its reverse; a merge that would not fit is refused, not cut, so the agent's end survives.
        Array.Reverse(_path, 0, _count);
        bool merged = MergeFront(_scratch.AsSpan(0, n), truncate: false);
        Array.Reverse(_path, 0, _count);
        if (!merged)
        {
            return false;
        }
        _target = r.Position;
        _targetPoly = _path[_count - 1];
        return true;
    }

    // Merges a polygon chain that starts at the first polygon into the front of the corridor: the chain is walked back from
    // its last polygon to the farthest corridor polygon it touches, and what follows that polygon in the corridor is kept.
    // The result is connected, starts at the chain's last polygon and holds no polygon twice (the kept tail has no chain polygon).
    private bool MergeFront(ReadOnlySpan<PolyRef> chain, bool truncate)
    {
        int bestI = -1, bestJ = -1;
        for (int i = _count - 1; i >= 0 && bestI < 0; i--)
        {
            for (int j = chain.Length - 1; j >= 0; j--)
            {
                if (_path[i] == chain[j])
                {
                    bestI = i;
                    bestJ = j;
                    break;
                }
            }
        }
        if (bestI < 0)
        {
            return false; // cannot happen when the chain starts at the first polygon
        }
        int head = chain.Length - bestJ;
        int tail = _count - bestI - 1;
        if (head + tail > _path.Length)
        {
            if (!truncate)
            {
                return false;
            }
            tail = _path.Length - head;
            _complete = false;
            _needsReplan = true;
        }
        if (tail > 0)
        {
            Array.Copy(_path, bestI + 1, _path, head, tail);
        }
        for (int k = 0; k < head; k++)
        {
            _path[k] = chain[chain.Length - 1 - k];
        }
        _count = head + tail;
        return true;
    }

    // --- corners ---

    /// <summary>
    /// The corners of the shortest way through the corridor from <see cref="Position"/> to <see cref="Target"/> (string pulling).
    /// The first point is the position itself (flag <see cref="StraightPathMarks.Start"/>); the last has
    /// <see cref="StraightPathMarks.End"/>, and is <see cref="Target"/> only when <see cref="IsComplete"/> (otherwise the nearest
    /// point of the last polygon). A point flagged <see cref="StraightPathMarks.OffMeshConnection"/> is the entry of an off-mesh
    /// link and the next point its exit.
    /// </summary>
    /// <param name="query">The query object to use.</param>
    /// <param name="corners">Receives the points.</param>
    /// <param name="count">Number written.</param>
    /// <returns><see cref="QueryStatus.Success"/>, or <see cref="QueryStatus.InvalidCorridor"/> (nothing written) when a polygon is stale or two polygons no longer touch: call <see cref="Validate"/>.</returns>
    public QueryStatus FindCorners(NavMeshQuery query, Span<StraightPathPoint> corners, out int count)
    {
        ArgumentNullException.ThrowIfNull(query);
        count = 0;
        if (_count == 0)
        {
            return QueryStatus.InvalidCorridor;
        }
        return query.FindStraightPath(_position, _target, _path.AsSpan(0, _count), corners, out count, out _);
    }

    // --- shortcuts ---

    /// <summary>
    /// Visibility shortcut: casts a ray along the surface from the agent toward <paramref name="point"/> (cut to
    /// <paramref name="maxRange"/>) and, where the ray runs through polygons of the corridor, replaces the corridor's front
    /// by the polygons the ray crossed. One unit of <paramref name="budget"/> is one polygon of the ray that is considered; a
    /// ray longer than the budget is cut there, so the result depends only on the budget, not on time.
    /// </summary>
    /// <param name="query">The query object.</param>
    /// <param name="point">A point ahead (usually a corner).</param>
    /// <param name="maxRange">Longest ray, horizontal distance; positive.</param>
    /// <param name="filter">Which polygons the ray may cross.</param>
    /// <param name="budget">Most polygons considered, at least 2 (smaller values do nothing). At most <see cref="Capacity"/> are used.</param>
    /// <returns>True if the corridor changed.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or is beyond ±2^21, or the range is not positive.</exception>
    public bool OptimizeVisibility(NavMeshQuery query, Vector3D point, double maxRange, QueryFilter? filter = null, int budget = 32)
    {
        ArgumentNullException.ThrowIfNull(query);
        NavMeshQuery.CheckPos(point, nameof(point));
        if (!(maxRange > 0) || double.IsNaN(maxRange))
        {
            throw new ArgumentOutOfRangeException(nameof(maxRange), maxRange, "The range must be positive.");
        }
        if (_count < 2 || budget < 2)
        {
            return false;
        }
        double dx = point.X - _position.X, dz = point.Z - _position.Z;
        double d = Math.Sqrt((dx * dx) + (dz * dz));
        Vector3D end = point;
        if (d > maxRange)
        {
            double s = maxRange / d;
            end = new Vector3D(_position.X + (dx * s), _position.Y + ((point.Y - _position.Y) * s), _position.Z + (dz * s));
        }
        int len = Math.Min(budget, _scratch.Length);
        RaycastResult r = query.Raycast(_path[0], _position, end, filter, _scratch.AsSpan(0, len), out int n);
        if (r.Status != RaycastStatus.Ok || n < 2)
        {
            return false;
        }
        ReadOnlySpan<PolyRef> chain = _scratch.AsSpan(0, n);
        int bestI = -1, bestJ = -1;
        for (int i = _count - 1; i >= 1 && bestI < 0; i--)
        {
            for (int j = n - 1; j >= 1; j--)
            {
                if (_path[i] == chain[j])
                {
                    bestI = i;
                    bestJ = j;
                    break;
                }
            }
        }
        if (bestI < 0)
        {
            return false;
        }
        // Replace corridor[0..bestI] by chain[0..bestJ]; the corridor tail after bestI holds no chain polygon.
        int head = bestJ + 1;
        int tail = _count - bestI - 1;
        bool same = head == bestI + 1;
        for (int k = 0; same && k < head; k++)
        {
            same = _path[k] == chain[k];
        }
        if (same || head + tail > _path.Length)
        {
            return false; // unchanged, or a detour through more polygons than the corridor can hold: not a shortcut
        }
        if (tail > 0)
        {
            Array.Copy(_path, bestI + 1, _path, head, tail);
        }
        for (int k = 0; k < head; k++)
        {
            _path[k] = chain[k];
        }
        _count = head + tail;
        return true;
    }

    /// <summary>
    /// Local topology optimization: a small path search from the agent's polygon to a polygon up to
    /// <paramref name="window"/> polygons ahead, under an iteration budget; if it finds a route it replaces that part of the
    /// corridor (loops where the new route meets the rest are cut). This lets the corridor take a better way round an obstacle
    /// that the walked-in route ignored.
    /// </summary>
    /// <param name="query">The query object (its current search is replaced).</param>
    /// <param name="filter">Polygon filter and area costs.</param>
    /// <param name="budget">Most search iterations (node expansions), at least 1. Default 32. An unfinished search changes nothing.</param>
    /// <param name="window">How many polygons ahead the search aims, at least 3. Default 16.</param>
    /// <returns>True if the corridor changed.</returns>
    public bool OptimizeTopology(NavMeshQuery query, QueryFilter? filter = null, int budget = 32, int window = 16)
    {
        ArgumentNullException.ThrowIfNull(query);
        ArgumentOutOfRangeException.ThrowIfLessThan(budget, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(window, 3);
        if (_count < 3)
        {
            return false;
        }
        int t = Math.Min(_count - 1, window - 1);
        if (!query.ClosestPointOnPoly(_path[t], _target, out Vector3D endPos, out _))
        {
            return false;
        }
        QueryStatus s = query.BeginPath(_path[0], _path[t], _position, endPos, filter);
        if (s != QueryStatus.InProgress || query.UpdatePath(budget) != QueryStatus.Success)
        {
            return false;
        }
        Span<PolyRef> found = _scratch;
        query.FinishPath(found, out int n, out bool truncated, out _);
        if (truncated || n == 0 || found[0] != _path[0] || found[n - 1] != _path[t])
        {
            return false;
        }
        int tail = _count - t - 1;
        bool same = n == t + 1;
        for (int k = 0; same && k < n; k++)
        {
            same = _path[k] == found[k];
        }
        if (same)
        {
            return false;
        }
        // A route that is cheaper to the search is not always shorter to walk (a polygon-node search charges polygon centres, and
        // its choice between two ways round an obstacle can change with the agent's place in its polygon, so the corridor flips
        // back and forth and the agent stands between them). Take the new route only if the string-pulled walk to the same point
        // gets shorter.
        double oldLen = PulledLength(query, _path.AsSpan(0, t + 1), endPos);
        double newLen = PulledLength(query, found[..n], endPos);
        if (!(newLen < oldLen * (1 - 1e-4)))
        {
            return false;
        }
        // Splice found ++ corridor[t+1..] with loops cut (first occurrence kept); built in place after shifting the tail.
        // The result is at most n + tail long, which can exceed the capacity only if the found route is longer than the window.
        int total = n + tail;
        if (total > _path.Length)
        {
            tail = _path.Length - n;
            total = _path.Length;
            _complete = false;
            _needsReplan = true;
        }
        if (tail > 0)
        {
            Array.Copy(_path, t + 1, _path, n, tail);
        }
        found[..n].CopyTo(_path);
        _count = CutLoops(_path, total);
        return true;
    }

    // Length of the shortest way from the agent's position to `end` through `polys` (infinity when it cannot be pulled).
    private double PulledLength(NavMeshQuery query, ReadOnlySpan<PolyRef> polys, Vector3D end)
    {
        if (query.FindStraightPath(_position, end, polys, _pullScratch, out int n, out bool truncated) != QueryStatus.Success || truncated || n < 1)
        {
            return double.PositiveInfinity;
        }
        double sum = 0;
        for (int i = 1; i < n; i++)
        {
            sum += Vector3D.Distance(_pullScratch[i - 1].Position, _pullScratch[i].Position);
        }
        return sum;
    }

    // Removes loops in place: a polygon met again cuts everything between its two occurrences (the first stays).
    internal static int CutLoops(PolyRef[] path, int count)
    {
        int w = 0;
        for (int i = 0; i < count; i++)
        {
            PolyRef p = path[i];
            int seen = -1;
            for (int j = 0; j < w; j++)
            {
                if (path[j] == p)
                {
                    seen = j;
                    break;
                }
            }
            if (seen >= 0)
            {
                w = seen + 1;
            }
            else
            {
                path[w++] = p;
            }
        }
        return w;
    }

    // --- mesh changes ---

    /// <summary>
    /// Checks the first <paramref name="lookAhead"/> polygons against the mesh as it is now: each must be a valid reference,
    /// be accepted by the filter and be connected to the one before it (an in-tile edge, a tile-portal link or an enabled
    /// off-mesh link the filter accepts). A bad polygon at index k &gt; 0 cuts the corridor to its first k polygons, marks it
    /// incomplete and asks for a replan; a bad first polygon changes nothing and returns <see cref="CorridorStatus.Invalid"/>.
    /// </summary>
    /// <param name="query">The query object (only its mesh is used).</param>
    /// <param name="filter">The polygon filter (null accepts everything).</param>
    /// <param name="lookAhead">How many polygons to check, at least 1. Default 8; <see cref="int.MaxValue"/> checks all.</param>
    public CorridorStatus Validate(NavMeshQuery query, QueryFilter? filter = null, int lookAhead = 8)
    {
        ArgumentNullException.ThrowIfNull(query);
        ArgumentOutOfRangeException.ThrowIfLessThan(lookAhead, 1);
        if (_count == 0)
        {
            return CorridorStatus.Invalid;
        }
        QueryFilter f = filter ?? s_defaultFilter;
        TiledNavMesh mesh = query.Mesh;
        int n = Math.Min(_count, lookAhead);
        int bad = -1;
        for (int i = 0; i < n; i++)
        {
            if (!PolyOk(mesh, _path[i], f) || (i > 0 && !Connected(mesh, _path[i - 1], _path[i], f)))
            {
                bad = i;
                break;
            }
        }
        if (bad < 0)
        {
            // The target polygon is part of the corridor's meaning: if it went stale, the corridor is no longer complete.
            if (_complete && n == _count && !PolyOk(mesh, _targetPoly, f))
            {
                _complete = false;
                _needsReplan = true;
                _fromStart = false;
            }
            return CorridorStatus.Valid;
        }
        if (bad == 0)
        {
            return CorridorStatus.Invalid;
        }
        _count = bad;
        _complete = false;
        _needsReplan = true;
        _fromStart = false;
        return CorridorStatus.Trimmed;
    }

    private static bool PolyOk(TiledNavMesh mesh, PolyRef p, QueryFilter f) =>
        mesh.TryGetPoly(p, out NavTile t, out int pi) && f.Accepts(t.PolyFlags[pi], t.PolyAreas[pi]);

    // True if b can be entered from a: through a shared in-tile edge, a tile-portal link or an off-mesh link (which the
    // filter must accept by the link's own flags and area), and b itself passes the filter.
    internal static bool Connected(TiledNavMesh mesh, PolyRef a, PolyRef b, QueryFilter f)
    {
        if (!mesh.TryGetPoly(a, out NavTile ta, out int pa) || !mesh.TryGetPoly(b, out NavTile tb, out int pb) || !f.Accepts(tb.PolyFlags[pb], tb.PolyAreas[pb]))
        {
            return false;
        }
        if (a.TileSlot == b.TileSlot)
        {
            int start = ta.PolyFirstIndex(pa);
            int n = ta.PolyVertexCounts[pa];
            ReadOnlySpan<int> nbr = ta.EdgeNeighbors;
            for (int e = 0; e < n; e++)
            {
                if (nbr[start + e] == pb + 1)
                {
                    return true;
                }
            }
        }
        foreach (NavLink l in mesh.GetLinks(a))
        {
            if (l.Target != b)
            {
                continue;
            }
            if (l.Kind == NavLinkKind.Portal)
            {
                return true;
            }
            if (mesh.GetTileBySlot(l.OwnerSlot) is { } owner && (uint)l.Index < (uint)owner.OffMeshLinks.Length)
            {
                OffMeshLink o = owner.OffMeshLinks[l.Index];
                if (f.Accepts(o.PolyFlags, o.Area))
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Puts the agent back on the mesh after its polygon was lost: the corridor becomes the single nearest accepted polygon to
    /// <see cref="Position"/> (position moved onto it), the target is kept, and a replan from the start is requested.
    /// </summary>
    /// <param name="query">The query object.</param>
    /// <param name="filter">The polygon filter.</param>
    /// <param name="searchExtents">Half-size of the search box around the position; null means (2, 4, 2).</param>
    /// <returns>False if no accepted polygon is near (nothing changed).</returns>
    public bool Relocate(NavMeshQuery query, QueryFilter? filter = null, Vector3D? searchExtents = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        NearestPoly np = query.FindNearestPoly(_position, searchExtents ?? new Vector3D(2, 4, 2), filter);
        if (!np.Poly.IsSet)
        {
            return false;
        }
        _path[0] = np.Poly;
        _count = 1;
        _position = np.Point;
        _complete = np.Poly == _targetPoly;
        _needsReplan = !_complete;
        _fromStart = true;
        return true;
    }

    /// <summary>
    /// Repairs or recomputes the route to the target within an iteration budget: one search from the end of the still-valid part
    /// of the corridor (or from the agent's polygon after <see cref="SetTarget"/>, <see cref="Relocate"/> or
    /// <see cref="RequestReplan"/>), falling back to a search from the agent's polygon if that finds no route. The result is
    /// spliced into the corridor. When the budget runs out the corridor leads to the polygon closest to the target found so
    /// far and a replan stays requested; when no route exists the corridor leads to the closest polygon reachable and no replan
    /// stays requested. For slicing one search over several calls, use <see cref="CorridorReplan"/>.
    /// </summary>
    /// <param name="query">The query object (its current search is replaced).</param>
    /// <param name="filter">Polygon filter and area costs.</param>
    /// <param name="budget">Most search iterations (node expansions), at least 1.</param>
    /// <param name="searchExtents">Half-size of the box searched for the polygon nearest an unresolved target; null means (2, 4, 2).</param>
    /// <returns><see cref="QueryStatus.Success"/> (complete), <see cref="QueryStatus.Partial"/> (no route; corridor ends at the closest polygon), <see cref="QueryStatus.InProgress"/> (budget used up; corridor extended), or why no search could start (<see cref="QueryStatus.InvalidStartPoly"/>, <see cref="QueryStatus.NoEndNearby"/>); then nothing changed.</returns>
    public QueryStatus Replan(NavMeshQuery query, QueryFilter? filter = null, int budget = 2048, Vector3D? searchExtents = null)
    {
        _session ??= new CorridorReplan();
        _session.SearchExtents = searchExtents ?? new Vector3D(2, 4, 2);
        QueryStatus s = _session.Begin(query, this, filter);
        if (s != QueryStatus.InProgress)
        {
            return s;
        }
        _session.Update(budget);
        return _session.Apply();
    }

    // --- used by CorridorReplan and the follower ---

    internal PolyRef[] PathArray => _path;

    internal int ReplaceFrom(int keep, ReadOnlySpan<PolyRef> found, Vector3D target, PolyRef targetPoly, bool complete, bool needsReplan)
    {
        // keep = number of leading corridor polygons that stay; found follows them (found[0] is the polygon that was at keep, or the start).
        int w = keep;
        foreach (PolyRef p in found)
        {
            if (w == _path.Length)
            {
                complete = false;
                needsReplan = true;
                break;
            }
            _path[w++] = p;
        }
        _count = CutLoops(_path, w);
        _target = target;
        _targetPoly = targetPoly;
        _complete = complete && _count > 0 && _path[_count - 1] == targetPoly;
        _needsReplan = needsReplan;
        _fromStart = false;
        return _count;
    }

    internal void SetPosition(Vector3D p) => _position = p;

    /// <summary>
    /// The agent crossed an off-mesh link: puts it at <paramref name="exit"/> in <paramref name="exitPoly"/>, which must be in
    /// the corridor (it is trimmed to start there). A follower does this itself; use it when a host drives the crossing at the
    /// raw level.
    /// </summary>
    /// <returns>False, changing nothing, if the polygon is not in the corridor.</returns>
    public bool CrossOffMesh(PolyRef exitPoly, Vector3D exit)
    {
        for (int i = 0; i < _count; i++)
        {
            if (_path[i] == exitPoly)
            {
                if (i > 0)
                {
                    Array.Copy(_path, i, _path, 0, _count - i);
                    _count -= i;
                }
                _position = exit;
                return true;
            }
        }
        return false;
    }

    // Trims the corridor so the polygon `poly` (found among the first few) is first. Used to snap to an off-mesh link entry.
    internal bool TrimTo(PolyRef poly)
    {
        for (int i = 0; i < _count; i++)
        {
            if (_path[i] == poly)
            {
                if (i > 0)
                {
                    Array.Copy(_path, i, _path, 0, _count - i);
                    _count -= i;
                }
                return true;
            }
        }
        return false;
    }

    // --- byte-exact state ---

    /// <summary>Size in bytes of <see cref="Write"/>'s output for the current state.</summary>
    public int ByteSize => 1 + 1 + 4 + 4 + 48 + 8 + (8 * _count);

    /// <summary>
    /// Writes the state (see <c>docs/crowds.md</c>, "State bytes"): the same state gives the same bytes on every platform.
    /// A search being sliced through <see cref="CorridorReplan"/> is not part of the state.
    /// </summary>
    /// <param name="destination">At least <see cref="ByteSize"/> bytes.</param>
    /// <returns>The number of bytes written.</returns>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int Write(Span<byte> destination)
    {
        int size = ByteSize;
        if (destination.Length < size)
        {
            throw new ArgumentException($"The destination holds {destination.Length} bytes; {size} are needed.", nameof(destination));
        }
        var w = new StateWriter(destination);
        w.U8(FormatVersion);
        w.U8((byte)((_complete ? FlagComplete : 0) | (_needsReplan ? FlagNeedsReplan : 0) | (_fromStart ? FlagFromStart : 0)));
        w.I32(_path.Length);
        w.I32(_count);
        w.V3(_position);
        w.V3(_target);
        w.U64(_targetPoly.Value);
        for (int i = 0; i < _count; i++)
        {
            w.U64(_path[i].Value);
        }
        return w.Position;
    }

    /// <summary>Returns the state as a new array (see <see cref="Write"/>).</summary>
    public byte[] ToBytes()
    {
        byte[] b = new byte[ByteSize];
        Write(b);
        return b;
    }

    /// <summary>Reads a corridor written by <see cref="Write"/>.</summary>
    /// <param name="source">The bytes.</param>
    /// <param name="consumed">Number of bytes used.</param>
    /// <exception cref="NavMeshFormatException">The bytes are truncated, of another version or hold impossible values (the message names which).</exception>
    public static PathCorridor Read(ReadOnlySpan<byte> source, out int consumed)
    {
        var r = new StateReader(source, "corridor");
        PathCorridor c = ReadFrom(ref r);
        consumed = r.Position;
        return c;
    }

    internal static PathCorridor ReadFrom(ref StateReader r)
    {
        r.Version(FormatVersion);
        int flags = r.U8();
        if ((flags & ~7) != 0)
        {
            throw r.Error("flags", flags);
        }
        int capacity = r.I32();
        if (capacity < 2 || capacity > MaxCapacity)
        {
            throw r.Error("capacity", capacity);
        }
        int count = r.I32();
        if (count < 0 || count > capacity)
        {
            throw r.Error("polygon count", count);
        }
        var c = new PathCorridor(capacity)
        {
            _count = count,
            _complete = (flags & FlagComplete) != 0,
            _needsReplan = (flags & FlagNeedsReplan) != 0,
            _fromStart = (flags & FlagFromStart) != 0,
        };
        c._position = r.V3("position");
        c._target = r.V3("target");
        c._targetPoly = new PolyRef(r.U64());
        for (int i = 0; i < count; i++)
        {
            c._path[i] = new PolyRef(r.U64());
        }
        if (count == 0 && c._complete)
        {
            throw r.Error("complete flag of an empty corridor", 1);
        }
        return c;
    }

    internal void WriteTo(ref StateWriter w) => w.Skip(Write(w.Remaining));
}
