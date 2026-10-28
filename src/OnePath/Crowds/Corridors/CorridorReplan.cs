using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Crowds.Corridors;

/// <summary>
/// A corridor replan that can be spread over several calls (frames) by an iteration budget: <see cref="Begin"/>, then
/// <see cref="Update"/> until it stops returning <see cref="QueryStatus.InProgress"/>, then <see cref="Apply"/>. The
/// result after any split of the same total work equals one unlimited call.
/// <para>
/// The search runs inside the <see cref="NavMeshQuery"/> given to <see cref="Begin"/>: that query must not be used for anything
/// else (another search, a corridor shortcut) between <see cref="Begin"/> and <see cref="Apply"/>, and the corridor must not be
/// changed in between. Share one session object between corridors that are replanned one after the other.
/// </para>
/// </summary>
public sealed class CorridorReplan
{
    private enum Phase { None, Splice, FromStart, Done, Applied }

    private NavMeshQuery? _query;
    private PathCorridor? _corridor;
    private QueryFilter? _filter;
    private Phase _phase;
    private QueryStatus _status;
    private int _keep;
    private PolyRef _endPoly;
    private Vector3D _endPos;
    private Vector3D _newTarget;
    private PolyRef[] _found = [];

    /// <summary>Half-size of the box searched for the polygon nearest to the target. Default (2, 4, 2).</summary>
    public Vector3D SearchExtents { get; set; } = new(2, 4, 2);

    /// <summary>True between a successful <see cref="Begin"/> and <see cref="Apply"/>.</summary>
    public bool IsActive => _phase is Phase.Splice or Phase.FromStart;

    /// <summary>Search iterations used so far by this replan.</summary>
    public int Iterations { get; private set; }

    /// <summary>
    /// Starts a replan of <paramref name="corridor"/>: from the end of its valid part, or from the agent's polygon if the
    /// corridor asks for a full replan (after <see cref="PathCorridor.SetTarget"/>, <see cref="PathCorridor.Relocate"/>,
    /// <see cref="PathCorridor.RequestReplan"/>) or the splice search finds no route. The target polygon is the accepted
    /// polygon nearest to <see cref="PathCorridor.Target"/> within <see cref="SearchExtents"/>. The target itself is kept as given, so a
    /// later replan asks the same question about the same point (snapping it would let a boundary point resolve to another floor).
    /// </summary>
    /// <returns><see cref="QueryStatus.InProgress"/> when a search runs; otherwise why none can: <see cref="QueryStatus.InvalidStartPoly"/> (the corridor is empty or its polygons are stale: validate or relocate first) or <see cref="QueryStatus.NoEndNearby"/>.</returns>
    public QueryStatus Begin(NavMeshQuery query, PathCorridor corridor, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        ArgumentNullException.ThrowIfNull(corridor);
        _phase = Phase.None;
        Iterations = 0;
        if (corridor.Count == 0)
        {
            return _status = QueryStatus.InvalidStartPoly;
        }
        Vector3D target = corridor.Target;
        _newTarget = target;
        // The target polygon is resolved afresh each time, so a replan after a mesh change answers the same question a new
        // query would: where is the nearest accepted polygon to the target now?
        NearestPoly np = query.FindNearestPoly(target, SearchExtents, filter);
        if (!np.Poly.IsSet)
        {
            return _status = QueryStatus.NoEndNearby;
        }
        PolyRef endPoly = np.Poly;
        _endPos = np.Point;
        _endPoly = endPoly;
        _query = query;
        _corridor = corridor;
        _filter = filter;
        int e = corridor.ReplanFromStart ? 0 : corridor.Count - 1;
        QueryStatus s = QueryStatus.InvalidStartPoly;
        if (e > 0)
        {
            PolyRef sp = corridor.Path[e];
            if (query.ClosestPointOnPoly(sp, corridor.Position, out Vector3D startPos, out _))
            {
                s = query.BeginPath(sp, endPoly, startPos, _endPos, filter);
            }
            if (s == QueryStatus.InProgress)
            {
                _keep = e;
                _phase = Phase.Splice;
                return _status = s;
            }
        }
        s = query.BeginPath(corridor.Path[0], endPoly, corridor.Position, _endPos, filter);
        if (s == QueryStatus.InProgress)
        {
            _keep = 0;
            _phase = Phase.FromStart;
        }
        return _status = s;
    }

    /// <summary>
    /// Runs up to <paramref name="budget"/> search iterations (node expansions) in total, moving on to the search from the
    /// agent's polygon if the first one finds no route.
    /// </summary>
    /// <param name="budget">Iterations, at least 1.</param>
    /// <returns><see cref="QueryStatus.InProgress"/> while unfinished, then <see cref="QueryStatus.Success"/> or <see cref="QueryStatus.Partial"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The budget is below 1.</exception>
    public QueryStatus Update(int budget)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(budget, 1);
        if (_phase is not (Phase.Splice or Phase.FromStart))
        {
            return _status;
        }
        NavMeshQuery q = _query!;
        PathCorridor c = _corridor!;
        int left = budget;
        while (true)
        {
            int before = q.Iterations;
            QueryStatus s = q.UpdatePath(left);
            int used = q.Iterations - before;
            left -= used;
            Iterations += used;
            if (s == QueryStatus.InProgress)
            {
                return _status = s;
            }
            if (s == QueryStatus.Partial && _phase == Phase.Splice)
            {
                // No route from the end of the valid part (a one-way link, say): try again from the agent's polygon.
                QueryStatus b = q.BeginPath(c.Path[0], _endPoly, c.Position, _endPos, _filter);
                if (b != QueryStatus.InProgress)
                {
                    _phase = Phase.Done;
                    return _status = b;
                }
                _phase = Phase.FromStart;
                _keep = 0;
                if (left <= 0)
                {
                    return _status = QueryStatus.InProgress;
                }
                continue;
            }
            _phase = Phase.Done;
            return _status = s;
        }
    }

    /// <summary>
    /// Ends the search and writes its route into the corridor. After <see cref="QueryStatus.Success"/> the corridor is complete;
    /// after <see cref="QueryStatus.Partial"/> it leads to the closest polygon found and no replan stays requested; if the
    /// search is unfinished (budget used up) it leads to the closest polygon so far and a replan stays requested.
    /// </summary>
    /// <returns>The status applied, or the failure from <see cref="Begin"/> (nothing changed).</returns>
    public QueryStatus Apply()
    {
        if (_phase is Phase.None or Phase.Applied)
        {
            return _status;
        }
        NavMeshQuery q = _query!;
        PathCorridor c = _corridor!;
        int cap = c.Capacity;
        if (_found.Length < cap)
        {
            _found = new PolyRef[cap];
        }
        QueryStatus s = q.FinishPath(_found, out int n, out bool truncated, out _);
        bool complete = s == QueryStatus.Success && !truncated;
        bool needs = s == QueryStatus.InProgress || truncated;
        if (n > 0)
        {
            c.ReplaceFrom(_keep, _found.AsSpan(0, n), _newTarget, _endPoly, complete, needs);
        }
        _phase = Phase.Applied;
        return _status = s;
    }
}
