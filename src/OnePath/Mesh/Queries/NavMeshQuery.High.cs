using System;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    private PolyRef[] _corridorBuf = new PolyRef[256];
    private StraightPathPoint[] _pointBuf = new StraightPathPoint[256];

    private static readonly PathOptions s_defaultOptions = new();

    /// <summary>
    /// The one-call path query: finds the polygon nearest to each position (search box = options' extents), searches the
    /// cheapest corridor between them, and string-pulls it. If the end cannot be reached the path ends at the closest
    /// reachable point and <see cref="PathResult.Status"/> is Partial.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    public PathResult FindPath(Vector3D start, Vector3D end, PathOptions? options = null)
    {
        PathOptions o = options ?? s_defaultOptions;
        QueryStatus s = BeginHigh(start, end, o, out NearestPoly sn, out NearestPoly en);
        if (s == QueryStatus.InProgress)
        {
            UpdatePath(int.MaxValue);
        }
        return FinishHigh(s, start, end, o, sn, en);
    }

    /// <summary>
    /// Same as <see cref="FindPath(Vector3D, Vector3D, PathOptions?)"/>, run in slices of <paramref name="iterationsPerSlice"/>
    /// expansions with a yield between slices, so a host stays responsive. The result is identical to the synchronous one.
    /// Do not use the query object for anything else while the returned task runs.
    /// </summary>
    /// <exception cref="OperationCanceledException">The token was cancelled between slices.</exception>
    public async Task<PathResult> FindPathAsync(Vector3D start, Vector3D end, PathOptions? options = null, int iterationsPerSlice = 256, CancellationToken cancellationToken = default)
    {
        if (iterationsPerSlice < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(iterationsPerSlice), iterationsPerSlice, "A slice must be at least one iteration.");
        }
        PathOptions o = options ?? s_defaultOptions;
        QueryStatus s = BeginHigh(start, end, o, out NearestPoly sn, out NearestPoly en);
        while (s == QueryStatus.InProgress)
        {
            cancellationToken.ThrowIfCancellationRequested();
            s = UpdatePath(iterationsPerSlice);
            if (s == QueryStatus.InProgress)
            {
                await Task.Yield();
            }
        }
        return FinishHigh(s, start, end, o, sn, en);
    }

    private QueryStatus BeginHigh(Vector3D start, Vector3D end, PathOptions o, out NearestPoly sn, out NearestPoly en)
    {
        CheckPos(start, nameof(start));
        CheckPos(end, nameof(end));
        QueryFilter f = o.Filter ?? s_defaultFilter;
        sn = FindNearestPoly(start, o.SearchExtents, f);
        en = FindNearestPoly(end, o.SearchExtents, f);
        if (!sn.Poly.IsSet)
        {
            return QueryStatus.NoStartNearby;
        }
        if (!en.Poly.IsSet)
        {
            return QueryStatus.NoEndNearby;
        }
        return BeginPath(sn.Poly, en.Poly, sn.Point, en.Point, f);
    }

    private PathResult FinishHigh(QueryStatus s, Vector3D start, Vector3D end, PathOptions o, NearestPoly sn, NearestPoly en)
    {
        _ = start;
        _ = end;
        if (s != QueryStatus.InProgress && s != QueryStatus.Success && s != QueryStatus.Partial)
        {
            return new PathResult(s, [], [], 0, false);
        }
        int maxCorr = Math.Clamp(o.MaxCorridor, 1, 1 << 20);
        if (_corridorBuf.Length < maxCorr)
        {
            _corridorBuf = new PolyRef[maxCorr];
        }
        QueryStatus status = FinishPath(_corridorBuf.AsSpan(0, maxCorr), out int cc, out bool trunc, out double cost);
        Vector3D goal = status == QueryStatus.Success ? en.Point : default;
        var corridor = _corridorBuf.AsSpan(0, cc);
        if (status == QueryStatus.Partial)
        {
            // End at the closest point of the last corridor polygon to the requested end.
            ClosestPointOnPoly(corridor[^1], en.Point, out goal, out _);
        }
        int maxPts = Math.Clamp(o.MaxPoints, 2, 1 << 20);
        if (_pointBuf.Length < maxPts)
        {
            _pointBuf = new StraightPathPoint[maxPts];
        }
        QueryStatus ss = FindStraightPath(sn.Point, goal, corridor, _pointBuf.AsSpan(0, maxPts), out int pc, out bool ptrunc);
        if (ss != QueryStatus.Success)
        {
            return new PathResult(ss, [], [], 0, false);
        }
        return new PathResult(status, _pointBuf.AsSpan(0, pc).ToArray(), corridor.ToArray(), cost, trunc || ptrunc);
    }
}

/// <summary>
/// Tier-1 entry point for meshes: <c>mesh.FindPath(a, b)</c> style queries without holding a query object. For many queries
/// create one <see cref="NavMeshQuery"/> and reuse it.
/// </summary>
public static class TiledNavMeshQueryExtensions
{
    /// <summary>Finds the shortest walkable path between two world positions (see <see cref="NavMeshQuery.FindPath(Vector3D, Vector3D, PathOptions?)"/>).</summary>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    public static PathResult FindPath(this TiledNavMesh mesh, Vector3D start, Vector3D end, PathOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        return new NavMeshQuery(mesh).FindPath(start, end, options);
    }
}
