using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    /// <summary>
    /// Moves a point toward an end position along the walkable surface, sliding along a wall it runs into. The straight move
    /// is traced as by <see cref="Raycast{TFilter}"/>; if it reaches the end the result is the end position on the surface of the
    /// polygon it ends in. If a wall stops it, the result is the point of that wall edge closest to the end position (in the
    /// X/Z plane, clamped to the edge, height interpolated along it) and the polygon owning the edge. The result is therefore
    /// always on the mesh; call again from the result for further sliding.
    /// </summary>
    /// <param name="startPoly">The polygon containing <paramref name="start"/>.</param>
    /// <param name="start">Where the move begins (within 1e-6 of the polygon outline).</param>
    /// <param name="end">Where the mover wants to go.</param>
    /// <param name="filter">Which neighbours may be entered.</param>
    /// <param name="visited">Receives the polygons traced, start first (may be empty).</param>
    /// <param name="visitedCount">Number written to <paramref name="visited"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public MoveResult MoveAlongSurface<TFilter>(PolyRef startPoly, Vector3D start, Vector3D end, in TFilter filter, Span<PolyRef> visited, out int visitedCount)
        where TFilter : struct, IQueryFilter
    {
        RaycastResult r = RaycastCore(startPoly, start, end, in filter, visited, out visitedCount, out PolyRef last);
        if (r.Status != RaycastStatus.Ok)
        {
            return new MoveResult(r.Status, start, r.Status == RaycastStatus.StartOutsidePolygon ? startPoly : PolyRef.Invalid, false, false);
        }
        if (!r.Hit)
        {
            _mesh.TryGetPoly(last, out NavTile lt, out int lp);
            double y = lt.OriginY + PolyGeometry.HeightAt(lt, lp, end.X - lt.OriginX, end.Z - lt.OriginZ);
            return new MoveResult(RaycastStatus.Ok, new Vector3D(end.X, y, end.Z), last, false, r.Truncated);
        }
        _mesh.TryGetPoly(r.HitPoly, out NavTile t, out int p);
        int n = t.PolyVertexCounts[p];
        int b = t.PolyStart[p];
        int a = 3 * t.PolyIndices[b + r.HitEdge], c = 3 * t.PolyIndices[b + ((r.HitEdge + 1) % n)];
        double ax = t.OriginX + (double)t.Vertices[a], az = t.OriginZ + (double)t.Vertices[a + 2];
        double ex = (t.OriginX + (double)t.Vertices[c]) - ax, ez = (t.OriginZ + (double)t.Vertices[c + 2]) - az;
        double len2 = (ex * ex) + (ez * ez);
        double u = len2 > 0 ? Math.Clamp((((end.X - ax) * ex) + ((end.Z - az) * ez)) / len2, 0.0, 1.0) : 0.0;
        double wy = t.OriginY + t.Vertices[a + 1] + ((t.Vertices[c + 1] - (double)t.Vertices[a + 1]) * u);
        return new MoveResult(RaycastStatus.Ok, new Vector3D(ax + (ex * u), wy, az + (ez * u)), r.HitPoly, true, r.Truncated);
    }

    /// <summary>Moves along the surface with the plain filter (null accepts everything).</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public MoveResult MoveAlongSurface(PolyRef startPoly, Vector3D start, Vector3D end, QueryFilter? filter, Span<PolyRef> visited, out int visitedCount)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return MoveAlongSurface(startPoly, start, end, in f, visited, out visitedCount);
    }
}
