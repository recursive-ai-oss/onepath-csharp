using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Debug;

/// <summary>Plain-data picture of one path query for a host to draw.</summary>
public sealed class QueryDebugGeometry
{
    internal QueryDebugGeometry(IReadOnlyList<DebugPolygon> visited, IReadOnlyList<DebugPolygon> corridor, IReadOnlyList<Vector3D> straight)
    {
        Visited = visited;
        Corridor = corridor;
        StraightPath = straight;
    }

    /// <summary>Outlines of the polygons the search expanded, in expansion order (each once).</summary>
    public IReadOnlyList<DebugPolygon> Visited { get; }

    /// <summary>Outlines of the corridor polygons, start first.</summary>
    public IReadOnlyList<DebugPolygon> Corridor { get; }

    /// <summary>The straight path's points.</summary>
    public IReadOnlyList<Vector3D> StraightPath { get; }
}

/// <summary>Builds <see cref="QueryDebugGeometry"/>.</summary>
public static class QueryDebug
{
    /// <summary>
    /// Collects the visited polygons of the query's last search, the corridor and the straight path of a result.
    /// Allocates; meant for debug drawing, not for the hot path.
    /// </summary>
    public static QueryDebugGeometry Collect(NavMeshQuery query, PathResult result)
    {
        ArgumentNullException.ThrowIfNull(query);
        ArgumentNullException.ThrowIfNull(result);
        var visited = new List<DebugPolygon>();
        var seen = new HashSet<ulong>();
        foreach (PolyRef p in query.GetExpandedPolygons())
        {
            if (seen.Add(p.Value) && Outline(query.Mesh, p) is { } o)
            {
                visited.Add(o);
            }
        }
        var corridor = new List<DebugPolygon>();
        foreach (PolyRef p in result.Corridor)
        {
            if (Outline(query.Mesh, p) is { } o)
            {
                corridor.Add(o);
            }
        }
        var straight = new List<Vector3D>();
        foreach (StraightPathPoint pt in result.Points)
        {
            straight.Add(pt.Position);
        }
        return new QueryDebugGeometry(visited, corridor, straight);
    }

    internal static DebugPolygon? Outline(TiledNavMesh mesh, PolyRef p)
    {
        if (!mesh.TryGetPoly(p, out NavTile t, out int pi))
        {
            return null;
        }
        int n = t.PolyVertexCounts[pi], b = t.PolyFirstIndex(pi);
        var v = new Vector3D[n];
        for (int i = 0; i < n; i++)
        {
            int vi = 3 * t.PolyIndices[b + i];
            v[i] = new Vector3D(t.OriginX + t.Vertices[vi], t.OriginY + t.Vertices[vi + 1], t.OriginZ + t.Vertices[vi + 2]);
        }
        return new DebugPolygon(p, t.PolyAreas[pi], t.PolyFlags[pi], v);
    }
}
