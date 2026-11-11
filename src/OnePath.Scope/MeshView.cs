using System;
using System.Collections.Generic;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Scope;

/// <summary>The drawable, inspectable form of a navigation mesh. Built once per source and shared by its worlds; never changes.</summary>
internal sealed class MeshView
{
    private readonly Dictionary<ulong, int> _byEntity = [];

    public MeshView(TiledNavMesh mesh)
    {
        NavMeshDebugGeometry g = NavMeshDebug.Collect(mesh);
        Polygons = g.Polygons;
        Links = g.Links;
        TileCount = mesh.TileCount;
        bool planar = Polygons.Count > 0;
        double y0 = planar ? Polygons[0].Vertices[0].Y : 0;
        foreach (DebugPolygon p in Polygons)
        {
            foreach (Vector3D v in p.Vertices)
            {
                if (v.Y != y0)
                {
                    planar = false;
                }
            }
        }

        Planar = planar;
        FloorY = y0;
        var verts = new List<Vector3>();
        var idx = new List<int>();
        var edges = new List<Vector3>();
        for (int i = 0; i < Polygons.Count; i++)
        {
            DebugPolygon p = Polygons[i];
            _byEntity[ScopeInfo.Poly(p.Poly.Value).Value] = i;
            int first = verts.Count;
            for (int k = 0; k < p.Vertices.Count; k++)
            {
                Vector3 a = ScopeInfo.P(p.Vertices[k], planar);
                verts.Add(a);
                edges.Add(a);
                edges.Add(ScopeInfo.P(p.Vertices[(k + 1) % p.Vertices.Count], planar));
                if (k >= 2)
                {
                    idx.Add(first);
                    idx.Add(first + k - 1);
                    idx.Add(first + k);
                }
            }
        }

        Vertices = [.. verts];
        Indices = [.. idx];
        Edges = [.. edges];
    }

    public IReadOnlyList<DebugPolygon> Polygons { get; }

    public IReadOnlyList<DebugLink> Links { get; }

    public int TileCount { get; }

    public bool Planar { get; }

    public double FloorY { get; }

    public Vector3[] Vertices { get; }

    public int[] Indices { get; }

    /// <summary>Polygon outline segments, two points each.</summary>
    public Vector3[] Edges { get; }

    public bool TryGet(EntityId entity, out DebugPolygon polygon)
    {
        if (_byEntity.TryGetValue(entity.Value, out int i))
        {
            polygon = Polygons[i];
            return true;
        }

        polygon = null!;
        return false;
    }

    /// <summary>The mesh's polygons as entities, at most <paramref name="limit"/>.</summary>
    public void CollectEntities(List<EntityInfo> into, int limit)
    {
        int n = Math.Min(Polygons.Count, limit);
        for (int i = 0; i < n; i++)
        {
            DebugPolygon p = Polygons[i];
            into.Add(new EntityInfo(ScopeInfo.Poly(p.Poly.Value), ScopeInfo.MeshRoot, "polygon", $"polygon {p.Poly.TileSlot}.{p.Poly.PolyIndex}"));
        }
    }
}
