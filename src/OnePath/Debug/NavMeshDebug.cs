using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.NavMesh;

namespace RecursiveAI.OnePath.Debug;

/// <summary>A polygon outline for drawing: world-space vertices in order.</summary>
public sealed class DebugPolygon
{
    internal DebugPolygon(PolyRef poly, byte area, ushort flags, Vector3D[] vertices)
    {
        Poly = poly;
        Area = area;
        Flags = flags;
        Vertices = vertices;
    }

    /// <summary>The polygon.</summary>
    public PolyRef Poly { get; }

    /// <summary>Its area id.</summary>
    public byte Area { get; }

    /// <summary>Its flags.</summary>
    public ushort Flags { get; }

    /// <summary>World-space outline.</summary>
    public IReadOnlyList<Vector3D> Vertices { get; }
}

/// <summary>A tile-boundary portal or off-mesh link as a world-space segment.</summary>
public sealed class DebugLink
{
    internal DebugLink(PolyRef from, PolyRef to, NavLinkKind kind, Vector3D a, Vector3D b)
    {
        From = from;
        To = to;
        Kind = kind;
        A = a;
        B = b;
    }

    /// <summary>Source polygon.</summary>
    public PolyRef From { get; }

    /// <summary>Target polygon.</summary>
    public PolyRef To { get; }

    /// <summary>Portal or off-mesh.</summary>
    public NavLinkKind Kind { get; }

    /// <summary>For a portal: the start of the shared part of the edge. For off-mesh: the departure point.</summary>
    public Vector3D A { get; }

    /// <summary>For a portal: the end of the shared part. For off-mesh: the arrival point.</summary>
    public Vector3D B { get; }
}

/// <summary>Plain-data geometry of a mesh for a host to draw.</summary>
public sealed class NavMeshDebugGeometry
{
    internal NavMeshDebugGeometry(List<DebugPolygon> polygons, List<DebugLink> links)
    {
        Polygons = polygons;
        Links = links;
    }

    /// <summary>All polygons in slot then polygon order.</summary>
    public IReadOnlyList<DebugPolygon> Polygons { get; }

    /// <summary>All stitched links in slot, polygon then link order.</summary>
    public IReadOnlyList<DebugLink> Links { get; }
}

/// <summary>Extracts drawable geometry from a mesh.</summary>
public static class NavMeshDebug
{
    /// <summary>Collects every polygon outline and stitched link of <paramref name="mesh"/> in world coordinates.</summary>
    public static NavMeshDebugGeometry Collect(TiledNavMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        var polys = new List<DebugPolygon>();
        var links = new List<DebugLink>();
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            NavTile? t = mesh.GetTileBySlot(s);
            if (t is null)
            {
                continue;
            }
            var tr = new TileRef(mesh.GetSlotSalt(s), s);
            ReadOnlySpan<float> v = t.Vertices;
            ReadOnlySpan<ushort> idx = t.PolyIndices;
            for (int p = 0; p < t.PolyCount; p++)
            {
                int n = t.PolyVertexCounts[p];
                int b = t.PolyFirstIndex(p);
                var outline = new Vector3D[n];
                for (int i = 0; i < n; i++)
                {
                    int vi = idx[b + i];
                    outline[i] = new Vector3D(t.OriginX + v[3 * vi], t.OriginY + v[(3 * vi) + 1], t.OriginZ + v[(3 * vi) + 2]);
                }
                PolyRef pr = tr.Poly(p);
                polys.Add(new DebugPolygon(pr, t.PolyAreas[p], t.PolyFlags[p], outline));
                foreach (NavLink l in mesh.GetLinks(pr))
                {
                    if (l.Kind == NavLinkKind.Portal)
                    {
                        int i0 = idx[b + l.Index], i1 = idx[b + ((l.Index + 1) % n)];
                        Vector3D e0 = new(t.OriginX + v[3 * i0], t.OriginY + v[(3 * i0) + 1], t.OriginZ + v[(3 * i0) + 2]);
                        Vector3D e1 = new(t.OriginX + v[3 * i1], t.OriginY + v[(3 * i1) + 1], t.OriginZ + v[(3 * i1) + 2]);
                        double f0 = l.RangeMin / 255.0, f1 = l.RangeMax / 255.0;
                        links.Add(new DebugLink(pr, l.Target, l.Kind, Lerp(e0, e1, f0), Lerp(e0, e1, f1)));
                    }
                    else
                    {
                        NavTile owner = mesh.GetTileBySlot(l.OwnerSlot)!;
                        OffMeshLink o = owner.OffMeshLinks[l.Index];
                        Vector3D a = new(owner.OriginX + o.StartX, owner.OriginY + o.StartY, owner.OriginZ + o.StartZ);
                        Vector3D e = new(owner.OriginX + o.EndX, owner.OriginY + o.EndY, owner.OriginZ + o.EndZ);
                        links.Add(l.Kind == NavLinkKind.OffMesh ? new DebugLink(pr, l.Target, l.Kind, a, e) : new DebugLink(pr, l.Target, l.Kind, e, a));
                    }
                }
            }
        }
        return new NavMeshDebugGeometry(polys, links);
    }

    private static Vector3D Lerp(Vector3D a, Vector3D b, double t) =>
        new(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t), a.Z + ((b.Z - a.Z) * t));
}
