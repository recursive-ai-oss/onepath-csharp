using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Deterministic random mutations of a tiled mesh (tiles added, removed and replaced; polygon flags and areas edited;
/// off-mesh links switched) and a digest of everything observable about the mesh. Shared by the snapshot tests, the fuzz
/// target and the hash-matrix scenarios. An operation resolves its targets from the mesh's current state and the
/// generator alone, so two meshes in the same state given the same generator stay in the same state.
/// </summary>
internal static class NavMeshMutations
{
    public const int Kinds = 6;

    /// <summary>Applies one random operation; returns its kind.</summary>
    public static int Apply(TiledNavMesh mesh, NavMeshWorlds.Rng r, ulong worldSeed)
    {
        NavMeshParams p = mesh.Params;
        int kind = r.Int(Kinds);
        switch (kind)
        {
            case 0:
            case 1:
            {
                // Kind 0 puts the world's own tile at a position, kind 1 a differently seeded one; either adds or replaces.
                int x = r.Int(3) - 1, z = r.Int(3) - 1, l = r.Int(3);
                ulong ts = kind == 0 ? worldSeed : worldSeed + 1 + (ulong)r.Int(3);
                NavTile t = NavMeshWorlds.MakeTile(p, ts, x, z, l, detail: r.Int(2) == 0, offMesh: r.Int(3));
                mesh.ReplaceTile(t);
                break;
            }
            case 2:
            {
                int s = PickSlot(mesh, r);
                if (s >= 0)
                {
                    mesh.RemoveTile(mesh.GetSlotRef(s));
                }
                break;
            }
            case 3:
            case 4:
            {
                int s = PickSlot(mesh, r);
                if (s >= 0)
                {
                    NavTile t = mesh.GetTileBySlot(s)!;
                    if (t.PolyCount > 0)
                    {
                        PolyRef pr = mesh.GetSlotRef(s).Poly(r.Int(t.PolyCount));
                        if (kind == 3)
                        {
                            mesh.SetPolyFlags(pr, (ushort)r.Int(8));
                        }
                        else
                        {
                            mesh.SetPolyArea(pr, (byte)r.Int(5));
                        }
                    }
                }
                break;
            }
            default:
            {
                int s = PickSlot(mesh, r);
                if (s >= 0)
                {
                    NavTile t = mesh.GetTileBySlot(s)!;
                    if (t.OffMeshLinks.Length > 0)
                    {
                        int li = r.Int(t.OffMeshLinks.Length);
                        TileRef tr = mesh.GetSlotRef(s);
                        mesh.SetOffMeshLinkEnabled(tr, li, !mesh.IsOffMeshLinkEnabled(tr, li));
                    }
                }
                break;
            }
        }
        return kind;
    }

    private static int PickSlot(TiledNavMesh mesh, NavMeshWorlds.Rng r)
    {
        if (mesh.TileCount == 0)
        {
            return -1;
        }
        int n = r.Int(mesh.TileCount);
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            if (mesh.GetTileBySlot(s) is not null && n-- == 0)
            {
                return s;
            }
        }
        return -1;
    }

    /// <summary>Feeds everything observable about the mesh to <paramref name="sink"/>: slots, generations, areas, flags, links and query answers.</summary>
    public static void Observe(TiledNavMesh m, ulong seed, Action<long> sink)
    {
        sink(m.TileCount);
        for (int s = 0; s < m.SlotCount; s++)
        {
            sink(m.GetSlotSalt(s));
            if (m.GetTileBySlot(s) is { } t)
            {
                sink(t.PolyCount);
                for (int i = 0; i < t.PolyCount; i++)
                {
                    sink((t.PolyAreas[i] << 16) | t.PolyFlags[i]);
                }
                for (int l = 0; l < t.OffMeshLinks.Length; l++)
                {
                    sink(m.IsOffMeshLinkEnabled(m.GetSlotRef(s), l) ? 1 : 0);
                }
            }
        }
        NavMeshWorlds.DumpLinks(m, sink);
        var q = new NavMeshQuery(m);
        var r = new NavMeshWorlds.Rng(seed);
        var visited = new PolyRef[64];
        Vector3D Spot() => new(-12 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0), (r.Int(1 << 20) / (double)(1 << 20) * 5.0) - 0.25, -12 + (r.Int(1 << 20) / (double)(1 << 20) * 24.0));
        for (int i = 0; i < 12; i++)
        {
            Vector3D a = Spot();
            Vector3D b = Spot();
            NearestPoly n = q.FindNearestPoly(a, new Vector3D(2, 5, 2));
            sink((long)n.Poly.Value);
            sink(BitConverter.DoubleToInt64Bits(n.DistanceSquared));
            var o = new PathOptions { SearchExtents = new Vector3D(2, 5, 2) };
            if (i % 3 == 1)
            {
                o.Filter = new QueryFilter { ExcludeFlags = 2 }.SetAreaCost(1, 3).SetAreaCost(9, 2);
            }
            PathResult pr = q.FindPath(a, b, o);
            sink((int)pr.Status);
            sink(BitConverter.DoubleToInt64Bits(pr.Cost));
            foreach (PolyRef c in pr.Corridor)
            {
                sink((long)c.Value);
            }
            foreach (StraightPathPoint pt in pr.Points)
            {
                sink(BitConverter.DoubleToInt64Bits(pt.Position.X));
                sink(BitConverter.DoubleToInt64Bits(pt.Position.Z));
                sink((long)pt.Poly.Value);
            }
            if (n.Poly.IsSet)
            {
                RaycastResult rr = q.Raycast(n.Poly, n.Point, b, null, visited, out int vc);
                sink((int)rr.Status);
                sink(BitConverter.DoubleToInt64Bits(rr.T));
                sink(vc);
                for (int k = 0; k < vc; k++)
                {
                    sink((long)visited[k].Value);
                }
            }
        }
    }

    public static ulong Digest(TiledNavMesh m, ulong seed)
    {
        ulong h = 14695981039346656037UL;
        Observe(m, seed, v =>
        {
            h = (h ^ (ulong)v) * 1099511628211UL;
        });
        return h;
    }
}
