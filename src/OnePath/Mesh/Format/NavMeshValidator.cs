using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// Mesh-level consistency checks that go beyond what <see cref="NavTile.Create"/> guarantees per tile: stitched
/// links point at live polygons, portal links come in opposite-side pairs, and bidirectional off-mesh links have
/// their reverse link. Used by tests and by hosts that want to assert a streamed mesh is sound.
/// </summary>
public static class NavMeshValidator
{
    /// <summary>Returns one message per problem found; an empty list means the mesh is consistent.</summary>
    public static IReadOnlyList<string> Validate(TiledNavMesh mesh)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        var problems = new List<string>();
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            NavTile? t = mesh.GetTileBySlot(s);
            if (t is null)
            {
                continue;
            }
            var tr = new TileRef(mesh.GetSlotSalt(s), s);
            for (int p = 0; p < t.PolyCount; p++)
            {
                PolyRef from = tr.Poly(p);
                foreach (NavLink l in mesh.GetLinks(from))
                {
                    if (!mesh.TryGetPoly(l.Target, out NavTile? tt, out int tp))
                    {
                        problems.Add($"slot {s} polygon {p}: link target {l.Target} is not a live polygon.");
                        continue;
                    }
                    bool back = false;
                    foreach (NavLink m in mesh.GetLinks(l.Target))
                    {
                        if (m.Target != from)
                        {
                            continue;
                        }
                        if (l.Kind == NavLinkKind.Portal && m.Kind == NavLinkKind.Portal && m.Side == NavMeshFormat.OppositeSide(l.Side))
                        {
                            back = true;
                        }
                        else if (l.Kind == NavLinkKind.OffMesh && m.Kind == NavLinkKind.OffMeshReverse && m.Index == l.Index && m.OwnerSlot == l.OwnerSlot)
                        {
                            back = true;
                        }
                        else if (l.Kind == NavLinkKind.OffMeshReverse && m.Kind == NavLinkKind.OffMesh && m.Index == l.Index && m.OwnerSlot == l.OwnerSlot)
                        {
                            back = true;
                        }
                    }
                    if (!back && !(l.Kind == NavLinkKind.OffMesh && !mesh.GetTileBySlot(l.OwnerSlot)!.OffMeshLinks[l.Index].Bidirectional))
                    {
                        problems.Add($"slot {s} polygon {p}: {l.Kind} link to slot {l.Target.TileSlot} polygon {tp} has no matching link back.");
                    }
                    if (l.Kind == NavLinkKind.Portal)
                    {
                        if (tt.TileX != t.TileX + NavMeshFormat.SideDx(l.Side) || tt.TileZ != t.TileZ + NavMeshFormat.SideDz(l.Side))
                        {
                            problems.Add($"slot {s} polygon {p}: portal on side {l.Side} leads to a tile that is not adjacent on that side.");
                        }
                        if (l.RangeMin > l.RangeMax)
                        {
                            problems.Add($"slot {s} polygon {p}: portal range {l.RangeMin}..{l.RangeMax} is reversed.");
                        }
                    }
                }
            }
        }
        return problems;
    }
}
