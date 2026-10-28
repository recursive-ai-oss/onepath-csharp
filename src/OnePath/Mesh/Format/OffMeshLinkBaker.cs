using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// Bakes authored <see cref="OffMeshLinkDefinition"/>s into the off-mesh slots of freshly built tiles (see <c>docs/navmesh-format.md</c>, "Authoring").
/// A link goes to the tile that holds its start: the tile of the start's column, and among several layers the one with the polygon nearest
/// to the start (lowest layer on a tie). Columns the build produced no tile for take no link.
/// </summary>
internal static class OffMeshLinkBaker
{
    /// <summary>Checks the definitions against the mesh parameters; throws <see cref="ArgumentOutOfRangeException"/> naming the link and the value.</summary>
    public static void Validate(NavMeshParams p, IReadOnlyList<OffMeshLinkDefinition>? links)
    {
        if (links is null)
        {
            return;
        }
        for (int i = 0; i < links.Count; i++)
        {
            OffMeshLinkDefinition l = links[i];
            Span<double> c = [l.Start.X, l.Start.Y, l.Start.Z, l.End.X, l.End.Y, l.End.Z];
            for (int k = 0; k < c.Length; k++)
            {
                if (!double.IsFinite(c[k]) || !(Math.Abs(c[k]) < NavMeshParams.MaxWorld))
                {
                    throw new ArgumentOutOfRangeException(nameof(links), c[k], $"OffMeshLinks[{i}]: coordinate {c[k]:R} is not finite or is outside ±{NavMeshParams.MaxWorld:R} (2^21).");
                }
            }
            if (!(l.Radius > 0) || !double.IsFinite(l.Radius) || l.Radius > p.MaxLocalExtent)
            {
                throw new ArgumentOutOfRangeException(nameof(links), l.Radius, $"OffMeshLinks[{i}]: the radius must be positive, finite and at most {p.MaxLocalExtent:R}.");
            }
            if (l.Area > NavMeshFormat.MaxArea)
            {
                throw new ArgumentOutOfRangeException(nameof(links), l.Area, $"OffMeshLinks[{i}]: the area must be 0 to {NavMeshFormat.MaxArea}.");
            }
            p.WorldToTile(l.Start.X, l.Start.Z, out int tx, out int tz);
            double ex = l.End.X - p.TileOriginX(tx), ez = l.End.Z - p.TileOriginZ(tz);
            if (Math.Abs(ex) > p.MaxLocalExtent || Math.Abs(ez) > p.MaxLocalExtent)
            {
                throw new ArgumentOutOfRangeException(nameof(links), l.End, $"OffMeshLinks[{i}]: the end lies {Math.Max(Math.Abs(ex), Math.Abs(ez)):R} from its start's tile origin; links may reach at most {p.MaxLocalExtent:R} (16384 cells).");
            }
        }
    }

    /// <summary>Appends to the tiles <c>tiles[from..]</c> the links whose start lies in their column.</summary>
    public static void Attach(NavMeshParams p, IReadOnlyList<OffMeshLinkDefinition>? links, List<NavTileData> tiles, int from)
    {
        if (links is null || links.Count == 0 || from >= tiles.Count)
        {
            return;
        }
        var added = new Dictionary<NavTileData, List<OffMeshLink>>(ReferenceEqualityComparer.Instance);
        for (int i = 0; i < links.Count; i++)
        {
            OffMeshLinkDefinition l = links[i];
            p.WorldToTile(l.Start.X, l.Start.Z, out int tx, out int tz);
            NavTileData? best = null;
            double bestScore = double.PositiveInfinity;
            for (int t = from; t < tiles.Count; t++)
            {
                NavTileData d = tiles[t];
                if (d.TileX != tx || d.TileZ != tz)
                {
                    continue;
                }
                double score = Nearest(d, l.Start.X - p.TileOriginX(tx), l.Start.Y - d.OriginY, l.Start.Z - p.TileOriginZ(tz), l.Radius, p.WalkableClimb);
                if (best is null || score < bestScore)
                {
                    best = d;
                    bestScore = score;
                }
            }
            if (best is null)
            {
                continue;
            }
            double ox = p.TileOriginX(tx), oz = p.TileOriginZ(tz);
            double sy = l.Start.Y - best.OriginY, ey = l.End.Y - best.OriginY;
            if (Math.Abs(sy) > p.MaxLocalExtent || Math.Abs(ey) > p.MaxLocalExtent)
            {
                throw new InvalidOperationException($"OffMeshLinks[{i}]: its height is more than {p.MaxLocalExtent:R} (16384 cells) from the lowest point of tile ({tx}, {tz}).");
            }
            if (!added.TryGetValue(best, out List<OffMeshLink>? list))
            {
                added[best] = list = new List<OffMeshLink>();
            }
            list.Add(new OffMeshLink
            {
                StartX = (float)(l.Start.X - ox),
                StartY = (float)sy,
                StartZ = (float)(l.Start.Z - oz),
                EndX = (float)(l.End.X - ox),
                EndY = (float)ey,
                EndZ = (float)(l.End.Z - oz),
                Radius = (float)l.Radius,
                Area = l.Area,
                PolyFlags = l.Flags,
                Bidirectional = l.Bidirectional,
                UserId = l.UserId,
            });
        }
        foreach (KeyValuePair<NavTileData, List<OffMeshLink>> kv in added)
        {
            OffMeshLink[] old = kv.Key.OffMeshLinks ?? Array.Empty<OffMeshLink>();
            var all = new OffMeshLink[old.Length + kv.Value.Count];
            old.CopyTo(all, 0);
            kv.Value.CopyTo(all, old.Length);
            kv.Key.OffMeshLinks = all;
        }
    }

    // Squared distance score of the tile's nearest polygon to a tile-local point (horizontal distance beyond the polygon plus the height
    // outside its vertex range); infinity if none is within radius and climb. The same rule stitching uses to attach the start.
    private static double Nearest(NavTileData d, double px, double py, double pz, double radius, double climb)
    {
        double best = double.PositiveInfinity;
        byte[] counts = d.PolyVertexCounts ?? Array.Empty<byte>();
        ushort[] idx = d.PolyIndices ?? Array.Empty<ushort>();
        float[] v = d.Vertices ?? Array.Empty<float>();
        double r2 = radius * radius;
        int b = 0;
        for (int poly = 0; poly < counts.Length; poly++)
        {
            int n = counts[poly];
            bool inside = true;
            double d2 = double.PositiveInfinity, ymin = double.PositiveInfinity, ymax = double.NegativeInfinity;
            for (int e = 0; e < n; e++)
            {
                int i0 = idx[b + e], i1 = idx[b + ((e + 1) % n)];
                double x0 = v[3 * i0], z0 = v[(3 * i0) + 2], x1 = v[3 * i1], z1 = v[(3 * i1) + 2], y0 = v[(3 * i0) + 1];
                ymin = Math.Min(ymin, y0);
                ymax = Math.Max(ymax, y0);
                double ex = x1 - x0, ez = z1 - z0;
                if ((ex * (pz - z0)) - (ez * (px - x0)) < 0)
                {
                    inside = false;
                }
                double len2 = (ex * ex) + (ez * ez);
                double s = len2 > 0 ? Math.Clamp((((px - x0) * ex) + ((pz - z0) * ez)) / len2, 0.0, 1.0) : 0.0;
                double dx = px - (x0 + (ex * s)), dz = pz - (z0 + (ez * s));
                d2 = Math.Min(d2, (dx * dx) + (dz * dz));
            }
            b += n;
            if (inside)
            {
                d2 = 0;
            }
            double dy = py < ymin ? ymin - py : py > ymax ? py - ymax : 0.0;
            if (d2 <= r2 && dy <= climb)
            {
                best = Math.Min(best, d2 + (dy * dy));
            }
        }
        return best;
    }
}
