using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.NavMesh;

/// <summary>
/// Builds a <see cref="NavTileData"/> from convex polygons given over world-space double vertices: the shape a
/// polygon builder hands over (vertices, polygon vertex lists, areas and flags). It converts to tile-local
/// float32, welds nothing (vertices are shared by index), derives in-tile adjacency from shared edges and marks
/// tile-boundary edges as portals. The caller then adds detail or off-mesh links if it has any and calls
/// <see cref="NavTile.Create"/>, which validates the result.
/// </summary>
public static class NavTileAssembler
{
    /// <summary>
    /// Assembles tile data.
    /// </summary>
    /// <param name="p">Mesh parameters.</param>
    /// <param name="tileX">Tile column.</param>
    /// <param name="tileZ">Tile row.</param>
    /// <param name="layer">Layer.</param>
    /// <param name="originY">World Y that becomes tile-local Y zero (for example the lowest vertex, rounded down).</param>
    /// <param name="worldVertices">World positions, three doubles (x, y, z) per vertex.</param>
    /// <param name="polyVertexCounts">Vertex count of each polygon.</param>
    /// <param name="polyIndices">Vertex indices of all polygons, concatenated. A polygon wound clockwise in X (right) / Z (up) is reversed.</param>
    /// <param name="areas">Area of each polygon (null: all 0).</param>
    /// <param name="flags">Flags of each polygon (null: all 0).</param>
    /// <returns>Tile data. Coordinates within <c>CellSize/1024</c> of a tile boundary are snapped onto it so that tiles meet exactly.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A world coordinate is non-finite or beyond ±2^21; the message names the value.</exception>
    /// <exception cref="NavMeshFormatException">The input lists are inconsistent.</exception>
    public static NavTileData FromPolygons(
        NavMeshParams p,
        int tileX,
        int tileZ,
        int layer,
        double originY,
        ReadOnlySpan<double> worldVertices,
        ReadOnlySpan<int> polyVertexCounts,
        ReadOnlySpan<int> polyIndices,
        ReadOnlySpan<byte> areas = default,
        ReadOnlySpan<ushort> flags = default)
    {
        ArgumentNullException.ThrowIfNull(p);
        p.Validate();
        NavMeshParams.CheckWorld(originY, nameof(originY));
        if (worldVertices.Length % 3 != 0)
        {
            throw new NavMeshFormatException("worldVertices must hold 3 doubles per vertex.");
        }
        int np = polyVertexCounts.Length;
        if ((areas.Length != 0 && areas.Length != np) || (flags.Length != 0 && flags.Length != np))
        {
            throw new NavMeshFormatException("areas and flags must be empty or hold one entry per polygon.");
        }
        long total = 0;
        foreach (int c in polyVertexCounts)
        {
            if (c < 3)
            {
                throw new NavMeshFormatException($"A polygon has {c} vertices; at least 3 are needed.");
            }
            total += c;
        }
        if (total != polyIndices.Length)
        {
            throw new NavMeshFormatException($"polyIndices holds {polyIndices.Length} entries but the polygon vertex counts sum to {total}.");
        }

        double ox = p.TileOriginX(tileX);
        double oz = p.TileOriginZ(tileZ);
        int nv = worldVertices.Length / 3;
        var verts = new float[nv * 3];
        double snap = p.CellSize * (1.0 / 1024.0);
        float ts = (float)p.TileSize;
        for (int i = 0; i < nv; i++)
        {
            double x = worldVertices[3 * i], y = worldVertices[(3 * i) + 1], z = worldVertices[(3 * i) + 2];
            NavMeshParams.CheckWorld(x, "worldVertices.x");
            NavMeshParams.CheckWorld(y, "worldVertices.y");
            NavMeshParams.CheckWorld(z, "worldVertices.z");
            verts[3 * i] = Snap(x - ox, snap, ts);
            verts[(3 * i) + 1] = (float)(y - originY);
            verts[(3 * i) + 2] = Snap(z - oz, snap, ts);
        }

        var idx = new ushort[polyIndices.Length];
        for (int i = 0; i < idx.Length; i++)
        {
            if ((uint)polyIndices[i] >= (uint)nv || nv > ushort.MaxValue + 1)
            {
                throw new NavMeshFormatException($"polyIndices[{i}] = {polyIndices[i]} is not a vertex (there are {nv}, at most 65536 are supported).");
            }
            idx[i] = (ushort)polyIndices[i];
        }
        var counts = new byte[np];
        int pos = 0;
        for (int pi = 0; pi < np; pi++)
        {
            int n = polyVertexCounts[pi];
            if (n > 255)
            {
                throw new NavMeshFormatException($"Polygon {pi} has {n} vertices.");
            }
            counts[pi] = (byte)n;
            // Signed area (shoelace) in X/Z; reverse clockwise polygons. Exact validity is checked by NavTile.Create.
            double a2 = 0;
            for (int e = 0; e < n; e++)
            {
                int i0 = idx[pos + e], i1 = idx[pos + ((e + 1) % n)];
                a2 += ((double)verts[3 * i0] * verts[(3 * i1) + 2]) - ((double)verts[3 * i1] * verts[(3 * i0) + 2]);
            }
            if (a2 < 0)
            {
                Array.Reverse(idx, pos, n);
            }
            pos += n;
        }

        // In-tile adjacency: directed edges (a, b) pair with (b, a). Sorting by (min, max, polygon, edge) makes the
        // pairing independent of any hash order; an edge shared by more than two polygons stays a wall.
        var edges = new List<(int Lo, int Hi, int Poly, int Edge, bool Fwd)>();
        pos = 0;
        for (int pi = 0; pi < np; pi++)
        {
            int n = counts[pi];
            for (int e = 0; e < n; e++)
            {
                int a = idx[pos + e], b = idx[pos + ((e + 1) % n)];
                edges.Add((Math.Min(a, b), Math.Max(a, b), pi, e, a < b));
            }
            pos += n;
        }
        edges.Sort(static (x, y) =>
        {
            int c = x.Lo.CompareTo(y.Lo);
            if (c != 0)
            {
                return c;
            }
            c = x.Hi.CompareTo(y.Hi);
            if (c != 0)
            {
                return c;
            }
            c = x.Poly.CompareTo(y.Poly);
            return c != 0 ? c : x.Edge.CompareTo(y.Edge);
        });
        var start = new int[np + 1];
        for (int pi = 0; pi < np; pi++)
        {
            start[pi + 1] = start[pi] + counts[pi];
        }
        var nbr = new int[idx.Length];
        for (int i = 0; i < edges.Count;)
        {
            int j = i + 1;
            while (j < edges.Count && edges[j].Lo == edges[i].Lo && edges[j].Hi == edges[i].Hi)
            {
                j++;
            }
            if (j - i == 2 && edges[i].Fwd != edges[i + 1].Fwd && edges[i].Poly != edges[i + 1].Poly)
            {
                var e0 = edges[i];
                var e1 = edges[i + 1];
                nbr[start[e0.Poly] + e0.Edge] = e1.Poly + 1;
                nbr[start[e1.Poly] + e1.Edge] = e0.Poly + 1;
            }
            i = j;
        }

        var sides = new byte[idx.Length];
        Array.Fill(sides, NavMeshFormat.NoSide);
        for (int pi = 0; pi < np; pi++)
        {
            int n = counts[pi];
            for (int e = 0; e < n; e++)
            {
                int g = start[pi] + e;
                if (nbr[g] != 0)
                {
                    continue;
                }
                int a = idx[g], b = idx[start[pi] + ((e + 1) % n)];
                for (int s = 0; s < 4; s++)
                {
                    int axis = (s & 1) == 0 ? 0 : 2;
                    float line = s < 2 ? ts : 0f;
                    if (verts[(3 * a) + axis] == line && verts[(3 * b) + axis] == line)
                    {
                        sides[g] = (byte)s;
                        break;
                    }
                }
            }
        }

        return new NavTileData
        {
            TileX = tileX,
            TileZ = tileZ,
            Layer = layer,
            OriginY = originY,
            Vertices = verts,
            PolyVertexCounts = counts,
            PolyIndices = idx,
            EdgeNeighbors = nbr,
            EdgePortalSides = sides,
            PolyAreas = areas.Length == 0 ? new byte[np] : areas.ToArray(),
            PolyFlags = flags.Length == 0 ? new ushort[np] : flags.ToArray(),
        };
    }

    private static float Snap(double local, double tol, float tileSize)
    {
        if (Math.Abs(local) <= tol)
        {
            return 0f;
        }
        if (Math.Abs(local - tileSize) <= tol)
        {
            return tileSize;
        }
        return (float)local;
    }
}
