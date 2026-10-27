using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// Convex polygons over the cell grid: the hand-off from the polygon stage to the detail and tile stages. Vertices are
/// integer cell corners (x, floor row, z) of <see cref="Frame"/>; every polygon is convex and counter-clockwise (x to the
/// right, z up) and lies inside the contour it came from.
/// </summary>
public sealed class PolyMesh
{
    internal PolyMesh(HeightfieldFrame frame, int maxVertsPerPoly, int[] vertices, int[] counts, int[] indices, byte[] areas, int[] regions, int degenerateEars)
    {
        Frame = frame;
        MaxVertsPerPoly = maxVertsPerPoly;
        VertexArray = vertices;
        CountArray = counts;
        IndexArray = indices;
        AreaArray = areas;
        RegionArray = regions;
        DegenerateEars = degenerateEars;
    }

    /// <summary>The grid the coordinates refer to.</summary>
    public HeightfieldFrame Frame { get; }

    /// <summary>The largest vertex count a polygon may have.</summary>
    public int MaxVertsPerPoly { get; }

    /// <summary>Number of vertices.</summary>
    public int VertexCount => VertexArray.Length / 3;

    /// <summary>Number of polygons.</summary>
    public int PolygonCount => CountArray.Length;

    /// <summary>Vertices, three ints (x, floor row, z) each.</summary>
    public ReadOnlySpan<int> Vertices => VertexArray;

    /// <summary>Vertex count of each polygon.</summary>
    public ReadOnlySpan<int> PolygonVertexCounts => CountArray;

    /// <summary>The vertex indices of all polygons, concatenated.</summary>
    public ReadOnlySpan<int> PolygonIndices => IndexArray;

    /// <summary>Area id of each polygon (the region's area).</summary>
    public ReadOnlySpan<byte> Areas => AreaArray;

    /// <summary>The region each polygon came from.</summary>
    public ReadOnlySpan<int> Regions => RegionArray;

    /// <summary>Ear-clipping steps that had to settle for a collinear or forced ear (0 on well-formed contours).</summary>
    public int DegenerateEars { get; }

    internal int[] VertexArray { get; }

    internal int[] CountArray { get; }

    internal int[] IndexArray { get; }

    internal byte[] AreaArray { get; }

    internal int[] RegionArray { get; }
}

/// <summary>
/// Cuts contours into convex polygons: ear clipping (O'Rourke, Computational Geometry in C, ch. 1: a diagonal is valid when
/// it lies in the cone at both ends and crosses no edge) with exact integer orientation, then greedy merging of triangles across
/// their longest shared edge while the result stays convex and within the vertex limit (Hertel and Mehlhorn 1983). The rules and
/// tie-breaks are in docs/build3d-polys.md.
/// </summary>
public static class Polygonizer
{
    /// <summary>
    /// Builds the polygons of every contour except those of <paramref name="skipRegion"/>.
    /// </summary>
    /// <param name="contours">The contours.</param>
    /// <param name="maxVertsPerPoly">Largest polygon size, 3 to 16.</param>
    /// <param name="skipRegion">Region id whose contours are ignored (the tile border), or a negative number for none.</param>
    /// <param name="map">Maps grid coordinates to the tile-local float32 positions the tile will store: polygons are kept strictly convex and triangles positively oriented in those positions, so the format's exact convexity check holds after rounding.</param>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The vertex limit is outside 3 to 16.</exception>
    public static PolyMesh Build(ContourSet contours, int maxVertsPerPoly, int skipRegion, TileCoordinates map)
    {
        ArgumentNullException.ThrowIfNull(map);
        ArgumentNullException.ThrowIfNull(contours);
        if (maxVertsPerPoly < 3 || maxVertsPerPoly > 16)
        {
            throw new ArgumentOutOfRangeException(nameof(maxVertsPerPoly), maxVertsPerPoly, "The polygon vertex limit must be 3 to 16.");
        }
        var weld = new Dictionary<(int, int), List<int>>();
        var verts = new List<int>();
        var counts = new List<int>();
        var indices = new List<int>();
        var areas = new List<byte>();
        var regions = new List<int>();
        int degenerate = 0;
        var tris = new List<int>();
        foreach (Contour c in contours.Contours)
        {
            if (c.Region == skipRegion)
            {
                continue;
            }
            ContourVertex[] v = c.Vertices;
            if (v.Length < 3)
            {
                continue;
            }
            tris.Clear();
            degenerate += EarClip(v, tris);
            // Weld by position: bridge duplicates collapse, and neighbouring regions share vertices on their common boundary. Corners at the same
            // (x, z) whose floor rows differ by at most one are one vertex (a contour that touches itself at a corner would otherwise give a polygon
            // two vertices at one place); the first vertex created there wins.
            var global = new int[v.Length];
            for (int i = 0; i < v.Length; i++)
            {
                var key = (v[i].X, v[i].Z);
                if (!weld.TryGetValue(key, out List<int>? ids))
                {
                    ids = new List<int>(1);
                    weld.Add(key, ids);
                }
                int id = -1;
                foreach (int cand in ids)
                {
                    if (Math.Abs(verts[(3 * cand) + 1] - v[i].Y) <= 1)
                    {
                        id = cand;
                        break;
                    }
                }
                if (id < 0)
                {
                    id = verts.Count / 3;
                    ids.Add(id);
                    verts.Add(v[i].X);
                    verts.Add(v[i].Y);
                    verts.Add(v[i].Z);
                }
                global[i] = id;
            }
            // Directed wall edges of this contour (no region on the other side), in welded vertex ids.
            var walls = new HashSet<(int, int)>();
            for (int i = 0; i < v.Length; i++)
            {
                int j = (i + 1) % v.Length;
                if (v[i].Neighbor == 0 && global[i] != global[j])
                {
                    walls.Add((global[i], global[j]));
                }
            }
            var polys = new List<int[]>();
            for (int t = 0; t < tris.Count; t += 3)
            {
                int a = global[tris[t]], b = global[tris[t + 1]], d = global[tris[t + 2]];
                if (a == b || b == d || a == d || FloatOrient(verts, map, a, b, d) <= 0)
                {
                    continue;
                }
                polys.Add(new[] { a, b, d });
            }
            MergeConvex(polys, verts, maxVertsPerPoly, map, walls);
            foreach (int[] p in polys)
            {
                counts.Add(p.Length);
                indices.AddRange(p);
                areas.Add(c.Area);
                regions.Add(c.Region);
            }
        }
        return new PolyMesh(contours.Frame, maxVertsPerPoly, verts.ToArray(), counts.ToArray(), indices.ToArray(), areas.ToArray(), regions.ToArray(), degenerate);
    }

    // Twice the signed area of (a, b, c) with x right, z up; exact in long for coordinates below 2^20.
    private static long Cross(ReadOnlySpan<ContourVertex> v, int a, int b, int c) =>
        ((long)(v[b].X - v[a].X) * (v[c].Z - v[a].Z)) - ((long)(v[b].Z - v[a].Z) * (v[c].X - v[a].X));

    private static bool Left(ReadOnlySpan<ContourVertex> v, int a, int b, int c) => Cross(v, a, b, c) > 0;

    private static bool LeftOn(ReadOnlySpan<ContourVertex> v, int a, int b, int c) => Cross(v, a, b, c) >= 0;

    private static bool Collinear(ReadOnlySpan<ContourVertex> v, int a, int b, int c) => Cross(v, a, b, c) == 0;

    private static bool SamePos(ReadOnlySpan<ContourVertex> v, int a, int b) => v[a].X == v[b].X && v[a].Z == v[b].Z;

    private static bool Between(ReadOnlySpan<ContourVertex> v, int a, int b, int c)
    {
        if (!Collinear(v, a, b, c))
        {
            return false;
        }
        return v[a].X != v[b].X
            ? (v[a].X <= v[c].X && v[c].X <= v[b].X) || (v[a].X >= v[c].X && v[c].X >= v[b].X)
            : (v[a].Z <= v[c].Z && v[c].Z <= v[b].Z) || (v[a].Z >= v[c].Z && v[c].Z >= v[b].Z);
    }

    // Segments ab and cd share a point (proper crossing, or an endpoint of one on the other).
    private static bool Meets(ReadOnlySpan<ContourVertex> v, int a, int b, int c, int d)
    {
        long o1 = Cross(v, a, b, c), o2 = Cross(v, a, b, d), o3 = Cross(v, c, d, a), o4 = Cross(v, c, d, b);
        if (((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0)))
        {
            return true;
        }
        return Between(v, a, b, c) || Between(v, a, b, d) || Between(v, c, d, a) || Between(v, c, d, b);
    }

    private static bool InCone(ReadOnlySpan<ContourVertex> v, int[] prev, int[] next, int a, int b)
    {
        int a0 = prev[a], a1 = next[a];
        if (LeftOn(v, a, a1, a0))
        {
            return Left(v, a, b, a0) && Left(v, b, a, a1);
        }
        return !(LeftOn(v, a, b, a1) && LeftOn(v, b, a, a0));
    }

    private static bool CrossesNoEdge(ReadOnlySpan<ContourVertex> v, int[] next, int head, int a, int b)
    {
        int k = head;
        do
        {
            int k1 = next[k];
            if (!SamePos(v, k, a) && !SamePos(v, k, b) && !SamePos(v, k1, a) && !SamePos(v, k1, b) && Meets(v, a, b, k, k1))
            {
                return false;
            }
            k = k1;
        }
        while (k != head);
        return true;
    }

    private static bool IsDiagonal(ReadOnlySpan<ContourVertex> v, int[] prev, int[] next, int head, int a, int b) =>
        InCone(v, prev, next, a, b) && InCone(v, prev, next, b, a) && CrossesNoEdge(v, next, head, a, b);

    // Returns the number of degenerate steps. Triangles are appended as local index triples, counter-clockwise.
    // Among the valid ears the one with the shortest diagonal is clipped (ties: lowest vertex index), which keeps triangles compact so that
    // more of them merge into convex polygons. A valid ear stays valid when another ear is clipped, so only the two neighbours of a clipped
    // vertex and the ears that were blocked are tested again.
    private static int EarClip(ReadOnlySpan<ContourVertex> v, List<int> tris)
    {
        int n = v.Length;
        var prev = new int[n];
        var next = new int[n];
        var alive = new bool[n];
        var valid = new bool[n];
        for (int i = 0; i < n; i++)
        {
            prev[i] = (i + n - 1) % n;
            next[i] = (i + 1) % n;
            alive[i] = true;
        }
        int count = n;
        int head = 0;
        int degenerate = 0;
        while (count > 3)
        {
            int found = -1;
            long bestLen = long.MaxValue;
            for (int i = 0; i < n; i++)
            {
                if (!alive[i])
                {
                    continue;
                }
                if (!valid[i])
                {
                    valid[i] = Cross(v, prev[i], i, next[i]) > 0 && IsDiagonal(v, prev, next, head, prev[i], next[i]);
                }
                if (valid[i])
                {
                    long dx = v[next[i]].X - v[prev[i]].X, dz = v[next[i]].Z - v[prev[i]].Z;
                    long len = (dx * dx) + (dz * dz);
                    if (len < bestLen)
                    {
                        bestLen = len;
                        found = i;
                    }
                }
            }
            if (found < 0)
            {
                // No proper ear: settle for a collinear one, then any vertex (hostile or degenerate contours only).
                degenerate++;
                int i = head;
                for (int step = 0; step < count; step++, i = next[i])
                {
                    if (Cross(v, prev[i], i, next[i]) >= 0 && IsDiagonal(v, prev, next, head, prev[i], next[i]))
                    {
                        found = i;
                        break;
                    }
                }
                if (found < 0)
                {
                    found = head;
                }
            }
            int p = prev[found], q = next[found];
            if (Cross(v, p, found, q) > 0)
            {
                tris.Add(p);
                tris.Add(found);
                tris.Add(q);
            }
            alive[found] = false;
            valid[found] = false;
            next[p] = q;
            prev[q] = p;
            valid[p] = false;
            valid[q] = false;
            head = p;
            count--;
        }
        int a = head, b = next[a], c = next[b];
        if (Cross(v, a, b, c) > 0)
        {
            tris.Add(a);
            tris.Add(b);
            tris.Add(c);
        }
        return degenerate;
    }

    private static long Cross(List<int> verts, int a, int b, int c) =>
        ((long)(verts[(3 * b)] - verts[3 * a]) * (verts[(3 * c) + 2] - verts[(3 * a) + 2])) - ((long)(verts[(3 * b) + 2] - verts[(3 * a) + 2]) * (verts[3 * c] - verts[3 * a]));

    private static int FloatOrient(List<int> verts, TileCoordinates map, int a, int b, int c) =>
        OneMath.Predicates.ExactPredicates.Orient2D(
            map.Local(verts[3 * a]), map.Local(verts[(3 * a) + 2]), map.Local(verts[3 * b]), map.Local(verts[(3 * b) + 2]), map.Local(verts[3 * c]), map.Local(verts[(3 * c) + 2]));

    private static void MergeConvex(List<int[]> polys, List<int> verts, int maxVerts, TileCoordinates map, HashSet<(int, int)> walls)
    {
        if (maxVerts <= 3)
        {
            return;
        }
        while (true)
        {
            // Directed edge (a, b) of polygon p, edge e: look up the polygon holding (b, a).
            var edgeOwner = new Dictionary<(int, int), (int Poly, int Edge)>();
            for (int p = 0; p < polys.Count; p++)
            {
                int[] pv = polys[p];
                for (int e = 0; e < pv.Length; e++)
                {
                    edgeOwner[(pv[e], pv[(e + 1) % pv.Length])] = (p, e);
                }
            }
            long best = -1;
            int bestA = -1, bestEa = -1, bestB = -1, bestEb = -1;
            for (int pa = 0; pa < polys.Count; pa++)
            {
                int[] A = polys[pa];
                for (int ea = 0; ea < A.Length; ea++)
                {
                    int a = A[ea], b = A[(ea + 1) % A.Length];
                    if (!edgeOwner.TryGetValue((b, a), out var other) || other.Poly <= pa)
                    {
                        continue;
                    }
                    int[] B = polys[other.Poly];
                    if (A.Length + B.Length - 2 > maxVerts)
                    {
                        continue;
                    }
                    long dx = verts[3 * a] - verts[3 * b], dz = verts[(3 * a) + 2] - verts[(3 * b) + 2];
                    long len = (dx * dx) + (dz * dz);
                    if (len <= best || TryMerge(A, ea, B, other.Edge, verts, map, walls, null) is null)
                    {
                        continue;
                    }
                    best = len;
                    bestA = pa;
                    bestEa = ea;
                    bestB = other.Poly;
                    bestEb = other.Edge;
                }
            }
            if (best < 0)
            {
                return;
            }
            var added = new List<(int, int)>();
            polys[bestA] = TryMerge(polys[bestA], bestEa, polys[bestB], bestEb, verts, map, walls, added)!;
            foreach ((int, int) w in added)
            {
                walls.Add(w);
            }
            polys.RemoveAt(bestB);
        }
    }

    // Merged ring: A from b round to a, then B's vertices after a up to before b.
    private static int[] Join(int[] A, int ea, int[] B, int eb)
    {
        var r = new int[A.Length + B.Length - 2];
        int k = 0;
        for (int i = 0; i < A.Length; i++)
        {
            r[k++] = A[(ea + 1 + i) % A.Length];
        }
        for (int i = 2; i < B.Length; i++)
        {
            r[k++] = B[(eb + i) % B.Length];
        }
        return r;
    }

    // The merged polygon, or null when the merge would leave a joint turning right (in cells or in the stored float32 positions, where rounding can
    // tip a collinear joint to the wrong side) or repeat a vertex. A joint that is collinear in cells and sits between two wall edges is dropped from
    // the polygon instead: nothing else refers to it, and the polygon stays exactly the same shape. The new wall edges created by drops are returned.
    private static int[]? TryMerge(int[] A, int ea, int[] B, int eb, List<int> verts, TileCoordinates map, HashSet<(int, int)> walls, List<(int, int)>? newWalls)
    {
        int a = A[ea], b = A[(ea + 1) % A.Length];
        // Distinct vertices: the only vertices the two polygons share are a and b unless a bridge welded more together.
        for (int i = 0; i < B.Length; i++)
        {
            int u = B[i];
            if (u == a || u == b)
            {
                continue;
            }
            for (int j = 0; j < A.Length; j++)
            {
                if (A[j] == u)
                {
                    return null;
                }
            }
        }
        int[] r = Join(A, ea, B, eb);
        int n = r.Length;
        int ia = A.Length - 1, ib = 0;
        bool dropA = false, dropB = false;
        for (int k = 0; k < 2; k++)
        {
            int j = k == 0 ? ia : ib;
            int pv = r[(j + n - 1) % n], cur = r[j], nx = r[(j + 1) % n];
            long c = Cross(verts, pv, cur, nx);
            if (c < 0)
            {
                return null;
            }
            bool drop = c == 0 && walls.Contains((pv, cur)) && walls.Contains((cur, nx));
            if (!drop && FloatOrient(verts, map, pv, cur, nx) < 0)
            {
                return null;
            }
            if (k == 0)
            {
                dropA = drop;
            }
            else
            {
                dropB = drop;
            }
        }
        if (!dropA && !dropB)
        {
            return IsConvex(r, verts, map) ? r : null;
        }
        var kept = new List<int>(n);
        for (int j = 0; j < n; j++)
        {
            if ((j == ia && dropA) || (j == ib && dropB))
            {
                continue;
            }
            kept.Add(r[j]);
        }
        // The turns at the neighbours of dropped vertices must still not turn right in float32.
        int m = kept.Count;
        if (m < 3)
        {
            return null;
        }
        int[] ring = kept.ToArray();
        if (!IsConvex(ring, verts, map))
        {
            return null;
        }
        if (newWalls is not null)
        {
            for (int j = 0; j < n; j++)
            {
                if ((j == ia && dropA) || (j == ib && dropB))
                {
                    newWalls.Add((r[(j + n - 1) % n], r[(j + 1) % n]));
                }
            }
        }
        return ring;
    }

    // Every vertex left of or on every edge, in cells and in the stored float32 positions. Local turns are not enough: pieces cut from a contour that
    // overlaps itself can join into a ring that winds twice.
    private static bool IsConvex(int[] ring, List<int> verts, TileCoordinates map)
    {
        int n = ring.Length;
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                // Two vertices at one place (corners of different heights over the same x, z) would make a zero-length edge.
                if (verts[3 * ring[i]] == verts[3 * ring[j]] && verts[(3 * ring[i]) + 2] == verts[(3 * ring[j]) + 2])
                {
                    return false;
                }
            }
        }
        for (int e = 0; e < n; e++)
        {
            int a = ring[e], b = ring[(e + 1) % n];
            for (int k = 0; k < n; k++)
            {
                int c = ring[k];
                if (c == a || c == b)
                {
                    continue;
                }
                if (Cross(verts, a, b, c) < 0 || FloatOrient(verts, map, a, b, c) < 0)
                {
                    return false;
                }
            }
        }
        return true;
    }
}
