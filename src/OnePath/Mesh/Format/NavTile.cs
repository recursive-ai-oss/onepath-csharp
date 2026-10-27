using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// One immutable, validated tile of a tiled navigation mesh: convex polygons over shared tile-local float32
/// vertices, in-tile adjacency, tile-boundary portal edges, areas and flags, optional per-polygon height
/// detail and off-mesh link slots. Create one with <see cref="Create"/>; a successfully created tile satisfies
/// every structural rule in <c>docs/navmesh-format.md</c>, so readers need not re-check indices.
/// </summary>
public sealed class NavTile
{
    private readonly float[] _vertices;
    private readonly byte[] _polyVertexCounts;
    private readonly ushort[] _polyIndices;
    private readonly int[] _edgeNeighbors;
    private readonly byte[] _edgePortalSides;
    private readonly byte[] _polyAreas;
    private readonly ushort[] _polyFlags;
    private readonly DetailMesh[] _detailMeshes;
    private readonly float[] _detailVertices;
    private readonly byte[] _detailTriangles;
    private readonly OffMeshLink[] _offMesh;
    private readonly int[] _polyStart;

    // Portal edges per side, as (polygon index, global edge index into the index arrays), in polygon then edge order.
    private readonly int[][] _portalPoly;
    private readonly int[][] _portalEdge;

    private NavTile(
        NavMeshParams p, int tileX, int tileZ, int layer, double ox, double oy, double oz,
        float[] vertices, byte[] counts, ushort[] indices, int[] neighbors, byte[] sides, byte[] areas, ushort[] flags,
        DetailMesh[] detailMeshes, float[] detailVerts, byte[] detailTris, OffMeshLink[] offMesh, int[] polyStart,
        int[][] portalPoly, int[][] portalEdge, Vector3F bmin, Vector3F bmax)
    {
        Params = p;
        TileX = tileX;
        TileZ = tileZ;
        Layer = layer;
        OriginX = ox;
        OriginY = oy;
        OriginZ = oz;
        _vertices = vertices;
        _polyVertexCounts = counts;
        _polyIndices = indices;
        _edgeNeighbors = neighbors;
        _edgePortalSides = sides;
        _polyAreas = areas;
        _polyFlags = flags;
        _detailMeshes = detailMeshes;
        _detailVertices = detailVerts;
        _detailTriangles = detailTris;
        _offMesh = offMesh;
        _polyStart = polyStart;
        _portalPoly = portalPoly;
        _portalEdge = portalEdge;
        BoundsMin = bmin;
        BoundsMax = bmax;
    }

    /// <summary>The mesh parameters this tile was validated against.</summary>
    public NavMeshParams Params { get; }

    /// <summary>Tile column.</summary>
    public int TileX { get; }

    /// <summary>Tile row.</summary>
    public int TileZ { get; }

    /// <summary>Layer.</summary>
    public int Layer { get; }

    /// <summary>World X of the tile origin (<c>OriginX + TileX * TileSize</c>).</summary>
    public double OriginX { get; }

    /// <summary>World Y of the tile-local Y zero.</summary>
    public double OriginY { get; }

    /// <summary>World Z of the tile origin.</summary>
    public double OriginZ { get; }

    /// <summary>Number of vertices.</summary>
    public int VertexCount => _vertices.Length / 3;

    /// <summary>Number of polygons.</summary>
    public int PolyCount => _polyVertexCounts.Length;

    /// <summary>Smallest tile-local vertex coordinate on each axis (zero for a tile without vertices).</summary>
    public Vector3F BoundsMin { get; }

    /// <summary>Largest tile-local vertex coordinate on each axis.</summary>
    public Vector3F BoundsMax { get; }

    /// <summary>Vertices, three float32 each.</summary>
    public ReadOnlySpan<float> Vertices => _vertices;

    /// <summary>Vertex count of each polygon.</summary>
    public ReadOnlySpan<byte> PolyVertexCounts => _polyVertexCounts;

    /// <summary>All polygon vertex indices.</summary>
    public ReadOnlySpan<ushort> PolyIndices => _polyIndices;

    /// <summary>Per edge neighbour code: 0 none, otherwise polygon index + 1.</summary>
    public ReadOnlySpan<int> EdgeNeighbors => _edgeNeighbors;

    /// <summary>Per edge portal side, 255 when the edge is not a portal.</summary>
    public ReadOnlySpan<byte> EdgePortalSides => _edgePortalSides;

    /// <summary>Polygon areas.</summary>
    public ReadOnlySpan<byte> PolyAreas => _polyAreas;

    /// <summary>Polygon flags.</summary>
    public ReadOnlySpan<ushort> PolyFlags => _polyFlags;

    /// <summary>Per-polygon detail ranges (always one per polygon; zeros when there is no detail).</summary>
    public ReadOnlySpan<DetailMesh> DetailMeshes => _detailMeshes;

    /// <summary>Extra detail vertices, three float32 each.</summary>
    public ReadOnlySpan<float> DetailVertices => _detailVertices;

    /// <summary>Detail triangles, four bytes each.</summary>
    public ReadOnlySpan<byte> DetailTriangles => _detailTriangles;

    /// <summary>Off-mesh link slots owned by this tile.</summary>
    public ReadOnlySpan<OffMeshLink> OffMeshLinks => _offMesh;

    /// <summary>Index into <see cref="PolyIndices"/> of the first vertex of polygon <paramref name="poly"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="poly"/> is not a polygon index of this tile.</exception>
    public int PolyFirstIndex(int poly)
    {
        if ((uint)poly >= (uint)PolyCount)
        {
            throw new ArgumentOutOfRangeException(nameof(poly), poly, "Polygon index is outside the tile.");
        }
        return _polyStart[poly];
    }

    internal int[] PolyStart => _polyStart;

    internal int PortalCount(int side) => _portalPoly[side].Length;

    internal int PortalPoly(int side, int i) => _portalPoly[side][i];

    internal int PortalEdge(int side, int i) => _portalEdge[side][i];

    /// <summary>Copies the tile back into an editable <see cref="NavTileData"/>.</summary>
    public NavTileData ToData() => new()
    {
        TileX = TileX,
        TileZ = TileZ,
        Layer = Layer,
        OriginY = OriginY,
        Vertices = (float[])_vertices.Clone(),
        PolyVertexCounts = (byte[])_polyVertexCounts.Clone(),
        PolyIndices = (ushort[])_polyIndices.Clone(),
        EdgeNeighbors = (int[])_edgeNeighbors.Clone(),
        EdgePortalSides = (byte[])_edgePortalSides.Clone(),
        PolyAreas = (byte[])_polyAreas.Clone(),
        PolyFlags = (ushort[])_polyFlags.Clone(),
        DetailMeshes = (DetailMesh[])_detailMeshes.Clone(),
        DetailVertices = (float[])_detailVertices.Clone(),
        DetailTriangles = (byte[])_detailTriangles.Clone(),
        OffMeshLinks = (OffMeshLink[])_offMesh.Clone(),
    };

    /// <summary>
    /// Validates <paramref name="data"/> against <paramref name="p"/> and builds the tile (the arrays are copied).
    /// </summary>
    /// <exception cref="NavMeshFormatException">The data breaks a structural rule; the message names the polygon, edge or array.</exception>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The parameters are invalid, or the tile origin is beyond ±2^21.</exception>
    public static NavTile Create(NavTileData data, NavMeshParams p)
    {
        ArgumentNullException.ThrowIfNull(data);
        ArgumentNullException.ThrowIfNull(p);
        p.Validate();
        string where = $"tile ({data.TileX}, {data.TileZ}, layer {data.Layer})";
        double ox = p.TileOriginX(data.TileX);
        double oz = p.TileOriginZ(data.TileZ);
        if (!(ox > -NavMeshParams.MaxWorld && ox + p.TileSize < NavMeshParams.MaxWorld)
            || !(oz > -NavMeshParams.MaxWorld && oz + p.TileSize < NavMeshParams.MaxWorld))
        {
            throw new ArgumentOutOfRangeException(nameof(data), $"{where}: the tile spans world X {ox:R}..{ox + p.TileSize:R}, Z {oz:R}..{oz + p.TileSize:R}, beyond the bound ±{NavMeshParams.MaxWorld:R}.");
        }
        if (data.Layer < 0 || data.Layer > NavMeshFormat.MaxLayer)
        {
            throw new NavMeshFormatException($"{where}: layer must be 0 to {NavMeshFormat.MaxLayer}.");
        }
        if (!double.IsFinite(data.OriginY) || Math.Abs(data.OriginY) >= NavMeshParams.MaxWorld)
        {
            throw new NavMeshFormatException($"{where}: OriginY {data.OriginY:R} must be finite and within ±{NavMeshParams.MaxWorld:R}.");
        }

        float[] verts = Copy(data.Vertices);
        byte[] counts = Copy(data.PolyVertexCounts);
        ushort[] idx = Copy(data.PolyIndices);
        int[] nbr = Copy(data.EdgeNeighbors);
        byte[] sides = Copy(data.EdgePortalSides);
        byte[] areas = Copy(data.PolyAreas);
        ushort[] flags = Copy(data.PolyFlags);
        float[] dverts = Copy(data.DetailVertices);
        byte[] dtris = Copy(data.DetailTriangles);
        OffMeshLink[] offMesh = Copy(data.OffMeshLinks);
        int polyCount = counts.Length;
        DetailMesh[] dmesh = data.DetailMeshes is { Length: > 0 } dm ? (DetailMesh[])dm.Clone() : new DetailMesh[polyCount];

        if (verts.Length % 3 != 0 || verts.Length / 3 > ushort.MaxValue + 1)
        {
            throw new NavMeshFormatException($"{where}: Vertices must hold 3 floats per vertex and at most 65536 vertices (length {verts.Length}).");
        }
        if (polyCount > NavMeshFormat.MaxPolygonsPerTile)
        {
            throw new NavMeshFormatException($"{where}: {polyCount} polygons exceed the limit of {NavMeshFormat.MaxPolygonsPerTile}.");
        }
        long total = 0;
        int[] start = new int[polyCount + 1];
        for (int i = 0; i < polyCount; i++)
        {
            start[i] = (int)total;
            total += counts[i];
        }
        start[polyCount] = (int)total;
        if (idx.Length != total || nbr.Length != total || sides.Length != total)
        {
            throw new NavMeshFormatException($"{where}: PolyIndices ({idx.Length}), EdgeNeighbors ({nbr.Length}) and EdgePortalSides ({sides.Length}) must each hold {total} entries (the sum of PolyVertexCounts).");
        }
        if (areas.Length != polyCount || flags.Length != polyCount || dmesh.Length != polyCount)
        {
            throw new NavMeshFormatException($"{where}: PolyAreas ({areas.Length}), PolyFlags ({flags.Length}) and DetailMeshes ({dmesh.Length}) must hold one entry per polygon ({polyCount}).");
        }

        int vcount = verts.Length / 3;
        double ext = p.MaxLocalExtent;
        float ts = (float)p.TileSize;
        double lo = -p.TileSize;
        double hi = 2 * p.TileSize;
        Vector3F bmin = default;
        Vector3F bmax = default;
        for (int i = 0; i < vcount; i++)
        {
            float x = verts[3 * i], y = verts[(3 * i) + 1], z = verts[(3 * i) + 2];
            if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z))
            {
                throw new NavMeshFormatException($"{where}: vertex {i} has a non-finite coordinate ({x}, {y}, {z}).");
            }
            if (x < lo || x > hi || z < lo || z > hi || Math.Abs(y) > ext)
            {
                throw new NavMeshFormatException($"{where}: vertex {i} ({x}, {y}, {z}) is outside the tile-local bounds (X and Z within [{lo:R}, {hi:R}], |Y| at most {ext:R}).");
            }
            // The exact orientation test supports coordinates down to 2^-140; reject smaller nonzero ones outright.
            if ((x != 0 && Math.Abs(x) < MinHorizontal) || (z != 0 && Math.Abs(z) < MinHorizontal))
            {
                throw new NavMeshFormatException($"{where}: vertex {i} has a nonzero X or Z magnitude below {MinHorizontal}.");
            }
            if (i == 0)
            {
                bmin = new Vector3F(x, y, z);
                bmax = bmin;
            }
            else
            {
                bmin = new Vector3F(Math.Min(bmin.X, x), Math.Min(bmin.Y, y), Math.Min(bmin.Z, z));
                bmax = new Vector3F(Math.Max(bmax.X, x), Math.Max(bmax.Y, y), Math.Max(bmax.Z, z));
            }
        }

        var portalPoly = new List<int>[4];
        var portalEdge = new List<int>[4];
        for (int s = 0; s < 4; s++)
        {
            portalPoly[s] = new List<int>();
            portalEdge[s] = new List<int>();
        }

        for (int pi = 0; pi < polyCount; pi++)
        {
            int n = counts[pi];
            int b = start[pi];
            if (n < 3 || n > p.MaxVertsPerPoly)
            {
                throw new NavMeshFormatException($"{where}: polygon {pi} has {n} vertices; the mesh allows 3 to {p.MaxVertsPerPoly}.");
            }
            if (areas[pi] > NavMeshFormat.MaxArea)
            {
                throw new NavMeshFormatException($"{where}: polygon {pi} has area {areas[pi]}; the largest area id is {NavMeshFormat.MaxArea}.");
            }
            for (int i = 0; i < n; i++)
            {
                if (idx[b + i] >= vcount)
                {
                    throw new NavMeshFormatException($"{where}: polygon {pi} vertex {i} refers to vertex {idx[b + i]} but the tile has {vcount}.");
                }
            }
            CheckConvex(where, pi, verts, idx, b, n);
            for (int e = 0; e < n; e++)
            {
                int g = b + e;
                int a0 = idx[g];
                int a1 = idx[b + ((e + 1) % n)];
                int nb = nbr[g];
                byte side = sides[g];
                if (nb < 0 || nb > polyCount)
                {
                    throw new NavMeshFormatException($"{where}: polygon {pi} edge {e} has neighbour code {nb}; it must be 0 to {polyCount}.");
                }
                if (nb != 0)
                {
                    int q = nb - 1;
                    if (q == pi)
                    {
                        throw new NavMeshFormatException($"{where}: polygon {pi} edge {e} is its own neighbour.");
                    }
                    if (side != NavMeshFormat.NoSide)
                    {
                        throw new NavMeshFormatException($"{where}: polygon {pi} edge {e} has an in-tile neighbour and a portal side.");
                    }
                    int qn = counts[q];
                    int qb = start[q];
                    bool back = false;
                    for (int f = 0; f < qn; f++)
                    {
                        if (idx[qb + f] == a1 && idx[qb + ((f + 1) % qn)] == a0)
                        {
                            back = nbr[qb + f] == pi + 1;
                            break;
                        }
                    }
                    if (!back)
                    {
                        throw new NavMeshFormatException($"{where}: polygon {pi} edge {e} names polygon {q} as neighbour, but polygon {q} has no matching reversed edge that names it back.");
                    }
                }
                else if (side != NavMeshFormat.NoSide)
                {
                    if (side > 3)
                    {
                        throw new NavMeshFormatException($"{where}: polygon {pi} edge {e} has portal side {side}; it must be 0 to 3 or 255.");
                    }
                    CheckOnSide(where, pi, e, side, verts, a0, a1, ts);
                    portalPoly[side].Add(pi);
                    portalEdge[side].Add(g);
                }
            }
        }

        // Detail meshes: ranges inside the arrays, triangle indices inside polygon vertices plus the polygon's extra vertices.
        if (dverts.Length % 3 != 0 || dtris.Length % 4 != 0)
        {
            throw new NavMeshFormatException($"{where}: DetailVertices must hold 3 floats per vertex and DetailTriangles 4 bytes per triangle.");
        }
        for (int i = 0; i < dverts.Length; i++)
        {
            float v = dverts[i];
            double lim = i % 3 == 1 ? ext : hi;
            double lolim = i % 3 == 1 ? -ext : lo;
            if (!float.IsFinite(v) || v < lolim || v > lim)
            {
                throw new NavMeshFormatException($"{where}: detail vertex {i / 3} has a non-finite or out-of-bounds coordinate ({v}).");
            }
        }
        long dvc = dverts.Length / 3;
        long dtc = dtris.Length / 4;
        for (int pi = 0; pi < polyCount; pi++)
        {
            DetailMesh m = dmesh[pi];
            if (m.VertexBase + (long)m.VertexCount > dvc || m.TriangleBase + (long)m.TriangleCount > dtc)
            {
                throw new NavMeshFormatException($"{where}: polygon {pi} detail range (vertices {m.VertexBase}+{m.VertexCount} of {dvc}, triangles {m.TriangleBase}+{m.TriangleCount} of {dtc}) is outside the detail arrays.");
            }
            int limit = counts[pi] + m.VertexCount;
            for (long t = m.TriangleBase; t < m.TriangleBase + m.TriangleCount; t++)
            {
                for (int k = 0; k < 3; k++)
                {
                    if (dtris[(4 * t) + k] >= limit)
                    {
                        throw new NavMeshFormatException($"{where}: polygon {pi} detail triangle {t - m.TriangleBase} refers to vertex {dtris[(4 * t) + k]} but the polygon has {limit}.");
                    }
                }
            }
        }

        for (int i = 0; i < offMesh.Length; i++)
        {
            OffMeshLink l = offMesh[i];
            Span<float> c = [l.StartX, l.StartY, l.StartZ, l.EndX, l.EndY, l.EndZ, l.Radius];
            for (int k = 0; k < c.Length; k++)
            {
                if (!float.IsFinite(c[k]) || Math.Abs(c[k]) > ext)
                {
                    throw new NavMeshFormatException($"{where}: off-mesh link {i} has a non-finite or out-of-bounds value ({c[k]}) at position {k}.");
                }
            }
            if (!(l.Radius > 0) || l.Area > NavMeshFormat.MaxArea)
            {
                throw new NavMeshFormatException($"{where}: off-mesh link {i} needs a positive radius and an area of at most {NavMeshFormat.MaxArea}.");
            }
        }

        var pp = new int[4][];
        var pe = new int[4][];
        for (int s = 0; s < 4; s++)
        {
            pp[s] = portalPoly[s].ToArray();
            pe[s] = portalEdge[s].ToArray();
        }
        return new NavTile(p, data.TileX, data.TileZ, data.Layer, ox, data.OriginY, oz, verts, counts, idx, nbr, sides, areas, flags,
            dmesh, dverts, dtris, offMesh, start, pp, pe, bmin, bmax);
    }

    private const float MinHorizontal = 1e-30f;

    private static T[] Copy<T>(T[]? a) => a is null ? [] : (T[])a.Clone();

    // Convex, counter-clockwise (X right, Z up), non-degenerate. Every vertex lies left of or on every edge, decided
    // with the exact orientation predicate, which also rules out self-overlapping star shapes.
    private static void CheckConvex(string where, int poly, float[] v, ushort[] idx, int b, int n)
    {
        bool positive = false;
        for (int e = 0; e < n; e++)
        {
            int a = idx[b + e];
            int c = idx[b + ((e + 1) % n)];
            if (v[3 * a] == v[3 * c] && v[(3 * a) + 2] == v[(3 * c) + 2])
            {
                throw new NavMeshFormatException($"{where}: polygon {poly} edge {e} has zero length in X/Z.");
            }
            for (int k = 0; k < n; k++)
            {
                int d = idx[b + k];
                if (d == a || d == c)
                {
                    continue;
                }
                int o = ExactPredicates.Orient2D(v[3 * a], v[(3 * a) + 2], v[3 * c], v[(3 * c) + 2], v[3 * d], v[(3 * d) + 2]);
                if (o < 0)
                {
                    throw new NavMeshFormatException($"{where}: polygon {poly} is not convex and counter-clockwise: vertex {k} is right of edge {e}.");
                }
                if (o > 0)
                {
                    positive = true;
                }
            }
        }
        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (idx[b + i] == idx[b + j])
                {
                    throw new NavMeshFormatException($"{where}: polygon {poly} repeats vertex {idx[b + i]}.");
                }
            }
        }
        if (!positive)
        {
            throw new NavMeshFormatException($"{where}: polygon {poly} has zero area in X/Z.");
        }
    }

    // A portal edge lies on the tile boundary of its side, with extent along it.
    private static void CheckOnSide(string where, int poly, int edge, byte side, float[] v, int a0, int a1, float tileSize)
    {
        int axis = (side & 1) == 0 ? 0 : 2;
        int along = 2 - axis;
        float line = side < 2 ? tileSize : 0f;
        if (v[(3 * a0) + axis] != line || v[(3 * a1) + axis] != line)
        {
            throw new NavMeshFormatException($"{where}: polygon {poly} edge {edge} is a portal on side {side} but does not lie exactly on the tile boundary {line}.");
        }
        if (v[(3 * a0) + along] == v[(3 * a1) + along])
        {
            throw new NavMeshFormatException($"{where}: polygon {poly} edge {edge} is a portal of zero length.");
        }
    }
}
