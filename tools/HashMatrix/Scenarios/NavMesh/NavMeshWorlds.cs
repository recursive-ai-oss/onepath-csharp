using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Deterministic random tiled meshes used by the tests and the hash-matrix scenarios. Heights are a function of the
/// world vertex position, so neighbouring tiles agree along their shared boundary and portal links appear; layer 1
/// floats 4 units above layer 0 (no links between them) and layer 2 sits 0.25 above it (links across layers).
/// </summary>
internal static class NavMeshWorlds
{
    public const int Cells = 8;

    public static NavMeshParams Params() => new()
    {
        OriginX = -16,
        OriginZ = -16,
        TileSize = Cells,
        CellSize = 0.25,
        MaxVertsPerPoly = 6,
        WalkableClimb = 0.5f,
    };

    public sealed class Rng
    {
        private ulong _s;

        public Rng(ulong seed) => _s = seed;

        public ulong Next()
        {
            _s += 0x9E3779B97F4A7C15UL;
            ulong z = _s;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return z ^ (z >> 31);
        }

        public int Int(int n) => (int)(Next() % (ulong)n);
    }

    private static double Height(int layer, int ix, int iz)
    {
        double b = layer == 1 ? 4.0 : layer == 2 ? 0.25 : 0.0;
        return b + (((ix * 7) + (iz * 13) + (layer * 5)) & 3) * 0.125;
    }

    private static bool Present(ulong seed, int layer, int gx, int gz)
    {
        var r = new Rng(seed ^ ((ulong)(uint)gx * 0x100000001B3UL) ^ ((ulong)(uint)gz << 20) ^ ((ulong)layer << 52));
        int pct = layer == 0 ? 85 : layer == 1 ? 60 : 35;
        return r.Int(100) < pct;
    }

    public static NavTile MakeTile(NavMeshParams p, ulong seed, int tx, int tz, int layer, bool detail, int offMesh)
    {
        var rng = new Rng(seed ^ ((ulong)(uint)tx << 8) ^ ((ulong)(uint)tz << 36) ^ ((ulong)layer << 60));
        int n = Cells + 1;
        double ox = p.TileOriginX(tx), oz = p.TileOriginZ(tz);
        var wv = new double[n * n * 3];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                int k = ((i * n) + j) * 3;
                wv[k] = ox + i;
                wv[k + 1] = Height(layer, (tx * Cells) + i, (tz * Cells) + j);
                wv[k + 2] = oz + j;
            }
        }
        var counts = new List<int>();
        var idx = new List<int>();
        var areas = new List<byte>();
        var flags = new List<ushort>();
        int V(int i, int j) => (i * n) + j;
        for (int i = 0; i < Cells; i++)
        {
            for (int j = 0; j < Cells; j++)
            {
                if (!Present(seed, layer, (tx * Cells) + i, (tz * Cells) + j))
                {
                    continue;
                }
                int kind = rng.Int(4);
                byte area = (byte)rng.Int(4);
                ushort fl = (ushort)(1 << rng.Int(3));
                // Counter-clockwise in X (right) / Z (up): (i,j) -> (i+1,j) -> (i+1,j+1) -> (i,j+1).
                int[] quad = [V(i, j), V(i + 1, j), V(i + 1, j + 1), V(i, j + 1)];
                if (kind == 0 || kind == 1)
                {
                    Add(quad);
                }
                else if (kind == 2)
                {
                    Add([quad[0], quad[1], quad[2]]);
                    Add([quad[0], quad[2], quad[3]]);
                }
                else
                {
                    Add([quad[0], quad[1], quad[3]]);
                    Add([quad[1], quad[2], quad[3]]);
                }
                void Add(int[] poly)
                {
                    counts.Add(poly.Length);
                    idx.AddRange(poly);
                    areas.Add(area);
                    flags.Add(fl);
                }
            }
        }
        NavTileData d = NavTileAssembler.FromPolygons(p, tx, tz, layer, 0.0, wv, counts.ToArray(), idx.ToArray(), areas.ToArray(), flags.ToArray());
        if (detail)
        {
            AddDetail(d);
        }
        if (offMesh > 0)
        {
            var list = new List<OffMeshLink>();
            for (int k = 0; k < offMesh; k++)
            {
                if (TryOffMesh(p, seed, rng, tx, tz, layer, out var link))
                {
                    list.Add(link);
                }
            }
            d.OffMeshLinks = list.ToArray();
        }
        return NavTile.Create(d, p);
    }

    private static bool TryOffMesh(NavMeshParams p, ulong seed, Rng rng, int tx, int tz, int layer, out OffMeshLink link)
    {
        link = default;
        int sx = rng.Int(Cells), sz = rng.Int(Cells);
        if (!Present(seed, layer, (tx * Cells) + sx, (tz * Cells) + sz))
        {
            return false;
        }
        for (int t = 0; t < 8; t++)
        {
            int ex = (tx * Cells) + rng.Int(3 * Cells) - Cells;
            int ez = (tz * Cells) + rng.Int(3 * Cells) - Cells;
            int el = rng.Int(3);
            if (Present(seed, el, ex, ez))
            {
                link = new OffMeshLink
                {
                    StartX = sx + 0.5f,
                    StartY = (float)Height(layer, (tx * Cells) + sx, (tz * Cells) + sz),
                    StartZ = sz + 0.5f,
                    EndX = ex - (tx * Cells) + 0.5f,
                    EndY = (float)Height(el, ex, ez),
                    EndZ = ez - (tz * Cells) + 0.5f,
                    Radius = 0.75f,
                    Area = (byte)(8 + rng.Int(4)),
                    PolyFlags = 0x100,
                    Bidirectional = rng.Int(2) == 0,
                    UserId = (uint)rng.Int(1000),
                };
                return true;
            }
        }
        return false;
    }

    // One extra vertex per polygon (above its centroid) and a triangle fan around it.
    private static void AddDetail(NavTileData d)
    {
        var dv = new List<float>();
        var dt = new List<byte>();
        var meshes = new DetailMesh[d.PolyVertexCounts!.Length];
        int pos = 0;
        for (int pi = 0; pi < meshes.Length; pi++)
        {
            int n = d.PolyVertexCounts[pi];
            float cx = 0, cy = 0, cz = 0;
            for (int e = 0; e < n; e++)
            {
                int v = d.PolyIndices![pos + e];
                cx += d.Vertices![3 * v];
                cy += d.Vertices[(3 * v) + 1];
                cz += d.Vertices[(3 * v) + 2];
            }
            meshes[pi] = new DetailMesh { VertexBase = (uint)(dv.Count / 3), TriangleBase = (uint)(dt.Count / 4), VertexCount = 1, TriangleCount = (ushort)n };
            dv.Add(cx / n);
            dv.Add((cy / n) + 0.0625f);
            dv.Add(cz / n);
            for (int e = 0; e < n; e++)
            {
                dt.Add((byte)e);
                dt.Add((byte)((e + 1) % n));
                dt.Add((byte)n);
                dt.Add(1);
            }
            pos += n;
        }
        d.DetailMeshes = meshes;
        d.DetailVertices = dv.ToArray();
        d.DetailTriangles = dt.ToArray();
    }

    public static TiledNavMesh RandomMesh(ulong seed, int x0, int z0, int nx, int nz, int layers, bool detail, int offMesh)
    {
        NavMeshParams p = Params();
        var mesh = new TiledNavMesh(p);
        for (int x = x0; x < x0 + nx; x++)
        {
            for (int z = z0; z < z0 + nz; z++)
            {
                for (int l = 0; l < layers; l++)
                {
                    mesh.AddTile(MakeTile(p, seed, x, z, l, detail, offMesh));
                }
            }
        }
        return mesh;
    }

    /// <summary>Feeds every stitched link of the mesh, in slot/polygon/link order, to <paramref name="sink"/> as integers.</summary>
    public static void DumpLinks(TiledNavMesh m, Action<long> sink)
    {
        sink(m.SlotCount);
        for (int s = 0; s < m.SlotCount; s++)
        {
            NavTile? t = m.GetTileBySlot(s);
            sink(t is null ? -1 : t.PolyCount);
            if (t is null)
            {
                continue;
            }
            for (int p = 0; p < t.PolyCount; p++)
            {
                var links = m.GetLinks(m.GetSlotRef(s).Poly(p));
                sink(links.Length);
                foreach (NavLink l in links)
                {
                    sink((long)l.Target.Value);
                    sink(((long)l.Kind << 40) | ((long)l.Side << 32) | ((long)l.RangeMin << 24) | ((long)l.RangeMax << 16));
                    sink(l.Index);
                    sink(l.OwnerSlot);
                }
            }
        }
    }
}
