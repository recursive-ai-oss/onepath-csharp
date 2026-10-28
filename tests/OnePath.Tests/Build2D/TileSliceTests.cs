using System;
using System.Collections.Generic;
using System.Numerics;
using RecursiveAI.OneMath;
using RecursiveAI.OneMath.Predicates;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

public class TileSliceTests
{
    private struct Rng
    {
        public ulong S;

        public int Next(int n)
        {
            ulong z = S += 0x9E3779B97F4A7C15UL;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return (int)((z ^ (z >> 31)) % (ulong)n);
        }
    }

    private static readonly Geometry2DOptions Grid10 = new() { SnapBits = 10 };

    private static Outline Poly(params Vector2D[] pts) => new(new IReadOnlyList<Vector2D>[] { pts });

    private static List<Outline> RandomOutlines(ref Rng rng, int count, int extent, int denom, bool rectilinear)
    {
        var list = new List<Outline>();
        for (int o = 0; o < count; o++)
        {
            Vector2D[] pts;
            if (rectilinear)
            {
                // Axis-aligned rectangles and 45-degree diamonds on integer coordinates: every crossing with an integer line is a grid point.
                int x = rng.Next(extent), y = rng.Next(extent), w = 1 + rng.Next(extent / 2), h = 1 + rng.Next(extent / 2);
                pts = rng.Next(2) == 0
                    ? new[] { new Vector2D(x, y), new Vector2D(x + w, y), new Vector2D(x + w, y + h), new Vector2D(x, y + h) }
                    : new[] { new Vector2D(x + w, y), new Vector2D(x + (2 * w), y + w), new Vector2D(x + w, y + (2 * w)), new Vector2D(x, y + w) };
            }
            else
            {
                pts = new Vector2D[3 + rng.Next(6)];
                for (int i = 0; i < pts.Length; i++)
                {
                    pts[i] = new Vector2D(rng.Next((extent * denom) + 1) / (double)denom, rng.Next((extent * denom) + 1) / (double)denom);
                }
            }
            list.Add(Poly(pts));
        }
        return list;
    }

    private static BigInteger TwiceArea(ConvexPartition cp, int bits)
    {
        BigInteger twice = 0;
        for (int p = 0; p < cp.PolygonCount; p++)
        {
            var ids = cp.GetIndices(p).ToArray();
            var g = new (long, long)[ids.Length];
            for (int i = 0; i < ids.Length; i++)
            {
                g[i] = GeoOracle.Grid(new[] { cp.Vertices[ids[i]] }, bits)[0];
            }
            BigInteger a = GeoOracle.TwiceArea(g);
            Assert.True(a > 0, "polygon has no area");
            twice += a;
            for (int i = 0; i < ids.Length; i++)
            {
                Vector2D p0 = cp.Vertices[ids[i]], p1 = cp.Vertices[ids[(i + 1) % ids.Length]], p2 = cp.Vertices[ids[(i + 2) % ids.Length]];
                Assert.True(ExactPredicates.Orient2D(p0.X, p0.Y, p1.X, p1.Y, p2.X, p2.Y) >= 0, "polygon not convex");
            }
        }
        return twice;
    }

    private static (TiledNavMesh Mesh, IReadOnlyList<TilePartition> Tiles, BigInteger Twice) Slice(Region region, NavMeshParams p)
    {
        IReadOnlyList<TilePartition> tiles = TileSlicer.Slice(region, p);
        BigInteger twice = 0;
        var data = new List<NavTileData>();
        foreach (TilePartition t in tiles)
        {
            Assert.True(t.Partition.PolygonCount <= int.MaxValue);
            twice += TwiceArea(t.Partition, region.SnapBits);
            data.Add(Mesh2DPipeline.ToTileData(t, p, 1.5, 0, 1));
        }
        TiledNavMesh mesh = Mesh2DPipeline.Assemble(p, data);
        Assert.Empty(NavMeshValidator.Validate(mesh));
        return (mesh, tiles, twice);
    }

    private static void CheckPortalsCover(TiledNavMesh mesh)
    {
        // Both sides of a tile line are cut at the same vertices, so edges that overlap are identical: each such pair must be linked.
        var expected = new SortedSet<(ulong, ulong)>();
        var edgeOwner = new Dictionary<(double, double, double, double), PolyRef>();
        var tilesList = new List<(int Slot, NavTile Tile)>();
        for (int s = 0; s < mesh.SlotCount; s++)
        {
            NavTile? t = mesh.GetTileBySlot(s);
            if (t is null) { continue; }
            tilesList.Add((s, t));
            TileRef tr = mesh.GetSlotRef(s);
            for (int p = 0; p < t.PolyCount; p++)
            {
                int n = t.PolyVertexCounts[p], f = t.PolyFirstIndex(p);
                for (int e = 0; e < n; e++)
                {
                    if (t.EdgePortalSides[f + e] == NavMeshFormat.NoSide) { continue; }
                    int a = t.PolyIndices[f + e], b = t.PolyIndices[f + ((e + 1) % n)];
                    var key = (t.OriginX + t.Vertices[3 * a], t.OriginZ + t.Vertices[(3 * a) + 2], t.OriginX + t.Vertices[3 * b], t.OriginZ + t.Vertices[(3 * b) + 2]);
                    edgeOwner[key] = tr.Poly(p);
                }
            }
        }
        foreach (var kv in edgeOwner)
        {
            var (ax, az, bx, bz) = kv.Key;
            if (edgeOwner.TryGetValue((bx, bz, ax, az), out PolyRef other))
            {
                expected.Add((kv.Value.Value, other.Value));
            }
        }
        var actual = new SortedSet<(ulong, ulong)>();
        foreach (var (slot, t) in tilesList)
        {
            TileRef tr = mesh.GetSlotRef(slot);
            for (int p = 0; p < t.PolyCount; p++)
            {
                foreach (NavLink l in mesh.GetLinks(tr.Poly(p)))
                {
                    if (l.Kind == NavLinkKind.Portal) { actual.Add((tr.Poly(p).Value, l.Target.Value)); }
                }
            }
        }
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void Rectilinear_outlines_are_cut_exactly()
    {
        var rng = new Rng { S = 7 };
        for (int i = 0; i < 300; i++)
        {
            Region region = Region.FromOutlines(RandomOutlines(ref rng, 1 + rng.Next(4), 24, 1, true), Grid10);
            double ts = new[] { 2.0, 4.0, 8.0, 16.0 }[rng.Next(4)];
            var p = new NavMeshParams { TileSize = ts, CellSize = 0.25, OriginX = rng.Next(3) - 1, OriginZ = rng.Next(3) - 1 };
            var (mesh, tiles, twice) = Slice(region, p);
            Assert.Equal(GeoOracle.ValidateRegion(region), twice);
            CheckPortalsCover(mesh);
            foreach (TilePartition t in tiles)
            {
                foreach (Vector2D v in t.Partition.Vertices)
                {
                    Assert.InRange(v.X, p.TileOriginX(t.TileX), p.TileOriginX(t.TileX + 1));
                    Assert.InRange(v.Y, p.TileOriginZ(t.TileZ), p.TileOriginZ(t.TileZ + 1));
                }
            }
        }
    }

    [Fact]
    public void General_outlines_keep_area_within_rounding_and_membership()
    {
        var rng = new Rng { S = 11 };
        for (int i = 0; i < 300; i++)
        {
            int denom = new[] { 1, 8, 1024 }[rng.Next(3)];
            Region region = Region.FromOutlines(RandomOutlines(ref rng, 1 + rng.Next(3), 20, denom, false), Grid10);
            double ts = new[] { 2.0, 4.0, 8.0 }[rng.Next(3)];
            var p = new NavMeshParams { TileSize = ts, CellSize = 0.25, OriginX = (rng.Next(5) * 0.5) - 1, OriginZ = rng.Next(5) * 0.25 };
            var (mesh, tiles, twice) = Slice(region, p);
            CheckPortalsCover(mesh);
            // Only cut points on the boundary are rounded, each by under one grid unit: bound the area change by the perimeter.
            BigInteger perimeter = 0;
            var rings = new List<Vector2D[]>();
            for (int r = 0; r < region.RingCount; r++)
            {
                rings.Add(region.GetRing(r));
                var g = GeoOracle.Grid(rings[r], region.SnapBits);
                for (int k = 0; k < g.Length; k++)
                {
                    var (a, b) = (g[k], g[(k + 1) % g.Length]);
                    perimeter += BigInteger.Abs(a.X - b.X) + BigInteger.Abs(a.Y - b.Y);
                }
            }
            BigInteger diff = BigInteger.Abs(twice - GeoOracle.ValidateRegion(region));
            Assert.True(diff <= 2 * perimeter, $"case {i}: area differs by {diff}, perimeter {perimeter}");
            for (int k = 0; k < 60; k++)
            {
                double x = rng.Next(21000) / 1000.0, y = rng.Next(21000) / 1000.0;
                if (GeoOracle.MinEdgeDistance(rings, x, y) < 1e-3) { continue; }
                bool inside = false;
                foreach (TilePartition t in tiles)
                {
                    for (int q = 0; q < t.Partition.PolygonCount; q++)
                    {
                        var ids = t.Partition.GetIndices(q);
                        bool all = true;
                        for (int e = 0; e < ids.Length && all; e++)
                        {
                            Vector2D a = t.Partition.Vertices[ids[e]], b = t.Partition.Vertices[ids[(e + 1) % ids.Length]];
                            all = (((b.X - a.X) * (y - a.Y)) - ((b.Y - a.Y) * (x - a.X))) >= 0;
                        }
                        inside |= all;
                    }
                }
                Assert.Equal(GeoOracle.InRegion(region, x, y), inside);
            }
        }
    }

    [Fact]
    public void Degenerate_boundaries()
    {
        // Edges on tile lines, vertices on tile corners, a sliver straddling a line, a triangle with a vertex on a corner.
        var shapes = new List<Outline>
        {
            Poly(new Vector2D(0, 0), new Vector2D(4, 0), new Vector2D(4, 4), new Vector2D(0, 4)),
            Poly(new Vector2D(4, 4), new Vector2D(8, 4), new Vector2D(8, 8), new Vector2D(4, 8)),
            Poly(new Vector2D(10, 1), new Vector2D(10 + (1.0 / 1024), 1), new Vector2D(10, 9)),
            Poly(new Vector2D(12, 0), new Vector2D(20, 8), new Vector2D(12, 8)),
        };
        Region region = Region.FromOutlines(shapes, Grid10);
        foreach (double ts in new[] { 2.0, 4.0, 8.0 })
        {
            var p = new NavMeshParams { TileSize = ts, CellSize = 0.25 };
            var (mesh, _, _) = Slice(region, p);
            CheckPortalsCover(mesh);
        }
    }

    [Fact]
    public void A_grid_finer_than_the_cell_size_allows_is_rejected_with_the_bits_to_use()
    {
        Region region = Region.FromOutlines(new[] { Poly(new Vector2D(0, 0), new Vector2D(10, 0), new Vector2D(10, 10)) });
        var ex = Assert.Throws<ArgumentException>(() => TileSlicer.Slice(region, new NavMeshParams { TileSize = 8, CellSize = 0.25 }));
        Assert.Contains("SnapBits 10", ex.Message);
    }

    [Fact]
    public void Tile_lines_off_the_snap_grid_are_rejected_with_a_message()
    {
        Region region = Region.FromOutlines(
            new[] { Poly(new Vector2D(0, 0), new Vector2D(100, 0), new Vector2D(100, 100)) },
            new Geometry2DOptions { SnapBits = 0 });
        var p = new NavMeshParams { TileSize = 12.5, CellSize = 0.25 };
        var ex = Assert.Throws<ArgumentException>(() => TileSlicer.Slice(region, p));
        Assert.Contains("snap grid", ex.Message);
    }

    [Fact]
    public void Tier1_builds_and_shrinks_by_the_agent_radius()
    {
        var cells = new bool[20 * 20];
        Array.Fill(cells, true);
        NavMesh nav = NavMesh.Build2D(cells, 20, 20, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { TileSize = 8 });
        Assert.True(nav.Tiles.TileCount >= 9);
        Assert.Empty(NavMeshValidator.Validate(nav.Tiles));
        double area = 0;
        for (int s = 0; s < nav.Tiles.SlotCount; s++)
        {
            NavTile? t = nav.Tiles.GetTileBySlot(s);
            if (t is null) { continue; }
            for (int p = 0; p < t.PolyCount; p++)
            {
                int n = t.PolyVertexCounts[p], f = t.PolyFirstIndex(p);
                for (int e = 0; e < n; e++)
                {
                    int a = t.PolyIndices[f + e], b = t.PolyIndices[f + ((e + 1) % n)];
                    double ax = t.OriginX + t.Vertices[3 * a], az = t.OriginZ + t.Vertices[(3 * a) + 2];
                    double bx = t.OriginX + t.Vertices[3 * b], bz = t.OriginZ + t.Vertices[(3 * b) + 2];
                    area += (ax * bz) - (bx * az);
                }
            }
        }
        // A 20 x 20 floor shrunk by 0.6 on every side is 18.8 x 18.8; the offset is rounded to the 2^-9 grid (0.6 -> 307/512).
        Assert.InRange(area / 2, 353.40, 353.50);
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build2D(cells, 20, 20, 1.0, new Vector2D(3e6, 0)));
    }

    [Fact]
    public void Tier1_find_path_crosses_tile_boundaries()
    {
        var cells = new bool[40 * 10];
        Array.Fill(cells, true);
        NavMesh nav = NavMesh.Build2D(cells, 40, 10, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { TileSize = 8 });
        PathResult r = nav.FindPath(new Vector3D(2, 0, 5), new Vector3D(38, 0, 5));
        Assert.Equal(PathStatus.Found, r.Status);
        Assert.True(r.Points.Count >= 2);
        Assert.Equal(36.0, r.Points[^1].Position.X - r.Points[0].Position.X, 2);
    }
}
