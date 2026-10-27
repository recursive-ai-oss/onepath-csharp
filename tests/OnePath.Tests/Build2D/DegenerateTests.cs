using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.NavMesh.Build2D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

/// <summary>The degenerate-input suite: every case must give a valid, exactly accounted result.</summary>
public class DegenerateTests
{
    private static Vector2D V(double x, double y) => new(x, y);

    private static Region R(params Outline[] o) => Region.FromOutlines(o);

    private static void Check(Region r)
    {
        GeoOracle.ValidateRegion(r);
        GeoOracle.ValidateTriangulation(r, r.Triangulate());
        GeoOracle.ValidatePartition(r, r.PartitionConvex());
        GeoOracle.ValidatePartition(r, r.PartitionConvex(3));
    }

    [Fact]
    public void Collinear_edges_and_t_junctions()
    {
        // A plus-shaped union where edges overlap partially along shared lines, and a T-junction.
        Region r = R(RegionTests.Rect(0, 0, 6, 2), RegionTests.Rect(2, 0, 4, 6), RegionTests.Rect(1, 2, 3, 3));
        Assert.Equal(21.0, r.Area);
        Check(r);
        Region t = R(RegionTests.Rect(0, 0, 4, 2), RegionTests.Rect(1, 2, 3, 4));
        Assert.Equal(12.0, t.Area);
        Check(t);
    }

    [Fact]
    public void Holes_in_holes_and_nested_islands()
    {
        // Rings nested three deep within one outline: even-odd gives a frame and an island.
        var rings = new List<IReadOnlyList<Vector2D>>
        {
            new[] { V(0, 0), V(30, 0), V(30, 30), V(0, 30) },
            new[] { V(5, 5), V(25, 5), V(25, 25), V(5, 25) },
            new[] { V(10, 10), V(20, 10), V(20, 20), V(10, 20) },
        };
        Region r = R(new Outline(rings));
        Assert.Equal((900.0 - 400.0) + 100.0, r.Area);
        Assert.Equal(3, r.RingCount);
        Check(r);
        // A separate outline filling the hole's hole region merges by union.
        Region u = R(new Outline(rings), RegionTests.Rect(8, 8, 22, 22));
        Assert.Equal(900.0 - 36.0 * 0 - (400.0 - 196.0), u.Area);
        Check(u);
    }

    [Fact]
    public void Slivers_and_zero_area_input()
    {
        // A sliver much thinner than the grid unit collapses; a thin but representable one survives.
        Assert.True(R(new Outline(new[] { V(0, 0), V(100, 0), V(100, 1e-9) })).IsEmpty);
        Region thin = R(new Outline(new[] { V(0, 0), V(100, 0), V(100, 0.001) }));
        Assert.True(thin.Area > 0.04 && thin.Area < 0.06);
        Check(thin);
        Region both = R(new Outline(new[] { V(0, 0), V(100, 0), V(100, 0.001) }), new Outline(new[] { V(0, 0.0005), V(100, 0.0005), V(100, 0.0015) }));
        Check(both);
        // A ring that retraces itself has no area.
        Assert.True(R(new Outline(new[] { V(0, 0), V(5, 5), V(0, 0), V(5, 5) })).IsEmpty);
    }

    [Fact]
    public void Huge_and_tiny_at_the_bound()
    {
        double m = 2097152.0;
        Region big = R(RegionTests.Rect(-m, -m, m, m));
        Region speck = R(RegionTests.Rect(m - 0.0002, m - 0.0002, m, m));
        Region u = big.Union(speck);
        Assert.Equal(GeoOracle.ValidateRegion(big), GeoOracle.ValidateRegion(u));
        Region d = big.Difference(speck);
        // Exact accounting in integers: the notch is a 13x13 grid-unit square.
        Assert.Equal(GeoOracle.ValidateRegion(big) - GeoOracle.ValidateRegion(speck), GeoOracle.ValidateRegion(d));
        Assert.True(GeoOracle.ValidateRegion(speck) > 0);
        Check(d);
        Region x = big.SymmetricDifference(R(RegionTests.Rect(0, 0, 1e-3, 1e-3)));
        Check(x);
        Assert.Equal(2, x.RingCount);
    }

    [Fact]
    public void Opposite_direction_duplicate_edges_cancel()
    {
        // Two outlines sharing a whole edge with opposite directions: union has no internal edge.
        Region r = R(RegionTests.Rect(0, 0, 2, 2), RegionTests.Rect(2, 0, 4, 2, cw: true));
        Assert.Equal(1, r.RingCount);
        Assert.Equal(8.0, r.Area);
        Check(r);
    }

    [Fact]
    public void Crossing_exactly_on_a_pixel_corner_converges()
    {
        // Found by fuzzing: two unit diagonals cross on the corner shared by four snap pixels; the crossing belongs to
        // exactly one of them (the pixels are half-open), otherwise the repair loop would never settle.
        double u = 1.0 / 65536;
        Region r = R(new Outline(new[] { V(0, 0), V(u, u), V(u, 0), V(0, u) }));
        Check(r);
        Region s = R(new Outline(new[] { V(100, 100), V(100 + (3 * u), 100 + (3 * u)), V(100 + (3 * u), 100), V(100, 100 + (3 * u)) }));
        Check(s);
    }

    [Fact]
    public void Many_edges_through_one_point()
    {
        // A star of long thin triangles all passing through the centre.
        var outlines = new List<Outline>();
        for (int i = 0; i < 12; i++)
        {
            double a = i * Math.PI / 12;
            double dx = Math.Round(Math.Cos(a) * 1000) / 4, dy = Math.Round(Math.Sin(a) * 1000) / 4;
            outlines.Add(new Outline(new[] { V(-dx, -dy), V(dx + 1, dy), V(dx - 1, dy + 1) }));
        }
        Region r = Region.FromOutlines(outlines);
        Check(r);
    }

    [Fact]
    public void Tilemap_matches_cells_exactly_on_random_maps()
    {
        var rng = new Random(5);
        for (int it = 0; it < 200; it++)
        {
            int w = 1 + rng.Next(9), h = 1 + rng.Next(9);
            var cells = new bool[w * h];
            for (int i = 0; i < cells.Length; i++) { cells[i] = rng.Next(100) < 55; }
            double cs = 0.3 + (rng.Next(5) * 0.1);
            Region r = Region.FromTileMap(cells, w, h, cs, V(-3.25, 7.5));
            int count = 0;
            foreach (bool c in cells) { if (c) { count++; } }
            Assert.Equal(count * cs * cs, r.Area, 2);
            Check(r);
            for (int s = 0; s < 40; s++)
            {
                double px = -3.25 + (rng.NextDouble() * w * cs), py = 7.5 + (rng.NextDouble() * h * cs);
                int cx = (int)Math.Floor((px + 3.25) / cs), cy = (int)Math.Floor((py - 7.5) / cs);
                bool want = cx >= 0 && cy >= 0 && cx < w && cy < h && cells[(cy * w) + cx];
                double fx = ((px + 3.25) / cs) - cx, fy = ((py - 7.5) / cs) - cy;
                if (Math.Min(Math.Min(fx, 1 - fx), Math.Min(fy, 1 - fy)) < 1e-3) { continue; }
                Assert.Equal(want, GeoOracle.InRegion(r, px, py));
            }
        }
    }

    [Fact]
    public void Partition_respects_vertex_limit_and_merges_a_rectangle()
    {
        Region r = R(RegionTests.Rect(0, 0, 10, 4));
        ConvexPartition p = r.PartitionConvex(4);
        Assert.Equal(1, p.PolygonCount);
        Region l = R(new Outline(new[] { V(0, 0), V(6, 0), V(6, 2), V(2, 2), V(2, 6), V(0, 6) }));
        ConvexPartition lp = l.PartitionConvex(8);
        Assert.Equal(2, lp.PolygonCount);
        for (int i = 0; i < lp.PolygonCount; i++) { Assert.True(lp.GetIndices(i).Length <= 8); }
        Assert.Throws<ArgumentOutOfRangeException>(() => l.PartitionConvex(2));
        Assert.Throws<ArgumentOutOfRangeException>(() => l.PartitionConvex(17));
        Check(l);
    }

    [Fact]
    public void Empty_results_are_valid()
    {
        Region a = R(RegionTests.Rect(0, 0, 1, 1)), b = R(RegionTests.Rect(5, 5, 6, 6));
        Region i = a.Intersection(b);
        Assert.True(i.IsEmpty);
        Assert.Equal(0, i.Triangulate().TriangleCount);
        Assert.Equal(0, i.PartitionConvex().PolygonCount);
        Assert.Equal(0.0, i.Area);
        Assert.True(a.Difference(a).IsEmpty);
        Assert.True(Region.Empty().Offset(1).IsEmpty);
    }
}
