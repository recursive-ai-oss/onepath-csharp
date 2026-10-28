using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

/// <summary>The build pipeline's shrink builds fewer pieces than <see cref="Region.Offset"/>; it must still remove exactly what the offset removes.</summary>
public class ShrinkTests
{
    [Fact]
    public void Shrink_matches_distance_to_boundary()
    {
        for (long i = 0; i < 300; i++)
        {
            var rng = new Random(unchecked((int)(9_100_001L + i)));
            Region r = Region.FromOutlines(RegionTests.RandomOutlines(rng, 1 + rng.Next(3), 8, 4));
            double d = (rng.Next(6) + 1) * 0.25;
            Region shrunk = Mesh2DPipeline.Shrink(r, d);
            GeoOracle.ValidateRegion(shrunk);
            GeoOracle.ValidateTriangulation(shrunk, shrunk.Triangulate());
            var rings = new List<Vector2D[]>();
            for (int k = 0; k < r.RingCount; k++) { rings.Add(r.GetRing(k)); }
            // Arcs stay within a sixteenth of the radius of the true circle.
            double slack = 0.02 + (d / 16) + (d / 64);
            for (int s = 0; s < 40; s++)
            {
                double px = rng.NextDouble() * 8, py = rng.NextDouble() * 8;
                double dist = GeoOracle.MinEdgeDistance(rings, px, py);
                bool inside = GeoOracle.InRegion(r, px, py);
                bool got = GeoOracle.InRegion(shrunk, px, py);
                if (inside && dist > d + slack) { Assert.True(got, $"case {i}: deep inside point lost"); }
                if (inside && dist < d - slack) { Assert.False(got, $"case {i}: near-boundary point kept"); }
                if (!inside) { Assert.False(got, $"case {i}: outside point appeared"); }
            }
        }
    }

    [Fact]
    public void Shrink_of_a_tile_map_has_the_area_of_the_exact_offset()
    {
        var rng = new Random(5);
        var cells = new bool[40 * 40];
        for (int i = 0; i < cells.Length; i++) { cells[i] = rng.Next(100) < 60; }
        Region map = Region.FromTileMap(cells, 40, 40, 2.0, new Vector2D(0, 0));
        Region exact = map.Offset(-0.6);
        Region fast = Mesh2DPipeline.Shrink(map, 0.6);
        Assert.Equal(exact.RingCount, fast.RingCount);
        Assert.InRange(fast.Area / exact.Area, 0.99, 1.01);
        // Whatever the exact offset keeps and the shrink drops (or the reverse) is a thin rim along the arcs.
        double rim = exact.SymmetricDifference(fast).Area;
        Assert.True(rim < 0.01 * exact.Area, $"rim {rim} of {exact.Area}");
    }

    [Fact]
    public void Shrink_by_zero_returns_the_region_and_thin_parts_vanish()
    {
        Region sq = Region.FromOutlines(new[] { new Outline(new[] { new Vector2D(0, 0), new Vector2D(2, 0), new Vector2D(2, 2), new Vector2D(0, 2) }) });
        Assert.Same(sq, Mesh2DPipeline.Shrink(sq, 0));
        Assert.True(Mesh2DPipeline.Shrink(sq, 1.5).IsEmpty);
        Region small = Mesh2DPipeline.Shrink(sq, 0.5);
        Assert.InRange(small.Area, 0.98, 1.0);
    }

    // A 3x3-tile room (2 units per tile) with a pillar tile in the middle, on an 8x8 map. The exact eroded area is
    // 8 tiles minus, per tile, 2 r per wall side, plus r^2 per wall corner pair, plus a quarter disc per inside corner.
    private static Region PillarRoom()
    {
        var cells = new bool[64];
        for (int j = 1; j <= 3; j++) { for (int i = 1; i <= 3; i++) { cells[(j * 8) + i] = !(i == 2 && j == 2); } }
        return Region.FromTileMap(cells, 8, 8, 2.0, new Vector2D(0, 0));
    }

    // The circumscribed arc polygon used to take the whole excess of its tangent polygon off every round corner: with
    // 16 sides that was 0.0103 per quarter disc, 4.7% of the four slivers left at radius 1 (pillar tile cusps), and an
    // 8-gon at radius 0.75 and tolerance 0.0625 lost 1.1% of the room. The arc polygon now has a quarter disc's area.
    [Theory]
    [InlineData(0.75, 0.0625, 8.482854132355742)]
    [InlineData(0.75, 0.03125, 8.482854132355742)]
    [InlineData(1.0, 0.0625, 0.8584073464102069)]
    [InlineData(1.0, 0.03125, 0.8584073464102069)]
    public void Shrinking_round_corners_keeps_the_exact_eroded_area(double radius, double tolerance, double exact)
    {
        Region shrunk = Mesh2DPipeline.Shrink(PillarRoom(), radius, tolerance);
        GeoOracle.ValidateRegion(shrunk);
        Assert.InRange(shrunk.Area / exact, 0.995, 1.005);
    }

    // An L of five tiles at radius 1: the reflex corner leaves a 1x1 square minus a quarter disc, with a cusp at each end.
    [Fact]
    public void Shrinking_leaves_the_cusp_between_a_quarter_disc_and_two_walls()
    {
        var cells = new bool[64];
        foreach ((int i, int j) in new[] { (1, 1), (1, 2), (1, 3), (2, 3), (3, 3) }) { cells[(j * 8) + i] = true; }
        Region shrunk = Mesh2DPipeline.Shrink(Region.FromTileMap(cells, 8, 8, 2.0, new Vector2D(0, 0)), 1.0, 0.0625);
        GeoOracle.ValidateRegion(shrunk);
        Assert.InRange(shrunk.Area / (1 - (Math.PI / 4)), 0.99, 1.01);
    }
}
