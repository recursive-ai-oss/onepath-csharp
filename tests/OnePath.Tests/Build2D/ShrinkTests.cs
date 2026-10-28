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
}
