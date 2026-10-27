using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build2D;

public class RegionTests
{
    internal static Vector2D V(double x, double y) => new(x, y);

    internal static Outline Rect(double x0, double y0, double x1, double y1, bool cw = false) =>
        new(cw ? new[] { V(x0, y0), V(x0, y1), V(x1, y1), V(x1, y0) } : new[] { V(x0, y0), V(x1, y0), V(x1, y1), V(x0, y1) });

    private static Region R(params Outline[] o) => Region.FromOutlines(o);

    private static void Check(Region r)
    {
        GeoOracle.ValidateRegion(r);
        GeoOracle.ValidateTriangulation(r, r.Triangulate());
        GeoOracle.ValidatePartition(r, r.PartitionConvex());
    }

    [Fact]
    public void Square_area_and_orientation()
    {
        Region r = R(Rect(0, 0, 4, 3));
        Assert.Equal(12.0, r.Area);
        Assert.Equal(1, r.RingCount);
        Region cw = R(Rect(0, 0, 4, 3, cw: true));
        Assert.Equal(12.0, cw.Area);
        Check(r);
        Assert.Equal(1, r.PartitionConvex().PolygonCount);
    }

    [Fact]
    public void Hole_by_even_odd_in_any_orientation()
    {
        var outer = new[] { V(0, 0), V(10, 0), V(10, 10), V(0, 10) };
        var hole = new[] { V(3, 3), V(3, 7), V(7, 7), V(7, 3) };
        Region r = Region.FromOutlines(new[] { new Outline(outer, new IReadOnlyList<Vector2D>[] { hole }) });
        Assert.Equal(84.0, r.Area);
        Assert.Equal(2, r.RingCount);
        Check(r);
        var hole2 = new[] { V(3, 3), V(7, 3), V(7, 7), V(3, 7) };
        Assert.Equal(84.0, Region.FromOutlines(new[] { new Outline(outer, new IReadOnlyList<Vector2D>[] { hole2 }) }).Area);
    }

    [Fact]
    public void Overlapping_outlines_union_and_ops()
    {
        Region a = R(Rect(0, 0, 4, 4));
        Region b = R(Rect(2, 2, 6, 6));
        Assert.Equal(28.0, a.Union(b).Area);
        Assert.Equal(4.0, a.Intersection(b).Area);
        Assert.Equal(12.0, a.Difference(b).Area);
        Assert.Equal(24.0, a.SymmetricDifference(b).Area);
        Check(a.Union(b));
        Check(a.Difference(b));
        Check(a.SymmetricDifference(b));
        Assert.Equal(28.0, Region.FromOutlines(new[] { Rect(0, 0, 4, 4), Rect(2, 2, 6, 6) }).Area);
    }

    [Fact]
    public void Touching_and_identical_and_nested()
    {
        Assert.Equal(32.0, R(Rect(0, 0, 4, 4), Rect(4, 0, 8, 4)).Area); // shared edge
        Assert.Equal(1, R(Rect(0, 0, 4, 4), Rect(4, 0, 8, 4)).RingCount);
        Assert.Equal(16.0, R(Rect(0, 0, 4, 4), Rect(0, 0, 4, 4)).Area);
        Assert.Equal(16.0, R(Rect(0, 0, 4, 4), Rect(1, 1, 2, 2)).Area); // nested union
        Region diamondTouch = R(Rect(0, 0, 2, 2), Rect(2, 2, 4, 4)); // corner contact
        Assert.Equal(8.0, diamondTouch.Area);
        Check(diamondTouch);
    }

    [Fact]
    public void Bowtie_self_intersection_is_cleaned()
    {
        var bow = new Outline(new[] { V(0, 0), V(4, 4), V(4, 0), V(0, 4) });
        Region r = Region.FromOutlines(new[] { bow });
        Assert.Equal(8.0, r.Area);
        Check(r);
    }

    [Fact]
    public void Zero_area_and_duplicate_points_are_harmless()
    {
        Assert.True(R(new Outline(new[] { V(0, 0), V(1, 1), V(2, 2) })).IsEmpty);
        Assert.True(R(new Outline(new[] { V(0, 0), V(0, 0), V(0, 0) })).IsEmpty);
        Region r = R(new Outline(new[] { V(0, 0), V(4, 0), V(4, 0), V(4, 4), V(2, 4), V(0, 4), V(0, 4.0000000000001) }));
        Assert.Equal(16.0, r.Area, 6);
    }

    [Fact]
    public void Hostile_input_throws_typed_exceptions()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => R(new Outline(new[] { V(0, 0), V(double.NaN, 0), V(1, 1) })));
        Assert.Throws<ArgumentOutOfRangeException>(() => R(new Outline(new[] { V(0, 0), V(double.PositiveInfinity, 0), V(1, 1) })));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => R(new Outline(new[] { V(0, 0), V(2097152.5, 0), V(1, 1) })));
        Assert.Contains("vertex 1", ex.Message);
        Assert.Throws<ArgumentException>(() => new Outline(new[] { V(0, 0), V(1, 1) }));
        Assert.Throws<ArgumentException>(() => new Outline(new List<IReadOnlyList<Vector2D>>()));
        Assert.Throws<ArgumentException>(() => Region.FromOutlines(new List<Outline>()));
        Assert.Throws<ArgumentNullException>(() => Region.FromOutlines(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Geometry2DOptions { SnapBits = 17 });
        Assert.Throws<ArgumentException>(() => R(Rect(0, 0, 1, 1)).Union(Region.FromOutlines(new[] { Rect(0, 0, 1, 1) }, new Geometry2DOptions { SnapBits = 10 })));
    }

    [Fact]
    public void Coordinates_at_the_bound_work()
    {
        double m = 2097152.0;
        Region r = R(Rect(-m, -m, m, m), Rect(m - 1, m - 1, m, m));
        Assert.Equal(4 * m * m, r.Area);
        Region tiny = R(Rect(m - 0.001, m - 0.001, m, m));
        Assert.True(tiny.Area > 0);
        Check(tiny);
    }

    [Fact]
    public void One_ulp_apart_points_snap_together()
    {
        double e = Math.BitIncrement(1.0);
        Region r = R(new Outline(new[] { V(0, 0), V(1, 0), V(e, 0), V(1, 1), V(0, 1) }));
        Assert.Equal(1.0, r.Area);
        Check(r);
    }

    [Fact]
    public void TileMap_merges_cells()
    {
        // 4x3 map with a hole at (1,1) and a diagonal-only contact at the top right.
        bool[] cells =
        {
            true, true, true, false,
            true, false, true, true,
            true, true, true, true,
        };
        Region r = Region.FromTileMap(cells, 4, 3, 2.0, V(10, 20));
        Assert.Equal(40.0, r.Area);
        Check(r);
        Assert.Equal(2, r.RingCount);
        // Diagonal contact only.
        bool[] diag = { true, false, false, true };
        Region d = Region.FromTileMap(diag, 2, 2, 1.0, V(0, 0));
        Assert.Equal(2.0, d.Area);
        Check(d);
        Assert.Throws<ArgumentException>(() => Region.FromTileMap(cells, 5, 3, 1, V(0, 0)));
    }

    [Fact]
    public void Offset_shrinks_and_grows_a_square()
    {
        Region sq = R(Rect(0, 0, 10, 10));
        Region shrunk = sq.Offset(-1, JoinStyle.Square);
        Assert.Equal(64.0, shrunk.Area, 3);
        Check(shrunk);
        Region round = sq.Offset(-1);
        Assert.Equal(64.0, round.Area, 3);
        Region grown = sq.Offset(1);
        // 100 + 4*10*1 + disc area pi at the corners (circumscribed polygon is slightly larger).
        Assert.InRange(grown.Area, 100 + 40 + Math.PI - 0.01, 100 + 40 + Math.PI + 0.2);
        Check(grown);
        Assert.Equal(144.0, sq.Offset(1, JoinStyle.Square).Area, 3);
        Assert.True(sq.Offset(-5).IsEmpty || sq.Offset(-5).Area < 0.5);
        Assert.True(sq.Offset(-6).IsEmpty);
    }

    [Fact]
    public void Offset_closes_narrow_gaps_and_opens_around_holes()
    {
        // Two blocks 1.5 apart: growing by 1 merges them; shrinking a corridor of width 1.5 by 1 removes it.
        Region two = R(Rect(0, 0, 4, 4), Rect(5.5, 0, 9.5, 4));
        Assert.Equal(1, two.Offset(1, JoinStyle.Square).RingCount);
        Region corridor = R(Rect(0, 0, 10, 1.5));
        Assert.True(corridor.Offset(-1, JoinStyle.Square).IsEmpty);
        // A hole grows when the region shrinks.
        Region ring = Region.FromOutlines(new[] { new Outline(new[] { V(0, 0), V(20, 0), V(20, 20), V(0, 20) }, new IReadOnlyList<Vector2D>[] { new[] { V(8, 8), V(12, 8), V(12, 12), V(8, 12) } }) });
        Region s = ring.Offset(-1, JoinStyle.Square);
        Assert.Equal(18.0 * 18.0 - 6.0 * 6.0, s.Area, 3);
        Check(s);
    }

    [Fact]
    public void Output_is_independent_of_input_order_and_start_vertex()
    {
        var rng = new Random(11);
        for (int it = 0; it < 60; it++)
        {
            var outlines = RandomOutlines(rng, 1 + rng.Next(3));
            Region a = Region.FromOutlines(outlines);
            var shuffled = new List<Outline>();
            foreach (Outline o in outlines)
            {
                var rings = new List<IReadOnlyList<Vector2D>>();
                for (int i = 0; i < o.RingCount; i++)
                {
                    Vector2D[] ring = o.GetRing(i);
                    int s = rng.Next(ring.Length);
                    var rot = new Vector2D[ring.Length];
                    for (int k = 0; k < ring.Length; k++) { rot[k] = ring[(s + k) % ring.Length]; }
                    if (rng.Next(2) == 0) { Array.Reverse(rot); }
                    rings.Add(rot);
                }
                for (int i = rings.Count - 1; i > 0; i--) { int j = rng.Next(i + 1); (rings[i], rings[j]) = (rings[j], rings[i]); }
                shuffled.Add(new Outline(rings));
            }
            for (int i = shuffled.Count - 1; i > 0; i--) { int j = rng.Next(i + 1); (shuffled[i], shuffled[j]) = (shuffled[j], shuffled[i]); }
            Region b = Region.FromOutlines(shuffled);
            Assert.Equal(Describe(a), Describe(b));
            Assert.Equal(Describe(a.Triangulate()), Describe(b.Triangulate()));
            Assert.Equal(Describe(a.PartitionConvex()), Describe(b.PartitionConvex()));
        }
    }

    internal static string Describe(Region r)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < r.RingCount; i++)
        {
            foreach (Vector2D p in r.GetRing(i)) { sb.Append(p.X).Append(',').Append(p.Y).Append(';'); }
            sb.Append('|');
        }
        return sb.ToString();
    }

    internal static string Describe(Triangulation t) => string.Join(",", t.Indices.ToArray()) + "/" + string.Join(",", t.Neighbors.ToArray());

    internal static string Describe(ConvexPartition c)
    {
        var sb = new System.Text.StringBuilder();
        for (int p = 0; p < c.PolygonCount; p++)
        {
            sb.Append(string.Join(",", c.GetIndices(p).ToArray())).Append('/').Append(string.Join(",", c.GetNeighbors(p).ToArray())).Append('|');
        }
        return sb.ToString();
    }

    internal static List<Outline> RandomOutlines(Random rng, int count, int extent = 8, int denom = 8, double ox = 0, double oy = 0, bool nudge = false)
    {
        var list = new List<Outline>();
        for (int o = 0; o < count; o++)
        {
            int rings = 1 + (rng.Next(3) == 0 ? 1 : 0);
            var rs = new List<IReadOnlyList<Vector2D>>();
            for (int r = 0; r < rings; r++)
            {
                int n = 3 + rng.Next(5);
                var pts = new Vector2D[n];
                for (int i = 0; i < n; i++)
                {
                    double x = ox + (rng.Next(extent * denom + 1) / (double)denom), y = oy + (rng.Next(extent * denom + 1) / (double)denom);
                    if (nudge && rng.Next(4) == 0)
                    {
                        x = rng.Next(2) == 0 ? Math.BitIncrement(x) : Math.BitDecrement(x);
                        y = rng.Next(2) == 0 ? Math.BitIncrement(y) : Math.BitDecrement(y);
                    }
                    pts[i] = V(x, y);
                }
                rs.Add(pts);
            }
            list.Add(new Outline(rs));
        }
        return list;
    }
}
