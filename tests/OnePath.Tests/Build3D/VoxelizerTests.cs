using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.NavMesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

public class VoxelizerTests
{
    // Independent oracle: the footprint of triangle ∩ closed cell prism is a convex polygon whose vertices are triangle
    // vertices inside the cell, triangle edges crossing the cell's sides, and cell corners lying over the triangle.
    private static (bool Occupied, int Min, int Max, bool Fuzzy) CellOracle(double[][] t, int i, int j, int height)
    {
        const double Eps = 1e-9;
        var pts = new List<(double U, double V, double W)>();
        bool In(double u, double w) => u >= i - Eps && u <= i + 1 + Eps && w >= j - Eps && w <= j + 1 + Eps;
        foreach (double[] p in t)
        {
            if (In(p[0], p[2]))
            {
                pts.Add((p[0], p[1], p[2]));
            }
        }
        for (int e = 0; e < 3; e++)
        {
            double[] p = t[e];
            double[] q = t[(e + 1) % 3];
            for (int axis = 0; axis < 2; axis++)
            {
                int a = axis == 0 ? 0 : 2;
                foreach (double c in new double[] { axis == 0 ? i : j, (axis == 0 ? i : j) + 1 })
                {
                    double dp = p[a] - c;
                    double dq = q[a] - c;
                    if (dp * dq <= 0 && dp != dq)
                    {
                        double s = dp / (dp - dq);
                        double u = p[0] + (s * (q[0] - p[0]));
                        double v = p[1] + (s * (q[1] - p[1]));
                        double w = p[2] + (s * (q[2] - p[2]));
                        if (In(u, w))
                        {
                            pts.Add((u, v, w));
                        }
                    }
                }
            }
        }
        double det = ((t[1][0] - t[0][0]) * (t[2][2] - t[0][2])) - ((t[2][0] - t[0][0]) * (t[1][2] - t[0][2]));
        if (det != 0)
        {
            foreach (int cu in new[] { i, i + 1 })
            {
                foreach (int cw in new[] { j, j + 1 })
                {
                    double l1 = (((cu - t[0][0]) * (t[2][2] - t[0][2])) - ((t[2][0] - t[0][0]) * (cw - t[0][2]))) / det;
                    double l2 = (((t[1][0] - t[0][0]) * (cw - t[0][2])) - ((cu - t[0][0]) * (t[1][2] - t[0][2]))) / det;
                    if (l1 >= -Eps && l2 >= -Eps && l1 + l2 <= 1 + Eps)
                    {
                        pts.Add((cu, t[0][1] + (l1 * (t[1][1] - t[0][1])) + (l2 * (t[2][1] - t[0][1])), cw));
                    }
                }
            }
        }
        if (pts.Count < 3)
        {
            return (false, 0, 0, false);
        }
        double area = HullArea(pts);
        if (area < 1e-12)
        {
            return (false, 0, 0, area > 0 || pts.Count > 0 && area > -1);
        }
        double vmin = double.MaxValue, vmax = double.MinValue;
        foreach (var p in pts)
        {
            vmin = Math.Min(vmin, p.V);
            vmax = Math.Max(vmax, p.V);
        }
        bool fuzzy = area < 1e-3 || Near(vmin) || Near(vmax) || vmax - vmin < 1e-6;
        int lo = (int)Math.Floor(vmin);
        int hi = (int)Math.Ceiling(vmax);
        if (hi <= 0 || lo >= height)
        {
            return (false, 0, 0, fuzzy);
        }
        return (true, Math.Max(lo, 0), Math.Min(hi, height), fuzzy);
    }

    private static bool Near(double v) => Math.Abs(v - Math.Round(v)) < 1e-3;

    private static double HullArea(List<(double U, double V, double W)> pts)
    {
        var p = new List<(double X, double Y)>();
        foreach (var q in pts)
        {
            p.Add((q.U, q.W));
        }
        p.Sort((a, b) => a.X != b.X ? a.X.CompareTo(b.X) : a.Y.CompareTo(b.Y));
        var h = new List<(double X, double Y)>();
        for (int pass = 0; pass < 2; pass++)
        {
            int start = h.Count;
            for (int k = 0; k < p.Count; k++)
            {
                var c = p[pass == 0 ? k : p.Count - 1 - k];
                while (h.Count >= start + 2 && Cross(h[^2], h[^1], c) <= 0)
                {
                    h.RemoveAt(h.Count - 1);
                }
                h.Add(c);
            }
            h.RemoveAt(h.Count - 1);
        }
        double a = 0;
        for (int k = 0; k < h.Count; k++)
        {
            var u = h[k];
            var v = h[(k + 1) % h.Count];
            a += (u.X * v.Y) - (v.X * u.Y);
        }
        return Math.Abs(a) / 2;
    }

    private static double Cross((double X, double Y) o, (double X, double Y) a, (double X, double Y) b) =>
        ((a.X - o.X) * (b.Y - o.Y)) - ((a.Y - o.Y) * (b.X - o.X));

    [Fact]
    public void RandomSoupsMatchGeometricOracle()
    {
        var rng = new TestRng(11);
        float[] sizes = { 1f, 0.5f, 0.25f };
        int soups = 12000;
        int cellsChecked = 0;
        int fuzzySkipped = 0;
        for (int n = 0; n < soups; n++)
        {
            float cs = sizes[rng.Int(3)];
            float ch = sizes[rng.Int(3)];
            int w = 3 + rng.Int(4), d = 3 + rng.Int(4), h = 3 + rng.Int(6);
            double ox = Math.Round(rng.Range(-50, 50), 2), oy = Math.Round(rng.Range(-50, 50), 2), oz = Math.Round(rng.Range(-50, 50), 2);
            var frame = new HeightfieldFrame(ox, oy, oz, cs, ch, w, d, h);
            int tris = 1 + rng.Int(3);
            var verts = new double[tris * 9];
            var idx = new int[tris * 3];
            var uvw = new double[tris][][];
            for (int t = 0; t < tris; t++)
            {
                uvw[t] = new double[3][];
                for (int k = 0; k < 3; k++)
                {
                    float lx = (float)rng.Range(-1, w + 1) * cs;
                    float ly = (float)rng.Range(-1, h + 1) * ch;
                    float lz = (float)rng.Range(-1, d + 1) * cs;
                    double x = ox + lx, y = oy + ly, z = oz + lz;
                    verts[(t * 9) + (k * 3)] = x;
                    verts[(t * 9) + (k * 3) + 1] = y;
                    verts[(t * 9) + (k * 3) + 2] = z;
                    idx[(t * 3) + k] = (t * 3) + k;
                    uvw[t][k] = new double[] { (float)(x - ox) / cs, (float)(y - oy) / ch, (float)(z - oz) / cs };
                }
            }
            var hf = new SolidHeightfield(frame);
            var areas = new byte[tris];
            Voxelizer.Rasterize(hf, verts, idx, areas, 0);
            var cols = Dump.Columns(hf);
            for (int j = 0; j < d; j++)
            {
                for (int i = 0; i < w; i++)
                {
                    var expect = new bool[h];
                    bool fuzzy = false;
                    for (int t = 0; t < tris; t++)
                    {
                        var o = CellOracle(uvw[t], i, j, h);
                        fuzzy |= o.Fuzzy;
                        if (o.Occupied)
                        {
                            for (int y = o.Min; y < o.Max; y++)
                            {
                                expect[y] = true;
                            }
                        }
                    }
                    if (fuzzy)
                    {
                        fuzzySkipped++;
                        continue;
                    }
                    var actual = new bool[h];
                    foreach (var (a, b, _) in cols[frame.Column(i, j)])
                    {
                        for (int y = a; y < b; y++)
                        {
                            actual[y] = true;
                        }
                    }
                    cellsChecked++;
                    Assert.True(expect.AsSpan().SequenceEqual(actual), $"soup {n} cell ({i},{j}): expected {string.Join("", expect)} got {string.Join("", actual)}");
                }
            }
        }
        Assert.True(cellsChecked > 100000, $"only {cellsChecked} cells checked ({fuzzySkipped} skipped)");
        Assert.True(fuzzySkipped * 20 < cellsChecked, $"{fuzzySkipped} cells skipped as near-ties");
    }

    private static readonly int[] Quad = { 0, 1, 2, 0, 2, 3 };
    private static readonly int[] Tri = { 0, 1, 2 };

    private static HeightfieldFrame Frame(int w, int d, int h, float cs = 1f, float ch = 1f) => new(0, 0, 0, cs, ch, w, d, h);

    private static SolidHeightfield Raster(HeightfieldFrame f, double[] v, int[] i, byte area = 63)
    {
        var hf = new SolidHeightfield(f);
        var areas = new byte[i.Length / 3];
        Array.Fill(areas, area);
        Voxelizer.Rasterize(hf, v, i, areas, 0);
        return hf;
    }

    [Fact]
    public void FlatFloorOnLatticeOccupiesRowBelowItsSurface()
    {
        // Quad covering cells 0..3 x 0..3 at y = 2, facing up: spans [1,2).
        double[] v = { 0, 2, 0, 0, 2, 4, 4, 2, 4, 4, 2, 0 };
        var hf = Raster(Frame(4, 4, 5), v, Quad);
        var cols = Dump.Columns(hf);
        foreach (var c in cols)
        {
            Dump.Expect(c, (1, 2, 63));
        }
    }

    [Fact]
    public void FlatCeilingOnLatticeOccupiesRowAboveItsSurface()
    {
        double[] v = { 0, 2, 0, 4, 2, 0, 4, 2, 4, 0, 2, 4 }; // wound to face down
        var hf = Raster(Frame(4, 4, 5), v, Quad, 0);
        foreach (var c in Dump.Columns(hf))
        {
            Dump.Expect(c, (2, 3, 0));
        }
    }

    [Fact]
    public void TriangleTouchingOnlyACornerOrEdgeOfACellDoesNotMarkIt()
    {
        // Right triangle (0,0)-(2,0)-(0,2) in u/w at y 0.5: the cell (1,1) is touched only at the corner (1,1)... actually the hypotenuse passes through it.
        // Cell (1,1) = [1,2]x[1,2]: the hypotenuse u+w=2 touches it only at the point (1,1). It must stay empty.
        double[] v = { 0, 0.5, 0, 0, 0.5, 2, 2, 0.5, 0 };
        var hf = Raster(Frame(3, 3, 2), v, Tri);
        var cols = Dump.Columns(hf);
        Assert.Empty(cols[1 + (1 * 3)]);
        Assert.Single(cols[0]);
        Assert.Single(cols[1]);
        Assert.Single(cols[3]);
        Assert.Empty(cols[2]);
    }

    [Fact]
    public void VerticalWallOnGridLineBelongsToTheCellBehindIt()
    {
        // Wall in the plane x = 2 from z 0..2, y 0..3. Facing +x (counter-clockwise seen from +x): belongs to column x=1.
        double[] v = { 2, 0, 0, 2, 3, 0, 2, 3, 2, 2, 0, 2 };
        // Seen from +x, with z to the left and y up: (z0,y0)->(z2,y0)... choose winding by computed normal instead.
        int[] idx = { 0, 3, 2, 0, 2, 1 };
        var hfPlus = Raster(Frame(4, 2, 4), v, idx, 0);
        int[] rev = { 0, 2, 3, 0, 1, 2 };
        var hfMinus = Raster(Frame(4, 2, 4), v, rev, 0);
        var a = Dump.Columns(hfPlus);
        var b = Dump.Columns(hfMinus);
        // The two windings face opposite ways: one lands in x=1 and the other in x=2.
        int nPlus = 0, nMinus = 0;
        for (int z = 0; z < 2; z++)
        {
            nPlus += a[1 + (z * 4)].Count + a[2 + (z * 4)].Count;
            nMinus += b[1 + (z * 4)].Count + b[2 + (z * 4)].Count;
            Assert.Equal(1, a[1 + (z * 4)].Count + a[2 + (z * 4)].Count);
            Assert.Equal(1, b[1 + (z * 4)].Count + b[2 + (z * 4)].Count);
            Assert.NotEqual(a[1 + (z * 4)].Count, b[1 + (z * 4)].Count);
            var s = (a[1 + (z * 4)].Count == 1 ? a[1 + (z * 4)] : a[2 + (z * 4)])[0];
            Assert.Equal((0, 3), (s.Min, s.Max));
        }
        Assert.Equal(2, nPlus);
        Assert.Equal(2, nMinus);
        Assert.Empty(a[0]);
        Assert.Empty(a[3]);
    }

    [Fact]
    public void ObliqueVerticalWallMarksCellsAlongItsFootprint()
    {
        // Diagonal wall from (0.5,0.5) to (2.5,0.5+2) in x/z, height 0..2.
        double[] v = { 0.5, 0, 0.5, 0.5, 2, 0.5, 2.5, 2, 2.5, 2.5, 0, 2.5 };
        var hf = Raster(Frame(4, 4, 3), v, Quad, 0);
        var cols = Dump.Columns(hf);
        // Cells along the diagonal: (0,0), (1,1), (2,2); the corner-only contacts (1,0) (0,1) etc. stay empty.
        Assert.Single(cols[0]);
        Assert.Single(cols[5]);
        Assert.Single(cols[10]);
        Assert.Empty(cols[1]);
        Assert.Empty(cols[4]);
        Assert.Empty(cols[6]);
    }

    [Fact]
    public void SlopeClassification()
    {
        var f = Frame(4, 4, 4);
        // Upward triangle: walkable. Same triangle reversed: not. 45-degree-ish steep triangle: threshold decides.
        double[] v = { 0, 0, 0, 0, 0, 4, 4, 0, 0, 0, 3, 0, 0, 3, 4, 3, 0, 0 };
        var areas = new byte[3];
        Voxelizer.ClassifyTriangles(f, v, new[] { 0, 1, 2, 2, 1, 0, 3, 4, 5 }, 0.7072f, areas);
        Assert.Equal(new byte[] { 63, 0, 0 }, areas);
        // The 3,3 hypotenuse face has normal y = 3/sqrt(18) = 0.707 + ...: walkable only if the cosine is lower.
        Voxelizer.ClassifyTriangles(f, v, new[] { 3, 4, 5 }, 0.5f, areas.AsSpan(0, 1));
        Assert.Equal(63, areas[0]);
    }

    [Fact]
    public void TrianglesOutsideTheGridAndDegenerateOnesAreSkipped()
    {
        var f = Frame(3, 3, 3);
        double[] v = { 10, 1, 10, 10, 1, 12, 12, 1, 10, 0, 0, 0, 1, 1, 1, 2, 2, 2, 0, 1, 0, 0, 1, 2, 2, 1, 0 };
        var hf = new SolidHeightfield(f);
        var st = Voxelizer.Rasterize(hf, v, new[] { 0, 1, 2, 3, 4, 5, 6, 7, 8 }, SpanSettings.Humanoid);
        Assert.Equal(3, st.Triangles);
        Assert.Equal(1, st.Degenerate);
        Assert.Equal(1, st.Touching);
    }

    [Fact]
    public void HostileInputThrowsTypedErrors()
    {
        var f = Frame(3, 3, 3);
        var s = SpanSettings.Humanoid;
        SolidHeightfield New() => new(f);
        double[] ok = { 0, 0, 0, 1, 0, 0, 0, 0, 1 };
        Assert.Throws<ArgumentException>(() => Voxelizer.Rasterize(New(), new[] { 0, 0, double.NaN, 1, 0, 0, 0, 0, 1 }, new[] { 0, 1, 2 }, s));
        Assert.Throws<ArgumentException>(() => Voxelizer.Rasterize(New(), new[] { 0, 0, double.PositiveInfinity, 1, 0, 0, 0, 0, 1 }, new[] { 0, 1, 2 }, s));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => Voxelizer.Rasterize(New(), new[] { 0, 0, 3e6, 1, 0, 0, 0, 0, 1 }, new[] { 0, 1, 2 }, s));
        Assert.Contains("3000000", ex.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => Voxelizer.Rasterize(New(), ok, new[] { 0, 1, 3 }, s));
        Assert.Throws<ArgumentOutOfRangeException>(() => Voxelizer.Rasterize(New(), ok, new[] { 0, 1, -1 }, s));
        Assert.Throws<ArgumentException>(() => Voxelizer.Rasterize(New(), ok, new[] { 0, 1 }, s));
        Assert.Throws<ArgumentException>(() => Voxelizer.Rasterize(New(), new double[4], new[] { 0, 0, 0 }, s));
        // Empty input is fine.
        var st = Voxelizer.Rasterize(New(), ReadOnlySpan<double>.Empty, ReadOnlySpan<int>.Empty, s);
        Assert.Equal(0, st.Triangles);
        // Frames.
        Assert.Throws<ArgumentException>(() => HeightfieldFrame.FromBounds(1, 0, 0, 0, 1, 1, 0.3f, 0.2f));
        Assert.Throws<ArgumentException>(() => HeightfieldFrame.FromBounds(0, 0, 0, 0, 1, 1, 0.3f, 0.2f));
        Assert.Throws<ArgumentException>(() => HeightfieldFrame.FromBounds(double.NaN, 0, 0, 1, 1, 1, 0.3f, 0.2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => HeightfieldFrame.FromBounds(0, 0, 0, 3e6, 1, 1, 0.3f, 0.2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => HeightfieldFrame.FromBounds(0, 0, 0, 100000, 1, 100, 0.01f, 0.2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => HeightfieldFrame.FromBounds(0, 0, 0, 1, 1, 1, 0f, 0.2f));
        Assert.Throws<ArgumentOutOfRangeException>(() => HeightfieldFrame.FromBounds(0, 0, 0, 1, 1, 1, 0.3f, float.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightfieldFrame(0, 0, 0, 1e-30f, 1f, 2, 2, 2));
        Assert.Throws<ArgumentOutOfRangeException>(() => new HeightfieldFrame(0, 0, 0, 1f, float.PositiveInfinity, 2, 2, 2));
        // Giant and near-boundary triangles stay finite.
        double big = 2097152.0;
        var hf = New();
        Voxelizer.Rasterize(hf, new[] { -big, 0, -big, -big, 0, big, big, 0, -big }, new[] { 0, 1, 2 }, s);
        Voxelizer.Rasterize(hf, new[] { -big, -big, -big, big, big, big, 0, 1e-30, 0 }, new[] { 0, 1, 2 }, s);
    }

    [Fact]
    public void HeightGridRasterizesLikeItsTriangles()
    {
        var f = Frame(4, 4, 6);
        double[] hts = new double[25];
        for (int i = 0; i < hts.Length; i++)
        {
            hts[i] = 1.5;
        }
        var hf = new SolidHeightfield(f);
        var st = Voxelizer.RasterizeHeightGrid(hf, hts, 5, 5, 0, 0, 1, SpanSettings.Humanoid);
        Assert.Equal(32, st.Triangles);
        foreach (var c in Dump.Columns(hf))
        {
            Dump.Expect(c, (1, 2, 63));
        }
        Assert.Throws<ArgumentException>(() => Voxelizer.RasterizeHeightGrid(hf, hts, 4, 5, 0, 0, 1, SpanSettings.Humanoid));
    }
}
