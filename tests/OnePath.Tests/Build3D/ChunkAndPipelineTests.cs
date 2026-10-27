using System;
using System.Collections.Generic;
using System.Linq;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

public class ChunkAndPipelineTests
{
    private readonly record struct Box(int X0, int Y0, int Z0, int X1, int Y1, int Z1);

    // Twelve outward-facing triangles for a lattice-aligned box, wound by checking each normal against the face direction.
    private static void AddBox(List<double> verts, List<int> idx, Box b, float cs, float ch)
    {
        double[] lo = { b.X0 * (double)cs, b.Y0 * (double)ch, b.Z0 * (double)cs };
        double[] hi = { b.X1 * (double)cs, b.Y1 * (double)ch, b.Z1 * (double)cs };
        for (int axis = 0; axis < 3; axis++)
        {
            for (int side = 0; side < 2; side++)
            {
                int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
                double c = side == 0 ? lo[axis] : hi[axis];
                var corners = new double[4][];
                (double, double)[] uv = { (lo[a1], lo[a2]), (hi[a1], lo[a2]), (hi[a1], hi[a2]), (lo[a1], hi[a2]) };
                for (int k = 0; k < 4; k++)
                {
                    var p = new double[3];
                    p[axis] = c;
                    p[a1] = uv[k].Item1;
                    p[a2] = uv[k].Item2;
                    corners[k] = p;
                }
                double[] e1 = { corners[1][0] - corners[0][0], corners[1][1] - corners[0][1], corners[1][2] - corners[0][2] };
                double[] e2 = { corners[2][0] - corners[0][0], corners[2][1] - corners[0][1], corners[2][2] - corners[0][2] };
                double[] n = { (e1[1] * e2[2]) - (e1[2] * e2[1]), (e1[2] * e2[0]) - (e1[0] * e2[2]), (e1[0] * e2[1]) - (e1[1] * e2[0]) };
                bool up = n[axis] > 0;
                bool wantUp = side == 1;
                int[] order = up == wantUp ? new[] { 0, 1, 2, 0, 2, 3 } : new[] { 0, 2, 1, 0, 3, 2 };
                int baseIndex = verts.Count / 3;
                foreach (var p in corners)
                {
                    verts.AddRange(p);
                }
                foreach (int o in order)
                {
                    idx.Add(baseIndex + o);
                }
            }
        }
    }

    [Fact]
    public void BoxSoupAndEquivalentChunkGiveIdenticalFields()
    {
        var rng = new TestRng(61);
        int[] sizesW = { 6, 9, 12 };
        float[] cell = { 1f, 0.5f, 0.25f };
        for (int n = 0; n < 400; n++)
        {
            int w = sizesW[rng.Int(3)], d = sizesW[rng.Int(3)], h = 8 + rng.Int(8);
            float cs = cell[rng.Int(3)], ch = cell[rng.Int(3)];
            var frame = new HeightfieldFrame(-3.5, 10.25, 7.0, cs, ch, w, d, h);
            var verts = new List<double>();
            var idx = new List<int>();
            var dense = new byte[w * d * h];
            var used = new bool[w, d];
            for (int b = 1 + rng.Int(8); b > 0; b--)
            {
                int x0 = rng.Int(w), z0 = rng.Int(d), y0 = rng.Int(h);
                var box = new Box(x0, y0, z0, x0 + 1 + rng.Int(w - x0), y0 + 1 + rng.Int(h - y0), z0 + 1 + rng.Int(d - z0));
                // Surfaces of overlapping or stacked boxes leave interior floors that a solid chunk does not have, so footprints are kept disjoint.
                bool clash = false;
                for (int z = box.Z0; z < box.Z1 && !clash; z++)
                {
                    for (int x = box.X0; x < box.X1; x++)
                    {
                        clash |= used[x, z];
                    }
                }
                if (clash)
                {
                    continue;
                }
                var local = new List<double>();
                int before = idx.Count;
                AddBox(local, idx, box, cs, ch);
                int shift = verts.Count / 3;
                for (int i = before; i < idx.Count; i++)
                {
                    idx[i] += shift;
                }
                for (int i = 0; i < local.Count; i += 3)
                {
                    verts.Add(local[i] + frame.OriginX);
                    verts.Add(local[i + 1] + frame.OriginY);
                    verts.Add(local[i + 2] + frame.OriginZ);
                }
                for (int z = box.Z0; z < box.Z1; z++)
                {
                    for (int x = box.X0; x < box.X1; x++)
                    {
                        used[x, z] = true;
                        for (int y = box.Y0; y < box.Y1; y++)
                        {
                            dense[(((z * w) + x) * h) + y] = 1;
                        }
                    }
                }
            }
            var settings = new SpanSettings { WalkableHeight = 2, WalkableClimb = 1, WalkableRadius = 1, WalkableSlopeCosine = 0.5f };
            var soup = new SolidHeightfield(frame);
            Voxelizer.Rasterize(soup, verts.ToArray(), idx.ToArray(), settings);
            var chunk = VoxelChunk.FromDense(w, d, h, dense);
            var fromChunk = new SolidHeightfield(frame);
            Voxelizer.AddChunk(fromChunk, chunk, 0, 0, 0, default);
            // A soup of closed surfaces is a shell: interior columns hold a floor span and a ceiling span with a gap between.
            // The shell and the solid chunk differ only in that gap, which no walkable floor can see, so the compact fields match.
            AssertSameOutsideShellGaps(fromChunk, soup);

            var a = new SpanPipeline(settings).BuildFromTriangles(frame, verts.ToArray(), idx.ToArray());
            var b2 = new SpanPipeline(settings).BuildFromChunk(frame, chunk);
            Assert.Equal(Dump.Text(b2), Dump.Text(a));
        }
    }

    private static void AssertSameOutsideShellGaps(SolidHeightfield solid, SolidHeightfield shell)
    {
        var a = Dump.Columns(solid);
        var b = Dump.Columns(shell);
        for (int c = 0; c < a.Length; c++)
        {
            // Same tops with the same areas, and the same union of rows once the shell's gaps are filled.
            Assert.Equal(a[c].Count == 0, b[c].Count == 0);
            if (a[c].Count == 0)
            {
                continue;
            }
            Assert.Equal(a[c][^1].Max, b[c][^1].Max);
            Assert.Equal(a[c][^1].Area, b[c][^1].Area);
            Assert.Equal(a[c][0].Min, b[c][0].Min);
        }
    }

    [Fact]
    public void DenseAndRunLengthChunksAgreeAndMaterialsMapToAreas()
    {
        var dense = new byte[2 * 1 * 6];
        // column 0: rows 0-1 material 3, rows 2-3 material 5, row 5 material 3; column 1: empty
        dense[0] = 3;
        dense[1] = 3;
        dense[2] = 5;
        dense[3] = 5;
        dense[5] = 3;
        var a = VoxelChunk.FromDense(2, 1, 6, dense);
        Assert.Equal(3, a.Runs.Length);
        var b = VoxelChunk.FromRuns(2, 1, 6, new[] { 0, 3, 3 }, new[] { new VoxelRun(0, 2, 3), new VoxelRun(2, 2, 5), new VoxelRun(5, 1, 3) });
        Assert.True(a.Runs.SequenceEqual(b.Runs));
        var areas = new byte[256];
        areas[3] = 20;
        areas[5] = 0;
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 2, 1, 6));
        Assert.Equal(3, Voxelizer.AddChunk(hf, a, 0, 0, 0, areas));
        // Rows 0-3 merge into one span whose top belongs to material 5 (area 0); row 5 stands alone.
        Dump.Expect(Dump.Columns(hf)[0], (0, 4, 0), (5, 6, 20));
        Assert.Empty(Dump.Columns(hf)[1]);
        // Offsets clip.
        var hf2 = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 2, 1, 4));
        Voxelizer.AddChunk(hf2, a, 1, -1, 0, default);
        Dump.Expect(Dump.Columns(hf2)[1], (0, 3, 63));
    }

    [Fact]
    public void ChunkValidationNamesTheProblem()
    {
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromRuns(1, 1, 4, new[] { 0, 2 }, new[] { new VoxelRun(0, 2, 1), new VoxelRun(1, 1, 1) }));
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromRuns(1, 1, 4, new[] { 0, 1 }, new[] { new VoxelRun(3, 2, 1) }));
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromRuns(1, 1, 4, new[] { 0, 1 }, new[] { new VoxelRun(0, 1, 0) }));
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromRuns(1, 1, 4, new[] { 0, 1 }, new[] { new VoxelRun(0, 0, 1) }));
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromRuns(1, 1, 4, new[] { 1, 1 }, ReadOnlySpan<VoxelRun>.Empty));
        Assert.Throws<ArgumentException>(() => VoxelChunk.FromDense(2, 2, 2, new byte[7]));
        Assert.Throws<ArgumentOutOfRangeException>(() => VoxelChunk.FromDense(0, 2, 2, ReadOnlySpan<byte>.Empty));
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, 1, 1, 4));
        Assert.Throws<ArgumentException>(() => Voxelizer.AddChunk(hf, VoxelChunk.FromDense(1, 1, 1, new byte[1]), 0, 0, 0, new byte[5]));
    }

    [Fact]
    public void PipelineStagesAreReplaceable()
    {
        var frame = new HeightfieldFrame(0, 0, 0, 1f, 1f, 8, 8, 8);
        var dense = new byte[8 * 8 * 8];
        for (int c = 0; c < 64; c++)
        {
            dense[c * 8] = 1;
        }
        var chunk = VoxelChunk.FromDense(8, 8, 8, dense);
        var settings = new SpanSettings { WalkableHeight = 2, WalkableClimb = 1, WalkableRadius = 2 };
        var def = new SpanPipeline(settings).BuildFromChunk(frame, chunk);
        int defWalkable = Count(def);
        Assert.True(defWalkable < 64);

        var noErode = new SpanPipeline(settings) { Erode = null, LedgeFilter = null };
        Assert.Equal(64, Count(noErode.BuildFromChunk(frame, chunk)));

        int called = 0;
        var custom = new SpanPipeline(settings) { LedgeFilter = (hf, s) => called++ };
        custom.Volumes.Add(new ConvexVolume(new double[] { 0, 0, 8, 0, 8, 8, 0, 8 }, -10, 10, 9));
        var r = custom.BuildFromChunk(frame, chunk);
        Assert.Equal(1, called);
        for (int i = 0; i < r.SpanCount; i++)
        {
            Assert.True(r.Spans[i].Area is 0 or 9);
        }
        Assert.Throws<InvalidOperationException>(() => new SpanPipeline(settings) { Compact = null! }.BuildFromChunk(frame, chunk));
        Assert.Throws<ArgumentOutOfRangeException>(() => new SpanPipeline(new SpanSettings { WalkableHeight = 0 }).BuildFromChunk(frame, chunk));
    }

    private static int Count(CompactHeightfield c)
    {
        int n = 0;
        for (int i = 0; i < c.SpanCount; i++)
        {
            n += c.Spans[i].Area != 0 ? 1 : 0;
        }
        return n;
    }
}
