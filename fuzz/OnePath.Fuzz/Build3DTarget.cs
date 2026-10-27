using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target: bytes become a small grid, agent settings and either a triangle soup (with hostile coordinates mixed in)
/// or a voxel chunk; the span pipeline runs and its output invariants are checked. Documented input errors
/// (<see cref="ArgumentException"/>, which includes <see cref="ArgumentOutOfRangeException"/>) are expected; anything else is a finding.
/// </summary>
internal static class Build3DTarget
{
    private static readonly double[] Special =
    {
        double.NaN, double.PositiveInfinity, double.NegativeInfinity, 3e6, -3e6, 1e300, -0.0, 4.9e-324, 2097152.0, -2097152.0, 2097152.5,
    };

    public static void Run(byte[] data) => Run(data, false);

    /// <summary>The contour variant: runs the region partition and contour tracing as well and checks the contours.</summary>
    public static void RunContours(byte[] data) => Run(data, true);

    private static void Run(byte[] data, bool contours)
    {
        var r = new Reader(data);
        int w = 1 + (r.Byte() % 12);
        int d = 1 + (r.Byte() % 12);
        int h = 1 + (r.Byte() % 16);
        float[] sizes = { 0.25f, 0.5f, 1f, 0.3f, 0.2f };
        float cs = sizes[r.Byte() % sizes.Length];
        float ch = sizes[r.Byte() % sizes.Length];
        double ox = (r.Coordinate(false) * 4.0) - 3.0;
        double oy = r.Coordinate(false);
        double oz = r.Coordinate(false);
        var settings = new SpanSettings
        {
            WalkableClimb = r.Byte() % 6,
            WalkableHeight = 1 + (r.Byte() % 6),
            WalkableRadius = r.Byte() % 4,
            WalkableSlopeCosine = (r.Byte() - 32) / 200f,
            MergeThreshold = (r.Byte() % 5) - 1,
        };
        bool chunkMode = (r.Byte() & 1) == 1;
        try
        {
            var frame = new HeightfieldFrame(ox, oy, oz, cs, ch, w, d, h);
            var pipe = new SpanPipeline(settings);
            if (r.Byte() % 3 == 0)
            {
                double[] xz = { r.Coordinate(false), r.Coordinate(false), r.Coordinate(false), r.Coordinate(false), r.Coordinate(false), r.Coordinate(false) };
                pipe.Volumes.Add(new ConvexVolume(xz, r.Coordinate(false), r.Coordinate(false), (byte)r.Byte()));
            }
            CompactHeightfield chf;
            if (chunkMode)
            {
                int cw = 1 + (r.Byte() % 12), cd = 1 + (r.Byte() % 12), chh = 1 + (r.Byte() % 16);
                var voxels = new byte[cw * cd * chh];
                for (int i = 0; i < voxels.Length && r.Remaining > 0; i++)
                {
                    voxels[i] = (byte)(r.Byte() % 3);
                }
                var chunk = VoxelChunk.FromDense(cw, cd, chh, voxels);
                var solid = new SolidHeightfield(frame);
                Voxelizer.AddChunk(solid, chunk, (r.Byte() % 5) - 2, (r.Byte() % 5) - 2, (r.Byte() % 5) - 2, default);
                CheckSolid(solid);
                chf = pipe.Run(solid);
            }
            else
            {
                int nv = 3 + (r.Byte() % 24);
                var verts = new double[nv * 3];
                double[] origin = { ox, oy, oz };
                for (int i = 0; i < verts.Length; i++)
                {
                    double v = r.Coordinate(true);
                    verts[i] = double.IsFinite(v) && Math.Abs(v) < 1e5 ? origin[i % 3] + (v / 8.0) : v;
                }
                int nt = r.Byte() % 24;
                var idx = new int[nt * 3];
                for (int i = 0; i < idx.Length; i++)
                {
                    idx[i] = r.Byte() % (nv + 1); // sometimes out of range
                }
                var solid = new SolidHeightfield(frame);
                Voxelizer.Rasterize(solid, verts, idx, settings);
                CheckSolid(solid);
                chf = pipe.Run(solid);
            }
            CheckCompact(chf);
            if (contours)
            {
                var rs = new RegionSettings { Partition = (PartitionType)(r.Byte() % 3), MinRegionArea = r.Byte() % 12, MergeRegionArea = r.Byte() % 40 };
                Regions.Build(chf, rs);
                var set = Contours.Build(chf, new ContourSettings { MaxError = (r.Byte() % 5) * 0.5f, MaxEdgeLength = r.Byte() % 16, KeepRawVertices = true });
                CheckContours(chf, set);
            }
        }
        catch (ArgumentException)
        {
            // Documented rejection of bad input.
        }
    }

    private static void CheckSolid(SolidHeightfield hf)
    {
        int count = 0;
        for (int c = 0; c < hf.Frame.ColumnCount; c++)
        {
            int prevMax = -2;
            for (int i = hf.Heads[c]; i != -1; i = hf.Span(i).Next)
            {
                SolidSpan s = hf.Span(i);
                if (s.Min < 0 || s.Max > hf.Frame.Height || s.Min >= s.Max || s.Min <= prevMax)
                {
                    throw new InvalidOperationException($"bad span {s.Min}-{s.Max} in column {c}");
                }
                prevMax = s.Max;
                count++;
            }
        }
        if (count != hf.SpanCount)
        {
            throw new InvalidOperationException("span count mismatch");
        }
    }

    private static void CheckCompact(CompactHeightfield chf)
    {
        HeightfieldFrame f = chf.Frame;
        int total = 0;
        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                total += chf.CellCount[c];
                for (int i = chf.CellFirst[c]; i < chf.CellFirst[c] + chf.CellCount[c]; i++)
                {
                    CompactSpan s = chf.Spans[i];
                    if (s.Y < 0 || s.Y > f.Height || s.Height < 1)
                    {
                        throw new InvalidOperationException("bad compact span");
                    }
                    for (int dir = 0; dir < 4; dir++)
                    {
                        int con = CompactHeightfield.GetConnection(s, dir);
                        if (con == CompactHeightfield.NotConnected)
                        {
                            continue;
                        }
                        int nx = x + CompactHeightfield.DirX[dir];
                        int nz = z + CompactHeightfield.DirZ[dir];
                        if ((uint)nx >= (uint)f.Width || (uint)nz >= (uint)f.Depth || con >= chf.CellCount[f.Column(nx, nz)])
                        {
                            throw new InvalidOperationException("bad link");
                        }
                    }
                }
            }
        }
        if (total != chf.SpanCount)
        {
            throw new InvalidOperationException("compact count mismatch");
        }
    }

    private static void CheckContours(CompactHeightfield chf, ContourSet set)
    {
        int[] seen = new int[chf.RegionCount + 1];
        for (int i = 0; i < chf.SpanCount; i++)
        {
            int reg = chf.Spans[i].Region;
            if (reg < 0 || reg > chf.RegionCount || (reg > 0 && chf.Spans[i].Area == 0))
            {
                throw new InvalidOperationException("bad region id");
            }
        }
        foreach (Contour k in set.Contours)
        {
            if (k.Region < 1 || k.Region > chf.RegionCount || k.Vertices.Length < 3)
            {
                throw new InvalidOperationException("bad contour");
            }
            seen[k.Region]++;
            long area = 0;
            for (int i = 0, j = k.Vertices.Length - 1; i < k.Vertices.Length; j = i++)
            {
                ContourVertex a = k.Vertices[j], b = k.Vertices[i];
                if (a.X < 0 || a.X > chf.Frame.Width || a.Z < 0 || a.Z > chf.Frame.Depth)
                {
                    throw new InvalidOperationException("contour vertex outside the grid");
                }
                area += ((long)a.X * b.Z) - ((long)b.X * a.Z);
            }
            if (area <= 0)
            {
                throw new InvalidOperationException("contour is not counter-clockwise");
            }
        }
    }

    private sealed class Reader
    {
        private readonly byte[] _d;
        private int _p;

        public Reader(byte[] d) => _d = d;

        public int Remaining => _d.Length - _p;

        public int Byte() => _p < _d.Length ? _d[_p++] : 0;

        // Two bytes: mostly a multiple of 1/16 within ±2048 (so geometry lands on and near cell boundaries), sometimes a hostile value.
        public double Coordinate(bool allowSpecial)
        {
            int v = (short)(Byte() | (Byte() << 8));
            if (allowSpecial && (v & 0xF000) == 0xF000)
            {
                return Special[((v >> 4) & 0xFF) % Special.Length];
            }
            return (v & 0x0FFF) / 16.0 - ((v & 0x0800) != 0 ? 256.0 : 0.0);
        }
    }
}
