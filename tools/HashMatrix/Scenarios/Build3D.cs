using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.NavMesh.Build3D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Scenarios for the span stage of the 3D builder: rasterization of seeded random soups, the filters, compaction with
/// erosion and volumes, the voxel-chunk path, and a whole pipeline on a generated terrain. Output is the span data itself.
/// </summary>
internal static class Build3DScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build3d/voxelize", Voxelize);
        Scenarios.Register("build3d/filters", Filters);
        Scenarios.Register("build3d/erosion-volumes", ErosionVolumes);
        Scenarios.Register("build3d/chunk", Chunk);
        Scenarios.Register("build3d/pipeline-terrain", Terrain);
    }

    private sealed class Lcg
    {
        private ulong _s;

        public Lcg(ulong seed) => _s = (seed * 2862933555777941757UL) + 3037000493UL;

        public uint Next()
        {
            _s = (_s * 6364136223846793005UL) + 1442695040888963407UL;
            return (uint)(_s >> 33);
        }

        public int Int(int n) => (int)(Next() % (uint)n);

        public double Range(double lo, double hi) => lo + ((hi - lo) * (Next() / 2147483648.0));
    }

    private static void Write(ScenarioContext ctx, SolidHeightfield hf)
    {
        ctx.Write(hf.SpanCount);
        for (int c = 0; c < hf.Frame.ColumnCount; c++)
        {
            int n = 0;
            for (int i = hf.Heads[c]; i != -1; i = hf.Span(i).Next)
            {
                n++;
            }
            ctx.Write(n);
            for (int i = hf.Heads[c]; i != -1; i = hf.Span(i).Next)
            {
                ctx.Write(hf.Span(i).Min);
                ctx.Write(hf.Span(i).Max);
                ctx.Write((int)hf.Span(i).Area);
            }
        }
    }

    private static void Write(ScenarioContext ctx, CompactHeightfield chf)
    {
        ctx.Write(chf.SpanCount);
        for (int c = 0; c < chf.CellCount.Length; c++)
        {
            ctx.Write(chf.CellCount[c]);
        }
        for (int i = 0; i < chf.SpanCount; i++)
        {
            CompactSpan s = chf.Spans[i];
            ctx.Write(s.Y);
            ctx.Write(s.Height);
            ctx.Write((int)s.Area);
            ctx.Write(s.Connections);
        }
    }

    private static (double[] V, int[] I) Soup(Lcg rng, int tris, double w, double d, double h, double ox, double oy, double oz)
    {
        var v = new double[tris * 9];
        var idx = new int[tris * 3];
        for (int t = 0; t < tris; t++)
        {
            // Small triangles near a random centre, plus a few large ones, wound upward or downward at random.
            double cx = ox + rng.Range(0, w), cy = oy + rng.Range(0, h), cz = oz + rng.Range(0, d);
            double r = t % 7 == 0 ? 6.0 : 1.5;
            for (int k = 0; k < 3; k++)
            {
                v[(t * 9) + (k * 3)] = cx + rng.Range(-r, r);
                v[(t * 9) + (k * 3) + 1] = cy + rng.Range(-r * 0.3, r * 0.3);
                v[(t * 9) + (k * 3) + 2] = cz + rng.Range(-r, r);
                idx[(t * 3) + k] = (t * 3) + k;
            }
        }
        return (v, idx);
    }

    private static void Voxelize(ScenarioContext ctx)
    {
        var rng = new Lcg(1);
        for (int n = 0; n < 40; n++)
        {
            float cs = n % 3 == 0 ? 0.3f : n % 3 == 1 ? 0.25f : 0.5f;
            float ch = n % 2 == 0 ? 0.2f : 0.25f;
            var frame = new HeightfieldFrame(-12.5 + n, 1.25, 1000.125 - n, cs, ch, 30 + (n % 7), 28 + (n % 5), 40);
            var (v, idx) = Soup(rng, 60, frame.Width * cs, frame.Depth * cs, frame.Height * ch, frame.OriginX, frame.OriginY, frame.OriginZ);
            var hf = new SolidHeightfield(frame);
            var st = Voxelizer.Rasterize(hf, v, idx, SpanSettings.Humanoid);
            ctx.Write(st.Degenerate);
            ctx.Write(st.Touching);
            Write(ctx, hf);
        }
    }

    private static SolidHeightfield RandomField(Lcg rng, int w, int d, int h)
    {
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, w, d, h));
        byte[] areas = { 0, 63, 63, 10 };
        for (int c = 0; c < w * d; c++)
        {
            int y = rng.Int(5);
            for (int s = rng.Int(4); s > 0 && y < h - 1; s--)
            {
                int top = Math.Min(h, y + 1 + rng.Int(5));
                hf.AddSpan(c, y, top, areas[rng.Int(4)], 0);
                y = top + 1 + rng.Int(6);
            }
        }
        return hf;
    }

    private static void Filters(ScenarioContext ctx)
    {
        var rng = new Lcg(2);
        for (int n = 0; n < 30; n++)
        {
            var hf = RandomField(rng, 24, 24, 30);
            SpanFilters.FilterLowHangingWalkableObstacles(hf, 1 + (n % 4));
            Write(ctx, hf);
            SpanFilters.FilterLedgeSpans(hf, 2 + (n % 3), 1 + (n % 4));
            Write(ctx, hf);
            SpanFilters.FilterWalkableLowHeightSpans(hf, 2 + (n % 3));
            Write(ctx, hf);
        }
    }

    private static void ErosionVolumes(ScenarioContext ctx)
    {
        var rng = new Lcg(3);
        for (int n = 0; n < 20; n++)
        {
            var hf = RandomField(rng, 32, 32, 24);
            var chf = CompactHeightfield.Build(hf, 2, 1 + (n % 3));
            Write(ctx, chf);
            Erosion.ErodeWalkableArea(chf, 1 + (n % 4));
            Write(ctx, chf);
            var vol = new ConvexVolume(new[] { 4.0, 4.0, 20.0, 6.0, 24.0, 20.0, 8.0, 26.0 }, 0, 12 + n, (byte)(20 + n));
            ctx.Write(vol.Apply(chf));
            Write(ctx, chf);
        }
    }

    private static void Chunk(ScenarioContext ctx)
    {
        var rng = new Lcg(4);
        for (int n = 0; n < 10; n++)
        {
            int w = 20 + n, d = 18 + n, h = 24;
            var dense = new byte[w * d * h];
            for (int c = 0; c < w * d; c++)
            {
                int y = rng.Int(8);
                while (y < h)
                {
                    int len = 1 + rng.Int(5);
                    byte m = (byte)(1 + rng.Int(3));
                    for (int k = 0; k < len && y + k < h; k++)
                    {
                        dense[(c * h) + y + k] = m;
                    }
                    y += len + rng.Int(7);
                }
            }
            var chunk = VoxelChunk.FromDense(w, d, h, dense);
            var frame = new HeightfieldFrame(-4.5, 0.5, 11.5, 0.5f, 0.25f, w + 2, d + 2, h);
            var areas = new byte[256];
            areas[1] = 63;
            areas[2] = 12;
            areas[3] = 0;
            var solid = new SolidHeightfield(frame);
            Voxelizer.AddChunk(solid, chunk, 1, 0, 1, areas);
            Write(ctx, solid);
            var chf = new SpanPipeline(new SpanSettings { WalkableHeight = 6, WalkableClimb = 2, WalkableRadius = 1 }).Run(solid);
            Write(ctx, chf);
        }
    }

    // A terrain from integer arithmetic only: bumpy height samples smoothed twice, fed as a height grid.
    private static void Terrain(ScenarioContext ctx)
    {
        var rng = new Lcg(5);
        const int N = 97;
        var hts = new double[N * N];
        for (int i = 0; i < hts.Length; i++)
        {
            hts[i] = rng.Int(40) / 8.0;
        }
        for (int pass = 0; pass < 2; pass++)
        {
            var next = new double[hts.Length];
            for (int z = 0; z < N; z++)
            {
                for (int x = 0; x < N; x++)
                {
                    double sum = 0;
                    int cnt = 0;
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            int xx = x + dx, zz = z + dz;
                            if (xx >= 0 && zz >= 0 && xx < N && zz < N)
                            {
                                sum += hts[xx + (zz * N)];
                                cnt++;
                            }
                        }
                    }
                    next[x + (z * N)] = sum / cnt;
                }
            }
            hts = next;
        }
        var frame = HeightfieldFrame.FromBounds(0, -1, 0, 96, 8, 96, 0.5f, 0.2f);
        var hf = new SolidHeightfield(frame);
        var settings = new SpanSettings { WalkableHeight = 10, WalkableClimb = 3, WalkableRadius = 2 };
        var st = Voxelizer.RasterizeHeightGrid(hf, hts, N, N, 0, 0, 1.0, settings);
        ctx.Write(st.Triangles);
        ctx.Write(st.Touching);
        var pipe = new SpanPipeline(settings);
        pipe.Volumes.Add(new ConvexVolume(new[] { 10.0, 10.0, 60.0, 12.0, 50.0, 70.0 }, -1, 8, 5));
        Write(ctx, hf);
        Write(ctx, pipe.Run(hf));
    }
}
