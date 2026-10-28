using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Scenarios for the region and contour stages of the 3D builder: all three partitions and the contours on seeded fields and a generated terrain.</summary>
internal static class Build3DRegionScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("build3d/regions-watershed", ctx => Regions(ctx, PartitionType.Watershed));
        Scenarios.Register("build3d/regions-monotone", ctx => Regions(ctx, PartitionType.Monotone));
        Scenarios.Register("build3d/regions-layers", ctx => Regions(ctx, PartitionType.Layers));
        Scenarios.Register("build3d/contours", Contours);
        Scenarios.Register("build3d/regions-contours-terrain", Terrain);
        Scenarios.Register("build3d/contours-straight-borders", StraightBorders);
    }

    private static ulong s_seed;

    private static int Next(int n)
    {
        s_seed = (s_seed * 6364136223846793005UL) + 1442695040888963407UL;
        return (int)((uint)(s_seed >> 33) % (uint)n);
    }

    private static CompactHeightfield Field(int w, int d)
    {
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, w, d, 24));
        for (int c = 0; c < w * d; c++)
        {
            if (Next(100) < 12)
            {
                continue;
            }
            hf.AddSpan(c, 0, 1 + (Next(10) < 8 ? 0 : 1), Next(10) < 8 ? (byte)63 : (byte)10, 0);
            if (Next(8) == 0)
            {
                hf.AddSpan(c, 8, 9 + Next(2), 63, 0);
            }
        }
        return CompactHeightfield.Build(hf, 3, 1);
    }

    private static void WriteRegions(ScenarioContext ctx, CompactHeightfield chf)
    {
        ctx.Write(chf.RegionCount);
        for (int i = 0; i < chf.SpanCount; i++)
        {
            ctx.Write(chf.Spans[i].Region);
        }
    }

    private static void WriteContours(ScenarioContext ctx, ContourSet set)
    {
        ctx.Write(set.Contours.Count);
        ctx.Write(set.DroppedContours);
        ctx.Write(set.DroppedHoles);
        ctx.Write(set.RawFallbacks);
        foreach (Contour k in set.Contours)
        {
            ctx.Write(k.Region);
            ctx.Write((int)k.Area);
            ctx.Write(k.Vertices.Length);
            foreach (ContourVertex v in k.Vertices)
            {
                ctx.Write(v.X);
                ctx.Write(v.Y);
                ctx.Write(v.Z);
                ctx.Write(v.Neighbor);
            }
        }
    }

    private static void Regions(ScenarioContext ctx, PartitionType type)
    {
        s_seed = 11;
        for (int n = 0; n < 40; n++)
        {
            CompactHeightfield chf = Field(20 + (n % 9), 18 + (n % 7));
            Mesh.Build3D.Regions.Build(chf, new RegionSettings { Partition = type, MinRegionArea = 4 * (n % 3), MergeRegionArea = type == PartitionType.Layers ? 0 : 20 * (n % 4) });
            WriteRegions(ctx, chf);
        }
    }

    private static void Contours(ScenarioContext ctx)
    {
        s_seed = 12;
        for (int n = 0; n < 40; n++)
        {
            CompactHeightfield chf = Field(24 + (n % 5), 22 + (n % 11));
            Mesh.Build3D.Regions.Build(chf, new RegionSettings { Partition = (PartitionType)(n % 3), MinRegionArea = 4, MergeRegionArea = (n % 3 == 2) ? 0 : 30 });
            WriteContours(ctx, Mesh.Build3D.Contours.Build(chf, new ContourSettings { MaxError = (n % 4) * 0.65f, MaxEdgeLength = (n % 3) * 6 }));
        }
    }

    // A wide plane with pillars: the watershed splits it into large regions with long diagonal borders, where straight borders save vertices.
    private static void StraightBorders(ScenarioContext ctx)
    {
        s_seed = 14;
        const int W = 120;
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 1f, 1f, W, W, 24));
        var blocked = new bool[W * W];
        for (int p = 0; p < 14; p++)
        {
            int px = 6 + Next(W - 14), pz = 6 + Next(W - 14);
            for (int dz = 0; dz < 3; dz++)
            {
                for (int dx = 0; dx < 3; dx++)
                {
                    blocked[((pz + dz) * W) + px + dx] = true;
                }
            }
        }
        for (int c = 0; c < W * W; c++)
        {
            if (!blocked[c])
            {
                hf.AddSpan(c, 0, 1, (c / W) < W / 2 ? (byte)63 : (byte)10, 0);
            }
        }
        CompactHeightfield chf = CompactHeightfield.Build(hf, 3, 1);
        Mesh.Build3D.Regions.Build(chf, new RegionSettings { Partition = PartitionType.Watershed, MinRegionArea = 4, MergeRegionArea = 0 });
        WriteRegions(ctx, chf);
        foreach (bool straight in new[] { false, true })
        {
            ContourSet set = Mesh.Build3D.Contours.Build(chf, new ContourSettings { MaxError = 1.3f, MaxEdgeLength = 12, StraightRegionBorders = straight });
            WriteContours(ctx, set);
            int total = 0;
            foreach (Contour k in set.Contours)
            {
                total += k.Vertices.Length;
            }
            ctx.Write(total);
        }
    }

    private static void Terrain(ScenarioContext ctx)
    {
        s_seed = 13;
        const int N = 97;
        var hts = new double[N * N];
        for (int i = 0; i < hts.Length; i++)
        {
            hts[i] = Next(40) / 8.0;
        }
        var frame = HeightfieldFrame.FromBounds(0, -1, 0, 96, 8, 96, 0.5f, 0.2f);
        var hf = new SolidHeightfield(frame);
        var span = new SpanSettings { WalkableHeight = 10, WalkableClimb = 3, WalkableRadius = 2 };
        Voxelizer.RasterizeHeightGrid(hf, hts, N, N, 0, 0, 1.0, span);
        var pipe = new RegionPipeline(span, new RegionSettings { MinRegionArea = 16, MergeRegionArea = 64 }, new ContourSettings());
        WriteContours(ctx, pipe.Run(hf, out CompactHeightfield chf));
        WriteRegions(ctx, chf);
    }
}
