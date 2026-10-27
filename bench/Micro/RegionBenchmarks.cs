using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath.NavMesh.Build3D;

namespace RecursiveAI.OnePath.Micro;

/// <summary>Region and contour rows on a bumpy terrain with walls: distance field, the three partitions, contour tracing and simplification.</summary>
[MemoryDiagnoser]
public class RegionBenchmarks
{
    private CompactHeightfield _compact = null!;
    private SpanSettings _settings = null!;

    [GlobalSetup]
    public void Setup()
    {
        const int N = 257;
        var heights = new double[N * N];
        ulong s = 777;
        for (int i = 0; i < heights.Length; i++)
        {
            s = (s * 6364136223846793005UL) + 1442695040888963407UL;
            // Plateaus separated by steep drops so there are walls to trace.
            heights[i] = (((i % N) / 32) + ((i / N) / 32)) % 3 * 1.5 + (((s >> 40) % 8) / 32.0);
        }
        var frame = HeightfieldFrame.FromBounds(0, -1, 0, 256, 8, 256, 0.3f, 0.2f);
        _settings = SpanSettings.Humanoid;
        var solid = new SolidHeightfield(frame);
        Voxelizer.RasterizeHeightGrid(solid, heights, N, N, 0, 0, 1.0, _settings);
        _compact = new SpanPipeline(_settings).Run(solid);
    }

    [Benchmark]
    public int DistanceField()
    {
        Regions.BuildDistanceField(_compact, out int max);
        return max;
    }

    [Benchmark]
    public int Watershed()
    {
        Regions.BuildWatershed(_compact, new RegionSettings { Partition = PartitionType.Watershed });
        return _compact.RegionCount;
    }

    [Benchmark]
    public int Monotone()
    {
        Regions.BuildMonotone(_compact, new RegionSettings { Partition = PartitionType.Monotone });
        return _compact.RegionCount;
    }

    [Benchmark]
    public int Layers()
    {
        Regions.BuildLayers(_compact, new RegionSettings { Partition = PartitionType.Layers });
        return _compact.RegionCount;
    }

    [Benchmark]
    public int TraceContours()
    {
        Regions.BuildWatershed(_compact, new RegionSettings());
        return Contours.Build(_compact, new ContourSettings { MaxEdgeLength = 40 }).Contours.Count;
    }
}
