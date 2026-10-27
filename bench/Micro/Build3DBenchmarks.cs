using System;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Micro;

/// <summary>Span-stage rows: rasterizing a bumpy terrain, the filters, compaction, erosion and the voxel-chunk path.</summary>
[MemoryDiagnoser]
public class Build3DBenchmarks
{
    private const int N = 257;
    private double[] _heights = null!;
    private HeightfieldFrame _frame = null!;
    private SpanSettings _settings = null!;
    private SolidHeightfield _solid = null!;
    private CompactHeightfield _compact = null!;
    private VoxelChunk _chunk = null!;

    [GlobalSetup]
    public void Setup()
    {
        _heights = new double[N * N];
        ulong s = 12345;
        for (int i = 0; i < _heights.Length; i++)
        {
            s = (s * 6364136223846793005UL) + 1442695040888963407UL;
            _heights[i] = ((s >> 40) % 64) / 16.0;
        }
        _frame = HeightfieldFrame.FromBounds(0, -1, 0, 256, 8, 256, 0.3f, 0.2f);
        _settings = SpanSettings.Humanoid;
        _solid = new SolidHeightfield(_frame);
        Voxelizer.RasterizeHeightGrid(_solid, _heights, N, N, 0, 0, 1.0, _settings);
        var dense = new byte[N * N * 16];
        for (int c = 0; c < N * N; c++)
        {
            dense[c * 16] = 1;
            dense[(c * 16) + 1 + (int)((_heights[c] * 2) % 6)] = 1;
        }
        _chunk = VoxelChunk.FromDense(N, N, 16, dense);
    }

    [Benchmark]
    public int RasterizeTerrain()
    {
        _solid.Clear();
        return Voxelizer.RasterizeHeightGrid(_solid, _heights, N, N, 0, 0, 1.0, _settings).Touching;
    }

    [Benchmark]
    public int FiltersAndCompact()
    {
        _solid.Clear();
        Voxelizer.RasterizeHeightGrid(_solid, _heights, N, N, 0, 0, 1.0, _settings);
        SpanFilters.FilterLowHangingWalkableObstacles(_solid, _settings.WalkableClimb);
        SpanFilters.FilterLedgeSpans(_solid, _settings.WalkableHeight, _settings.WalkableClimb);
        SpanFilters.FilterWalkableLowHeightSpans(_solid, _settings.WalkableHeight);
        _compact = CompactHeightfield.Build(_solid, _settings.WalkableHeight, _settings.WalkableClimb);
        return _compact.SpanCount;
    }

    [Benchmark]
    public int Erode()
    {
        var c = _compact ?? CompactHeightfield.Build(_solid, _settings.WalkableHeight, _settings.WalkableClimb);
        Erosion.ErodeWalkableArea(c, _settings.WalkableRadius);
        return c.SpanCount;
    }

    [Benchmark]
    public int ChunkToSpans()
    {
        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 0.3f, 0.2f, N, N, 16));
        return Voxelizer.AddChunk(hf, _chunk, 0, 0, 0, default);
    }
}
