using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// The 3D builder up to contours: the <see cref="SpanPipeline"/> (filters, compaction, erosion, volumes) followed by the region partition and contour
/// tracing, each a replaceable step. The result is a <see cref="ContourSet"/>, the hand-off to the polygon stage.
/// </summary>
public sealed class RegionPipeline
{
    /// <summary>Creates a pipeline with default steps and settings.</summary>
    /// <param name="span">Agent parameters of the span stage; defaults to <see cref="SpanSettings.Humanoid"/>.</param>
    /// <param name="region">Region stage parameters; defaults to watershed with the default thresholds.</param>
    /// <param name="contour">Contour stage parameters; defaults to error 1.3 and edge length 12.</param>
    public RegionPipeline(SpanSettings? span = null, RegionSettings? region = null, ContourSettings? contour = null)
    {
        Spans = new SpanPipeline(span);
        RegionOptions = region ?? new RegionSettings();
        ContourOptions = contour ?? new ContourSettings();
    }

    /// <summary>The span stage, with its own replaceable steps.</summary>
    public SpanPipeline Spans { get; }

    /// <summary>Parameters of <see cref="Partition"/>.</summary>
    public RegionSettings RegionOptions { get; set; }

    /// <summary>Parameters of <see cref="Trace"/>.</summary>
    public ContourSettings ContourOptions { get; set; }

    /// <summary>The region partition step; must write <see cref="CompactSpan.Region"/>. Default: <see cref="Regions.Build"/>.</summary>
    public Action<CompactHeightfield, RegionSettings> Partition { get; set; } = Regions.Build;

    /// <summary>The contour step. Default: <see cref="Contours.Build"/>.</summary>
    public Func<CompactHeightfield, ContourSettings, ContourSet> Trace { get; set; } = Contours.Build;

    /// <summary>Runs the span stage on a rasterized heightfield, then partitions and traces.</summary>
    /// <param name="solid">The rasterized heightfield (the filters modify it).</param>
    /// <param name="compact">Receives the compact heightfield with regions, for the stages that need spans as well as contours.</param>
    /// <exception cref="InvalidOperationException">A step is missing or returns null.</exception>
    public ContourSet Run(SolidHeightfield solid, out CompactHeightfield compact)
    {
        ArgumentNullException.ThrowIfNull(solid);
        compact = Spans.Run(solid);
        (Partition ?? throw new InvalidOperationException("The Partition step is missing.")).Invoke(compact, RegionOptions);
        return (Trace ?? throw new InvalidOperationException("The Trace step is missing.")).Invoke(compact, ContourOptions)
            ?? throw new InvalidOperationException("The Trace step returned null.");
    }

    /// <summary>Rasterizes triangle soup on <paramref name="frame"/> and runs the whole pipeline.</summary>
    public ContourSet BuildFromTriangles(HeightfieldFrame frame, ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices)
    {
        var solid = new SolidHeightfield(frame);
        Voxelizer.Rasterize(solid, vertices, indices, Spans.Settings);
        return Run(solid, out _);
    }

    /// <summary>Adds a voxel chunk at the frame's origin and runs the whole pipeline.</summary>
    public ContourSet BuildFromChunk(HeightfieldFrame frame, VoxelChunk chunk, ReadOnlySpan<byte> materialAreas = default)
    {
        var solid = new SolidHeightfield(frame);
        Voxelizer.AddChunk(solid, chunk, 0, 0, 0, materialAreas);
        return Run(solid, out _);
    }
}
