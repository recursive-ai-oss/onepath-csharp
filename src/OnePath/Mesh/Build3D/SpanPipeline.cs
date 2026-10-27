using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// The span stage as a pipeline with every step visible and replaceable: filters on the solid heightfield, compaction,
/// erosion and area volumes. Each step is a public property holding a delegate; the defaults are the library's
/// implementations (<see cref="SpanFilters"/>, <see cref="CompactHeightfield.Build"/>, <see cref="Erosion"/>, <see cref="ConvexVolume"/>).
/// The result is a <see cref="CompactHeightfield"/>, the hand-off to the region stage.
/// </summary>
public sealed class SpanPipeline
{
    /// <summary>Creates a pipeline with default steps.</summary>
    /// <param name="settings">Agent parameters; defaults to <see cref="SpanSettings.Humanoid"/>.</param>
    public SpanPipeline(SpanSettings? settings = null)
    {
        Settings = settings ?? SpanSettings.Humanoid;
    }

    /// <summary>Agent parameters passed to every step.</summary>
    public SpanSettings Settings { get; set; }

    /// <summary>Step 1: low-hanging obstacles. Set to null to skip.</summary>
    public Action<SolidHeightfield, SpanSettings>? LowObstacleFilter { get; set; } = static (hf, s) => SpanFilters.FilterLowHangingWalkableObstacles(hf, s.WalkableClimb);

    /// <summary>Step 2: ledge spans. Set to null to skip.</summary>
    public Action<SolidHeightfield, SpanSettings>? LedgeFilter { get; set; } = static (hf, s) => SpanFilters.FilterLedgeSpans(hf, s.WalkableHeight, s.WalkableClimb);

    /// <summary>Step 3: low-clearance spans. Set to null to skip.</summary>
    public Action<SolidHeightfield, SpanSettings>? LowHeightFilter { get; set; } = static (hf, s) => SpanFilters.FilterWalkableLowHeightSpans(hf, s.WalkableHeight);

    /// <summary>Step 4: compaction into the open-span heightfield. Must not be null.</summary>
    public Func<SolidHeightfield, SpanSettings, CompactHeightfield> Compact { get; set; } = static (hf, s) => CompactHeightfield.Build(hf, s.WalkableHeight, s.WalkableClimb);

    /// <summary>Step 5: erosion by the agent radius. Set to null to skip.</summary>
    public Action<CompactHeightfield, SpanSettings>? Erode { get; set; } = static (chf, s) => Erosion.ErodeWalkableArea(chf, s.WalkableRadius);

    /// <summary>Step 6: area volumes, applied in list order after erosion.</summary>
    public IList<ConvexVolume> Volumes { get; } = new List<ConvexVolume>();

    /// <summary>Runs steps 1 to 6 on an already rasterized heightfield (which the filters modify).</summary>
    /// <exception cref="InvalidOperationException">Compact is null or returns null.</exception>
    public CompactHeightfield Run(SolidHeightfield solid)
    {
        ArgumentNullException.ThrowIfNull(solid);
        Settings.Validate();
        LowObstacleFilter?.Invoke(solid, Settings);
        LedgeFilter?.Invoke(solid, Settings);
        LowHeightFilter?.Invoke(solid, Settings);
        CompactHeightfield chf = Compact?.Invoke(solid, Settings) ?? throw new InvalidOperationException("The Compact step is missing or returned null.");
        Erode?.Invoke(chf, Settings);
        foreach (ConvexVolume v in Volumes)
        {
            v.Apply(chf);
        }
        return chf;
    }

    /// <summary>Rasterizes triangle soup (classified by slope) on <paramref name="frame"/> and runs the pipeline.</summary>
    /// <param name="frame">The grid.</param>
    /// <param name="vertices">x, y, z world coordinates, three doubles per vertex.</param>
    /// <param name="indices">Three vertex indices per triangle, counter-clockwise seen from the side they face.</param>
    /// <exception cref="ArgumentException">Invalid geometry; see <see cref="Voxelizer.Rasterize(SolidHeightfield, ReadOnlySpan{double}, ReadOnlySpan{int}, SpanSettings)"/>.</exception>
    public CompactHeightfield BuildFromTriangles(HeightfieldFrame frame, ReadOnlySpan<double> vertices, ReadOnlySpan<int> indices)
    {
        var solid = new SolidHeightfield(frame);
        Voxelizer.Rasterize(solid, vertices, indices, Settings);
        return Run(solid);
    }

    /// <summary>Adds a voxel chunk at the frame's origin (see <see cref="Voxelizer.AddChunk"/>) and runs the pipeline.</summary>
    public CompactHeightfield BuildFromChunk(HeightfieldFrame frame, VoxelChunk chunk, ReadOnlySpan<byte> materialAreas = default)
    {
        var solid = new SolidHeightfield(frame);
        Voxelizer.AddChunk(solid, chunk, 0, 0, 0, materialAreas);
        return Run(solid);
    }
}
