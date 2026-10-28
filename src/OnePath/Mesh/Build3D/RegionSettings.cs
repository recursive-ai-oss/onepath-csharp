using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>Parameters of the region stage. Areas are counted in spans (cells of one floor).</summary>
public sealed class RegionSettings
{
    /// <summary>The partition method (default <see cref="PartitionType.Watershed"/>).</summary>
    public PartitionType Partition { get; set; } = PartitionType.Watershed;

    /// <summary>Regions with fewer spans than this that touch no other region are removed (their spans become unregioned). Default 64. 0 disables.</summary>
    public int MinRegionArea { get; set; } = 64;

    /// <summary>Regions with fewer spans than this are merged into the same-area neighbour they share the longest boundary with. Default 400. 0 disables. Not applied to layers.</summary>
    public int MergeRegionArea { get; set; } = 400;

    /// <summary>Checks the values.</summary>
    /// <exception cref="ArgumentOutOfRangeException">An area is negative or the partition is not a defined value.</exception>
    public void Validate()
    {
        if (MinRegionArea < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(MinRegionArea), MinRegionArea, "The minimum region area must not be negative.");
        }
        if (MergeRegionArea < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(MergeRegionArea), MergeRegionArea, "The merge region area must not be negative.");
        }
        if (!Enum.IsDefined(Partition))
        {
            throw new ArgumentOutOfRangeException(nameof(Partition), Partition, "Unknown partition type.");
        }
    }
}

/// <summary>Parameters of the contour stage. Lengths are in cells.</summary>
public sealed class ContourSettings
{
    /// <summary>Largest distance, in cells, a raw boundary vertex may lie from the simplified contour. Default 1.3. 0 keeps every raw vertex that is not collinear-redundant.</summary>
    public float MaxError { get; set; } = 1.3f;

    /// <summary>Wall edges longer than this many cells are split. Default 12. 0 disables.</summary>
    public int MaxEdgeLength { get; set; } = 12;

    /// <summary>Region id of a tile's border band, or 0 for none. Stretches next to it are simplified like walls: their ends meet the neighbouring tile's polygons. Default 0.</summary>
    public int BorderRegion { get; set; }

    /// <summary>Leave a border shared with a region of the same area as the straight chord between its two ends (no refinement; walls and borders between areas are always refined). Default true. A loop that crosses itself because of this is simplified again with every border refined.</summary>
    public bool StraightRegionBorders { get; set; } = true;

    /// <summary>Keep the unsimplified loops in <see cref="Contour.RawVertices"/> (for checking and debugging). Default false.</summary>
    public bool KeepRawVertices { get; set; }

    /// <summary>Checks the values.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is negative, NaN or infinite.</exception>
    public void Validate()
    {
        if (!(MaxError >= 0f) || float.IsInfinity(MaxError))
        {
            throw new ArgumentOutOfRangeException(nameof(MaxError), MaxError, "The maximum error must be finite and not negative.");
        }
        if (MaxEdgeLength < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(MaxEdgeLength), MaxEdgeLength, "The maximum edge length must not be negative.");
        }
    }
}
