namespace RecursiveAI.OnePath;

/// <summary>How the walkable spans are partitioned into regions.</summary>
public enum PartitionType
{
    /// <summary>Watershed on the distance field: best shapes, slowest. Regions grow outward from the points farthest from any wall.</summary>
    Watershed = 0,

    /// <summary>Monotone sweep along z: fast, long thin regions, no distance field.</summary>
    Monotone = 1,

    /// <summary>Monotone sweep, then regions that do not overlap in any column are merged into layers: for multi-floor levels.</summary>
    Layers = 2,
}
