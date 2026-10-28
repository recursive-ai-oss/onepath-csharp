using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>Options of the one-call path query (tier 1). Immutable once constructed: set the properties in an object initializer.</summary>
public sealed record PathOptions
{
    /// <summary>Which polygons may be used and what they cost; null means everything at cost 1.</summary>
    public QueryFilter? Filter { get; init; }

    /// <summary>Half extents of the box around the start and end positions that is searched for the nearest polygon. Default (2, 4, 2).</summary>
    public Vector3D SearchExtents { get; init; } = new(2, 4, 2);

    /// <summary>Largest corridor length returned, at least 1; longer corridors are cut. Default 4096.</summary>
    public int MaxCorridor { get; init => field = Require.That(value >= 1, value, nameof(MaxCorridor), "at least 1"); } = 4096;

    /// <summary>Largest number of straight-path points, at least 2. Default 4096.</summary>
    public int MaxPoints { get; init => field = Require.That(value >= 2, value, nameof(MaxPoints), "at least 2"); } = 4096;

    /// <summary>
    /// Expansions <see cref="NavMesh.FindPathAsync"/> searches between yields
    /// to the caller's context. Counts expansions, never time, and never changes the result. At least 1. Default 256.
    /// </summary>
    public int IterationsPerSlice { get; init => field = Require.That(value >= 1, value, nameof(IterationsPerSlice), "at least 1"); } = 256;
}
