using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath;

/// <summary>
/// An authored off-mesh link: a jump, ladder, door or teleport that joins two places the polygon surface does not.
/// Give a list of them in <see cref="NavSettings.OffMeshLinks"/>; every build bakes the ones that find walkable ground into the tile
/// that contains their start, so rebuilt tiles (obstacles, volumes, streaming) keep them. Positions are world coordinates (double,
/// within ±2^21). A link is resolved to polygons when tiles are stitched: its start must be within <see cref="Radius"/> of a polygon of its
/// own tile column and its end within <see cref="Radius"/> of a polygon of the end's column (both also within the agent's climb in height);
/// a link that finds no polygon at either end stays unconnected and costs nothing.
/// </summary>
/// <param name="Start">Where the link begins (world).</param>
/// <param name="End">Where the link ends (world).</param>
/// <param name="Radius">How far from a polygon an end may be and still attach to it; positive and finite. Default 0.5 when built with <see cref="Create"/>.</param>
/// <param name="Bidirectional">True if the link can also be crossed from the end to the start.</param>
/// <param name="Area">Area id given to the link, 0 to 63.</param>
/// <param name="Flags">Flags given to the link, for query filters.</param>
/// <param name="UserId">A number carried through unchanged; the host reads it back to know which link an agent is crossing.</param>
public readonly record struct OffMeshLinkDefinition(Vector3D Start, Vector3D End, double Radius, bool Bidirectional, byte Area, ushort Flags, uint UserId)
{
    /// <summary>A two-way link of radius 0.5, area 0 and flag 1 (walk).</summary>
    /// <param name="start">Where the link begins (world).</param>
    /// <param name="end">Where the link ends (world).</param>
    /// <param name="userId">A caller number carried through unchanged.</param>
    public static OffMeshLinkDefinition Create(Vector3D start, Vector3D end, uint userId = 0) => new(start, end, 0.5, true, 0, 1, userId);

    /// <summary>The same link, one-way (start to end only).</summary>
    public OffMeshLinkDefinition OneWay() => this with { Bidirectional = false };
}
