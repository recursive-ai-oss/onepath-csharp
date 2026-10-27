using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Debug;

/// <summary>Plain-data picture of one path follower for a host to draw: where it is and moves, its corridor and its corners.</summary>
public sealed class FollowerDebugGeometry
{
    internal FollowerDebugGeometry(FollowerState state, Vector3D position, Vector3D velocity, Vector3D destination, IReadOnlyList<DebugPolygon> corridor, IReadOnlyList<StraightPathPoint> corners, bool offMesh, Vector3D offStart, Vector3D offEnd)
    {
        State = state;
        Position = position;
        Velocity = velocity;
        Destination = destination;
        Corridor = corridor;
        Corners = corners;
        OffMeshActive = offMesh;
        OffMeshStart = offStart;
        OffMeshEnd = offEnd;
    }

    /// <summary>What the follower is doing.</summary>
    public FollowerState State { get; }

    /// <summary>Where it is.</summary>
    public Vector3D Position { get; }

    /// <summary>Its velocity in the last step.</summary>
    public Vector3D Velocity { get; }

    /// <summary>The destination it was sent to.</summary>
    public Vector3D Destination { get; }

    /// <summary>Outlines of the corridor polygons, the agent's polygon first (stale references are left out).</summary>
    public IReadOnlyList<DebugPolygon> Corridor { get; }

    /// <summary>The corners found by the last step (the first is the agent's position).</summary>
    public IReadOnlyList<StraightPathPoint> Corners { get; }

    /// <summary>True while crossing an off-mesh link.</summary>
    public bool OffMeshActive { get; }

    /// <summary>Entry of the current off-mesh crossing (zero when not crossing).</summary>
    public Vector3D OffMeshStart { get; }

    /// <summary>Exit of the current off-mesh crossing (zero when not crossing).</summary>
    public Vector3D OffMeshEnd { get; }
}

/// <summary>Builds <see cref="FollowerDebugGeometry"/>.</summary>
public static class FollowerDebug
{
    /// <summary>Collects a follower's state, corridor outlines and corners. Allocates; meant for debug drawing, not for the hot path.</summary>
    /// <exception cref="ArgumentNullException">An argument is null.</exception>
    public static FollowerDebugGeometry Collect(NavMeshQuery query, PathFollower follower)
    {
        ArgumentNullException.ThrowIfNull(query);
        ArgumentNullException.ThrowIfNull(follower);
        var corridor = new List<DebugPolygon>();
        foreach (RecursiveAI.OnePath.Mesh.PolyRef p in follower.Corridor.Path)
        {
            if (QueryDebug.Outline(query.Mesh, p) is { } o)
            {
                corridor.Add(o);
            }
        }
        bool off = follower.State == FollowerState.OffMeshLink;
        return new FollowerDebugGeometry(follower.State, follower.Position, follower.Velocity, follower.Destination, corridor, follower.Corners.ToArray(), off, off ? follower.OffMeshStart : default, off ? follower.OffMeshEnd : default);
    }
}
