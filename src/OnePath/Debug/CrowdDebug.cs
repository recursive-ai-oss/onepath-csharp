using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Debug;

/// <summary>An avoidance constraint of one agent in its last step: the velocities <c>v</c> with <c>(v - Point) . Normal &gt;= 0</c> are allowed (velocity-space, X/Z plane).</summary>
public readonly struct DebugConstraint
{
    internal DebugConstraint(HalfPlane h)
    {
        PointX = h.Point.X;
        PointZ = h.Point.Y;
        NormalX = h.Normal.X;
        NormalZ = h.Normal.Y;
    }

    /// <summary>X of a point on the boundary line.</summary>
    public double PointX { get; }

    /// <summary>Z of a point on the boundary line.</summary>
    public double PointZ { get; }

    /// <summary>X of the unit normal pointing into the allowed side.</summary>
    public double NormalX { get; }

    /// <summary>Z of the unit normal pointing into the allowed side.</summary>
    public double NormalZ { get; }
}

/// <summary>Everything a host needs to draw one agent of a crowd, as plain data.</summary>
public sealed class DebugAgent
{
    internal DebugAgent(long id, double radius, Vector3D preferredVelocity, FollowerDebugGeometry path, IReadOnlyList<WallSegment> walls, IReadOnlyList<DebugConstraint> constraints)
    {
        Id = id;
        Radius = radius;
        PreferredVelocity = preferredVelocity;
        Path = path;
        Walls = walls;
        Constraints = constraints;
    }

    /// <summary>The agent's id.</summary>
    public long Id { get; }

    /// <summary>Body radius between agents.</summary>
    public double Radius { get; }

    /// <summary>The velocity the agent wanted in its last step (after separation), before avoidance; Y is 0.</summary>
    public Vector3D PreferredVelocity { get; }

    /// <summary>State, position, velocity, destination, corridor outlines and corners.</summary>
    public FollowerDebugGeometry Path { get; }

    /// <summary>The wall pieces looked up around the agent (the nearest first within the lookup radius).</summary>
    public IReadOnlyList<WallSegment> Walls { get; }

    /// <summary>The avoidance constraints of the last step (walls first, then neighbours); empty unless <see cref="CrowdOptions.RecordConstraints"/> is on.</summary>
    public IReadOnlyList<DebugConstraint> Constraints { get; }
}

/// <summary>Plain-data description of a crowd's agents.</summary>
public sealed class CrowdDebugGeometry
{
    internal CrowdDebugGeometry(IReadOnlyList<DebugAgent> agents) => Agents = agents;

    /// <summary>The agents in slot order.</summary>
    public IReadOnlyList<DebugAgent> Agents { get; }
}

/// <summary>Collects debug geometry from a <see cref="Crowd"/>.</summary>
public static class CrowdDebug
{
    /// <summary>Collects every agent's state, corridor, corners, nearby walls and (if recorded) constraints. Allocates; for debug drawing, not the hot path. Call between steps.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="crowd"/> is null.</exception>
    public static CrowdDebugGeometry Collect(Crowd crowd)
    {
        ArgumentNullException.ThrowIfNull(crowd);
        var list = new List<DebugAgent>();
        NavMeshQuery q = crowd.Query;
        for (int i = 0; i < crowd.SlotCount; i++)
        {
            if (crowd.AgentAt(i) is not { } a)
            {
                continue;
            }
            (double px, double pz) = crowd.PreferredAt(i);
            var walls = new List<WallSegment>();
            foreach (WallSegment w in crowd.WallsAt(i))
            {
                walls.Add(w);
            }
            var cons = new List<DebugConstraint>();
            foreach (HalfPlane h in crowd.ConstraintsAt(i))
            {
                cons.Add(new DebugConstraint(h));
            }
            list.Add(new DebugAgent(a.Id, a.Options.Radius, new Vector3D(px, 0, pz), FollowerDebug.Collect(q, a.Path), walls, cons));
        }
        return new CrowdDebugGeometry(list);
    }
}
