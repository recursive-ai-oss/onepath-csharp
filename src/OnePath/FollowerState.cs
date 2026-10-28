namespace RecursiveAI.OnePath;

/// <summary>What a <see cref="Crowds.Corridors.PathFollower"/> is doing.</summary>
public enum FollowerState : byte
{
    /// <summary>Standing on the mesh with no destination (or stopped).</summary>
    Idle = 0,

    /// <summary>Walking its corridor toward the destination (this includes waiting at the end of a corridor that a replan is still extending).</summary>
    Moving = 1,

    /// <summary>Crossing an off-mesh link: automatically over a fixed duration, or until the host calls <see cref="Crowds.Corridors.PathFollower.CompleteOffMeshLink"/>.</summary>
    OffMeshLink = 2,

    /// <summary>Stopped at the end of its corridor; <see cref="Crowds.Corridors.PathFollower.ReachedTarget"/> tells whether that is the destination.</summary>
    Arrived = 3,

    /// <summary>Gave up after making no progress through every allowed recovery.</summary>
    Stuck = 4,

    /// <summary>No route could be started or kept: no polygon near the destination, the agent was lost off the mesh, or replanning kept running out of budget.</summary>
    NoPath = 5,
}
