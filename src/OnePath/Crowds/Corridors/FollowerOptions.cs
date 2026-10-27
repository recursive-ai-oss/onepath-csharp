using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Corridors;

/// <summary>Who moves an agent across an off-mesh link.</summary>
public enum OffMeshMode : byte
{
    /// <summary>The follower moves the agent in a straight line from the link's entry to its exit over a fixed duration (length ÷ <see cref="FollowerOptions.OffMeshSpeed"/>) counted from the dt passed to each step.</summary>
    Automatic = 0,

    /// <summary>The host animates the crossing; the follower waits in <see cref="FollowerState.OffMeshLink"/> until <see cref="PathFollower.CompleteOffMeshLink"/> is called.</summary>
    Host = 1,
}

/// <summary>
/// Tuning of a <see cref="PathFollower"/>. Lengths are world units, times seconds (the sum of the dt values the host passes),
/// intervals and budgets are counts of steps and search iterations, never wall-clock time. Use a record <c>with</c>
/// expression to change fields.
/// </summary>
public sealed record FollowerOptions
{
    /// <summary>A person walking: speed 3.5, acceleration 8, slowing over the last metre.</summary>
    public static FollowerOptions Humanoid { get; } = new();

    /// <summary>A small creature: speed 2.5, acceleration 6, slowing over the last 0.5.</summary>
    public static FollowerOptions Small { get; } = new() { MaxSpeed = 2.5, MaxAcceleration = 6, SlowRadius = 0.5, OffMeshSpeed = 2.5 };

    /// <summary>Highest speed. Default 3.5; positive and finite.</summary>
    public double MaxSpeed { get; init; } = 3.5;

    /// <summary>Largest change of velocity per second, in both speeding up and slowing down. Default 8; positive (infinity means velocity changes at once).</summary>
    public double MaxAcceleration { get; init; } = 8;

    /// <summary>Arrival slowing: within this path distance of the end (or of an off-mesh link) the wanted speed falls in proportion to the distance (Reynolds' arrival). Default 1; 0 turns slowing off.</summary>
    public double SlowRadius { get; init; } = 1;

    /// <summary>Lowest wanted speed while slowing, so the last stretch takes finite time. Default 0.25; at least 0 and not above <see cref="MaxSpeed"/>.</summary>
    public double ArrivalMinSpeed { get; init; } = 0.25;

    /// <summary>The agent has arrived when it is this close (horizontally) to the end point. Default 0.01; at least 0. A step that can reach the end lands on it exactly.</summary>
    public double ArrivalRadius { get; init; } = 0.01;

    /// <summary>Size of the corner buffer, 2 to 64. Default 8. Slowing for the end needs the end within these corners.</summary>
    public int MaxCorners { get; init; } = 8;

    /// <summary>Polygons of the corridor checked against the mesh every step, at least 1. Default 8.</summary>
    public int ValidateLookAhead { get; init; } = 8;

    /// <summary>Every this many steps a visibility shortcut is tried; 0 turns it off. Default 4.</summary>
    public int OptimizeInterval { get; init; } = 4;

    /// <summary>Longest shortcut ray (horizontal distance). Default 10; positive.</summary>
    public double OptimizeRange { get; init; } = 10;

    /// <summary>Most polygons a shortcut ray considers (see <see cref="PathCorridor.OptimizeVisibility"/>), at least 2. Default 16.</summary>
    public int OptimizeBudget { get; init; } = 16;

    /// <summary>Every this many steps a local topology optimization is tried; 0 turns it off. Default 16.</summary>
    public int TopologyInterval { get; init; } = 16;

    /// <summary>Search iterations per topology optimization, at least 1. Default 32.</summary>
    public int TopologyBudget { get; init; } = 32;

    /// <summary>Search iterations per replan call (one call per step while a replan is pending), at least 1. Default 2048.</summary>
    public int ReplanBudget { get; init; } = 2048;

    /// <summary>Replan calls that end without finishing before the follower gives up with <see cref="FollowerState.NoPath"/>, at least 1. Default 16.</summary>
    public int MaxReplanRounds { get; init; } = 16;

    /// <summary>Stuck check: the agent must cover at least <see cref="StuckProgress"/> of its full-speed distance in every window of this many seconds. Default 1; 0 turns stuck detection off.</summary>
    public double StuckWindow { get; init; } = 1;

    /// <summary>Fraction of full-speed travel (0 to 1) the agent must make per window. Default 0.1.</summary>
    public double StuckProgress { get; init; } = 0.1;

    /// <summary>Replans from the agent's polygon tried on a stuck agent before it stops in <see cref="FollowerState.Stuck"/>, at least 0. Default 2.</summary>
    public int MaxStuckRecoveries { get; init; } = 2;

    /// <summary>Who moves the agent across off-mesh links. Default <see cref="OffMeshMode.Automatic"/>.</summary>
    public OffMeshMode OffMeshMode { get; init; } = OffMeshMode.Automatic;

    /// <summary>Speed of an automatic off-mesh crossing (3D length ÷ this is its duration). Default 3.5; positive and finite.</summary>
    public double OffMeshSpeed { get; init; } = 3.5;

    /// <summary>The agent starts an off-mesh crossing when it is this close (horizontally) to the link's entry. Default 0.05; at least 0.</summary>
    public double OffMeshTriggerRadius { get; init; } = 0.05;

    /// <summary>Half-size of the box searched for the polygon nearest a position or target. Default (2, 4, 2).</summary>
    public Vector3D SearchExtents { get; init; } = new(2, 4, 2);

    /// <summary>Polygons the corridor can hold, 2 to <see cref="PathCorridor.MaxCapacity"/>. Default 256.</summary>
    public int CorridorCapacity { get; init; } = 256;

    /// <summary>Checks every field.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is out of range; the message names the field.</exception>
    public void Validate()
    {
        Check(nameof(MaxSpeed), MaxSpeed > 0 && double.IsFinite(MaxSpeed), MaxSpeed);
        Check(nameof(MaxAcceleration), MaxAcceleration > 0, MaxAcceleration);
        Check(nameof(SlowRadius), SlowRadius >= 0 && double.IsFinite(SlowRadius), SlowRadius);
        Check(nameof(ArrivalMinSpeed), ArrivalMinSpeed >= 0 && ArrivalMinSpeed <= MaxSpeed, ArrivalMinSpeed);
        Check(nameof(ArrivalRadius), ArrivalRadius >= 0 && double.IsFinite(ArrivalRadius), ArrivalRadius);
        Check(nameof(MaxCorners), MaxCorners is >= 2 and <= 64, MaxCorners);
        Check(nameof(ValidateLookAhead), ValidateLookAhead >= 1, ValidateLookAhead);
        Check(nameof(OptimizeInterval), OptimizeInterval >= 0, OptimizeInterval);
        Check(nameof(OptimizeRange), OptimizeRange > 0 && double.IsFinite(OptimizeRange), OptimizeRange);
        Check(nameof(OptimizeBudget), OptimizeBudget >= 2, OptimizeBudget);
        Check(nameof(TopologyInterval), TopologyInterval >= 0, TopologyInterval);
        Check(nameof(TopologyBudget), TopologyBudget >= 1, TopologyBudget);
        Check(nameof(ReplanBudget), ReplanBudget >= 1, ReplanBudget);
        Check(nameof(MaxReplanRounds), MaxReplanRounds >= 1, MaxReplanRounds);
        Check(nameof(StuckWindow), StuckWindow >= 0 && double.IsFinite(StuckWindow), StuckWindow);
        Check(nameof(StuckProgress), StuckProgress >= 0 && StuckProgress <= 1, StuckProgress);
        Check(nameof(MaxStuckRecoveries), MaxStuckRecoveries >= 0, MaxStuckRecoveries);
        Check(nameof(OffMeshSpeed), OffMeshSpeed > 0 && double.IsFinite(OffMeshSpeed), OffMeshSpeed);
        Check(nameof(OffMeshTriggerRadius), OffMeshTriggerRadius >= 0 && double.IsFinite(OffMeshTriggerRadius), OffMeshTriggerRadius);
        Check(nameof(SearchExtents), double.IsFinite(SearchExtents.X) && double.IsFinite(SearchExtents.Y) && double.IsFinite(SearchExtents.Z) && SearchExtents.X >= 0 && SearchExtents.Y >= 0 && SearchExtents.Z >= 0, SearchExtents);
        Check(nameof(CorridorCapacity), CorridorCapacity is >= 2 and <= PathCorridor.MaxCapacity, CorridorCapacity);
        Check(nameof(OffMeshMode), OffMeshMode is OffMeshMode.Automatic or OffMeshMode.Host, OffMeshMode);
    }

    private static void Check(string name, bool ok, object value)
    {
        if (!ok)
        {
            throw new ArgumentOutOfRangeException(name, value, $"{name} is out of range.");
        }
    }
}
