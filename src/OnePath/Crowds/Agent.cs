using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>
/// One agent of a <see cref="Crowd"/>: it walks a path to a destination while avoiding walls and other agents. Created with
/// <see cref="Crowd.Add"/>; this type also holds the presets (<see cref="Humanoid"/>, <see cref="Small"/>, <see cref="Large"/>).
/// An agent is only valid while it is in its crowd; after <see cref="Crowd.Remove"/> its members throw
/// <see cref="InvalidOperationException"/> except <see cref="IsActive"/> and <see cref="Id"/>.
/// </summary>
public sealed class Agent
{
    private readonly Crowd _crowd;
    private int _slot;

    internal Agent(Crowd crowd, int slot, long id, AgentOptions options)
    {
        _crowd = crowd;
        _slot = slot;
        Id = id;
        Options = options;
    }

    /// <summary>A person: radius 0.6, speed 3.5, acceleration 8 (matches <see cref="NavSettings.Humanoid"/>).</summary>
    public static AgentOptions Humanoid { get; } = new();

    /// <summary>A small agent: radius 0.3, speed 2.5, acceleration 6, a slower and tighter walker (matches <see cref="NavSettings.Small"/>).</summary>
    public static AgentOptions Small { get; } = new() { Radius = 0.3, MaxSpeed = 2.5, MaxAcceleration = 6, ArrivalRadius = 0.3 };

    /// <summary>A large agent: radius 1.5, speed 3, acceleration 5 (matches <see cref="NavSettings.Large"/>).</summary>
    public static AgentOptions Large { get; } = new() { Radius = 1.5, MaxSpeed = 3, MaxAcceleration = 5, ArrivalRadius = 0.6 };

    /// <summary>A number that identifies the agent in its crowd for good: 0, 1, 2, ... in the order agents were added. Never reused.</summary>
    public long Id { get; }

    /// <summary>The settings the agent was added with.</summary>
    public AgentOptions Options { get; }

    /// <summary>True while the agent is in its crowd.</summary>
    public bool IsActive => _slot >= 0;

    internal int Slot => _slot;

    internal void Deactivate() => _slot = -1;

    private PathFollower Follower => _slot >= 0 ? _crowd.FollowerAt(_slot) : throw new InvalidOperationException("The agent was removed from its crowd.");

    /// <summary>Where the agent stands (world; always on the mesh).</summary>
    public Vector3D Position => Follower.Position;

    /// <summary>How fast and which way it moved in the last step (units per second, 3D).</summary>
    public Vector3D Velocity => Follower.Velocity;

    /// <summary>Where it is going (the position it stands at when it has no destination).</summary>
    public Vector3D Destination => Follower.Destination;

    /// <summary>True while the agent is walking a path (or crossing an off-mesh link).</summary>
    public bool HasPath => Follower.HasPath;

    /// <summary>True when it arrived and the path led to the destination (not just to the nearest reachable point).</summary>
    public bool ReachedTarget => Follower.ReachedTarget;

    /// <summary>Distance left along the path to the destination (or the next off-mesh link); infinity when not known yet.</summary>
    public double RemainingDistance => Follower.RemainingDistance;

    /// <summary>What the agent is doing.</summary>
    public FollowerState State => Follower.State;

    /// <summary>The path follower (tier 2), for corridor and corner access.</summary>
    public PathFollower Path => Follower;

    /// <summary>Weight in avoidance (see <see cref="AgentOptions.Priority"/>); may be changed at any time. Above 0.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is not above 0 and finite.</exception>
    public double Priority
    {
        get
        {
            _ = Follower;
            return _crowd.PriorityAt(_slot);
        }
        set
        {
            _ = Follower;
            if (!(value > 0) || !double.IsFinite(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "Priority must be above 0 and finite.");
            }
            _crowd.SetPriorityAt(_slot, value);
        }
    }

    /// <summary>
    /// Sends the agent to a world position: the route is searched at once and walked by the following steps. A destination off the
    /// mesh means its nearest point within the search extents; one with no walkable polygon near is refused.
    /// </summary>
    /// <returns><see cref="PathStatus.Found"/>, <see cref="PathStatus.Partial"/> (unreachable: it walks to the nearest point it can) or <see cref="PathStatus.InProgress"/> (long route, finished by the steps); <see cref="PathStatus.GoalInvalid"/> when nothing walkable is near the destination, <see cref="PathStatus.StartInvalid"/> when the agent is not on the mesh.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or is beyond ±2^21.</exception>
    /// <exception cref="InvalidOperationException">The agent was removed.</exception>
    public PathStatus SetDestination(Vector3D destination) => _crowd.SetDestinationAt(_slot >= 0 ? _slot : throw new InvalidOperationException("The agent was removed from its crowd."), destination).ToPathStatus();

    /// <summary>Stops the agent where it stands (it can still be pushed by others).</summary>
    public void Stop() => Follower.Stop();

    /// <summary>Puts the agent at the nearest walkable point to <paramref name="position"/>, standing still with no destination.</summary>
    /// <returns>False, changing nothing, if no walkable polygon is near.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or is beyond ±2^21.</exception>
    public bool Teleport(Vector3D position) => _crowd.TeleportAt(_slot >= 0 ? _slot : throw new InvalidOperationException("The agent was removed from its crowd."), position);

    /// <summary>Ends an off-mesh crossing the host animates (see <see cref="OffMeshMode.Host"/>).</summary>
    /// <exception cref="InvalidOperationException">The agent is not crossing an off-mesh link, or was removed.</exception>
    public void CompleteOffMeshLink() => _crowd.CompleteOffMeshAt(_slot >= 0 ? _slot : throw new InvalidOperationException("The agent was removed from its crowd."));
}
