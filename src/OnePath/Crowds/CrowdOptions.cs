using System;
using RecursiveAI.OnePath.Crowds.Corridors;

namespace RecursiveAI.OnePath;

/// <summary>How carefully an agent avoids others: how many neighbours it considers and how far ahead it looks.</summary>
public enum AvoidanceQuality : byte
{
    /// <summary>No avoidance: the agent walks its path and ignores agents and the wall constraints (it still stays on the mesh).</summary>
    Off = 0,

    /// <summary>4 neighbours, look ahead 1.5 s: cheapest, fine for sparse crowds.</summary>
    Low = 1,

    /// <summary>8 neighbours, look ahead 2 s.</summary>
    Medium = 2,

    /// <summary>12 neighbours, look ahead 2.5 s: for dense crowds and narrow passages.</summary>
    High = 3,
}

/// <summary>
/// Settings of one agent in a <see cref="Crowd"/>. Presets: <see cref="Agent.Humanoid"/>, <see cref="Agent.Small"/>,
/// <see cref="Agent.Large"/>; everything can be overridden with <c>with { ... }</c>. Lengths are in world units (a unit is a
/// metre), speeds in units per second. The agent's walkable surface is the mesh as built: a mesh built for an agent radius is
/// already shrunk by it, so <see cref="Radius"/> only matters between agents.
/// </summary>
public sealed record AgentOptions
{
    /// <summary>Body radius used between agents (two agents keep their centres apart by the sum of their radii). Default 0.6; above 0 and below 1000.</summary>
    public double Radius { get; init => field = Require.That(value > 0 && value < 1000, value, nameof(Radius), "above 0 and below 1000"); } = 0.6;

    /// <summary>Top speed. Default 3.5; above 0.</summary>
    public double MaxSpeed { get; init => field = Require.That(value > 0 && double.IsFinite(value) && value < 1e6, value, nameof(MaxSpeed), "above 0 and below 1,000,000"); } = 3.5;

    /// <summary>Largest change of velocity per second. Default 8; above 0.</summary>
    public double MaxAcceleration { get; init => field = Require.That(value > 0 && double.IsFinite(value), value, nameof(MaxAcceleration), "above 0 and finite"); } = 8;

    /// <summary>Distance from the destination within which the agent has arrived. Default 0.25; 0 or more.</summary>
    public double ArrivalRadius { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(ArrivalRadius), "0 or more and finite"); } = 0.25;

    /// <summary>
    /// Weight of this agent in avoidance: of two agents that must give way, the one with the greater priority gives way less (its share
    /// of the evasion is the other's weight over the sum). Default 1; above 0.
    /// </summary>
    public double Priority { get; init => field = Require.That(value > 0 && double.IsFinite(value), value, nameof(Priority), "above 0 and finite"); } = 1;

    /// <summary>Avoidance quality preset. Default <see cref="AvoidanceQuality.Medium"/>.</summary>
    public AvoidanceQuality Quality { get; init => field = Require.That(value <= AvoidanceQuality.High, value, nameof(Quality), "a defined AvoidanceQuality"); } = AvoidanceQuality.Medium;

    /// <summary>
    /// Distance from the wall (a mesh boundary edge) the agent keeps in avoidance. Default 0.05. The mesh is already shrunk by the
    /// agent radius it was built for, so this is a small margin, not the body radius. 0 or more.
    /// </summary>
    public double WallClearance { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(WallClearance), "0 or more and finite"); } = 0.05;

    /// <summary>
    /// Separation: neighbours closer than this (centre to centre) push the agent away, in proportion to
    /// <see cref="SeparationWeight"/>. 0 turns separation off (the default). Spreads agents standing around a goal.
    /// </summary>
    public double SeparationDistance { get; init => field = Require.That(value >= 0 && double.IsFinite(value), value, nameof(SeparationDistance), "0 or more and finite"); }

    /// <summary>Separation strength: at touching distance a neighbour adds this fraction of <see cref="MaxSpeed"/> to the wanted velocity. Default 0.3; 0 to 1.</summary>
    public double SeparationWeight { get; init => field = Require.That(value >= 0 && value <= 1, value, nameof(SeparationWeight), "0 to 1"); } = 0.3;

    /// <summary>
    /// How far the wanted velocity is turned to one fixed side (the same for every agent) to break the symmetry of agents meeting head
    /// on, which exact avoidance otherwise resolves by stopping: the wanted velocity gets a sideways part of this fraction of its length.
    /// Default 0.05 (about 3 degrees); 0 turns it off; 0 to 0.5.
    /// </summary>
    public double SidePreference { get; init => field = Require.That(value >= 0 && value <= 0.5, value, nameof(SidePreference), "0 to 0.5"); } = 0.05;

    /// <summary>
    /// Settings of the path corridor and path follower (tier 2); null means <see cref="FollowerOptions.Humanoid"/>. The speed,
    /// acceleration and arrival radius above replace the same fields of this record, and stuck detection is off unless this sets it.
    /// </summary>
    public FollowerOptions? Path { get; init; }

    /// <summary>Neighbours considered by avoidance for <see cref="Quality"/>.</summary>
    internal int MaxNeighbors => Quality switch { AvoidanceQuality.Low => 4, AvoidanceQuality.Medium => 8, AvoidanceQuality.High => 12, _ => 0 };

    /// <summary>Look-ahead time of avoidance between agents for <see cref="Quality"/>.</summary>
    internal double TimeHorizon => Quality switch { AvoidanceQuality.Low => 1.5, AvoidanceQuality.Medium => 2.0, _ => 2.5 };

    /// <summary>The follower settings this agent uses.</summary>
    internal FollowerOptions ToFollowerOptions()
    {
        FollowerOptions f = Path ?? FollowerOptions.Humanoid;
        return f with
        {
            MaxSpeed = MaxSpeed,
            MaxAcceleration = MaxAcceleration,
            ArrivalRadius = ArrivalRadius,
            // Agents queue at a link: they start the crossing within their body diameter of its entry (a link end is attached to the mesh within a few radii), not within a hair of it.
            OffMeshTriggerRadius = Path is null ? Math.Max(f.OffMeshTriggerRadius, 2 * Radius) : f.OffMeshTriggerRadius,
            // Avoidance keeps agents WallClearance off walls and corners lie on wall vertices: the agent aims a little outside that.
            CornerClearance = Path is null ? (2 * WallClearance) + 0.01 : f.CornerClearance,
            StuckWindow = Path is null ? 0 : f.StuckWindow,
            CorridorCapacity = Path is null ? 64 : f.CorridorCapacity,
        };
    }

    /// <summary>Throws <see cref="ArgumentOutOfRangeException"/> naming the first field out of range.</summary>
    public void Validate()
    {
        Check(nameof(Radius), Radius > 0 && Radius < 1000, Radius);
        Check(nameof(MaxSpeed), MaxSpeed > 0 && double.IsFinite(MaxSpeed) && MaxSpeed < 1e6, MaxSpeed);
        Check(nameof(MaxAcceleration), MaxAcceleration > 0 && double.IsFinite(MaxAcceleration), MaxAcceleration);
        Check(nameof(ArrivalRadius), ArrivalRadius >= 0 && double.IsFinite(ArrivalRadius), ArrivalRadius);
        Check(nameof(Priority), Priority > 0 && double.IsFinite(Priority), Priority);
        Check(nameof(Quality), Quality <= AvoidanceQuality.High, Quality);
        Check(nameof(WallClearance), WallClearance >= 0 && double.IsFinite(WallClearance), WallClearance);
        Check(nameof(SeparationDistance), SeparationDistance >= 0 && double.IsFinite(SeparationDistance), SeparationDistance);
        Check(nameof(SeparationWeight), SeparationWeight >= 0 && SeparationWeight <= 1, SeparationWeight);
        Check(nameof(SidePreference), SidePreference >= 0 && SidePreference <= 0.5, SidePreference);
        ToFollowerOptions().Validate();
    }

    private static void Check(string name, bool ok, object value)
    {
        if (!ok)
        {
            throw new ArgumentOutOfRangeException(name, value, $"{name} is out of range.");
        }
    }
}

/// <summary>Settings of a <see cref="Crowd"/> as a whole.</summary>
public sealed record CrowdOptions
{
    /// <summary>The default settings.</summary>
    public static CrowdOptions Default { get; } = new();

    /// <summary>Most agents at once. Default 65536; 1 to 4,000,000.</summary>
    public int MaxAgents { get; init => field = Require.That(value is >= 1 and <= 4_000_000, value, nameof(MaxAgents), "1 to 4,000,000"); } = 65536;

    /// <summary>
    /// Threads a step may use. Default 1. The result of a step is the same for any value (agents are split into fixed groups by
    /// index and every agent writes only its own state), so this is only a speed setting. At least 1.
    /// </summary>
    public int MaxThreads { get; init => field = Require.That(value >= 1, value, nameof(MaxThreads), "at least 1"); } = 1;

    /// <summary>Which polygons agents may walk and what areas cost; null accepts everything at cost 1. Read, never changed, by the crowd.</summary>
    public QueryFilter? Filter { get; init; }

    /// <summary>Nodes of each query object used for path searches and wall searches. Default 16384; 64 to 4,000,000.</summary>
    public int QueryNodes { get; init => field = Require.That(value is >= 64 and <= 4_000_000, value, nameof(QueryNodes), "64 to 4,000,000"); } = 16384;

    /// <summary>
    /// What one search node stands for in the crowd's path searches (corridors and replans). Default <see cref="PathNodeModel.Crossing"/>:
    /// its corridors keep agents off door jambs. With <see cref="PathNodeModel.Polygon"/> (the default of a bare query object)
    /// a lone agent can stall beside a jamb in a narrow doorway, steered at the corner and pushed off it by the wall.
    /// </summary>
    public PathNodeModel QueryNodeModel { get; init => field = Require.That(value is PathNodeModel.Crossing or PathNodeModel.Polygon, value, nameof(QueryNodeModel), "Crossing or Polygon"); } = PathNodeModel.Crossing;

    /// <summary>
    /// Wall pieces kept near each agent. The walls are looked up when the agent has moved farther than
    /// <see cref="WallRefreshDistance"/> from where they were looked up, or the mesh changed. Default 48; 4 to 1024.
    /// </summary>
    public int MaxWallsPerAgent { get; init => field = Require.That(value is >= 4 and <= 1024, value, nameof(MaxWallsPerAgent), "4 to 1024"); } = 48;

    /// <summary>How far an agent may move before its walls are looked up again. Default 1; above 0.</summary>
    public double WallRefreshDistance { get; init => field = Require.That(value > 0 && double.IsFinite(value), value, nameof(WallRefreshDistance), "above 0 and finite"); } = 1;

    /// <summary>Wall pieces an agent avoids at once (the nearest). Default 8; 1 to 64.</summary>
    public int MaxWallsAvoided { get; init => field = Require.That(value is >= 1 and <= 64, value, nameof(MaxWallsAvoided), "1 to 64"); } = 8;

    /// <summary>Keep the half-planes each agent solved in its last step for <see cref="RecursiveAI.OnePath.Debug.CrowdDebug"/>. Default false (it costs time and memory).</summary>
    public bool RecordConstraints { get; init; }

    /// <summary>Throws <see cref="ArgumentOutOfRangeException"/> naming the first field out of range.</summary>
    public void Validate()
    {
        Check(nameof(MaxAgents), MaxAgents is >= 1 and <= 4_000_000, MaxAgents);
        Check(nameof(MaxThreads), MaxThreads >= 1, MaxThreads);
        Check(nameof(QueryNodes), QueryNodes is >= 64 and <= 4_000_000, QueryNodes);
        Check(nameof(QueryNodeModel), QueryNodeModel is PathNodeModel.Crossing or PathNodeModel.Polygon, QueryNodeModel);
        Check(nameof(MaxWallsPerAgent), MaxWallsPerAgent is >= 4 and <= 1024, MaxWallsPerAgent);
        Check(nameof(WallRefreshDistance), WallRefreshDistance > 0 && double.IsFinite(WallRefreshDistance), WallRefreshDistance);
        Check(nameof(MaxWallsAvoided), MaxWallsAvoided is >= 1 and <= 64, MaxWallsAvoided);
    }

    private static void Check(string name, bool ok, object value)
    {
        if (!ok)
        {
            throw new ArgumentOutOfRangeException(name, value, $"{name} is out of range.");
        }
    }
}
