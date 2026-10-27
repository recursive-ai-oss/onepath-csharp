using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Orca;

/// <summary>
/// The state and parameters of one simulated agent. All lengths share one unit and all speeds are length per time.
/// Positions are in a single local frame; callers that work in tiles keep the frame small enough for their precision.
/// </summary>
public struct OrcaAgent
{
    /// <summary>Position of the agent's centre.</summary>
    public Vector2D Position { get; set; }

    /// <summary>Current velocity; it is also the optimization velocity other agents assume for this agent.</summary>
    public Vector2D Velocity { get; set; }

    /// <summary>The velocity the agent would like to have; set by the caller before each step.</summary>
    public Vector2D PreferredVelocity { get; set; }

    /// <summary>Radius of the agent's disc; at least 0.</summary>
    public double Radius { get; set; }

    /// <summary>Maximum speed (radius of the speed disc); at least 0.</summary>
    public double MaxSpeed { get; set; }

    /// <summary>Other agents whose centres are farther away than this are ignored; at least 0.</summary>
    public double NeighborDistance { get; set; }

    /// <summary>At most this many nearest neighbours are used (nearest first; ties by agent index); at least 0.</summary>
    public int MaxNeighbors { get; set; }

    /// <summary>Look-ahead time for other agents; greater than 0.</summary>
    public double TimeHorizon { get; set; }

    /// <summary>Look-ahead time for obstacles; greater than 0 (usually smaller than <see cref="TimeHorizon"/>).</summary>
    public double ObstacleTimeHorizon { get; set; }
}

/// <summary>A static line-segment obstacle (a navmesh boundary edge, a wall). It blocks from both sides.</summary>
public readonly struct OrcaSegment
{
    /// <summary>One end.</summary>
    public Vector2D A { get; }

    /// <summary>The other end; may equal <see cref="A"/> for a point obstacle.</summary>
    public Vector2D B { get; }

    /// <summary>Creates a segment.</summary>
    public OrcaSegment(Vector2D a, Vector2D b)
    {
        A = a;
        B = b;
    }
}
