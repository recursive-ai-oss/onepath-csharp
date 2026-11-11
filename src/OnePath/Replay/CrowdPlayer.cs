using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Replay;

/// <summary>
/// Rebuilds a recorded crowd from a replay's setup bytes and plays operations and steps into it, the way the host ran them. The
/// crowd's <see cref="Crowd.StateHash"/> after each step equals the hash the recording holds. One thread at a time.
/// </summary>
public sealed class CrowdPlayer
{
    private readonly List<CrowdOp> _scratch = [];

    /// <summary>Builds the mesh and crowd from a setup block (see docs/replay.md) and puts the crowd at the setup's step.</summary>
    /// <param name="setup">The setup bytes of a <see cref="CrowdReplay"/>, or the live snapshot a <see cref="CrowdRecorder"/> wrote. Untrusted: every length is checked before it is used.</param>
    /// <exception cref="CrowdReplayException">The setup is malformed, or its mesh or crowd state is rejected; the message says what and where.</exception>
    public CrowdPlayer(ReadOnlySpan<byte> setup)
    {
        CrowdSetup.Parsed p = CrowdSetup.Parse(setup);
        Dt = p.Dt;
        Options = p.Options;
        try
        {
            TiledNavMesh mesh = NavMeshSnapshot.Restore(p.Mesh.Span);
            Crowd = new Crowd(mesh, p.Options);
            Crowd.RestoreState(p.State.Span);
        }
        catch (OnePathFormatException e) when (e is not CrowdReplayException)
        {
            throw new CrowdReplayException($"Invalid replay setup: {e.Message}", e);
        }

        if (Crowd.StepCount != p.StartStep)
        {
            throw new CrowdReplayException($"Invalid replay setup: the setup says step {p.StartStep} but its crowd state is at step {Crowd.StepCount}.");
        }
    }

    /// <summary>The crowd. Read it freely; change it only through <see cref="Apply(in CrowdOp)"/> and <see cref="Step"/> so it stays equal to the recording.</summary>
    public Crowd Crowd { get; }

    /// <summary>The seconds each <see cref="Step"/> advances the crowd: finite, above 0 and at most 3600.</summary>
    public double Dt { get; }

    /// <summary>The crowd settings the recording used (the filter is not recorded; the crowd uses none).</summary>
    public CrowdOptions Options { get; }

    /// <summary>Applies one operation. Returns false when an add found no room or no walkable polygon (the recording's host saw the same).</summary>
    /// <exception cref="CrowdReplayException">The operation targets an agent that is not in the crowd.</exception>
    public bool Apply(in CrowdOp op) => Apply(op, out _);

    /// <summary>Applies one operation like <see cref="Apply(in CrowdOp)"/> and reports the agent it concerned.</summary>
    /// <param name="op">The operation.</param>
    /// <param name="agentId">For an add that succeeded, the new agent's id; for other kinds, <see cref="CrowdOp.AgentId"/>; 0 for an add that failed.</param>
    /// <exception cref="CrowdReplayException">The operation targets an agent that is not in the crowd.</exception>
    public bool Apply(in CrowdOp op, out long agentId)
    {
        agentId = op.AgentId;
        if (op.Kind == CrowdOpKind.Add)
        {
            bool added = Crowd.TryAdd(op.Position, op.Options, out Agent? created);
            agentId = created?.Id ?? 0;
            return added;
        }

        Agent agent = Crowd.GetAgent(op.AgentId) ?? throw new CrowdReplayException($"Invalid replay step: {op.Kind} names agent {op.AgentId}, which is not in the crowd at step {Crowd.StepCount}.");
        switch (op.Kind)
        {
            case CrowdOpKind.Remove:
                return Crowd.Remove(agent);
            case CrowdOpKind.SetDestination:
                agent.SetDestination(op.Position);
                return true;
            case CrowdOpKind.Stop:
                agent.Stop();
                return true;
            case CrowdOpKind.Teleport:
                return agent.Teleport(op.Position);
            default:
                throw new CrowdReplayException($"Invalid replay step: unknown operation kind {(byte)op.Kind}.");
        }
    }

    /// <summary>Parses and applies one step's operation bytes in order. Nothing is applied when the bytes do not parse; an operation that cannot apply stops the rest.</summary>
    /// <exception cref="CrowdReplayException">The bytes are malformed (see <see cref="CrowdOps.Parse"/>) or an operation names an agent that is not there.</exception>
    public void Apply(ReadOnlySpan<byte> ops)
    {
        CrowdOps.Parse(ops, _scratch);
        for (int i = 0; i < _scratch.Count; i++)
        {
            Apply(_scratch[i]);
        }
    }

    /// <summary>Advances the crowd by <see cref="Dt"/>.</summary>
    public void Step() => Crowd.Step(Dt);
}
