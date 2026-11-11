using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Replay;

/// <summary>Receives one live record from a <see cref="CrowdRecorder"/>: see docs/replay.md, "Live records".</summary>
/// <param name="record">The record's bytes, valid only during the call.</param>
public delegate void CrowdRecordHandler(ReadOnlySpan<byte> record);

/// <summary>
/// Runs a host's crowd and records what the host does to it: add, remove, set destination, stop, teleport and step. Call these
/// methods instead of the crowd's own for everything that changes the crowd; a change made behind the recorder's back makes the recording
/// diverge. The recorder can keep the run as a <see cref="CrowdReplay"/> and can hand each change to a live viewer as it happens.
/// The mesh must not be edited while recording (<see cref="Step"/> throws if it was). One thread at a time.
/// </summary>
public sealed class CrowdRecorder
{
    private readonly List<ReadOnlyMemory<byte>>? _steps;
    private readonly List<ulong>? _hashes;
    private readonly byte[]? _setup;
    private readonly ArrayBufferWriter<byte> _ops = new();
    private readonly ArrayBufferWriter<byte> _record = new();
    private readonly long _meshEdits;

    /// <summary>Starts recording <paramref name="crowd"/> as it is now.</summary>
    /// <param name="crowd">The crowd. Agents already in it are part of the setup.</param>
    /// <param name="dt">The seconds each <see cref="Step"/> advances the crowd: finite, above 0 and at most 3600.</param>
    /// <param name="keepSteps">True to keep every step in memory so <see cref="ToReplay"/> works; false for a live-only recorder whose memory does not grow.</param>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is out of range.</exception>
    public CrowdRecorder(Crowd crowd, double dt, bool keepSteps = false)
    {
        ArgumentNullException.ThrowIfNull(crowd);
        if (!(dt > 0 && dt <= 3600))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "The time step must be above 0 and at most 3600 seconds.");
        }

        Crowd = crowd;
        Dt = dt;
        _meshEdits = crowd.Mesh.EditCount;
        if (keepSteps)
        {
            var w = new ArrayBufferWriter<byte>();
            CrowdSetup.Write(crowd, dt, w);
            _setup = w.WrittenSpan.ToArray();
            _steps = [];
            _hashes = [];
        }
    }

    /// <summary>The crowd being recorded.</summary>
    public Crowd Crowd { get; }

    /// <summary>The seconds each <see cref="Step"/> advances the crowd.</summary>
    public double Dt { get; }

    /// <summary>Called after each change with its record, e.g. a probe's record call. Records are dropped while it is null. A handler must not call back into the recorder.</summary>
    public CrowdRecordHandler? Record { get; set; }

    /// <summary>Adds an agent like <see cref="Crowd.Add"/> and records it.</summary>
    /// <exception cref="ArgumentException">The settings are invalid, or no walkable polygon is near <paramref name="position"/>.</exception>
    /// <exception cref="InvalidOperationException">The crowd is full.</exception>
    public Agent Add(Vector3D position, AgentOptions? options = null)
    {
        options ??= Agent.Humanoid;
        Agent agent = Crowd.Add(position, options);
        Emit(CrowdOp.Add(position, options));
        return agent;
    }

    /// <summary>Adds an agent like <see cref="Crowd.TryAdd"/>; records it only when it was added.</summary>
    public bool TryAdd(Vector3D position, AgentOptions? options, out Agent? agent)
    {
        options ??= Agent.Humanoid;
        if (!Crowd.TryAdd(position, options, out agent))
        {
            return false;
        }

        Emit(CrowdOp.Add(position, options));
        return true;
    }

    /// <summary>Removes an agent like <see cref="Crowd.Remove"/>; records it only when the agent was in the crowd.</summary>
    public bool Remove(Agent agent)
    {
        ArgumentNullException.ThrowIfNull(agent);
        long id = agent.Id;
        if (!Crowd.Remove(agent))
        {
            return false;
        }

        Emit(CrowdOp.Remove(id));
        return true;
    }

    /// <summary>Sets an agent's destination like <see cref="Agent.SetDestination"/> and records it.</summary>
    /// <exception cref="InvalidOperationException">The agent is not in the crowd.</exception>
    public PathStatus SetDestination(Agent agent, Vector3D destination)
    {
        ArgumentNullException.ThrowIfNull(agent);
        NavMeshQuery.CheckPos(destination, nameof(destination));
        PathStatus status = agent.SetDestination(destination);
        Emit(CrowdOp.SetDestination(agent.Id, destination));
        return status;
    }

    /// <summary>Stops an agent like <see cref="Agent.Stop"/> and records it.</summary>
    public void Stop(Agent agent)
    {
        ArgumentNullException.ThrowIfNull(agent);
        agent.Stop();
        Emit(CrowdOp.Stop(agent.Id));
    }

    /// <summary>Teleports an agent like <see cref="Agent.Teleport"/>; records it whether or not a polygon was found, since the crowd runs the same call either way.</summary>
    /// <exception cref="InvalidOperationException">The agent is not in the crowd.</exception>
    public bool Teleport(Agent agent, Vector3D position)
    {
        ArgumentNullException.ThrowIfNull(agent);
        NavMeshQuery.CheckPos(position, nameof(position));
        bool ok = agent.Teleport(position);
        Emit(CrowdOp.Teleport(agent.Id, position));
        return ok;
    }

    /// <summary>Advances the crowd by <see cref="Dt"/> and records the step with the crowd's hash after it.</summary>
    /// <exception cref="InvalidOperationException">The mesh was edited since recording began; edits are not recorded.</exception>
    public void Step()
    {
        if (Crowd.Mesh.EditCount != _meshEdits)
        {
            throw new InvalidOperationException("The mesh was edited while recording; edits are not recorded, so the recording would not replay.");
        }

        Crowd.Step(Dt);
        if (_steps is null && Record is null)
        {
            return;
        }

        ulong hash = Crowd.StateHash();
        if (_steps is not null)
        {
            _steps.Add(_ops.WrittenSpan.ToArray());
            _hashes!.Add(hash);
            _ops.ResetWrittenCount();
        }

        if (Record is { } handler)
        {
            Span<byte> r = stackalloc byte[9];
            r[0] = 2;
            BinaryPrimitives.WriteUInt64LittleEndian(r[1..], hash);
            handler(r);
        }
    }

    /// <summary>
    /// Writes the crowd as it is now as a setup block: what a live viewer needs to start from this moment. Call it from a probe's snapshot
    /// source; the crowd is then exactly as it is after the last record passed to <see cref="Record"/>.
    /// </summary>
    public void WriteSnapshot(IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        CrowdSetup.Write(Crowd, Dt, into);
    }

    /// <summary>The run so far as a replay: the setup from when the recorder was made and every step since.</summary>
    /// <exception cref="InvalidOperationException">The recorder was made without <c>keepSteps</c>.</exception>
    public CrowdReplay ToReplay()
    {
        if (_steps is null)
        {
            throw new InvalidOperationException("This recorder was made without keepSteps, so it kept nothing to replay.");
        }

        return CrowdReplay.Create(_setup, _steps, _hashes!);
    }

    private void Emit(in CrowdOp op)
    {
        if (_steps is not null)
        {
            CrowdOps.Write(op, _ops);
        }

        if (Record is { } handler)
        {
            _record.ResetWrittenCount();
            Span<byte> kind = _record.GetSpan(1);
            kind[0] = 1;
            _record.Advance(1);
            CrowdOps.Write(op, _record);
            handler(_record.WrittenSpan);
        }
    }
}
