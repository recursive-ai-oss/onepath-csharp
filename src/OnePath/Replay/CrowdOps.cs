using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using CrowdStateReader = RecursiveAI.OnePath.Crowds.Corridors.StateReader;

namespace RecursiveAI.OnePath.Replay;

/// <summary>What a recorded crowd operation does.</summary>
public enum CrowdOpKind : byte
{
    /// <summary>Adds an agent at a position with settings. The crowd numbers it with the next agent id.</summary>
    Add = 1,

    /// <summary>Removes the agent with the id.</summary>
    Remove = 2,

    /// <summary>Sets the agent's destination.</summary>
    SetDestination = 3,

    /// <summary>Stops the agent.</summary>
    Stop = 4,

    /// <summary>Moves the agent to a position on the mesh and drops its path.</summary>
    Teleport = 5,
}

/// <summary>One operation on a crowd, applied before the step it is recorded with. See docs/replay.md.</summary>
/// <param name="Kind">What the operation does.</param>
/// <param name="AgentId">The <see cref="RecursiveAI.OnePath.Agent.Id"/> the operation targets; 0 for <see cref="CrowdOpKind.Add"/>.</param>
/// <param name="Position">The position (world units) for add, destination and teleport; zero otherwise. Each axis must be finite and within ±2^21.</param>
/// <param name="Options">The settings of an added agent; null for other kinds.</param>
public readonly record struct CrowdOp(CrowdOpKind Kind, long AgentId, Vector3D Position, AgentOptions? Options)
{
    /// <summary>An add operation. A null <paramref name="options"/> means <see cref="RecursiveAI.OnePath.Agent.Humanoid"/>.</summary>
    public static CrowdOp Add(Vector3D position, AgentOptions? options = null) => new(CrowdOpKind.Add, 0, position, options ?? RecursiveAI.OnePath.Agent.Humanoid);

    /// <summary>A remove operation.</summary>
    public static CrowdOp Remove(long agent) => new(CrowdOpKind.Remove, agent, default, null);

    /// <summary>A set-destination operation.</summary>
    public static CrowdOp SetDestination(long agent, Vector3D destination) => new(CrowdOpKind.SetDestination, agent, destination, null);

    /// <summary>A stop operation.</summary>
    public static CrowdOp Stop(long agent) => new(CrowdOpKind.Stop, agent, default, null);

    /// <summary>A teleport operation.</summary>
    public static CrowdOp Teleport(long agent, Vector3D position) => new(CrowdOpKind.Teleport, agent, position, null);
}

/// <summary>Reads and writes the operation bytes a replay step holds: operations one after another, each starting with its kind byte.</summary>
public static class CrowdOps
{
    /// <summary>The most bytes one step's operations may hold when read from a replay (16 MiB).</summary>
    public const int MaxStepBytes = 16 << 20;

    private const double Bound = 2097152.0;

    /// <summary>Appends the bytes of <paramref name="op"/> to <paramref name="into"/>.</summary>
    /// <exception cref="ArgumentException">The kind is unknown, a position is not finite or is beyond ±2^21, or an add has no settings.</exception>
    public static void Write(in CrowdOp op, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        bool hasPosition = op.Kind is CrowdOpKind.Add or CrowdOpKind.SetDestination or CrowdOpKind.Teleport;
        bool hasAgent = op.Kind is CrowdOpKind.Remove or CrowdOpKind.SetDestination or CrowdOpKind.Stop or CrowdOpKind.Teleport;
        if (!hasPosition && !hasAgent && op.Kind != CrowdOpKind.Add)
        {
            throw new ArgumentException($"Unknown operation kind {(byte)op.Kind}.", nameof(op));
        }

        if (hasPosition && !(Math.Abs(op.Position.X) < Bound && Math.Abs(op.Position.Y) < Bound && Math.Abs(op.Position.Z) < Bound))
        {
            throw new ArgumentException($"Position {op.Position} must be finite and within ±{Bound:R} on every axis.", nameof(op));
        }

        byte[] options = [];
        if (op.Kind == CrowdOpKind.Add)
        {
            if (op.Options is null)
            {
                throw new ArgumentException("An add operation needs agent settings.", nameof(op));
            }

            op.Options.Validate();
            options = Crowd.AgentOptionsBytes(op.Options);
        }

        int size = 1 + (hasAgent ? 8 : 0) + (hasPosition ? 24 : 0) + options.Length;
        Span<byte> d = into.GetSpan(size);
        int n = 0;
        d[n++] = (byte)op.Kind;
        if (hasAgent)
        {
            BinaryPrimitives.WriteInt64LittleEndian(d[n..], op.AgentId);
            n += 8;
        }

        if (hasPosition)
        {
            BinaryPrimitives.WriteInt64LittleEndian(d[n..], BitConverter.DoubleToInt64Bits(op.Position.X));
            BinaryPrimitives.WriteInt64LittleEndian(d[(n + 8)..], BitConverter.DoubleToInt64Bits(op.Position.Y));
            BinaryPrimitives.WriteInt64LittleEndian(d[(n + 16)..], BitConverter.DoubleToInt64Bits(op.Position.Z));
            n += 24;
        }

        options.CopyTo(d[n..]);
        into.Advance(size);
    }

    /// <summary>Clears <paramref name="into"/> and fills it with the operations in <paramref name="ops"/>. Empty bytes give none.</summary>
    /// <exception cref="CrowdReplayException">The bytes are longer than <see cref="MaxStepBytes"/>, an operation is cut short, a kind is unknown, a value is out of range or the settings are invalid; the message gives the byte position.</exception>
    public static void Parse(ReadOnlySpan<byte> ops, List<CrowdOp> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        if (ops.Length > MaxStepBytes)
        {
            throw new CrowdReplayException($"Invalid replay step: {ops.Length} bytes of operations, at most {MaxStepBytes} are allowed.");
        }

        var r = new CrowdStateReader(ops, "replay");
        try
        {
            while (r.Remaining > 0)
            {
                int at = r.Position;
                int kind = r.U8();
                switch ((CrowdOpKind)kind)
                {
                    case CrowdOpKind.Add:
                        {
                            Vector3D p = r.V3("position");
                            AgentOptions o = Crowd.ReadAgentOptionsBytes(ops[r.Position..], out int used);
                            r.Bytes(used);
                            into.Add(CrowdOp.Add(p, o));
                            break;
                        }

                    case CrowdOpKind.Remove:
                        into.Add(CrowdOp.Remove((long)r.U64()));
                        break;
                    case CrowdOpKind.Stop:
                        into.Add(CrowdOp.Stop((long)r.U64()));
                        break;
                    case CrowdOpKind.SetDestination:
                        {
                            long id = (long)r.U64();
                            into.Add(CrowdOp.SetDestination(id, r.V3("destination")));
                            break;
                        }

                    case CrowdOpKind.Teleport:
                        {
                            long id = (long)r.U64();
                            into.Add(CrowdOp.Teleport(id, r.V3("position")));
                            break;
                        }

                    default:
                        throw new CrowdReplayException($"Invalid replay step: unknown operation kind {kind} at byte {at}.");
                }
            }
        }
        catch (CrowdStateException e)
        {
            throw new CrowdReplayException(e.Message, e);
        }
    }
}
