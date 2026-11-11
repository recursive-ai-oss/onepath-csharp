using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using CrowdStateReader = RecursiveAI.OnePath.Crowds.Corridors.StateReader;

namespace RecursiveAI.OnePath.Replay;

/// <summary>
/// A recorded crowd run: a setup block, then for each step the operations applied before it and the crowd's <see cref="Crowd.StateHash"/> after it.
/// Replaying gives the recorded hashes on every platform. The byte layout is in docs/replay.md.
/// </summary>
public sealed class CrowdReplay
{
    /// <summary>The most steps a replay read from bytes may hold.</summary>
    public const int MaxSteps = 1 << 26;

    private static ReadOnlySpan<byte> Magic => "OPRP"u8;

    private CrowdReplay(ReadOnlyMemory<byte> setup, ReadOnlyMemory<byte>[] steps, ulong[] hashes)
    {
        Setup = setup;
        Steps = steps;
        Hashes = hashes;
    }

    /// <summary>The setup block (the first thing <see cref="CrowdPlayer"/> reads).</summary>
    public ReadOnlyMemory<byte> Setup { get; }

    /// <summary>For each step, the operation bytes applied before it (see <see cref="CrowdOps.Parse"/>); empty for a step with none.</summary>
    public IReadOnlyList<ReadOnlyMemory<byte>> Steps { get; }

    /// <summary>For each step, the <see cref="Crowd.StateHash"/> after it.</summary>
    public IReadOnlyList<ulong> Hashes { get; }

    /// <summary>Makes a replay from its parts (copied). Nothing is checked beyond the lengths; <see cref="FindMismatch"/> plays it.</summary>
    /// <exception cref="ArgumentException">The two lists differ in length, or a step is longer than <see cref="CrowdOps.MaxStepBytes"/>.</exception>
    public static CrowdReplay Create(ReadOnlySpan<byte> setup, IReadOnlyList<ReadOnlyMemory<byte>> steps, IReadOnlyList<ulong> hashes)
    {
        ArgumentNullException.ThrowIfNull(steps);
        ArgumentNullException.ThrowIfNull(hashes);
        if (steps.Count != hashes.Count)
        {
            throw new ArgumentException($"{steps.Count} steps but {hashes.Count} hashes.", nameof(hashes));
        }

        var s = new ReadOnlyMemory<byte>[steps.Count];
        for (int i = 0; i < s.Length; i++)
        {
            if (steps[i].Length > CrowdOps.MaxStepBytes)
            {
                throw new ArgumentException($"Step {i} holds {steps[i].Length} bytes of operations, at most {CrowdOps.MaxStepBytes} are allowed.", nameof(steps));
            }

            s[i] = steps[i].ToArray();
        }

        var h = new ulong[hashes.Count];
        for (int i = 0; i < h.Length; i++)
        {
            h[i] = hashes[i];
        }

        return new CrowdReplay(setup.ToArray(), s, h);
    }

    /// <summary>Reads a replay file's bytes. The structure is checked here (magic, version, lengths that fit, no trailing bytes); the setup is checked when a <see cref="CrowdPlayer"/> is built from it.</summary>
    /// <exception cref="CrowdReplayException">The bytes are not a replay; the message names the problem and the byte position.</exception>
    public static CrowdReplay Read(ReadOnlySpan<byte> bytes)
    {
        var r = new CrowdStateReader(bytes, "replay");
        try
        {
            if (!r.PeekEquals(Magic))
            {
                throw new CrowdReplayException("Invalid replay: expected the bytes 'OPRP' at byte 0.");
            }

            r.Bytes(4);
            int version = r.U8();
            int kind = r.U8();
            int reserved = r.U16();
            if (version != 1 || kind != CrowdSetup.KindCrowd || reserved != 0)
            {
                throw new CrowdReplayException($"Invalid replay: version {version}, kind {kind}, reserved {reserved}; expected version 1, kind 1 (crowd), reserved 0.");
            }

            int setupLength = r.I32();
            if (setupLength < 0 || setupLength > r.Remaining)
            {
                throw r.Error("setup length", setupLength);
            }

            ReadOnlySpan<byte> setup = r.Bytes(setupLength);
            if (!setup.StartsWith(CrowdSetup.Magic))
            {
                throw new CrowdReplayException($"Invalid replay: the setup block should start with 'OPRS' at byte {r.Position - setupLength}.");
            }

            int count = r.I32();
            // Every step takes at least 12 bytes, so a count the rest cannot hold is wrong before anything is allocated.
            if (count < 0 || count > MaxSteps || (long)count * 12 > r.Remaining)
            {
                throw r.Error("step count", count);
            }

            var steps = new ReadOnlyMemory<byte>[count];
            var hashes = new ulong[count];
            for (int i = 0; i < count; i++)
            {
                int length = r.I32();
                if (length < 0 || length > CrowdOps.MaxStepBytes || length > r.Remaining - 8)
                {
                    throw r.Error($"length of step {i}", length);
                }

                steps[i] = r.Bytes(length).ToArray();
                hashes[i] = r.U64();
            }

            if (r.Remaining != 0)
            {
                throw new CrowdReplayException($"Invalid replay: {r.Remaining} unexpected bytes after the last step, at byte {r.Position}.");
            }

            return new CrowdReplay(setup.ToArray(), steps, hashes);
        }
        catch (CrowdStateException e)
        {
            throw new CrowdReplayException(e.Message, e);
        }
    }

    /// <summary>Appends the replay's bytes. Reading them back and writing again gives the same bytes.</summary>
    public void Write(IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        Span<byte> head = into.GetSpan(12);
        Magic.CopyTo(head);
        head[4] = 1;
        head[5] = CrowdSetup.KindCrowd;
        head[6] = 0;
        head[7] = 0;
        BinaryPrimitives.WriteInt32LittleEndian(head[8..], Setup.Length);
        into.Advance(12);
        into.Write(Setup.Span);
        Span<byte> count = into.GetSpan(4);
        BinaryPrimitives.WriteInt32LittleEndian(count, Steps.Count);
        into.Advance(4);
        for (int i = 0; i < Steps.Count; i++)
        {
            Span<byte> len = into.GetSpan(4);
            BinaryPrimitives.WriteInt32LittleEndian(len, Steps[i].Length);
            into.Advance(4);
            into.Write(Steps[i].Span);
            Span<byte> h = into.GetSpan(8);
            BinaryPrimitives.WriteUInt64LittleEndian(h, Hashes[i]);
            into.Advance(8);
        }
    }

    /// <summary>The replay's bytes in a new array.</summary>
    public byte[] ToBytes()
    {
        var w = new ArrayBufferWriter<byte>();
        Write(w);
        return w.WrittenSpan.ToArray();
    }

    /// <summary>
    /// Plays the whole replay and compares the crowd's hash after each step with the recorded one. Returns the index of the first step
    /// that differs, or null when every step matches.
    /// </summary>
    /// <exception cref="CrowdReplayException">The setup or an operation is invalid.</exception>
    public int? FindMismatch()
    {
        var player = new CrowdPlayer(Setup.Span);
        for (int i = 0; i < Steps.Count; i++)
        {
            player.Apply(Steps[i].Span);
            player.Step();
            if (player.Crowd.StateHash() != Hashes[i])
            {
                return i;
            }
        }

        return null;
    }
}
