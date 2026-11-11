using System;
using System.Buffers;
using System.Buffers.Binary;
using OneScope.Sdk;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.Scope;

/// <summary>
/// The OnePath module for the OneScope viewer. It opens crowd replays (<c>OPRP</c>, written by <see cref="CrowdRecorder"/>) and navigation-mesh
/// build inputs (<c>OPBI</c>, written by <see cref="BuildInput"/>), and attaches to a live crowd that a host records with a <see cref="CrowdRecorder"/>
/// behind a probe. The shell constructs it; there is nothing to configure.
/// </summary>
public sealed class OnePathModule : IScopeModule, ILiveModule
{
    /// <inheritdoc />
    public string Id => ScopeInfo.ModuleId;

    /// <inheritdoc />
    public string Name => "OnePath";

    /// <inheritdoc />
    /// <exception cref="ScopeFormatException">The bytes are neither a crowd replay nor a build input, or are damaged; the message says what was expected and where.</exception>
    public IScopeSource Open(ReadOnlyMemory<byte> data, string name)
    {
        ArgumentNullException.ThrowIfNull(name);
        ReadOnlySpan<byte> s = data.Span;
        if (s.StartsWith("OPRP"u8))
        {
            return OpenCrowd(data, name);
        }

        if (s.StartsWith(BuildInput.Magic))
        {
            return BuildSource.Open(data, name);
        }

        throw new ScopeFormatException(name, "expected a OnePath crowd replay (starting 'OPRP') or navigation-mesh build input (starting 'OPBI').");
    }

    /// <inheritdoc />
    /// <exception cref="ScopeFormatException">The format version is not 1, or the snapshot is not a crowd setup block from <see cref="CrowdRecorder.WriteSnapshot"/>.</exception>
    public ILiveSource OpenLive(string libraryVersion, int formatVersion, ReadOnlyMemory<byte> snapshot, string name)
    {
        ArgumentNullException.ThrowIfNull(name);
        if (formatVersion != ScopeInfo.FormatVersion)
        {
            throw new ScopeFormatException(name, $"expected crowd replay format version {ScopeInfo.FormatVersion}, the host announced {formatVersion}.");
        }

        CrowdSource source = Make(snapshot.ToArray(), [], null, name, startFromSetup: true);
        return new LiveCrowd(source);
    }

    private static CrowdSource OpenCrowd(ReadOnlyMemory<byte> data, string name)
    {
        CrowdReplay replay;
        try
        {
            replay = CrowdReplay.Read(data.Span);
        }
        catch (CrowdReplayException e)
        {
            throw new ScopeFormatException(name, e.Message);
        }

        ulong[] expected = new ulong[replay.Hashes.Count + 1];
        for (int i = 0; i < replay.Hashes.Count; i++)
        {
            expected[i + 1] = replay.Hashes[i];
        }

        return Make(replay.Setup.ToArray(), replay.Steps, expected, name, startFromSetup: false);
    }

    private static CrowdSource Make(byte[] setup, System.Collections.Generic.IReadOnlyList<ReadOnlyMemory<byte>> steps, ulong[]? expected, string name, bool startFromSetup)
    {
        CrowdPlayer player;
        try
        {
            player = new CrowdPlayer(setup);
        }
        catch (CrowdReplayException e)
        {
            throw new ScopeFormatException(name, e.Message);
        }

        long start = player.Crowd.StepCount;
        if (expected is not null)
        {
            expected[0] = player.Crowd.StateHash();
        }

        Recording recording = Recording.Create(setup, steps, start, expected);
        if (startFromSetup)
        {
            recording = recording.Fork(start);
        }

        return new CrowdSource(name, recording, new MeshView(player.Crowd.Mesh), player.Options, player.Dt);
    }

    private sealed class LiveCrowd(CrowdSource source) : ILiveSource
    {
        private readonly System.Collections.Generic.List<CrowdOp> _check = [];
        private readonly ArrayBufferWriter<byte> _pending = new();

        public IScopeSource Source => source;

        public void AppendRecord(ReadOnlySpan<byte> record)
        {
            if (record.IsEmpty)
            {
                throw new ScopeFormatException(source.Name, "expected a record of at least one byte.");
            }

            switch (record[0])
            {
                case 1:
                    try
                    {
                        CrowdOps.Parse(record[1..], _check);
                    }
                    catch (CrowdReplayException e)
                    {
                        throw new ScopeFormatException(source.Name, e.Message);
                    }

                    if (_pending.WrittenCount + record.Length - 1 > CrowdOps.MaxStepBytes)
                    {
                        throw new ScopeFormatException(source.Name, $"expected at most {CrowdOps.MaxStepBytes} bytes of operations before a step.");
                    }

                    _pending.Write(record[1..]);
                    break;
                case 2:
                    if (record.Length != 9)
                    {
                        throw new ScopeFormatException(source.Name, $"expected a step record of 9 bytes, got {record.Length}.");
                    }

                    // The hash in the record is the host's; the viewer's own hash is checked by comparing with the host's list.
                    source.Recording.Append(_pending.WrittenSpan);
                    _pending.ResetWrittenCount();
                    break;
                default:
                    throw new ScopeFormatException(source.Name, $"expected a record of kind 1 (operations) or 2 (step), got {record[0]}.");
            }
        }
    }
}
