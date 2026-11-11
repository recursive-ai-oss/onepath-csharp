using System;
using CrowdStateReader = RecursiveAI.OnePath.Crowds.Corridors.StateReader;

namespace RecursiveAI.OnePath;

// Agent settings as bytes for the replay format (docs/replay.md). The layout is the one the crowd state uses for an agent's settings.
public sealed partial class Crowd
{
    internal static byte[] AgentOptionsBytes(AgentOptions options)
    {
        var measure = new StateSink(Span<byte>.Empty, false);
        WriteAgentOptions(ref measure, options);
        byte[] bytes = new byte[measure.Length];
        var w = new StateSink(bytes, false);
        WriteAgentOptions(ref w, options);
        return bytes;
    }

    internal static AgentOptions ReadAgentOptionsBytes(ReadOnlySpan<byte> source, out int consumed)
    {
        var r = new CrowdStateReader(source, "replay");
        AgentOptions options = ReadAgentOptions(ref r);
        consumed = r.Position;
        return options;
    }
}
