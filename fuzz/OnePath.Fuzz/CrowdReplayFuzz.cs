using System;
using System.Buffers;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for the crowd replay reader. The first byte picks how the input becomes bytes: a whole replay as given, edits to a valid
/// replay, a valid replay cut short, or one step's operation bytes. Reading must succeed or throw <see cref="OnePathFormatException"/>
/// (and nothing else); a replay that reads must write back to the same bytes, and a short one must play without any other exception.
/// Operation bytes that parse must write back to the same bytes.
/// </summary>
internal static class CrowdReplayFuzz
{
    // Built on first use, not in a type initializer: the coverage instrumentation is not ready inside one.
    private static byte[]? s_valid;

    public static void Run(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }

        s_valid ??= MakeValid();
        ReadOnlySpan<byte> rest = data.AsSpan(1);
        switch (data[0] & 3)
        {
            case 0:
                Replay(rest.ToArray());
                break;
            case 1:
                {
                    byte[] bytes = (byte[])s_valid.Clone();
                    for (int i = 0; i + 2 < rest.Length; i += 3)
                    {
                        int at = (((rest[i] << 8) | rest[i + 1]) * 7) % bytes.Length;
                        bytes[at] ^= rest[i + 2];
                    }

                    Replay(bytes);
                    break;
                }

            case 2:
                Replay(s_valid.AsSpan(0, ((rest[0] << 8) | (rest.Length > 1 ? rest[1] : 0)) % (s_valid.Length + 1)).ToArray());
                break;
            default:
                Ops(rest.ToArray());
                break;
        }
    }

    private static void Ops(byte[] ops)
    {
        var parsed = new List<CrowdOp>();
        try
        {
            CrowdOps.Parse(ops, parsed);
        }
        catch (OnePathFormatException)
        {
            return;
        }

        var w = new ArrayBufferWriter<byte>();
        foreach (CrowdOp op in parsed)
        {
            CrowdOps.Write(op, w);
        }

        if (!w.WrittenSpan.SequenceEqual(ops))
        {
            throw new InvalidOperationException("operations that parsed did not write back to the same bytes");
        }
    }

    private static void Replay(byte[] bytes)
    {
        CrowdReplay replay;
        try
        {
            replay = CrowdReplay.Read(bytes);
        }
        catch (OnePathFormatException)
        {
            return;
        }

        if (!replay.ToBytes().AsSpan().SequenceEqual(bytes))
        {
            throw new InvalidOperationException("a replay that read did not write back to the same bytes");
        }

        if (replay.Steps.Count <= 32)
        {
            try
            {
                replay.FindMismatch();
            }
            catch (OnePathFormatException)
            {
            }
        }
    }

    private static byte[] MakeValid()
    {
        var crowd = new Crowd(CrowdWorlds.Arena(14, 10), new CrowdOptions { MaxAgents = 64 });
        var rec = new CrowdRecorder(crowd, CrowdWorlds.Dt, keepSteps: true);
        var agents = new List<Agent>();
        for (int i = 0; i < 4; i++)
        {
            agents.Add(rec.Add(new Vector3D(2 + i, 0, 3), Agent.Small));
            rec.SetDestination(agents[i], new Vector3D(11, 0, 3 + i));
        }

        for (int s = 0; s < 10; s++)
        {
            if (s == 4)
            {
                rec.Remove(agents[1]);
                rec.Teleport(agents[2], new Vector3D(6, 0, 6));
                rec.Stop(agents[3]);
            }

            rec.Step();
        }

        return rec.ToReplay().ToBytes();
    }
}
