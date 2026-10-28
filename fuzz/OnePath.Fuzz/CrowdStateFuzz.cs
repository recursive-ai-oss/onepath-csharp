using System;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for the crowd state reader. The first byte picks how the input becomes state bytes: as they are, as edits to a valid
/// state (position and xor pairs), cut short, or a valid state with the input's bytes spliced in. A restore must either succeed or
/// throw <see cref="OnePathFormatException"/> and leave the crowd as it was; after a success the crowd must step twice, and saving
/// then restoring then saving must give the same bytes. Any other exception crashes the target.
/// </summary>
internal static class CrowdStateFuzz
{
    // Built on first use, not in a type initializer: the coverage instrumentation is not ready inside one.
    private static Crowd? s_crowd;
    private static byte[]? s_valid;

    public static void Run(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }
        if (s_crowd is null || s_valid is null)
        {
            CrowdSim sim = CrowdRollback.Populated(1);
            CrowdRollback.Run(sim, 0, 25, false);
            s_crowd = sim.Crowd;
            s_valid = sim.Crowd.SaveState();
        }
        Crowd crowd = s_crowd;
        crowd.RestoreState(s_valid);
        ReadOnlySpan<byte> rest = data.AsSpan(1);
        byte[] bytes;
        switch (data[0] & 3)
        {
            case 0:
                bytes = rest.ToArray();
                break;
            case 1:
                bytes = (byte[])s_valid.Clone();
                for (int i = 0; i + 2 < rest.Length; i += 3)
                {
                    int at = (((rest[i] << 8) | rest[i + 1]) * 7) % bytes.Length;
                    bytes[at] ^= rest[i + 2];
                }
                break;
            case 2:
                bytes = s_valid.AsSpan(0, ((rest[0] << 8) | (rest.Length > 1 ? rest[1] : 0)) % (s_valid.Length + 1)).ToArray();
                break;
            default:
            {
                int at = ((rest[0] << 8) | (rest.Length > 1 ? rest[1] : 0)) % s_valid.Length;
                bytes = new byte[at + Math.Max(0, rest.Length - 2)];
                s_valid.AsSpan(0, at).CopyTo(bytes);
                rest[Math.Min(2, rest.Length)..].CopyTo(bytes.AsSpan(at));
                break;
            }
        }
        // Restore from a span inside a larger buffer, so a reader that trusts the span's start or end shows up.
        int lead = (data[0] >> 2) & 7;
        byte[] padded = new byte[lead + bytes.Length + 5];
        bytes.CopyTo(padded, lead);
        ulong validHash = crowd.StateHash();
        try
        {
            crowd.RestoreState(padded.AsSpan(lead, bytes.Length));
        }
        catch (OnePathFormatException)
        {
            if (crowd.StateHash() != validHash || !s_valid.AsSpan().SequenceEqual(crowd.SaveState()))
            {
                throw new InvalidOperationException("a rejected restore changed the crowd");
            }
            return;
        }
        byte[] again = crowd.SaveState();
        var span = new byte[again.Length];
        if (!crowd.TrySaveState(span, out int written) || written != again.Length || !span.AsSpan().SequenceEqual(again) || crowd.GetStateSize() != again.Length)
        {
            throw new InvalidOperationException("the span save differs from the array save");
        }
        ulong h = crowd.StateHash();
        crowd.RestoreState(again);
        if (crowd.StateHash() != h)
        {
            throw new InvalidOperationException("save, restore did not keep the state hash");
        }
        if (!again.AsSpan().SequenceEqual(crowd.SaveState()))
        {
            throw new InvalidOperationException("save, restore, save did not give the same bytes");
        }
        crowd.Step(CrowdRollback.Dt);
        crowd.Step(CrowdRollback.Dt);
    }
}
