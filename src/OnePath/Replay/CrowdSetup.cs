using System;
using System.Buffers;
using System.Buffers.Binary;
using CrowdStateReader = RecursiveAI.OnePath.Crowds.Corridors.StateReader;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Replay;

// The setup block of a crowd replay: everything needed to rebuild the crowd at its first step (docs/replay.md, "Setup").
internal static class CrowdSetup
{
    internal const int Version = 1;
    internal const int KindCrowd = 1;
    internal static ReadOnlySpan<byte> Magic => "OPRS"u8;

    internal readonly record struct Parsed(double Dt, long StartStep, CrowdOptions Options, ReadOnlyMemory<byte> Mesh, ReadOnlyMemory<byte> State);

    internal static void Write(Crowd crowd, double dt, IBufferWriter<byte> into)
    {
        byte[] mesh = NavMeshSnapshot.Take(crowd.Mesh);
        byte[] state = crowd.SaveState();
        CrowdOptions o = crowd.Options;
        int size = 4 + 4 + 8 + 8 + 4 + 4 + 1 + 4 + 8 + 4 + 4 + mesh.Length + 4 + state.Length;
        Span<byte> d = into.GetSpan(size);
        Magic.CopyTo(d);
        d[4] = Version;
        d[5] = KindCrowd;
        d[6] = 0;
        d[7] = 0;
        int n = 8;
        BinaryPrimitives.WriteInt64LittleEndian(d[n..], BitConverter.DoubleToInt64Bits(dt));
        BinaryPrimitives.WriteInt64LittleEndian(d[(n + 8)..], crowd.StepCount);
        n += 16;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], o.MaxAgents);
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 4)..], o.QueryNodes);
        d[n + 8] = (byte)o.QueryNodeModel;
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 9)..], o.MaxWallsPerAgent);
        BinaryPrimitives.WriteInt64LittleEndian(d[(n + 13)..], BitConverter.DoubleToInt64Bits(o.WallRefreshDistance));
        BinaryPrimitives.WriteInt32LittleEndian(d[(n + 21)..], o.MaxWallsAvoided);
        n += 25;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], mesh.Length);
        mesh.CopyTo(d[(n + 4)..]);
        n += 4 + mesh.Length;
        BinaryPrimitives.WriteInt32LittleEndian(d[n..], state.Length);
        state.CopyTo(d[(n + 4)..]);
        n += 4 + state.Length;
        into.Advance(n);
    }

    internal static Parsed Parse(ReadOnlySpan<byte> setup)
    {
        var r = new CrowdStateReader(setup, "replay setup");
        try
        {
            if (!r.PeekEquals(Magic))
            {
                throw new CrowdReplayException("Invalid replay setup: expected the bytes 'OPRS' at byte 0.");
            }

            r.Bytes(4);
            int version = r.U8();
            int kind = r.U8();
            int reserved = r.U16();
            if (version != Version || kind != KindCrowd || reserved != 0)
            {
                throw new CrowdReplayException($"Invalid replay setup: version {version}, kind {kind}, reserved {reserved}; expected version {Version}, kind {KindCrowd} (crowd), reserved 0.");
            }

            double dt = r.F64("time step");
            if (!(dt > 0 && dt <= 3600))
            {
                throw r.Error("time step", dt);
            }

            long start = (long)r.U64();
            if (start < 0)
            {
                throw r.Error("start step", start);
            }

            int maxAgents = r.I32(), queryNodes = r.I32(), model = r.U8(), maxWalls = r.I32();
            double refresh = r.F64("wall refresh distance");
            int avoided = r.I32();
            CrowdOptions options;
            try
            {
                options = new CrowdOptions
                {
                    MaxAgents = maxAgents,
                    QueryNodes = queryNodes,
                    QueryNodeModel = (PathNodeModel)model,
                    MaxWallsPerAgent = maxWalls,
                    WallRefreshDistance = refresh,
                    MaxWallsAvoided = avoided,
                };
                options.Validate();
            }
            catch (ArgumentOutOfRangeException e)
            {
                throw new CrowdReplayException($"Invalid replay setup: crowd options: {e.Message}", e);
            }

            int meshLength = r.I32();
            if (meshLength < 0 || meshLength > r.Remaining)
            {
                throw r.Error("mesh length", meshLength);
            }

            int meshAt = r.Position;
            r.Bytes(meshLength);
            int stateLength = r.I32();
            if (stateLength < 0 || stateLength != r.Remaining)
            {
                throw r.Error("crowd state length", stateLength);
            }

            int stateAt = r.Position;
            return new Parsed(dt, start, options, setup.Slice(meshAt, meshLength).ToArray(), setup.Slice(stateAt, stateLength).ToArray());
        }
        catch (CrowdStateException e)
        {
            throw new CrowdReplayException(e.Message, e);
        }
    }
}
