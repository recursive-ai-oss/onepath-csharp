using System;
using System.Buffers;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Replay;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Replay;

public class CrowdReplayTests
{
    private static (CrowdReplay Replay, List<ulong> Hashes, List<byte[]> Records) Record(int steps = 60, bool live = false)
    {
        var crowd = new Crowd(CrowdWorlds.Arena(24, 16));
        var rec = new CrowdRecorder(crowd, CrowdWorlds.Dt, keepSteps: true);
        var records = new List<byte[]>();
        if (live)
        {
            rec.Record = r => records.Add(r.ToArray());
        }

        var hashes = new List<ulong>();
        var agents = new List<Agent>();
        for (int i = 0; i < 6; i++)
        {
            agents.Add(rec.Add(new Vector3D(2 + i, 0, 3), Agent.Small));
            rec.SetDestination(agents[i], new Vector3D(20, 0, 3 + i));
        }

        for (int s = 0; s < steps; s++)
        {
            if (s == 10)
            {
                rec.Remove(agents[1]);
                rec.Stop(agents[2]);
            }

            if (s == 20)
            {
                rec.Teleport(agents[3], new Vector3D(10, 0, 10));
                rec.Add(new Vector3D(3, 0, 12), Agent.Small);
            }

            rec.Step();
            hashes.Add(crowd.StateHash());
        }

        return (rec.ToReplay(), hashes, records);
    }

    [Fact]
    public void ReplayPlaysToTheRecordedHashesAndRoundTrips()
    {
        (CrowdReplay replay, List<ulong> hashes, _) = Record();
        Assert.Equal(hashes, replay.Hashes);
        byte[] bytes = replay.ToBytes();
        CrowdReplay read = CrowdReplay.Read(bytes);
        Assert.Null(read.FindMismatch());
        Assert.Equal(bytes, read.ToBytes());
        Assert.Equal(60, read.Steps.Count);
    }

    [Fact]
    public void LiveRecordsAreTheStepsOperationsThenAHash()
    {
        (CrowdReplay replay, _, List<byte[]> records) = Record(live: true);
        // 6 adds + 6 destinations + remove + stop + teleport + add = 16 operation records, and 60 step records.
        Assert.Equal(16 + 60, records.Count);
        var ops = new List<byte>();
        int step = 0;
        foreach (byte[] r in records)
        {
            if (r[0] == 1)
            {
                ops.AddRange(r.AsSpan(1).ToArray());
            }
            else
            {
                Assert.Equal(9, r.Length);
                Assert.Equal(replay.Steps[step].ToArray(), ops.ToArray());
                Assert.Equal(replay.Hashes[step], BitConverter.ToUInt64(r, 1));
                ops.Clear();
                step++;
            }
        }
    }

    [Fact]
    public void ARecorderThatKeepsNothingCannotMakeAReplayAndAnEditedMeshStopsIt()
    {
        var crowd = new Crowd(CrowdWorlds.Arena(10, 10));
        var rec = new CrowdRecorder(crowd, CrowdWorlds.Dt);
        Assert.Throws<InvalidOperationException>(() => rec.ToReplay());
        Assert.Throws<ArgumentOutOfRangeException>(() => new CrowdRecorder(crowd, 0));
        var edited = new Crowd(CrowdWorlds.Arena(10, 10));
        var rec2 = new CrowdRecorder(edited, CrowdWorlds.Dt);
        edited.Mesh.SetPolyFlags(edited.Mesh.GetPolyRef(edited.Mesh.GetSlotRef(0), 0), 7);
        Assert.Throws<InvalidOperationException>(() => rec2.Step());
    }

    [Fact]
    public void SnapshotStartsAReplayMidRun()
    {
        var crowd = new Crowd(CrowdWorlds.Arena(24, 16));
        var rec = new CrowdRecorder(crowd, CrowdWorlds.Dt);
        Agent a = rec.Add(new Vector3D(2, 0, 3), Agent.Small);
        rec.SetDestination(a, new Vector3D(20, 0, 8));
        for (int i = 0; i < 25; i++)
        {
            rec.Step();
        }

        var w = new ArrayBufferWriter<byte>();
        rec.WriteSnapshot(w);
        var player = new CrowdPlayer(w.WrittenSpan);
        Assert.Equal(25, player.Crowd.StepCount);
        Assert.Equal(crowd.StateHash(), player.Crowd.StateHash());
        rec.Step();
        player.Step();
        Assert.Equal(crowd.StateHash(), player.Crowd.StateHash());
    }

    [Fact]
    public void DamagedReplaysAreTypedErrors()
    {
        byte[] bytes = Record(steps: 12).Replay.ToBytes();
        for (int n = 0; n < bytes.Length; n += n < 200 ? 1 : 41)
        {
            Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(bytes.AsSpan(0, n)));
        }

        byte[] huge = (byte[])bytes.Clone();
        BitConverter.GetBytes(int.MaxValue).CopyTo(huge, 8);
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(huge));
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(new byte[] { 200 }, new List<CrowdOp>()));
        var rng = new Random(7);
        for (int round = 0; round < 200; round++)
        {
            byte[] m = (byte[])bytes.Clone();
            m[rng.Next(m.Length)] ^= (byte)(1 + rng.Next(255));
            try
            {
                CrowdReplay.Read(m).FindMismatch();
            }
            catch (OnePathFormatException)
            {
            }
        }
    }
}
