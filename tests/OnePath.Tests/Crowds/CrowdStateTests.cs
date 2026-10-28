using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class CrowdStateTests
{
    private const int Total = 260, SnapAt = 90, DivergeTo = 170;
    private readonly ITestOutputHelper _out;

    public CrowdStateTests(ITestOutputHelper output) => _out = output;

    // The state bytes after every step of the straight run (the one that never rolled back).
    private static List<byte[]> Straight(int threads, bool meshEdits, out int crossingSteps, int upTo = Total)
    {
        CrowdSim sim = CrowdRollback.Populated(threads);
        var states = new List<byte[]>();
        int crossing = 0;
        CrowdRollback.Run(sim, 0, upTo, meshEdits, _ =>
        {
            states.Add(sim.Crowd.SaveState());
            crossing += CrowdRollback.Crossing(sim.Crowd);
        });
        crossingSteps = crossing;
        return states;
    }

    private static void AssertSame(List<byte[]> expected, int firstStep, List<byte[]> actual, string what)
    {
        for (int i = 0; i < actual.Count; i++)
        {
            Assert.True(expected[firstStep + i].AsSpan().SequenceEqual(actual[i]), $"{what}: state differs after step {firstStep + i}");
        }
    }

    [Theory]
    [InlineData(1, true)]
    [InlineData(4, true)]
    [InlineData(1, false)]
    [InlineData(3, false)]
    public void RollbackInPlaceEqualsStraightRun(int threads, bool meshEdits)
    {
        List<byte[]> straight = Straight(1, meshEdits, out int straightCrossing);
        CrowdSim sim = CrowdRollback.Populated(threads);
        CrowdRollback.Run(sim, 0, SnapAt, meshEdits);
        byte[] snap = meshEdits ? sim.Crowd.SaveStateWithMesh() : sim.Crowd.SaveState();
        Assert.True(straight[SnapAt - 1].AsSpan().SequenceEqual(sim.Crowd.SaveState()), "run before the snapshot differs");

        var atSnap = new List<Agent>(sim.Crowd);

        // Another timeline: the script goes on, then more is done to the crowd and the mesh that the straight run never did.
        CrowdRollback.Run(sim, SnapAt, DivergeTo, meshEdits);
        atSnap.RemoveAll(a => !a.IsActive); // the script removed some in the window
        Assert.True(atSnap.Count >= 4);
        Agent removed = atSnap[0], teleported = atSnap[1], keep = atSnap[3];
        sim.Crowd.Remove(removed);
        teleported.Teleport(new Vector3D(0.5, 0, 0.5));
        atSnap[2].Priority = 9;
        if (meshEdits)
        {
            NavMeshMutations.Apply(sim.Mesh, new NavMeshWorlds.Rng(77), CrowdRollback.WorldSeed);
        }

        if (meshEdits)
        {
            sim.Crowd.RestoreStateWithMesh(snap);
        }
        else
        {
            sim.Crowd.RestoreState(snap);
        }
        Assert.True(straight[SnapAt - 1].AsSpan().SequenceEqual(sim.Crowd.SaveState()), "restored state is not the saved one");
        Assert.Same(keep, sim.Crowd.GetAgent(keep.Id));
        Assert.True(keep.IsActive);
        Assert.False(removed.IsActive); // the old object stays inactive; the agent is back under a new object with the same id
        Assert.NotNull(sim.Crowd.GetAgent(removed.Id));
        var replay = new List<byte[]>();
        int crossing = 0;
        CrowdRollback.Run(sim, SnapAt, Total, meshEdits, _ =>
        {
            replay.Add(sim.Crowd.SaveState());
            crossing += CrowdRollback.Crossing(sim.Crowd);
        });
        AssertSame(straight, SnapAt, replay, $"threads {threads}, edits {meshEdits}");
        _out.WriteLine($"straight crossing-steps {straightCrossing}, replay crossing-steps {crossing}");
        Assert.True(crossing > 0, "the replayed window has no agent crossing an off-mesh link");
    }

    [Theory]
    [InlineData(1, 4)]
    [InlineData(4, 1)]
    public void FreshCrowdOverFreshMeshContinuesTheRun(int threadsBefore, int threadsAfter)
    {
        List<byte[]> straight = Straight(threadsBefore, true, out _);
        CrowdSim a = CrowdRollback.Populated(threadsBefore);
        CrowdRollback.Run(a, 0, SnapAt, true);
        byte[] snap = a.Crowd.SaveStateWithMesh();

        CrowdSim b = CrowdRollback.EmptyOverWorld(threadsAfter);
        b.Crowd.RestoreStateWithMesh(snap);
        Assert.True(straight[SnapAt - 1].AsSpan().SequenceEqual(b.Crowd.SaveState()));
        var replay = new List<byte[]>();
        CrowdRollback.Run(b, SnapAt, Total, true, _ => replay.Add(b.Crowd.SaveState()));
        AssertSame(straight, SnapAt, replay, "fresh crowd");
    }

    [Fact]
    public void RoundTripAndRepeatedSavesAreByteIdentical()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 120, true);
        byte[] one = sim.Crowd.SaveState();
        Assert.True(one.AsSpan().SequenceEqual(sim.Crowd.SaveState()));
        sim.Crowd.RestoreState(one);
        Assert.True(one.AsSpan().SequenceEqual(sim.Crowd.SaveState()));
        byte[] withMesh = sim.Crowd.SaveStateWithMesh();
        sim.Crowd.RestoreStateWithMesh(withMesh);
        Assert.True(withMesh.AsSpan().SequenceEqual(sim.Crowd.SaveStateWithMesh()));
        Assert.True(one.AsSpan().SequenceEqual(sim.Crowd.SaveState()));
        CrowdSim fresh = CrowdRollback.EmptyOverWorld(2);
        fresh.Crowd.RestoreStateWithMesh(withMesh);
        Assert.True(withMesh.AsSpan().SequenceEqual(fresh.Crowd.SaveStateWithMesh()));
    }

    [Fact]
    public void StepCountSlotsAndIdsComeBack()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 60, false);
        byte[] snap = sim.Crowd.SaveState();
        long steps = sim.Crowd.StepCount;
        int count = sim.Crowd.Count;
        Agent added = sim.Crowd.Add(sim.Agents[0].Position);
        long id = added.Id;
        CrowdRollback.Run(sim, 60, 70, false);
        sim.Crowd.RestoreState(snap);
        Assert.Equal(steps, sim.Crowd.StepCount);
        Assert.Equal(count, sim.Crowd.Count);
        Assert.False(added.IsActive);
        Assert.Null(sim.Crowd.GetAgent(id));
        // The next agent gets the id and slot the snapshot says (ids are never reused within one timeline, but a rollback rewinds the counter).
        Assert.Equal(id, sim.Crowd.Add(sim.Agents[0].Position).Id);
    }

    [Fact]
    public void EveryTruncationIsATypedError()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        sim.Crowd.Step(CrowdRollback.Dt);
        CrowdRollback.Run(sim, 0, 30, false);
        byte[] snap = sim.Crowd.SaveState();
        byte[] before = sim.Crowd.SaveState();
        for (int n = 0; n < snap.Length; n++)
        {
            byte[] cut = snap.AsSpan(0, n).ToArray();
            Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreState(cut));
        }
        Assert.True(before.AsSpan().SequenceEqual(sim.Crowd.SaveState()), "a rejected restore changed the crowd");
        byte[] trailing = new byte[snap.Length + 1];
        snap.CopyTo(trailing, 0);
        Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreState(trailing));

        byte[] both = sim.Crowd.SaveStateWithMesh();
        for (int n = 0; n < both.Length; n += Math.Max(1, both.Length / 400))
        {
            byte[] cut = both.AsSpan(0, n).ToArray();
            Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreStateWithMesh(cut));
        }
        Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreStateWithMesh(snap));
        Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreState(both));
        Assert.True(before.AsSpan().SequenceEqual(sim.Crowd.SaveState()));
    }

    [Fact]
    public void CorruptionIsATypedErrorAndLeavesTheCrowdAloneOrRestoresCleanly()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 40, false);
        byte[] snap = sim.Crowd.SaveState();
        var rng = new NavMeshWorlds.Rng(31337);
        int rejected = 0, accepted = 0;
        for (int trial = 0; trial < 1500; trial++)
        {
            byte[] bad = (byte[])snap.Clone();
            int flips = 1 + rng.Int(3);
            for (int k = 0; k < flips; k++)
            {
                // Half the flips go to the first 64 bytes (header and counts), the rest anywhere.
                int at = rng.Int(2) == 0 ? rng.Int(Math.Min(64, bad.Length)) : rng.Int(bad.Length);
                bad[at] ^= (byte)(1 + rng.Int(255));
            }
            CrowdSim victim = CrowdRollback.Populated(1);
            byte[] before = victim.Crowd.SaveState();
            try
            {
                victim.Crowd.RestoreState(bad);
                accepted++;
                // A restore that was accepted must leave a crowd that steps without throwing.
                victim.Crowd.Step(CrowdRollback.Dt);
                victim.Crowd.Step(CrowdRollback.Dt);
            }
            catch (CrowdStateException)
            {
                rejected++;
                Assert.True(before.AsSpan().SequenceEqual(victim.Crowd.SaveState()), $"trial {trial}: a rejected restore changed the crowd");
            }
        }
        _out.WriteLine($"rejected {rejected}, accepted {accepted}");
        Assert.True(rejected > 500);
    }

    [Fact]
    public void WrongSettingsAreRefused()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        byte[] snap = sim.Crowd.SaveState();
        var other = new Crowd(sim.Mesh, new CrowdOptions { MaxWallsPerAgent = 24 });
        Assert.Throws<CrowdStateException>(() => other.RestoreState(snap));
        var small = new Crowd(sim.Mesh, new CrowdOptions { MaxAgents = 4 });
        Assert.Throws<CrowdStateException>(() => small.RestoreState(snap));
    }
}
