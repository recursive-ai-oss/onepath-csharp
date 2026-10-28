using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

// The span forms of the crowd state: size, save into caller memory, restore from a span, the state hash, and that none of
// them allocates in steady state (measured per thread with AllocationProbe, so other tests running at the same time cannot disturb it).
public class CrowdStateSpanTests
{
    private const ulong FnvBasis = 0xCBF29CE484222325UL, FnvPrime = 0x100000001B3UL;

    private static ulong Fnv(ReadOnlySpan<byte> bytes)
    {
        ulong h = FnvBasis;
        foreach (byte b in bytes)
        {
            h = (h ^ b) * FnvPrime;
        }
        return h;
    }

    [Fact]
    public void SpanSaveEqualsArraySave_AndHashIsFnvOfTheBytes()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        for (int s = 0; s < 120; s += 20)
        {
            CrowdRollback.Run(sim, s, s + 20, false);
            byte[] bytes = sim.Crowd.SaveState();
            Assert.Equal(bytes.Length, sim.Crowd.GetStateSize());
            var buf = new byte[bytes.Length + 7];
            Assert.True(sim.Crowd.TrySaveState(buf, out int written));
            Assert.Equal(bytes.Length, written);
            Assert.True(bytes.AsSpan().SequenceEqual(buf.AsSpan(0, written)));
            Assert.Equal(Fnv(bytes), sim.Crowd.StateHash());

            Assert.False(sim.Crowd.TrySaveState(new byte[bytes.Length - 1], out int none));
            Assert.Equal(0, none);
        }
    }

    [Fact]
    public void HashSurvivesSaveRestoreAndMatchesAcrossThreads()
    {
        CrowdSim one = CrowdRollback.Populated(1), many = CrowdRollback.Populated(4);
        for (int s = 0; s < 150; s += 30)
        {
            CrowdRollback.Run(one, s, s + 30, false);
            CrowdRollback.Run(many, s, s + 30, false);
            Assert.Equal(one.Crowd.StateHash(), many.Crowd.StateHash());
        }
        ulong h = one.Crowd.StateHash();
        byte[] snap = one.Crowd.SaveState();
        CrowdRollback.Run(one, 150, 200, false);
        Assert.NotEqual(h, one.Crowd.StateHash());
        one.Crowd.RestoreState(snap);
        Assert.Equal(h, one.Crowd.StateHash());
        Assert.True(snap.AsSpan().SequenceEqual(one.Crowd.SaveState()));
    }

    [Fact]
    public void HashChangesWithAnyStateChange()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 60, false);
        var agents = new List<Agent>(sim.Crowd);
        byte[] snap = sim.Crowd.SaveState();
        ulong baseline = sim.Crowd.StateHash();
        var seen = new HashSet<ulong> { baseline };

        void Check(string what, Action change)
        {
            change();
            ulong h = sim.Crowd.StateHash();
            Assert.True(seen.Add(h), $"{what} did not change the hash");
            sim.Crowd.RestoreState(snap);
            Assert.Equal(baseline, sim.Crowd.StateHash());
        }

        Check("a step", () => sim.Crowd.Step(CrowdWorlds.Dt));
        Check("priority", () => agents[0].Priority = agents[0].Priority + 1);
        Check("teleport", () => agents[1].Teleport(agents[1].Position + new Vector3D(0.25, 0, 0)));
        Check("stop", () => agents[2].Stop());
        Check("destination", () => agents[3].SetDestination(agents[4].Position));
        Check("remove", () => sim.Crowd.Remove(agents[5]));
        Check("add", () => sim.Crowd.Add(agents[6].Position, Agent.Small));
    }

    [Theory]
    [InlineData(1)]
    [InlineData(4)]
    public void RollbackAndResimulateGiveTheSameHashes(int threads)
    {
        CrowdSim sim = CrowdRollback.Populated(threads);
        CrowdRollback.Run(sim, 0, 80, false);
        byte[] snap = new byte[sim.Crowd.GetStateSize()];
        Assert.True(sim.Crowd.TrySaveState(snap, out _));
        var first = new List<ulong>();
        CrowdRollback.Run(sim, 80, 140, false, _ => first.Add(sim.Crowd.StateHash()));
        sim.Crowd.RestoreState(snap);
        var second = new List<ulong>();
        CrowdRollback.Run(sim, 80, 140, false, _ => second.Add(sim.Crowd.StateHash()));
        Assert.Equal(first, second);
    }

    [Fact]
    public void RejectedRestoreLeavesTheCrowdUnchanged()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 50, false);
        byte[] snap = sim.Crowd.SaveState();
        CrowdRollback.Run(sim, 50, 60, false);
        ulong before = sim.Crowd.StateHash();
        int rejected = 0;
        for (int at = 0; at < snap.Length; at += 17)
        {
            byte[] bad = (byte[])snap.Clone();
            bad[at] ^= 0xFF;
            try
            {
                sim.Crowd.RestoreState(bad);
                before = sim.Crowd.StateHash(); // a flipped byte that is still a valid state: the new baseline
            }
            catch (CrowdStateException)
            {
                rejected++;
                Assert.Equal(before, sim.Crowd.StateHash());
            }
        }
        Assert.True(rejected > 0);
        for (int cut = 0; cut < snap.Length; cut += 53)
        {
            Assert.Throws<CrowdStateException>(() => sim.Crowd.RestoreState(snap.AsSpan(0, cut)));
            Assert.Equal(before, sim.Crowd.StateHash());
        }
    }

    [Fact]
    public void SteadyStateHashSizeSaveAndRestoreAllocateNothing()
    {
        CrowdSim sim = CrowdRollback.Populated(1);
        CrowdRollback.Run(sim, 0, 80, false);
        byte[] snap = sim.Crowd.SaveState();
        byte[] buf = new byte[snap.Length * 2];
        // Warm up: the first calls size the scratch, JIT and tiering.
        for (int i = 0; i < 3; i++)
        {
            sim.Crowd.StateHash();
            sim.Crowd.GetStateSize();
            sim.Crowd.TrySaveState(buf, out _);
            sim.Crowd.RestoreState(snap);
            CrowdRollback.Run(sim, 80, 83, false);
            sim.Crowd.RestoreState(snap);
        }
        CrowdRollback.Run(sim, 80, 84, false); // a few steps on: same agents, different dynamic state
        byte[] later = sim.Crowd.SaveState();

        ulong h = 0;
        int size = 0;
        bool ok = false;
        long allocated = AllocationProbe.SteadyState(() =>
        {
            for (int i = 0; i < 20; i++)
            {
                h = sim.Crowd.StateHash();
                size += sim.Crowd.GetStateSize();
                ok = sim.Crowd.TrySaveState(buf, out _);
                sim.Crowd.RestoreState(snap);
                sim.Crowd.RestoreState(later);
            }
        });
        Assert.True(ok);
        Assert.True(size > 0 && h != 0);
        Assert.Equal(0, allocated);
        Assert.True(later.AsSpan().SequenceEqual(sim.Crowd.SaveState()));
    }

    [Fact]
    public void HugeButValidWallClearanceStepsWithoutThrowing()
    {
        // Found by fuzzing restored state: the wall query refused a radius beyond 2^22.
        CrowdSim sim = CrowdRollback.Populated(1);
        var agents = new List<Agent>(sim.Crowd);
        sim.Crowd.Add(agents[0].Position, Agent.Small with { WallClearance = 1e9 });
        sim.Crowd.Step(CrowdWorlds.Dt);
    }
}
