using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OnePath.Replay;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class HostileTests
{
    private static byte[] Crowd2D => File.ReadAllBytes(Worlds.DataPath("crowd2d.oprp"));

    [Fact]
    public void EveryTruncationOfAReplayIsATypedError()
    {
        byte[] file = Crowd2D;
        for (int n = 0; n < file.Length; n += Math.Max(1, n < 400 ? 1 : 37))
        {
            Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(file.AsSpan(0, n)));
            Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(file.AsMemory(0, n), "cut"));
        }

        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(file.AsSpan(0, file.Length - 1)));
        byte[] extra = [.. file, 0];
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(extra));
    }

    [Fact]
    public void OversizedAndWrongHeadersAreRejectedBeforeAllocating()
    {
        byte[] file = Crowd2D;
        // Setup length and step count claiming far more than the file holds.
        byte[] a = (byte[])file.Clone();
        BitConverter.GetBytes(int.MaxValue).CopyTo(a, 8);
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(a));
        int setupLength = BitConverter.ToInt32(file, 8);
        byte[] b = (byte[])file.Clone();
        BitConverter.GetBytes(int.MaxValue).CopyTo(b, 12 + setupLength);
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(b));
        BitConverter.GetBytes(-1).CopyTo(b, 12 + setupLength);
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(b));
        byte[] c = (byte[])file.Clone();
        c[4] = 2;
        Assert.Throws<CrowdReplayException>(() => CrowdReplay.Read(c));
        byte[] d = (byte[])file.Clone();
        d[0] = (byte)'X';
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(d, "x"));
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(Array.Empty<byte>(), "empty"));
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(new byte[] { 1, 2, 3, 4, 5, 6, 7, 8, 9 }, "garbage"));
    }

    [Fact]
    public void RandomDamageNeverEscapesAsAnythingButATypedError()
    {
        byte[] file = Crowd2D;
        var rng = new Random(1234);
        int opened = 0, refused = 0;
        for (int round = 0; round < 600; round++)
        {
            byte[] m = (byte[])file.Clone();
            int edits = 1 + rng.Next(4);
            for (int e = 0; e < edits; e++)
            {
                m[rng.Next(m.Length)] = (byte)rng.Next(256);
            }

            try
            {
                IScopeSource source = new OnePathModule().Open(m, "damaged");
                var history = new History(source);
                history.Scan();
                opened++;
            }
            catch (ScopeFormatException)
            {
                refused++;
            }
        }

        Assert.True(refused > 0 && opened + refused == 600);
    }

    [Fact]
    public void StepOperationsAreCheckedBeforeAnythingApplies()
    {
        var into = new List<CrowdOp>();
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(new byte[] { 9 }, into));
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(new byte[] { 2, 1, 2 }, into));
        // A destination with a NaN coordinate.
        byte[] nan = new byte[1 + 8 + 24];
        nan[0] = 3;
        BitConverter.GetBytes(double.NaN).CopyTo(nan, 9);
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(nan, into));
        // A coordinate beyond the library's range.
        BitConverter.GetBytes(1e300).CopyTo(nan, 9);
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(nan, into));
        Assert.Throws<CrowdReplayException>(() => CrowdOps.Parse(new byte[CrowdOps.MaxStepBytes + 1], into));
        Assert.Throws<ArgumentException>(() => CrowdOps.Write(CrowdOp.Remove(1) with { Kind = (CrowdOpKind)77 }, new ArrayBufferWriter<byte>()));

        // An operation naming a missing agent is a typed error from the player, and the world reports it as a format error.
        var w = new ArrayBufferWriter<byte>();
        CrowdOps.Write(CrowdOp.Remove(424242), w);
        IScopeWorld world = new OnePathModule().Open(Crowd2D, "x").CreateWorld();
        Assert.Throws<ScopeFormatException>(() => world.Advance(w.WrittenSpan, TimelineSink.None));
    }

    [Fact]
    public void BuildInputsAreCheckedLikeReplays()
    {
        byte[] file = File.ReadAllBytes(Worlds.DataPath("terrain.opbi"));
        for (int n = 0; n < file.Length; n += 13)
        {
            Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(file.AsMemory(0, n), "cut"));
        }

        byte[] huge = (byte[])file.Clone();
        BitConverter.GetBytes(int.MaxValue).CopyTo(huge, 104);
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(huge, "huge"));
        byte[] badIndex = (byte[])file.Clone();
        BitConverter.GetBytes(10_000).CopyTo(badIndex, badIndex.Length - 4);
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(badIndex, "index"));
        byte[] badCell = (byte[])file.Clone();
        BitConverter.GetBytes(-1.0).CopyTo(badCell, 8 + 32);
        Assert.Throws<ScopeFormatException>(() => new OnePathModule().Open(badCell, "cell"));
        var rng = new Random(99);
        for (int round = 0; round < 300; round++)
        {
            byte[] m = (byte[])file.Clone();
            m[rng.Next(m.Length)] = (byte)rng.Next(256);
            try
            {
                var h = new History(new OnePathModule().Open(m, "damaged"));
                h.Seek("stage", 7);
            }
            catch (Exception e) when (e is ScopeFormatException or ArgumentException)
            {
            }
        }
    }
}
