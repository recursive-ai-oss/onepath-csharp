using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Replay;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class ForkLiveTests
{
    private static byte[] File2D => File.ReadAllBytes(Worlds.DataPath("crowd2d.oprp"));

    private static List<ulong> Hashes(History h, long from, long to)
    {
        var l = new List<ulong>();
        for (long s = from; s <= to; s++)
        {
            h.Seek(s);
            l.Add(h.StateHash);
        }

        return l;
    }

    private static (long Agent, PokeCall[] Calls)[] Pokes(History h)
    {
        var entities = new List<EntityInfo>();
        h.World.CollectEntities(entities);
        long Agent(int n) => (long)entities.Where(e => e.Kind == "agent").ElementAt(n).Id.Value & 0x3FFF_FFFF_FFFF_FFFF;
        EntityId E(int n) => ScopeInfo.Agent(Agent(n));
        return
        [
            (0, [new PokeCall("set_target", [ScopeValue.OfEntity(E(3)), ScopeValue.OfVector(new Vector3(5, 5, 0))])]),
            (0, [new PokeCall("add_agent", [ScopeValue.OfVector(new Vector3(6, 6, 0))])]),
            (0, [new PokeCall("remove_agent", [ScopeValue.OfEntity(E(4))])]),
            (0, [new PokeCall("stop_agent", [ScopeValue.OfEntity(E(5))])]),
            (0, [new PokeCall("teleport_agent", [ScopeValue.OfEntity(E(6)), ScopeValue.OfVector(new Vector3(30, 20, 0))])]),
        ];
    }

    [Fact]
    public void EveryPokeForksWithoutTouchingTheOriginalAndTheForkReplays()
    {
        IScopeSource source = new OnePathModule().Open(File2D, "crowd2d");
        var original = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        List<ulong> before = Hashes(original, 0, original.EndStep);
        byte[][] inputs = Enumerable.Range(0, (int)original.EndStep).Select(i => original.Recording.InputAt(i).ToArray()).ToArray();
        var setupBefore = original.Recording.Setup.ToArray();

        foreach ((long _, PokeCall[] calls) in Pokes(original))
        {
            Branch branch = original.Fork(100);
            branch.History.Seek(100);
            branch.Poke(calls[0]);
            Assert.True(branch.Step());
            for (int i = 0; i < 40; i++)
            {
                Assert.True(branch.Step());
            }

            Assert.Equal(100, branch.DivergenceStep);

            // The original is untouched: hashes, inputs and setup bytes are exactly what they were.
            Assert.Equal(before, Hashes(original, 0, original.EndStep));
            for (int i = 0; i < inputs.Length; i++)
            {
                Assert.Equal(inputs[i], original.Recording.InputAt(i).ToArray());
            }

            Assert.Equal(setupBefore, original.Recording.Setup.ToArray());

            // The fork's saved replay opens and reproduces the fork's hashes; the fork differs from the original after the poke.
            long end = branch.History.EndStep;
            List<ulong> forkHashes = Hashes(branch.History, 0, end);
            var bytes = new ArrayBufferWriter<byte>();
            branch.History.Source.WriteReplay(bytes);
            var reopened = new History(new OnePathModule().Open(bytes.WrittenMemory, "fork"), new ScopeHistoryOptions { KeyframeInterval = 64 });
            Assert.Equal(forkHashes, Hashes(reopened, 0, end));
            reopened.Scan();
            Assert.Null(reopened.FirstHashMismatch);
            Assert.NotEqual(forkHashes[101], before[101]);
            Assert.Equal(forkHashes[100], before[100]);

            Difference? d = original.Diff(branch.History, 101);
            Assert.NotNull(d);
            Assert.True(ScopeInfo.IsAgent(d!.Value.Entity, out _) || d.Value.Entity == ScopeInfo.CrowdRoot);
            Assert.False(string.IsNullOrEmpty(d.Value.Field));
        }
    }

    [Fact]
    public void DiffNamesThePlantedAgentAndField()
    {
        IScopeSource source = new OnePathModule().Open(File2D, "x");
        IScopeWorld wa = source.CreateWorld(), wb = source.CreateWorld();
        wa.Advance(source.Recording.InputAt(0).Span, TimelineSink.None);
        wb.Advance(source.Recording.InputAt(0).Span, TimelineSink.None);
        var entities = new List<EntityInfo>();
        wa.CollectEntities(entities);
        EntityId target = entities.First(e => e.Kind == "agent" && e.Name == "agent 7").Id;
        Assert.Null(wa.FirstDifference(wb));
        var call = new PokeCall("set_target", [ScopeValue.OfEntity(target), ScopeValue.OfVector(new Vector3(3, 3, 0))]);
        var w = new ArrayBufferWriter<byte>();
        wb.EncodePoke(call, w);
        var sink = TimelineSink.None;
        wa.Advance(default, sink);
        wb.Advance(w.WrittenSpan, sink);
        Difference? d = wa.FirstDifference(wb);
        Assert.NotNull(d);
        Assert.Equal(target, d!.Value.Entity);
        Assert.Equal("position", d.Value.Field); // the first field in the order position, velocity, destination, state, remaining distance
    }

    [Fact]
    public void PokesThatCannotApplyAreRefused()
    {
        IScopeWorld w = new OnePathModule().Open(File2D, "x").CreateWorld();
        var into = new ArrayBufferWriter<byte>();
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("remove_agent", [ScopeValue.OfEntity(ScopeInfo.Agent(9999))]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("set_target", [ScopeValue.OfVector(default)]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("explode", []), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("add_agent", [ScopeValue.OfVector(new Vector3(float.NaN, 0, 0))]), into));
        Assert.Equal(0, into.WrittenCount);
    }

    [Theory]
    [InlineData(0)]
    [InlineData(100)]
    public void LiveSourceFollowsTheHostFromAnAttachPoint(int attachAt)
    {
        var module = new OnePathModule();
        Crowd crowd = Worlds.Arena2D().CreateCrowd();
        var rec = new CrowdRecorder(crowd, Worlds.Dt);
        var hostHashes = new Dictionary<long, ulong> { [0] = crowd.StateHash() };
        ILiveSource? live = null;
        long attachStep = -1;
        rec.Record = record => live?.AppendRecord(record);

        void Attach()
        {
            var snapshot = new ArrayBufferWriter<byte>();
            rec.WriteSnapshot(snapshot);
            live = module.OpenLive(ScopeInfo.LibraryVersion, ScopeInfo.FormatVersion, snapshot.WrittenMemory, "live");
            attachStep = crowd.StepCount;
        }

        if (attachAt == 0)
        {
            Attach();
        }

        Worlds.RunScript(new Worlds.RecordingHost(rec), 11, 0, 24, 18, 22, 39, afterStep: s =>
        {
            hostHashes[crowd.StepCount] = crowd.StateHash();
            if (s + 1 == attachAt)
            {
                Attach();
            }
        });

        Assert.NotNull(live);
        Assert.Equal(attachAt, attachStep);
        IScopeSource source = live!.Source;
        Assert.True(source.Recording.IsWritable);
        Assert.Equal(attachAt, source.Recording.StartStep);
        Assert.Equal(Worlds.Steps, source.Recording.EndStep);
        foreach (int interval in new[] { 1, 7, 64 })
        {
            var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = interval });
            foreach (long step in new long[] { Worlds.Steps, attachAt, 150, 101, Worlds.Steps - 1, attachAt + 1 }.Where(s => s >= attachAt && s <= Worlds.Steps))
            {
                history.Seek(step);
                Assert.Equal(hostHashes[step], history.StateHash);
            }
        }
    }

    [Fact]
    public void LiveRecordsThatAreWrongAreTypedErrors()
    {
        var module = new OnePathModule();
        Crowd crowd = Worlds.Arena2D().CreateCrowd();
        var rec = new CrowdRecorder(crowd, Worlds.Dt);
        var snapshot = new ArrayBufferWriter<byte>();
        rec.WriteSnapshot(snapshot);
        Assert.Throws<ScopeFormatException>(() => module.OpenLive("1", 2, snapshot.WrittenMemory, "x"));
        Assert.Throws<ScopeFormatException>(() => module.OpenLive("1", 1, snapshot.WrittenMemory[..40], "x"));
        ILiveSource live = module.OpenLive("1", 1, snapshot.WrittenMemory, "x");
        Assert.Throws<ScopeFormatException>(() => live.AppendRecord(default));
        Assert.Throws<ScopeFormatException>(() => live.AppendRecord(new byte[] { 9, 1, 2 }));
        Assert.Throws<ScopeFormatException>(() => live.AppendRecord(new byte[] { 2, 1, 2 }));
        Assert.Throws<ScopeFormatException>(() => live.AppendRecord(new byte[] { 1, 99 }));
        Assert.Equal(0, live.Source.Recording.StepCount);
    }
}
