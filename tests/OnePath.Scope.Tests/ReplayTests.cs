using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Replay;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class ReplayTests
{
    private static readonly long[] SeekPlan = [0, 1, 5, 40, 41, 100, 239, 240, 120, 60, 3, 200, 17, 64, 63, 65, 128, 129, 0, 240, 239, 1];

    [Fact]
    public void CommittedFilesAreWhatTheGeneratorsWrite()
    {
        // Run with ONEPATH_REGENERATE_TESTDATA=<repo testdata/scope folder> to rewrite the files after a deliberate format change.
        byte[] two = Worlds.Record2D(out _).ToBytes(), three = Worlds.Record3D(out _).ToBytes(), build = Worlds.BuildInputBytes();
        string? regen = Environment.GetEnvironmentVariable("ONEPATH_REGENERATE_TESTDATA");
        if (!string.IsNullOrEmpty(regen))
        {
            File.WriteAllBytes(Path.Combine(regen, "crowd2d.oprp"), two);
            File.WriteAllBytes(Path.Combine(regen, "crowd3d.oprp"), three);
            File.WriteAllBytes(Path.Combine(regen, "terrain.opbi"), build);
        }

        (string, byte[])[] grids = [("grid-astar.opgr", Worlds.GridSearchFile(GridAlgorithm.AStar)), ("grid-jps.opgr", Worlds.GridSearchFile(GridAlgorithm.Jps)), ("grid-astar2.opgr", Worlds.GridSearchFile(GridAlgorithm.AStar, 2)), ("grid-flow.opgr", Worlds.GridFlowFile())];
        if (!string.IsNullOrEmpty(regen))
        {
            foreach ((string file, byte[] bytes) in grids)
            {
                File.WriteAllBytes(Path.Combine(regen, file), bytes);
            }
        }

        foreach ((string file, byte[] bytes) in grids)
        {
            Assert.Equal(bytes, File.ReadAllBytes(Worlds.DataPath(file)));
        }

        Assert.Equal(two, File.ReadAllBytes(Worlds.DataPath("crowd2d.oprp")));
        Assert.Equal(three, File.ReadAllBytes(Worlds.DataPath("crowd3d.oprp")));
        Assert.Equal(build, File.ReadAllBytes(Worlds.DataPath("terrain.opbi")));
    }

    [Theory]
    [InlineData("crowd2d.oprp", 2, 1)]
    [InlineData("crowd2d.oprp", 2, 7)]
    [InlineData("crowd2d.oprp", 2, 64)]
    [InlineData("crowd2d.oprp", 2, 1000)]
    [InlineData("crowd3d.oprp", 3, 1)]
    [InlineData("crowd3d.oprp", 3, 7)]
    [InlineData("crowd3d.oprp", 3, 64)]
    [InlineData("crowd3d.oprp", 3, 1000)]
    public void SeekLandsOnTheLibrarysHashes(string file, int dimensions, int interval)
    {
        // The reference is the committed replay's own hashes AND a bare crowd run by the same script with no recorder.
        CrowdReplay replay = CrowdReplay.Read(File.ReadAllBytes(Worlds.DataPath(file)));
        List<ulong> bare = BareHashes(dimensions);
        Assert.Equal(replay.Hashes.ToArray(), bare.ToArray());

        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath(file)), file);
        Assert.Equal(dimensions, source.Info.Dimensions);
        var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = interval });
        ulong start = new CrowdPlayer(replay.Setup.Span).Crowd.StateHash();
        foreach (long step in SeekPlan)
        {
            history.Seek(step);
            Assert.Equal(step == 0 ? start : bare[(int)step - 1], history.StateHash);
        }

        // Stepping one at a time from the start gives the same hashes, and the history finds no mismatch with the file's own list.
        history.Seek(0);
        for (int s = 1; s <= Worlds.Steps; s++)
        {
            Assert.True(history.Step());
            Assert.Equal(bare[s - 1], history.StateHash);
        }

        history.Scan();
        Assert.Null(history.FirstHashMismatch);
        Assert.Empty(history.Findings);
    }

    [Fact]
    public void ReplayRoundTripsByteForByte()
    {
        byte[] file = File.ReadAllBytes(Worlds.DataPath("crowd3d.oprp"));
        IScopeSource source = new OnePathModule().Open(file, "x");
        var w = new System.Buffers.ArrayBufferWriter<byte>();
        source.WriteReplay(w);
        Assert.Equal(file, w.WrittenSpan.ToArray());
    }

    [Fact]
    public void TimelineShowsAddsRemovesAndOrders()
    {
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("crowd2d.oprp")), "x");
        var history = new History(source);
        history.Scan();
        TrackData agents = history.Tracks.Single(t => t.Info.Id == "agents");
        Assert.Equal(30 + 4 + 3 + 2 + 2, agents.Samples.Count);
        Assert.Equal(41, agents.Samples.First(s => s.Text!.Contains("added") && s.Step > 5).Step);
        Assert.Equal(34, history.Tracks.Single(t => t.Info.Id == "agent-count").Samples[40].Value);
    }

    private static List<ulong> BareHashes(int dimensions)
    {
        var hashes = new List<ulong>();
        Crowd crowd;
        if (dimensions == 2)
        {
            crowd = Worlds.Arena2D().CreateCrowd();
            Worlds.RunScript(new Worlds.BareHost(crowd), 11, 0, 24, 18, 22, 39, afterStep: _ => hashes.Add(crowd.StateHash()));
        }
        else
        {
            crowd = Worlds.Mesh3D().CreateCrowd();
            Worlds.RunScript(new Worlds.BareHost(crowd), 12, 0, 24, 7, 14, 23, afterStep: _ => hashes.Add(crowd.StateHash()));
        }

        return hashes;
    }
}
