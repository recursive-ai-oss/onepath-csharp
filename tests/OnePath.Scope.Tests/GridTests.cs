using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;
using Xunit;

namespace RecursiveAI.OnePath.Scope.Tests;

public class GridTests
{
    public static IEnumerable<object[]> Files() =>
    [
        ["grid-astar.opgr", 24],
        ["grid-jps.opgr", 24],
        ["grid-astar2.opgr", 24],
        ["grid-flow.opgr", 60],
    ];

    // The engine state the world saved: the tail of its snapshot (see GridWorld.WriteSnapshot).
    private static byte[] EngineState(IScopeWorld world)
    {
        var w = new ArrayBufferWriter<byte>();
        world.WriteSnapshot(w);
        ReadOnlySpan<byte> s = w.WrittenSpan;
        int gl = BinaryPrimitives.ReadInt32LittleEndian(s[8..]);
        int n = 12 + gl;
        int pc = BinaryPrimitives.ReadInt32LittleEndian(s[n..]);
        n += 4 + (pc * 8);
        int sl = BinaryPrimitives.ReadInt32LittleEndian(s[n..]);
        return s.Slice(n + 4, sl).ToArray();
    }

    /// <summary>Runs the file straight on the library's own types, with no module code, and returns the engine state after every step (index 0 is before step 0).</summary>
    private static List<byte[]> Direct(byte[] file, int upTo, int partialIterations = 0)
    {
        GridRun.Parsed run = GridRun.Read(file, "x");
        Grid grid = Grid.FromState(run.GridState);
        var options = new GridOptions { Algorithm = run.Algorithm, Connectivity = run.Connectivity, Diagonals = run.Diagonals, AgentSize = run.AgentSize, IterationBudget = run.Slice };
        bool flow = run.Kind == 2;
        Cell start = run.Points[0], goal = run.Points[^1];
        Cell[] goals = run.Points;
        GridSearch? search = flow ? null : grid.CreateSearch(options);
        GridFlowField? field = flow ? grid.CreateFlowField(goals, options) : null;
        search?.TryBegin(start, goal);
        var states = new List<byte[]> { flow ? field!.SaveState() : search!.SaveState() };
        for (int s = 0; s < upTo; s++)
        {
            byte[] ops = run.Steps[s];
            bool edited = ops.Length > 0;
            for (int i = 0; i < ops.Length; i += GridRun.OpBytes)
            {
                int x = BinaryPrimitives.ReadInt32LittleEndian(ops.AsSpan(i + 1)), y = BinaryPrimitives.ReadInt32LittleEndian(ops.AsSpan(i + 5));
                switch ((GridRunOpKind)ops[i])
                {
                    case GridRunOpKind.SetArea:
                        grid.SetArea(x, y, ops[i + 9]);
                        break;
                    case GridRunOpKind.SetStart:
                        start = new Cell(x, y);
                        break;
                    default:
                        goal = new Cell(x, y);
                        goals = [goal];
                        break;
                }
            }

            if (edited)
            {
                if (flow)
                {
                    field!.Dispose();
                    field = grid.CreateFlowField(goals, options);
                }
                else
                {
                    search!.TryBegin(start, goal);
                }
            }

            bool partial = partialIterations > 0 && s == upTo - 1;
            if (flow ? field!.Status == FlowFieldStatus.Running : search!.Status == PathStatus.InProgress)
            {
                int n = partial ? partialIterations : run.Slice;
                if (flow)
                {
                    field!.Step(n);
                }
                else
                {
                    search!.Step(n);
                }
            }

            states.Add(flow ? field!.SaveState() : search!.SaveState());
        }

        return states;
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void SeekGivesTheLibrarysOwnStateAtEveryIntervalAndPath(string file, int slice)
    {
        byte[] bytes = File.ReadAllBytes(Worlds.DataPath(file));
        IScopeSource source = new OnePathModule().Open(bytes, file);
        int steps = (int)source.Recording.StepCount;
        Assert.Equal(file.Contains("flow") ? 60 : 80, steps);
        Assert.Contains(source.Info.Axes, a => a.PositionCount == slice);
        List<byte[]> direct = Direct(bytes, steps);
        long[] plan = [0, 1, 9, 10, 11, 25, 26, 40, 41, 56, steps, 5, steps - 1, 64, 63, 3, 0, steps, 30];
        foreach (int interval in new[] { 1, 7, 64, 1000 })
        {
            var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = interval });
            foreach (long step in plan.Where(p => p <= steps))
            {
                history.Seek(step);
                Assert.Equal(direct[(int)step], EngineState(history.World));
            }
        }

        // The module's hash is a function of the state, so two paths to a step agree.
        var a = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        var b = new History(source, new ScopeHistoryOptions { KeyframeInterval = 64 });
        a.Seek(steps);
        b.Seek(50);
        b.Seek(steps);
        Assert.Equal(a.StateHash, b.StateHash);
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void TheIterationAxisEqualsSteppingTheEngineThatManyPops(string file, int slice)
    {
        byte[] bytes = File.ReadAllBytes(Worlds.DataPath(file));
        IScopeSource source = new OnePathModule().Open(bytes, file);
        Assert.Contains(source.Info.Axes, a => a.Id == "iteration" && a.PositionCount == slice);
        var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        foreach (int step in new[] { 0, 3, 10, 25, 26, 40 })
        {
            foreach (int k in new[] { 1, 2, slice / 2, slice - 1, slice }.Distinct())
            {
                history.Seek(step);
                ulong hash = history.StateHash;
                history.Seek("iteration", k);
                Assert.Equal("iteration", history.ViewAxis);
                Assert.Equal(Direct(bytes, step + 1, k)[step + 1], EngineState(history.View));
                Assert.Equal(hash, history.StateHash);
            }
        }

        // k = slice is the same state the next time step reaches.
        history.Seek(7);
        history.Seek("iteration", slice);
        byte[] viaAxis = EngineState(history.View);
        history.Seek(8);
        Assert.Equal(viaAxis, EngineState(history.World));
    }

    [Fact]
    public void TheFrontierGrowsAndMatchesTheSearch()
    {
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("grid-astar.opgr")), "a");
        var history = new History(source);
        var list = new DrawList();
        int lastClosed = -1;
        foreach (int step in new[] { 1, 3, 6, 9 })
        {
            history.Seek(step);
            var props = new PropertyList();
            Assert.True(history.World.Inspect(ScopeInfo.GridRoot, props));
            int closed = (int)props.Rows.Single(r => r.Name == "closed cells").Value.GetInteger();
            int open = (int)props.Rows.Single(r => r.Name == "open cells").Value.GetInteger();
            int expanded = (int)props.Rows.Single(r => r.Name == "cells expanded").Value.GetInteger();
            Assert.True(closed >= expanded - 1 && closed <= expanded + 1, $"closed {closed} expanded {expanded}");
            Assert.True(open > 0 && closed > lastClosed);
            lastClosed = closed;
            list.Clear();
            history.World.Draw(list, new DrawRequest(1UL << ScopeInfo.GridClosed));
            Assert.Equal(closed, list.Items.Length);
            list.Clear();
            history.World.Draw(list, new DrawRequest(1UL << ScopeInfo.GridOpen));
            Assert.Equal(open, list.Items.Length);
        }

        var rows = new PropertyList();
        for (int guard = 0; guard < 80; guard++)
        {
            history.Step();
            history.World.Inspect(ScopeInfo.GridRoot, rows);
            if (rows.Rows.Single(r => r.Group == "Run" && r.Name == "status").Value.GetText() == "Found")
            {
                break;
            }
        }

        Assert.Equal("Found", rows.Rows.Single(r => r.Group == "Run" && r.Name == "status").Value.GetText());
        list.Clear();
        history.World.Draw(list, new DrawRequest(1UL << ScopeInfo.GridPath));
        Assert.True(list.Items.Length > 40 && list.Items.ToArray().All(i => i.Kind == DrawKind.Line));
    }

    [Theory]
    [InlineData("grid-astar.opgr", true)]
    [InlineData("grid-jps.opgr", true)]
    [InlineData("grid-astar2.opgr", false)]
    public void WhichEnginesShowTheirFrontier(string file, bool known)
    {
        var history = new History(new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath(file)), file));
        history.Seek(2);
        var rows = new PropertyList();
        history.World.Inspect(ScopeInfo.GridRoot, rows);
        Assert.Equal(known, rows.Rows.Any(r => r.Name == "open cells"));
    }

    [Fact]
    public void CellsAreInspectableAndPickable()
    {
        var history = new History(new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("grid-flow.opgr")), "f"));
        history.Seek(60);
        var list = new DrawList();
        history.World.Draw(list, new DrawRequest(1UL << ScopeInfo.GridCells));
        Assert.Equal(48 * 32, list.Items.Length);
        EntityId wall = list.Items[24].Entity, floor = list.Items[0].Entity;
        var rows = new PropertyList();
        Assert.True(history.World.Inspect(floor, rows));
        Assert.Equal(0, rows.Rows.Single(r => r.Name == "x").Value.GetInteger());
        Assert.Contains(rows.Rows, r => r.Group == "Flow" && r.Name == "cost");
        Assert.True(history.World.Inspect(wall, rows));
        Assert.False(rows.Rows.Single(r => r.Name == "walkable").Value.GetBool());
        Assert.False(history.World.Inspect(ScopeInfo.CellEntity(48 * 32), rows));
        Assert.False(history.World.Inspect(new EntityId(12345), rows));
        Assert.True(history.World.Inspect(ScopeInfo.GridRoot, rows));
        Assert.Equal("Found", rows.Rows.Single(r => r.Group == "Run" && r.Name == "status").Value.GetText());
    }

    [Theory]
    [MemberData(nameof(Files))]
    public void GridDrawIsStableAcrossPaths(string file, int slice)
    {
        _ = slice;
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath(file)), file);
        var a = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        var b = new History(source, new ScopeHistoryOptions { KeyframeInterval = 64 });
        var list = new DrawList();
        a.Seek(30);
        a.World.Draw(list, new DrawRequest(ulong.MaxValue));
        ulong first = DrawHash.Of(list);
        Assert.True(list.Items.Length > 1500);
        for (int i = 0; i < 30; i++)
        {
            b.Step();
        }

        list.Clear();
        b.World.Draw(list, new DrawRequest(ulong.MaxValue));
        Assert.Equal(first, DrawHash.Of(list));
        a.Seek(70 > a.EndStep ? a.EndStep : 70);
        a.Seek(30);
        list.Clear();
        a.World.Draw(list, new DrawRequest(ulong.MaxValue));
        Assert.Equal(first, DrawHash.Of(list));
        Assert.Equal(0, list.Dropped);
    }

    [Theory]
    [InlineData("grid-astar.opgr")]
    [InlineData("grid-flow.opgr")]
    public void ObstaclePokesFork(string file)
    {
        IScopeSource source = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath(file)), file);
        var original = new History(source, new ScopeHistoryOptions { KeyframeInterval = 7 });
        original.Seek(original.EndStep);
        ulong endHash = original.StateHash;
        byte[][] inputs = Enumerable.Range(0, (int)original.EndStep).Select(i => original.Recording.InputAt(i).ToArray()).ToArray();
        PokeCall[] calls =
        [
            new PokeCall("set_wall", [ScopeValue.OfVector(new Vector3(20.5f, 16.5f, 0)), ScopeValue.OfBool(true)]),
            new PokeCall("set_area", [ScopeValue.OfVector(new Vector3(10.2f, 10.9f, 0)), ScopeValue.OfInteger(255)]),
            new PokeCall("move_goal", [ScopeValue.OfEntity(ScopeInfo.GoalMarker), ScopeValue.OfVector(new Vector3(40, 10, 0))]),
        ];
        foreach (PokeCall call in calls)
        {
            Branch branch = original.Fork(5);
            branch.Poke(call);
            Assert.True(branch.Step());
            for (int i = 0; i < 10; i++)
            {
                branch.Step();
            }

            Assert.Equal(5, branch.DivergenceStep);
            original.Seek(original.EndStep);
            Assert.Equal(endHash, original.StateHash);
            for (int i = 0; i < inputs.Length; i++)
            {
                Assert.Equal(inputs[i], original.Recording.InputAt(i).ToArray());
            }

            var bytes = new ArrayBufferWriter<byte>();
            branch.History.Source.WriteReplay(bytes);
            var reopened = new History(new OnePathModule().Open(bytes.WrittenMemory, "fork"), new ScopeHistoryOptions { KeyframeInterval = 64 });
            for (long s = 0; s <= branch.History.EndStep; s += 3)
            {
                branch.History.Seek(s);
                reopened.Seek(s);
                Assert.Equal(branch.History.StateHash, reopened.StateHash);
            }

            Difference? d = original.Diff(branch.History, 6);
            Assert.NotNull(d);
            if (call.Poke == "set_wall")
            {
                Assert.Equal(ScopeInfo.CellEntity((16 * 48) + 20), d!.Value.Entity);
                Assert.Equal("area", d.Value.Field);
            }
            else if (call.Poke == "set_area")
            {
                Assert.Equal(ScopeInfo.CellEntity((10 * 48) + 10), d!.Value.Entity);
            }
            else
            {
                Assert.Equal(ScopeInfo.GoalMarker, d!.Value.Entity);
            }
        }
    }

    [Fact]
    public void BadPokesAreRefusedAndWriteNothing()
    {
        IScopeWorld w = new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("grid-flow.opgr")), "f").CreateWorld();
        var into = new ArrayBufferWriter<byte>();
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("set_wall", [ScopeValue.OfVector(new Vector3(-1, 2, 0)), ScopeValue.OfBool(true)]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("set_wall", [ScopeValue.OfVector(new Vector3(float.NaN, 2, 0)), ScopeValue.OfBool(true)]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("set_area", [ScopeValue.OfVector(new Vector3(1, 2, 0)), ScopeValue.OfInteger(300)]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("move_start", [ScopeValue.OfVector(new Vector3(1, 2, 0))]), into));
        Assert.Throws<ArgumentException>(() => w.EncodePoke(new PokeCall("move_goal", [ScopeValue.OfEntity(ScopeInfo.StartMarker), ScopeValue.OfVector(new Vector3(1, 2, 0))]), into));
        Assert.Equal(0, into.WrittenCount);
    }

    [Fact]
    public void GridRunsAreCheckedLikeTheOtherFormats()
    {
        byte[] file = File.ReadAllBytes(Worlds.DataPath("grid-flow.opgr"));
        var module = new OnePathModule();
        for (int n = 0; n < file.Length; n += n < 300 ? 1 : 97)
        {
            Assert.Throws<ScopeFormatException>(() => module.Open(file.AsMemory(0, n), "cut"));
        }

        Assert.Throws<ScopeFormatException>(() => module.Open(file.Concat(new byte[] { 0 }).ToArray(), "extra"));
        foreach ((int at, int value) in new[] { (8, 0), (8, 70000), (8, -1), (16, 0), (16, 999), (20, 0), (20, int.MaxValue), (20, 100000), (24 + (2 * 8), int.MaxValue), (24 + (2 * 8), -5) })
        {
            byte[] m = (byte[])file.Clone();
            BitConverter.GetBytes(value).CopyTo(m, at);
            Assert.Throws<ScopeFormatException>(() => module.Open(m, "x"));
        }

        byte[] kind = (byte[])file.Clone();
        kind[5] = 9;
        Assert.Throws<ScopeFormatException>(() => module.Open(kind, "kind"));
        // A cell outside the grid, and an edit outside the grid.
        byte[] goal = (byte[])file.Clone();
        BitConverter.GetBytes(9999).CopyTo(goal, 24);
        Assert.Throws<ScopeFormatException>(() => module.Open(goal, "goal"));
        var rng = new Random(5);
        int refused = 0;
        for (int round = 0; round < 400; round++)
        {
            byte[] m = (byte[])file.Clone();
            for (int e = 0; e < 1 + rng.Next(3); e++)
            {
                m[rng.Next(m.Length)] = (byte)rng.Next(256);
            }

            try
            {
                var h = new History(module.Open(m, "damaged"));
                h.Seek(Math.Min(h.EndStep, 30));
            }
            catch (ScopeFormatException)
            {
                refused++;
            }
        }

        Assert.True(refused > 0);

        // An edit that names a cell outside the grid is refused when the step runs.
        var w = module.Open(file, "f").CreateWorld();
        var bad = new ArrayBufferWriter<byte>();
        GridRun.Encode(GridRunOp.SetArea(500, 2, 0), bad);
        Assert.Throws<ScopeFormatException>(() => w.Advance(bad.WrittenSpan, TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => w.Advance(new byte[3], TimelineSink.None));
        Assert.Throws<ScopeFormatException>(() => w.Restore(new byte[30]));
    }

    [Fact]
    public void RunsRoundTripByteForByte()
    {
        foreach (string file in new[] { "grid-astar.opgr", "grid-jps.opgr", "grid-flow.opgr" })
        {
            byte[] bytes = File.ReadAllBytes(Worlds.DataPath(file));
            var w = new ArrayBufferWriter<byte>();
            new OnePathModule().Open(bytes, file).WriteReplay(w);
            Assert.Equal(bytes, w.WrittenSpan.ToArray());
        }
    }

    [Fact]
    public void TimelineTracksSearchProgress()
    {
        var history = new History(new OnePathModule().Open(File.ReadAllBytes(Worlds.DataPath("grid-astar.opgr")), "a"));
        history.Scan();
        Assert.Contains(history.Tracks.Single(t => t.Info.Id == "edits").Samples, s => s.Text!.Contains("goal moved"));
        TrackData expanded = history.Tracks.Single(t => t.Info.Id == "expanded");
        Assert.Equal(80, expanded.Samples.Count);
        Assert.Contains(history.Tracks.Single(t => t.Info.Id == "search").Samples, s => s.Text!.Contains("path found"));
    }
}
