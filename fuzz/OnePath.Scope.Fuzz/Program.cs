using System;
using System.Buffers;
using System.Collections.Generic;
using System.IO;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Replay;
using SharpFuzz;

namespace RecursiveAI.OnePath.Scope.Fuzz;

/// <summary>
/// libFuzzer entry point for the module's parsers: <c>OnePath.Scope.Fuzz &lt;target&gt;</c>. A target must either succeed or throw
/// <see cref="ScopeFormatException"/> (the typed error for bad bytes); any other exception is a crash. A file path as the second
/// argument replays that input once.
/// </summary>
public static class Program
{
    // Built on first use, not in a type initializer: the coverage instrumentation is not ready inside one.
    private static OnePathModule? s_module;
    private static byte[]? s_snapshot;

    private static OnePathModule Module => s_module ??= new OnePathModule();

    private static readonly SortedDictionary<string, Action<byte[]>> Targets = new(StringComparer.Ordinal)
    {
        ["scope-grid-run"] = data => Explore(data, "iteration"),
        ["scope-build-input"] = data => Explore(data, "stage"),
        ["scope-open"] = data => Explore(data, null),
        ["scope-live"] = Live,
    };

    public static int Main(string[] args)
    {
        if (args.Length == 2 && args[0] == "--seeds")
        {
            WriteSeeds(args[1]);
            return 0;
        }

        if (args.Length == 0 || !Targets.TryGetValue(args[0], out Action<byte[]>? target))
        {
            Console.Error.WriteLine("usage: OnePath.Scope.Fuzz <target> [input file]");
            Console.Error.WriteLine("targets: " + string.Join(", ", Targets.Keys));
            return 2;
        }

        if (args.Length > 1 && File.Exists(args[1]))
        {
            target(File.ReadAllBytes(args[1]));
            return 0;
        }

        Fuzzer.LibFuzzer.Run(span => target(span.ToArray()));
        return 0;
    }

    // Opens the bytes as any file the module reads, then drives what a viewer does: seek, the extra axis, draw, inspect, diff, save.
    private static void Explore(byte[] data, string? axis)
    {
        try
        {
            IScopeSource source = Module.Open(data, "fuzz");
            var history = new History(source, new ScopeHistoryOptions { KeyframeInterval = 5 });
            history.Seek(Math.Min(history.EndStep, 12));
            if (axis is not null)
            {
                AxisInfo? a = null;
                foreach (AxisInfo candidate in source.Info.Axes)
                {
                    if (candidate.Id == axis)
                    {
                        a = candidate;
                    }
                }

                if (a is { } info && history.EndStep > 0)
                {
                    history.Seek(history.EndStep - 1);
                    history.Seek(axis, info.PositionCount);
                }
            }

            var list = new DrawList();
            history.View.Draw(list, new DrawRequest(ulong.MaxValue));
            var entities = new List<EntityInfo>();
            history.View.CollectEntities(entities);
            var props = new PropertyList();
            for (int i = 0; i < Math.Min(entities.Count, 8); i++)
            {
                history.View.Inspect(entities[i].Id, props);
            }

            history.Seek(history.EndStep);
            var saved = new ArrayBufferWriter<byte>();
            source.WriteReplay(saved);
            IScopeSource again = Module.Open(saved.WrittenMemory, "fuzz-again");
            var h2 = new History(again);
            h2.Seek(h2.EndStep);
            if (h2.StateHash != history.StateHash)
            {
                throw new InvalidOperationException("a replay written from an opened source did not reproduce its state");
            }
        }
        catch (ScopeFormatException)
        {
        }
    }

    // A live attach: the input is a snapshot (first byte even), or a valid snapshot followed by records cut from the input (first byte odd).
    private static void Live(byte[] data)
    {
        if (data.Length < 2)
        {
            return;
        }

        try
        {
            if ((data[0] & 1) == 0)
            {
                Module.OpenLive("1.2.0", ScopeInfo.FormatVersion, data.AsMemory(1), "fuzz");
                return;
            }

            s_snapshot ??= MakeSnapshot();
            ILiveSource live = Module.OpenLive("1.2.0", ScopeInfo.FormatVersion, s_snapshot, "fuzz");
            int at = 1;
            while (at < data.Length)
            {
                int len = Math.Min(data[at], data.Length - at - 1);
                live.AppendRecord(data.AsSpan(at + 1, len));
                at += 1 + len;
            }

            var history = new History(live.Source);
            history.Seek(history.EndStep);
        }
        catch (ScopeFormatException)
        {
        }
    }

    private static byte[] MakeSnapshot()
    {
        var rec = new CrowdRecorder(new Crowd(CrowdWorlds.Arena(14, 10), new CrowdOptions { MaxAgents = 64 }), CrowdWorlds.Dt);
        for (int i = 0; i < 3; i++)
        {
            Agent a = rec.Add(new Vector3D(2 + i, 0, 3), Agent.Small);
            rec.SetDestination(a, new Vector3D(11, 0, 4));
        }

        rec.Step();
        var w = new ArrayBufferWriter<byte>();
        rec.WriteSnapshot(w);
        return w.WrittenSpan.ToArray();
    }

    // Valid inputs for the corpus: the committed test files, a live snapshot and live records.
    private static void WriteSeeds(string dir)
    {
        string data = Path.Combine(AppContext.BaseDirectory, "testdata");
        foreach ((string target, string[] files) in new[] { ("scope-grid-run", new[] { "grid-astar.opgr", "grid-jps.opgr", "grid-flow.opgr" }), ("scope-build-input", ["terrain.opbi"]), ("scope-open", ["crowd2d.oprp", "grid-flow.opgr", "terrain.opbi"]) })
        {
            Directory.CreateDirectory(Path.Combine(dir, target));
            foreach (string f in files)
            {
                File.Copy(Path.Combine(data, f), Path.Combine(dir, target, f), true);
            }
        }

        Directory.CreateDirectory(Path.Combine(dir, "scope-live"));
        byte[] snap = MakeSnapshot();
        File.WriteAllBytes(Path.Combine(dir, "scope-live", "snapshot"), [.. new byte[] { 0 }, .. snap]);
        var rec = new CrowdRecorder(new Crowd(CrowdWorlds.Arena(14, 10)), CrowdWorlds.Dt);
        var records = new List<byte>();
        rec.Record = r =>
        {
            records.Add((byte)r.Length);
            records.AddRange(r.ToArray());
        };
        Agent agent = rec.Add(new Vector3D(2, 0, 3), Agent.Small);
        rec.SetDestination(agent, new Vector3D(11, 0, 4));
        rec.Step();
        rec.Step();
        File.WriteAllBytes(Path.Combine(dir, "scope-live", "records"), [.. new byte[] { 1 }, .. records]);
    }
}
