using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Mesh;
using SharpFuzz;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// libFuzzer entry point. Usage: <c>OnePath.Fuzz &lt;target&gt; [args]</c>. Targets register in <see cref="Targets"/>.
/// The library assembly is instrumented with the SharpFuzz tool before a campaign.
/// </summary>
public static class Program
{
    public static int Main(string[] args)
    {
        // AFL mode (SharpFuzz out-of-process): OnePath.Fuzz --afl <target>. Used where clang and libFuzzer are not available.
        if (args.Length == 2 && args[0] == "--afl" && Targets.All.TryGetValue(args[1], out var aflTarget))
        {
            string? lastInput = Environment.GetEnvironmentVariable("ONEPATH_FUZZ_LAST_INPUT");
            if (lastInput != null)
            {
                // AFL discards the child's output; keep what killed it next to the input.
                AppDomain.CurrentDomain.UnhandledException += (_, e) => System.IO.File.WriteAllText(lastInput + ".error", e.ExceptionObject.ToString());
            }
            Fuzzer.Run(stream =>
            {
                using var ms = new System.IO.MemoryStream();
                stream.CopyTo(ms);
                byte[] input = ms.ToArray();
                // The process is its own fork server, so a crash that cannot be caught (stack overflow, out of memory)
                // loses the input; with this variable set the current input is kept on disk to replay.
                string? last = Environment.GetEnvironmentVariable("ONEPATH_FUZZ_LAST_INPUT");
                if (last != null)
                {
                    System.IO.File.WriteAllBytes(last, input);
                }
                aflTarget(input);
            });
            return 0;
        }

        // Writes valid seed inputs for the deserializer targets: OnePath.Fuzz --seeds <dir>.
        if (args.Length == 2 && args[0] == "--seeds")
        {
            NavMeshFuzz.WriteSeeds(args[1]);
            NavMeshSnapshotFuzz.WriteSeeds(System.IO.Path.Combine(args[1], "snapshot"));
            return 0;
        }

        if (args.Length == 0 || !Targets.All.TryGetValue(args[0], out var target))
        {
            Console.Error.WriteLine("usage: OnePath.Fuzz <target> [libFuzzer args]");
            Console.Error.WriteLine("targets: " + string.Join(", ", Targets.All.Keys));
            return 2;
        }

        // Input files given on the command line are replayed once (for reproducing a crash).
        if (args.Length > 1 && System.IO.File.Exists(args[1]))
        {
            target(System.IO.File.ReadAllBytes(args[1]));
            return 0;
        }

        // AFL mode (no native libFuzzer host needed): the input arrives on standard input.
        if (args.Length > 1 && args[1] == "--afl")
        {
            Fuzzer.Run(stream =>
            {
                using var ms = new System.IO.MemoryStream();
                stream.CopyTo(ms);
                target(ms.ToArray());
            });
            return 0;
        }

        Fuzzer.LibFuzzer.Run(span => target(span.ToArray()));
        return 0;
    }
}

/// <summary>The registered fuzz targets, by name.</summary>
public static class Targets
{
    /// <summary>All targets. Later work adds builders, deserializers and query entry points here.</summary>
    public static readonly System.Collections.Generic.SortedDictionary<string, Action<byte[]>> All = new(StringComparer.Ordinal)
    {
        // Placeholder target: proves the scaffold builds and runs.
        ["info"] = _ => GC.KeepAlive(OnePathInfo.Version),
        ["build3d"] = Build3DTarget.Run,
        ["build3d-contours"] = Build3DTarget.RunContours,
        ["build2d"] = Build2DFuzz.Run,
        ["build2d-smoke"] = Build2DFuzz.Smoke,
        ["build2d-tiles"] = Build2DFuzz.RunTiles,
        ["build2d-tiles-smoke"] = Build2DFuzz.SmokeTiles,
        ["build2d-describe"] = Build2DFuzz.Describe,
        ["navmesh-mesh"] = NavMeshFuzz.Mesh,
        ["navmesh-tile"] = NavMeshFuzz.Tile,
        ["movingai-map"] = GridTargets.MapParser,
        ["movingai-scen"] = GridTargets.ScenarioParser,
        ["grid-query"] = GridTargets.Query,
        ["navmesh-query"] = NavMeshQueryFuzz.Run,
        ["grid-jps"] = GridTargets.Jps,
        ["navmesh-snapshot"] = NavMeshSnapshotFuzz.Run,
        ["grid-theta"] = ThetaTarget.Run,
        ["grid-flow"] = FlowTargets.Run,
    };
}
