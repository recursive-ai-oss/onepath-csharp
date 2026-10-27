using System;
using RecursiveAI.OnePath;
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
    };
}
