using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace RecursiveAI.OnePath.Bar;

/// <summary>Runs a reference driver as a child process and turns its output into rows. Never looks inside the reference.</summary>
internal static class RefRunner
{
    public sealed record Result(List<Row> Rows, int ExitCode, string Stderr);

    private static string Dotnet(Paths p)
    {
        var env = Environment.GetEnvironmentVariable("ONEPATH_DOTNET");
        if (!string.IsNullOrEmpty(env))
        {
            return env;
        }

        var user = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        foreach (var c in new[] { Path.Combine(user, "dotnet10", "dotnet"), Path.Combine(p.Home, "dotnet", "dotnet") })
        {
            if (File.Exists(c))
            {
                return c;
            }
        }

        return "dotnet";
    }

    /// <summary>Resolves a driver name to the program and the arguments that precede the suite row's own arguments.</summary>
    public static (string Exe, string[] Prefix, string Version)? Resolve(string driver, Paths p)
    {
        string Ref(string name, string rel) => Path.Combine(p.Refs, name, rel);
        var refHarness = Path.Combine(p.Home, "bin", "refharness", "OnePath.RefHarness.dll");
        return driver switch
        {
            "recast" => (Path.Combine(p.Home, "bin", "recast-harness"), [], "recast-1.6.0"),
            "rvo2" => (Path.Combine(p.Home, "bin", "rvo-harness"), [], "rvo2-2.0.2"),
            "warthog" => (Ref("warthog-core-0.6.2", "build/warthog"), [], "warthog-0.6.2"),
            "jps" => (Ref("warthog-jps-0.6.2", "build/warthog-jps"), [], "warthog-jps-0.6.2"),
            "dotrecast" => (Dotnet(p), [refHarness, "dotrecast"], "dotrecast-2026.3.1"),
            "rvo2cs" => (Dotnet(p), [refHarness, "rvo2cs"], "rvo2cs-2.0.1"),
            _ => null,
        };
    }

    public static Result Run(SuiteRow row, Paths paths)
    {
        var resolved = Resolve(row.Driver, paths) ?? throw new InvalidOperationException($"unknown driver '{row.Driver}' in row {row.Id}");
        var (exe, prefix, version) = resolved;
        if (!File.Exists(exe) && Path.IsPathRooted(exe))
        {
            throw new FileNotFoundException($"driver for '{row.Driver}' not built: {exe} (run tools/refs/refs.sh all)");
        }

        var tokens = paths.Expand(row.Args).Split(' ', StringSplitOptions.RemoveEmptyEntries);
        var envVars = tokens.Where(t => t.StartsWith("env:", StringComparison.Ordinal)).Select(t => t[4..]).ToList();
        var args = prefix.Concat(tokens.Where(t => !t.StartsWith("env:", StringComparison.Ordinal))).ToList();
        string? cwd = null;
        if (row.Driver is "warthog" or "jps")
        {
            // Scenario files name their map relative to their own folder.
            var i = args.IndexOf("--scen");
            if (i >= 0 && i + 1 < args.Count)
            {
                cwd = Path.GetDirectoryName(args[i + 1]);
            }
        }

        var psi = new ProcessStartInfo
        {
            RedirectStandardOutput = true,
            RedirectStandardError = true,
            UseShellExecute = false,
            WorkingDirectory = cwd ?? Directory.GetCurrentDirectory(),
        };

        // /usr/bin/time gives the reference's peak resident memory: -v on Linux, -l on macOS.
        var timeFlag = OperatingSystem.IsLinux() ? "-v" : OperatingSystem.IsMacOS() ? "-l" : null;
        if (timeFlag is not null && File.Exists("/usr/bin/time"))
        {
            psi.FileName = "/usr/bin/time";
            psi.ArgumentList.Add(timeFlag);
            psi.ArgumentList.Add(exe);
        }
        else
        {
            psi.FileName = exe;
            timeFlag = null;
        }

        foreach (var a in args)
        {
            psi.ArgumentList.Add(a);
        }

        foreach (var kv in envVars)
        {
            var eq = kv.IndexOf('=', StringComparison.Ordinal);
            psi.Environment[kv[..eq]] = kv[(eq + 1)..];
        }

        var wall = Stopwatch.StartNew();
        using var proc = Process.Start(psi) ?? throw new InvalidOperationException("could not start " + exe);
        var stderrTask = proc.StandardError.ReadToEndAsync();
        var stdout = proc.StandardOutput.ReadToEnd();
        proc.WaitForExit();
        var stderr = stderrTask.Result;
        wall.Stop();

        var rows = row.Driver is "warthog" or "jps" ? ParseWarthog(stdout, row, version, wall.Elapsed.TotalMilliseconds) : ParseRows(stdout, row, version);
        var rss = timeFlag is null ? null : ParsePeakRss(stderr);
        if (rss is not null)
        {
            rows.RemoveAll(r => r.Metric == "peak_rss");
            rows.Add(new Row(row.Part, row.Id, version, "peak_rss", rss.Value.ToString("F3", CultureInfo.InvariantCulture), "MB"));
        }

        return new Result(rows, proc.ExitCode, stderr);
    }

    /// <summary>Parses the drivers' <c>ROW,part,scenario,side,metric,value,unit</c> lines; the scenario becomes the suite row id.</summary>
    private static List<Row> ParseRows(string stdout, SuiteRow row, string version)
    {
        var rows = new List<Row>();
        foreach (var line in stdout.Split('\n'))
        {
            if (!line.StartsWith("ROW,", StringComparison.Ordinal))
            {
                continue;
            }

            var f = line.TrimEnd('\r').Split(',', 7);
            if (f.Length < 7)
            {
                continue;
            }

            rows.Add(new Row(f[1], row.Id, f[3].Length > 0 ? f[3] : version, f[4], f[5], f[6]));
        }

        return rows;
    }

    /// <summary>
    /// Warthog prints one tab-separated line per scenario instance (header: id snapshot alg expanded generated reopen
    /// surplus heapops nanos plen pcost scost map). We sum them per scenario file.
    /// </summary>
    private static List<Row> ParseWarthog(string stdout, SuiteRow row, string version, double wallMs)
    {
        string[]? header = null;
        long n = 0;
        double nanos = 0, expanded = 0, generated = 0, pcost = 0;
        long mismatches = 0;
        foreach (var raw in stdout.Split('\n'))
        {
            var line = raw.TrimEnd('\r');
            if (line.StartsWith("id\t", StringComparison.Ordinal))
            {
                header = line.Split('\t');
                continue;
            }

            if (header is null || line.Length == 0 || !char.IsDigit(line[0]))
            {
                continue;
            }

            var f = line.Split('\t');
            double Col(string name)
            {
                var i = Array.IndexOf(header, name);
                return i >= 0 && i < f.Length && double.TryParse(f[i], NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? d : double.NaN;
            }

            n++;
            nanos += Col("nanos");
            expanded += Col("expanded");
            generated += Col("generated");
            var pc = Col("pcost");
            var sc = Col("scost");
            pcost += pc;
            if (!double.IsNaN(sc) && Math.Abs(pc - sc) > 1e-3 * Math.Max(1.0, Math.Abs(sc)))
            {
                mismatches++;
            }
        }

        string F(double d) => d.ToString("F6", CultureInfo.InvariantCulture);
        return
        [
            new Row(row.Part, row.Id, version, "instances", F(n), "count"),
            new Row(row.Part, row.Id, version, "search_time", F(nanos / 1e6), "ms"),
            new Row(row.Part, row.Id, version, "wall_time", F(wallMs), "ms"),
            new Row(row.Part, row.Id, version, "expanded", F(expanded), "count"),
            new Row(row.Part, row.Id, version, "generated", F(generated), "count"),
            new Row(row.Part, row.Id, version, "cost_sum", F(pcost), "cost"),
            new Row(row.Part, row.Id, version, "cost_mismatch_vs_scenario", F(mismatches), "count"),
        ];
    }

    internal static double? ParsePeakRss(string stderr)
    {
        foreach (var raw in stderr.Split('\n'))
        {
            var line = raw.Trim();
            if (line.StartsWith("Maximum resident set size (kbytes):", StringComparison.Ordinal))
            {
                return double.Parse(line[(line.LastIndexOf(':') + 1)..].Trim(), CultureInfo.InvariantCulture) / 1024.0;
            }

            if (line.EndsWith("maximum resident set size", StringComparison.Ordinal))
            {
                var sb = new StringBuilder();
                foreach (var ch in line)
                {
                    if (char.IsDigit(ch))
                    {
                        sb.Append(ch);
                    }
                    else
                    {
                        break;
                    }
                }

                return double.Parse(sb.ToString(), CultureInfo.InvariantCulture) / (1024.0 * 1024.0); // bytes on macOS
            }
        }

        return null;
    }
}
