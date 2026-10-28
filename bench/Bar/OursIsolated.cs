using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Runs our side of a row in a fresh child process under <c>/usr/bin/time</c>, like every reference driver. Two things follow:
/// the peak resident memory of a row is that row's own (an in-process run reports the peak of everything the runner did before),
/// and our timings come from a cold process with its own warm-up, the same footing as a reference's.
/// The child (<c>bar ours-row --suite f --id row</c>) prints one tab-separated line per metric.
/// </summary>
internal static class OursIsolated
{
    private const string Tag = "OURROW";

    /// <summary>The child's entry point: runs the row and prints its metrics.</summary>
    public static int Child(string suitePath, string id, Paths paths)
    {
        var row = Suite.Load(suitePath).FirstOrDefault(r => r.Id == id);
        if (row is null)
        {
            Console.Error.WriteLine("no row " + id);
            return 2;
        }

        var rows = OurScenarios.Run(row, paths);
        if (rows is null)
        {
            return 3; // nothing registered for this row
        }

        foreach (var r in rows)
        {
            Console.WriteLine(string.Join('\t', Tag, r.Part, r.Scenario, r.Side, r.Metric, r.Value, r.Unit));
        }

        return 0;
    }

    /// <summary>Runs a row in a child; null when we have nothing for it (as <see cref="OurScenarios.Run"/>).</summary>
    public static List<Row>? Run(SuiteRow row, string suitePath, Paths paths)
    {
        var psi = new ProcessStartInfo { RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false };
        var timeFlag = OperatingSystem.IsLinux() ? "-v" : OperatingSystem.IsMacOS() ? "-l" : null;
        var args = new List<string>();
        if (timeFlag is not null && File.Exists("/usr/bin/time"))
        {
            psi.FileName = "/usr/bin/time";
            args.Add(timeFlag);
            args.Add(Environment.ProcessPath ?? "dotnet");
        }
        else
        {
            psi.FileName = Environment.ProcessPath ?? "dotnet";
            timeFlag = null;
        }

        args.Add(typeof(OursIsolated).Assembly.Location);
        args.AddRange(["ours-row", "--suite", Path.GetFullPath(suitePath), "--id", row.Id, "--home", paths.Home]);
        if (paths.Light)
        {
            args.Add("--light");
        }

        foreach (var a in args)
        {
            psi.ArgumentList.Add(a);
        }

        using var proc = Process.Start(psi) ?? throw new InvalidOperationException("could not start our row process");
        var stderrTask = proc.StandardError.ReadToEndAsync();
        var stdout = proc.StandardOutput.ReadToEnd();
        proc.WaitForExit();
        var stderr = stderrTask.Result;
        if (proc.ExitCode == 3)
        {
            return null;
        }

        if (proc.ExitCode != 0)
        {
            throw new InvalidOperationException($"our row {row.Id} exited {proc.ExitCode}: {stderr.Split('\n').FirstOrDefault(l => l.Length > 0)}");
        }

        var rows = new List<Row>();
        foreach (var line in stdout.Split('\n'))
        {
            var f = line.TrimEnd('\r').Split('\t');
            if (f.Length == 7 && f[0] == Tag)
            {
                rows.Add(new Row(f[1], f[2], f[3], f[4], f[5], f[6]));
            }
        }

        var rss = timeFlag is null ? null : RefRunner.ParsePeakRss(stderr);
        if (rss is not null)
        {
            var side = rows.FirstOrDefault()?.Side ?? OurScenarios.Side;
            rows.RemoveAll(r => r.Metric == "peak_rss");
            rows.Add(new Row(row.Part, row.Id, side, "peak_rss", rss.Value.ToString("F3", CultureInfo.InvariantCulture), "MB"));
        }

        return rows;
    }
}
