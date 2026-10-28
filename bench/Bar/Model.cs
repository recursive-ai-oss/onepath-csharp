using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text.RegularExpressions;

namespace RecursiveAI.OnePath.Bar;

/// <summary>One measurement: the bar's long row format (see docs/benchmarking.md).</summary>
internal sealed record Row(string Part, string Scenario, string Side, string Metric, string Value, string Unit)
{
    public double? Number => double.TryParse(Value, NumberStyles.Float, CultureInfo.InvariantCulture, out var d) ? d : null;
}

/// <summary>One line of a suite file: a scenario, the reference driver that runs it and how ours is compared.</summary>
internal sealed record SuiteRow(string Id, string Part, string Driver, string Metric, bool HigherIsBetter, double Tolerance, string Args);

internal static class Suite
{
    /// <summary>
    /// Reads a suite file: tab-separated <c>id part driver metric better tolerance args</c>; <c>#</c> comments and blank
    /// lines are skipped. <c>better</c> is <c>lower</c> or <c>higher</c>; <c>tolerance</c> is the allowed ratio (1.15).
    /// </summary>
    public static List<SuiteRow> Load(string path)
    {
        var rows = new List<SuiteRow>();
        int lineNo = 0;
        foreach (var raw in File.ReadLines(path))
        {
            lineNo++;
            var line = raw.TrimEnd();
            if (line.Length == 0 || line[0] == '#')
            {
                continue;
            }

            var f = line.Split('\t');
            if (f.Length < 7)
            {
                throw new FormatException($"{path}:{lineNo}: expected 7 tab-separated fields (id part driver metric better tolerance args), got {f.Length}");
            }

            rows.Add(new SuiteRow(f[0], f[1], f[2], f[3], f[4] == "higher", double.Parse(f[5], CultureInfo.InvariantCulture), f[6]));
        }

        return rows;
    }
}

/// <summary>Where the references, maps and generated geometry live (the same folder tools/refs/refs.sh fills).</summary>
internal sealed class Paths
{
    public string Home { get; }

    public string Maps => Path.Combine(Home, "maps");

    /// <summary>Light-check mode: small levels, one repetition and short crowd runs. Proves a row runs; its numbers are not publishable.</summary>
    public bool Light { get; }

    public string Geom => Path.Combine(Home, "geom", Light ? "96" : "256");

    public string GeomSmall => Path.Combine(Home, "geom", "96");

    public Paths(string? home, bool light = false)
    {
        Home = home ?? Environment.GetEnvironmentVariable("ONEPATH_HOME") ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "onepath");
        Light = light;
    }

    public string Expand(string s)
    {
        s = s.Replace("{home}", Home, StringComparison.Ordinal).Replace("{maps}", Maps, StringComparison.Ordinal).Replace("{geomsmall}", GeomSmall, StringComparison.Ordinal).Replace("{geom}", Geom, StringComparison.Ordinal);
        if (Light)
        {
            s = Regex.Replace(s, @"--reps \d+", "--reps 1");
            s = Regex.Replace(s, @"--steps \d+", "--steps 20");
            s = Regex.Replace(s, @"--warmup \d+", "--warmup 5");
            s = Regex.Replace(s, @"--agents (\d+)", m => "--agents " + Math.Min(int.Parse(m.Groups[1].Value, CultureInfo.InvariantCulture), 200).ToString(CultureInfo.InvariantCulture));
        }

        return s;
    }

    public string Refs => Path.Combine(Home, "refs");
}

internal static class Aggregate
{
    /// <summary>
    /// Merges the rows of several runs of one scenario into best-of-N: times keep the minimum, throughputs and memory
    /// the maximum, everything else the last run's value.
    /// </summary>
    public static List<Row> BestOf(IEnumerable<List<Row>> runs)
    {
        var merged = new Dictionary<(string, string, string, string), Row>();
        var order = new List<(string, string, string, string)>();
        foreach (var run in runs)
        {
            foreach (var r in run)
            {
                var key = (r.Part, r.Scenario, r.Side, r.Metric);
                if (!merged.TryGetValue(key, out var old))
                {
                    merged[key] = r;
                    order.Add(key);
                    continue;
                }

                var a = old.Number;
                var b = r.Number;
                if (a is null || b is null)
                {
                    merged[key] = r;
                }
                else if (r.Unit == "ms")
                {
                    merged[key] = b < a ? r : old;
                }
                else if (r.Unit.Contains("/s", StringComparison.Ordinal) || r.Unit.Contains("/ms", StringComparison.Ordinal) || r.Unit == "MB")
                {
                    merged[key] = b > a ? r : old;
                }
                else
                {
                    merged[key] = r;
                }
            }
        }

        return order.Select(k => merged[k]).ToList();
    }
}
