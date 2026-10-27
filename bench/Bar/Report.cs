using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;

namespace RecursiveAI.OnePath.Bar;

internal static class Report
{
    private static string Csv(string s) => s.IndexOfAny([',', '"', '\n']) >= 0 ? "\"" + s.Replace("\"", "\"\"", StringComparison.Ordinal) + "\"" : s;

    public static void WriteCsv(string path, string runId, string suite, IEnumerable<(SuiteRow Row, List<Row> Ours, List<Row> Ref)> results)
    {
        var sb = new StringBuilder("run_id,suite,row_id,part,scenario,side,metric,value,unit\n");
        foreach (var (row, ours, refs) in results)
        {
            foreach (var r in ours.Concat(refs))
            {
                sb.Append(Csv(runId)).Append(',').Append(Csv(suite)).Append(',').Append(Csv(row.Id)).Append(',').Append(Csv(r.Part)).Append(',')
                  .Append(Csv(r.Scenario)).Append(',').Append(Csv(r.Side)).Append(',').Append(Csv(r.Metric)).Append(',').Append(Csv(r.Value)).Append(',')
                  .Append(Csv(r.Unit)).Append('\n');
            }
        }

        File.WriteAllText(path, sb.ToString());
    }

    private static double? Get(List<Row> rows, string metric) => rows.FirstOrDefault(r => r.Metric == metric)?.Number;

    private static string Fmt(double? d) => d is null ? "n/a" : d.Value.ToString("G5", CultureInfo.InvariantCulture);

    /// <summary>
    /// The markdown summary: per row the compared metric for both sides, the ratio (ours / reference for times,
    /// reference / ours for throughputs, so lower is better and the bar is the row's tolerance), peak memory, verdict.
    /// </summary>
    public static void WriteMarkdown(string path, string runId, string suite, string envJsonName, IEnumerable<(SuiteRow Row, List<Row> Ours, List<Row> Ref)> results)
    {
        var sb = new StringBuilder();
        sb.Append("# Bar run ").Append(runId).Append(" (").Append(suite).Append(")\n\n");
        sb.Append("Environment, load before/during/after and reference versions: `").Append(envJsonName).Append("`.\n\n");
        sb.Append("Ratio = ours / reference for times, reference / ours for throughputs; a row passes at or below its tolerance.\n\n");
        sb.Append("| row | part | metric | ours | reference | ratio | tolerance | ours MB | reference MB | verdict |\n");
        sb.Append("|---|---|---|---:|---:|---:|---:|---:|---:|---|\n");
        foreach (var (row, ours, refs) in results)
        {
            var o = Get(ours, row.Metric);
            var r = Get(refs, row.Metric);
            double? ratio = o is null || r is null || o == 0 || r == 0 ? null : row.HigherIsBetter ? r / o : o / r;
            var verdict = r is null ? "reference failed" : o is null ? "no ours yet" : ratio <= row.Tolerance ? "pass" : "FAIL";
            sb.Append("| ").Append(row.Id).Append(" | ").Append(row.Part).Append(" | ").Append(row.Metric).Append(row.HigherIsBetter ? " (higher better)" : " (lower better)")
              .Append(" | ").Append(Fmt(o)).Append(" | ").Append(Fmt(r)).Append(" | ").Append(Fmt(ratio)).Append(" | ").Append(row.Tolerance.ToString("G3", CultureInfo.InvariantCulture))
              .Append(" | ").Append(Fmt(Get(ours, "peak_rss"))).Append(" | ").Append(Fmt(Get(refs, "peak_rss"))).Append(" | ").Append(verdict).Append(" |\n");
        }

        sb.Append("\n## Reference quality and details\n\n| row | side | metric | value | unit |\n|---|---|---|---:|---|\n");
        foreach (var (row, ours, refs) in results)
        {
            foreach (var m in ours.Concat(refs).Where(x => x.Metric is not ("peak_rss" or "time_median")))
            {
                sb.Append("| ").Append(row.Id).Append(" | ").Append(m.Side).Append(" | ").Append(m.Metric).Append(" | ").Append(m.Number is { } n ? n.ToString("G6", CultureInfo.InvariantCulture) : m.Value)
                  .Append(" | ").Append(m.Unit).Append(" |\n");
            }
        }

        File.WriteAllText(path, sb.ToString());
    }
}
