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

        WriteGate2D(sb, results);
        File.WriteAllText(path, sb.ToString());
    }

    /// <summary>
    /// The 2D builder gate: our walkable coverage against the exact eroded area of the same tile map (must be within 1%), then the
    /// reference's coverage and our delta against it as information. Recast keeps a ledge-and-region margin of one cell on top of
    /// the agent radius, so its coverage equals the exact area at radius + one cell and sits below the exact area by construction.
    /// </summary>
    private static void WriteGate2D(StringBuilder sb, IEnumerable<(SuiteRow Row, List<Row> Ours, List<Row> Ref)> results)
    {
        var rows = results.Where(r => Get(r.Ours, "exact_area") is not null).ToList();
        if (rows.Count == 0)
        {
            return;
        }

        sb.Append("\n## 2D gate: coverage against the exact eroded area\n\n");
        sb.Append("The gate is ours within 1% of the exact area (computed without any builder). The reference columns are information only: ")
          .Append("Recast's coverage equals the exact area at radius + one cell, so it falls below the exact area by construction.\n\n");
        sb.Append("| row | exact area | ours | ours vs exact | gate (1%) | reference | reference coverage | reference vs exact | ours vs reference |\n");
        sb.Append("|---|---:|---:|---:|---|---|---:|---:|---:|\n");
        foreach (var (row, ours, refs) in rows)
        {
            double exact = Get(ours, "exact_area")!.Value;
            double? o = Get(ours, "coverage_area");
            double? r = Get(refs, "coverage_area");
            double? dOurs = o is null ? null : exact > 0 ? (o - exact) * 100 / exact : (o == 0 ? 0 : double.PositiveInfinity);
            double? dRef = r is null ? null : exact > 0 ? (r - exact) * 100 / exact : (r == 0 ? 0 : double.PositiveInfinity);
            double? dOursRef = o is null || r is null || r == 0 ? null : (o - r) * 100 / r;
            // Within 1% of the exact area; where the exact area is a few cells (a sliver left at a large radius) 1% is below the
            // rounding of one polygon vertex, so an absolute difference of up to 0.02 world units^2 also passes.
            string gate = dOurs is null ? "no ours" : Math.Abs(dOurs.Value) <= 1.0 ? "pass" : Math.Abs(o!.Value - exact) <= GateAbsolute ? "pass (absolute)" : "FAIL";
            string refName = refs.FirstOrDefault()?.Side ?? "-";
            sb.Append("| ").Append(row.Id).Append(" | ").Append(Fmt(exact)).Append(" | ").Append(Fmt(o)).Append(" | ").Append(Pct(dOurs)).Append(" | ").Append(gate)
              .Append(" | ").Append(refName).Append(" | ").Append(Fmt(r)).Append(" | ").Append(Pct(dRef)).Append(" | ").Append(Pct(dOursRef)).Append(" |\n");
        }
    }

    private const double GateAbsolute = 0.02;

    private static string Pct(double? d) => d is null ? "n/a" : d.Value.ToString("+0.00;-0.00;0.00", CultureInfo.InvariantCulture) + "%";
}
