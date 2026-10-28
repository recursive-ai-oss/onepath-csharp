using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using RecursiveAI.OnePath.Bar;

// Bar runner. Runs each row of a suite on our side and on the reference side, best of N, and writes
//   <out>/rows.csv   every measurement, long format
//   <out>/env.json   machine, CPU, cores, load before/during/after, SDK, runtime, reference pins
//   <out>/report.md  one line per row: ours vs reference, ratio, memory, verdict
// Usage: bar run --suite <file.tsv> [--ours-only 1] [--in-process 1] [--light 1] [--out <dir>] [--reps N] [--only <substring>] [--home <dir>] [--machine <name>]
//        bar env
// Heavy runs take the worker lock outside this program: tools/bench/locked-run.sh.
return Cli.Execute(args);

internal static class Cli
{
    public static int Execute(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            Console.Error.WriteLine("usage: bar run --suite <file.tsv> [--out <dir>] [--reps N] [--only <substring>] [--home <dir>] [--machine <name>] | bar env");
            return args.Length == 0 ? 2 : 0;
        }

        var opts = new Dictionary<string, string>();
        for (var i = 1; i < args.Length; i++)
        {
            if (args[i].StartsWith("--", StringComparison.Ordinal))
            {
                // A flag with no value (last argument, or followed by another option) is switched on with "1".
                bool hasValue = i + 1 < args.Length && !args[i + 1].StartsWith("--", StringComparison.Ordinal);
                opts[args[i][2..]] = hasValue ? args[++i] : "1";
            }
        }

        switch (args[0])
        {
            case "env":
                Env.Write("/dev/stdout", "env", "-", 0, [], Env.Load1(), null, opts.GetValueOrDefault("machine"), Directory.GetCurrentDirectory());
                return 0;
            case "run":
                return Run(opts);
            case "ours-row":
                return OursIsolated.Child(opts.GetValueOrDefault("suite") ?? "", opts.GetValueOrDefault("id") ?? "", new Paths(opts.GetValueOrDefault("home"), opts.ContainsKey("light")));
            default:
                Console.Error.WriteLine("unknown command " + args[0]);
                return 2;
        }
    }

    private static int Run(Dictionary<string, string> opts)
    {
        if (!opts.TryGetValue("suite", out var suitePath))
        {
            Console.Error.WriteLine("--suite is required");
            return 2;
        }

        var reps = opts.TryGetValue("reps", out var r) ? int.Parse(r, CultureInfo.InvariantCulture) : 3;
        var runId = DateTime.UtcNow.ToString("yyyyMMdd'T'HHmmss'Z'", CultureInfo.InvariantCulture);
        var outDir = opts.GetValueOrDefault("out") ?? Path.Combine(Directory.GetCurrentDirectory(), "bar-" + runId);
        Directory.CreateDirectory(outDir);
        var paths = new Paths(opts.GetValueOrDefault("home"), opts.ContainsKey("light"));
        var rows = Suite.Load(suitePath).Where(x => !opts.TryGetValue("only", out var only) || x.Id.Contains(only, StringComparison.Ordinal)).ToList();

        var loadBefore = Env.Load1();
        var during = new List<double?>();
        var results = new List<(SuiteRow Row, List<Row> Ours, List<Row> Ref)>();
        var failures = 0;
        var oursOnly = opts.ContainsKey("ours-only");
        foreach (var row in rows)
        {
            Console.Error.WriteLine($"[{row.Id}] {row.Driver} x{reps}");
            var refRuns = new List<List<Row>>();
            var ourRuns = new List<List<Row>>();
            for (var i = 0; i < reps; i++)
            {
                try
                {
                    if (oursOnly)
                    {
                        refRuns.Add([]);
                        throw new InvalidOperationException("ours-only");
                    }

                    var res = RefRunner.Run(row, paths);
                    // Warthog's --checkopt exits with the number of instances whose cost differs from the scenario's, after printing every
                    // instance: the rows are valid, the code is reported beside them (reference_exit_code) and cost_mismatch_vs_scenario is ours to read.
                    bool checkOptExit = row.Driver is "warthog" or "jps" && res.Rows.Count > 0;
                    if (res.ExitCode == 0 || checkOptExit)
                    {
                        if (res.ExitCode != 0)
                        {
                            res.Rows.Add(new Row(row.Part, row.Id, res.Rows[0].Side, "reference_exit_code", res.ExitCode.ToString(CultureInfo.InvariantCulture), "count"));
                        }

                        refRuns.Add(res.Rows);
                    }
                    else
                    {
                        Console.Error.WriteLine($"  reference exited {res.ExitCode}: {res.Stderr.Split('\n').FirstOrDefault(l => l.Length > 0)}");
                    }
                }
                catch (Exception e) when (e is IOException or InvalidOperationException)
                {
                    if (!oursOnly)
                    {
                        Console.Error.WriteLine("  reference failed: " + e.Message);
                        break;
                    }
                }

                during.Add(Env.Load1());
                // A fresh process per run, so the row's peak memory is its own; --in-process runs it here (debugging).
                List<Row>? ours = null;
                try
                {
                    ours = opts.ContainsKey("in-process") ? OurScenarios.Run(row, paths) : OursIsolated.Run(row, suitePath, paths);
                }
                catch (InvalidOperationException e)
                {
                    Console.Error.WriteLine("  ours failed: " + e.Message);
                    failures++;
                }

                if (ours is not null)
                {
                    ourRuns.Add(ours);
                }
            }

            if (refRuns.Count == 0 && !oursOnly)
            {
                failures++;
            }

            results.Add((row, Aggregate.BestOf(ourRuns), Aggregate.BestOf(refRuns)));
        }

        var loadAfter = Env.Load1();
        var suiteName = Path.GetFileNameWithoutExtension(suitePath);
        Env.Write(Path.Combine(outDir, "env.json"), runId, suiteName, reps, during, loadBefore, loadAfter, opts.GetValueOrDefault("machine"), Path.GetDirectoryName(Path.GetFullPath(suitePath)) ?? ".");
        Report.WriteCsv(Path.Combine(outDir, "rows.csv"), runId, suiteName, results);
        Report.WriteMarkdown(Path.Combine(outDir, "report.md"), runId, suiteName, "env.json", results);
        Console.WriteLine(outDir);
        return failures == 0 ? 0 : 1;
    }
}
