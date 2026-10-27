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
// Usage: bar run --suite <file.tsv> [--ours-only 1] [--out <dir>] [--reps N] [--only <substring>] [--home <dir>] [--machine <name>]
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
            if (args[i].StartsWith("--", StringComparison.Ordinal) && i + 1 < args.Length)
            {
                opts[args[i][2..]] = args[++i];
            }
        }

        switch (args[0])
        {
            case "env":
                Env.Write("/dev/stdout", "env", "-", 0, [], Env.Load1(), null, opts.GetValueOrDefault("machine"), Directory.GetCurrentDirectory());
                return 0;
            case "run":
                return Run(opts);
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
        var paths = new Paths(opts.GetValueOrDefault("home"));
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
                    if (res.ExitCode == 0)
                    {
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
                var ours = OurScenarios.Run(row, paths);
                if (ours is not null)
                {
                    ourRuns.Add(ours);
                }
            }

            if (refRuns.Count == 0)
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
