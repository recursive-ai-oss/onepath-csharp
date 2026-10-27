using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Runtime.InteropServices;
using System.Text.Json;

namespace RecursiveAI.OnePath.Bar;

/// <summary>The machine and software a run happened on, written next to every result so a row can be trusted or redone.</summary>
internal static class Env
{
    public static double? Load1()
    {
        try
        {
            if (File.Exists("/proc/loadavg"))
            {
                return double.Parse(File.ReadAllText("/proc/loadavg").Split(' ')[0], CultureInfo.InvariantCulture);
            }

            var o = Exec("sysctl", "-n vm.loadavg"); // "{ 1.20 1.50 1.60 }"
            var t = o?.Trim('{', '}', ' ', '\n').Split(' ', StringSplitOptions.RemoveEmptyEntries);
            return t is { Length: > 0 } ? double.Parse(t[0], CultureInfo.InvariantCulture) : null;
        }
        catch (Exception e) when (e is IOException or FormatException or UnauthorizedAccessException)
        {
            return null;
        }
    }

    public static string? Exec(string file, string args)
    {
        try
        {
            var psi = new ProcessStartInfo(file, args) { RedirectStandardOutput = true, RedirectStandardError = true, UseShellExecute = false };
            using var p = Process.Start(psi);
            if (p is null)
            {
                return null;
            }

            var o = p.StandardOutput.ReadToEnd();
            p.WaitForExit();
            return o;
        }
        catch (System.ComponentModel.Win32Exception)
        {
            return null;
        }
    }

    private static string Cpu()
    {
        if (File.Exists("/proc/cpuinfo"))
        {
            var l = File.ReadLines("/proc/cpuinfo").FirstOrDefault(x => x.StartsWith("model name", StringComparison.Ordinal));
            if (l is not null)
            {
                return l[(l.IndexOf(':', StringComparison.Ordinal) + 1)..].Trim();
            }
        }

        return Exec("sysctl", "-n machdep.cpu.brand_string")?.Trim() ?? RuntimeInformation.ProcessArchitecture.ToString();
    }

    /// <summary>The pinned reference releases, read from tools/refs/manifest.tsv (found by walking up from <paramref name="start"/>).</summary>
    private static Dictionary<string, string> Pins(string start)
    {
        var pins = new Dictionary<string, string>();
        for (var d = new DirectoryInfo(Path.GetFullPath(start)); d is not null; d = d.Parent)
        {
            var m = Path.Combine(d.FullName, "tools", "refs", "manifest.tsv");
            if (!File.Exists(m))
            {
                continue;
            }

            foreach (var line in File.ReadLines(m).Where(l => l.Length > 0 && l[0] != '#'))
            {
                var f = line.Split('\t');
                if (f.Length >= 4)
                {
                    pins[f[0]] = f[1] + " sha256:" + f[3];
                }
            }

            break;
        }

        return pins;
    }

    public static void Write(string path, string runId, string suite, int reps, IReadOnlyList<double?> loadDuring, double? loadBefore, double? loadAfter, string? machineName, string suiteDir)
    {
        var lockFile = Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "WORKER-BENCH.lock");
        using var fs = File.Create(path);
        using var w = new Utf8JsonWriter(fs, new JsonWriterOptions { Indented = true });
        w.WriteStartObject();
        w.WriteString("run_id", runId);
        w.WriteString("suite", suite);
        w.WriteNumber("reps_per_row", reps);
        w.WriteString("machine", machineName ?? Environment.MachineName);
        w.WriteString("os", RuntimeInformation.OSDescription);
        w.WriteString("arch", RuntimeInformation.OSArchitecture.ToString());
        w.WriteString("cpu", Cpu());
        w.WriteNumber("cores", Environment.ProcessorCount);
        w.WriteString("runtime", RuntimeInformation.FrameworkDescription);
        w.WriteString("sdk", Exec("dotnet", "--version")?.Trim() ?? "unknown");
        w.WriteString("onepath_version", OnePathInfo.Version.ToString());
        if (loadBefore is { } b)
        {
            w.WriteNumber("load1_before", b);
        }

        if (loadAfter is { } a)
        {
            w.WriteNumber("load1_after", a);
        }

        w.WriteStartArray("load1_during");
        foreach (var l in loadDuring.Where(x => x is not null))
        {
            w.WriteNumberValue(l!.Value);
        }

        w.WriteEndArray();
        w.WriteString("bench_lock_file", File.Exists(lockFile) ? File.ReadAllText(lockFile).Trim() : "(none)");
        w.WriteStartObject("references");
        foreach (var (k, v) in Pins(suiteDir))
        {
            w.WriteString(k, v);
        }

        w.WriteEndObject();
        w.WriteEndObject();
    }
}
