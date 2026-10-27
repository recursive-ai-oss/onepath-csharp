using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;

namespace RecursiveAI.OnePath.RefHarness;

/// <summary>Command-line arguments: positional words and <c>--key value</c> pairs.</summary>
internal sealed class Args
{
    public List<string> Pos { get; } = new();

    private readonly Dictionary<string, string> kv = new();

    public Args(string[] argv)
    {
        for (int i = 0; i < argv.Length; i++)
        {
            if (argv[i].StartsWith("--", StringComparison.Ordinal))
            {
                string k = argv[i].Substring(2);
                kv[k] = i + 1 < argv.Length && !argv[i + 1].StartsWith("--", StringComparison.Ordinal) ? argv[++i] : "1";
            }
            else
            {
                Pos.Add(argv[i]);
            }
        }
    }

    public string Str(string k, string d = "") => kv.TryGetValue(k, out var v) ? v : d;

    public double Num(string k, double d) => kv.TryGetValue(k, out var v) ? double.Parse(v, CultureInfo.InvariantCulture) : d;

    public int Int(string k, int d) => kv.TryGetValue(k, out var v) ? int.Parse(v, CultureInfo.InvariantCulture) : d;
}

internal static class Out
{
    public static void Row(string part, string scenario, string side, string metric, double value, string unit) =>
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "ROW,{0},{1},{2},{3},{4:F6},{5}", part, scenario, side, metric, value, unit));

    public static void Text(string part, string scenario, string side, string metric, string value) =>
        Console.WriteLine(string.Format(CultureInfo.InvariantCulture, "ROW,{0},{1},{2},{3},{4},text", part, scenario, side, metric, value));

    public static double Best(List<double> v) { double m = double.MaxValue; foreach (var x in v) { m = Math.Min(m, x); } return m; }

    public static double Median(List<double> v) { var c = new List<double>(v); c.Sort(); return c[c.Count / 2]; }

    public static double PeakRssMb() => Process.GetCurrentProcess().PeakWorkingSet64 / (1024.0 * 1024.0);
}

/// <summary>Minimal Wavefront OBJ reader ("v x y z", "f a b c ..." fan-triangulated) and point-file reader.</summary>
internal static class Input
{
    public static bool LoadObj(string path, out float[] verts, out int[] tris)
    {
        var v = new List<float>();
        var t = new List<int>();
        foreach (var line in File.ReadLines(path))
        {
            if (line.StartsWith("v ", StringComparison.Ordinal))
            {
                var f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                v.Add(float.Parse(f[1], CultureInfo.InvariantCulture));
                v.Add(float.Parse(f[2], CultureInfo.InvariantCulture));
                v.Add(float.Parse(f[3], CultureInfo.InvariantCulture));
            }
            else if (line.StartsWith("f ", StringComparison.Ordinal))
            {
                var f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
                for (int i = 3; i < f.Length; i++)
                {
                    t.Add(int.Parse(f[1].Split('/')[0], CultureInfo.InvariantCulture) - 1);
                    t.Add(int.Parse(f[i - 1].Split('/')[0], CultureInfo.InvariantCulture) - 1);
                    t.Add(int.Parse(f[i].Split('/')[0], CultureInfo.InvariantCulture) - 1);
                }
            }
        }

        verts = v.ToArray();
        tris = t.ToArray();
        return tris.Length > 0;
    }

    public static float[] LoadPoints(string path)
    {
        var p = new List<float>();
        foreach (var line in File.ReadLines(path))
        {
            if (line.Length == 0 || line[0] == '#')
            {
                continue;
            }

            var f = line.Split(' ', StringSplitOptions.RemoveEmptyEntries);
            if (f.Length >= 3)
            {
                p.Add(float.Parse(f[0], CultureInfo.InvariantCulture));
                p.Add(float.Parse(f[1], CultureInfo.InvariantCulture));
                p.Add(float.Parse(f[2], CultureInfo.InvariantCulture));
            }
        }

        return p.ToArray();
    }
}

internal static class Program
{
    private static int Main(string[] argv)
    {
        var a = new Args(argv);
        if (a.Pos.Count == 0)
        {
            Console.Error.WriteLine("usage: OnePath.RefHarness dotrecast build|query|crowd ... | rvo2cs circle ...");
            return 2;
        }

        switch (a.Pos[0])
        {
            case "dotrecast":
                return DotRecastDriver.Run(a);
            case "rvo2cs":
                return RunRvo2Cs(a);
            default:
                Console.Error.WriteLine("unknown driver " + a.Pos[0]);
                return 2;
        }
    }

    private static int RunRvo2Cs(Args a)
    {
#if RVO2CS
        return Rvo2CsDriver.Run(a);
#else
        Console.Error.WriteLine("built without Rvo2Cs (pass -p:Rvo2CsDll=...)");
        return 3;
#endif
    }
}
