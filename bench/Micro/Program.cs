using System;
using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Running;
using RecursiveAI.OnePath;

namespace RecursiveAI.OnePath.Micro;

/// <summary>Placeholder benchmark that keeps the harness wired up.</summary>
[MemoryDiagnoser]
public class InfoBenchmarks
{
    [Benchmark]
    public int Version() => OnePathInfo.Version.Major;
}

public static class Program
{
    public static void Main(string[] args)
    {
        if (args.Length > 0 && args[0] == "--scale2d")
        {
            Build2DScaling.Run(args[1..]);
            return;
        }
        if (args.Length > 1 && args[0] == "--check-tilemap")
        {
            Environment.Exit(Build2DScaling.CheckTileMap(args[1], args.Length > 2 ? double.Parse(args[2], System.Globalization.CultureInfo.InvariantCulture) : 256, args.Length > 3 ? int.Parse(args[3], System.Globalization.CultureInfo.InvariantCulture) : 1));
        }
        if (args.Length > 0 && args[0] == "--loop2d")
        {
            Build2DScaling.Loop(int.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture), int.Parse(args[2], System.Globalization.CultureInfo.InvariantCulture));
            return;
        }
        if (args.Length > 0 && args[0] == "--digest2d")
        {
            Build2DScaling.Digest(args.Length > 1 ? int.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture) : 200);
            return;
        }
        BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
    }
}
