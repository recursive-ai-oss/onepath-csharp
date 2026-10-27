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
    public static void Main(string[] args) => BenchmarkSwitcher.FromAssembly(typeof(Program).Assembly).Run(args);
}
