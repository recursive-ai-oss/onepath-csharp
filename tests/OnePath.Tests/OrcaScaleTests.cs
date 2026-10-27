using System;
using System.Diagnostics;
using System.IO;
using RecursiveAI.OnePath.HashMatrix;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Larger runs, skipped unless ONEPATH_SCALE=1 (they take tens of seconds). A summary line per scenario goes to the file named by
/// ONEPATH_SCALE_OUT when set. Timing here is informational; the benchmarks measure speed.
/// </summary>
public class OrcaScaleTests
{
    private static void Report(string line)
    {
        Console.WriteLine(line);
        string? path = Environment.GetEnvironmentVariable("ONEPATH_SCALE_OUT");
        if (!string.IsNullOrEmpty(path))
        {
            File.AppendAllText(path, line + "\n");
        }
    }

    private static void Run(string name, OrcaWorld w, int maxSteps, double penetrationLimit)
    {
        Assert.SkipUnless(Environment.GetEnvironmentVariable("ONEPATH_SCALE") == "1", "set ONEPATH_SCALE=1");
        double worst = double.NegativeInfinity;
        int steps = 0;
        long start = Stopwatch.GetTimestamp();
        for (; steps < maxSteps; steps++)
        {
            w.Step();
            if (steps % 10 == 0)
            {
                worst = Math.Max(worst, w.MaxPenetration());
            }
            if (w.ArrivedCount() == w.Sim.Count)
            {
                break;
            }
        }
        double seconds = Stopwatch.GetElapsedTime(start).TotalSeconds;
        Report($"{name}: agents={w.Sim.Count} steps={steps} arrived={w.ArrivedCount()} deepest-overlap={worst:F3} (radius {OrcaWorld.Radius}) wall-seconds={seconds:F1}");
        Assert.Equal(w.Sim.Count, w.ArrivedCount());
        Assert.True(worst <= penetrationLimit, $"{name}: deepest overlap {worst}");
    }

    [Fact]
    public void Circle_100() => Run("circle-100", OrcaWorld.Circle(100), 6000, 0.5 * OrcaWorld.Radius);

    [Fact]
    public void Circle_1000() => Run("circle-1000", OrcaWorld.Circle(1000), 12000, 0.75 * OrcaWorld.Radius);

    [Fact]
    public void Crossing_1000() => Run("crossing-1000", OrcaWorld.Crossing(22), 20000, 0.5 * OrcaWorld.Radius);

    [Fact]
    public void Corridor_300() => Run("corridor-300", OrcaWorld.Corridor(300), 12000, 0.3 * OrcaWorld.Radius);
}
