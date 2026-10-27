using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.HashMatrix;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class OrcaSimulationTests
{
    // Agents may overlap slightly when no permitted velocity exists and the paper's fallback is used. Stated limits on the deepest overlap
    // (a fraction of the radius): sparse scenes, and the dense centre of a big antipodal circle.
    private const double PenetrationLimit = 0.1 * OrcaWorld.Radius;
    private const double DensePenetrationLimit = 0.5 * OrcaWorld.Radius;

    private static (double Worst, int Steps, int Arrived) Run(OrcaWorld w, int maxSteps)
    {
        double worst = double.NegativeInfinity;
        int steps = 0;
        for (; steps < maxSteps; steps++)
        {
            w.Step();
            worst = Math.Max(worst, w.MaxPenetration());
            if (w.ArrivedCount() == w.Sim.Count)
            {
                break;
            }
        }
        return (worst, steps, w.ArrivedCount());
    }

    [Fact]
    public void TwoAgents_PassEachOther()
    {
        var sim = new OrcaSimulator();
        sim.AddAgent(OrcaWorld.DefaultAgent(new Vector2D(-10, 0)));
        sim.AddAgent(OrcaWorld.DefaultAgent(new Vector2D(10, 0)));
        var w = new OrcaWorld(sim, new[] { new Vector2D(10, 0), new Vector2D(-10, 0) });
        var (worst, steps, arrived) = Run(w, 600);
        Assert.Equal(2, arrived);
        Assert.True(worst <= 1e-6, $"overlap {worst}");
        Assert.True(steps < 400);
    }

    [Theory]
    [InlineData(5)]
    [InlineData(24)]
    [InlineData(100)]
    public void AntipodalCircle_ReachesGoalsWithoutOverlap(int n)
    {
        var w = OrcaWorld.Circle(n);
        var (worst, steps, arrived) = Run(w, 4000);
        Assert.Equal(n, arrived);
        Assert.True(worst <= (n >= 100 ? DensePenetrationLimit : PenetrationLimit), $"deepest overlap {worst}");
        Assert.True(steps < 3000, $"{steps} steps");
    }

    [Fact]
    public void CrossingBlocks_ReachGoalsWithoutOverlap()
    {
        var w = OrcaWorld.Crossing(5);
        var (worst, steps, arrived) = Run(w, 4000);
        Assert.Equal(w.Sim.Count, arrived);
        Assert.True(worst <= PenetrationLimit, $"deepest overlap {worst}");
        Assert.True(steps < 3000, $"{steps} steps");
    }

    [Fact]
    public void GapInWall_AgentsNeverCrossTheWall()
    {
        var w = OrcaWorld.Gap(24);
        double worstWall = double.NegativeInfinity;
        for (int steps = 0; steps < 3000; steps++)
        {
            w.Step();
            for (int i = 0; i < w.Sim.Count; i++)
            {
                Vector2D p = w.Sim[i].Position;
                // The wall is the line x = 0 except for |y| < 1.5: agents may not be within their radius of the wall's solid parts.
                if (Math.Abs(p.Y) >= 1.5)
                {
                    worstWall = Math.Max(worstWall, w.Sim[i].Radius - Math.Abs(p.X));
                }
                else
                {
                    double corner = w.Sim[i].Radius - new Vector2D(p.X, Math.Abs(p.Y) - 1.5).Length();
                    worstWall = Math.Max(worstWall, corner);
                }
            }
            if (w.ArrivedCount() == w.Sim.Count)
            {
                break;
            }
        }
        Assert.Equal(w.Sim.Count, w.ArrivedCount());
        Assert.True(worstWall <= 1e-6, $"agents penetrated the wall by {worstWall}");
        Assert.True(w.MaxPenetration() <= PenetrationLimit);
    }

    [Fact]
    public void Corridor_TwoWayTrafficPasses()
    {
        var w = OrcaWorld.Corridor(42);
        var (worst, _, arrived) = Run(w, 5000);
        Assert.Equal(42, arrived);
        Assert.True(worst <= PenetrationLimit, $"deepest overlap {worst}");
        for (int i = 0; i < w.Sim.Count; i++)
        {
            Assert.True(Math.Abs(w.Sim[i].Position.Y) <= 5.0 - w.Sim[i].Radius + 1e-6, "left the corridor");
        }
    }

    [Fact]
    public void Results_DoNotDependOnTheThreadCount()
    {
        static Vector2D[] Final(int threads)
        {
            var w = OrcaWorld.Crossing(6, threads);
            for (int s = 0; s < 150; s++)
            {
                w.Step();
            }
            var p = new Vector2D[w.Sim.Count];
            for (int i = 0; i < p.Length; i++)
            {
                p[i] = w.Sim[i].Position;
            }
            return p;
        }
        Vector2D[] one = Final(1);
        foreach (int t in new[] { 2, 3, 8 })
        {
            Vector2D[] other = Final(t);
            for (int i = 0; i < one.Length; i++)
            {
                Assert.True(
                    BitConverter.DoubleToInt64Bits(one[i].X) == BitConverter.DoubleToInt64Bits(other[i].X) &&
                    BitConverter.DoubleToInt64Bits(one[i].Y) == BitConverter.DoubleToInt64Bits(other[i].Y),
                    $"agent {i} differs at {t} threads");
            }
        }
    }

    [Fact]
    public void Neighbors_AreTheNearestWithinRange_TiesByIndex()
    {
        // Agent 0 at the origin uses at most 2 neighbours. Four agents sit at distance 1.2 in the axis directions; the two lowest indices
        // are the ones used, so moving the other two far away must not change agent 0's velocity.
        static OrcaSimulator Build(bool moveFar)
        {
            var sim = new OrcaSimulator();
            OrcaAgent center = OrcaWorld.DefaultAgent(Vector2D.Zero);
            center.MaxNeighbors = 2;
            center.PreferredVelocity = new Vector2D(1, 0);
            sim.AddAgent(center);
            Vector2D[] ps = { new(1.2, 0), new(-1.2, 0), new(0, 1.2), new(0, -1.2) };
            for (int i = 0; i < 4; i++)
            {
                OrcaAgent a = OrcaWorld.DefaultAgent(moveFar && i >= 2 ? ps[i] * 5 : ps[i]);
                a.MaxSpeed = 0;
                sim.AddAgent(a);
            }
            return sim;
        }
        OrcaSimulator s1 = Build(false), s2 = Build(true);
        s1.Step(0.1);
        s2.Step(0.1);
        Assert.Equal(s1[0].Velocity, s2[0].Velocity);
    }

    [Fact]
    public void Step_DoesNotAllocateOnceWarm()
    {
        var w = OrcaWorld.Gap(40);
        for (int s = 0; s < 20; s++)
        {
            w.Step();
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int s = 0; s < 50; s++)
        {
            w.Step();
        }
        long after = GC.GetAllocatedBytesForCurrentThread();
        Assert.Equal(0, after - before);
    }

    [Fact]
    public void Inputs_AreValidated()
    {
        var sim = new OrcaSimulator();
        OrcaAgent good = OrcaWorld.DefaultAgent(Vector2D.Zero);
        OrcaAgent bad = good;
        bad.Radius = -1;
        Assert.Throws<ArgumentException>(() => sim.AddAgent(bad));
        bad = good;
        bad.Position = new Vector2D(double.NaN, 0);
        Assert.Throws<ArgumentException>(() => sim.AddAgent(bad));
        bad = good;
        bad.Position = new Vector2D(3_000_000, 0);
        Assert.Throws<ArgumentOutOfRangeException>(() => sim.AddAgent(bad));
        bad = good;
        bad.TimeHorizon = 0;
        Assert.Throws<ArgumentException>(() => sim.AddAgent(bad));
        Assert.Throws<ArgumentException>(() => sim.AddObstacle(new OrcaSegment(new Vector2D(double.PositiveInfinity, 0), Vector2D.Zero)));
        Assert.Throws<ArgumentOutOfRangeException>(() => sim.Step(0.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => sim[0].Radius);
        Assert.Throws<ArgumentOutOfRangeException>(() => sim.MaxThreads = 0);
    }
}
