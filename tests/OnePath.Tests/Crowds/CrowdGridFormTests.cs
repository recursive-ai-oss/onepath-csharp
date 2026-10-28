using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Crowds;

public class CrowdGridFormTests
{
    // The neighbour grid has a dense form (counting sort by cell) and a sparse one (sorted keys, for widely spread agents). One far
    // away agent switches a simulation from the first to the second without being anyone's neighbour: the rest must not notice.
    [Fact]
    public void DenseAndSparseNeighbourGridsGiveTheSameSteps()
    {
        Vector2D[] dense = Run(false), sparse = Run(true);
        for (int i = 0; i < dense.Length; i++)
        {
            Assert.Equal(BitConverter.DoubleToInt64Bits(dense[i].X), BitConverter.DoubleToInt64Bits(sparse[i].X));
            Assert.Equal(BitConverter.DoubleToInt64Bits(dense[i].Y), BitConverter.DoubleToInt64Bits(sparse[i].Y));
        }
    }

    private static Vector2D[] Run(bool withFarAgent)
    {
        var rnd = new Random(99);
        var sim = new OrcaSimulator();
        const int n = 80;
        for (int i = 0; i < n; i++)
        {
            var p = new Vector2D(rnd.NextDouble() * 60, rnd.NextDouble() * 60);
            sim.AddAgent(new OrcaAgent { Position = p, Radius = 1, MaxSpeed = 2, NeighborDistance = 8, MaxNeighbors = 8, TimeHorizon = 5, ObstacleTimeHorizon = 2 });
        }
        if (withFarAgent)
        {
            sim.AddAgent(new OrcaAgent { Position = new Vector2D(1e6, -1e6), Radius = 1, MaxSpeed = 2, NeighborDistance = 8, MaxNeighbors = 8, TimeHorizon = 5, ObstacleTimeHorizon = 2 });
        }
        for (int step = 0; step < 30; step++)
        {
            for (int i = 0; i < n; i++)
            {
                sim[i].PreferredVelocity = new Vector2D(Math.Sin(i + (step * 0.3)), Math.Cos((i * 2) - (step * 0.2)));
            }
            sim.Step(0.25);
        }
        var result = new Vector2D[n];
        for (int i = 0; i < n; i++)
        {
            result[i] = sim[i].Position;
        }
        return result;
    }
}
