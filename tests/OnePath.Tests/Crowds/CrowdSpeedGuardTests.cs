using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Crowds;

/// <summary>
/// Guards the agent-to-agent constraint's closed form (cutoff arc and two legs): for 400,000 seeded inputs (near, far, overlapping,
/// coincident, zero radius, equal velocities) it is finite with a unit normal and agrees with the general support-function construction.
/// </summary>
public class CrowdSpeedGuardTests
{
    [Fact]
    public void ForAgentMatchesTheGeneralConstruction()
    {
        var rnd = new Random(12345);
        double R(double s) => (rnd.NextDouble() * 2 - 1) * s;
        int compared = 0;
        for (int i = 0; i < 400000; i++)
        {
            double scale = Math.Pow(10, rnd.Next(-1, 3));
            var rel = new Vector2D(R(scale), R(scale));
            if (rnd.Next(10) == 0)
            {
                rel = new Vector2D(0, 0);
            }
            if (rnd.Next(10) == 0)
            {
                rel = new Vector2D(Math.Round(rel.X), 0);
            }
            var va = new Vector2D(R(3), R(3));
            var vb = new Vector2D(R(3), R(3));
            if (rnd.Next(8) == 0)
            {
                vb = va;
            }
            double r = rnd.NextDouble() * 4 * (rnd.Next(5) == 0 ? 0 : 1);
            double tau = 0.5 + rnd.NextDouble() * 10, dt = 0.05 + rnd.NextDouble();
            HalfPlane h = OrcaConstraints.ForAgent(rel, va, vb, r, tau, dt, 0.5);
            Assert.True(double.IsFinite(h.Point.X) && double.IsFinite(h.Point.Y) && double.IsFinite(h.Normal.X) && double.IsFinite(h.Normal.Y), $"input {i}");
            Assert.True(Math.Abs(h.Normal.Length() - 1) < 1e-9, $"input {i}: normal {h.Normal}");
            // Away from the touching case, where the two forms differ by the rounding of sqrt(|a|^2 - r^2), they agree closely.
            double len = rel.Length();
            if (len > r * (1 + 1e-4) && len > 1e-6)
            {
                OrcaConstraints.ClosestBoundaryPoint(rel, rel, r, tau, dt, va - vb, out Vector2D u, out Vector2D n);
                Vector2D expected = va + (u * 0.5);
                Assert.True((h.Normal - n).Length() < 1e-6, $"input {i}: normal {h.Normal} against {n}");
                Assert.True((h.Point - expected).Length() < 1e-6 * (1 + u.Length()), $"input {i}: point {h.Point} against {expected}");
                compared++;
            }
        }
        Assert.True(compared > 100000);
    }
}
