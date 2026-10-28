using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Crowds;

/// <summary>
/// Guards the agent-to-agent constraint fast path: the half-planes for 400,000 seeded inputs (near, far, overlapping, coincident,
/// zero radius, equal velocities) hash to the value the general segment construction produced before the fast path existed.
/// </summary>
public class CrowdSpeedGuardTests
{
    [Fact]
    public void ForAgentMatchesTheGeneralConstructionBitForBit()
    {
        var rnd = new Random(12345);
        double R(double s) => (rnd.NextDouble() * 2 - 1) * s;
        var bytes = new List<byte>(400000 * 32);
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
            HalfPlane h = OrcaConstraints.ForAgent(rel, va, vb, r, 0.5 + rnd.NextDouble() * 10, 0.05 + rnd.NextDouble(), 0.5);
            bytes.AddRange(BitConverter.GetBytes(h.Point.X));
            bytes.AddRange(BitConverter.GetBytes(h.Point.Y));
            bytes.AddRange(BitConverter.GetBytes(h.Normal.X));
            bytes.AddRange(BitConverter.GetBytes(h.Normal.Y));
        }
        Assert.Equal("1FEB73E5F74D3F319B37F2ABCC80F77344924473889F4083164F7E1B3262EB49", Convert.ToHexString(SHA256.HashData(bytes.ToArray())));
    }
}
