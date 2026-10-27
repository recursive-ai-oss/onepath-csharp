using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class OrcaConstraintTests
{
    // Membership in the velocity obstacle { v | exists t in [0, tau]: t v in capsule(a, b, r) }, with slack eps on the radius.
    // The distance from t*v to the segment is convex in t, so a ternary search finds its minimum.
    private static bool InVo(Vector2D v, Vector2D a, Vector2D b, double r, double tau, double eps)
    {
        double lo = 0, hi = tau;
        for (int i = 0; i < 100; i++)
        {
            double m1 = lo + ((hi - lo) / 3), m2 = hi - ((hi - lo) / 3);
            if (SegDist(v * m1, a, b) < SegDist(v * m2, a, b))
            {
                hi = m2;
            }
            else
            {
                lo = m1;
            }
        }
        return SegDist(v * ((lo + hi) / 2), a, b) < r + eps;
    }

    private static double SegDist(Vector2D p, Vector2D a, Vector2D b)
    {
        Vector2D e = b - a;
        double ee = Vector2D.Dot(e, e);
        double s = ee == 0 ? 0 : Math.Clamp(Vector2D.Dot(p - a, e) / ee, 0, 1);
        return (p - (a + (e * s))).Length();
    }

    private static Vector2D Dir(double angle) => new(Math.Cos(angle), Math.Sin(angle));

    private static void CheckClosest(Random rng, bool segment, bool overlapping)
    {
        double r = 0.2 + (rng.NextDouble() * 0.8);
        double tau = 0.5 + (rng.NextDouble() * 3.5);
        double dt = 0.05 + (rng.NextDouble() * 0.2);
        Vector2D a, b;
        while (true)
        {
            a = new Vector2D((rng.NextDouble() * 10) - 5, (rng.NextDouble() * 10) - 5);
            b = segment ? new Vector2D((rng.NextDouble() * 10) - 5, (rng.NextDouble() * 10) - 5) : a;
            // Occasionally a segment seen (nearly) end on, or touching the origin's line.
            if (segment && rng.Next(10) == 0)
            {
                b = a * (1 + (rng.NextDouble() * 2));
            }
            double d2 = OrcaConstraints.SegmentDistanceSquared(a, b);
            if (overlapping ? d2 < r * r * 0.9 : d2 > r * r * 1.1)
            {
                break;
            }
        }
        Vector2D x = new((rng.NextDouble() * 8) - 4, (rng.NextDouble() * 8) - 4);
        OrcaConstraints.ClosestBoundaryPoint(a, b, r, tau, dt, x, out Vector2D u, out Vector2D n);
        Assert.Equal(1.0, n.Length(), 12);

        // The shape: the grown segment at time tau, or (overlapping) the capsule at time dt, scaled by 1/t.
        Func<Vector2D, double, bool> inside = overlapping
            ? (v, eps) => SegDist(v * dt, a, b) < r + eps
            : (v, eps) => InVo(v, a, b, r, tau, eps);
        Vector2D c = x + u;
        double dist = u.Length();
        bool xInside = inside(x, 0);

        // The point is (essentially) on the boundary.
        Assert.True(inside(c, 1e-6 * (overlapping ? 1 : tau) / (overlapping ? dt : 1)), "closest point is not in the closure");
        Assert.False(inside(c + (n * 1e-4), 0), "the point beyond the closest point along the normal is still inside");

        // The normal supports the obstacle at the closest point: every sampled member lies behind the tangent line.
        for (int k = 0; k < 400; k++)
        {
            Vector2D v = new((rng.NextDouble() * 16) - 8, (rng.NextDouble() * 16) - 8);
            if (inside(v, 0))
            {
                Assert.True(Vector2D.Dot(v - c, n) <= 1e-7, "a velocity inside the obstacle is in front of the tangent line");
            }
        }

        // No closer boundary point: a slightly smaller circle around x is entirely inside (x inside) or entirely outside (x outside).
        for (int k = 0; k < 720; k++)
        {
            Vector2D s = x + (Dir(k * Math.PI / 360) * (dist * 0.995));
            Assert.Equal(xInside, inside(s, 0));
        }
        if (xInside)
        {
            Assert.True(Vector2D.Dot(u, n) >= -1e-9, "inside: u should point along the outward normal");
        }
        else
        {
            Assert.True(Vector2D.Dot(u, n) <= 1e-9, "outside: u should point against the outward normal");
        }
    }

    [Fact]
    public void AgentCone_ClosestBoundaryPoint() { var rng = new Random(1); for (int i = 0; i < 1500; i++) { CheckClosest(rng, false, false); } }

    [Fact]
    public void SegmentCone_ClosestBoundaryPoint() { var rng = new Random(2); for (int i = 0; i < 1500; i++) { CheckClosest(rng, true, false); } }

    [Fact]
    public void Overlapping_AgentAndSegment_UseTheTimeStepCapsule()
    {
        var rng = new Random(3);
        for (int i = 0; i < 600; i++)
        {
            CheckClosest(rng, false, true);
            CheckClosest(rng, true, true);
        }
    }

    [Fact]
    public void ForAgent_KnownAnswer_HeadOn()
    {
        // B is 10 units ahead on the x axis, radius sum 2, tau 5, both at rest relative to each other (x = 0): the cutoff circle at (2, 0) with radius 0.4
        // is the nearest part of the boundary, so u points back from the origin along -x... the closest boundary point is (1.6, 0).
        HalfPlane h = OrcaConstraints.ForAgent(new Vector2D(10, 0), Vector2D.Zero, Vector2D.Zero, 2, 5, 0.1);
        Assert.Equal(-1.0, h.Normal.X, 12);
        Assert.Equal(0.0, h.Normal.Y, 12);
        // u = (1.6, 0) and the point is own velocity + u/2 = (0.8, 0); permitted: (v - (0.8, 0)) . (-1, 0) >= 0, i.e. vx <= 0.8.
        Assert.Equal(0.8, h.Point.X, 12);
        Assert.Equal(0.0, h.Point.Y, 12);
    }

    [Fact]
    public void ForSegment_ZeroVelocityIsAlwaysPermittedWhenNotOverlapping()
    {
        var rng = new Random(4);
        for (int i = 0; i < 5000; i++)
        {
            Vector2D a = new((rng.NextDouble() * 10) - 5, (rng.NextDouble() * 10) - 5);
            Vector2D b = new((rng.NextDouble() * 10) - 5, (rng.NextDouble() * 10) - 5);
            double r = 0.3;
            if (OrcaConstraints.SegmentDistanceSquared(a, b) <= r * r)
            {
                continue;
            }
            HalfPlane h = OrcaConstraints.ForSegment(a, b, r, 2.0, 0.1);
            Assert.True(h.SignedDistance(Vector2D.Zero) >= -1e-9, "the zero velocity must be permitted");
        }
    }

    [Fact]
    public void Arguments_AreValidated()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaConstraints.ForAgent(new Vector2D(1, 0), Vector2D.Zero, Vector2D.Zero, 1, 0, 0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaConstraints.ForAgent(new Vector2D(1, 0), Vector2D.Zero, Vector2D.Zero, 1, 1, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaConstraints.ForAgent(new Vector2D(1, 0), Vector2D.Zero, Vector2D.Zero, -1, 1, 0.1));
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaConstraints.ForSegment(Vector2D.Zero, Vector2D.One, 1, double.NaN, 0.1));
    }
}
