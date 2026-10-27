using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class OrcaLinearProgramTests
{
    private const double Tol = 1e-7;

    private static Vector2D Dir(double angle) => new(Math.Cos(angle), Math.Sin(angle));

    private static HalfPlane[] RandomSet(Random rng, out double radius, out Vector2D preferred)
    {
        int n = 1 + rng.Next(8);
        var c = new HalfPlane[n];
        radius = 0.5 + (rng.NextDouble() * 2.5);
        preferred = new Vector2D((rng.NextDouble() * 8) - 4, (rng.NextDouble() * 8) - 4);
        int style = rng.Next(3);
        for (int i = 0; i < n; i++)
        {
            Vector2D nrm = Dir(rng.NextDouble() * 2 * Math.PI);
            double s = style switch
            {
                0 => (rng.NextDouble() * 1.5) - 0.5,
                1 => (rng.NextDouble() * 3) - 2.0,
                _ => (rng.NextDouble() * 0.6) - 0.1,
            };
            // Permitted: v . n >= -s  (the point is -s * n).
            c[i] = new HalfPlane(nrm * -s, nrm);
        }
        return c;
    }

    // Brute-force optimum by vertex enumeration: the closest point of a convex region to a point is the point itself, a projection onto
    // an edge, or a vertex.
    private static bool BruteLp(HalfPlane[] c, double r, Vector2D pref, out Vector2D best)
    {
        var cand = new System.Collections.Generic.List<Vector2D>();
        double len = pref.Length();
        cand.Add(len > r ? pref / len * r : pref);
        if (len > 0)
        {
            cand.Add(pref / len * r);
        }
        for (int i = 0; i < c.Length; i++)
        {
            Vector2D n = c[i].Normal;
            Vector2D d = new(-n.Y, n.X);
            // Projection of pref onto line i.
            double t = Vector2D.Dot(pref - c[i].Point, d);
            cand.Add(c[i].Point + (d * t));
            // Line i with the circle.
            double pd = Vector2D.Dot(c[i].Point, d);
            double disc = (pd * pd) + (r * r) - Vector2D.Dot(c[i].Point, c[i].Point);
            if (disc >= 0)
            {
                double s = Math.Sqrt(disc);
                cand.Add(c[i].Point + (d * (-pd - s)));
                cand.Add(c[i].Point + (d * (-pd + s)));
            }
            for (int j = i + 1; j < c.Length; j++)
            {
                Vector2D m = c[j].Normal;
                double det = (n.X * m.Y) - (n.Y * m.X);
                if (Math.Abs(det) < 1e-12)
                {
                    continue;
                }
                double bi = Vector2D.Dot(n, c[i].Point), bj = Vector2D.Dot(m, c[j].Point);
                cand.Add(new Vector2D(((bi * m.Y) - (bj * n.Y)) / det, ((n.X * bj) - (m.X * bi)) / det));
            }
        }
        bool any = false;
        best = default;
        double bestD = double.PositiveInfinity;
        foreach (Vector2D v in cand)
        {
            if (v.Length() > r + 1e-9)
            {
                continue;
            }
            bool ok = true;
            foreach (HalfPlane h in c)
            {
                if (h.SignedDistance(v) < -1e-9)
                {
                    ok = false;
                    break;
                }
            }
            if (!ok)
            {
                continue;
            }
            double dd = (v - pref).Length();
            if (dd < bestD)
            {
                bestD = dd;
                best = v;
                any = true;
            }
        }
        return any;
    }

    private static double MaxViolation(HalfPlane[] c, Vector2D v)
    {
        double worst = double.NegativeInfinity;
        foreach (HalfPlane h in c)
        {
            worst = Math.Max(worst, -h.SignedDistance(v));
        }
        return worst;
    }

    // Brute-force minimum of the largest penetration over the speed disc: vertices of the epigraph polytope cut by the cylinder.
    private static double BruteMinMax(HalfPlane[] c, double r)
    {
        var cand = new System.Collections.Generic.List<Vector2D>();
        int n = c.Length;
        for (int i = 0; i < n; i++)
        {
            cand.Add(c[i].Normal * r);
            for (int j = i + 1; j < n; j++)
            {
                // g_i = g_j: v . (n_i - n_j) = p_i.n_i - p_j.n_j; intersect with the circle.
                Vector2D m = c[i].Normal - c[j].Normal;
                double len2 = Vector2D.Dot(m, m);
                if (len2 < 1e-18)
                {
                    continue;
                }
                double rhs = Vector2D.Dot(c[i].Point, c[i].Normal) - Vector2D.Dot(c[j].Point, c[j].Normal);
                Vector2D q = m * (rhs / len2);
                double h2 = (r * r) - Vector2D.Dot(q, q);
                if (h2 >= 0)
                {
                    Vector2D perp = new(-m.Y, m.X);
                    double s = Math.Sqrt(h2 / len2);
                    cand.Add(q + (perp * s));
                    cand.Add(q - (perp * s));
                }
                for (int k = j + 1; k < n; k++)
                {
                    // g_i = g_j = g_k = t.
                    Vector2D m2 = c[i].Normal - c[k].Normal;
                    double rhs2 = Vector2D.Dot(c[i].Point, c[i].Normal) - Vector2D.Dot(c[k].Point, c[k].Normal);
                    double det = (m.X * m2.Y) - (m.Y * m2.X);
                    if (Math.Abs(det) < 1e-12)
                    {
                        continue;
                    }
                    cand.Add(new Vector2D(((rhs * m2.Y) - (rhs2 * m.Y)) / det, ((m.X * rhs2) - (m2.X * rhs)) / det));
                }
            }
        }
        double best = double.PositiveInfinity;
        foreach (Vector2D v in cand)
        {
            if (v.Length() <= r + 1e-9)
            {
                best = Math.Min(best, MaxViolation(c, v));
            }
        }
        return best;
    }

    [Fact]
    public void RandomSets_AgreeWithBruteForce()
    {
        var rng = new Random(20260101);
        int feasibleCount = 0, infeasibleCount = 0;
        var scratch = new HalfPlane[16];
        for (int iter = 0; iter < 120_000; iter++)
        {
            HalfPlane[] c = RandomSet(rng, out double r, out Vector2D pref);
            Vector2D v = OrcaLinearProgram.Solve(c, 0, r, pref, scratch, out bool feasible);
            Assert.True(v.Length() <= r + 1e-9, $"set {iter}: speed disc violated");
            bool brute = BruteLp(c, r, pref, out Vector2D bv);
            if (feasible)
            {
                feasibleCount++;
                Assert.True(brute, $"set {iter}: solver feasible, brute force infeasible");
                Assert.True(MaxViolation(c, v) <= Tol, $"set {iter}: infeasible velocity returned");
                Assert.True((v - pref).Length() <= (bv - pref).Length() + Tol, $"set {iter}: not optimal ({(v - pref).Length()} vs {(bv - pref).Length()})");
            }
            else
            {
                infeasibleCount++;
                Assert.False(brute, $"set {iter}: solver infeasible, brute force feasible");
                double ours = MaxViolation(c, v);
                double theirs = BruteMinMax(c, r);
                Assert.True(ours <= theirs + 1e-6, $"set {iter}: penetration {ours} vs minimum {theirs}");
                Assert.True(ours >= theirs - 1e-6, $"set {iter}: below the minimum?! {ours} vs {theirs}");
            }
        }
        Assert.True(feasibleCount > 20_000 && infeasibleCount > 20_000, $"{feasibleCount} feasible, {infeasibleCount} infeasible");
    }

    [Fact]
    public void HardConstraints_AreNeverRelaxed()
    {
        var rng = new Random(77);
        var scratch = new HalfPlane[16];
        int relaxedRuns = 0;
        for (int iter = 0; iter < 20_000; iter++)
        {
            HalfPlane[] c = RandomSet(rng, out double r, out Vector2D pref);
            if (c.Length < 3)
            {
                continue;
            }
            int hard = 1 + rng.Next(2);
            // Skip sets whose hard part alone is infeasible.
            var hardOnly = c.AsSpan(0, hard).ToArray();
            if (!BruteLp(hardOnly, r, pref, out _))
            {
                continue;
            }
            Vector2D v = OrcaLinearProgram.Solve(c, hard, r, pref, scratch, out bool feasible);
            if (!feasible)
            {
                relaxedRuns++;
            }
            Assert.True(v.Length() <= r + 1e-9);
            Assert.True(MaxViolation(hardOnly, v) <= Tol, $"set {iter}: a hard constraint was violated");
        }
        Assert.True(relaxedRuns > 1000);
    }

    [Fact]
    public void Arguments_AreValidated()
    {
        var scratch = new HalfPlane[2];
        var one = new[] { new HalfPlane(Vector2D.Zero, new Vector2D(1, 0)) };
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaLinearProgram.Solve(one, 2, 1, Vector2D.Zero, scratch, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaLinearProgram.Solve(one, 0, -1, Vector2D.Zero, scratch, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => OrcaLinearProgram.Solve(one, 0, double.NaN, Vector2D.Zero, scratch, out _));
        Assert.Throws<ArgumentException>(() => OrcaLinearProgram.Solve(one, 0, 1, Vector2D.Zero, Span<HalfPlane>.Empty, out _));
    }

    [Fact]
    public void NoConstraints_ReturnsClampedPreferredVelocity()
    {
        var scratch = new HalfPlane[1];
        Vector2D v = OrcaLinearProgram.Solve(ReadOnlySpan<HalfPlane>.Empty, 0, 2, new Vector2D(3, 4), scratch, out bool feasible);
        Assert.True(feasible);
        Assert.Equal(new Vector2D(1.2, 1.6), v);
        v = OrcaLinearProgram.Solve(ReadOnlySpan<HalfPlane>.Empty, 0, 10, new Vector2D(3, 4), scratch, out _);
        Assert.Equal(new Vector2D(3, 4), v);
    }
}
