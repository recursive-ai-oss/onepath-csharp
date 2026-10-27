using System;
using System.Runtime.CompilerServices;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Hash-matrix scenarios for the ORCA constraint builder, the linear programs and the simulator.</summary>
internal static class OrcaScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("orca/constraints", Constraints);
        Scenarios.Register("orca/lp", LinearPrograms);
        Scenarios.Register("orca/circle-48", ctx => Simulate(ctx, OrcaWorld.Circle(48, ctx.Threads), 400));
        Scenarios.Register("orca/crossing-6", ctx => Simulate(ctx, OrcaWorld.Crossing(6, ctx.Threads), 300));
        Scenarios.Register("orca/gap-24", ctx => Simulate(ctx, OrcaWorld.Gap(24, ctx.Threads), 400));
        Scenarios.Register("orca/corridor-28", ctx => Simulate(ctx, OrcaWorld.Corridor(28, ctx.Threads), 400));
    }

    // SplitMix64; every value is built from IEEE-exact operations.
    private sealed class Rng
    {
        private ulong _s;

        public Rng(ulong seed) => _s = seed;

        public double Next()
        {
            _s += 0x9E3779B97F4A7C15UL;
            ulong z = _s;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            z ^= z >> 31;
            return (z >> 11) * (1.0 / 9007199254740992.0);
        }

        public Vector2D Unit()
        {
            while (true)
            {
                Vector2D v = new((Next() * 2) - 1, (Next() * 2) - 1);
                double l2 = v.LengthSquared();
                if (l2 > 1e-6 && l2 <= 1)
                {
                    return v / v.Length();
                }
            }
        }
    }

    private static void W(ScenarioContext c, Vector2D v)
    {
        c.Write(v.X);
        c.Write(v.Y);
    }

    private static void Constraints(ScenarioContext ctx)
    {
        var r = new Rng(0x0CA1);
        for (int i = 0; i < 3000; i++)
        {
            Vector2D p = new((r.Next() * 20) - 10, (r.Next() * 20) - 10);
            Vector2D own = new((r.Next() * 4) - 2, (r.Next() * 4) - 2);
            Vector2D other = new((r.Next() * 4) - 2, (r.Next() * 4) - 2);
            double rad = 0.4 + (r.Next() * 1.2);
            double tau = 0.5 + (r.Next() * 5);
            HalfPlane h = OrcaConstraints.ForAgent(p, own, other, rad, tau, 0.1);
            W(ctx, h.Point);
            W(ctx, h.Normal);
            Vector2D a = new((r.Next() * 20) - 10, (r.Next() * 20) - 10);
            Vector2D b = i % 7 == 0 ? a : new Vector2D((r.Next() * 20) - 10, (r.Next() * 20) - 10);
            HalfPlane s = OrcaConstraints.ForSegment(a, b, rad, tau, 0.1);
            W(ctx, s.Point);
            W(ctx, s.Normal);
        }
    }

    private static void LinearPrograms(ScenarioContext ctx)
    {
        var r = new Rng(0x1B);
        var scratch = new HalfPlane[16];
        var c = new HalfPlane[12];
        for (int i = 0; i < 6000; i++)
        {
            int n = 1 + (int)(r.Next() * 12);
            for (int k = 0; k < n; k++)
            {
                Vector2D nrm = r.Unit();
                double s = (r.Next() * 3) - 2;
                c[k] = new HalfPlane(nrm * -s, nrm);
            }
            double speed = 0.5 + (r.Next() * 2.5);
            Vector2D pref = new((r.Next() * 8) - 4, (r.Next() * 8) - 4);
            int hard = n > 2 ? (int)(r.Next() * 3) : 0;
            Vector2D v = OrcaLinearProgram.Solve(c.AsSpan(0, n), hard, speed, pref, scratch, out bool feasible);
            W(ctx, v);
            ctx.Write(feasible ? 1 : 0);
        }
    }

    private static void Simulate(ScenarioContext ctx, OrcaWorld w, int steps)
    {
        for (int s = 1; s <= steps; s++)
        {
            w.Step();
            if (s % 100 == 0 || s == steps)
            {
                for (int i = 0; i < w.Sim.Count; i++)
                {
                    W(ctx, w.Sim[i].Position);
                    W(ctx, w.Sim[i].Velocity);
                }
            }
        }
    }
}
