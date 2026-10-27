// Black-box driver for RVO2-CS, written from the public documentation
// (https://gamma.cs.unc.edu/RVO2/documentation/cs-2.0/). Same circle scenario as the C++ driver.
using System;
using System.Collections.Generic;
using System.Diagnostics;
using RVO;

namespace RecursiveAI.OnePath.RefHarness;

internal static class Rvo2CsDriver
{
    public static int Run(Args a)
    {
        if (a.Pos.Count < 2 || a.Pos[1] != "circle")
        {
            Console.Error.WriteLine("usage: rvo2cs circle --agents N --steps M [--warmup W] [--dt 0.25]");
            return 2;
        }

        int n = a.Int("agents", 250), steps = a.Int("steps", 200), warm = a.Int("warmup", 20);
        float dt = (float)a.Num("dt", 0.25);
        string scen = "circle" + n;
        float pi = 3.14159265358979f;
        float R = 200.0f * (float)Math.Sqrt(n / 250.0);

        var sim = Simulator.Instance;
        sim.Clear();
        sim.setTimeStep(dt);
        sim.setAgentDefaults(15.0f, 10, 10.0f, 5.0f, 2.0f, 2.0f, new Vector2(0.0f, 0.0f));
        var goals = new List<Vector2>();
        for (int i = 0; i < n; i++)
        {
            float ang = 2.0f * pi * i / n;
            sim.addAgent(new Vector2(R * (float)Math.Cos(ang), R * (float)Math.Sin(ang)));
            goals.Add(-sim.getAgentPosition(i));
        }

        void Step()
        {
            for (int i = 0; i < n; i++)
            {
                Vector2 d = goals[i] - sim.getAgentPosition(i);
                if (RVOMath.absSq(d) > 1.0f)
                {
                    d = RVOMath.normalize(d);
                }

                sim.setAgentPrefVelocity(i, d);
            }

            sim.doStep();
        }

        for (int i = 0; i < warm; i++)
        {
            Step();
        }

        var sw = Stopwatch.StartNew();
        for (int i = 0; i < steps; i++)
        {
            Step();
        }

        double ms = sw.Elapsed.TotalMilliseconds;
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            var p = sim.getAgentPosition(i);
            sum += p.x() + 2.0 * p.y();
        }

        const string side = "rvo2cs-2.0.1";
        Out.Row("crowd", scen, side, "agents", n, "count");
        Out.Row("crowd", scen, side, "steps", steps, "count");
        Out.Row("crowd", scen, side, "time", ms, "ms");
        Out.Row("crowd", scen, side, "agents_per_ms", (double)n * steps / ms, "agent-steps/ms");
        Out.Row("crowd", scen, side, "position_checksum", sum, "units");
        return 0;
    }
}
