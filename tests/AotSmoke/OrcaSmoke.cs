using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Orca;

/// <summary>A small ORCA run used by the NativeAOT and WebAssembly smoke apps: two groups pass each other beside a wall.</summary>
internal static class OrcaSmoke
{
    public static bool Run()
    {
        var sim = new OrcaSimulator();
        sim.AddObstacle(new OrcaSegment(new Vector2D(-20, 4), new Vector2D(20, 4)));
        var goals = new Vector2D[8];
        for (int i = 0; i < 8; i++)
        {
            double y = ((i / 2) - 1.5) * 1.5;
            bool left = (i & 1) == 0;
            var a = new OrcaAgent
            {
                Position = new Vector2D(left ? -8 : 8, y + (left ? 0 : 0.2)),
                Radius = 0.5,
                MaxSpeed = 1.5,
                NeighborDistance = 10,
                MaxNeighbors = 8,
                TimeHorizon = 4,
                ObstacleTimeHorizon = 2,
            };
            sim.AddAgent(a);
            goals[i] = new Vector2D(left ? 8 : -8, y);
        }
        for (int s = 0; s < 400; s++)
        {
            for (int i = 0; i < sim.Count; i++)
            {
                Vector2D d = goals[i] - sim[i].Position;
                double len = d.Length();
                sim[i].PreferredVelocity = len > 1 ? d / len : d;
            }
            sim.Step(0.1);
        }
        for (int i = 0; i < sim.Count; i++)
        {
            if ((goals[i] - sim[i].Position).Length() > 1.0)
            {
                return false;
            }
            for (int j = i + 1; j < sim.Count; j++)
            {
                if ((sim[i].Position - sim[j].Position).Length() < 0.9)
                {
                    return false;
                }
            }
        }
        return true;
    }
}
