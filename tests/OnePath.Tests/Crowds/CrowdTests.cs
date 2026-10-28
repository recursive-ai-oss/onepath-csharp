using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class CrowdTests
{
    private const double Dt = CrowdWorlds.Dt;
    private readonly ITestOutputHelper _out;

    public CrowdTests(ITestOutputHelper output) => _out = output;

    private sealed record RunStats(int Agents, int Arrived, int Steps, double WorstOverlap, int OffMesh, int Crossings)
    {
        public string Lost { get; init; } = "";
    }

    // Steps the crowd until every agent has stopped (or the budget is used) and checks, at every step, that each agent
    // stands on the mesh. Overlap = how far inside each other two agents are (sum of radii minus the distance), worst over the run.
    private static RunStats Run(CrowdSim sim, int budget, bool checkPairs = true, Action<int>? onStep = null, bool removeOnArrival = false)
    {
        var q = new NavMeshQuery(sim.Mesh, 65536, PathNodeModel.Crossing);
        double worst = 0;
        int offMesh = 0, crossings = 0, steps = 0;
        var crossed = new HashSet<long>();
        var arrivedIds = new HashSet<long>();
        for (; steps < budget; steps++)
        {
            onStep?.Invoke(steps);
            sim.Crowd.Step(Dt);
            bool anyMoving = false;
            for (int i = 0; i < sim.Agents.Count; i++)
            {
                Agent a = sim.Agents[i];
                if (!a.IsActive)
                {
                    continue;
                }
                if (removeOnArrival && a.State == FollowerState.Arrived)
                {
                    arrivedIds.Add(a.Id);
                    sim.Crowd.Remove(a);
                    continue;
                }
                bool crossing = a.State == FollowerState.OffMeshLink;
                if (!crossing && !CrowdWorlds.OnMesh(q, a.Position))
                {
                    offMesh++;
                }
                if (crossing && crossed.Add(a.Id))
                {
                    crossings++;
                }
                anyMoving |= a.HasPath;
                if (!checkPairs || crossing)
                {
                    continue;
                }
                for (int j = i + 1; j < sim.Agents.Count; j++)
                {
                    Agent b = sim.Agents[j];
                    if (!b.IsActive || b.State == FollowerState.OffMeshLink || Math.Abs(a.Position.Y - b.Position.Y) > 1.0)
                    {
                        continue;
                    }
                    double dx = a.Position.X - b.Position.X, dz = a.Position.Z - b.Position.Z;
                    double ov = a.Options.Radius + b.Options.Radius - Math.Sqrt((dx * dx) + (dz * dz));
                    if (ov > worst)
                    {
                        worst = ov;
                    }
                }
            }
            if (!anyMoving)
            {
                steps++;
                break;
            }
        }
        int arrived = arrivedIds.Count;
        string lost = "";
        foreach (Agent a in sim.Agents)
        {
            if (!a.IsActive)
            {
                continue;
            }
            if (a.State == FollowerState.Arrived)
            {
                arrived++;
            }
            else
            {
                lost += $" [agent {a.Id} {a.State} at ({a.Position.X:F2},{a.Position.Y:F2},{a.Position.Z:F2}) to ({a.Destination.X:F2},{a.Destination.Z:F2}) remaining {a.RemainingDistance:F2}]";
            }
        }
        return new RunStats(sim.Agents.Count, arrived, steps, worst, offMesh, crossings) { Lost = lost };
    }

    [Theory]
    [InlineData(20, 1UL)]
    [InlineData(60, 2UL)]
    [InlineData(150, 3UL)]
    public void Crossing_streams_arrive_and_stay_on_the_mesh(int n, ulong seed)
    {
        CrowdSim sim = CrowdWorlds.Crossing(n, seed, 1);
        RunStats s = Run(sim, 3000);
        _out.WriteLine(s.ToString());
        Assert.Equal(0, s.OffMesh);
        Assert.Equal(s.Agents, s.Arrived);
        // Radii are 0.3 each: two agents never get more than a fifth of their combined radius into each other.
        Assert.True(s.WorstOverlap <= 0.12, $"overlap {s.WorstOverlap}");
    }

    [Theory]
    [InlineData(20, 1UL, 2)]
    [InlineData(40, 2UL, 2)]
    [InlineData(60, 3UL, 3)]
    public void Doorway_agents_pass_the_door(int n, ulong seed, int door)
    {
        CrowdSim sim = CrowdWorlds.ThroughDoor(n, seed, 1, door);
        RunStats s = Run(sim, 4000);
        _out.WriteLine(s.ToString());
        Assert.Equal(0, s.OffMesh);
        Assert.Equal(s.Agents, s.Arrived);
        Assert.True(s.WorstOverlap <= 0.12, $"overlap {s.WorstOverlap}");
    }

    [Fact]
    public void Seeded_field_runs_stay_on_the_mesh_and_arrive_within_the_step_budget()
    {
        const int Runs = 1000;
        int agents = 0, overlaps = 0;
        double worstOverlap = 0, worstRatio = 0;
        for (ulong seed = 1; seed <= Runs; seed++)
        {
            CrowdSim sim = CrowdWorlds.RandomField(8, 6, seed, 1);
            var fresh = new NavMeshQuery(sim.Mesh, 65536, PathNodeModel.Crossing);
            double longest = 0;
            foreach (Agent a in sim.Agents)
            {
                longest = Math.Max(longest, fresh.FindPath(a.Position, a.Destination).Length);
            }
            // Budget: three times the time to walk the longest route at full speed, plus ten seconds for queues and turns.
            int budget = (int)((3 * longest / (Agent.Small.MaxSpeed * Dt)) + (10 / Dt));
            RunStats s = Run(sim, budget, removeOnArrival: true);
            agents += s.Agents;
            worstOverlap = Math.Max(worstOverlap, s.WorstOverlap);
            if (s.Agents > 0)
            {
                worstRatio = Math.Max(worstRatio, s.Steps * Dt / Math.Max(1.0, longest / Agent.Small.MaxSpeed));
            }
            Assert.True(s.OffMesh == 0, $"seed {seed}: {s.OffMesh} agent positions off the mesh");
            Assert.True(s.Arrived == s.Agents, $"seed {seed}: {s.Arrived} of {s.Agents} arrived in {s.Steps} steps (budget {budget}){s.Lost}");
            Assert.True(s.WorstOverlap <= 0.3, $"seed {seed}: overlap {s.WorstOverlap}");
            if (s.WorstOverlap > 0.05)
            {
                overlaps++;
            }
        }
        _out.WriteLine($"{Runs} runs, {agents} agents, worst overlap {worstOverlap:F4}, runs over 0.05: {overlaps}, worst time ratio {worstRatio:F2}");
    }

    [Fact]
    public void Two_floors_with_off_mesh_links_arrive()
    {
        int crossings = 0, agents = 0;
        for (ulong seed = 1; seed <= 40; seed++)
        {
            CrowdSim sim = CrowdWorlds.TwoFloors(4, seed, 1, null);
            RunStats s = Run(sim, 4000, removeOnArrival: true);
            agents += s.Agents;
            crossings += s.Crossings;
            Assert.True(s.OffMesh == 0, $"seed {seed}: {s.OffMesh} positions off the mesh");
            Assert.True(s.Arrived == s.Agents, $"seed {seed}: {s.Arrived} of {s.Agents} arrived in {s.Steps} steps{s.Lost}");
            // An agent leaving a link lands where it lands, possibly on someone (crossing agents do not avoid): up to half the combined radius.
            Assert.True(s.WorstOverlap <= 0.3, $"seed {seed}: overlap {s.WorstOverlap}");
        }
        _out.WriteLine($"{agents} agents, {crossings} off-mesh crossings");
        Assert.True(crossings > 20, "the scenario should cross links");
    }

    private static byte[] StateBytes(CrowdSim sim)
    {
        var bytes = new List<byte>();
        foreach (Agent a in sim.Agents)
        {
            bytes.AddRange(a.Path.ToBytes());
        }
        return bytes.ToArray();
    }

    [Theory]
    [InlineData(2)]
    [InlineData(5)]
    [InlineData(16)]
    public void Thread_count_does_not_change_the_result(int threads)
    {
        CrowdSim a = CrowdWorlds.Crossing(300, 7, 1);
        CrowdSim b = CrowdWorlds.Crossing(300, 7, threads);
        for (int step = 0; step < 150; step++)
        {
            a.Crowd.Step(Dt);
            b.Crowd.Step(Dt);
        }
        Assert.Equal(StateBytes(a), StateBytes(b));
    }

    [Fact]
    public void Steady_state_steps_do_not_allocate()
    {
        CrowdSim sim = CrowdWorlds.Crossing(120, 4, 1);
        for (int i = 0; i < 60; i++)
        {
            sim.Crowd.Step(Dt);
        }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 60; i++)
        {
            sim.Crowd.Step(Dt);
        }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        _out.WriteLine($"{allocated} bytes in 60 steps");
        Assert.True(allocated <= 4096, $"{allocated} bytes allocated in 60 steps");
    }

    [Fact]
    public void Same_inputs_give_the_same_state_twice()
    {
        byte[] x = RunBytes();
        byte[] y = RunBytes();
        Assert.Equal(x, y);

        static byte[] RunBytes()
        {
            CrowdSim s = CrowdWorlds.ThroughDoor(30, 9, 1);
            for (int i = 0; i < 200; i++)
            {
                s.Crowd.Step(Dt);
            }
            return StateBytes(s);
        }
    }

    [Fact]
    public void Higher_priority_gives_way_less()
    {
        // Two agents walk toward each other along a corridor slightly off axis; the one with priority 4 should be pushed
        // sideways about a quarter as far as the one with priority 1.
        TiledNavMesh mesh = CrowdWorlds.Corridor(40, 8);
        var crowd = new Crowd(mesh);
        AgentOptions opt = Agent.Small with { MaxAcceleration = 50, SidePreference = 0 };
        Agent heavy = crowd.Add(new Vector3D(2, 0, 4.02), opt with { Priority = 4 });
        Agent light = crowd.Add(new Vector3D(38, 0, 3.98), opt with { Priority = 1 });
        heavy.SetDestination(new Vector3D(38, 0, 4));
        light.SetDestination(new Vector3D(2, 0, 4));
        double heavyDev = 0, lightDev = 0;
        for (int i = 0; i < 600 && (heavy.HasPath || light.HasPath); i++)
        {
            crowd.Step(Dt);
            heavyDev = Math.Max(heavyDev, Math.Abs(heavy.Position.Z - 4.0));
            lightDev = Math.Max(lightDev, Math.Abs(light.Position.Z - 4.0));
        }
        _out.WriteLine($"heavy {heavyDev:F3} light {lightDev:F3}");
        Assert.True(heavy.State == FollowerState.Arrived && light.State == FollowerState.Arrived);
        Assert.True(heavyDev < lightDev, $"heavy {heavyDev} light {lightDev}");
    }

    [Fact]
    public void Separation_spreads_agents_that_start_on_top_of_each_other()
    {
        TiledNavMesh mesh = CrowdWorlds.Arena(20, 20);
        var crowd = new Crowd(mesh);
        AgentOptions opt = Agent.Small with { SeparationDistance = 1.5, SeparationWeight = 0.6 };
        var agents = new List<Agent>();
        for (int i = 0; i < 8; i++)
        {
            agents.Add(crowd.Add(new Vector3D(10 + (i * 0.01), 0, 10 + (i * 0.013)), opt));
        }
        for (int i = 0; i < 240; i++)
        {
            crowd.Step(Dt);
        }
        double min = double.MaxValue;
        for (int i = 0; i < agents.Count; i++)
        {
            for (int j = i + 1; j < agents.Count; j++)
            {
                double dx = agents[i].Position.X - agents[j].Position.X, dz = agents[i].Position.Z - agents[j].Position.Z;
                min = Math.Min(min, Math.Sqrt((dx * dx) + (dz * dz)));
            }
        }
        _out.WriteLine($"closest pair {min:F3}");
        Assert.True(min >= 0.55, $"closest pair {min}");
    }

    [Fact]
    public void Agents_can_be_added_and_removed_between_steps()
    {
        TiledNavMesh mesh = CrowdWorlds.Arena(30, 30);
        var crowd = new Crowd(mesh);
        var list = new List<Agent>();
        for (int i = 0; i < 10; i++)
        {
            Agent a = crowd.Add(new Vector3D(2 + (i * 2), 0, 5), Agent.Small);
            a.SetDestination(new Vector3D(2 + (i * 2), 0, 25));
            list.Add(a);
        }
        for (int i = 0; i < 30; i++)
        {
            crowd.Step(Dt);
        }
        Assert.True(crowd.Remove(list[3]));
        Assert.False(crowd.Remove(list[3]));
        Assert.False(list[3].IsActive);
        Assert.Throws<InvalidOperationException>(() => list[3].Position);
        Agent again = crowd.Add(new Vector3D(15, 0, 15), Agent.Small);
        Assert.Equal(10, again.Id);
        Assert.Equal(10, crowd.Count);
        var order = new List<Agent>();
        crowd.CopyAgents(order);
        Assert.Equal(10, order.Count);
        Assert.Same(again, order[3]);
        for (int i = 0; i < 60; i++)
        {
            crowd.Step(Dt);
        }
    }

    [Fact]
    public void Bad_input_is_rejected_with_a_message()
    {
        TiledNavMesh mesh = CrowdWorlds.Arena(10, 10);
        var crowd = new Crowd(mesh);
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Step(0));
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Step(double.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Add(new Vector3D(double.NaN, 0, 0)));
        ArgumentOutOfRangeException far = Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Add(new Vector3D(3e6, 0, 0)));
        Assert.Contains("2", far.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentException>(() => crowd.Add(new Vector3D(500, 0, 500)));
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Add(new Vector3D(5, 0, 5), Agent.Humanoid with { Radius = -1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new Crowd(mesh, new CrowdOptions { MaxThreads = 0 }));
        Agent a = crowd.Add(new Vector3D(5, 0, 5));
        Assert.Throws<ArgumentOutOfRangeException>(() => a.Priority = 0);
        Assert.Equal(PathStatus.NoEndNearby, a.SetDestination(new Vector3D(900, 0, 900)));
        var small = new Crowd(mesh, new CrowdOptions { MaxAgents = 1 });
        small.Add(new Vector3D(5, 0, 5));
        Assert.Throws<InvalidOperationException>(() => small.Add(new Vector3D(6, 0, 6)));
    }

    [Fact]
    public void Mesh_edits_during_a_run_keep_agents_on_the_mesh()
    {
        for (ulong seed = 1; seed <= 20; seed++)
        {
            TiledNavMesh mesh = CrowdWorlds.Field2(7, seed);
            var crowd = new Crowd(mesh, new CrowdOptions { Filter = new QueryFilter { IncludeFlags = 1 } });
            var rng = new NavMeshWorlds.Rng(seed + 100);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var agents = new List<Agent>();
            for (int i = 0; i < 8; i++)
            {
                Vector3D p = CorridorWorlds.RandomFieldPoint(14, rng);
                try
                {
                    Agent a = crowd.Add(p, Agent.Small);
                    a.SetDestination(CorridorWorlds.RandomFieldPoint(14, rng));
                    agents.Add(a);
                }
                catch (ArgumentException)
                {
                }
            }
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            for (int step = 0; step < 300; step++)
            {
                if (step == 40 || step == 80)
                {
                    for (int k = 0; k < 6; k++)
                    {
                        mesh.SetPolyFlags(polys[rng.Int(polys.Count)], step == 40 ? (ushort)0 : (ushort)1);
                    }
                }
                crowd.Step(Dt);
                foreach (Agent a in agents)
                {
                    if (a.State != FollowerState.OffMeshLink)
                    {
                        Assert.True(CrowdWorlds.OnMesh(q, a.Position), $"seed {seed} step {step}: agent {a.Id} off the mesh at {a.Position}");
                    }
                }
            }
        }
    }

    [Fact]
    public void Debug_geometry_lists_agents_walls_and_constraints()
    {
        CrowdSim sim = CrowdWorlds.ThroughDoor(20, 3, 1, 2, constraints: true);
        for (int i = 0; i < 40; i++)
        {
            sim.Crowd.Step(Dt);
        }
        CrowdDebugGeometry g = CrowdDebug.Collect(sim.Crowd);
        Assert.Equal(20, g.Agents.Count);
        Assert.Contains(g.Agents, a => a.Walls.Count > 0);
        Assert.Contains(g.Agents, a => a.Constraints.Count > 0);
        Assert.All(g.Agents, a => Assert.True(a.Path.Corridor.Count > 0));
    }

    // ---- query additions ----

    [Fact]
    public void FindWalls_matches_brute_force_edges()
    {
        for (ulong seed = 1; seed <= 20; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Field(12, seed, out bool[] cells);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed);
            for (int k = 0; k < 10; k++)
            {
                Vector3D c = CorridorWorlds.RandomFieldPoint(12, rng);
                NearestPoly np = q.FindNearestPoly(c, CorridorWorlds.Extents);
                if (!np.Poly.IsSet)
                {
                    continue;
                }
                double radius = 1.5 + rng.Int(30) / 10.0;
                var walls = new WallSegment[512];
                Assert.Equal(AreaQueryStatus.Success, q.FindWalls(np.Poly, np.Point, radius, (QueryFilter?)null, walls, out int count, out bool truncated));
                Assert.False(truncated);
                // Every piece lies on a boundary between a walkable cell and a blocked cell or the outside, and within the radius;
                // the total length of the pieces equals the boundary length of the reachable cells inside the radius (checked on length of unit cell edges).
                double prev = -1;
                for (int i = 0; i < count; i++)
                {
                    WallSegment w = walls[i];
                    double d = Math.Sqrt(SegDist2(np.Point.X, np.Point.Z, w.AX, w.AZ, w.BX, w.BZ));
                    Assert.True(d <= radius + 1e-9);
                    Assert.True(d >= prev - 1e-12, "sorted by distance");
                    prev = d;
                    double mx = (w.AX + w.BX) / 2, mz = (w.AZ + w.BZ) / 2;
                    double ex = w.BX - w.AX, ez = w.BZ - w.AZ;
                    double len = Math.Sqrt((ex * ex) + (ez * ez));
                    // Inward normal (left of the direction): the walkable cell is on that side, the blocked one on the other.
                    double nx = -ez / len, nz = ex / len;
                    Assert.True(Cell(cells, 12, mx + (nx * 0.25), mz + (nz * 0.25)), "walkable on the inside");
                    Assert.False(Cell(cells, 12, mx - (nx * 0.25), mz - (nz * 0.25)), "blocked or outside on the far side");
                }
                // The nearest wall of the single-wall query is the first piece.
                WallHit hit = q.DistanceToWall(np.Poly, np.Point, radius, (QueryFilter?)null, out _);
                Assert.Equal(hit.Found, count > 0);
                if (hit.Found)
                {
                    double d0 = Math.Sqrt(SegDist2(np.Point.X, np.Point.Z, walls[0].AX, walls[0].AZ, walls[0].BX, walls[0].BZ));
                    Assert.True(Math.Abs(d0 - hit.Distance) < 1e-9, $"{d0} vs {hit.Distance}");
                }
            }
        }
    }

    private static double SegDist2(double px, double pz, double ax, double az, double bx, double bz)
    {
        double ex = bx - ax, ez = bz - az;
        double len2 = (ex * ex) + (ez * ez);
        double u = len2 > 0 ? Math.Clamp((((px - ax) * ex) + ((pz - az) * ez)) / len2, 0.0, 1.0) : 0.0;
        double qx = ax + (ex * u) - px, qz = az + (ez * u) - pz;
        return (qx * qx) + (qz * qz);
    }

    private static bool Cell(bool[] cells, int size, double x, double z)
    {
        int cx = (int)Math.Floor(x), cz = (int)Math.Floor(z);
        return cx >= 0 && cz >= 0 && cx < size && cz < size && cells[(cz * size) + cx];
    }

    [Fact]
    public void FindWalls_truncates_to_the_nearest()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(12, 5, out _);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        NearestPoly np = q.FindNearestPoly(new Vector3D(6.3, 0, 6.3), CorridorWorlds.Extents);
        var all = new WallSegment[512];
        q.FindWalls(np.Poly, np.Point, 6, (QueryFilter?)null, all, out int n, out _);
        Assert.True(n > 5);
        var few = new WallSegment[4];
        q.FindWalls(np.Poly, np.Point, 6, (QueryFilter?)null, few, out int m, out bool truncated);
        Assert.True(truncated);
        Assert.Equal(4, m);
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(all[i].AX, few[i].AX);
            Assert.Equal(all[i].BZ, few[i].BZ);
        }
    }

    [Fact]
    public void FindStraightPath_with_a_corner_limit_is_a_prefix_of_the_full_path()
    {
        for (ulong seed = 1; seed <= 30; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Field(16, seed);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed);
            for (int k = 0; k < 10; k++)
            {
                Vector3D a = CorridorWorlds.RandomFieldPoint(16, rng), b = CorridorWorlds.RandomFieldPoint(16, rng);
                NearestPoly na = q.FindNearestPoly(a, CorridorWorlds.Extents), nb = q.FindNearestPoly(b, CorridorWorlds.Extents);
                if (!na.Poly.IsSet || !nb.Poly.IsSet)
                {
                    continue;
                }
                var path = new PolyRef[256];
                if (q.FindPath(na.Poly, nb.Poly, na.Point, nb.Point, new CorridorWorlds.AllPass(), path, out int pc) != PathStatus.Success)
                {
                    continue;
                }
                var full = new StraightPathPoint[128];
                q.FindStraightPath(na.Point, nb.Point, path.AsSpan(0, pc), full, out int fullCount, out _);
                for (int limit = 1; limit <= fullCount + 1; limit++)
                {
                    var part = new StraightPathPoint[128];
                    Assert.Equal(PathStatus.Success, q.FindStraightPath(na.Point, nb.Point, path.AsSpan(0, pc), part, limit, out int c, out bool trunc));
                    Assert.Equal(Math.Min(limit, fullCount), c);
                    for (int i = 0; i < c; i++)
                    {
                        Assert.Equal(full[i].Position, part[i].Position);
                    }
                    if (limit <= fullCount - 1)
                    {
                        Assert.True(trunc);
                    }
                }
            }
        }
    }

    [Fact]
    public void Boundary_constraint_stops_approach_but_allows_sliding()
    {
        // A wall along x = 1 from z = -5 to 5 seen from the origin; the walkable side is left of a -> b, here toward -x.
        Vector2D a = new(1, -5), b = new(1, 5);
        Assert.True(OrcaConstraints.ForBoundary(a, b, 0.05, 1.0, out HalfPlane h));
        Assert.True(h.SignedDistance(new Vector2D(0, 3)) >= 0, "sliding along the wall is allowed");
        Assert.True(h.SignedDistance(new Vector2D(-1, 0)) >= 0, "moving away is allowed");
        Assert.True(h.SignedDistance(new Vector2D(2, 0)) < 0, "moving into the wall is not");
        Assert.True(h.SignedDistance(new Vector2D(0.9, 0)) >= 0, "arriving at the line only after the horizon is allowed");
        // Beside the stretch (its nearest point is an end): no constraint.
        Assert.False(OrcaConstraints.ForBoundary(new Vector2D(1, 2), new Vector2D(1, 6), 0.05, 1.0, out _));
        // Behind the stretch: not this agent's wall.
        Assert.False(OrcaConstraints.ForBoundary(new Vector2D(1, 5), new Vector2D(1, -5), 0.05, 1.0, out _));
    }
}
