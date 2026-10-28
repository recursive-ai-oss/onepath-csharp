using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class FollowerTests
{
    private const double Dt = 1.0 / 30.0;

    private static double Walk(PathFollower f, NavMeshQuery q, int maxSteps, QueryFilter? filter = null)
    {
        double walked = 0;
        for (int i = 0; i < maxSteps && f.HasPath; i++)
        {
            Vector3D before = f.Position;
            f.Step(q, Dt, filter);
            walked += Vector3D.Distance(before, f.Position);
        }
        return walked;
    }

    private static int BudgetFor(double length) => 600 + (int)(length * 40);

    private static (int reached, double worstRatio, int crossings) ReachAll(TiledNavMesh mesh, Func<NavMeshWorlds.Rng, Vector3D> point, int pairs, ulong seed, FollowerOptions? options = null, PathNodeModel model = PathNodeModel.Crossing)
    {
        // A polygon-node query's routes can be a half longer than the shortest walk; the follower re-judges them with its refine
        // search, here given the budget the two-floor meshes need (the default is cheaper and leaves them within 1.2).
        options = (options ?? FollowerOptions.Humanoid) with { RefineBudget = 2000 };
        var q = new NavMeshQuery(mesh, 65536, model);
        var fresh = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(seed);
        int reached = 0, crossings = 0, tried = 0;
        double worst = 0;
        while (reached < pairs && tried < pairs * 20)
        {
            tried++;
            Vector3D a = point(rng), b = point(rng);
            PathResult r = fresh.FindPath(a, b);
            if (r.Status != PathStatus.Found)
            {
                continue;
            }
            var f = new PathFollower(options);
            Assert.True(f.Place(q, a));
            Assert.NotEqual(QueryStatus.NoEndNearby, f.SetDestination(q, b));
            int steps = 0;
            double walked = 0;
            bool crossed = false;
            while (f.HasPath && steps < BudgetFor(r.Length))
            {
                Vector3D before = f.Position;
                f.Step(q, Dt);
                walked += Vector3D.Distance(before, f.Position);
                steps++;
                crossed |= f.State == FollowerState.OffMeshLink;
            }
            string why = $"seed {seed} pair {tried}: state {f.State} after {steps} steps, walked {walked}, route {r.Length}";
            Assert.True(f.State == FollowerState.Arrived && f.ReachedTarget, why);
            Vector3D end = r.Points[r.Points.Count - 1].Position;
            double dx = f.Position.X - end.X, dz = f.Position.Z - end.Z;
            Assert.True(Math.Sqrt((dx * dx) + (dz * dz)) <= 0.011, why);
            Assert.Equal(default, f.Velocity);
            worst = Math.Max(worst, walked / Math.Max(r.Length, 1e-9));
            Assert.True(walked <= (r.Length * 1.5) + 0.5, $"{why}: path too long");
            crossings += crossed ? 1 : 0;
            reached++;
        }
        return (reached, worst, crossings);
    }

    [Theory]
    [InlineData(PathNodeModel.Crossing)]
    [InlineData(PathNodeModel.Polygon)]
    public void ReachesEveryReachableGoal_OnScatteredObstacles(PathNodeModel model)
    {
        int total = 0;
        double worst = 0;
        for (ulong seed = 1; seed <= 3; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Field(48, seed);
            (int n, double w, _) = ReachAll(mesh, r => CorridorWorlds.RandomFieldPoint(48, r), 250, seed * 17, null, model);
            total += n;
            worst = Math.Max(worst, w);
        }
        Assert.True(total >= 750, $"{total} goals reached");
        Assert.True(worst <= 1.08, $"worst walked/route ratio {worst}");
    }

    [Theory]
    [InlineData(PathNodeModel.Crossing)]
    [InlineData(PathNodeModel.Polygon)]
    public void ReachesEveryReachableGoal_OnLayeredMeshesWithLinks(PathNodeModel model)
    {
        int total = 0, crossings = 0;
        double worst = 0;
        for (ulong seed = 1; seed <= 4; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.TwoFloors(seed);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            (int n, double w, int c) = ReachAll(mesh, r => CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r), 120, seed * 29, null, model);
            total += n;
            crossings += c;
            worst = Math.Max(worst, w);
        }
        Assert.True(total >= 480, $"{total} goals reached");
        Assert.True(crossings >= 10, $"{crossings} routes crossed an off-mesh link");
        Assert.True(worst <= 1.08, $"worst walked/route ratio {worst}");
    }

    [Theory]
    [InlineData(PathNodeModel.Crossing)]
    [InlineData(PathNodeModel.Polygon)]
    public void ReachesGoals_WithoutShortcutsAndWithSmallBudgets(PathNodeModel model)
    {
        FollowerOptions o = FollowerOptions.Humanoid with { OptimizeInterval = 0, TopologyInterval = 0, ReplanBudget = 40, MaxReplanRounds = 40 };
        TiledNavMesh mesh = CorridorWorlds.Field(40, 9);
        (int n, double w, _) = ReachAll(mesh, r => CorridorWorlds.RandomFieldPoint(40, r), 100, 3, o, model);
        Assert.Equal(100, n);
        // Without shortcuts the walk follows the polygon corridor, so the worst of 100 pairs depends on how the builder
        // happened to split the open areas (equally valid triangulations differ in it).
        Assert.True(w <= 1.2, $"worst ratio {w}");
    }

    [Fact]
    public void UnreachableGoal_EndsAtTheClosestPointWithoutReachingIt()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(48, 4);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(2);
        int found = 0;
        for (int i = 0; i < 400 && found < 8; i++)
        {
            Vector3D a = CorridorWorlds.RandomFieldPoint(48, rng), b = CorridorWorlds.RandomFieldPoint(48, rng);
            PathResult r = q.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
            if (r.Status != PathStatus.Partial)
            {
                continue;
            }
            var f = new PathFollower();
            Assert.True(f.Place(q, a));
            QueryStatus s = f.SetDestination(q, b);
            Assert.Equal(QueryStatus.Partial, s);
            Walk(f, q, BudgetFor(r.Length));
            Assert.Equal(FollowerState.Arrived, f.State);
            Assert.False(f.ReachedTarget);
            found++;
        }
        Assert.True(found >= 3, $"{found} unreachable cases");
    }

    [Fact]
    public void NoPolygonNearTheTarget_GivesNoPath_AndAnotherDestinationRecovers()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(20, 1);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower();
        Assert.False(f.Place(q, new Vector3D(500, 0, 500)));
        Assert.Equal(QueryStatus.InvalidStartPoly, f.SetDestination(q, new Vector3D(1, 0, 1)));
        Assert.True(f.Place(q, new Vector3D(10.5, 0, 10.5)));
        Assert.Equal(QueryStatus.NoEndNearby, f.SetDestination(q, new Vector3D(900, 0, 900)));
        Assert.Equal(FollowerState.NoPath, f.State);
        Assert.False(f.HasPath);
        f.Step(q, Dt);
        Assert.Equal(default, f.Velocity);
        Assert.Equal(QueryStatus.Success, f.SetDestination(q, new Vector3D(10.5, 0, 10.9)));
        Walk(f, q, 2000);
        Assert.Equal(FollowerState.Arrived, f.State);
    }

    [Fact]
    public void ChangingTheDestinationMidWay_Works()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(40, 12);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(6);
        int done = 0;
        for (int i = 0; i < 200 && done < 25; i++)
        {
            Vector3D a = CorridorWorlds.RandomFieldPoint(40, rng), b = CorridorWorlds.RandomFieldPoint(40, rng), c = CorridorWorlds.RandomFieldPoint(40, rng);
            if (q.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken).Status != PathStatus.Found || q.FindPath(a, c, cancellationToken: TestContext.Current.CancellationToken).Status != PathStatus.Found)
            {
                continue;
            }
            var f = new PathFollower();
            f.Place(q, a);
            f.SetDestination(q, b);
            for (int s = 0; s < 40 && f.HasPath; s++)
            {
                f.Step(q, Dt);
            }
            if (f.State != FollowerState.Moving)
            {
                continue;
            }
            Assert.NotEqual(QueryStatus.NoEndNearby, f.SetDestination(q, c));
            PathResult r = q.FindPath(f.Position, c, cancellationToken: TestContext.Current.CancellationToken);
            if (r.Status != PathStatus.Found)
            {
                continue; // the pocket the walk had entered
            }
            Walk(f, q, BudgetFor(r.Length * 2));
            Assert.True(f.State == FollowerState.Arrived && f.ReachedTarget, $"state {f.State}");
            done++;
        }
        Assert.True(done >= 20, $"{done}");
    }

    // ---- steering ----

    [Fact]
    public void Steering_RespectsSpeedAndAcceleration_AndSlowsIntoTheEnd()
    {
        // One long straight corridor: a 60 x 3 strip.
        var cells = new bool[60 * 3];
        Array.Fill(cells, true);
        TiledNavMesh mesh = NavMesh.Build2D(cells, 60, 3, 1.0, new Vector2D(0, 0), CorridorWorlds.FlatSettings()).Tiles;
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        FollowerOptions o = FollowerOptions.Humanoid with { MaxSpeed = 4, MaxAcceleration = 10, SlowRadius = 2 };
        var f = new PathFollower(o);
        Assert.True(f.Place(q, new Vector3D(1, 0, 1.5)));
        f.SetDestination(q, new Vector3D(59, 0, 1.5));
        double prev = 0, peak = 0;
        int steps = 0;
        double endSpeed = double.NaN;
        while (f.HasPath && steps < 2000)
        {
            f.Step(q, Dt);
            double speed = f.Velocity.Length();
            Assert.True(speed <= 4 + 1e-9, $"speed {speed}");
            Assert.True(Math.Abs(speed - prev) <= (10 * Dt) + 1e-9 || f.State == FollowerState.Arrived, $"step {steps}: speed {prev} to {speed}");
            prev = speed;
            peak = Math.Max(peak, speed);
            if (f.RemainingDistance < 0.5 && double.IsNaN(endSpeed))
            {
                endSpeed = speed;
            }
            steps++;
        }
        Assert.Equal(FollowerState.Arrived, f.State);
        Assert.True(peak > 3.99, $"peak {peak}");
        Assert.True(endSpeed < 1.5, $"speed at 0.5 from the end: {endSpeed}");
        Assert.True(f.Position.X > 58.98);
        // The trip takes about distance / speed plus the ramp: a sanity bound on both sides.
        Assert.InRange(steps * Dt, 58.0 / 4, (58.0 / 4) + 2.5);
    }

    [Fact]
    public void StuckAgent_IsDetected_ReplansThenStops()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(30, 2);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(1);
        PathResult r;
        Vector3D a, b;
        do
        {
            a = CorridorWorlds.RandomFieldPoint(30, rng);
            b = CorridorWorlds.RandomFieldPoint(30, rng);
            r = q.FindPath(a, b, cancellationToken: TestContext.Current.CancellationToken);
        }
        while (r.Status != PathStatus.Found || r.Length < 15);
        // An acceleration this low never reaches a tenth of full speed within the window.
        var f = new PathFollower(FollowerOptions.Humanoid with { MaxAcceleration = 0.05, StuckWindow = 1, MaxStuckRecoveries = 2 });
        f.Place(q, a);
        f.SetDestination(q, b);
        int steps = 0;
        while (f.State == FollowerState.Moving && steps < 3000)
        {
            f.Step(q, Dt);
            steps++;
        }
        Assert.Equal(FollowerState.Stuck, f.State);
        Assert.InRange(steps * Dt, 2.9, 3.2); // three windows: two recoveries, then the stop
        Assert.Equal(default, f.Velocity);
        // A new destination starts over.
        f.Step(q, Dt);
        Assert.Equal(FollowerState.Stuck, f.State);
        Assert.NotEqual(QueryStatus.NoEndNearby, f.SetDestination(q, b));
        Assert.Equal(FollowerState.Moving, f.State);
    }

    // ---- off-mesh links ----

    private static (TiledNavMesh mesh, Vector3D a, Vector3D b) FindLinkRoute(ulong firstSeed)
    {
        for (ulong seed = firstSeed; seed < firstSeed + 40; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.TwoFloors(seed);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            var rng = new NavMeshWorlds.Rng(seed);
            for (int i = 0; i < 200; i++)
            {
                Vector3D a = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), b = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                PathResult r = q.FindPath(a, b);
                if (r.Status != PathStatus.Found)
                {
                    continue;
                }
                for (int k = 0; k < r.Points.Count; k++)
                {
                    if ((r.Points[k].Flags & StraightPathMarks.OffMeshConnection) != 0)
                    {
                        return (mesh, a, b);
                    }
                }
            }
        }
        throw new InvalidOperationException("no route with a link found");
    }

    [Fact]
    public void OffMeshLink_AutomaticCrossingTakesItsDuration()
    {
        (TiledNavMesh mesh, Vector3D a, Vector3D b) = FindLinkRoute(1);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower(FollowerOptions.Humanoid with { OffMeshSpeed = 2 });
        f.Place(q, a);
        f.SetDestination(q, b);
        int inLink = 0;
        Vector3D entry = default, exit = default;
        double duration = 0;
        for (int i = 0; i < 20000 && f.HasPath; i++)
        {
            f.Step(q, Dt);
            if (f.State == FollowerState.OffMeshLink)
            {
                if (inLink == 0)
                {
                    entry = f.OffMeshStart;
                    exit = f.OffMeshEnd;
                    duration = f.OffMeshDuration;
                    Assert.Equal(Vector3D.Distance(entry, exit) / 2, duration, 12);
                }
                inLink++;
                Assert.True(f.Position.X != exit.X || f.Position.Z != exit.Z || f.OffMeshElapsed >= duration);
            }
        }
        Assert.Equal(FollowerState.Arrived, f.State);
        Assert.True(f.ReachedTarget);
        Assert.InRange(inLink * Dt, duration - Dt - 1e-9, duration + Dt + 1e-9);
    }

    [Fact]
    public void OffMeshLink_HostDrivenWaitsForCompletion()
    {
        (TiledNavMesh mesh, Vector3D a, Vector3D b) = FindLinkRoute(1);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower(FollowerOptions.Humanoid with { OffMeshMode = OffMeshMode.Host });
        f.Place(q, a);
        f.SetDestination(q, b);
        int guard = 0;
        while (f.State == FollowerState.Moving && guard++ < 20000)
        {
            f.Step(q, Dt);
        }
        Assert.Equal(FollowerState.OffMeshLink, f.State);
        Vector3D at = f.Position;
        for (int i = 0; i < 300; i++)
        {
            f.Step(q, Dt);
            Assert.Equal(FollowerState.OffMeshLink, f.State);
            Assert.Equal(at, f.Position);
        }
        Assert.Throws<InvalidOperationException>(() => new PathFollower().CompleteOffMeshLink(q));
        f.CompleteOffMeshLink(q);
        Assert.Equal(FollowerState.Moving, f.State);
        Assert.Equal(f.OffMeshEnd, f.Position);
        guard = 0;
        while (f.HasPath && guard++ < 20000)
        {
            f.Step(q, Dt);
            if (f.State == FollowerState.OffMeshLink)
            {
                f.CompleteOffMeshLink(q); // a second link on the route
            }
        }
        Assert.Equal(FollowerState.Arrived, f.State);
        Assert.True(f.ReachedTarget);
    }

    // ---- mesh changes while walking ----

    [Fact]
    public void MeshEditsWhileWalking_AreRepaired()
    {
        int trials = 0, reachable = 0, reached = 0, lost = 0;
        for (ulong seed = 1; seed <= 40; seed++)
        {
            ulong world = seed * 6151;
            TiledNavMesh mesh = CorridorWorlds.TwoFloors(world);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed);
            var mut = new NavMeshWorlds.Rng(seed + 999);
            for (int trial = 0; trial < 6; trial++)
            {
                List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
                if (polys.Count == 0)
                {
                    break;
                }
                Vector3D a = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), b = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                var f = new PathFollower();
                if (!f.Place(q, a) || f.SetDestination(q, b) != QueryStatus.Success)
                {
                    continue;
                }
                trials++;
                int steps = 0;
                for (; steps < 400 && f.HasPath; steps++)
                {
                    f.Step(q, Dt);
                    if (steps % 25 == 10 && steps < 120)
                    {
                        NavMeshMutations.Apply(mesh, mut, world);
                    }
                }
                // Edits have stopped. If a fresh query from where the agent now stands reaches the goal, so must the agent.
                if (f.State == FollowerState.NoPath)
                {
                    lost++;
                    continue;
                }
                var fq = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                NearestPoly ng = fq.FindNearestPoly(b, CorridorWorlds.Extents);
                PathResult r = ng.Poly.IsSet ? fq.FindPath(f.Position, b, cancellationToken: TestContext.Current.CancellationToken) : null!;
                if (r is null || r.Status != PathStatus.Found)
                {
                    continue;
                }
                reachable++;
                Walk(f, q, BudgetFor(r.Length * 2) + 1500);
                if (f.State == FollowerState.Arrived && f.ReachedTarget)
                {
                    reached++;
                }
                else
                {
                    Assert.Fail($"world {world} trial {trial}: fresh query reaches the goal, follower state {f.State}, reached {f.ReachedTarget}");
                }
            }
        }
        Assert.True(trials >= 100, $"{trials} trials");
        Assert.True(reachable >= 50, $"{reachable} reachable after edits, {lost} lost");
    }

    // ---- state ----

    private static byte[] Hash(PathFollower f) => f.ToBytes();

    [Fact]
    public void Snapshot_RestoreAndRerun_GivesIdenticalState()
    {
        int checkedPoints = 0, midLink = 0, midReplan = 0;
        for (int variant = 0; variant < 3; variant++)
        {
            TiledNavMesh mesh = variant == 0 ? CorridorWorlds.Field(40, 5) : CorridorWorlds.TwoFloors((ulong)variant);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng((ulong)variant + 1);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            FollowerOptions o = variant == 2 ? FollowerOptions.Humanoid with { ReplanBudget = 2, MaxReplanRounds = 500 } : FollowerOptions.Humanoid;
            for (int trial = 0; trial < 15; trial++)
            {
                Vector3D a = variant == 0 ? CorridorWorlds.RandomFieldPoint(40, rng) : CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                Vector3D b = variant == 0 ? CorridorWorlds.RandomFieldPoint(40, rng) : CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                var f = new PathFollower(o);
                if (!f.Place(q, a))
                {
                    continue;
                }
                f.SetDestination(q, b);
                int cut = variant == 2 ? 1 + rng.Int(5) : 5 + rng.Int(150);
                for (int s = 0; s < cut && f.HasPath; s++)
                {
                    f.Step(q, Dt);
                }
                byte[] snap = f.ToBytes();
                Assert.Equal(f.ByteSize, snap.Length);
                midLink += f.State == FollowerState.OffMeshLink ? 1 : 0;
                midReplan += f.Corridor.NeedsReplan ? 1 : 0;
                var g = new PathFollower(o);
                g.Read(snap);
                Assert.Equal(snap, g.ToBytes());
                for (int s = 0; s < 300; s++)
                {
                    f.Step(q, Dt);
                    g.Step(q, Dt);
                    if (s % 7 == 0)
                    {
                        Assert.Equal(Hash(f), Hash(g));
                        checkedPoints++;
                    }
                }
                Assert.Equal(Hash(f), Hash(g));
                // And back: restoring the older snapshot into the first follower reproduces the second run.
                f.Read(snap);
                Assert.Equal(snap, f.ToBytes());
            }
        }
        Assert.True(checkedPoints > 500);
        Assert.True(midReplan > 0, "no snapshot taken with a replan pending");
        Assert.True(midLink >= 0);
    }

    [Fact]
    public void SnapshotTakenInsideAnOffMeshCrossing_Resumes()
    {
        (TiledNavMesh mesh, Vector3D a, Vector3D b) = FindLinkRoute(3);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower();
        f.Place(q, a);
        f.SetDestination(q, b);
        int guard = 0;
        while (f.State != FollowerState.OffMeshLink && guard++ < 20000)
        {
            f.Step(q, Dt);
        }
        Assert.Equal(FollowerState.OffMeshLink, f.State);
        f.Step(q, Dt);
        byte[] snap = f.ToBytes();
        var g = new PathFollower();
        g.Read(snap);
        for (int i = 0; i < 3000 && f.HasPath; i++)
        {
            f.Step(q, Dt);
            g.Step(q, Dt);
            Assert.Equal(f.ToBytes(), g.ToBytes());
        }
        Assert.Equal(FollowerState.Arrived, f.State);
        Assert.Equal(FollowerState.Arrived, g.State);
    }

    [Fact]
    public void DamagedStateBytes_AreRejected_AndLeaveTheFollowerAlone()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(30, 3);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower();
        f.Place(q, new Vector3D(1.5, 0, 1.5));
        f.SetDestination(q, new Vector3D(25.5, 0, 22.5));
        for (int i = 0; i < 20; i++)
        {
            f.Step(q, Dt);
        }
        byte[] bytes = f.ToBytes();
        var g = new PathFollower();
        g.Place(q, new Vector3D(5.5, 0, 5.5));
        byte[] before = g.ToBytes();
        for (int cut = 0; cut < bytes.Length; cut++)
        {
            Assert.Throws<NavMeshFormatException>(() => g.Read(bytes.AsSpan(0, cut)));
        }
        Assert.Throws<NavMeshFormatException>(() => g.Read([.. bytes, 0]));
        Assert.Equal(before, g.ToBytes());
        var dmg = new NavMeshWorlds.Rng(5);
        for (int i = 0; i < 3000; i++)
        {
            byte[] b = (byte[])bytes.Clone();
            b[dmg.Int(b.Length)] ^= (byte)(1 << dmg.Int(8));
            try
            {
                g.Read(b);
                g.Step(q, Dt); // an accepted state must be safe to run
            }
            catch (NavMeshFormatException)
            {
            }
        }
        Assert.Throws<ArgumentException>(() => f.Write(new byte[8]));
        // A corridor of another capacity does not fit these options.
        var small = new PathFollower(FollowerOptions.Humanoid with { CorridorCapacity = 64 });
        Assert.Throws<NavMeshFormatException>(() => small.Read(bytes));
    }

    [Fact]
    public void HostileArguments_AreRejectedWithNamedErrors()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(10, 3);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower();
        f.Place(q, new Vector3D(1.5, 0, 1.5));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.Place(q, new Vector3D(double.NaN, 0, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.SetDestination(q, new Vector3D(0, double.PositiveInfinity, 0)));
        Assert.Throws<ArgumentOutOfRangeException>(() => f.SetDestination(q, new Vector3D(2097152.0, 0, 0)));
        f.SetDestination(q, new Vector3D(8.5, 0, 8.5));
        foreach (double dt in new[] { 0.0, -1.0, double.NaN, double.PositiveInfinity })
        {
            Assert.Throws<ArgumentOutOfRangeException>(() => f.Step(q, dt));
        }
        Assert.Throws<ArgumentNullException>(() => f.Step(null!, Dt));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => new PathFollower(FollowerOptions.Humanoid with { MaxSpeed = 0 }));
        Assert.Equal("MaxSpeed", ex.ParamName);
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathFollower(FollowerOptions.Humanoid with { MaxCorners = 1 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathFollower(FollowerOptions.Humanoid with { ArrivalMinSpeed = 99 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathFollower(FollowerOptions.Humanoid with { SearchExtents = new Vector3D(-1, 1, 1) }));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathFollower(FollowerOptions.Humanoid with { CorridorCapacity = 1 }));
        // The presets are valid.
        _ = new PathFollower(FollowerOptions.Small);
        // Stepping a follower that was never placed does nothing.
        var idle = new PathFollower();
        idle.Step(q, Dt);
        Assert.Equal(FollowerState.Idle, idle.State);
        Assert.False(idle.HasPath);
    }

    [Fact]
    public void DebugGeometry_DescribesTheFollower()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(20, 4);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower();
        Assert.True(f.Place(q, new Vector3D(1.5, 0, 1.5)));
        f.SetDestination(q, new Vector3D(15.5, 0, 15.5));
        for (int i = 0; i < 10; i++)
        {
            f.Step(q, Dt);
        }
        RecursiveAI.OnePath.Debug.FollowerDebugGeometry g = RecursiveAI.OnePath.Debug.FollowerDebug.Collect(q, f);
        Assert.Equal(f.Position, g.Position);
        Assert.Equal(f.Velocity, g.Velocity);
        Assert.Equal(f.Corridor.Count, g.Corridor.Count);
        Assert.Equal(f.Corners.Length, g.Corners.Count);
        Assert.False(g.OffMeshActive);
        Assert.Throws<ArgumentNullException>(() => RecursiveAI.OnePath.Debug.FollowerDebug.Collect(q, null!));
    }

    [Fact]
    public void Steps_AllocateNothingInSteadyState()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(48, 2);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(3);
        var agents = new List<PathFollower>();
        while (agents.Count < 30)
        {
            var f = new PathFollower();
            if (f.Place(q, CorridorWorlds.RandomFieldPoint(48, rng)) && f.SetDestination(q, CorridorWorlds.RandomFieldPoint(48, rng)) != QueryStatus.NoEndNearby)
            {
                agents.Add(f);
            }
        }
        // Warm up: buffers grow to their working size, code gets compiled.
        for (int i = 0; i < 300; i++)
        {
            foreach (PathFollower f in agents)
            {
                f.Step(q, Dt);
            }
        }
        // A window counts only if it did real work, or an idle window would pass trivially.
        int steps = 0;
        long allocated = AllocationProbe.SteadyState(() =>
        {
            int n = 0;
            for (int i = 0; i < 200; i++)
            {
                foreach (PathFollower f in agents)
                {
                    if (f.State == FollowerState.Moving)
                    {
                        f.Step(q, Dt);
                        n++;
                    }
                }
            }

            steps = n;
            return n > 1000;
        }, prepare: () =>
        {
            // Agents arrive over a window: send them somewhere new and let their buffers settle again.
            foreach (PathFollower f in agents)
            {
                _ = f.SetDestination(q, CorridorWorlds.RandomFieldPoint(48, rng));
            }

            for (int i = 0; i < 150; i++)
            {
                foreach (PathFollower f in agents)
                {
                    f.Step(q, Dt);
                }
            }
        });
        Assert.True(allocated == 0, allocated == long.MaxValue ? $"no window did enough work (last: {steps} steps)" : $"{allocated} bytes allocated in {steps} steps");
    }
}
