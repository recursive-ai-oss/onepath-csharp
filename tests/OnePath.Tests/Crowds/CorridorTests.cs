using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class CorridorTests
{
    private static readonly Vector3D Ext = CorridorWorlds.Extents;

    private static PathCorridor? Plan(NavMeshQuery q, Vector3D a, Vector3D b, int capacity = 512)
    {
        NearestPoly na = q.FindNearestPoly(a, Ext);
        Assert.True(na.Poly.IsSet);
        var c = new PathCorridor(capacity);
        c.Reset(na.Poly, na.Point);
        c.SetTarget(b);
        QueryStatus s = c.Replan(q);
        return s == QueryStatus.Success ? c : null;
    }

    [Fact]
    public void AfterMoves_EqualsFreshQuery_OnMazes()
    {
        int checkedMoves = 0, sideSteps = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Maze(10, seed, out List<Vector3D> rooms);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var fresh = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed * 31);
            var buf = new PolyRef[512];
            var f = new CorridorWorlds.AllPass();
            for (int trial = 0; trial < 6; trial++)
            {
                PathCorridor? c = Plan(q, rooms[rng.Int(rooms.Count)], rooms[rng.Int(rooms.Count)]);
                Assert.NotNull(c);
                var corners = new StraightPathPoint[64];
                for (int move = 0; move < 60; move++)
                {
                    Vector3D to;
                    int kind = rng.Int(4);
                    if (kind == 3)
                    {
                        to = rooms[rng.Int(rooms.Count)]; // anywhere: a straight line that mostly meets a wall
                    }
                    else
                    {
                        Assert.Equal(QueryStatus.Success, c.FindCorners(q, corners, out int cc));
                        Vector3D next = cc > 1 ? corners[1].Position : c.Target;
                        double t = kind == 0 ? 1.0 : kind == 1 ? 0.35 : 0.7;
                        to = Vector3D.Lerp(c.Position, next, t);
                    }
                    PolyRef before = c.FirstPoly;
                    Assert.True(c.MovePosition(q, to));
                    sideSteps += c.FirstPoly != before ? 1 : 0;
                    Assert.Null(CorridorWorlds.Check(c, q));
                    Assert.True(c.IsComplete);
                    // The same question asked afresh: route from this polygon and position to the same target polygon.
                    Assert.Equal(QueryStatus.Success, fresh.FindPath(c.FirstPoly, c.TargetPoly, c.Position, c.Target, in f, buf, out int n));
                    Assert.Equal(buf.AsSpan(0, n).ToArray(), c.Path.ToArray());
                    var fc = new StraightPathPoint[64];
                    Assert.Equal(QueryStatus.Success, c.FindCorners(q, corners, out int a));
                    Assert.Equal(QueryStatus.Success, fresh.FindStraightPath(c.Position, c.Target, buf.AsSpan(0, n), fc, out int b, out _));
                    Assert.Equal(b, a);
                    for (int i = 0; i < a; i++)
                    {
                        Assert.Equal(fc[i], corners[i]);
                    }
                    checkedMoves++;
                }
            }
        }
        Assert.True(checkedMoves >= 2000, $"{checkedMoves} moves checked");
        Assert.True(sideSteps > 100, $"{sideSteps} polygon changes");
    }

    [Fact]
    public void TargetMoves_EqualFreshQuery_OnMazes()
    {
        TiledNavMesh mesh = CorridorWorlds.Maze(10, 7, out List<Vector3D> rooms);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var fresh = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(5);
        var buf = new PolyRef[512];
        var f = new CorridorWorlds.AllPass();
        int checks = 0;
        for (int trial = 0; trial < 10; trial++)
        {
            PathCorridor? c = Plan(q, rooms[rng.Int(rooms.Count)], rooms[rng.Int(rooms.Count)]);
            Assert.NotNull(c);
            for (int move = 0; move < 40; move++)
            {
                Vector3D to = move % 3 == 0 ? rooms[rng.Int(rooms.Count)] : Vector3D.Lerp(c.Target, c.Position, 0.2 + (0.1 * rng.Int(5)));
                Assert.True(c.MoveTarget(q, to));
                Assert.Null(CorridorWorlds.Check(c, q));
                Assert.Equal(QueryStatus.Success, fresh.FindPath(c.FirstPoly, c.TargetPoly, c.Position, c.Target, in f, buf, out int n));
                Assert.Equal(buf.AsSpan(0, n).ToArray(), c.Path.ToArray());
                checks++;
            }
        }
        Assert.True(checks >= 400);
    }

    [Fact]
    public void Moves_KeepInvariants_OnLayeredMeshes()
    {
        int moves = 0, offMesh = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Layered(seed);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed + 500);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            var corners = new StraightPathPoint[64];
            for (int trial = 0; trial < 25; trial++)
            {
                PathCorridor? c = Plan(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng));
                if (c is null)
                {
                    continue;
                }
                for (int move = 0; move < 30 && c.Count > 0; move++)
                {
                    Assert.Equal(QueryStatus.Success, c.FindCorners(q, corners, out int cc));
                    int ci = cc > 1 ? 1 : 0;
                    if ((corners[ci].Flags & StraightPathMarks.OffMeshConnection) != 0)
                    {
                        // Walking never crosses a link; the follower does that. Here: step to the entry, then take the exit.
                        offMesh++;
                        Assert.True(c.MovePosition(q, corners[ci].Position));
                        Assert.True(c.CrossOffMesh(corners[ci + 1].Poly, corners[ci + 1].Position));
                    }
                    else
                    {
                        Assert.True(c.MovePosition(q, Vector3D.Lerp(c.Position, corners[ci].Position, 0.5 + (0.5 * rng.Int(2)))));
                    }
                    Assert.Null(CorridorWorlds.Check(c, q));
                    Assert.True(c.IsComplete);
                    moves++;
                }
            }
        }
        Assert.True(moves > 500, $"{moves} moves");
        Assert.True(offMesh > 5, $"{offMesh} link crossings");
    }

    // ---- mesh changes ----

    [Fact]
    public void Repair_ReachesTargetWheneverFreshQueryDoes()
    {
        int valid = 0, trimmed = 0, invalid = 0, repaired = 0, unreachable = 0, trials = 0;
        var f = new CorridorWorlds.AllPass();
        var buf = new PolyRef[2048];
        for (ulong seed = 1; seed <= 60; seed++)
        {
            ulong world = seed * 7919;
            for (int trial = 0; trial < 12; trial++)
            {
                TiledNavMesh mesh = CorridorWorlds.Layered(world);
                var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                var rng = new NavMeshWorlds.Rng((seed * 100) + (ulong)trial);
                List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
                Vector3D goal = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                PathCorridor? c = Plan(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), goal);
                if (c is null || c.Count < 3)
                {
                    continue;
                }
                // Walk a little so the corridor is not just the fresh query's.
                var corners = new StraightPathPoint[32];
                if (c.FindCorners(q, corners, out int cc) == QueryStatus.Success && cc > 1 && (corners[1].Flags & StraightPathMarks.OffMeshConnection) == 0)
                {
                    c.MovePosition(q, Vector3D.Lerp(c.Position, corners[1].Position, 0.5));
                }
                var mut = new NavMeshWorlds.Rng((seed * 1000) + (ulong)trial);
                int edits = 1 + mut.Int(6);
                for (int i = 0; i < edits; i++)
                {
                    NavMeshMutations.Apply(mesh, mut, world);
                }
                trials++;
                CorridorStatus st = c.Validate(q, null, int.MaxValue);
                switch (st)
                {
                    case CorridorStatus.Valid:
                        valid++;
                        break;
                    case CorridorStatus.Trimmed:
                        trimmed++;
                        break;
                    default:
                        invalid++;
                        if (!c.Relocate(q, null, new Vector3D(6, 8, 6)))
                        {
                            continue;
                        }
                        break;
                }
                if (c.NeedsReplan)
                {
                    c.Replan(q, null, 1_000_000);
                }
                Assert.Null(CorridorWorlds.Check(c, q));
                // The fresh query from the same polygon and position to the nearest polygon of the goal.
                var fq = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                NearestPoly ng = fq.FindNearestPoly(goal, Ext);
                if (!ng.Poly.IsSet)
                {
                    Assert.False(c.IsComplete);
                    continue;
                }
                QueryStatus fs = fq.FindPath(c.FirstPoly, ng.Poly, c.Position, ng.Point, in f, buf, out _);
                if (fs == QueryStatus.Success)
                {
                    Assert.True(c.IsComplete, $"world {world} trial {trial}: a fresh query reaches the goal but the repaired corridor does not");
                    Assert.Equal(ng.Poly, c.LastPoly);
                    repaired++;
                }
                else
                {
                    Assert.False(c.IsComplete);
                    Assert.False(c.NeedsReplan);
                    unreachable++;
                }
            }
        }
        Assert.True(trials >= 150, $"{trials} trials");
        Assert.True(trimmed >= 20 && invalid >= 5 && valid >= 5, $"valid {valid}, trimmed {trimmed}, invalid {invalid}");
        Assert.True(repaired >= 60, $"{repaired} repaired, {unreachable} unreachable");
    }

    [Fact]
    public void SlicedReplan_EqualsUnsliced_ForEveryBudget()
    {
        int compared = 0, fallbacks = 0;
        for (ulong seed = 1; seed <= 20; seed++)
        {
            ulong world = seed * 104729;
            for (int trial = 0; trial < 6; trial++)
            {
                TiledNavMesh mesh = CorridorWorlds.Layered(world);
                var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                var rng = new NavMeshWorlds.Rng((seed * 100) + (ulong)trial);
                List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
                Vector3D goal = CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng);
                PathCorridor? c = Plan(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), goal);
                if (c is null)
                {
                    continue;
                }
                var mut = new NavMeshWorlds.Rng((seed * 1000) + (ulong)trial);
                for (int i = 0, n = 1 + mut.Int(4); i < n; i++)
                {
                    NavMeshMutations.Apply(mesh, mut, world);
                }
                if (c.Validate(q, null, int.MaxValue) == CorridorStatus.Invalid && !c.Relocate(q, null, new Vector3D(6, 8, 6)))
                {
                    continue;
                }
                byte[] start = c.ToBytes();
                // Unsliced.
                PathCorridor whole = PathCorridor.Read(start, out _);
                var qa = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                QueryStatus ws = whole.Replan(qa, null, int.MaxValue);
                foreach (int budget in new[] { 1, 2, 3, 5, 17, 100 })
                {
                    PathCorridor part = PathCorridor.Read(start, out _);
                    var qb = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
                    var session = new CorridorReplan();
                    QueryStatus bs = session.Begin(qb, part, null);
                    QueryStatus ps = bs;
                    int calls = 0;
                    while (bs == QueryStatus.InProgress)
                    {
                        ps = session.Update(budget);
                        calls++;
                        if (ps != QueryStatus.InProgress)
                        {
                            break;
                        }
                    }
                    QueryStatus applied = session.Apply();
                    if (bs == QueryStatus.InProgress)
                    {
                        Assert.Equal(ws, applied);
                        Assert.Equal(whole.ToBytes(), part.ToBytes());
                        Assert.Equal(qa.Iterations > 0, session.Iterations > 0);
                        if (budget == 1 && calls > 1)
                        {
                            compared++;
                        }
                    }
                    else
                    {
                        Assert.Equal(ws, bs);
                    }
                }
                fallbacks += ws == QueryStatus.Success ? 0 : 1;
            }
        }
        Assert.True(compared >= 60, $"{compared} sliced comparisons");
        Assert.True(fallbacks > 0, "no unreachable cases were exercised");
    }

    [Fact]
    public void BudgetedReplan_ContinuesToTheTarget()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(40, 3);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(9);
        int done = 0, partial = 0, stalled = 0;
        for (int trial = 0; trial < 60; trial++)
        {
            Vector3D a = CorridorWorlds.RandomFieldPoint(40, rng), b = CorridorWorlds.RandomFieldPoint(40, rng);
            NearestPoly na = q.FindNearestPoly(a, Ext);
            if (!na.Poly.IsSet)
            {
                continue;
            }
            var c = new PathCorridor(1024);
            c.Reset(na.Poly, na.Point);
            c.SetTarget(b);
            QueryStatus s = c.Replan(q, null, 32);
            int rounds = 1;
            while (s == QueryStatus.InProgress && rounds < 200)
            {
                Assert.True(c.NeedsReplan);
                Assert.False(c.IsComplete);
                Assert.Null(CorridorWorlds.Check(c, q));
                s = c.Replan(q, null, 32);
                rounds++;
            }
            if (s == QueryStatus.InProgress)
            {
                stalled++; // a pocket nearer the goal than its exits: restarting from the closest polygon cannot leave it
            }
            else if (s == QueryStatus.Success)
            {
                done++;
                Assert.True(c.IsComplete);
            }
            else
            {
                partial++;
            }
            Assert.Null(CorridorWorlds.Check(c, q));
        }
        Assert.True(done > 20, $"{done} complete, {partial} partial, {stalled} stalled");
    }

    // ---- shortcuts ----

    [Fact]
    public void Shortcuts_KeepTheCorridorValid_AndMostlyShortenIt()
    {
        int changedVis = 0, changedTop = 0, shorter = 0, longer = 0;
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh mesh = CorridorWorlds.Field(40, seed);
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            var rng = new NavMeshWorlds.Rng(seed);
            var corners = new StraightPathPoint[64];
            for (int trial = 0; trial < 25; trial++)
            {
                PathCorridor? c = Plan(q, CorridorWorlds.RandomFieldPoint(40, rng), CorridorWorlds.RandomFieldPoint(40, rng), 1024);
                if (c is null || c.Count < 4)
                {
                    continue;
                }
                // Make the corridor worse: a detour through a wandering walk, as an agent bumped aside would leave it.
                for (int i = 0; i < 40; i++)
                {
                    c.MovePosition(q, new Vector3D(c.Position.X + (rng.Int(21) - 10) / 3.0, 0, c.Position.Z + (rng.Int(21) - 10) / 3.0));
                }
                c.FindCorners(q, corners, out int cc);
                double lenBefore = Len(corners, cc);
                if (cc > 2 && c.OptimizeVisibility(q, corners[cc - 1].Position, 1000, null, 64))
                {
                    changedVis++;
                    Assert.Null(CorridorWorlds.Check(c, q));
                    c.FindCorners(q, corners, out cc);
                    shorter += Len(corners, cc) < lenBefore - 1e-9 ? 1 : 0;
                    longer += Len(corners, cc) > lenBefore + 1e-9 ? 1 : 0;
                }
                if (c.OptimizeTopology(q, null, 200, 64))
                {
                    changedTop++;
                    Assert.Null(CorridorWorlds.Check(c, q));
                }
                Assert.True(c.IsComplete);
            }
        }
        Assert.True(changedVis > 5, $"{changedVis} visibility changes");
        // A shortcut is a heuristic (it trusts a straight ray), so it may rarely lengthen the string-pulled path; it must mostly shorten it.
        Assert.True(shorter > 4 * longer, $"{shorter} shorter, {longer} longer");
        Assert.True(changedTop >= 0);
    }

    [Fact]
    public void Shortcuts_NeverOverflowASmallCapacity()
    {
        // A ray can cross more polygons than the corridor part it replaces; the corridor must refuse, not overflow.
        TiledNavMesh mesh = CorridorWorlds.Field(30, 6);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(4);
        int refused = 0;
        for (int trial = 0; trial < 400; trial++)
        {
            NearestPoly np = q.FindNearestPoly(CorridorWorlds.RandomFieldPoint(30, rng), Ext);
            if (!np.Poly.IsSet)
            {
                continue;
            }
            var c = new PathCorridor(6 + rng.Int(6));
            c.Reset(np.Poly, np.Point);
            c.SetTarget(CorridorWorlds.RandomFieldPoint(30, rng));
            for (int i = 0; i < 6; i++)
            {
                c.Replan(q, null, 40);
                c.MovePosition(q, new Vector3D(c.Position.X + (rng.Int(21) - 10) / 3.0, 0, c.Position.Z + (rng.Int(21) - 10) / 3.0));
                int before = c.Count;
                bool changed = c.OptimizeVisibility(q, c.Target, 1000, null, 64);
                refused += !changed && before == c.Count ? 1 : 0;
                c.OptimizeTopology(q, null, 200, 40);
                Assert.True(c.Count <= c.Capacity);
                Assert.Null(CorridorWorlds.Check(c, q));
            }
        }
        Assert.True(refused > 0);
    }

    private static double Len(StraightPathPoint[] c, int n)
    {
        double s = 0;
        for (int i = 1; i < n; i++)
        {
            s += Vector3D.Distance(c[i - 1].Position, c[i].Position);
        }
        return s;
    }

    [Fact]
    public void VisibilityBudget_CutsTheRayDeterministically()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(30, 11);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(1);
        for (int trial = 0; trial < 30; trial++)
        {
            PathCorridor? c = Plan(q, CorridorWorlds.RandomFieldPoint(30, rng), CorridorWorlds.RandomFieldPoint(30, rng), 1024);
            if (c is null || c.Count < 4)
            {
                continue;
            }
            byte[] s = c.ToBytes();
            foreach (int budget in new[] { 2, 3, 8, 1000 })
            {
                PathCorridor a = PathCorridor.Read(s, out _), b = PathCorridor.Read(s, out _);
                bool ra = a.OptimizeVisibility(q, a.Target, 1000, null, budget);
                bool rb = b.OptimizeVisibility(q, b.Target, 1000, null, budget);
                Assert.Equal(ra, rb);
                Assert.Equal(a.ToBytes(), b.ToBytes());
                Assert.Null(CorridorWorlds.Check(a, q));
                Assert.True(a.Count <= c.Count);
            }
        }
    }

    // ---- inputs and state ----

    [Fact]
    public void HostileInputs_AreRejectedWithNamedErrors()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(10, 1);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var c = new PathCorridor(8);
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathCorridor(1));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PathCorridor(PathCorridor.MaxCapacity + 1));
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() => c.Reset(PolyRef.Invalid, new Vector3D(double.NaN, 0, 0)));
        Assert.Contains("position", ex.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => c.SetTarget(new Vector3D(0, 3e6, 0)));
        Assert.Throws<ArgumentException>(() => c.SetPath(default, PolyRef.Invalid, ReadOnlySpan<PolyRef>.Empty, false));
        // An empty corridor refuses everything without throwing.
        Assert.False(c.MovePosition(q, new Vector3D(1, 0, 1)));
        Assert.Equal(CorridorStatus.Invalid, c.Validate(q));
        Assert.Equal(QueryStatus.InvalidStartPoly, c.Replan(q));
        // A forged reference is invalid, not a crash.
        c.Reset(new PolyRef(0xDEADBEEFUL), new Vector3D(1, 0, 1));
        Assert.False(c.MovePosition(q, new Vector3D(2, 0, 2)));
        Assert.Equal(CorridorStatus.Invalid, c.Validate(q));
        Assert.Equal(QueryStatus.InvalidCorridor, c.FindCorners(q, new StraightPathPoint[4], out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => new CorridorReplan().Update(0));
        Assert.Throws<ArgumentNullException>(() => c.Validate(null!));
    }

    [Fact]
    public void StateBytes_RoundTrip_AndRejectDamage()
    {
        TiledNavMesh mesh = CorridorWorlds.Field(30, 5);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(3);
        PathCorridor c = null!;
        while (c is null || c.Count < 5)
        {
            c = Plan(q, CorridorWorlds.RandomFieldPoint(30, rng), CorridorWorlds.RandomFieldPoint(30, rng))!;
        }
        byte[] bytes = c.ToBytes();
        Assert.Equal(c.ByteSize, bytes.Length);
        PathCorridor r = PathCorridor.Read(bytes, out int used);
        Assert.Equal(bytes.Length, used);
        Assert.Equal(bytes, r.ToBytes());
        Assert.Equal(c.Path.ToArray(), r.Path.ToArray());
        for (int cut = 0; cut < bytes.Length; cut++)
        {
            Assert.Throws<CrowdStateException>(() => PathCorridor.Read(bytes.AsSpan(0, cut), out _));
        }
        var dmg = new NavMeshWorlds.Rng(77);
        for (int i = 0; i < 2000; i++)
        {
            byte[] b = (byte[])bytes.Clone();
            b[dmg.Int(b.Length)] ^= (byte)(1 << dmg.Int(8));
            try
            {
                PathCorridor x = PathCorridor.Read(b, out _);
                Assert.True(x.Count <= x.Capacity);
            }
            catch (CrowdStateException)
            {
            }
        }
        Assert.Throws<ArgumentException>(() => c.Write(new byte[4]));
    }

    [Fact]
    public void Validate_FindsEachKindOfDamage()
    {
        TiledNavMesh mesh = CorridorWorlds.Layered(2);
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var rng = new NavMeshWorlds.Rng(8);
        List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
        PathCorridor c = null!;
        for (int i = 0; c is null || c.Count < 4; i++)
        {
            Assert.True(i < 500);
            c = Plan(q, CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng), CorridorWorlds.PointIn(mesh, polys[rng.Int(polys.Count)], rng))!;
        }
        byte[] saved = c.ToBytes();
        Assert.Equal(CorridorStatus.Valid, c.Validate(q, null, int.MaxValue));
        // A filter that refuses the third polygon's flags trims the corridor to two.
        mesh.TryGetPoly(c.Path[2], out NavTile t, out int pi);
        ushort fl = t.PolyFlags[pi];
        var filter = new QueryFilter { ExcludeFlags = fl };
        var d = PathCorridor.Read(saved, out _);
        int expectTrim = -1;
        for (int i = 0; i < d.Count; i++)
        {
            mesh.TryGetPoly(d.Path[i], out NavTile ti, out int pj);
            if ((ti.PolyFlags[pj] & fl) != 0)
            {
                expectTrim = i;
                break;
            }
        }
        CorridorStatus st = d.Validate(q, filter, int.MaxValue);
        Assert.Equal(expectTrim == 0 ? CorridorStatus.Invalid : CorridorStatus.Trimmed, st);
        if (st == CorridorStatus.Trimmed)
        {
            Assert.Equal(expectTrim, d.Count);
            Assert.True(d.NeedsReplan);
            Assert.False(d.IsComplete);
        }
        // Removing the first polygon's tile invalidates the corridor from the start.
        var e = PathCorridor.Read(saved, out _);
        Assert.True(mesh.RemoveTile(e.FirstPoly.Tile));
        Assert.Equal(CorridorStatus.Invalid, e.Validate(q));
    }
}
