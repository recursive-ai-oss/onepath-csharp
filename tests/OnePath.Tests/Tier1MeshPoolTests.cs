using System;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

#pragma warning disable xUnit1051

namespace RecursiveAI.OnePath.Tests;

/// <summary>The pooled query behind the one-call mesh forms, the tier-1 sample/raycast/random members and live agent settings.</summary>
public class Tier1MeshPoolTests
{
    private static NavMesh Level(int wallColumn = 3)
    {
        const int W = 24, H = 24;
        var cells = new bool[W * H];
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                cells[(y * W) + x] = !(x % 6 == wallColumn && y % 8 != 4);
            }
        }
        return NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(-12, -12));
    }

    private static bool SameBits(PathResult a, PathResult b) =>
        a.Detail == b.Detail && BitConverter.DoubleToInt64Bits(a.Cost) == BitConverter.DoubleToInt64Bits(b.Cost)
        && a.Points.Count == b.Points.Count
        && a.Points.Zip(b.Points).All(t => BitConverter.DoubleToInt64Bits(t.First.Position.X) == BitConverter.DoubleToInt64Bits(t.Second.Position.X)
            && BitConverter.DoubleToInt64Bits(t.First.Position.Y) == BitConverter.DoubleToInt64Bits(t.Second.Position.Y)
            && BitConverter.DoubleToInt64Bits(t.First.Position.Z) == BitConverter.DoubleToInt64Bits(t.Second.Position.Z))
        && a.Corridor.SequenceEqual(b.Corridor);

    [Fact]
    public void FindPath_DoesNotAllocateAQueryPerCall()
    {
        NavMesh nav = Level();
        var a = new Vector3D(-11, 0, -11);
        var b = new Vector3D(10, 0, 10);
        for (int i = 0; i < 5; i++)
        {
            _ = nav.FindPath(a, b);
        }
        long allocated = AllocationProbe.SteadyState(() =>
        {
            for (int i = 0; i < 20; i++)
            {
                _ = nav.FindPath(a, b);
            }
        }) / 20;
        // The returned path and corridor, nothing near the ~10 MB a fresh query object costs.
        Assert.True(allocated < 4096, $"{allocated} bytes per call");
    }

    [Fact]
    public void PooledPaths_EqualFreshQueries_AcrossMeshEdits()
    {
        NavMesh nav = Level();
        var r = new Random(3);
        for (int i = 0; i < 120; i++)
        {
            if (i == 60)
            {
                // Replace the mesh content between calls: the pooled query follows the edit.
                NavMesh other = Level(2);
                RecursiveAI.OnePath.Mesh.NavMeshSnapshot.RestoreInto(nav.Tiles, RecursiveAI.OnePath.Mesh.NavMeshSnapshot.Take(other.Tiles));
            }
            var a = new Vector3D(-12 + (r.NextDouble() * 24), 0, -12 + (r.NextDouble() * 24));
            var b = new Vector3D(-12 + (r.NextDouble() * 24), 0, -12 + (r.NextDouble() * 24));
            Assert.True(SameBits(nav.FindPath(a, b), new NavMeshQuery(nav.Tiles).FindPath(a, b)), $"call {i}");
        }
    }

    [Fact]
    public async Task AsyncAndParallelCalls_AnswerLikeFreshQueries()
    {
        NavMesh nav = Level();
        var a = new Vector3D(-11, 0, -11);
        var b = new Vector3D(10, 0, 10);
        PathResult expected = new NavMeshQuery(nav.Tiles).FindPath(a, b);
        Task<PathResult>[] tasks = Enumerable.Range(0, 16).Select(i => i % 2 == 0 ? nav.FindPathAsync(a, b) : Task.Run(() => nav.FindPath(a, b))).ToArray();
        foreach (PathResult p in await Task.WhenAll(tasks))
        {
            Assert.True(SameBits(expected, p));
        }
        // A cancelled search leaves the pool usable.
        using var cts = new CancellationTokenSource();
        cts.Cancel();
        Assert.Throws<OperationCanceledException>(() => nav.FindPath(a, b, cancellationToken: cts.Token));
        Assert.True(SameBits(expected, nav.FindPath(a, b)));
    }

    [Fact]
    public void SamplePosition_SnapsWithinRangeOnly()
    {
        NavMesh nav = Level();
        Assert.True(nav.TrySamplePosition(new Vector3D(0.3, 2, 0.2), 3, out Vector3D p));
        Assert.True(Vector3D.Distance(p, new Vector3D(0.3, 2, 0.2)) <= 3);
        Assert.False(nav.TrySamplePosition(new Vector3D(100, 0, 100), 5, out Vector3D none));
        Assert.Equal(default, none);
        Assert.Throws<ArgumentOutOfRangeException>(() => nav.TrySamplePosition(new Vector3D(0, 0, 0), -1, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => nav.TrySamplePosition(new Vector3D(3e6, 0, 0), 1, out _));
    }

    [Fact]
    public void Raycast_StopsAtWallsAndMatchesTheTier2Trace()
    {
        NavMesh nav = Level();
        // The column of cells at world x -9..-8 is a wall except in the rows around z = -8 (the mesh is shrunk by the agent radius).
        var from = new Vector3D(-10.5, 0, -10.5);
        Assert.True(nav.Raycast(from, new Vector3D(-5.5, 0, -10.5), out NavMeshHit hit));
        Assert.InRange(hit.Position.X, -10, -9);
        Assert.InRange(hit.Distance, 0.5, 1.5);
        Assert.False(nav.Raycast(from, new Vector3D(-10.5, 0, -8.5), out NavMeshHit clear));
        Assert.Equal(default, clear);
        Assert.True(nav.Raycast(new Vector3D(500, 0, 500), from, out NavMeshHit off));
        Assert.Equal(0, off.Distance);

        var q = new NavMeshQuery(nav.Tiles);
        NearestPoly s = q.FindNearestPoly(from, new Vector3D(2, 4, 2));
        RaycastResult r = q.Raycast(s.Poly, s.Point, new Vector3D(-5.5, 0, -10.5), null, default, out _);
        Assert.True(r.Hit);
        Assert.Equal(hit.Normal, r.Normal);
    }

    [Fact]
    public void RandomPoint_IsSeededAndLandsOnTheMesh()
    {
        NavMesh nav = Level();
        var r1 = new SeededRandom(9);
        var r2 = new SeededRandom(9);
        for (int i = 0; i < 50; i++)
        {
            Assert.True(nav.TryGetRandomPoint(ref r1, out Vector3D p));
            Assert.True(nav.TryGetRandomPoint(ref r2, out Vector3D q));
            Assert.Equal(p, q);
            Assert.True(nav.TrySamplePosition(p, 0.001, out _));
        }
    }

    [Fact]
    public void SetOptions_ChangesALiveAgentInPlace()
    {
        NavMesh nav = Level();
        Crowd crowd = nav.CreateCrowd();
        Agent agent = crowd.Add(new Vector3D(-11.5, 0, -11.5), Agent.Humanoid);
        agent.SetDestination(new Vector3D(10, 0, 10));
        for (int i = 0; i < 30; i++)
        {
            crowd.Step(1.0 / 30);
        }
        long id = agent.Id;
        Vector3D pos = agent.Position;
        Assert.True(agent.HasPath);
        AgentOptions slow = agent.Options with { MaxSpeed = 1.0, Priority = 3 };
        agent.SetOptions(slow);
        Assert.Same(slow, agent.Options);
        Assert.Equal(id, agent.Id);
        Assert.Equal(pos, agent.Position);
        Assert.True(agent.HasPath);
        Assert.Equal(3, agent.Priority);
        for (int i = 0; i < 60; i++)
        {
            crowd.Step(1.0 / 30);
            if (i >= 30)
            {
                // Braking from the old top speed takes a few steps (acceleration is limited); after that the new top speed holds.
                Assert.True(Vector3D.Distance(agent.Velocity, default) <= 1.0 + 1e-9);
            }
        }
        Assert.True(agent.IsActive);

        Assert.Throws<ArgumentOutOfRangeException>(() => agent.SetOptions(slow with { MaxSpeed = 0 }));
        Assert.Same(slow, agent.Options);
        Assert.Throws<ArgumentNullException>(() => agent.SetOptions(null!));
        crowd.Remove(agent);
        Assert.Throws<InvalidOperationException>(() => agent.SetOptions(slow));
    }

    [Fact]
    public void SetOptions_IsCarriedBySnapshotsAndKeepsHandlesOnRestore()
    {
        NavMesh nav = Level();
        Crowd crowd = nav.CreateCrowd();
        Agent agent = crowd.Add(new Vector3D(-11.5, 0, -11.5), Agent.Humanoid);
        agent.SetDestination(new Vector3D(10, 0, 10));
        for (int i = 0; i < 20; i++)
        {
            crowd.Step(1.0 / 30);
        }
        byte[] before = crowd.SaveState();
        AgentOptions fast = agent.Options with { MaxSpeed = 6, Radius = 0.4 };
        agent.SetOptions(fast);
        byte[] after = crowd.SaveState();
        Assert.NotEqual(before, after);
        for (int i = 0; i < 40; i++)
        {
            crowd.Step(1.0 / 30);
        }
        byte[] end = crowd.SaveState();

        crowd.RestoreState(before);
        Assert.True(agent.IsActive);
        Assert.Equal(Agent.Humanoid, agent.Options);
        crowd.RestoreState(after);
        Assert.True(agent.IsActive);
        Assert.Equal(fast, agent.Options);
        Assert.Equal(after, crowd.SaveState());
        for (int i = 0; i < 40; i++)
        {
            crowd.Step(1.0 / 30);
        }
        Assert.Equal(end, crowd.SaveState());
    }
}
