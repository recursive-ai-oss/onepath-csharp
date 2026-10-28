using System;
using System.Linq;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

#pragma warning disable xUnit1051

namespace RecursiveAI.OnePath.Tests;

/// <summary>Saving and loading tier-1 meshes and worlds, and Crowd.TryAdd.</summary>
public class Tier1SaveLoadTests
{
    private static readonly NavSettings Settings2D = NavSettings.Humanoid with
    {
        CellSize = 0.5,
        TileSize = 8,
        FloorY = 0.25,
        SnapBits = 9,
        OffMeshLinks = [OffMeshLinkDefinition.Create(new Vector3D(4.5, 0.25, 6), new Vector3D(6.5, 0.25, 6), 9)],
    };

    private static NavMesh Level2D(NavSettings? s = null)
    {
        const int W = 16, H = 16;
        var cells = new bool[W * H];
        for (int y = 0; y < H; y++)
        {
            for (int x = 0; x < W; x++)
            {
                cells[(y * W) + x] = !((x == 5 && y != 3 && y != 10) || (y == 8 && x > 9));
            }
        }
        return NavMesh.Build2D(cells, W, H, 1.0, new Vector2D(0, 0), s ?? Settings2D);
    }

    private static NavMesh Floor3D()
    {
        // A flat 20 x 20 floor.
        double[] v = [0, 0, 0, 20, 0, 0, 20, 0, 20, 0, 0, 20];
        int[] ix = [0, 2, 1, 0, 3, 2];
        return NavMesh.Build(v, ix, NavSettings.Humanoid with { CellSize = 0.25, TileSize = 8 });
    }

    private static bool Same(PathResult a, PathResult b) =>
        a.Detail == b.Detail && BitConverter.DoubleToInt64Bits(a.Cost) == BitConverter.DoubleToInt64Bits(b.Cost) && a.Points.Count == b.Points.Count
        && a.Points.Zip(b.Points).All(t => t.First.Position.Equals(t.Second.Position) && t.First.Flags == t.Second.Flags)
        && a.Corridor.SequenceEqual(b.Corridor);

    private static void AssertEquivalent(NavMesh a, NavMesh b)
    {
        Assert.Equal(a.Settings, b.Settings);
        Assert.True(NavMeshSerializer.WriteMesh(a.Tiles).AsSpan().SequenceEqual(NavMeshSerializer.WriteMesh(b.Tiles)));
        var r = new Random(4);
        for (int i = 0; i < 40; i++)
        {
            var p = new Vector3D(r.NextDouble() * 16, 0.25, r.NextDouble() * 16);
            var q = new Vector3D(r.NextDouble() * 16, 0.25, r.NextDouble() * 16);
            Assert.True(Same(a.FindPath(p, q), b.FindPath(p, q)), $"path {i}");
            Assert.Equal(a.TrySamplePosition(q, 3, out Vector3D sa), b.TrySamplePosition(q, 3, out Vector3D sb));
            Assert.Equal(sa, sb);
            Assert.Equal(a.Raycast(p, q, out NavMeshHit ha), b.Raycast(p, q, out NavMeshHit hb));
            Assert.Equal(ha.Position, hb.Position);
        }
        var ra = new SeededRandom(3);
        var rb = new SeededRandom(3);
        for (int i = 0; i < 10; i++)
        {
            Assert.Equal(a.TryGetRandomPoint(ref ra, out Vector3D pa), b.TryGetRandomPoint(ref rb, out Vector3D pb));
            Assert.Equal(pa, pb);
        }
    }

    [Fact]
    public async Task RoundTrip_2D_KeepsSettingsAndEveryAnswer()
    {
        NavMesh nav = Level2D();
        byte[] bytes = nav.Save();
        NavMesh loaded = NavMesh.Load(bytes);
        AssertEquivalent(nav, loaded);
        Assert.True(loaded.Save().AsSpan().SequenceEqual(bytes));
        Assert.Equal(bytes.Length, nav.GetSaveSize());
        // The loaded mesh is a full tier-1 mesh: crowds, streaming and async paths work.
        Crowd crowd = loaded.CreateCrowd();
        Agent agent = crowd.Add(new Vector3D(2, 0.25, 2), Agent.Humanoid);
        Assert.Equal(PathStatus.Found, agent.SetDestination(new Vector3D(13, 0.25, 13)));
        for (int i = 0; i < 20; i++)
        {
            crowd.Step(1.0 / 30);
        }
        Assert.NotNull(loaded.CreateStreamer());
        PathResult asyncPath = await loaded.FindPathAsync(new Vector3D(2, 0.25, 2), new Vector3D(13, 0.25, 13));
        Assert.True(Same(asyncPath, nav.FindPath(new Vector3D(2, 0.25, 2), new Vector3D(13, 0.25, 13))));
    }

    [Fact]
    public void RoundTrip_3D_AfterFlagEdits()
    {
        NavMesh nav = Floor3D();
        byte[] bytes = nav.Save();
        NavMesh loaded = NavMesh.Load(bytes);
        Assert.Equal(nav.Settings, loaded.Settings);
        var a = new Vector3D(1, 0, 1);
        var b = new Vector3D(18, 0, 17);
        Assert.True(Same(nav.FindPath(a, b), loaded.FindPath(a, b)));
        Assert.True(loaded.Save().AsSpan().SequenceEqual(bytes));
    }

    [Fact]
    public void TrySave_WritesTheSameBytesAndRefusesASmallBuffer()
    {
        NavMesh nav = Level2D();
        byte[] bytes = nav.Save();
        var exact = new byte[bytes.Length];
        Assert.True(nav.TrySave(exact, out int n));
        Assert.Equal(bytes.Length, n);
        Assert.Equal(bytes, exact);
        Assert.False(nav.TrySave(new byte[bytes.Length - 1], out int none));
        Assert.Equal(0, none);
    }

    [Fact]
    public void BareMeshBytes_NeedTheirSettings_AndAreCheckedAgainstThem()
    {
        NavMesh nav = Level2D();
        byte[] bare = NavMeshSerializer.WriteMesh(nav.Tiles);
        Assert.Throws<NavMeshFormatException>(() => NavMesh.Load(bare));
        NavMesh loaded = NavMesh.Load(bare, Settings2D);
        Assert.Same(Settings2D, loaded.Settings);
        AssertEquivalent(nav, loaded);
        Assert.Throws<NavMeshFingerprintException>(() => NavMesh.Load(bare, Settings2D with { CellSize = 0.25 }));
        Assert.Throws<NavMeshFingerprintException>(() => NavMesh.Load(bare, Settings2D with { AgentRadius = 0.3 }));
        Assert.Throws<NavMeshFingerprintException>(() => NavMesh.Load(bare, Settings2D with { MinRegionArea = 3 }));
        Assert.Throws<NavMeshFingerprintException>(() => NavMesh.Load(bare, Settings2D with { OffMeshLinks = [] }));
        // The same checks apply to settings given beside a saved envelope.
        byte[] saved = nav.Save();
        Assert.Throws<NavMeshFingerprintException>(() => NavMesh.Load(saved, Settings2D with { FloorY = 1 }));
        Assert.Same(Settings2D, NavMesh.Load(saved, Settings2D).Settings);
    }

    [Fact]
    public void World_RoundTripsEveryClass()
    {
        var ring = new Vector2D[] { new(0, 0), new(16, 0), new(16, 16), new(0, 16) };
        NavWorld world = NavWorld.Build2D([new RecursiveAI.OnePath.Mesh.Build2D.Outline(ring)], NavSettings.Humanoid with { CellSize = 0.5, TileSize = 8 }, NavSettings.Small with { CellSize = 0.5, TileSize = 8 });
        byte[] bytes = world.Save();
        NavWorld loaded = NavWorld.Load(bytes);
        Assert.Equal(world.ClassCount, loaded.ClassCount);
        for (int i = 0; i < world.ClassCount; i++)
        {
            Assert.Equal(world[i].Settings, loaded[i].Settings);
            Assert.True(NavMeshSerializer.WriteMesh(world[i].Tiles).AsSpan().SequenceEqual(NavMeshSerializer.WriteMesh(loaded[i].Tiles)));
        }
        Assert.Equal(world.ClassFor(0.4), loaded.ClassFor(0.4));
        Assert.True(loaded.Save().AsSpan().SequenceEqual(bytes));
        var big = new byte[bytes.Length];
        Assert.True(world.TrySave(big, out int n) && n == bytes.Length);
        Assert.False(world.TrySave(new byte[3], out _));
    }

    [Fact]
    public void HostileBytes_AreRefusedWithTypedErrors()
    {
        byte[] mesh = Level2D().Save();
        var ring = new Vector2D[] { new(0, 0), new(8, 0), new(8, 8), new(0, 8) };
        byte[] world = NavWorld.Build2D([new RecursiveAI.OnePath.Mesh.Build2D.Outline(ring)], NavSettings.Humanoid with { CellSize = 0.5, TileSize = 8 }).Save();
        Assert.Throws<NavMeshFormatException>(() => NavMesh.Load([]));
        Assert.Throws<NavMeshFormatException>(() => NavMesh.Load(new byte[40]));
        Assert.Throws<NavMeshFormatException>(() => NavWorld.Load(mesh));
        Assert.Throws<NavMeshFormatException>(() => NavMesh.Load(world));
        foreach (byte[] good in new[] { mesh, world })
        {
            // Every truncation, and single-byte changes spread over the file, either load or throw the typed error: nothing else escapes.
            var r = new Random(11);
            for (int len = 0; len < good.Length; len += Math.Max(1, good.Length / 400))
            {
                Guard(good.AsSpan(0, len).ToArray(), good == world);
            }
            for (int i = 0; i < 1500; i++)
            {
                byte[] bad = (byte[])good.Clone();
                bad[r.Next(Math.Min(bad.Length, 300))] ^= (byte)(1 << r.Next(8));
                if (r.Next(3) == 0)
                {
                    bad[r.Next(bad.Length)] = (byte)r.Next(256);
                }
                Guard(bad, good == world);
            }
        }
    }

    private static void Guard(byte[] data, bool world)
    {
        try
        {
            if (world)
            {
                _ = NavWorld.Load(data);
            }
            else
            {
                _ = NavMesh.Load(data);
            }
        }
        catch (OnePathFormatException)
        {
        }
    }

    [Fact]
    public void TryAdd_ReportsAnOffMeshPositionAndAFullCrowdWithoutThrowing()
    {
        NavMesh nav = Level2D(NavSettings.Humanoid with { CellSize = 0.5, TileSize = 8 });
        Crowd crowd = nav.CreateCrowd(new CrowdOptions { MaxAgents = 2 });
        Assert.True(crowd.TryAdd(new Vector3D(2, 0, 2), null, out Agent? a));
        Assert.NotNull(a);
        Assert.Equal(0, a.Id);
        Assert.False(crowd.TryAdd(new Vector3D(500, 0, 500), null, out Agent? off));
        Assert.Null(off);
        Assert.Equal(1, crowd.Count);
        Assert.Throws<ArgumentException>(() => crowd.Add(new Vector3D(500, 0, 500)));
        Assert.True(crowd.TryAdd(new Vector3D(3, 0, 3), Agent.Small, out Agent? b));
        Assert.Equal(1, b.Id);
        Assert.False(crowd.TryAdd(new Vector3D(4, 0, 4), null, out Agent? full));
        Assert.Null(full);
        Assert.Throws<InvalidOperationException>(() => crowd.Add(new Vector3D(4, 0, 4)));
        // Mistakes in the call still throw.
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.TryAdd(new Vector3D(double.NaN, 0, 0), null, out _));
    }
}
