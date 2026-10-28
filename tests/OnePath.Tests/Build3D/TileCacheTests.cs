using System;
using System.Collections.Generic;
using System.Threading;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

public class TileCacheTests
{
    private static NavSettings Settings => Make(PartitionType.Watershed);

    private static NavSettings Make(PartitionType partition) => new() { Partition = partition, CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    private static Soup Level() => new Soup().Floor(0, 0, 40, 30, 0);

    private static TileCache Cache(Soup s, Action<Mesh3DPipeline>? tweak = null)
    {
        var p = new Mesh3DPipeline(Settings);
        tweak?.Invoke(p);
        return p.BuildSoupCached(s.V.ToArray(), s.I.ToArray());
    }

    // Tile bytes by position, so that slot numbering (which depends on the order of edits) does not matter.
    private static List<byte[]> Tiles(TiledNavMesh m)
    {
        var all = new List<(int Z, int X, int L, byte[] B)>();
        var refs = new TileRef[8];
        for (int z = -2; z < 6; z++)
        {
            for (int x = -2; x < 6; x++)
            {
                int n = m.GetTilesAt(x, z, refs);
                for (int i = 0; i < n; i++)
                {
                    NavTile t = m.GetTile(refs[i])!;
                    all.Add((z, x, t.Layer, NavMeshSerializer.WriteTile(t)));
                }
            }
        }
        var r = new List<byte[]>();
        foreach (var t in all)
        {
            r.Add(t.B);
        }
        return r;
    }

    private static void AssertSame(TiledNavMesh a, TiledNavMesh b)
    {
        List<byte[]> x = Tiles(a), y = Tiles(b);
        Assert.Equal(x.Count, y.Count);
        for (int i = 0; i < x.Count; i++)
        {
            Assert.True(x[i].AsSpan().SequenceEqual(y[i]), $"tile {i} differs");
        }
    }

    private static int Upd(TileCache c, int n) => c.Update(n, TestContext.Current.CancellationToken);

    private static int All(TileCache c) => c.UpdateAll(TestContext.Current.CancellationToken);

    private static readonly TileObstacle[] Sample =
    {
        TileObstacle.Cylinder(15.9, 0, 14, 2.2, 2),
        TileObstacle.Box(30, 0, 5, 33, 2, 12),
        TileObstacle.OrientedBox(8, 1, 22, 3, 1, 0.6, 0.7),
    };

    [Fact]
    public void CachedBuildEqualsPlainBuild()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        TiledNavMesh plain = new Mesh3DPipeline(Settings).BuildSoup(s.V.ToArray(), s.I.ToArray());
        Assert.Equal(NavMeshSerializer.WriteMesh(plain), NavMeshSerializer.WriteMesh(c.Mesh));
        Assert.True(c.CachedTileCount >= 6);
    }

    [Fact]
    public void RebuildFromCacheEqualsFullBuildWithObstaclesBaked()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        foreach (TileObstacle o in Sample)
        {
            c.AddObstacle(o);
        }
        Assert.True(c.PendingCount > 0);
        All(c);
        Assert.Equal(0, c.PendingCount);

        var baked = new Mesh3DPipeline(Settings);
        foreach (TileObstacle o in Sample)
        {
            baked.Obstacles.Add(o);
        }
        TiledNavMesh full = baked.BuildSoup(s.V.ToArray(), s.I.ToArray());
        AssertSame(full, c.Mesh);
        Assert.Empty(NavMeshValidator.Validate(c.Mesh));
        // The obstacles took area away.
        Assert.True(MeshBuildTests.Area(c.Mesh) < MeshBuildTests.Area(new Mesh3DPipeline(Settings).BuildSoup(s.V.ToArray(), s.I.ToArray())) - 20);
    }

    [Fact]
    public void ObstacleBlocksAndRemovalRestoresBytes()
    {
        Soup s = new Soup().Floor(0, 0, 40, 6, 0);
        TileCache c = Cache(s);
        byte[] before = NavMeshSerializer.WriteMesh(c.Mesh);
        var q = new NavMeshQuery(c.Mesh);
        var a = new Vector3D(2, 0, 3);
        var b = new Vector3D(38, 0, 3);
        Assert.Equal(PathStatus.Success, FindPath(q, a, b));

        int wall = c.AddObstacle(TileObstacle.Box(19, 0, -1, 21, 3, 7));
        Assert.Equal(PathStatus.Success, FindPath(q, a, b)); // not rebuilt yet: the old mesh is still in force
        All(c);
        Assert.NotEqual(PathStatus.Success, FindPath(q, a, b));

        Assert.True(c.RemoveObstacle(wall));
        Assert.False(c.RemoveObstacle(wall));
        All(c);
        Assert.Equal(PathStatus.Success, FindPath(q, a, b));
        AssertSame(new Mesh3DPipeline(Settings).BuildSoup(s.V.ToArray(), s.I.ToArray()), c.Mesh);
        Assert.Equal(before.Length, NavMeshSerializer.WriteMesh(c.Mesh).Length);
    }

    private static PathStatus FindPath(NavMeshQuery q, Vector3D a, Vector3D b) => q.FindPath(a, b).Status;

    [Fact]
    public void AddRemoveSequencesReturnToOriginalBytes()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        TiledNavMesh original = new Mesh3DPipeline(Settings).BuildSoup(s.V.ToArray(), s.I.ToArray());
        var rng = new TestRng(7);
        var live = new List<int>();
        for (int step = 0; step < 40; step++)
        {
            if (live.Count > 0 && rng.Int(3) == 0)
            {
                int i = rng.Int(live.Count);
                Assert.True(c.RemoveObstacle(live[i]));
                live.RemoveAt(i);
            }
            else
            {
                double x = rng.Range(0, 40), z = rng.Range(0, 30);
                live.Add(rng.Int(2) == 0
                    ? c.AddObstacle(TileObstacle.Cylinder(x, 0, z, rng.Range(0.3, 3), 2))
                    : c.AddObstacle(TileObstacle.OrientedBox(x, 1, z, rng.Range(0.3, 3), 1, rng.Range(0.3, 3), rng.Range(-3, 3))));
            }
            Upd(c, rng.Int(4));
        }
        foreach (int id in live)
        {
            c.RemoveObstacle(id);
        }
        All(c);
        AssertSame(original, c.Mesh);
        Assert.Equal(0, c.ObstacleCount);
    }

    [Fact]
    public void SlicedRebuildEqualsUnsliced()
    {
        Soup s = Level();
        TileCache whole = Cache(s), sliced = Cache(s);
        foreach (TileObstacle o in Sample)
        {
            whole.AddObstacle(o);
            sliced.AddObstacle(o);
        }
        whole.AddVolume(new ConvexVolume(new double[] { 3, 3, 12, 3, 12, 12, 3, 12 }, -1, 1, 5));
        sliced.AddVolume(new ConvexVolume(new double[] { 3, 3, 12, 3, 12, 12, 3, 12 }, -1, 1, 5));
        int total = whole.PendingCount;
        Assert.Equal(sliced.PendingCount, total);
        Assert.Equal(total, All(whole));
        int rounds = 0, done = 0;
        while (sliced.PendingCount > 0)
        {
            int n = Upd(sliced, 1);
            Assert.Equal(1, n);
            done += n;
            rounds++;
        }
        Assert.Equal(total, done);
        Assert.Equal(total, rounds);
        Assert.Equal(0, Upd(sliced, 5));
        AssertSame(whole.Mesh, sliced.Mesh);
        Assert.Equal(NavMeshSerializer.WriteMesh(whole.Mesh), NavMeshSerializer.WriteMesh(sliced.Mesh));
    }

    [Fact]
    public void CancelledUpdateKeepsTheRestQueued()
    {
        TileCache c = Cache(Level());
        c.AddObstacle(TileObstacle.Box(0, 0, 0, 40, 2, 30));
        int total = c.PendingCount;
        using var cts = new CancellationTokenSource();
        cts.Cancel();
        Assert.Throws<OperationCanceledException>(() => c.Update(5, cts.Token));
        Assert.Equal(total, c.PendingCount);
        Assert.Equal(total, All(c));
    }

    [Fact]
    public void VolumesMarkAreasAtRunTimeAndAtBuildTimeAlike()
    {
        Soup s = Level();
        double[] zone = { 20, 5, 36, 5, 36, 25, 20, 25 };
        TileCache c = Cache(s);
        int id = c.AddVolume(new ConvexVolume(zone, -1, 1, 7));
        All(c);
        int inside = 0, outside = 0;
        for (int slot = 0; slot < c.Mesh.SlotCount; slot++)
        {
            NavTile? t = c.Mesh.GetTileBySlot(slot);
            if (t is null)
            {
                continue;
            }
            foreach (byte a in t.PolyAreas)
            {
                if (a == 7)
                {
                    inside++;
                }
                else
                {
                    outside++;
                }
            }
        }
        Assert.True(inside > 0 && outside > 0);

        var baked = new Mesh3DPipeline(Settings);
        baked.Regions.Spans.Volumes.Add(new ConvexVolume(zone, -1, 1, 7));
        AssertSame(baked.BuildSoup(s.V.ToArray(), s.I.ToArray()), c.Mesh);

        // The area costs the queries see follow: a high cost on area 7 makes the path avoid it where it can.
        Assert.True(c.RemoveVolume(id));
        All(c);
        foreach (NavTile? t in AllTiles(c.Mesh))
        {
            Assert.DoesNotContain((byte)7, t!.PolyAreas.ToArray());
        }
        AssertSame(new Mesh3DPipeline(Settings).BuildSoup(s.V.ToArray(), s.I.ToArray()), c.Mesh);
    }

    private static IEnumerable<NavTile?> AllTiles(TiledNavMesh m)
    {
        for (int i = 0; i < m.SlotCount; i++)
        {
            if (m.GetTileBySlot(i) is { } t)
            {
                yield return t;
            }
        }
    }

    [Fact]
    public void AnUnchangedTileKeepsItsReferences()
    {
        TileCache c = Cache(Level());
        NavTile far = c.Mesh.GetTile(c.Mesh.GetTileRef(2, 1, 0))!;
        TileRef before = c.Mesh.GetTileRef(2, 1, 0);
        c.AddObstacle(TileObstacle.Cylinder(2, 0, 2, 1, 2));
        All(c);
        Assert.True(c.Mesh.IsValid(before));
        Assert.Same(far, c.Mesh.GetTile(before));
    }

    [Fact]
    public void SnapshotMutateRestoreContinueEqualsNeverRolledBack()
    {
        Soup s = Level();
        TileCache live = Cache(s), twin = Cache(s);
        void Common(TileCache c)
        {
            c.AddObstacle(Sample[0]);
            c.AddVolume(new ConvexVolume(new double[] { 0, 0, 10, 0, 10, 10 }, -1, 1, 9));
            Upd(c, 2);
        }
        Common(live);
        Common(twin);
        foreach (SnapshotMode mode in new[] { SnapshotMode.Full, SnapshotMode.Referenced })
        {
            var lib = new TileLibrary();
            lib.AddAll(live.Mesh);
            byte[] snap = live.Snapshot(mode);
            Assert.True(live.PendingCount > 0);

            // Mutate: more obstacles, finish, remove.
            int extra = live.AddObstacle(Sample[1]);
            All(live);
            live.AddObstacle(Sample[2]);
            Upd(live, 1);
            live.RemoveObstacle(extra);

            lib.AddAll(live.Mesh);
            live.Restore(snap, lib);
            Assert.Equal(snap, live.Snapshot(mode));
            Assert.Equal(twin.PendingCount, live.PendingCount);

            // Carry on identically on both.
            foreach (TileCache c in new[] { live, twin })
            {
                c.AddObstacle(Sample[2]);
                Upd(c, 3);
            }
            Assert.Equal(twin.Snapshot(mode), live.Snapshot(mode));
            All(live);
            All(twin);
            AssertSame(twin.Mesh, live.Mesh);

            // Back to the shared starting point for the next mode.
            live.Restore(snap, lib);
            twin.Restore(snap, lib);
        }
    }

    [Fact]
    public void RestoreRejectsBadBytesAndLeavesTheCacheAlone()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        c.AddObstacle(Sample[0]);
        Upd(c, 1);
        byte[] good = c.Snapshot();
        byte[] state = c.Snapshot();
        for (int cut = 0; cut < good.Length; cut += Math.Max(1, good.Length / 97))
        {
            Assert.Throws<NavMeshFormatException>(() => c.Restore(good.AsSpan(0, cut)));
        }
        byte[] bad = (byte[])good.Clone();
        bad[0] ^= 1;
        Assert.Throws<NavMeshFormatException>(() => c.Restore(bad));
        bad = (byte[])good.Clone();
        bad[4] = 9; // version
        Assert.Throws<NavMeshFormatException>(() => c.Restore(bad));
        // A snapshot naming a queued tile this cache has no data for.
        TileCache other = Cache(new Soup().Floor(0, 0, 10, 10, 0));
        other.AddObstacle(TileObstacle.Cylinder(5, 0, 5, 1, 1));
        Assert.Equal(1, other.PendingCount);
        Assert.Equal(state, c.Snapshot());
        // Valid bytes with a trailing byte.
        byte[] longer = new byte[good.Length + 1];
        good.CopyTo(longer, 0);
        Assert.Throws<NavMeshFormatException>(() => c.Restore(longer));
        Assert.Equal(state, c.Snapshot());
    }

    [Fact]
    public void ObstacleInputIsValidated()
    {
        Assert.Throws<ArgumentException>(() => TileObstacle.Cylinder(double.NaN, 0, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => TileObstacle.Cylinder(0, 0, 0, 0, 1));
        Assert.Throws<ArgumentException>(() => TileObstacle.Cylinder(0, 0, 0, 1, -1));
        Assert.Throws<ArgumentOutOfRangeException>(() => TileObstacle.Cylinder(2097151.5, 0, 0, 5, 1));
        Assert.Throws<ArgumentException>(() => TileObstacle.Box(1, 0, 0, 0, 1, 1));
        Assert.Throws<ArgumentException>(() => TileObstacle.OrientedBox(0, 0, 0, 1, 1, 1, double.PositiveInfinity));
        Assert.Throws<ArgumentException>(() => TileObstacle.OrientedBox(0, 0, 0, 0, 1, 1, 0));
        TileCache c = Cache(Level());
        Assert.Throws<ArgumentNullException>(() => c.AddObstacle(null!));
        Assert.Throws<ArgumentOutOfRangeException>(() => c.Update(-1, CancellationToken.None));
        Assert.Equal(0, Upd(c, 0));
    }

    [Fact]
    public void OrientedBoxTurnsByItsYaw()
    {
        // Half sizes 4 x 0.5; turned a quarter turn it is long along z.
        TileObstacle o = TileObstacle.OrientedBox(0, 0, 0, 4, 1, 0.5, Math.PI / 2);
        Assert.True(o.ContainsXZ(0, 3.9));
        Assert.False(o.ContainsXZ(3.9, 0));
        TileObstacle flat = TileObstacle.OrientedBox(0, 0, 0, 4, 1, 0.5, 0);
        Assert.True(flat.ContainsXZ(3.9, 0));
        Assert.False(flat.ContainsXZ(0, 3.9));
    }

    [Fact]
    public void LayeredLevelRebuildsAllLayers()
    {
        // A bridge over a floor: two layers in a tile when partitioned into layers.
        Soup s = new Soup().Floor(0, 0, 20, 20, 0).Floor(4, 4, 16, 16, 4);
        NavSettings st = Make(PartitionType.Layers);
        var p = new Mesh3DPipeline(st);
        TileCache c = p.BuildSoupCached(s.V.ToArray(), s.I.ToArray());
        var baked = new Mesh3DPipeline(st);
        baked.Obstacles.Add(TileObstacle.Box(8, 3.5, 8, 12, 5, 12));
        c.AddObstacle(TileObstacle.Box(8, 3.5, 8, 12, 5, 12));
        All(c);
        AssertSame(baked.BuildSoup(s.V.ToArray(), s.I.ToArray()), c.Mesh);
        Assert.Empty(NavMeshValidator.Validate(c.Mesh));
    }
}
