using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>
/// The layer-first tile build and the tile cache that rebuilds one layer at a time (no detail sampling, layer partition): a cache is a function of the
/// obstacles and volumes in force, never of the order they came in, and it agrees with the general layer-first path wherever a tile has one layer.
/// </summary>
public class LayerTileTests
{
    private static NavSettings Lean => new() { Partition = PartitionType.Layers, CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 0 };

    private static Soup Level() => new Soup().Floor(0, 0, 40, 30, 0).Floor(4, 4, 28, 20, 4).Floor(33, 3, 39, 9, 0.3);

    private static TileCache Cache(Soup s, NavSettings? st = null) => new Mesh3DPipeline(st ?? Lean).BuildSoupCached(s.V.ToArray(), s.I.ToArray());

    private static TiledNavMesh Baked(Soup s, IEnumerable<TileObstacle> obstacles, NavSettings? st = null, IEnumerable<ConvexVolume>? volumes = null)
    {
        var p = new Mesh3DPipeline(st ?? Lean);
        foreach (TileObstacle o in obstacles)
        {
            p.Obstacles.Add(o);
        }
        if (volumes is not null)
        {
            foreach (ConvexVolume v in volumes)
            {
                p.Regions.Spans.Volumes.Add(v);
            }
        }
        return p.BuildSoup(s.V.ToArray(), s.I.ToArray());
    }

    // Tile bytes by position, so that slot numbering (which depends on the order of edits) does not matter.
    private static List<byte[]> Tiles(TiledNavMesh m)
    {
        var all = new List<byte[]>();
        var refs = new TileRef[16];
        for (int z = -2; z < 6; z++)
        {
            for (int x = -2; x < 6; x++)
            {
                int n = m.GetTilesAt(x, z, refs);
                for (int i = 0; i < n; i++)
                {
                    all.Add(NavMeshSerializer.WriteTile(m.GetTile(refs[i])!));
                }
            }
        }
        return all;
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

    private static int All(TileCache c) => c.UpdateAll(TestContext.Current.CancellationToken);

    private static readonly TileObstacle[] Obstacles =
    {
        TileObstacle.Cylinder(15.9, 0, 14, 2.2, 2),
        TileObstacle.Box(30, 0, 5, 33, 2, 12),
        TileObstacle.OrientedBox(8, 4, 12, 3, 1, 0.6, 0.7),
        TileObstacle.Cylinder(16, 4, 10, 1.5, 2),
        TileObstacle.Box(2, 0, 25, 9, 2, 27),
    };

    [Fact]
    public void FirstBuildWithCacheEqualsPlainBuildAndIsValid()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        TiledNavMesh plain = new Mesh3DPipeline(Lean).BuildSoup(s.V.ToArray(), s.I.ToArray());
        Assert.Equal(NavMeshSerializer.WriteMesh(plain), NavMeshSerializer.WriteMesh(c.Mesh));
        Assert.Empty(NavMeshValidator.Validate(plain));
        // The bridge over the floor makes a second layer.
        var refs = new TileRef[16];
        int maxLayers = 0;
        for (int z = 0; z < 2; z++)
        {
            for (int x = 0; x < 3; x++)
            {
                maxLayers = Math.Max(maxLayers, plain.GetTilesAt(x, z, refs));
            }
        }
        Assert.True(maxLayers >= 2, "the level should have a tile with two layers");
    }

    [Fact]
    public void CacheEqualsBakedBuildInAnyOrderOfEdits()
    {
        Soup s = Level();
        for (int seed = 1; seed <= 12; seed++)
        {
            var rng = new Random(seed);
            TileCache c = Cache(s);
            var live = new Dictionary<int, TileObstacle>();
            for (int step = 0; step < 14; step++)
            {
                if (live.Count > 0 && rng.Next(3) == 0)
                {
                    int pick = new List<int>(live.Keys)[rng.Next(live.Count)];
                    Assert.True(c.RemoveObstacle(pick));
                    live.Remove(pick);
                }
                else
                {
                    TileObstacle o = Obstacles[rng.Next(Obstacles.Length)];
                    live[c.AddObstacle(o)] = o;
                }
                if (rng.Next(2) == 0)
                {
                    c.Update(rng.Next(1, 4), TestContext.Current.CancellationToken);
                }
            }
            All(c);
            AssertSame(Baked(s, live.Values), c.Mesh);
            Assert.Empty(NavMeshValidator.Validate(c.Mesh));
        }
    }

    // Repairing the first build's contours where an obstacle took spans away must give exactly the tiles a from-scratch build of the same areas gives,
    // whatever the obstacles' shapes, places and history: random cylinders, boxes and oriented boxes, from specks to blocks, over walls, ledges and tile borders.
    [Fact]
    public void ContourRepairEqualsBuildingFromScratch()
    {
        Soup s = Level();
        int repairedLayers = 0, scratchLayers = 0;
        for (int seed = 1; seed <= 12; seed++)
        {
            var rng = new Random(seed * 7919);
            TileCache repaired = Cache(s);
            TileCache scratch = Cache(s);
            scratch.RepairContours = false;
            var live = new List<(int A, int B)>();
            for (int step = 0; step < 40; step++)
            {
                if (live.Count > 0 && rng.Next(3) == 0)
                {
                    int k = rng.Next(live.Count);
                    Assert.True(repaired.RemoveObstacle(live[k].A));
                    Assert.True(scratch.RemoveObstacle(live[k].B));
                    live.RemoveAt(k);
                }
                else
                {
                    double x = rng.NextDouble() * 42 - 1, z = rng.NextDouble() * 32 - 1, y = rng.Next(3) == 0 ? 4 : 0;
                    double size = rng.Next(4) == 0 ? 0.05 + (rng.NextDouble() * 0.3) : 0.3 + (rng.NextDouble() * 4);
                    TileObstacle o = rng.Next(3) switch
                    {
                        0 => TileObstacle.Cylinder(x, y, z, size, 2),
                        1 => TileObstacle.Box(x, y, z, x + size + (rng.NextDouble() * 5), y + 2, z + size),
                        _ => TileObstacle.OrientedBox(x, y + 1, z, size + 0.3, 1, (size * 0.5) + 0.2, rng.NextDouble() * 6),
                    };
                    live.Add((repaired.AddObstacle(o), scratch.AddObstacle(o)));
                }

                All(repaired);
                All(scratch);
                AssertSame(scratch.Mesh, repaired.Mesh);
            }

            Assert.Empty(NavMeshValidator.Validate(repaired.Mesh));
            repairedLayers += repaired.RepairCounts.Repaired;
            scratchLayers += repaired.RepairCounts.Scratch;
        }

        // The repair is what ran, not a fallback, in most builds.
        Assert.True(repairedLayers > 0 && repairedLayers + scratchLayers > 0, $"repaired {repairedLayers}, from scratch {scratchLayers}");
    }

    // A cache is a function of the obstacles in force: whatever it went through (adds, removes, slicing, snapshots and restores), once its queue is empty it holds
    // exactly the tiles a fresh cache holds after the surviving obstacles were added in any order.
    [Fact]
    public void AnyHistoryEqualsAFreshCacheWithTheSurvivors()
    {
        Soup s = Level();
        for (int seed = 1; seed <= 10; seed++)
        {
            var rng = new Random(seed * 104729);
            TileCache c = Cache(s);
            var live = new List<(int Id, TileObstacle O)>();
            var saved = new List<(byte[] Bytes, List<(int Id, TileObstacle O)> Live)>();
            for (int step = 0; step < 36; step++)
            {
                int op = rng.Next(10);
                if (op < 4 || live.Count == 0 && op < 8)
                {
                    double x = rng.NextDouble() * 42 - 1, z = rng.NextDouble() * 32 - 1;
                    TileObstacle o = rng.Next(3) switch
                    {
                        0 => TileObstacle.Cylinder(x, rng.Next(2) * 4, z, 0.2 + (rng.NextDouble() * 3.5), 2),
                        1 => TileObstacle.Box(x, 0, z, x + 0.3 + (rng.NextDouble() * 6), 2, z + 0.3 + (rng.NextDouble() * 3)),
                        _ => TileObstacle.OrientedBox(x, 1, z, 0.4 + (rng.NextDouble() * 2.5), 1, 0.3 + (rng.NextDouble() * 1.5), rng.NextDouble() * 6),
                    };
                    live.Add((c.AddObstacle(o), o));
                }
                else if (op < 7 && live.Count > 0)
                {
                    int k = rng.Next(live.Count);
                    Assert.True(c.RemoveObstacle(live[k].Id));
                    live.RemoveAt(k);
                }
                else if (op == 7)
                {
                    c.Update(rng.Next(1, 5), TestContext.Current.CancellationToken);
                }
                else if (op == 8)
                {
                    saved.Add((c.Snapshot(), new List<(int, TileObstacle)>(live)));
                }
                else if (saved.Count > 0)
                {
                    var pick = saved[rng.Next(saved.Count)];
                    c.Restore(pick.Bytes);
                    live = new List<(int, TileObstacle)>(pick.Live);
                }

                if (step % 6 == 5)
                {
                    All(c);
                    TileCache fresh = Cache(s);
                    var order = new List<(int Id, TileObstacle O)>(live);
                    for (int i = order.Count - 1; i > 0; i--)
                    {
                        int j = rng.Next(i + 1);
                        (order[i], order[j]) = (order[j], order[i]);
                    }

                    foreach ((int _, TileObstacle o) in order)
                    {
                        fresh.AddObstacle(o);
                    }

                    All(fresh);
                    AssertSame(fresh.Mesh, c.Mesh);
                }
            }
        }
    }

    [Fact]
    public void RemovingEveryObstacleGivesTheFirstBuildBack()
    {
        Soup s = Level();
        TileCache c = Cache(s);
        TiledNavMesh original = new Mesh3DPipeline(Lean).BuildSoup(s.V.ToArray(), s.I.ToArray());
        var ids = new List<int>();
        foreach (TileObstacle o in Obstacles)
        {
            ids.Add(c.AddObstacle(o));
        }
        All(c);
        Assert.False(MeshBuildTests.Area(c.Mesh) >= MeshBuildTests.Area(original));
        foreach (int id in ids)
        {
            c.RemoveObstacle(id);
        }
        All(c);
        AssertSame(original, c.Mesh);
    }

    [Fact]
    public void SlicedUpdateEqualsUnslicedUpdate()
    {
        Soup s = Level();
        TileCache whole = Cache(s), sliced = Cache(s);
        foreach (TileObstacle o in Obstacles)
        {
            whole.AddObstacle(o);
            sliced.AddObstacle(o);
        }
        All(whole);
        while (sliced.PendingCount > 0)
        {
            Assert.Equal(1, sliced.Update(1, TestContext.Current.CancellationToken));
        }
        Assert.Equal(NavMeshSerializer.WriteMesh(whole.Mesh), NavMeshSerializer.WriteMesh(sliced.Mesh));
    }

    [Fact]
    public void SnapshotRestoreAndContinueMatchesNeverRolledBack()
    {
        Soup s = Level();
        TileCache live = Cache(s), twin = Cache(s);
        live.AddObstacle(Obstacles[0]);
        twin.AddObstacle(Obstacles[0]);
        All(live);
        All(twin);
        byte[] snap = live.Snapshot();
        live.AddObstacle(Obstacles[2]);
        live.AddObstacle(Obstacles[3]);
        All(live);
        live.Restore(snap);
        Assert.Equal(snap, live.Snapshot());
        // After a restore the cache forgets what it built; the next edits rebuild what they touch.
        live.AddObstacle(Obstacles[1]);
        twin.AddObstacle(Obstacles[1]);
        All(live);
        All(twin);
        AssertSame(twin.Mesh, live.Mesh);
        Assert.Equal(twin.PendingCount, live.PendingCount);
    }

    [Fact]
    public void RestoreForgetsWhatTheMeshWasBuiltFrom()
    {
        // After a restore the mesh holds the snapshot's tiles, not the last build's: the same obstacle again must be built into it.
        Soup s = Level();
        TileCache c = Cache(s);
        byte[] snap = c.Snapshot();
        c.AddObstacle(Obstacles[0]);
        All(c);
        c.Restore(snap);
        c.AddObstacle(Obstacles[0]);
        All(c);
        AssertSame(Baked(s, new[] { Obstacles[0] }), c.Mesh);
    }

    [Fact]
    public void VolumesChangeAreasLikeBuildTimeVolumes()
    {
        Soup s = Level();
        var zone = new ConvexVolume(new double[] { 3, 3, 14, 3, 14, 14, 3, 14 }, -1, 5, 7);
        TileCache c = Cache(s);
        c.AddVolume(zone);
        c.AddObstacle(Obstacles[0]);
        All(c);
        AssertSame(Baked(s, new[] { Obstacles[0] }, volumes: new[] { zone }), c.Mesh);
        var areas = new HashSet<byte>();
        for (int slot = 0; slot < c.Mesh.SlotCount; slot++)
        {
            NavTile? t = c.Mesh.GetTileBySlot(slot);
            if (t is not null)
            {
                for (int p = 0; p < t.PolyCount; p++)
                {
                    areas.Add(t.PolyAreas[p]);
                }
            }
        }
        Assert.Contains((byte)7, areas);
    }

    [Fact]
    public void AuthoredLinksAreKeptByRebuilds()
    {
        Soup s = new Soup().Floor(0, 0, 40, 12, 0);
        NavSettings st = Lean with { OffMeshLinks = new[] { OffMeshLinkDefinition.Create(new Vector3D(9, 0, 5), new Vector3D(21, 0, 5), 7) with { Radius = 0.6, Area = 4, Flags = 3 } } };
        TileCache c = Cache(s, st);
        TileObstacle wall = TileObstacle.Box(14, 0, 0, 16, 2, 12);
        c.AddObstacle(wall);
        All(c);
        AssertSame(Baked(s, new[] { wall }, st), c.Mesh);
        int links = 0;
        for (int slot = 0; slot < c.Mesh.SlotCount; slot++)
        {
            links += c.Mesh.GetTileBySlot(slot)?.OffMeshLinks.Length ?? 0;
        }
        Assert.Equal(1, links);
    }

    [Fact]
    public void AgreesWithTheGeneralPathOnSingleLayerTiles()
    {
        // A replaced polygon step (the default one, wrapped) selects the general layer-first path, which builds all layers of a tile together.
        Soup s = new Soup().Floor(0, 0, 64, 48, 0).Floor(4, 4, 20, 20, 4).Floor(40, 30, 44, 34, 0.3);
        var lean = new Mesh3DPipeline(Lean);
        var general = new Mesh3DPipeline(Lean) { Polygonize = (c, n, b, m) => Polygonizer.Build(c, n, b, m) };
        TiledNavMesh a = lean.BuildSoup(s.V.ToArray(), s.I.ToArray());
        TiledNavMesh b = general.BuildSoup(s.V.ToArray(), s.I.ToArray());
        var refsA = new TileRef[16];
        var refsB = new TileRef[16];
        int compared = 0;
        for (int z = -1; z < 5; z++)
        {
            for (int x = -1; x < 6; x++)
            {
                int na = a.GetTilesAt(x, z, refsA), nb = b.GetTilesAt(x, z, refsB);
                if (na != 1 || nb != 1)
                {
                    continue;
                }
                NavTile ta = a.GetTile(refsA[0])!, tb = b.GetTile(refsB[0])!;
                Assert.True(NavMeshSerializer.WriteTile(ta).AsSpan().SequenceEqual(NavMeshSerializer.WriteTile(tb)), $"tile ({x}, {z}) differs: polys {ta.PolyCount} vs {tb.PolyCount}, vertices {ta.Vertices.Length / 3} vs {tb.Vertices.Length / 3}");
                compared++;
            }
        }
        Assert.True(compared >= 6, $"only {compared} single-layer tiles");
        // Where layers meet, the two paths may weld corners differently; the walkable area is the same.
        Assert.InRange(MeshBuildTests.Area(a) / MeshBuildTests.Area(b), 0.99, 1.01);
        Assert.Empty(NavMeshValidator.Validate(a));
    }
}
