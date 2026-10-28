using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using System.Threading;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.FormatFixtures;

/// <summary>
/// What each fixture is read with, and what is recorded about the result. A probe loads the saved bytes with the current
/// readers, carries on from the loaded state (queries, steps, further edits), and writes everything it saw into a transcript;
/// the SHA-256 of the transcript is the recorded expectation. A probe never writes a format back out and compares bytes,
/// because a later release is free to write a newer version marker. Probes use only the public API and the other
/// fixtures, so what they exercise stays the same from release to release.
/// </summary>
internal static class FixtureProbes
{
    /// <summary>Every fixture name, in manifest order. A new format adds a name here and a probe below, never removes one.</summary>
    internal static readonly string[] Names =
    [
        "navmesh-mesh", "navmesh-tile", "navmesh-snapshot-full", "navmesh-snapshot-referenced",
        "tilecache-full", "tilecache-referenced",
        "crowd-state", "crowd-with-mesh-full", "crowd-with-mesh-referenced",
        "path-corridor", "path-follower",
        "grid-square", "grid-hex", "grid-voxel",
        "search-grid-astar", "search-grid-astar-generic", "search-grid-jps", "search-grid-jpsplus",
        "search-astar", "search-jump-block", "search-jump-table", "search-theta", "search-graph",
        "replanner-dstar", "replanner-lpa", "replanner-hex-dstar",
        "flow-square-partial", "flow-square-repair", "flow-hex-partial",
        "graph", "landmarks",
    ];

    // ------------------------------------------------------------------ shared pieces

    /// <summary>Little-endian transcript builder.</summary>
    internal sealed class Tx
    {
        private readonly List<byte> _bytes = [];

        internal void I(long v)
        {
            for (int i = 0; i < 8; i++)
            {
                _bytes.Add((byte)(v >> (8 * i)));
            }
        }

        internal void D(double v) => I(BitConverter.DoubleToInt64Bits(v));

        internal void F(float v) => I(BitConverter.SingleToInt32Bits(v));

        internal void B(ReadOnlySpan<byte> b)
        {
            I(b.Length);
            _bytes.AddRange(b.ToArray());
        }

        internal byte[] Digest() => SHA256.HashData(_bytes.ToArray());
    }

    /// <summary>A tiny seeded generator (SplitMix64) so probes do not depend on any library type.</summary>
    internal struct Mix
    {
        private ulong _s;

        internal Mix(ulong seed) => _s = seed;

        internal int Next(int n)
        {
            _s += 0x9E3779B97F4A7C15UL;
            ulong z = _s;
            z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
            z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
            return (int)((z ^ (z >> 31)) % (ulong)n);
        }
    }

    /// <summary>Several byte arrays in one file: a little-endian int count, then each part as an int length and its bytes.</summary>
    internal static byte[] Pack(params byte[][] parts)
    {
        var o = new List<byte>();
        void Int(int v)
        {
            for (int i = 0; i < 4; i++)
            {
                o.Add((byte)(v >> (8 * i)));
            }
        }
        Int(parts.Length);
        foreach (byte[] p in parts)
        {
            Int(p.Length);
            o.AddRange(p);
        }
        return o.ToArray();
    }

    internal static byte[][] Unpack(byte[] bundle)
    {
        int pos = 0;
        int Int()
        {
            int v = BitConverter.ToInt32(bundle, pos);
            pos += 4;
            return v;
        }
        var parts = new byte[Int()][];
        for (int i = 0; i < parts.Length; i++)
        {
            parts[i] = bundle.AsSpan(pos + 4, BitConverter.ToInt32(bundle, pos)).ToArray();
            pos += 4 + parts[i].Length;
        }
        return parts;
    }

    // ------------------------------------------------------------------ scene constants shared with the generator

    internal const ulong CrowdWorldSeed = 14;

    internal static readonly GridOptions[] SearchOptions =
    [
        new GridOptions { Algorithm = GridAlgorithm.AStar },
        new GridOptions { Algorithm = GridAlgorithm.AStar, SpecializedEngines = false },
        new GridOptions { Algorithm = GridAlgorithm.Jps },
        new GridOptions { Algorithm = GridAlgorithm.JpsPlus },
    ];

    internal static GridOptions ReplanOptions(GridAlgorithm algorithm) => new() { Algorithm = algorithm, RecordExpansions = true };

    internal static GridOptions FlowOptions(bool repair) => new() { IterationBudget = repair ? int.MaxValue : 60 };

    internal static readonly Cell[] FlowGoals = [new Cell(3, 4), new Cell(20, 15)];

    internal static readonly double[] TileCacheSoupVertices =
    [
        0, 0, 0, 0, 0, 18, 24, 0, 18, 24, 0, 0, // ground
        4, 3, 4, 4, 3, 12, 12, 3, 12, 12, 3, 4, // platform
    ];

    internal static readonly int[] TileCacheSoupIndices = [0, 1, 2, 0, 2, 3, 4, 5, 6, 4, 6, 7];

    internal static NavSettings TileCacheSettings() => new()
    {
        Partition = PartitionType.Layers, CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 0,
    };

    // ------------------------------------------------------------------ dispatch

    /// <summary>Loads fixture <paramref name="name"/> with the current readers and returns the SHA-256 of what was observed.</summary>
    internal static byte[] Run(string name, Func<string, byte[]> read)
    {
        byte[] bytes = read(name);
        var tx = new Tx();
        switch (name)
        {
            case "navmesh-mesh": MeshProbe(NavMeshSerializer.ReadMesh(bytes), tx); break;
            case "navmesh-tile": TileProbe(read, bytes, tx); break;
            case "navmesh-snapshot-full": MeshProbe(NavMeshSnapshot.Restore(bytes), tx); break;
            case "navmesh-snapshot-referenced": ReferencedMeshProbe(read, bytes, tx); break;
            case "tilecache-full": TileCacheProbe(bytes, false, tx); break;
            case "tilecache-referenced": TileCacheProbe(bytes, true, tx); break;
            case "crowd-state": CrowdStateProbe(read, bytes, tx); break;
            case "crowd-with-mesh-full": CrowdWithMeshProbe(read, bytes, false, tx); break;
            case "crowd-with-mesh-referenced": CrowdWithMeshProbe(read, bytes, true, tx); break;
            case "path-corridor": CorridorProbe(read, bytes, tx); break;
            case "path-follower": FollowerProbe(read, bytes, tx); break;
            case "grid-square": GridProbe(Grid.FromState(bytes), tx); break;
            case "grid-hex": HexProbe(HexGrid.FromState(bytes), tx); break;
            case "grid-voxel": VoxelProbe(VoxelGrid.FromState(bytes), tx); break;
            case "search-grid-astar": GridSearchProbe(bytes, 0, tx); break;
            case "search-grid-astar-generic": GridSearchProbe(bytes, 1, tx); break;
            case "search-grid-jps": GridSearchProbe(bytes, 2, tx); break;
            case "search-grid-jpsplus": GridSearchProbe(bytes, 3, tx); break;
            case "search-astar": GenericSearchProbe(bytes, 0, tx); break;
            case "search-jump-block": GenericSearchProbe(bytes, 1, tx); break;
            case "search-jump-table": GenericSearchProbe(bytes, 2, tx); break;
            case "search-theta": GenericSearchProbe(bytes, 3, tx); break;
            case "search-graph": GenericSearchProbe(bytes, 4, tx); break;
            case "replanner-dstar": ReplannerProbe(bytes, GridAlgorithm.DStarLite, false, tx); break;
            case "replanner-lpa": ReplannerProbe(bytes, GridAlgorithm.LpaStar, false, tx); break;
            case "replanner-hex-dstar": ReplannerProbe(bytes, GridAlgorithm.DStarLite, true, tx); break;
            case "flow-square-partial": FlowProbe(bytes, false, false, tx); break;
            case "flow-square-repair": FlowProbe(bytes, false, true, tx); break;
            case "flow-hex-partial": FlowProbe(bytes, true, false, tx); break;
            case "graph": GraphProbe(Graph.FromState(bytes), null, tx); break;
            case "landmarks": LandmarksProbe(bytes, tx); break;
            default: throw new ArgumentException("No probe for fixture " + name, nameof(name));
        }
        return tx.Digest();
    }

    // ------------------------------------------------------------------ navmesh

    private static void MeshProbe(TiledNavMesh m, Tx tx)
    {
        tx.I(m.TileCount);
        tx.I(m.SlotCount);
        for (int s = 0; s < m.SlotCount; s++)
        {
            tx.I(m.GetSlotSalt(s));
            if (m.GetTileBySlot(s) is not { } t)
            {
                tx.I(-1);
                continue;
            }
            TileContent(t, tx);
            for (int p = 0; p < t.PolyCount; p++)
            {
                tx.I(m.GetLinks(m.GetSlotRef(s).Poly(p)).Length);
            }
            for (int l = 0; l < t.OffMeshLinks.Length; l++)
            {
                tx.I(m.IsOffMeshLinkEnabled(m.GetSlotRef(s), l) ? 1 : 0);
            }
        }
        Queries(m, tx);
    }

    private static void TileContent(NavTile t, Tx tx)
    {
        tx.I(t.TileX);
        tx.I(t.TileZ);
        tx.I(t.Layer);
        tx.D(t.OriginX);
        tx.D(t.OriginY);
        tx.D(t.OriginZ);
        tx.I(t.PolyCount);
        foreach (float f in t.Vertices)
        {
            tx.F(f);
        }
        tx.B(t.PolyVertexCounts);
        foreach (ushort u in t.PolyIndices)
        {
            tx.I(u);
        }
        foreach (int n in t.EdgeNeighbors)
        {
            tx.I(n);
        }
        tx.B(t.EdgePortalSides);
        tx.B(t.PolyAreas);
        foreach (ushort u in t.PolyFlags)
        {
            tx.I(u);
        }
        tx.I(t.DetailMeshes.Length);
        foreach (float f in t.DetailVertices)
        {
            tx.F(f);
        }
        tx.B(t.DetailTriangles);
        tx.I(t.OffMeshLinks.Length);
    }

    private static void Queries(TiledNavMesh m, Tx tx)
    {
        var q = new NavMeshQuery(m);
        var r = new Mix(77);
        var visited = new PolyRef[64];
        // Query spots cover the mesh's own extent, so most of them land on polygons.
        double x0 = double.MaxValue, z0 = double.MaxValue, x1 = double.MinValue, z1 = double.MinValue, y1 = double.MinValue;
        for (int s = 0; s < m.SlotCount; s++)
        {
            if (m.GetTileBySlot(s) is { } t)
            {
                x0 = Math.Min(x0, t.OriginX + t.BoundsMin.X);
                x1 = Math.Max(x1, t.OriginX + t.BoundsMax.X);
                z0 = Math.Min(z0, t.OriginZ + t.BoundsMin.Z);
                z1 = Math.Max(z1, t.OriginZ + t.BoundsMax.Z);
                y1 = Math.Max(y1, t.OriginY + t.BoundsMax.Y);
            }
        }
        double U() => r.Next(1 << 20) / (double)(1 << 20);
        Vector3D Spot() => new(x0 + (U() * (x1 - x0)), (U() * y1) - 0.25, z0 + (U() * (z1 - z0)));
        for (int i = 0; i < 16; i++)
        {
            Vector3D a = Spot();
            Vector3D b = Spot();
            NearestPoly n = q.FindNearestPoly(a, new Vector3D(2, 5, 2));
            tx.I((long)n.Poly.Value);
            tx.D(n.DistanceSquared);
            var o = new PathOptions
            {
                SearchExtents = new Vector3D(2, 5, 2),
                Filter = i % 3 == 1 ? new QueryFilter { ExcludeFlags = 2 }.WithAreaCost(1, 3).WithAreaCost(9, 2) : null,
            };
            PathResult pr = q.FindPath(a, b, o);
            tx.I((int)pr.Detail);
            tx.D(pr.Cost);
            foreach (PolyRef c in pr.Corridor)
            {
                tx.I((long)c.Value);
            }
            foreach (StraightPathPoint pt in pr.Points)
            {
                tx.D(pt.Position.X);
                tx.D(pt.Position.Y);
                tx.D(pt.Position.Z);
                tx.I((long)pt.Poly.Value);
            }
            if (n.Poly.IsSet)
            {
                RaycastResult rr = q.Raycast(n.Poly, n.Point, b, null, visited, out int vc);
                tx.I((int)rr.Status);
                tx.D(rr.T);
                tx.I(vc);
                for (int k = 0; k < vc; k++)
                {
                    tx.I((long)visited[k].Value);
                }
            }
        }
    }

    private static void TileProbe(Func<string, byte[]> read, byte[] bytes, Tx tx)
    {
        TiledNavMesh mesh = NavMeshSerializer.ReadMesh(read("navmesh-mesh"));
        NavTile t = NavMeshSerializer.ReadTile(bytes, mesh.Params);
        TileContent(t, tx);
        // The tile joins a fresh mesh of the same parameters and answers queries there.
        var fresh = new TiledNavMesh(mesh.Params);
        fresh.AddTile(t);
        tx.I(fresh.TileCount);
        Queries(fresh, tx);
    }

    private static TileLibrary Library(TiledNavMesh target, byte[][] tiles)
    {
        var lib = new TileLibrary();
        for (int i = 1; i < tiles.Length; i++)
        {
            lib.Add(NavMeshSerializer.ReadTile(tiles[i], target.Params));
        }
        return lib;
    }

    private static void ReferencedMeshProbe(Func<string, byte[]> read, byte[] bundle, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // the snapshot, then every tile it names
        TiledNavMesh same = NavMeshSerializer.ReadMesh(read("navmesh-mesh")); // the mesh parameters the tiles were read with
        MeshProbe(NavMeshSnapshot.Restore(parts[0], Library(same, parts)), tx);
    }

    private static void TileCacheProbe(byte[] bundle, bool referenced, Tx tx)
    {
        byte[][] parts = Unpack(bundle);
        TileCache cache = new Mesh3DPipeline(TileCacheSettings()).BuildSoupCached(TileCacheSoupVertices, TileCacheSoupIndices);
        TileLibrary? lib = null;
        if (referenced)
        {
            lib = new TileLibrary();
            for (int i = 1; i < parts.Length; i++)
            {
                lib.Add(NavMeshSerializer.ReadTile(parts[i], cache.Mesh.Params));
            }
        }
        cache.Restore(parts[0], lib);
        tx.I(cache.ObstacleCount);
        tx.I(cache.VolumeCount);
        tx.I(cache.PendingCount);
        MeshProbe(cache.Mesh, tx);
        cache.UpdateAll(CancellationToken.None);
        tx.I(cache.PendingCount);
        MeshProbe(cache.Mesh, tx);
    }

    // ------------------------------------------------------------------ crowds

    private static Crowd NewCrowd(TiledNavMesh mesh) => new(mesh, new CrowdOptions { MaxThreads = 1 });

    private static void CrowdDigest(Crowd crowd, Tx tx)
    {
        var agents = new List<Agent>(crowd);
        tx.I(agents.Count);
        foreach (Agent a in agents)
        {
            tx.I(a.Id);
            tx.D(a.Position.X);
            tx.D(a.Position.Y);
            tx.D(a.Position.Z);
            tx.D(a.Velocity.X);
            tx.D(a.Velocity.Z);
            tx.D(a.Destination.X);
            tx.D(a.Destination.Z);
            tx.I((int)a.State);
            tx.D(a.RemainingDistance);
        }
    }

    private static void CrowdStateProbe(Func<string, byte[]> read, byte[] state, Tx tx)
    {
        Crowd crowd = NewCrowd(NavMeshSerializer.ReadMesh(read("navmesh-mesh")));
        crowd.RestoreState(state);
        CrowdDigest(crowd, tx);
        for (int s = 0; s < 60; s++)
        {
            crowd.Step(1.0 / 30.0);
        }
        CrowdDigest(crowd, tx);
    }

    private static void CrowdWithMeshProbe(Func<string, byte[]> read, byte[] bundle, bool referenced, Tx tx)
    {
        byte[][] parts = Unpack(bundle);
        TiledNavMesh mesh = NavMeshSerializer.ReadMesh(read("navmesh-mesh"));
        Crowd crowd = NewCrowd(mesh);
        TileLibrary? lib = referenced ? Library(mesh, parts) : null;
        crowd.RestoreStateWithMesh(parts[0], lib);
        MeshProbe(crowd.Mesh, tx);
        CrowdDigest(crowd, tx);
        for (int s = 0; s < 60; s++)
        {
            crowd.Step(1.0 / 30.0);
        }
        CrowdDigest(crowd, tx);
    }

    private static void CorridorProbe(Func<string, byte[]> read, byte[] bytes, Tx tx)
    {
        TiledNavMesh mesh = NavMeshSerializer.ReadMesh(read("navmesh-mesh"));
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        PathCorridor c = PathCorridor.Read(bytes, out int consumed);
        tx.I(consumed);
        CorridorDigest(c, tx);
        var corners = new StraightPathPoint[64];
        tx.I((int)c.FindCorners(q, corners, out int n));
        tx.I(n);
        for (int i = 0; i < n; i++)
        {
            tx.D(corners[i].Position.X);
            tx.D(corners[i].Position.Y);
            tx.D(corners[i].Position.Z);
        }
        for (int move = 0; move < 12; move++)
        {
            c.MovePosition(q, Vector3D.Lerp(c.Position, corners[n > 1 ? 1 : 0].Position, 0.5));
            c.FindCorners(q, corners, out n);
        }
        CorridorDigest(c, tx);
    }

    private static void CorridorDigest(PathCorridor c, Tx tx)
    {
        tx.I(c.Capacity);
        tx.I(c.Count);
        foreach (PolyRef p in c.Path)
        {
            tx.I((long)p.Value);
        }
        tx.D(c.Position.X);
        tx.D(c.Position.Y);
        tx.D(c.Position.Z);
        tx.D(c.Target.X);
        tx.D(c.Target.Y);
        tx.D(c.Target.Z);
        tx.I((long)c.TargetPoly.Value);
        tx.I(c.IsComplete ? 1 : 0);
        tx.I(c.NeedsReplan ? 1 : 0);
    }

    private static void FollowerDigest(PathFollower f, Tx tx)
    {
        tx.I((int)f.State);
        tx.I(f.ReachedTarget ? 1 : 0);
        tx.D(f.Position.X);
        tx.D(f.Position.Y);
        tx.D(f.Position.Z);
        tx.D(f.Velocity.X);
        tx.D(f.Velocity.Z);
        tx.D(f.Destination.X);
        tx.D(f.Destination.Z);
        tx.D(f.RemainingDistance);
        tx.I(f.StepCount);
        CorridorDigest(f.Corridor, tx);
    }

    private static void FollowerProbe(Func<string, byte[]> read, byte[] bytes, Tx tx)
    {
        TiledNavMesh mesh = NavMeshSerializer.ReadMesh(read("navmesh-mesh"));
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        var f = new PathFollower(FollowerOptions.Humanoid);
        f.Read(bytes);
        FollowerDigest(f, tx);
        for (int s = 0; s < 90; s++)
        {
            f.Step(q, 1.0 / 30.0);
        }
        FollowerDigest(f, tx);
    }

    // ------------------------------------------------------------------ grids

    private static void PathDigest(GridPath p, Tx tx)
    {
        tx.I((int)p.Status);
        tx.D(p.Cost);
        tx.I(p.Cells.Length);
        foreach (Cell c in p.Cells)
        {
            tx.I(c.X);
            tx.I(c.Y);
        }
        tx.I(p.ExpandedCount);
    }

    private static void GridContent(Grid g, Tx tx)
    {
        tx.I(g.Width);
        tx.I(g.Height);
        tx.I(g.Version);
        tx.D(g.Space.OriginX);
        tx.D(g.Space.OriginY);
        tx.D(g.Space.CellSize);
        tx.I(g.Areas.Version);
        for (int a = 0; a < 8; a++)
        {
            tx.D(g.Areas[(byte)a]);
        }
        tx.B(g.Cells);
    }

    private static void GridProbe(Grid g, Tx tx)
    {
        GridContent(g, tx);
        var r = new Mix(5);
        foreach (GridAlgorithm alg in new[] { GridAlgorithm.AStar, GridAlgorithm.Jps, GridAlgorithm.JpsPlus, GridAlgorithm.ThetaStar, GridAlgorithm.Hpa })
        {
            for (int i = 0; i < 6; i++)
            {
                var o = new GridOptions { Algorithm = alg, HpaChunkSize = 4 };
                PathDigest(g.FindPath(new Cell(r.Next(g.Width), r.Next(g.Height)), new Cell(r.Next(g.Width), r.Next(g.Height)), o), tx);
            }
        }
        // Edits after the load behave like edits to any grid.
        g.SetArea(2, 2, AreaCosts.Wall);
        g.SetRect(5, 5, 3, 2, 1);
        tx.I(g.Version);
        PathDigest(g.FindPath(new Cell(0, 0), new Cell(g.Width - 1, g.Height - 1)), tx);
    }

    private static void HexProbe(HexGrid g, Tx tx)
    {
        tx.I(g.Width);
        tx.I(g.Height);
        tx.I(g.Version);
        tx.I((int)g.Layout);
        tx.D(g.Space.OriginX);
        tx.D(g.Space.OriginY);
        tx.D(g.Space.CellSize);
        tx.I(g.Areas.Version);
        for (int a = 0; a < 8; a++)
        {
            tx.D(g.Areas[(byte)a]);
        }
        tx.B(g.Cells);
        var r = new Mix(6);
        for (int i = 0; i < 12; i++)
        {
            PathDigest(g.FindPath(new Cell(r.Next(g.Width), r.Next(g.Height)), new Cell(r.Next(g.Width), r.Next(g.Height))), tx);
        }
        g.SetArea(1, 1, AreaCosts.Wall);
        tx.I(g.Version);
    }

    private static void VoxelProbe(VoxelGrid g, Tx tx)
    {
        tx.I(g.SizeX);
        tx.I(g.SizeY);
        tx.I(g.SizeZ);
        tx.I(g.Version);
        tx.I(g.AllocatedChunks);
        tx.I(g.DefaultArea);
        tx.D(g.Space.OriginX);
        tx.D(g.Space.OriginY);
        tx.D(g.Space.OriginZ);
        tx.D(g.Space.CellSize);
        tx.I(g.Areas.Version);
        for (int a = 0; a < 8; a++)
        {
            tx.D(g.Areas[(byte)a]);
        }
        for (int z = 0; z < g.SizeZ; z++)
        {
            for (int y = 0; y < g.SizeY; y++)
            {
                for (int x = 0; x < g.SizeX; x++)
                {
                    tx.I(g.GetArea(x, y, z));
                }
            }
        }
        var r = new Mix(7);
        for (int i = 0; i < 6; i++)
        {
            var a = new VoxelCell(r.Next(g.SizeX), r.Next(g.SizeY), r.Next(g.SizeZ));
            var b = new VoxelCell(r.Next(g.SizeX), r.Next(g.SizeY), r.Next(g.SizeZ));
            VoxelPath p = g.FindPath(a, b);
            tx.I((int)p.Status);
            tx.D(p.Cost);
            foreach (VoxelCell c in p.Cells)
            {
                tx.I(c.X);
                tx.I(c.Y);
                tx.I(c.Z);
            }
        }
    }

    // ------------------------------------------------------------------ search states

    private static void FinishGridSearch(GridSearch s, Tx tx)
    {
        int guard = 0;
        while (s.Status == PathStatus.InProgress && guard++ < 100000)
        {
            tx.I((int)s.Step(25));
        }
        tx.I((int)s.Status);
        tx.D(s.PathCost);
        tx.I(s.ExpandedCount);
        tx.I(s.Iterations);
        if (s.Status == PathStatus.Found)
        {
            var path = new Cell[s.PathLength];
            s.CopyPath(path);
            foreach (Cell c in path)
            {
                tx.I(c.X);
                tx.I(c.Y);
            }
        }
    }

    private static void GridSearchProbe(byte[] bundle, int which, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // grid, search
        Grid g = Grid.FromState(parts[0]);
        GridSearch s = g.CreateSearch(SearchOptions[which]);
        s.RestoreState(parts[1]);
        tx.I((int)s.Status);
        tx.I(s.ExpandedCount);
        tx.I(s.Iterations);
        tx.I(s.Start.X);
        tx.I(s.Start.Y);
        tx.I(s.Goal.X);
        tx.I(s.Goal.Y);
        FinishGridSearch(s, tx);
    }

    /// <summary>The start and goal node of the tier-2 searches over a width x height grid (the generator and probe agree).</summary>
    internal static (int Start, int Goal) SearchEnds(Grid g)
    {
        var r = new Mix(21);
        var open = new List<int>();
        for (int i = 0; i < g.CellCount; i++)
        {
            if (g.Cells[i] != AreaCosts.Wall)
            {
                open.Add(i);
            }
        }
        return (open[r.Next(open.Count)], open[r.Next(open.Count)]);
    }

    private static void PutLoop<TSearch>(TSearch s, Func<TSearch, SearchStatus> status, Func<TSearch, int, SearchStatus> step, Action<TSearch, Tx> tail, Tx tx)
    {
        int guard = 0;
        while (status(s) == SearchStatus.Running && guard++ < 100000)
        {
            tx.I((int)step(s, 25));
        }
        tx.I((int)status(s));
        tail(s, tx);
    }

    private static void GenericSearchProbe(byte[] bundle, int which, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // grid, graph, search
        Grid g = Grid.FromState(parts[0]);
        Graph gr = Graph.FromState(parts[1]);
        CsrGraph csr = gr.Compile();
        int w = g.Width;
        GridSearchBuffers Buf() => new(Math.Max(g.CellCount, csr.NodeCount)) { RecordExpansions = true };
        using var index = new JpsPlusIndex(g);
        using var map = new BlockMap(g);
        switch (which)
        {
            case 0:
            {
                var s = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(w), Buf());
                s.RestoreState(parts[2]);
                PutLoop(s, x => x.Status, (x, n) => x.Step(n), (x, t) => Tail(t, x.Status, x.PathCost, x.ExpandedCount, x.Iterations, x.PathLength, p => x.CopyPath(p)), tx);
                break;
            }
            case 1:
            {
                var s = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(map), new OctileHeuristic(w), 1.0, Buf());
                s.RestoreState(parts[2]);
                PutLoop(s, x => x.Status, (x, n) => x.Step(n), (x, t) => Tail(t, x.Status, x.PathCost, x.ExpandedCount, x.Iterations, x.PathLength, p => x.CopyPath(p)), tx);
                break;
            }
            case 2:
            {
                var s = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(w), 1.0, Buf());
                s.RestoreState(parts[2]);
                PutLoop(s, x => x.Status, (x, n) => x.Step(n), (x, t) => Tail(t, x.Status, x.PathCost, x.ExpandedCount, x.Iterations, x.PathLength, p => x.CopyPath(p)), tx);
                break;
            }
            case 3:
            {
                var s = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(w), ThetaVariant.LazyRequeue, 1.0, Buf());
                s.RestoreState(parts[2]);
                PutLoop(s, x => x.Status, (x, n) => x.Step(n), (x, t) => Tail(t, x.Status, x.PathCost, x.ExpandedCount, x.Iterations, x.PathLength, p => x.CopyPath(p)), tx);
                break;
            }
            default:
            {
                var s = new GraphAStar<DefaultGraphCost, ZeroGraphHeuristic>(csr, default, default, Buf());
                s.RestoreState(parts[2]);
                PutLoop(s, x => x.Status, (x, n) => x.Step(n), (x, t) => Tail(t, x.Status, x.PathCost, x.ExpandedCount, x.Iterations, x.PathLength, p => x.CopyPath(p)), tx);
                break;
            }
        }
    }

    private static void Tail(Tx tx, SearchStatus status, double cost, int expanded, int iterations, Func<int> pathLength, Action<int[]> copy)
    {
        tx.D(cost);
        tx.I(expanded);
        tx.I(iterations);
        if (status == SearchStatus.Found)
        {
            var p = new int[pathLength()];
            tx.I(p.Length);
            copy(p);
            foreach (int n in p)
            {
                tx.I(n);
            }
        }
    }

    // ------------------------------------------------------------------ replanners and flow fields

    private static void ReplannerProbe(byte[] bundle, GridAlgorithm algorithm, bool hex, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // grid, replanner
        GridOptions o = ReplanOptions(algorithm);
        if (hex)
        {
            HexGrid g = HexGrid.FromState(parts[0]);
            HexReplanner rep = g.CreateReplanner(new Cell(1, 1), new Cell(g.Width - 2, g.Height - 2), o);
            rep.RestoreState(parts[1]);
            tx.I(rep.Start.X);
            tx.I(rep.Start.Y);
            tx.I(rep.Goal.X);
            tx.I(rep.Goal.Y);
            PathDigest(rep.Replan(), tx);
            g.SetArea(3, 3, AreaCosts.Wall);
            g.SetArea(4, 3, AreaCosts.Wall);
            PathDigest(rep.Replan(), tx);
            rep.MoveStart(new Cell(2, 2));
            PathDigest(rep.Replan(), tx);
        }
        else
        {
            Grid g = Grid.FromState(parts[0]);
            GridReplanner rep = g.CreateReplanner(new Cell(1, 1), new Cell(g.Width - 2, g.Height - 2), o);
            rep.RestoreState(parts[1]);
            tx.I(rep.Start.X);
            tx.I(rep.Start.Y);
            tx.I(rep.Goal.X);
            tx.I(rep.Goal.Y);
            PathDigest(rep.Replan(), tx);
            g.SetArea(3, 3, AreaCosts.Wall);
            g.SetRect(8, 8, 3, 2, AreaCosts.Wall);
            PathDigest(rep.Replan(), tx);
            rep.MoveStart(new Cell(2, 2));
            PathDigest(rep.Replan(), tx);
        }
    }

    private static void FieldDigest(GridFlowField f, Tx tx)
    {
        tx.I((int)f.Status);
        tx.I(f.Settled);
        foreach (double c in f.Costs)
        {
            tx.D(c);
        }
        foreach (int n in f.Next)
        {
            tx.I(n);
        }
    }

    private static void FlowProbe(byte[] bundle, bool hex, bool repair, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // grid, field
        if (hex)
        {
            HexGrid g = HexGrid.FromState(parts[0]);
            GridFlowField f = g.CreateFlowField(FlowGoals, FlowOptions(repair));
            f.RestoreState(parts[1]);
            FieldDigest(f, tx);
            f.Run();
            FieldDigest(f, tx);
            g.SetArea(6, 6, AreaCosts.Wall);
            f.Refresh();
            FieldDigest(f, tx);
        }
        else
        {
            Grid g = Grid.FromState(parts[0]);
            GridFlowField f = g.CreateFlowField(FlowGoals, FlowOptions(repair));
            f.RestoreState(parts[1]);
            FieldDigest(f, tx);
            f.Run();
            FieldDigest(f, tx);
            g.SetArea(6, 6, AreaCosts.Wall);
            f.Refresh();
            FieldDigest(f, tx);
        }
    }

    // ------------------------------------------------------------------ graphs

    private static void GraphProbe(Graph g, Landmarks? lm, Tx tx)
    {
        tx.I(g.NodeCount);
        tx.I(g.EdgeCount);
        tx.I((long)g.Compile().Fingerprint());
        var r = new Mix(8);
        for (int i = 0; i < 12; i++)
        {
            int a = r.Next(g.NodeCount);
            int b = r.Next(g.NodeCount);
            foreach (GraphOptions o in lm is null
                ? new[] { new GraphOptions { RecordExpansions = true }, GraphOptions.Bidirectional }
                : new[] { new GraphOptions { RecordExpansions = true }, GraphOptions.WithLandmarks(lm) })
            {
                GraphPath p = g.FindPath(a, b, o);
                tx.I((int)p.Status);
                tx.D(p.Cost);
                foreach (int n in p.Nodes)
                {
                    tx.I(n);
                }
            }
        }
    }

    private static void LandmarksProbe(byte[] bundle, Tx tx)
    {
        byte[][] parts = Unpack(bundle); // graph, landmarks
        Graph g = Graph.FromState(parts[0]);
        Landmarks lm = Landmarks.Deserialize(parts[1], g.Compile());
        GraphProbe(g, lm, tx);
    }
}
