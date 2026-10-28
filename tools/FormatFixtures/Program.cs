using System;
using RecursiveAI.OnePath;
using System.Collections.Generic;
using System.IO;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.FormatFixtures;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;
using RecursiveAI.OneMath;

// Writes the format fixtures for the release this tool is built from, into a directory named for that release.
//   dotnet run --project tools/FormatFixtures -- tests/OnePath.Tests/Fixtures/<release>
// The files of a published release are never regenerated: a later release adds a directory of its own, and the test
// reads every directory. The scenes are small generated worlds; nothing here comes from outside the repository.
internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.Error.WriteLine("usage: FormatFixtures <output directory>");
            return 2;
        }
        string dir = args[0];
        Directory.CreateDirectory(dir);
        var files = new Dictionary<string, byte[]>();
        Scenes.Make(files);
        foreach (string name in FixtureProbes.Names)
        {
            if (!files.ContainsKey(name))
            {
                Console.Error.WriteLine("no scene for fixture " + name);
                return 1;
            }
        }
        if (files.Count != FixtureProbes.Names.Length)
        {
            Console.Error.WriteLine("scenes wrote a fixture that has no probe");
            return 1;
        }
        var manifest = new StringBuilder();
        manifest.Append("# name\tbytes\tfile-sha256\tprobe-sha256\twritten-by\n");
        int total = 0;
        foreach (string name in FixtureProbes.Names)
        {
            byte[] bytes = files[name];
            File.WriteAllBytes(Path.Combine(dir, name + ".bin"), bytes);
            byte[] probe = FixtureProbes.Run(name, n => files[n]);
            manifest.Append($"{name}\t{bytes.Length}\t{Convert.ToHexString(SHA256.HashData(bytes)).ToLowerInvariant()}\t{Convert.ToHexString(probe).ToLowerInvariant()}\t{Scenes.Writer[name]}\n");
            total += bytes.Length;
            Console.WriteLine($"{name,-30} {bytes.Length,8} bytes");
        }
        File.WriteAllText(Path.Combine(dir, "fixtures.tsv"), manifest.ToString());
        Console.WriteLine($"{files.Count} fixtures, {total} bytes");
        return 0;
    }
}

internal static class Scenes
{
    internal static readonly Dictionary<string, string> Writer = new()
    {
        ["navmesh-mesh"] = "NavMeshSerializer.WriteMesh",
        ["navmesh-tile"] = "NavMeshSerializer.WriteTile",
        ["navmesh-snapshot-full"] = "TiledNavMesh.Snapshot(Full)",
        ["navmesh-snapshot-referenced"] = "TiledNavMesh.Snapshot(Referenced)",
        ["tilecache-full"] = "TileCache.Snapshot(Full)",
        ["tilecache-referenced"] = "TileCache.Snapshot(Referenced)",
        ["crowd-state"] = "Crowd.SaveState",
        ["crowd-with-mesh-full"] = "Crowd.SaveStateWithMesh(Full)",
        ["crowd-with-mesh-referenced"] = "Crowd.SaveStateWithMesh(Referenced)",
        ["path-corridor"] = "PathCorridor.ToBytes",
        ["path-follower"] = "PathFollower.ToBytes",
        ["grid-square"] = "Grid.SaveState",
        ["grid-hex"] = "HexGrid.SaveState",
        ["grid-voxel"] = "VoxelGrid.SaveState",
        ["search-grid-astar"] = "GridSearch.SaveState (specialised A*)",
        ["search-grid-astar-generic"] = "GridSearch.SaveState (generic A*)",
        ["search-grid-jps"] = "GridSearch.SaveState (JPS)",
        ["search-grid-jpsplus"] = "GridSearch.SaveState (JPS+)",
        ["search-astar"] = "AStar.SaveState",
        ["search-jump-block"] = "JumpSearch.SaveState (block jumper)",
        ["search-jump-table"] = "JumpSearch.SaveState (JPS+ table jumper)",
        ["search-theta"] = "ThetaSearch.SaveState",
        ["search-graph"] = "GraphAStar.SaveState",
        ["replanner-dstar"] = "GridReplanner.SaveState (D* Lite)",
        ["replanner-lpa"] = "GridReplanner.SaveState (LPA*)",
        ["replanner-hex-dstar"] = "HexReplanner.SaveState (D* Lite)",
        ["flow-square-partial"] = "GridFlowField.SaveState (square, part way)",
        ["flow-square-repair"] = "GridFlowField.SaveState (square, half-done repair)",
        ["flow-hex-partial"] = "GridFlowField.SaveState (hex, part way)",
        ["graph"] = "Graph.SaveState",
        ["landmarks"] = "Landmarks.Serialize",
    };

    private static TiledNavMesh BaseMesh() => NavMeshWorlds.RandomMesh(FixtureProbes.CrowdWorldSeed, 0, 0, 2, 1, 2, detail: false, offMesh: 2);

    // Edits a host makes to a loaded mesh: a tile swapped for another (its bytes join the library), flags and areas changed, a link switched off.
    private static void Edit(TiledNavMesh m, List<byte[]> library)
    {
        NavTile replacement = NavMeshWorlds.MakeTile(m.Params, FixtureProbes.CrowdWorldSeed + 1, 1, 0, 1, detail: true, offMesh: 1);
        library.Add(NavMeshSerializer.WriteTile(replacement));
        m.ReplaceTile(replacement);
        for (int s = 0; s < m.SlotCount; s++)
        {
            if (m.GetTileBySlot(s) is { PolyCount: > 3 } t)
            {
                m.SetPolyFlags(m.GetSlotRef(s).Poly(1), 5);
                m.SetPolyArea(m.GetSlotRef(s).Poly(2), 3);
                if (t.OffMeshLinks.Length > 0)
                {
                    m.SetOffMeshLinkEnabled(m.GetSlotRef(s), 0, false);
                }
            }
        }
    }

    private static Crowd Populate(TiledNavMesh mesh)
    {
        var crowd = new Crowd(mesh, new CrowdOptions { MaxThreads = 1 });
        var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
        List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
        var r = new NavMeshWorlds.Rng(5);
        for (int tries = 0, n = 0; n < 8 && tries < 400; tries++)
        {
            Vector3D a = CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r);
            Vector3D b = CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r);
            if (q.FindPath(a, b).Status != PathStatus.Found)
            {
                continue;
            }
            try
            {
                crowd.Add(a, Agent.Small).SetDestination(b);
                n++;
            }
            catch (ArgumentException)
            {
            }
        }
        return crowd;
    }

    // Steps the crowd; with a library list the mesh is edited at step 20 and the swapped-in tile's bytes are added to the list.
    private static void Run(Crowd crowd, int from, int to, List<byte[]>? library)
    {
        for (int s = from; s < to; s++)
        {
            if (library is not null && s == 20)
            {
                Edit(crowd.Mesh, library);
            }
            crowd.Step(1.0 / 30.0);
        }
    }

    private static Grid MakeGrid(ulong seed, int w, int h, bool varied)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        if (varied)
        {
            g.Areas[1] = 2.5;
            g.Areas[2] = 0.75;
        }
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                double r = rng.NextDouble();
                if (r < 0.2)
                {
                    g.SetArea(x, y, AreaCosts.Wall);
                }
                else if (varied && r < 0.35)
                {
                    g.SetArea(x, y, (byte)(1 + rng.NextInt(2)));
                }
            }
        }
        return g;
    }

    private static HexGrid MakeHex(ulong seed, int w, int h, HexLayout layout)
    {
        Grid t = MakeGrid(seed, w, h, true);
        HexGrid hx = HexGrid.From(t.Cells, t.Width, t.Height, layout);
        hx.Areas.CopyFrom(t.Areas);
        return hx;
    }

    private static Graph MakeGraph(ulong seed, int nodes, int edges)
    {
        var rng = new SeededRandom(seed);
        var g = new Graph(nodes);
        for (int i = 0; i < edges; i++)
        {
            g.AddEdge(rng.NextInt(nodes), rng.NextInt(nodes), 1.0 + rng.NextInt(20));
        }
        return g;
    }

    private static void Step(Action<int> step, Func<SearchStatus> status, int n, string what)
    {
        step(n);
        if (status() != SearchStatus.Running)
        {
            throw new InvalidOperationException(what + " finished within " + n + " iterations; the fixture must be a search in progress");
        }
    }

    private static List<byte[]> Tiles(TiledNavMesh m)
    {
        var l = new List<byte[]>();
        for (int s = 0; s < m.SlotCount; s++)
        {
            if (m.GetTileBySlot(s) is { } t)
            {
                l.Add(NavMeshSerializer.WriteTile(t));
            }
        }
        return l;
    }

    private static byte[] WithTiles(byte[] first, List<byte[]> tiles)
    {
        var parts = new List<byte[]> { first };
        parts.AddRange(tiles);
        return FixtureProbes.Pack(parts.ToArray());
    }

    internal static void Make(Dictionary<string, byte[]> f)
    {
        // ---- navmesh: two floors of two tiles, joined by off-mesh links
        TiledNavMesh mesh = BaseMesh();
        List<byte[]> libraryTiles = Tiles(mesh); // before any edit: the tiles a host keeps in its library
        f["navmesh-mesh"] = NavMeshSerializer.WriteMesh(mesh);
        f["navmesh-tile"] = libraryTiles[0];

        TiledNavMesh edited = BaseMesh();
        Edit(edited, libraryTiles);
        f["navmesh-snapshot-full"] = edited.Snapshot();
        f["navmesh-snapshot-referenced"] = WithTiles(edited.Snapshot(SnapshotMode.Referenced), libraryTiles);

        // ---- tile cache: a two-level floor, one obstacle applied, one waiting to be rebuilt
        {
            TileCache cache = new Mesh3DPipeline(FixtureProbes.TileCacheSettings()).BuildSoupCached(FixtureProbes.TileCacheSoupVertices, FixtureProbes.TileCacheSoupIndices);
            cache.AddObstacle(TileObstacle.Cylinder(18, 0, 6, 1.0, 2.0));
            cache.UpdateAll(CancellationToken.None);
            cache.AddObstacle(TileObstacle.Box(6, 0, 12, 9, 2, 14));
            if (cache.PendingCount == 0)
            {
                throw new InvalidOperationException("the tile cache fixture needs a rebuild waiting");
            }
            f["tilecache-full"] = FixtureProbes.Pack(cache.Snapshot());
            f["tilecache-referenced"] = WithTiles(cache.Snapshot(SnapshotMode.Referenced), Tiles(cache.Mesh));
        }

        // ---- crowds: eight agents walking the mesh; the second crowd has the mesh edited under it half way
        {
            Crowd plain = Populate(BaseMesh());
            Run(plain, 0, 45, null);
            f["crowd-state"] = plain.SaveState();
            var extra = new List<byte[]>(libraryTiles);
            Crowd withEdits = Populate(BaseMesh());
            Run(withEdits, 0, 45, extra);
            f["crowd-with-mesh-full"] = FixtureProbes.Pack(withEdits.SaveStateWithMesh());
            f["crowd-with-mesh-referenced"] = WithTiles(withEdits.SaveStateWithMesh(SnapshotMode.Referenced), extra);
        }

        // ---- corridor and follower
        {
            var q = new NavMeshQuery(mesh, 65536, PathNodeModel.Crossing);
            List<PolyRef> polys = CorridorWorlds.AllPolys(mesh);
            var r = new NavMeshWorlds.Rng(3);
            PathCorridor? corridor = null;
            for (int tries = 0; tries < 200 && corridor is null; tries++)
            {
                Vector3D a = CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r);
                Vector3D b = CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r);
                NearestPoly na = q.FindNearestPoly(a, CorridorWorlds.Extents);
                if (!na.Poly.IsSet)
                {
                    continue;
                }
                var c = new PathCorridor(256);
                c.Reset(na.Poly, na.Point);
                c.SetTarget(b);
                if (c.Replan(q) == QueryStatus.Success && c.Count > 3)
                {
                    var corners = new StraightPathPoint[64];
                    for (int m = 0; m < 3; m++)
                    {
                        c.FindCorners(q, corners, out int n);
                        c.MovePosition(q, Vector3D.Lerp(c.Position, corners[n > 1 ? 1 : 0].Position, 0.4));
                    }
                    corridor = c;
                }
            }
            f["path-corridor"] = (corridor ?? throw new InvalidOperationException("no corridor found")).ToBytes();

            PathFollower? follower = null;
            for (int tries = 0; tries < 200 && follower is null; tries++)
            {
                var pf = new PathFollower(FollowerOptions.Humanoid);
                if (pf.Place(q, CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r)) &&
                    pf.SetDestination(q, CorridorWorlds.PointIn(mesh, polys[r.Int(polys.Count)], r)) == QueryStatus.Success && pf.HasPath)
                {
                    for (int s = 0; s < 40; s++)
                    {
                        pf.Step(q, 1.0 / 30.0);
                    }
                    if (pf.HasPath)
                    {
                        follower = pf;
                    }
                }
            }
            f["path-follower"] = (follower ?? throw new InvalidOperationException("no follower found")).ToBytes();
        }

        // ---- grids
        {
            Grid g = MakeGrid(11, 24, 18, true);
            g.Areas[3] = 4.0;
            g.Space = new GridSpace(-6.5, 12.25, 0.5);
            g.SetArea(2, 2, 3);
            g.SetRect(10, 6, 3, 2, 2);
            f["grid-square"] = g.SaveState();

            HexGrid hx = MakeHex(12, 20, 16, HexLayout.FlatEvenColumn);
            hx.Space = new GridSpace(3.0, -2.0, 1.5);
            hx.SetArea(4, 4, 2);
            f["grid-hex"] = hx.SaveState();

            var v = new VoxelGrid(20, 6, 5, 0);
            var vr = new SeededRandom(13);
            for (int i = 0; i < 90; i++)
            {
                v.SetArea(vr.NextInt(20), vr.NextInt(6), vr.NextInt(5), vr.NextInt(3) == 0 ? (byte)2 : AreaCosts.Wall);
            }
            v.Areas[2] = 3.0;
            v.Space = new VoxelSpace(1.0, 2.0, 3.0, 0.5);
            f["grid-voxel"] = v.SaveState();
        }

        // ---- search states: each is a search in progress over a saved grid
        {
            Grid g = MakeGrid(21, 40, 30, false);
            var open = new List<Cell>();
            for (int y = 0; y < g.Height; y++)
            {
                for (int x = 0; x < g.Width; x++)
                {
                    if (g.IsWalkable(x, y))
                    {
                        open.Add(new Cell(x, y));
                    }
                }
            }
            Cell a = open[0], b = open[^1];
            string[] names = ["search-grid-astar", "search-grid-astar-generic", "search-grid-jps", "search-grid-jpsplus"];
            for (int i = 0; i < names.Length; i++)
            {
                GridSearch s = g.CreateSearch(FixtureProbes.SearchOptions[i]);
                s.Begin(a, b);
                Step(n => s.Step(n), () => s.Status == PathStatus.InProgress ? SearchStatus.Running : SearchStatus.NotStarted, 20, names[i]);
                f[names[i]] = FixtureProbes.Pack(g.SaveState(), s.SaveState());
            }

            Graph gr = MakeGraph(22, 120, 500);
            CsrGraph csr = gr.Compile();
            int w = g.Width;
            (int start, int goal) = FixtureProbes.SearchEnds(g);
            if (start == goal)
            {
                goal = (goal + 1) % g.CellCount;
            }
            GridSearchBuffers Buf() => new(Math.Max(g.CellCount, csr.NodeCount)) { RecordExpansions = true };
            using var index = new JpsPlusIndex(g);
            using var map = new BlockMap(g);
            {
                var s = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(g.CreateTopology(), default, new OctileHeuristic(w), Buf());
                s.Begin(start, goal);
                Step(n => s.Step(n), () => s.Status, 15, "search-astar");
                f["search-astar"] = FixtureProbes.Pack(g.SaveState(), gr.SaveState(), s.SaveState());
            }
            {
                var s = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(map), new OctileHeuristic(w), 1.0, Buf());
                s.Begin(start, goal);
                Step(n => s.Step(n), () => s.Status, 4, "search-jump-block");
                f["search-jump-block"] = FixtureProbes.Pack(g.SaveState(), gr.SaveState(), s.SaveState());
            }
            {
                var s = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(index), new OctileHeuristic(w), 1.0, Buf());
                s.Begin(start, goal);
                Step(n => s.Step(n), () => s.Status, 4, "search-jump-table");
                f["search-jump-table"] = FixtureProbes.Pack(g.SaveState(), gr.SaveState(), s.SaveState());
            }
            {
                var s = new ThetaSearch<EuclideanHeuristic>(g, new EuclideanHeuristic(w), ThetaVariant.LazyRequeue, 1.0, Buf());
                s.Begin(start, goal);
                Step(n => s.Step(n), () => s.Status, 10, "search-theta");
                f["search-theta"] = FixtureProbes.Pack(g.SaveState(), gr.SaveState(), s.SaveState());
            }
            {
                var s = new GraphAStar<DefaultGraphCost, ZeroGraphHeuristic>(csr, default, default, Buf());
                s.Begin(0, 119);
                Step(n => s.Step(n), () => s.Status, 6, "search-graph");
                f["search-graph"] = FixtureProbes.Pack(g.SaveState(), gr.SaveState(), s.SaveState());
            }
        }

        // ---- replanners: edited, partly replanned
        {
            foreach ((string name, GridAlgorithm alg) in new[] { ("replanner-dstar", GridAlgorithm.DStarLite), ("replanner-lpa", GridAlgorithm.LpaStar) })
            {
                Grid g = MakeGrid(31, 30, 22, true);
                GridReplanner rep = g.CreateReplanner(new Cell(1, 1), new Cell(g.Width - 2, g.Height - 2), FixtureProbes.ReplanOptions(alg));
                rep.Replan();
                g.SetArea(7, 7, AreaCosts.Wall);
                g.SetRect(12, 9, 3, 2, AreaCosts.Wall);
                rep.SetGoal(new Cell(g.Width - 4, g.Height - 3));
                rep.Replan(25);
                f[name] = FixtureProbes.Pack(g.SaveState(), rep.SaveState());
            }
            HexGrid hx = MakeHex(32, 26, 20, HexLayout.PointyOddRow);
            HexReplanner hrep = hx.CreateReplanner(new Cell(1, 1), new Cell(hx.Width - 2, hx.Height - 2), FixtureProbes.ReplanOptions(GridAlgorithm.DStarLite));
            hrep.Replan();
            hx.SetArea(6, 6, AreaCosts.Wall);
            hx.SetArea(7, 6, AreaCosts.Wall);
            hrep.Replan(25);
            f["replanner-hex-dstar"] = FixtureProbes.Pack(hx.SaveState(), hrep.SaveState());
        }

        // ---- flow fields
        {
            Grid g = MakeGrid(41, 32, 24, true);
            GridFlowField part = g.CreateFlowField(FixtureProbes.FlowGoals, FixtureProbes.FlowOptions(false));
            part.Step(40);
            f["flow-square-partial"] = FixtureProbes.Pack(g.SaveState(), part.SaveState());

            Grid g2 = MakeGrid(42, 32, 24, true);
            GridFlowField rep = g2.CreateFlowField(FixtureProbes.FlowGoals, FixtureProbes.FlowOptions(true));
            rep.Run();
            g2.SetArea(9, 9, AreaCosts.Wall);
            g2.SetArea(10, 9, AreaCosts.Wall);
            using var cancelled = new CancellationTokenSource();
            cancelled.Cancel();
            try
            {
                rep.Refresh(cancelled.Token); // stops straight after the repair is set up: a half-done repair
            }
            catch (OperationCanceledException)
            {
            }
            f["flow-square-repair"] = FixtureProbes.Pack(g2.SaveState(), rep.SaveState());

            HexGrid hx = MakeHex(43, 28, 22, HexLayout.FlatOddColumn);
            GridFlowField hf = hx.CreateFlowField(FixtureProbes.FlowGoals, FixtureProbes.FlowOptions(false));
            hf.Step(40);
            f["flow-hex-partial"] = FixtureProbes.Pack(hx.SaveState(), hf.SaveState());
        }

        // ---- graph and landmarks
        {
            Graph g = MakeGraph(51, 70, 260);
            g.RemoveEdge(5);
            g.AddEdge(3, 60, 2.5);
            g.SetWeight(10, 7.5);
            f["graph"] = g.SaveState();
            Landmarks lm = g.BuildLandmarks(new LandmarkOptions { Count = 4 });
            f["landmarks"] = FixtureProbes.Pack(g.SaveState(), lm.Serialize());
        }
    }
}
