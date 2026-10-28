using System;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Voxel grid scenarios: A* on generated blocks (6, 18 and 26 moves, every corner rule, area costs), sliced against
/// unsliced, an edit sequence, map-text parsing and queries split across threads. Every result (status, cost bits, path,
/// expansion count) is written, so any difference in tie-breaking or arithmetic between platforms changes the hash.
/// </summary>
internal static class VoxelHashScenarios
{
    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("grids/voxel/astar-26-no-corner-cutting", ctx => Queries(ctx, 0x7001, VoxelConnectivity.TwentySix, VoxelCornerRule.NoCornerCutting, false));
        Scenarios.Register("grids/voxel/astar-26-no-squeezing", ctx => Queries(ctx, 0x7002, VoxelConnectivity.TwentySix, VoxelCornerRule.NoSqueezing, false));
        Scenarios.Register("grids/voxel/astar-26-allow-all-areas", ctx => Queries(ctx, 0x7003, VoxelConnectivity.TwentySix, VoxelCornerRule.AllowAll, true));
        Scenarios.Register("grids/voxel/astar-18-areas", ctx => Queries(ctx, 0x7004, VoxelConnectivity.Eighteen, VoxelCornerRule.NoCornerCutting, true));
        Scenarios.Register("grids/voxel/astar-6", ctx => Queries(ctx, 0x7005, VoxelConnectivity.Six, VoxelCornerRule.NoCornerCutting, false));
        Scenarios.Register("grids/voxel/sliced-equals-unsliced", Sliced);
        Scenarios.Register("grids/voxel/edits", Edits);
        Scenarios.Register("grids/voxel/map-text", MapText);
        Scenarios.Register("grids/voxel/threads", Threads);
        Scenarios.Register("grids/voxel/world-mapping", WorldMapping);
    }

    private const int Size = 24;

    private static VoxelGrid MakeGrid(ulong seed, bool areas)
    {
        var rng = new SeededRandom(seed);
        var g = new VoxelGrid(Size, Size - 3, Size + 5);
        for (int z = 0; z < g.SizeZ; z++)
        {
            for (int y = 0; y < g.SizeY; y++)
            {
                for (int x = 0; x < g.SizeX; x++)
                {
                    if (rng.NextDouble() < 0.22)
                    {
                        g.SetArea(x, y, z, AreaCosts.Wall);
                    }
                    else if (areas)
                    {
                        g.SetArea(x, y, z, (byte)rng.NextInt(4));
                    }
                }
            }
        }
        if (areas)
        {
            g.Areas[1] = 0.5;
            g.Areas[2] = 3.0;
            g.Areas[3] = 9.75;
        }
        return g;
    }

    private static VoxelCell RandomCell(ref SeededRandom rng, VoxelGrid g) => new(rng.NextInt(g.SizeX), rng.NextInt(g.SizeY), rng.NextInt(g.SizeZ));

    private static void WritePath(ScenarioContext ctx, VoxelPath p)
    {
        ctx.Write((int)p.Status);
        ctx.Write(p.Cost);
        ctx.Write(p.ExpandedCount);
        ctx.Write(p.Cells.Length);
        foreach (VoxelCell c in p.Cells)
        {
            ctx.Write(c.X);
            ctx.Write(c.Y);
            ctx.Write(c.Z);
        }
    }

    private static void Queries(ScenarioContext ctx, ulong seed, VoxelConnectivity conn, VoxelCornerRule rule, bool areas)
    {
        VoxelGrid g = MakeGrid(seed, areas);
        var rng = new SeededRandom(seed ^ 0xFFFF);
        var options = new VoxelOptions { Connectivity = conn, Corners = rule };
        for (int q = 0; q < 150; q++)
        {
            WritePath(ctx, g.FindPath(RandomCell(ref rng, g), RandomCell(ref rng, g), options));
        }
    }

    private static void Sliced(ScenarioContext ctx)
    {
        VoxelGrid g = MakeGrid(0x7101, true);
        var rng = new SeededRandom(0x7102);
        VoxelTopology topo = g.CreateTopology();
        for (int q = 0; q < 20; q++)
        {
            VoxelCell a = RandomCell(ref rng, g);
            VoxelCell b = RandomCell(ref rng, g);
            int na = g.NodeIndex(a.X, a.Y, a.Z);
            int nb = g.NodeIndex(b.X, b.Y, b.Z);
            if (!topo.IsWalkable(na) || !topo.IsWalkable(nb))
            {
                continue;
            }
            int budget = 1 + rng.NextInt(40);
            var buffers = new GridSearchBuffers(g.VoxelCount) { RecordExpansions = true };
            var search = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, new VoxelHeuristic(g.SizeX, g.SizeY, VoxelHeuristicKind.Octile26, g.Areas.MinCost), buffers);
            search.Begin(na, nb);
            int calls = 0;
            while (search.Step(budget) == SearchStatus.Running)
            {
                calls++;
            }
            ctx.Write(calls);
            ctx.Write((int)search.Status);
            ctx.Write(search.Iterations);
            ctx.Write(search.PathCost);
            foreach (int e in buffers.ExpandedNodes)
            {
                ctx.Write(e);
            }
        }
    }

    // Edits between queries; the bytes also pin the chunk allocation count.
    private static void Edits(ScenarioContext ctx)
    {
        var g = new VoxelGrid(40, 30, 20, AreaCosts.Wall);
        var rng = new SeededRandom(0x7201);
        for (int round = 0; round < 12; round++)
        {
            for (int e = 0; e < 300; e++)
            {
                g.SetArea(rng.NextInt(40), rng.NextInt(30), rng.NextInt(20), rng.NextInt(5) == 0 ? AreaCosts.Wall : (byte)rng.NextInt(3));
            }
            ctx.Write(g.AllocatedChunks);
            ctx.Write(g.Version);
            for (int q = 0; q < 6; q++)
            {
                WritePath(ctx, g.FindPath(RandomCell(ref rng, g), RandomCell(ref rng, g)));
            }
        }
    }

    private static void MapText(ScenarioContext ctx)
    {
        VoxelMap m = VoxelMap.Parse("voxel 6 5 4\n0 0 0\n1 1 1\n2 2 2\n3 3 3\n5 4 3\n2 1 0\n");
        VoxelGrid g = m.ToGrid();
        ctx.Write(m.FilledCount);
        WritePath(ctx, g.FindPath(new VoxelCell(0, 1, 0), new VoxelCell(5, 3, 3)));
        VoxelScenarioEntry[] e = VoxelScenarios.Parse("version 1\nx\n1 2 3 4 5 6 7.25 1.5\n");
        ctx.Write(e[0].OptimalLength);
        ctx.Write(e[0].Ratio);
    }

    private static void Threads(ScenarioContext ctx)
    {
        VoxelGrid g = MakeGrid(0x7301, true);
        const int Queries = 64;
        var results = new VoxelPath[Queries];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Queries, options, i =>
        {
            var rng = new SeededRandom(0x7400 + (ulong)i);
            results[i] = g.FindPath(RandomCell(ref rng, g), RandomCell(ref rng, g));
        });
        foreach (VoxelPath p in results)
        {
            WritePath(ctx, p);
        }
    }

    private static void WorldMapping(ScenarioContext ctx)
    {
        var g = new VoxelGrid(12, 10, 8) { Space = new VoxelSpace(-1000.125, 2048.5, 7.75, 0.3) };
        g.SetWalkable(5, 5, 4, false);
        VoxelPath p = g.FindPath(new VoxelPoint(-1000.0, 2048.6, 7.8), new VoxelPoint(-997.0, 2051.0, 9.9));
        WritePath(ctx, p);
        foreach (VoxelPoint w in p.Points)
        {
            ctx.Write(w.X);
            ctx.Write(w.Y);
            ctx.Write(w.Z);
        }
    }
}
