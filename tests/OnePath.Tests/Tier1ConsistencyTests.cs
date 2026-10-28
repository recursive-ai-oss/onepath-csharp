using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Threading;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

// These tests pass their own tokens (live, cancelled, mid-search) or none, on purpose, so the xUnit token advice does not apply.
#pragma warning disable xUnit1051

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Tier-1 cancellation, async forms and option immutability: a token that is never cancelled changes nothing, a
/// cancelled one throws mid-search and leaves the object usable, the async forms return what the sync forms return,
/// and the option classes cannot be mutated after construction.
/// </summary>
public class Tier1ConsistencyTests
{
    private static readonly GridAlgorithm[] Algorithms =
    [
        GridAlgorithm.AStar, GridAlgorithm.Jps, GridAlgorithm.JpsPlus, GridAlgorithm.ThetaStar,
        GridAlgorithm.LazyThetaStar, GridAlgorithm.Hpa, GridAlgorithm.DStarLite,
    ];

    private static Grid RandomGrid(ulong seed, int w, int h, int wallPercent)
    {
        var rng = new SeededRandom(seed);
        var g = new Grid(w, h);
        for (int y = 0; y < h; y++)
        {
            for (int x = 0; x < w; x++)
            {
                if (rng.NextInt(100) < wallPercent)
                {
                    g.SetWalkable(x, y, false);
                }
            }
        }
        g.SetWalkable(0, 0, true);
        g.SetWalkable(w - 1, h - 1, true);
        return g;
    }

    private static void AssertSame(GridPath a, GridPath b)
    {
        Assert.Equal(a.Status, b.Status);
        Assert.Equal(a.Algorithm, b.Algorithm);
        Assert.Equal(BitConverter.DoubleToInt64Bits(a.Cost), BitConverter.DoubleToInt64Bits(b.Cost));
        Assert.Equal(a.ExpandedCount, b.ExpandedCount);
        Assert.True(a.Cells.SequenceEqual(b.Cells));
        Assert.True(a.ExpandedCells.SequenceEqual(b.ExpandedCells));
        Assert.True(a.Points.SequenceEqual(b.Points));
    }

    [Fact]
    public void Grid_live_token_changes_nothing()
    {
        using var cts = new CancellationTokenSource();
        for (ulong seed = 1; seed <= 6; seed++)
        {
            Grid g = RandomGrid(seed, 90, 70, 25);
            foreach (GridAlgorithm alg in Algorithms)
            {
                foreach (int budget in new[] { int.MaxValue, 700 })
                {
                    var o = new GridOptions { Algorithm = alg, IterationBudget = budget, RecordExpansions = true };
                    AssertSame(g.FindPath(new Cell(0, 0), new Cell(89, 69), o), g.FindPath(new Cell(0, 0), new Cell(89, 69), o, cts.Token));
                }
            }
            // Generic engine (delegate hook), Dijkstra, four-connected.
            var hook = new GridOptions { StepCost = (_, _, c) => c * 1.5, RecordExpansions = true };
            AssertSame(g.FindPath(new Cell(0, 0), new Cell(89, 69), hook), g.FindPath(new Cell(0, 0), new Cell(89, 69), hook, cts.Token));
            var four = new GridOptions { Connectivity = Connectivity.Four, Heuristic = GridHeuristic.Zero };
            AssertSame(g.FindPath(new Cell(0, 0), new Cell(89, 69), four), g.FindPath(new Cell(0, 0), new Cell(89, 69), four, cts.Token));
        }
    }

    [Fact]
    public void Grid_world_overload_takes_a_token()
    {
        var g = new Grid(32, 32) { Space = new GridSpace(0, 0, 1) };
        using var cts = new CancellationTokenSource();
        GridPath a = g.FindPath(new WorldPoint(0.5, 0.5), new WorldPoint(30.5, 20.5));
        GridPath b = g.FindPath(new WorldPoint(0.5, 0.5), new WorldPoint(30.5, 20.5), null, cts.Token);
        AssertSame(a, b);
        Assert.NotEmpty(b.Points.ToArray());
    }

    [Fact]
    public void Grid_cancelled_token_throws_for_every_engine_and_the_grid_stays_usable()
    {
        Grid g = RandomGrid(9, 120, 120, 20);
        var cancelled = new CancellationToken(true);
        foreach (GridAlgorithm alg in Algorithms)
        {
            var o = new GridOptions { Algorithm = alg };
            Assert.Throws<OperationCanceledException>(() => g.FindPath(new Cell(0, 0), new Cell(119, 119), o, cancelled));
            // The same grid answers the next query as if nothing had happened.
            GridPath after = g.FindPath(new Cell(0, 0), new Cell(119, 119), o);
            GridPath fresh = RandomGrid(9, 120, 120, 20).FindPath(new Cell(0, 0), new Cell(119, 119), o);
            AssertSame(fresh, after);
        }
    }

    [Fact]
    public void Grid_cancel_in_the_middle_of_a_search()
    {
        Grid g = new(200, 200);
        {
            using var cts = new CancellationTokenSource();
            int calls = 0;
            // The cost hook runs once per considered move; cancelling after 5000 of them stops the search at its next slice.
            var o = new GridOptions
            {
                Heuristic = GridHeuristic.Zero,
                StepCost = (_, _, c) =>
                {
                    if (++calls == 5000)
                    {
                        cts.Cancel();
                    }
                    return c;
                },
            };
            Assert.Throws<OperationCanceledException>(() => g.FindPath(new Cell(0, 0), new Cell(199, 199), o, cts.Token));
            Assert.True(calls < 40000, "the search should stop soon after the token is cancelled");
        }
        GridPath ok = g.FindPath(new Cell(0, 0), new Cell(199, 199));
        Assert.True(ok.Found);
    }

    [Fact]
    public void Hex_and_voxel_tokens()
    {
        var hex = HexGrid.FromText("""
            ..........
            ..........
            ....##....
            ..........
            """, HexLayout.PointyOddRow);
        using var live = new CancellationTokenSource();
        AssertSame(hex.FindPath(new Cell(0, 0), new Cell(9, 3)), hex.FindPath(new Cell(0, 0), new Cell(9, 3), null, live.Token));
        Assert.Throws<OperationCanceledException>(() => hex.FindPath(new Cell(0, 0), new Cell(9, 3), null, new CancellationToken(true)));

        var vox = new VoxelGrid(24, 24, 24);
        VoxelPath plain = vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(23, 23, 23), new VoxelOptions { RecordExpansions = true });
        VoxelPath withToken = vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(23, 23, 23), new VoxelOptions { RecordExpansions = true }, live.Token);
        Assert.Equal(plain.Status, withToken.Status);
        Assert.Equal(BitConverter.DoubleToInt64Bits(plain.Cost), BitConverter.DoubleToInt64Bits(withToken.Cost));
        Assert.Equal(plain.ExpandedCount, withToken.ExpandedCount);
        Assert.True(plain.Cells.SequenceEqual(withToken.Cells));

        using var cts = new CancellationTokenSource();
        int calls = 0;
        var o = new VoxelOptions
        {
            Heuristic = GridHeuristic.Zero,
            StepCost = (_, _, c) =>
            {
                if (++calls == 20000)
                {
                    cts.Cancel();
                }
                return c;
            },
        };
        Assert.Throws<OperationCanceledException>(() => vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(23, 23, 23), o, cts.Token));
        VoxelPath after = vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(23, 23, 23));
        Assert.Equal(plain.Cost, after.Cost);
        Assert.Throws<OperationCanceledException>(() => vox.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(23, 23, 23), null, new CancellationToken(true)));
    }

    private static NavMesh TileMapMesh()
    {
        var cells = new bool[60 * 60];
        Array.Fill(cells, true);
        for (int y = 0; y < 50; y++)
        {
            for (int x = 28; x < 31; x++)
            {
                cells[(y * 60) + x] = false;
            }
        }
        return NavMesh.Build2D(cells, 60, 60, 1.0, new Vector2D(0, 0));
    }

    private static void AssertSame(PathResult a, PathResult b)
    {
        Assert.Equal(a.Detail, b.Detail);
        Assert.Equal(BitConverter.DoubleToInt64Bits(a.Cost), BitConverter.DoubleToInt64Bits(b.Cost));
        Assert.Equal(a.Truncated, b.Truncated);
        Assert.True(a.Corridor.SequenceEqual(b.Corridor));
        Assert.Equal(a.Points.Count, b.Points.Count);
        for (int i = 0; i < a.Points.Count; i++)
        {
            Assert.Equal(a.Points[i].Position, b.Points[i].Position);
        }
    }

    // Rows of wall with alternating gaps: the route snakes through every row, so the corridor search is long.
    private static NavMesh SerpentineMesh()
    {
        const int n = 120;
        var cells = new bool[n * n];
        Array.Fill(cells, true);
        for (int row = 0; row < n / 4 - 1; row++)
        {
            int y = (row * 4) + 3;
            for (int x = 0; x < n; x++)
            {
                bool gap = row % 2 == 0 ? x >= n - 4 : x < 4;
                cells[(y * n) + x] = gap;
            }
        }
        return NavMesh.Build2D(cells, n, n, 1.0, new Vector2D(0, 0));
    }

    [Fact]
    public void NavMesh_token_changes_nothing_and_cancels()
    {
        NavMesh mesh = TileMapMesh();
        Vector3D a = new(3, 0, 3);
        Vector3D b = new(56, 0, 3);
        using var live = new CancellationTokenSource();
        PathResult plain = mesh.FindPath(a, b);
        Assert.True(plain.Found);
        AssertSame(plain, mesh.FindPath(a, b, null, live.Token));

        Assert.Throws<OperationCanceledException>(() => mesh.FindPath(a, b, null, new CancellationToken(true)));

        mesh = SerpentineMesh();
        a = new Vector3D(2, 0, 1);
        b = new Vector3D(2, 0, 115);
        plain = mesh.FindPath(a, b);
        Assert.True(plain.Found);
        AssertSame(plain, mesh.FindPath(a, b, null, live.Token));

        // Cancelled from inside the query (the filter predicate runs during the nearest-polygon lookups): the search stops
        // at its first slice boundary.
        using var cts = new CancellationTokenSource();
        int calls = 0;
        var o = new PathOptions
        {
            Filter = new QueryFilter
            {
                Predicate = (_, _) =>
                {
                    if (++calls == 1)
                    {
                        cts.Cancel();
                    }
                    return true;
                },
            },
        };
        Assert.Throws<OperationCanceledException>(() => mesh.FindPath(a, b, o, cts.Token));
        Assert.True(calls >= 1);
        // The mesh and the next query are unaffected.
        AssertSame(plain, mesh.FindPath(a, b));
    }

    [Fact]
    public async Task NavMesh_async_equals_sync_for_any_slice()
    {
        NavMesh mesh = TileMapMesh();
        var pairs = new[]
        {
            (new Vector3D(3, 0, 3), new Vector3D(56, 0, 3)),
            (new Vector3D(3, 0, 55), new Vector3D(56, 0, 55)),
            (new Vector3D(10, 0, 10), new Vector3D(10, 0, 20)),
        };
        foreach ((Vector3D s, Vector3D e) in pairs)
        {
            PathResult sync = mesh.FindPath(s, e);
            foreach (int slice in new[] { 1, 7, 256, 100000 })
            {
                AssertSame(sync, await mesh.FindPathAsync(s, e, null, slice, TestContext.Current.CancellationToken));
            }
        }
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => mesh.FindPathAsync(pairs[0].Item1, pairs[0].Item2, null, 1, new CancellationToken(true)));
    }

    [Fact]
    public async Task Grid_async_equals_sync()
    {
        for (ulong seed = 20; seed < 24; seed++)
        {
            Grid g = RandomGrid(seed, 80, 80, 25);
            foreach (GridAlgorithm alg in Algorithms)
            {
                var o = new GridOptions { Algorithm = alg, RecordExpansions = true };
                GridPath sync = g.FindPath(new Cell(0, 0), new Cell(79, 79), o);
                GridPath asyncPath = await g.FindPathAsync(new Cell(0, 0), new Cell(79, 79), o, TestContext.Current.CancellationToken);
                AssertSame(sync, asyncPath);
            }
        }
        var world = new Grid(20, 20) { Space = new GridSpace(0, 0, 1) };
        GridPath ws = world.FindPath(new WorldPoint(0.5, 0.5), new WorldPoint(18.5, 12.5));
        GridPath wa = await world.FindPathAsync(new WorldPoint(0.5, 0.5), new WorldPoint(18.5, 12.5), null, TestContext.Current.CancellationToken);
        AssertSame(ws, wa);
        // Concurrent queries on one grid give the sequential answers.
        Grid shared = RandomGrid(31, 100, 100, 25);
        GridPath expected = shared.FindPath(new Cell(0, 0), new Cell(99, 99));
        GridPath[] many = await Task.WhenAll(Enumerable.Range(0, 16).Select(_ => shared.FindPathAsync(new Cell(0, 0), new Cell(99, 99), null, TestContext.Current.CancellationToken)));
        foreach (GridPath p in many)
        {
            AssertSame(expected, p);
        }
    }

    [Fact]
    public async Task Grid_async_cancellation_and_errors_come_through_the_task()
    {
        var g = new Grid(300, 300);
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => g.FindPathAsync(new Cell(0, 0), new Cell(299, 299), null, new CancellationToken(true)));

        using var cts = new CancellationTokenSource();
        var o = new GridOptions { StepCost = (_, _, c) => { cts.Cancel(); return c; } };
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => g.FindPathAsync(new Cell(0, 0), new Cell(299, 299), o, cts.Token));

        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() => g.FindPathAsync(new WorldPoint(double.NaN, 0), new WorldPoint(1, 1), null, TestContext.Current.CancellationToken));
    }

    // ---- option immutability ----

    private static bool IsInitOnly(MethodInfo setter) =>
        setter.ReturnParameter.GetRequiredCustomModifiers().Any(m => m.FullName == "System.Runtime.CompilerServices.IsExternalInit");

    private static IEnumerable<Type> OptionTypes() =>
        typeof(NavMesh).Assembly.GetExportedTypes()
            .Where(t => t.IsClass && t.Namespace is "RecursiveAI.OnePath" or "RecursiveAI.OnePath.Grids" or "RecursiveAI.OnePath.Graphs" or "RecursiveAI.OnePath.Crowds"
                && (t.Name.EndsWith("Options", StringComparison.Ordinal) || t.Name == "NavSettings"));

    [Fact]
    public void Tier1_option_classes_are_immutable_after_construction()
    {
        var mutable = new List<string>();
        foreach (Type t in new[] { typeof(GridOptions), typeof(VoxelOptions), typeof(PathOptions), typeof(NavSettings), typeof(CrowdOptions), typeof(AgentOptions), typeof(GraphOptions), typeof(QueryFilter) })
        {
            foreach (PropertyInfo p in t.GetProperties(BindingFlags.Public | BindingFlags.Instance))
            {
                MethodInfo? set = p.SetMethod;
                if (set is { IsPublic: true } && !IsInitOnly(set))
                {
                    mutable.Add($"{t.Name}.{p.Name}");
                }
            }
        }
        Assert.True(mutable.Count == 0, "settable after construction: " + string.Join(", ", mutable));
        Assert.NotEmpty(OptionTypes());
    }

    [Fact]
    public void Static_presets_are_never_mutable_shared_instances()
    {
        // Every public static property or field that hands out an options/settings object either builds a fresh one
        // per call or returns an instance with no public setters.
        var offenders = new List<string>();
        foreach (Type t in typeof(NavMesh).Assembly.GetExportedTypes().Where(t => (t.IsClass || t.IsValueType) && !t.ContainsGenericParameters))
        {
            foreach (PropertyInfo p in t.GetProperties(BindingFlags.Public | BindingFlags.Static))
            {
                if (p.GetIndexParameters().Length == 0 && p.GetMethod is { IsPublic: true } && !t.ContainsGenericParameters && !p.PropertyType.IsByRefLike)
                {
                    Check(p.PropertyType, p.GetValue(null), p.GetValue(null), $"{t.Name}.{p.Name}", offenders);
                }
            }
            foreach (FieldInfo f in t.GetFields(BindingFlags.Public | BindingFlags.Static))
            {
                if (!f.IsLiteral)
                {
                    Check(f.FieldType, f.GetValue(null), f.GetValue(null), $"{t.Name}.{f.Name}", offenders);
                }
            }
        }
        Assert.True(offenders.Count == 0, "shared mutable presets: " + string.Join(", ", offenders));

        // And the ones that matter in practice return values that stay what they were.
        Assert.Equal(Connectivity.Eight, GridOptions.Default.Connectivity);
        Assert.NotSame(GridOptions.Default, GridOptions.Default);
        Assert.Equal(0.6, Agent.Humanoid.Radius);
        Assert.Same(Agent.Humanoid, Agent.Humanoid);
        Assert.Throws<ArgumentOutOfRangeException>(() => new GridOptions { AgentSize = 0 });
    }

    private static void Check(Type type, object? first, object? second, string where, List<string> offenders, int depth = 0)
    {
        // Only a preset that hands out the very same object on every read can leak a change to the next caller.
        if (!type.IsClass || first is null || !ReferenceEquals(first, second))
        {
            return;
        }
        type = first.GetType();
        foreach (PropertyInfo p in type.GetProperties(BindingFlags.Public | BindingFlags.Instance))
        {
            if (p.SetMethod is { IsPublic: true } set && !IsInitOnly(set))
            {
                offenders.Add($"{where} ({type.Name}.{p.Name})");
            }
            // Objects reachable from the preset (a filter inside a crowd's options, say) are shared too.
            if (p.GetIndexParameters().Length == 0 && p.PropertyType.IsClass && p.PropertyType.Assembly == typeof(NavMesh).Assembly && depth < 4)
            {
                object? inner = p.GetValue(first);
                Check(p.PropertyType, inner, inner, $"{where}.{p.Name}", offenders, depth + 1);
            }
        }
    }

    [Fact]
    public void Query_filter_is_immutable_and_derived_copies_leave_the_original_alone()
    {
        var baseFilter = new QueryFilter { ExcludeFlags = 2, Predicate = (f, _) => f != 8 };
        QueryFilter derived = baseFilter.WithAreaCost(5, 4.0).WithAreaCost(6, double.PositiveInfinity);
        Assert.NotSame(baseFilter, derived);
        Assert.Equal(1.0, baseFilter.GetAreaCost(5));
        Assert.Equal(4.0, derived.GetAreaCost(5));
        Assert.True(double.IsPositiveInfinity(derived.GetAreaCost(6)));
        Assert.Equal(1.0, baseFilter.GetAreaCost(6));
        Assert.Equal(2, derived.ExcludeFlags);
        Assert.Same(baseFilter.Predicate, derived.Predicate);
        Assert.Throws<ArgumentOutOfRangeException>(() => baseFilter.WithAreaCost(64, 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => baseFilter.WithAreaCost(0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => baseFilter.WithAreaCost(0, double.NaN));

        // The filters a preset hands out (options with a filter set) have no public setter to reach.
        foreach (PropertyInfo p in typeof(QueryFilter).GetProperties(BindingFlags.Public | BindingFlags.Instance))
        {
            Assert.True(p.SetMethod is null || IsInitOnly(p.SetMethod), "QueryFilter." + p.Name);
        }
        Assert.Null(CrowdOptions.Default.Filter);
        var shared = new CrowdOptions { Filter = derived };
        Assert.Same(derived, shared.Filter);
    }

    [Fact]
    public async Task Hex_async_equals_sync()
    {
        var rng = new Random(7);
        var open = new bool[40, 40];
        for (int x = 0; x < 40; x++)
        {
            for (int y = 0; y < 40; y++)
            {
                open[x, y] = rng.Next(100) >= 25 || (x, y) is (0, 0) or (39, 39);
            }
        }
        HexGrid g = HexGrid.From(open);
        foreach (GridAlgorithm alg in new[] { GridAlgorithm.AStar, GridAlgorithm.ThetaStar })
        {
            var o = new GridOptions { Algorithm = alg, RecordExpansions = true };
            AssertSame(g.FindPath(new Cell(0, 0), new Cell(39, 39), o), await g.FindPathAsync(new Cell(0, 0), new Cell(39, 39), o, TestContext.Current.CancellationToken));
        }
        var world = new HexGrid(20, 20) { Space = new GridSpace(0, 0, 1) };
        AssertSame(world.FindPath(new WorldPoint(0.5, 0.5), new WorldPoint(10.5, 9.5)), await world.FindPathAsync(new WorldPoint(0.5, 0.5), new WorldPoint(10.5, 9.5), null, TestContext.Current.CancellationToken));

        var wide = new HexGrid(300, 300);
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => wide.FindPathAsync(new Cell(0, 0), new Cell(299, 299), null, new CancellationToken(true)));
        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() => wide.FindPathAsync(new WorldPoint(double.NaN, 0), new WorldPoint(1, 1), null, TestContext.Current.CancellationToken));
    }

    [Fact]
    public async Task Voxel_async_equals_sync()
    {
        var rng = new Random(11);
        var open = new bool[16, 12, 16];
        for (int x = 0; x < 16; x++)
        {
            for (int y = 0; y < 12; y++)
            {
                for (int z = 0; z < 16; z++)
                {
                    open[x, y, z] = rng.Next(100) >= 20 || (x, y, z) is (0, 0, 0) or (15, 11, 15);
                }
            }
        }
        VoxelGrid g = VoxelGrid.From(open);
        foreach (VoxelConnectivity c in new[] { VoxelConnectivity.Six, VoxelConnectivity.TwentySix })
        {
            var o = new VoxelOptions { Connectivity = c };
            VoxelPath sync = g.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(15, 11, 15), o);
            VoxelPath asyncPath = await g.FindPathAsync(new VoxelCell(0, 0, 0), new VoxelCell(15, 11, 15), o, TestContext.Current.CancellationToken);
            Assert.Equal(sync.Status, asyncPath.Status);
            Assert.Equal(BitConverter.DoubleToInt64Bits(sync.Cost), BitConverter.DoubleToInt64Bits(asyncPath.Cost));
            Assert.True(sync.Cells.SequenceEqual(asyncPath.Cells));
        }
        var world = new VoxelGrid(10, 10, 10) { Space = new VoxelSpace(0, 0, 0, 1) };
        VoxelPath ws = world.FindPath(new VoxelPoint(0.5, 0.5, 0.5), new VoxelPoint(8.5, 7.5, 9.5));
        VoxelPath wa = await world.FindPathAsync(new VoxelPoint(0.5, 0.5, 0.5), new VoxelPoint(8.5, 7.5, 9.5), null, TestContext.Current.CancellationToken);
        Assert.Equal(ws.Status, wa.Status);
        Assert.True(ws.Cells.SequenceEqual(wa.Cells));
        Assert.True(ws.Points.SequenceEqual(wa.Points));

        var big = new VoxelGrid(64, 64, 64);
        await Assert.ThrowsAnyAsync<OperationCanceledException>(() => big.FindPathAsync(new VoxelCell(0, 0, 0), new VoxelCell(63, 63, 63), null, new CancellationToken(true)));
        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(() => big.FindPathAsync(new VoxelPoint(double.NaN, 0, 0), new VoxelPoint(1, 1, 1), null, TestContext.Current.CancellationToken));
    }
}
