using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// The shape rules of the public surface: one status zero that is never success, one exception base, one options shape with
/// one validation point, the happy path in one namespace, and a crowd you can enumerate without allocating.
/// </summary>
public class ApiShapeTests
{
    private static readonly Assembly Library = typeof(NavMesh).Assembly;
    private static readonly string[] NeutralZeros = ["None", "NotStarted"];

    // ---- status enums ----

    [Fact]
    public void Every_result_status_enum_has_a_neutral_zero()
    {
        // default(T) is what an unassigned field or a zeroed array slot reads as: it must be "nothing happened", never success.
        Type[] statuses =
        [
            typeof(PathStatus), typeof(Mesh.Queries.QueryStatus), typeof(Mesh.Queries.AreaQueryStatus), typeof(Mesh.Queries.RaycastStatus),
            typeof(SearchStatus), typeof(FlowFieldStatus),
        ];
        foreach (Type t in statuses)
        {
            Assert.Contains(Enum.GetName(t, 0), NeutralZeros);
        }
        Assert.Equal("Idle", Enum.GetName(typeof(FollowerState), 0));
        Assert.Equal("Valid", Enum.GetName(typeof(CorridorStatus), 0)); // a validation outcome, not a result that can be unassigned
    }

    // ---- exceptions ----

    [Fact]
    public void Every_data_error_derives_from_the_one_base_and_has_the_standard_constructors()
    {
        Type[] exceptions = Library.GetExportedTypes().Where(t => typeof(Exception).IsAssignableFrom(t)).ToArray();
        Assert.NotEmpty(exceptions);
        var problems = new List<string>();
        foreach (Type t in exceptions)
        {
            if (!typeof(OnePathFormatException).IsAssignableFrom(t))
            {
                problems.Add(t.Name + ": not an OnePathFormatException");
            }
            if (typeof(InvalidOperationException).IsAssignableFrom(t) || typeof(ArgumentException).IsAssignableFrom(t))
            {
                problems.Add(t.Name + ": derives from a usage-error type");
            }
            if (t != typeof(OnePathFormatException) && t != typeof(Mesh.NavMeshFormatException) && !t.IsSealed)
            {
                problems.Add(t.Name + ": not sealed");
            }
            if (t.GetConstructor(Type.EmptyTypes) is null
                || t.GetConstructor([typeof(string)]) is null
                || t.GetConstructor([typeof(string), typeof(Exception)]) is null)
            {
                problems.Add(t.Name + ": missing a standard constructor (CA1032)");
            }
        }
        Assert.True(problems.Count == 0, string.Join("; ", problems));
    }

    [Fact]
    public void Each_error_names_its_own_subject()
    {
        var crowd = Fixtures.SmallCrowd(out _);
        byte[] junk = [1, 2, 3, 4, 5, 6, 7, 8];
        CrowdStateException c = Assert.Throws<CrowdStateException>(() => crowd.RestoreState(junk));
        Assert.Contains("crowd", c.Message, StringComparison.OrdinalIgnoreCase);
        GraphStateException g = Assert.Throws<GraphStateException>(() => Graph.FromState(junk));
        Assert.Contains("graph", g.Message, StringComparison.OrdinalIgnoreCase);
        GridStateException s = Assert.Throws<GridStateException>(() => new Grid(4, 4).RestoreState(junk));
        Assert.Contains("grid", s.Message, StringComparison.OrdinalIgnoreCase);
        Assert.Throws<Mesh.NavMeshFormatException>(() => Mesh.NavMeshSerializer.ReadMesh(junk));
        Assert.Throws<GridFormatException>(() => Grid.FromText("..\n.x"));

        // One catch clause covers every kind of bad data.
        foreach (Action bad in new Action[]
        {
            () => crowd.RestoreState(junk),
            () => Graph.FromState(junk),
            () => new Grid(4, 4).RestoreState(junk),
            () => Mesh.NavMeshSerializer.ReadMesh(junk),
            () => Grid.FromText("..\n.x"),
            () => NavMesh.BuildGltf(junk),
        })
        {
            Assert.ThrowsAny<OnePathFormatException>(bad);
        }
    }

    // ---- options ----

    private static readonly Type[] OptionTypes =
    [
        typeof(GridOptions), typeof(VoxelOptions), typeof(PathOptions), typeof(NavSettings), typeof(CrowdOptions), typeof(AgentOptions),
        typeof(GraphOptions), typeof(PointGraphOptions), typeof(LandmarkOptions), typeof(FollowerOptions),
    ];

    [Fact]
    public void Every_tier1_options_type_is_a_sealed_record_without_Clone()
    {
        foreach (Type t in OptionTypes)
        {
            Assert.True(t.IsSealed, t.Name + " is not sealed");
            // Records get a synthesized, non-public "<Clone>$" method and a protected EqualityContract.
            Assert.NotNull(t.GetMethod("<Clone>$", BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic));
            Assert.Null(t.GetMethod("Clone", BindingFlags.Instance | BindingFlags.Public));
        }
    }

    [Fact]
    public void With_replaces_Clone_and_equal_options_are_equal()
    {
        GridOptions a = GridOptions.FourConnected with { Algorithm = GridAlgorithm.Jps, IterationBudget = 99 };
        Assert.Equal(GridAlgorithm.Jps, a.Algorithm);
        Assert.Equal(Connectivity.Four, a.Connectivity);
        Assert.Equal(a, GridOptions.FourConnected with { Algorithm = GridAlgorithm.Jps, IterationBudget = 99 });
        Assert.NotEqual(a, GridOptions.Default);
        Assert.Equal(VoxelOptions.SixConnected, new VoxelOptions { Connectivity = VoxelConnectivity.Six });
        Assert.Equal(new PathOptions { MaxPoints = 8 }, new PathOptions { MaxPoints = 8 });
        Assert.Equal(new GraphOptions { IterationBudget = 5 }, new GraphOptions { IterationBudget = 5 });
        Assert.Equal(new LandmarkOptions { Count = 4 }, new LandmarkOptions { Count = 4 });
    }

    [Fact]
    public void Link_lists_are_copied_so_options_cannot_change_under_a_caller()
    {
        var links = new[] { OffMeshLinkDefinition.Create(new Vector3D(1, 0, 1), new Vector3D(5, 1, 5)) };
        NavSettings s = NavSettings.Humanoid with { OffMeshLinks = links };
        links[0] = OffMeshLinkDefinition.Create(new Vector3D(9, 0, 9), new Vector3D(9, 0, 9));
        Assert.Equal(new Vector3D(1, 0, 1), s.OffMeshLinks[0].Start);
        // Equal contents compare equal even from different arrays.
        NavSettings t = NavSettings.Humanoid with { OffMeshLinks = new[] { OffMeshLinkDefinition.Create(new Vector3D(1, 0, 1), new Vector3D(5, 1, 5)) } };
        Assert.Equal(s, t);
        Assert.Empty(NavSettings.Humanoid.OffMeshLinks);
        Assert.Empty((NavSettings.Humanoid with { OffMeshLinks = null! }).OffMeshLinks);

        var pairs = new List<(int A, int B)> { (0, 1) };
        PointGraphOptions o = new() { Links = pairs };
        pairs.Add((1, 2));
        Assert.Single(o.Links!);
        Assert.Equal(new PointGraphOptions { Links = new[] { (0, 1) } }, o);

        var obstacles = new List<Mesh.Build3D.TileObstacle> { Mesh.Build3D.TileObstacle.Box(0, 0, 0, 1, 1, 1) };
        var pipeline = new Mesh.Build3D.Mesh3DPipeline(NavSettings.Humanoid) { Obstacles = obstacles };
        obstacles.Clear();
        Assert.Single(pipeline.Obstacles);
    }

    [Fact]
    public void Bad_option_values_throw_at_the_property_naming_it()
    {
        void Bad(string property, Func<object> make)
        {
            ArgumentOutOfRangeException e = Assert.Throws<ArgumentOutOfRangeException>(make);
            Assert.Equal(property, e.ParamName);
        }
        Bad("AgentSize", () => new GridOptions { AgentSize = 0 });
        Bad("HeuristicWeight", () => new GridOptions { HeuristicWeight = double.NaN });
        Bad("IterationBudget", () => new GridOptions { IterationBudget = 0 });
        Bad("IterationsPerSlice", () => new GridOptions { IterationsPerSlice = 0 });
        Bad("IterationBudget", () => new VoxelOptions { IterationBudget = 0 });
        Bad("IterationBudget", () => new GraphOptions { IterationBudget = -1 });
        Bad("Count", () => new LandmarkOptions { Count = 0 });
        Bad("MaxCorridor", () => new PathOptions { MaxCorridor = 0 });
        Bad("AgentRadius", () => NavSettings.Humanoid with { AgentRadius = -1 });
        Bad("CellSize", () => NavSettings.Humanoid with { CellSize = 0 });
        Bad("TileSize", () => NavSettings.Humanoid with { TileSize = 0.1 });
        Bad("AgentMaxSlopeDegrees", () => NavSettings.Humanoid with { AgentMaxSlopeDegrees = 91 });
        Bad("FloorY", () => NavSettings.Humanoid with { FloorY = double.NaN });
        Bad("MaxSpeed", () => Agent.Humanoid with { MaxSpeed = -5 });
        Bad("SeparationWeight", () => Agent.Humanoid with { SeparationWeight = 2 });
        Bad("MaxAgents", () => new CrowdOptions { MaxAgents = 0 });
        Bad("MaxThreads", () => new CrowdOptions { MaxThreads = 0 });
    }

    [Fact]
    public void Agent_quantities_in_NavSettings_are_doubles_and_FloorY_is_the_2D_floor()
    {
        foreach (string name in new[] { "AgentRadius", "AgentHeight", "WalkableClimb", "AgentMaxSlopeDegrees", "ContourMaxError", "CellSize", "TileSize", "FloorY" })
        {
            Assert.Equal(typeof(double), typeof(NavSettings).GetProperty(name)!.PropertyType);
        }
        Assert.Null(typeof(NavSettings).GetProperty("Height"));
        var open = new bool[20 * 20];
        Array.Fill(open, true);
        NavMesh mesh = NavMesh.Build2D(open, 20, 20, 1.0, new Vector2D(0, 0), NavSettings.Humanoid with { AgentRadius = 0.4, FloorY = 3 });
        PathResult p = mesh.FindPath(new Vector3D(2, 3, 2), new Vector3D(15, 3, 15), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(p.Found);
        Assert.Equal(3.0, p.Points[0].Position.Y, 6);
    }

    // ---- namespaces ----

    [Fact]
    public void The_happy_path_types_live_in_the_root_namespace()
    {
        foreach (Type t in new[] { typeof(PathResult), typeof(PathOptions), typeof(QueryFilter), typeof(FollowerState), typeof(PartitionType), typeof(PathNodeModel), typeof(StraightPathPoint), typeof(PathStatus) })
        {
            Assert.Equal("RecursiveAI.OnePath", t.Namespace);
        }
    }

    [Fact]
    public void Tier1_members_reach_into_sub_namespaces_only_for_the_documented_doorways()
    {
        // Each name here is a deliberate step down to tier 2 or 3 (a builder input, a handle, a raw engine); nothing a quickstart snippet
        // names is in this list.
        string[] doorways =
        [
            "PathFollower", "FollowerOptions", "TiledNavMesh", "NavMeshQuery", "TileLibrary", "SnapshotMode", "CsrGraph", "SquareTopology", "HexTopology",
            "VoxelTopology", "IGridChangeListener", "IVoxelChangeListener", "GltfScene", "SolidHeightfield", "VoxelChunk", "Outline", "Region", "GltfOptions",
            "TileStreamer", "NavMeshParams", "PolyRef", "QueryStatus", "StraightPathMarks",
        ];
        var unexpected = new SortedSet<string>(StringComparer.Ordinal);
        foreach (Type t in Library.GetExportedTypes().Where(t => t.Namespace == "RecursiveAI.OnePath"))
        {
            foreach (MemberInfo m in t.GetMembers(BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static | BindingFlags.DeclaredOnly))
            {
                var used = new List<Type>();
                switch (m)
                {
                    case MethodBase mb:
                        used.AddRange(mb.GetParameters().Select(p => p.ParameterType));
                        if (mb is MethodInfo mi)
                        {
                            used.Add(mi.ReturnType);
                        }
                        break;
                    case PropertyInfo pi:
                        used.Add(pi.PropertyType);
                        break;
                    case FieldInfo fi:
                        used.Add(fi.FieldType);
                        break;
                }
                foreach (Type x in used.SelectMany(Flatten))
                {
                    if (x.Namespace is { } ns && ns.StartsWith("RecursiveAI.OnePath.", StringComparison.Ordinal) && !doorways.Contains(x.Name))
                    {
                        unexpected.Add($"{t.Name}.{m.Name} -> {x.FullName}");
                    }
                }
            }
        }
        Assert.True(unexpected.Count == 0, string.Join(Environment.NewLine, unexpected));
    }

    private static IEnumerable<Type> Flatten(Type t)
    {
        while (t.HasElementType)
        {
            t = t.GetElementType()!;
        }
        yield return t;
        if (t.IsGenericType)
        {
            foreach (Type a in t.GetGenericArguments().SelectMany(Flatten))
            {
                yield return a;
            }
        }
    }

    // ---- crowd enumeration ----

    [Fact]
    public void A_crowd_is_enumerable_and_copies_into_a_span_without_allocating()
    {
        Crowd crowd = Fixtures.SmallCrowd(out Agent[] added);
        Assert.Equal(added, crowd.ToArray());
        var span = new Agent[crowd.Count];
        Assert.Equal(crowd.Count, crowd.CopyAgents(span));
        Assert.Equal(added, span);
        Assert.Throws<ArgumentException>(() => crowd.CopyAgents(new Agent[crowd.Count - 1]));

        long bytes = AllocationProbe.SteadyState(() =>
        {
            crowd.CopyAgents(span);
            long n = 0;
            foreach (Agent a in crowd)
            {
                n += a.Id;
            }
            return n > 0;
        });
        Assert.Equal(0, bytes);
    }

    [Fact]
    public void A_removed_agent_leaves_the_enumeration_and_slot_order_is_kept()
    {
        Crowd crowd = Fixtures.SmallCrowd(out Agent[] added);
        crowd.Remove(added[1]);
        Assert.Equal([added[0], added[2], added[3]], crowd.ToArray());
    }

    private static class Fixtures
    {
        internal static Crowd SmallCrowd(out Agent[] agents)
        {
            var open = new bool[30 * 30];
            Array.Fill(open, true);
            NavMesh mesh = NavMesh.Build2D(open, 30, 30, 1.0, new Vector2D(0, 0));
            Crowd crowd = mesh.CreateCrowd();
            agents = [.. Enumerable.Range(0, 4).Select(i => crowd.Add(new Vector3D(3 + (3 * i), 0, 5), Agent.Humanoid))];
            return crowd;
        }
    }
}
