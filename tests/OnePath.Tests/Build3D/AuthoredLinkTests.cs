using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>Authored off-mesh links, the four-world shapes of 0.3: baking, queries, rebuilds, crowds, 2D, validation.</summary>
public class AuthoredLinkTests
{
    private static OffMeshLinkDefinition Jump(uint id = 7) => OffMeshLinkDefinition.Create(new Vector3D(9, 0, 5), new Vector3D(21, 0, 5), id) with { Radius = 0.6, Area = 4, Flags = 3 };

    private static NavSettings Settings(params OffMeshLinkDefinition[] links) =>
        new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1, OffMeshLinks = links };

    // Two islands with a gap of ten units between them: only a link joins them. The end of the link lies in the next tile column.
    private static Soup Islands() => new Soup().Floor(0, 0, 10, 10, 0).Floor(20, 0, 30, 10, 0);

    private static readonly Vector3D A = new(2, 0, 5), B = new(28, 0, 5);

    // Tile bytes by position: slot numbers and generations depend on the order of edits, the tiles do not.
    internal static string TileHashes(TiledNavMesh m)
    {
        var sb = new System.Text.StringBuilder();
        var refs = new TileRef[8];
        for (int z = -2; z < 6; z++)
        {
            for (int x = -2; x < 6; x++)
            {
                int n = m.GetTilesAt(x, z, refs);
                for (int i = 0; i < n; i++)
                {
                    sb.Append(x).Append(',').Append(z).Append(',').Append(i).Append('=').Append(Convert.ToHexString(System.Security.Cryptography.SHA256.HashData(NavMeshSerializer.WriteTile(m.GetTile(refs[i])!)))).Append(';');
                }
            }
        }
        return sb.ToString();
    }

    [Fact]
    public void Without_a_link_the_islands_are_apart()
    {
        NavMesh m = Islands().Build(Settings());
        Assert.Equal(PathStatus.Partial, m.FindPath(A, B).Status);
    }

    [Fact]
    public void A_link_joins_the_islands_and_the_path_goes_through_it()
    {
        NavMesh m = Islands().Build(Settings(Jump()));
        PathResult r = m.FindPath(A, B);
        Assert.Equal(PathStatus.Found, r.Status);
        int marks = 0;
        foreach (StraightPathPoint p in r.Points)
        {
            if (p.Flags.HasFlag(StraightPathMarks.OffMeshConnection))
            {
                marks++;
            }
        }
        Assert.Equal(1, marks);
        // Straight line 26 long; the link's ends are on that line.
        Assert.InRange(r.Length, 25.9, 26.3);
        // Two-way by default.
        Assert.Equal(PathStatus.Found, m.FindPath(B, A).Status);
        // The slot carries what was authored, in the tile that holds the start.
        NavTile? owner = null;
        for (int s = 0; s < m.Tiles.SlotCount; s++)
        {
            NavTile? t = m.Tiles.GetTileBySlot(s);
            if (t is not null && t.OffMeshLinks.Length > 0)
            {
                Assert.Null(owner);
                owner = t;
            }
        }
        Assert.NotNull(owner);
        Assert.Equal(0, owner.TileX);
        OffMeshLink l = owner.OffMeshLinks[0];
        Assert.Equal((uint)7, l.UserId);
        Assert.Equal((byte)4, l.Area);
        Assert.Equal((ushort)3, l.PolyFlags);
        Assert.True(l.Bidirectional);
        Assert.Equal(9f, l.StartX);
        Assert.Equal(21f, l.EndX);
        Assert.Empty(NavMeshValidator.Validate(m.Tiles));
    }

    [Fact]
    public void One_way_links_only_go_one_way()
    {
        NavMesh m = Islands().Build(Settings(Jump().OneWay()));
        Assert.Equal(PathStatus.Found, m.FindPath(A, B).Status);
        Assert.Equal(PathStatus.Partial, m.FindPath(B, A).Status);
    }

    [Fact]
    public void Runtime_switch_and_area_filter_close_an_authored_link()
    {
        NavMesh m = Islands().Build(Settings(Jump()));
        TiledNavMesh t = m.Tiles;
        TileRef owner = t.GetTileRef(0, 0, 0);
        Assert.True(t.SetOffMeshLinkEnabled(owner, 0, false));
        Assert.Equal(PathStatus.Partial, m.FindPath(A, B).Status);
        Assert.True(t.SetOffMeshLinkEnabled(owner, 0, true));
        Assert.Equal(PathStatus.Found, m.FindPath(A, B).Status);
        var closed = new PathOptions { Filter = new QueryFilter().SetAreaCost(4, double.PositiveInfinity) };
        Assert.Equal(PathStatus.Partial, m.FindPath(A, B, closed).Status);
    }

    [Fact]
    public void A_link_that_finds_no_ground_stays_unconnected()
    {
        OffMeshLinkDefinition nowhere = OffMeshLinkDefinition.Create(new Vector3D(9, 0, 5), new Vector3D(15, 0, 5));
        NavMesh m = Islands().Build(Settings(nowhere));
        Assert.Equal(PathStatus.Partial, m.FindPath(A, B).Status);
        // A start outside every built tile column is dropped.
        NavMesh far = Islands().Build(Settings(OffMeshLinkDefinition.Create(new Vector3D(100, 0, 5), new Vector3D(21, 0, 5))));
        for (int s = 0; s < far.Tiles.SlotCount; s++)
        {
            Assert.True(far.Tiles.GetTileBySlot(s) is null || far.Tiles.GetTileBySlot(s)!.OffMeshLinks.Length == 0);
        }
    }

    [Fact]
    public void A_link_far_above_the_tile_is_dropped_not_fatal()
    {
        NavMesh m = Islands().Build(Settings(Jump() with { Start = new Vector3D(9, 100000, 5) }, Jump(8)));
        int links = 0;
        for (int s = 0; s < m.Tiles.SlotCount; s++)
        {
            links += m.Tiles.GetTileBySlot(s)?.OffMeshLinks.Length ?? 0;
        }
        Assert.Equal(1, links);
    }

    [Fact]
    public void Rebuilt_tiles_keep_their_links()
    {
        var pipeline = new Mesh3DPipeline(Settings(Jump()));
        var s = Islands();
        TileCache cache = pipeline.BuildSoupCached(s.V.ToArray(), s.I.ToArray());
        string before = TileHashes(cache.Mesh);
        Assert.True(cache.Mesh.FindPath(A, B).HasPath && cache.Mesh.FindPath(A, B).Status == PathStatus.Found);
        // The cached build is the plain build.
        Assert.Equal(before, TileHashes(Islands().Build(Settings(Jump())).Tiles));

        int id = cache.AddObstacle(TileObstacle.Cylinder(5, 0, 8, 1.2, 2));
        cache.UpdateAll(TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, cache.Mesh.FindPath(A, B).Status);
        // The tile that owns the link was rebuilt and still holds it.
        Assert.Single(cache.Mesh.GetTile(cache.Mesh.GetTileRef(0, 0, 0))!.OffMeshLinks.ToArray());
        cache.RemoveObstacle(id);
        cache.UpdateAll(TestContext.Current.CancellationToken);
        Assert.Equal(before, TileHashes(cache.Mesh));
    }

    [Fact]
    public void Agents_in_a_crowd_cross_the_link()
    {
        NavMesh m = Islands().Build(Settings(Jump()));
        Crowd crowd = m.CreateCrowd();
        Agent a = crowd.Add(A);
        Assert.NotEqual(PathStatus.GoalInvalid, a.SetDestination(B));
        bool crossed = false;
        for (int i = 0; i < 3000 && !a.ReachedTarget; i++)
        {
            crowd.Step(1.0 / 30);
            crossed |= a.State == FollowerState.OffMeshLink;
        }
        Assert.True(crossed);
        Assert.True(a.ReachedTarget);
        Assert.InRange(a.Position.X, 26, 30);
    }

    [Fact]
    public void The_2D_builder_bakes_links_too()
    {
        Outline Rect(double x0, double x1) => new(new List<Vector2D> { new(x0, 0), new(x1, 0), new(x1, 10), new(x0, 10) });
        var settings = new NavSettings { CellSize = 0.25, TileSize = 16, AgentRadius = 0.5f, OffMeshLinks = [Jump()] };
        NavMesh m = NavMesh.Build2D([Rect(0, 10), Rect(20, 30)], settings);
        Assert.Equal(PathStatus.Found, m.FindPath(A, B).Status);
        NavMesh plain = NavMesh.Build2D([Rect(0, 10), Rect(20, 30)], settings with { OffMeshLinks = [] });
        Assert.Equal(PathStatus.Partial, plain.FindPath(A, B).Status);
    }

    [Fact]
    public void The_links_of_a_set_apply_to_every_class()
    {
        var s = Islands();
        NavWorld w = NavWorld.Build(s.V.ToArray(), s.I.ToArray(), Settings(Jump()), Settings(Jump()) with { AgentRadius = 0.3f });
        Assert.Equal(PathStatus.Found, w[0].FindPath(A, B).Status);
        Assert.Equal(PathStatus.Found, w[1].FindPath(A, B).Status);
    }

    [Fact]
    public void Bad_links_are_named()
    {
        OffMeshLinkDefinition ok = Jump();
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => Islands().Build(Settings(ok, ok with { Start = new Vector3D(double.NaN, 0, 0) })));
        Assert.Contains("OffMeshLinks[1]", e.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => Islands().Build(Settings(ok with { Radius = 0 })));
        Assert.Throws<ArgumentOutOfRangeException>(() => Islands().Build(Settings(ok with { Area = 64 })));
        Assert.Throws<ArgumentOutOfRangeException>(() => Islands().Build(Settings(ok with { Start = new Vector3D(3e6, 0, 0) })));
        var far = Assert.Throws<ArgumentOutOfRangeException>(() => Islands().Build(Settings(ok with { End = new Vector3D(100000, 0, 5) })));
        Assert.Contains("OffMeshLinks[0]", far.Message, StringComparison.Ordinal);
    }
}
