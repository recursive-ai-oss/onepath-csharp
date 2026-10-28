using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>Agent classes in one world, and tile streaming.</summary>
public class WorldAndStreamingTests
{
    // A 20 x 20 floor split by a wall with a one-unit door: the small class fits through, the humanoid does not.
    private static Soup DoorLevel() => new Soup().Floor(0, 0, 20, 20, 0).Box(9, 0, 0, 11, 3, 9.5).Box(9, 0, 10.5, 11, 3, 20);

    private static readonly Vector3D Left = new(2, 0, 10), Right = new(18, 0, 10);

    private static byte[] Bytes(NavMesh m) => NavMeshSerializer.WriteMesh(m.Tiles);

    [Fact]
    public void Each_class_mesh_equals_a_single_class_build()
    {
        Soup s = DoorLevel();
        NavWorld w = NavWorld.Build(s.V.ToArray(), s.I.ToArray(), NavSettings.Small, NavSettings.Humanoid);
        Assert.Equal(2, w.ClassCount);
        Assert.Equal(Bytes(s.Build(NavSettings.Small)), Bytes(w[0]));
        Assert.Equal(Bytes(s.Build(NavSettings.Humanoid)), Bytes(w[1]));
        // The triangle overload reads the same geometry.
        var tris = new List<Triangle3D>();
        for (int i = 0; i < s.I.Count; i += 3)
        {
            Vector3D P(int k) => new(s.V[3 * s.I[i + k]], s.V[(3 * s.I[i + k]) + 1], s.V[(3 * s.I[i + k]) + 2]);
            tris.Add(new Triangle3D(P(0), P(1), P(2)));
        }
        Assert.Equal(Bytes(w[1]), Bytes(NavWorld.Build(tris, NavSettings.Small, NavSettings.Humanoid)[1]));
    }

    [Fact]
    public void Classes_see_different_doors()
    {
        Soup s = DoorLevel();
        NavWorld w = NavWorld.Build(s.V.ToArray(), s.I.ToArray(), NavSettings.Humanoid, NavSettings.Small);
        Assert.Equal(PathStatus.Partial, w[0].FindPath(Left, Right, cancellationToken: TestContext.Current.CancellationToken).Status);
        Assert.Equal(PathStatus.Found, w[1].FindPath(Left, Right, cancellationToken: TestContext.Current.CancellationToken).Status);
        Assert.Equal(PathStatus.Found, w.FindPath(Left, Right, 0.3, cancellationToken: TestContext.Current.CancellationToken).Status);
        Assert.Equal(PathStatus.Partial, w.FindPath(Left, Right, 0.6, cancellationToken: TestContext.Current.CancellationToken).Status);
    }

    [Fact]
    public void Class_choice_is_the_smallest_class_that_fits()
    {
        Soup s = new Soup().Floor(0, 0, 20, 20, 0);
        NavWorld w = NavWorld.Build(s.V.ToArray(), s.I.ToArray(), NavSettings.Large, NavSettings.Small, NavSettings.Humanoid);
        Assert.Equal(1, w.ClassFor(0));
        Assert.Equal(1, w.ClassFor(0.3));
        Assert.Equal(2, w.ClassFor(0.31));
        Assert.Equal(2, w.ClassFor(0.6));
        Assert.Equal(0, w.ClassFor(1.5));
        Assert.Equal(0, w.ClassFor(9));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClassFor(-1));
        Assert.Throws<ArgumentOutOfRangeException>(() => w.ClassFor(double.NaN));
        Assert.Throws<ArgumentException>(() => NavWorld.Build(s.V.ToArray(), s.I.ToArray()));
        Assert.Throws<ArgumentOutOfRangeException>(() => _ = w[3]);
    }

    [Fact]
    public void A_world_crowd_sends_each_agent_through_its_own_class()
    {
        Soup s = DoorLevel();
        NavWorld w = NavWorld.Build(s.V.ToArray(), s.I.ToArray(), NavSettings.Small, NavSettings.Humanoid);
        WorldCrowd crowd = w.CreateCrowd();
        Agent small = crowd.Add(Left, Agent.Small);
        Agent big = crowd.Add(Left + new Vector3D(0, 0, 3), Agent.Humanoid);
        Assert.Equal(2, crowd.Count);
        Assert.Equal(1, crowd.Crowds[0].Count);
        Assert.Equal(1, crowd.Crowds[1].Count);
        small.SetDestination(Right);
        big.SetDestination(Right);
        for (int i = 0; i < 1500 && !small.ReachedTarget; i++)
        {
            crowd.Step(1.0 / 30);
        }
        Assert.True(small.ReachedTarget);
        Assert.False(big.ReachedTarget);
        Assert.True(big.Position.X < 9);
        Assert.Throws<ArgumentOutOfRangeException>(() => crowd.Add(Left, Agent.Small, 5));
    }

    // ------------------------------------------------------------ streaming

    private static NavSettings Tiled => new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    private static NavMesh Field() => new Soup().Floor(0, 0, 48, 16, 0).Build(Tiled);

    [Fact]
    public void Streaming_a_column_out_and_in_returns_the_same_tiles_and_answers()
    {
        NavMesh m = Field();
        string before = AuthoredLinkTests.TileHashes(m.Tiles);
        PathResult p0 = m.FindPath(new Vector3D(2, 0, 8), new Vector3D(46, 0, 8), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, p0.Status);
        PolyRef oldRef = m.Tiles.GetPolyRef(m.Tiles.GetTileRef(1, 0, 0), 0);

        TileStreamer st = m.CreateStreamer();
        List<byte[]> out1 = st.StreamOut(1, 0);
        Assert.Single(out1);
        Assert.Equal(PathStatus.Partial, m.FindPath(new Vector3D(2, 0, 8), new Vector3D(46, 0, 8), cancellationToken: TestContext.Current.CancellationToken).Status);
        Assert.False(m.Tiles.IsValid(oldRef));

        st.QueueAdd(out1[0]);
        Assert.Equal(1, st.PendingCount);
        Assert.Equal(1, st.Update(8, TestContext.Current.CancellationToken));
        Assert.Equal(before, AuthoredLinkTests.TileHashes(m.Tiles));
        PathResult p1 = m.FindPath(new Vector3D(2, 0, 8), new Vector3D(46, 0, 8), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(p0.Status, p1.Status);
        Assert.Equal(p0.Points.Count, p1.Points.Count);
        for (int i = 0; i < p0.Points.Count; i++)
        {
            Assert.Equal(p0.Points[i].Position, p1.Points[i].Position);
        }
        // Old references stay dead even though the tile is identical.
        Assert.False(m.Tiles.IsValid(oldRef));
    }

    [Fact]
    public void Updates_spend_a_budget_of_operations_in_order()
    {
        NavMesh m = Field();
        TileStreamer st = m.CreateStreamer();
        byte[] t0 = st.StreamOut(0, 0)[0], t1 = st.StreamOut(1, 0)[0], t2 = st.StreamOut(2, 0)[0];
        Assert.Equal(0, m.Tiles.TileCount);
        st.QueueAdd(t0);
        st.QueueAdd(t1);
        st.QueueAdd(t2);
        st.QueueRemove(1, 0);
        Assert.Equal(0, st.Update(0, TestContext.Current.CancellationToken));
        Assert.Equal(2, st.Update(2, TestContext.Current.CancellationToken));
        Assert.Equal(2, m.Tiles.TileCount);
        Assert.Equal(2, st.PendingCount);
        Assert.Equal(2, st.UpdateAll(TestContext.Current.CancellationToken));
        Assert.Equal(2, m.Tiles.TileCount);
        Assert.True(m.Tiles.GetTileRef(1, 0, 0).IsSet == false);
        Assert.True(m.Tiles.GetTileRef(2, 0, 0).IsSet);
        // Removing what is not there is fine.
        st.QueueRemove(9, 9);
        Assert.Equal(1, st.UpdateAll(TestContext.Current.CancellationToken));
        Assert.Throws<ArgumentOutOfRangeException>(() => st.Update(-1, TestContext.Current.CancellationToken));
    }

    [Fact]
    public void Adding_a_present_tile_replaces_it()
    {
        NavMesh m = Field();
        TileStreamer st = m.CreateStreamer();
        byte[] bytes = NavMeshSerializer.WriteTile(m.Tiles.GetTile(m.Tiles.GetTileRef(1, 0, 0))!);
        int count = m.Tiles.TileCount;
        st.QueueAdd(bytes);
        st.UpdateAll(TestContext.Current.CancellationToken);
        Assert.Equal(count, m.Tiles.TileCount);
    }

    [Fact]
    public void Hostile_tile_bytes_are_typed_errors_and_do_not_block_the_queue()
    {
        NavMesh m = Field();
        TileStreamer st = m.CreateStreamer();
        byte[] good = NavMeshSerializer.WriteTile(m.Tiles.GetTile(m.Tiles.GetTileRef(0, 0, 0))!);
        var rng = new TestRng(5);
        for (int round = 0; round < 60; round++)
        {
            byte[] bad = (byte[])good.Clone();
            if (round % 3 == 0)
            {
                Array.Resize(ref bad, rng.Int(bad.Length));
            }
            else
            {
                for (int k = 0; k < 1 + rng.Int(4); k++)
                {
                    bad[rng.Int(bad.Length)] ^= (byte)(1 + rng.Int(255));
                }
            }
            st.QueueAdd(bad);
            st.QueueRemove(8, 8);
            try
            {
                st.UpdateAll(TestContext.Current.CancellationToken);
            }
            catch (NavMeshFormatException)
            {
            }
            st.UpdateAll(TestContext.Current.CancellationToken);
            Assert.Equal(0, st.PendingCount);
        }
    }

    private static Agent Walk(NavMesh m, out Crowd crowd, double startX)
    {
        crowd = m.CreateCrowd();
        Agent a = crowd.Add(new Vector3D(startX, 0, 8));
        a.SetDestination(new Vector3D(46, 0, 8));
        return a;
    }

    [Fact]
    public void A_crowd_walks_on_while_tiles_behind_it_stream()
    {
        NavMesh m = Field();
        Agent a = Walk(m, out Crowd crowd, 20);
        TileStreamer st = m.CreateStreamer();
        byte[]? held = null;
        for (int i = 0; i < 4000 && !a.ReachedTarget; i++)
        {
            if (i == 20)
            {
                held = st.StreamOut(0, 0)[0];
            }
            if (i == 60)
            {
                st.QueueAdd(held!);
            }
            st.Update(1, TestContext.Current.CancellationToken);
            crowd.Step(1.0 / 30);
        }
        Assert.True(a.ReachedTarget, $"{a.State} pos={a.Position}");
        Assert.InRange(a.Position.X, 44, 48);
        Assert.Equal(3, m.Tiles.TileCount);
    }

    [Fact]
    public void A_crowd_survives_losing_the_tile_ahead_and_goes_on_when_it_returns()
    {
        NavMesh m = Field();
        Agent a = Walk(m, out Crowd crowd, 2);
        TileStreamer st = m.CreateStreamer();
        byte[] held = st.StreamOut(2, 0)[0];
        for (int i = 0; i < 300; i++)
        {
            crowd.Step(1.0 / 30);
            Assert.InRange(a.Position.X, 0, 48);
        }
        Assert.False(a.ReachedTarget);
        // The tile comes back; the agent's route to the lost destination is gone, so the host sends it again.
        st.QueueAdd(held);
        Assert.Equal(1, st.UpdateAll(TestContext.Current.CancellationToken));
        Assert.NotEqual(PathStatus.GoalInvalid, a.SetDestination(new Vector3D(46, 0, 8)));
        for (int i = 0; i < 4000 && !a.ReachedTarget; i++)
        {
            crowd.Step(1.0 / 30);
        }
        Assert.True(a.ReachedTarget, $"{a.State} pos={a.Position}");
    }
}
