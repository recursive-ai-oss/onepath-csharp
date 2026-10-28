using System;
using System.Linq;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

/// <summary>The cached <see cref="Grid.ContentHash"/> and <see cref="NavMesh.Build2D(Grid, NavSettings?)"/>.</summary>
public class GridHashAndMeshTests
{
    // The definition of the hash, written again from its documented words, so the cache is checked against a fresh recompute.
    private static ulong Reference(Grid g)
    {
        const ulong P1 = 0x9E3779B185EBCA87UL, P2 = 0xC2B2AE3D27D4EB4FUL, P3 = 0x165667B19E3779F9UL;
        ulong Word(ulong h, ulong w) => (System.Numerics.BitOperations.RotateLeft(h ^ (w * P2), 31) * P1) + P3;
        ulong h = P1 ^ P2;
        h = Word(h, (uint)g.Width | ((ulong)(uint)g.Height << 32));
        byte[] c = g.Cells.ToArray();
        for (int i = 0; i < c.Length; i += 8)
        {
            ulong w = 0;
            for (int k = 0; k < 8 && i + k < c.Length; k++)
            {
                w |= (ulong)c[i + k] << (8 * k);
            }
            h = Word(h, w);
        }
        for (int a = 0; a < AreaCosts.Count; a++)
        {
            h = Word(h, (ulong)BitConverter.DoubleToInt64Bits(g.Areas[(byte)a]));
        }
        h ^= h >> 33;
        h *= P2;
        h ^= h >> 29;
        h *= P3;
        h ^= h >> 32;
        return h;
    }

    private sealed class EditOnNotify(Grid g) : IGridChangeListener
    {
        public ulong Seen;

        public void OnCellChanged(int x, int y) => Seen = g.ContentHash();
    }

    [Fact]
    public void CachedHashEqualsRecomputeAfterEveryKindOfEdit()
    {
        var g = new Grid(37, 29);
        for (int i = 0; i < 200; i++)
        {
            g.SetArea((i * 7) % 37, (i * 11) % 29, (byte)(i % 5 == 0 ? AreaCosts.Wall : i % 4));
        }
        Assert.Equal(Reference(g), g.ContentHash());
        Assert.Equal(Reference(g), g.ContentHash());

        g.SetArea(3, 4, 9);
        Assert.Equal(Reference(g), g.ContentHash());
        g.SetRect(2, 2, 10, 5, 4);
        Assert.Equal(Reference(g), g.ContentHash());
        g.SetRect(0, 0, 3, 2, new byte[] { 1, 2, 3, 4, 5, AreaCosts.Wall });
        Assert.Equal(Reference(g), g.ContentHash());
        g.Areas[4] = 3.5;
        Assert.Equal(Reference(g), g.ContentHash());
        var other = new AreaCosts();
        other[7] = 0.25;
        g.Areas.CopyFrom(other);
        Assert.Equal(Reference(g), g.ContentHash());

        // Restore to an earlier state, including one whose edit counters repeat.
        byte[] saved = g.SaveState();
        ulong savedHash = g.ContentHash();
        g.SetArea(1, 1, 6);
        g.SetRect(5, 5, 4, 4, AreaCosts.Wall);
        g.Areas[1] = 8;
        Assert.NotEqual(savedHash, g.ContentHash());
        g.RestoreState(saved);
        Assert.Equal(savedHash, g.ContentHash());
        Assert.Equal(Reference(g), g.ContentHash());
        Grid copy = Grid.FromState(saved);
        Assert.Equal(savedHash, copy.ContentHash());

        // An edit that changes nothing keeps the value; a listener that asks for the hash sees the edited grid.
        g.SetArea(1, 1, g.GetArea(1, 1));
        Assert.Equal(savedHash, g.ContentHash());
        var listener = new EditOnNotify(g);
        g.AddListener(listener);
        byte old = g.GetArea(8, 8);
        g.SetArea(8, 8, old == 2 ? (byte)3 : (byte)2);
        Assert.Equal(Reference(g), listener.Seen);
        g.SetRect(10, 10, 3, 3, 11);
        Assert.Equal(Reference(g), listener.Seen);
        Assert.Equal(Reference(g), g.ContentHash());
    }

    [Fact]
    public void RepeatedHashIsConstantTime()
    {
        var g = new Grid(2048, 2048);
        ulong first = g.ContentHash();
        long t0 = System.Diagnostics.Stopwatch.GetTimestamp();
        ulong sink = 0;
        for (int i = 0; i < 1000; i++)
        {
            sink ^= g.ContentHash();
        }
        double cachedUs = System.Diagnostics.Stopwatch.GetElapsedTime(t0).TotalMicroseconds / 1000.0;
        g.SetArea(0, 0, 1);
        long t1 = System.Diagnostics.Stopwatch.GetTimestamp();
        ulong fresh = g.ContentHash();
        double freshUs = System.Diagnostics.Stopwatch.GetElapsedTime(t1).TotalMicroseconds;
        Assert.Equal(first, sink == 0 ? first : first); // keep the loop alive
        Assert.NotEqual(first, fresh);
        // 4M cells take milliseconds to hash; a cached call takes nanoseconds. A wide margin keeps this stable on a busy machine.
        Assert.True(cachedUs * 50 < freshUs, $"cached {cachedUs} us per call, fresh {freshUs} us");
    }

    private static NavSettings Fast => new() { AgentRadius = 0.5f, CellSize = 0.25, TileSize = 16 };

    private static byte[] Bytes(NavMesh m) => NavMeshSerializer.WriteMesh(m.Tiles);

    [Fact]
    public void SingleAreaGridEqualsTileMapBuild()
    {
        var g = new Grid(30, 24) { Space = new GridSpace(-3.5, 10.0, 0.5) };
        g.SetRect(10, 0, 3, 18, AreaCosts.Wall);
        g.SetRect(20, 0, 4, 16, AreaCosts.Wall);
        g.SetArea(2, 2, AreaCosts.Wall);
        var cells = new bool[g.CellCount];
        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = g.Cells[i] != AreaCosts.Wall;
        }
        NavMesh a = NavMesh.Build2D(g, Fast);
        NavMesh b = NavMesh.Build2D(cells, g.Width, g.Height, 0.5, new Vector2D(-3.5, 10.0), Fast);
        Assert.Equal(Bytes(b), Bytes(a));

        PathResult p = a.FindPath(new Vector3D(-2.0, 0, 12.0), new Vector3D(10.5, 0, 20.0), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(p.Found, p.Detail.ToString() + " " + a.Tiles.TileCount);
        Assert.True(p.Points.Count >= 2);
        // The straight line is about 16.5 long; the walls force the path through the gaps below them.
        Assert.True(p.Length > 17.0);
    }

    [Fact]
    public void AreasAreDroppedWallsAreKept()
    {
        var plain = new Grid(20, 20);
        var mixed = new Grid(20, 20);
        mixed.SetRect(5, 5, 6, 6, 3);
        mixed.Areas[3] = 4.0;
        Assert.Equal(Bytes(NavMesh.Build2D(plain, Fast)), Bytes(NavMesh.Build2D(mixed, Fast)));
        mixed.SetRect(5, 5, 6, 6, AreaCosts.Wall);
        Assert.NotEqual(Bytes(NavMesh.Build2D(plain, Fast)), Bytes(NavMesh.Build2D(mixed, Fast)));
    }

    [Fact]
    public void EmptyAndBlockedGridsGiveNoPolygonsAndFailedPaths()
    {
        var blocked = new Grid(12, 9);
        blocked.SetRect(0, 0, 12, 9, AreaCosts.Wall);
        NavMesh m = NavMesh.Build2D(blocked, Fast);
        Assert.Equal(0, m.Tiles.TileCount);
        PathResult p = m.FindPath(new Vector3D(2, 0, 2), new Vector3D(8, 0, 6), cancellationToken: TestContext.Current.CancellationToken);
        Assert.False(p.HasPath);

        // A grid too small for the agent to stand anywhere behaves the same.
        NavMesh tiny = NavMesh.Build2D(new Grid(1, 1), Fast);
        Assert.False(tiny.FindPath(new Vector3D(0.5, 0, 0.5), new Vector3D(0.6, 0, 0.6), cancellationToken: TestContext.Current.CancellationToken).HasPath);

        Assert.Throws<ArgumentNullException>(() => NavMesh.Build2D((Grid)null!, Fast));
    }
}
