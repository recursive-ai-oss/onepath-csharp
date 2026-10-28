using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Debug;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class NavMeshFormatTests
{
    private static List<long> Links(TiledNavMesh m)
    {
        var l = new List<long>();
        NavMeshWorlds.DumpLinks(m, v => l.Add(v));
        return l;
    }

    private static int PortalLinkCount(TiledNavMesh m)
    {
        int n = 0;
        for (int s = 0; s < m.SlotCount; s++)
        {
            NavTile? t = m.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                foreach (NavLink l in m.GetLinks(m.GetSlotRef(s).Poly(p)))
                {
                    if (l.Kind == NavLinkKind.Portal)
                    {
                        n++;
                    }
                }
            }
        }
        return n;
    }

    [Fact]
    public void RoundTripGivesIdenticalBytesAndLinks()
    {
        int portals = 0;
        for (ulong seed = 1; seed <= 24; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, -1, -1, 3, 2, 1 + (int)(seed % 3), detail: seed % 2 == 0, offMesh: (int)(seed % 4));
            Assert.Empty(NavMeshValidator.Validate(m));
            byte[] a = NavMeshSerializer.WriteMesh(m);
            TiledNavMesh r = NavMeshSerializer.ReadMesh(a);
            byte[] b = NavMeshSerializer.WriteMesh(r);
            Assert.True(a.AsSpan().SequenceEqual(b), $"seed {seed}");
            Assert.Equal(Links(m), Links(r));
            Assert.Empty(NavMeshValidator.Validate(r));
            portals += PortalLinkCount(m);
        }
        Assert.True(portals > 100, "generated meshes should contain many portal links");
    }

    [Fact]
    public void TileRoundTrip()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        NavTile t = NavMeshWorlds.MakeTile(p, 3, 1, 0, 0, detail: true, offMesh: 3);
        byte[] a = NavMeshSerializer.WriteTile(t);
        NavTile u = NavMeshSerializer.ReadTile(a, p);
        Assert.True(a.AsSpan().SequenceEqual(NavMeshSerializer.WriteTile(u)));
        Assert.Equal(t.PolyCount, u.PolyCount);
    }

    [Fact]
    public void FrozenHeaderIsVersionTwoWithNoDraftFlag()
    {
        NavTile t = NavMeshWorlds.MakeTile(NavMeshWorlds.Params(), 3, 0, 0, 0, false, 0);
        byte[] a = NavMeshSerializer.WriteTile(t);
        Assert.Equal((byte)'O', a[0]);
        Assert.Equal(2, NavMeshFormat.Version);
        Assert.Equal(NavMeshFormat.Version, BitConverter.ToUInt16(a, 4));
        Assert.Equal(0, BitConverter.ToUInt16(a, 6));
        Assert.False(NavMeshFormat.IsUnstable);
        Assert.True(a.AsSpan(8, 16).SequenceEqual(FingerprintBytes(NavMeshWorlds.Params())));
    }

    private static byte[] FingerprintBytes(NavMeshParams p)
    {
        var b = new byte[16];
        p.Fingerprint.CopyTo(b);
        return b;
    }

    [Fact]
    public void VersionOneAndUnknownVersionsAreRejectedWithTheVersionsNamed()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        byte[] tile = NavMeshSerializer.WriteTile(NavMeshWorlds.MakeTile(p, 3, 0, 0, 0, false, 0));
        byte[] mesh = NavMeshSerializer.WriteMesh(NavMeshWorlds.RandomMesh(5, 0, 0, 2, 2, 1, false, 0));
        foreach (ushort v in new ushort[] { 1, 3, 0, 65535 })
        {
            byte[] t = (byte[])tile.Clone();
            BitConverter.TryWriteBytes(t.AsSpan(4), v);
            var e = Assert.Throws<NavMeshVersionException>(() => NavMeshSerializer.ReadTile(t, p));
            Assert.Equal(v, e.Found);
            Assert.Equal(2, e.Expected);
            Assert.Contains($"version {v}", e.Message);
            byte[] m = (byte[])mesh.Clone();
            BitConverter.TryWriteBytes(m.AsSpan(4), v);
            Assert.Throws<NavMeshVersionException>(() => NavMeshSerializer.ReadMesh(m));
        }
        byte[] draft = (byte[])tile.Clone();
        draft[6] = 1; // the unstable flag is not valid in a frozen version
        Assert.Throws<NavMeshFormatException>(() => NavMeshSerializer.ReadTile(draft, p));
    }

    [Fact]
    public void TileFromOtherSettingsIsRejectedOnAddReadAndStreamIn()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        TiledNavMesh mesh = new(p);
        NavTile good = NavMeshWorlds.MakeTile(p, 3, 0, 0, 0, false, 0);
        mesh.AddTile(good);

        // Same grid, other agent class and other build digest: every variant must be refused.
        NavMeshParams[] others =
        [
            Variant(p, radius: p.AgentRadius + 0.1f),
            Variant(p, climb: p.WalkableClimb + 0.1f),
            Variant(p, digest: 1),
        ];
        foreach (NavMeshParams o in others)
        {
            NavTile foreign = NavMeshWorlds.MakeTile(o, 4, 1, 0, 0, false, 0);
            Assert.NotEqual(p.Fingerprint, o.Fingerprint);
            var e = Assert.Throws<NavMeshFingerprintException>(() => mesh.AddTile(foreign));
            Assert.Equal(p.Fingerprint, e.Expected);
            Assert.Equal(o.Fingerprint, e.Found);
            Assert.Throws<NavMeshFingerprintException>(() => mesh.ReplaceTile(foreign));
            byte[] bytes = NavMeshSerializer.WriteTile(foreign);
            Assert.Throws<NavMeshFingerprintException>(() => NavMeshSerializer.ReadTile(bytes, p));
            var streamer = new TileStreamer(mesh);
            streamer.QueueAdd(bytes);
            Assert.Throws<NavMeshFingerprintException>(() => streamer.UpdateAll(TestContext.Current.CancellationToken));
            Assert.Equal(0, streamer.PendingCount);
        }
        Assert.Equal(1, mesh.TileCount);

        // The right tile streams in and the mesh round-trips.
        byte[] ok = NavMeshSerializer.WriteTile(NavMeshWorlds.MakeTile(p, 5, 1, 0, 0, false, 0));
        var s2 = new TileStreamer(mesh);
        s2.QueueAdd(ok);
        s2.UpdateAll(TestContext.Current.CancellationToken);
        Assert.Equal(2, mesh.TileCount);
        byte[] a = NavMeshSerializer.WriteMesh(mesh);
        Assert.True(a.AsSpan().SequenceEqual(NavMeshSerializer.WriteMesh(NavMeshSerializer.ReadMesh(a))));
    }

    private static NavMeshParams Variant(NavMeshParams p, float? radius = null, float? climb = null, ulong? digest = null) => new()
    {
        OriginX = p.OriginX,
        OriginY = p.OriginY,
        OriginZ = p.OriginZ,
        TileSize = p.TileSize,
        CellSize = p.CellSize,
        MaxVertsPerPoly = p.MaxVertsPerPoly,
        WalkableClimb = climb ?? p.WalkableClimb,
        AgentRadius = radius ?? p.AgentRadius,
        AgentHeight = p.AgentHeight,
        AgentMaxSlopeDegrees = p.AgentMaxSlopeDegrees,
        SettingsDigest = digest ?? p.SettingsDigest,
    };

    [Fact]
    public void FingerprintIsFixedAndSettingsDigestFollowsBuildSettings()
    {
        NavMeshParams a = NavSettings.Humanoid.ToParams();
        Assert.Equal(a.Fingerprint, NavSettings.Humanoid.ToParams().Fingerprint);
        NavSettings[] changed =
        [
            NavSettings.Humanoid with { CellHeight = 0.25 },
            NavSettings.Humanoid with { MinRegionArea = 10 },
            NavSettings.Humanoid with { DetailMaxError = 0.3 },
            NavSettings.Humanoid with { Partition = RecursiveAI.OnePath.PartitionType.Layers },
            NavSettings.Humanoid with { SnapBits = 8 },
            NavSettings.Humanoid with { AgentRadius = 0.7f },
            NavSettings.Humanoid with { TileSize = 32 },
        ];
        var seen = new HashSet<NavMeshFingerprint> { a.Fingerprint };
        foreach (NavSettings s in changed)
        {
            Assert.True(seen.Add(s.ToParams().Fingerprint), "each changed setting changes the fingerprint");
        }
        // A frozen value: any change to the canonical encoding changes this and needs a new format version.
        Assert.Equal("6316e2fbf9f85bbf39246de10d1a5c22", NavSettings.Humanoid.ToParams().Fingerprint.ToString());
    }

    [Fact]
    public void IncrementalEditsEqualFreshStitch()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        var rng = new NavMeshWorlds.Rng(77);
        for (int round = 0; round < 6; round++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(10UL + (ulong)round, 0, 0, 3, 3, 2, false, round % 2);
            for (int step = 0; step < 25; step++)
            {
                int x = rng.Int(4), z = rng.Int(3), l = rng.Int(3);
                switch (rng.Int(3))
                {
                    case 0:
                        m.RemoveTile(x, z, l);
                        break;
                    case 1:
                        m.ReplaceTile(NavMeshWorlds.MakeTile(p, (ulong)rng.Int(1000), x, z, l, false, rng.Int(3) == 0 ? 1 : 0));
                        break;
                    default:
                        if (!m.GetTileRef(x, z, l).IsSet)
                        {
                            m.AddTile(NavMeshWorlds.MakeTile(p, (ulong)rng.Int(1000), x, z, l, false, 0));
                        }
                        break;
                }
                // Reading the serialized mesh restitches everything from scratch, with the same slots.
                TiledNavMesh fresh = NavMeshSerializer.ReadMesh(NavMeshSerializer.WriteMesh(m));
                Assert.Equal(Links(fresh), Links(m));
                Assert.Empty(NavMeshValidator.Validate(m));
            }
        }
    }

    [Fact]
    public void LinksDoNotDependOnInsertionOrder()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        var tiles = new List<NavTile>();
        for (int x = 0; x < 3; x++)
        {
            for (int z = 0; z < 3; z++)
            {
                tiles.Add(NavMeshWorlds.MakeTile(p, 5, x, z, 0, false, 0));
            }
        }
        var forward = new TiledNavMesh(p);
        foreach (NavTile t in tiles)
        {
            forward.AddTile(t);
        }
        var backward = new TiledNavMesh(p);
        for (int i = tiles.Count - 1; i >= 0; i--)
        {
            backward.AddTile(tiles[i]);
        }
        // Slots differ, so compare by tile position: each polygon's links as (target tile x, z, poly, side, range).
        static List<string> ByPosition(TiledNavMesh m)
        {
            var o = new List<string>();
            for (int s = 0; s < m.SlotCount; s++)
            {
                NavTile t = m.GetTileBySlot(s)!;
                for (int pi = 0; pi < t.PolyCount; pi++)
                {
                    var parts = new List<string>();
                    foreach (NavLink l in m.GetLinks(m.GetSlotRef(s).Poly(pi)))
                    {
                        NavTile tt = m.GetTileBySlot(l.Target.TileSlot)!;
                        parts.Add($"{tt.TileX},{tt.TileZ},{l.Target.PolyIndex},{l.Side},{l.RangeMin},{l.RangeMax},{l.Index}");
                    }
                    o.Add($"{t.TileX},{t.TileZ},{pi}:{string.Join(';', parts)}");
                }
            }
            o.Sort(StringComparer.Ordinal);
            return o;
        }
        Assert.Equal(ByPosition(forward), ByPosition(backward));
    }

    [Fact]
    public void ReferencesAreInvalidatedByRemoveAndReplace()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        var m = new TiledNavMesh(p);
        TileRef t0 = m.AddTile(NavMeshWorlds.MakeTile(p, 1, 0, 0, 0, false, 0));
        TileRef t1 = m.AddTile(NavMeshWorlds.MakeTile(p, 1, 1, 0, 0, false, 0));
        PolyRef a = m.GetPolyRef(t0, 0);
        PolyRef b = m.GetPolyRef(t1, 0);
        Assert.True(m.IsValid(a) && m.IsValid(b));

        TileRef t0b = m.ReplaceTile(NavMeshWorlds.MakeTile(p, 2, 0, 0, 0, false, 0));
        Assert.Equal(t0.Slot, t0b.Slot);
        Assert.NotEqual(t0.Salt, t0b.Salt);
        Assert.False(m.IsValid(a));
        Assert.True(m.IsValid(b));
        Assert.False(m.IsValid(t0));
        Assert.True(m.IsValid(t0b));

        Assert.True(m.RemoveTile(t1));
        Assert.False(m.RemoveTile(t1));
        Assert.False(m.IsValid(b));
        TileRef t2 = m.AddTile(NavMeshWorlds.MakeTile(p, 3, 2, 0, 0, false, 0));
        Assert.Equal(t1.Slot, t2.Slot); // lowest free slot is reused
        Assert.NotEqual(t1.Salt, t2.Salt);
        Assert.False(m.IsValid(b));
        Assert.False(m.IsValid(PolyRef.Invalid));
        Assert.False(m.IsValid(new PolyRef(ulong.MaxValue)));
        Assert.Empty(m.GetLinks(b).ToArray());
        Assert.Throws<InvalidOperationException>(() => m.AddTile(NavMeshWorlds.MakeTile(p, 3, 2, 0, 0, false, 0)));
    }

    [Fact]
    public void GenerationsSurviveSerialization()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        var m = new TiledNavMesh(p);
        TileRef t0 = m.AddTile(NavMeshWorlds.MakeTile(p, 1, 0, 0, 0, false, 0));
        m.ReplaceTile(NavMeshWorlds.MakeTile(p, 2, 0, 0, 0, false, 0));
        m.ReplaceTile(NavMeshWorlds.MakeTile(p, 3, 0, 0, 0, false, 0));
        TileRef t1 = m.AddTile(NavMeshWorlds.MakeTile(p, 3, 1, 0, 0, false, 0));
        m.RemoveTile(t1);
        TiledNavMesh r = NavMeshSerializer.ReadMesh(NavMeshSerializer.WriteMesh(m));
        Assert.Equal(m.GetSlotSalt(0), r.GetSlotSalt(0));
        Assert.Equal(m.GetSlotSalt(1), r.GetSlotSalt(1));
        Assert.False(r.IsValid(t0));
        Assert.True(r.IsValid(r.GetTileRef(0, 0, 0)));
        Assert.Equal(m.GetTileRef(0, 0, 0), r.GetTileRef(0, 0, 0));
    }

    [Fact]
    public void OffMeshLinksConnectBothWays()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        var m = new TiledNavMesh(p);
        // Two solid squares far apart; a bidirectional link between them.
        NavTileData Square(int tx, bool link)
        {
            double ox = p.TileOriginX(tx), oz = p.TileOriginZ(0);
            double[] v = [ox + 1, 0, oz + 1, ox + 3, 0, oz + 1, ox + 3, 0, oz + 3, ox + 1, 0, oz + 3];
            NavTileData d = NavTileAssembler.FromPolygons(p, tx, 0, 0, 0, v, [4], [0, 1, 2, 3]);
            if (link)
            {
                d.OffMeshLinks = [new OffMeshLink { StartX = 2, StartY = 0, StartZ = 2, EndX = 2 + 16, EndY = 0.2f, EndZ = 2, Radius = 0.5f, Bidirectional = true, Area = 9, UserId = 7 }];
            }
            return d;
        }
        TileRef a = m.AddTile(NavTile.Create(Square(0, true), p));
        TileRef b = m.AddTile(NavTile.Create(Square(2, false), p));
        var la = m.GetLinks(a.Poly(0)).ToArray();
        var lb = m.GetLinks(b.Poly(0)).ToArray();
        Assert.Single(la);
        Assert.Single(lb);
        Assert.Equal(NavLinkKind.OffMesh, la[0].Kind);
        Assert.Equal(b.Poly(0), la[0].Target);
        Assert.Equal(NavLinkKind.OffMeshReverse, lb[0].Kind);
        Assert.Equal(a.Poly(0), lb[0].Target);
        Assert.Empty(NavMeshValidator.Validate(m));
        // Removing the far tile drops the link; re-adding restores it.
        m.RemoveTile(b);
        Assert.Empty(m.GetLinks(a.Poly(0)).ToArray());
        TileRef b2 = m.AddTile(NavTile.Create(Square(2, false), p));
        Assert.Single(m.GetLinks(a.Poly(0)).ToArray());
        Assert.Equal(b2.Poly(0), m.GetLinks(a.Poly(0))[0].Target);
    }

    [Fact]
    public void PortalsJoinOnlyWithinClimb()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        NavTileData Strip(int tx, double y)
        {
            double ox = p.TileOriginX(tx), oz = p.TileOriginZ(0);
            double[] v = [ox, y, oz + 2, ox + 8, y, oz + 2, ox + 8, y, oz + 4, ox, y, oz + 4];
            return NavTileAssembler.FromPolygons(p, tx, 0, 0, 0, v, [4], [0, 1, 2, 3]);
        }
        foreach ((double dy, bool linked) in new[] { (0.0, true), (0.5, true), (0.75, false) })
        {
            var m = new TiledNavMesh(p);
            TileRef a = m.AddTile(NavTile.Create(Strip(0, 0), p));
            TileRef b = m.AddTile(NavTile.Create(Strip(1, dy), p));
            Assert.Equal(linked ? 1 : 0, m.GetLinks(a.Poly(0)).Length);
            Assert.Equal(linked ? 1 : 0, m.GetLinks(b.Poly(0)).Length);
            if (linked)
            {
                NavLink l = m.GetLinks(a.Poly(0))[0];
                Assert.Equal(0, l.Side);
                Assert.Equal(0, l.RangeMin);
                Assert.Equal(255, l.RangeMax);
            }
        }
    }

    [Fact]
    public void ValidationRejectsBadTiles()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        NavTileData Good() => NavMeshWorlds.MakeTile(p, 4, 0, 0, 0, true, 1).ToData();
        void Bad(Action<NavTileData> mutate, string expect)
        {
            NavTileData d = Good();
            mutate(d);
            var e = Assert.Throws<NavMeshFormatException>(() => NavTile.Create(d, p));
            Assert.Contains(expect, e.Message, StringComparison.Ordinal);
        }
        _ = NavTile.Create(Good(), p);
        Bad(d => d.Vertices![0] = float.NaN, "non-finite");
        Bad(d => d.Vertices![1] = 1e9f, "outside the tile-local bounds");
        Bad(d => d.PolyIndices![0] = 60000, "refers to vertex");
        Bad(d => d.PolyAreas![0] = 200, "area");
        Bad(d => d.EdgeNeighbors![0] = 9999, "neighbour code");
        Bad(d => d.PolyVertexCounts![0] = 9, "sum of PolyVertexCounts");
        Bad(d => d.Layer = -1, "layer");
        Bad(d => d.DetailMeshes![0].VertexBase = 1_000_000, "detail range");
        Bad(d => d.DetailTriangles![0] = 200, "detail triangle");
        Bad(d => d.OffMeshLinks![0] = default, "off-mesh");
        Bad(d => d.PolyAreas = [], "one entry per polygon");
        // Reversed winding is not convex-and-counter-clockwise.
        Bad(d => Array.Reverse(d.PolyIndices!, 0, d.PolyVertexCounts![0]), "convex");
        // A neighbour that does not point back.
        Bad(
            d =>
            {
                for (int i = 0; i < d.EdgeNeighbors!.Length; i++)
                {
                    if (d.EdgeNeighbors[i] != 0)
                    {
                        d.EdgeNeighbors[i] = 0;
                        return;
                    }
                }
            },
            "neighbour");
        // Concave quad.
        var concave = new NavTileData
        {
            Vertices = [0, 0, 0, 4, 0, 0, 1, 0, 1, 0, 0, 4],
            PolyVertexCounts = [4],
            PolyIndices = [0, 1, 2, 3],
            EdgeNeighbors = [0, 0, 0, 0],
            EdgePortalSides = [255, 255, 255, 255],
            PolyAreas = [0],
            PolyFlags = [0],
        };
        Assert.Contains("not convex", Assert.Throws<NavMeshFormatException>(() => NavTile.Create(concave, p)).Message, StringComparison.Ordinal);
        // A portal edge that is not on the tile boundary.
        var portal = new NavTileData
        {
            Vertices = [1, 0, 0, 4, 0, 0, 4, 0, 4, 1, 0, 4],
            PolyVertexCounts = [4],
            PolyIndices = [0, 1, 2, 3],
            EdgeNeighbors = [0, 0, 0, 0],
            EdgePortalSides = [255, 0, 255, 255],
            PolyAreas = [0],
            PolyFlags = [0],
        };
        Assert.Contains("tile boundary", Assert.Throws<NavMeshFormatException>(() => NavTile.Create(portal, p)).Message, StringComparison.Ordinal);
    }

    [Fact]
    public void CoordinateRuleIsEnforced()
    {
        var far = new NavMeshParams { OriginX = 3_000_000 };
        Assert.Throws<ArgumentOutOfRangeException>(() => new TiledNavMesh(far));
        NavMeshParams p = NavMeshWorlds.Params();
        var e = Assert.Throws<ArgumentOutOfRangeException>(() => NavTileAssembler.FromPolygons(p, 0, 0, 0, 0, [0, 0, 0, 1, 0, 0, 3e6, 0, 1], [3], [0, 1, 2]));
        Assert.Contains("2097152", e.Message, StringComparison.Ordinal);
        Assert.Throws<ArgumentOutOfRangeException>(() => p.WorldToTile(2.5e6, 0, out _, out _));
        Assert.Throws<ArgumentOutOfRangeException>(() => new NavMeshParams { TileSize = 0.1 }.Validate()); // not float32-exact
        Assert.Throws<ArgumentOutOfRangeException>(() => new NavMeshParams { TileSize = 4096, CellSize = 0.25 }.Validate()); // beyond 8192 cells
        // A tile whose span leaves the bound.
        var m = new NavMeshParams { TileSize = 64, OriginX = 2_097_152 - 100 };
        var d = new NavTileData { TileX = 1 };
        Assert.Throws<ArgumentOutOfRangeException>(() => NavTile.Create(d, m));
    }

    [Fact]
    public void HostileBytesNeverEscapeAsOtherExceptions()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(31, 0, 0, 2, 1, 2, detail: true, offMesh: 2);
        byte[] good = NavMeshSerializer.WriteMesh(m);
        // Every truncation fails with the typed error.
        for (int len = 0; len < good.Length; len += Math.Max(1, len / 64))
        {
            Assert.Throws<NavMeshFormatException>(() => NavMeshSerializer.ReadMesh(good.AsSpan(0, len)));
        }
        Assert.Throws<NavMeshFormatException>(() => NavMeshSerializer.ReadMesh(Append(good, 0)));
        // Random byte damage either fails typed or yields a mesh that is valid and re-serializes.
        var rng = new NavMeshWorlds.Rng(5);
        int accepted = 0;
        for (int i = 0; i < 3000; i++)
        {
            byte[] bad = (byte[])good.Clone();
            int edits = 1 + rng.Int(3);
            for (int k = 0; k < edits; k++)
            {
                bad[rng.Int(bad.Length)] = (byte)rng.Int(256);
            }
            try
            {
                TiledNavMesh r = NavMeshSerializer.ReadMesh(bad);
                accepted++;
                Assert.Empty(NavMeshValidator.Validate(r));
                _ = NavMeshSerializer.WriteMesh(r);
            }
            catch (NavMeshFormatException)
            {
            }
        }
        Assert.True(accepted < 3000);
    }

    private static byte[] Append(byte[] a, byte b)
    {
        var r = new byte[a.Length + 1];
        a.CopyTo(r, 0);
        r[^1] = b;
        return r;
    }

    [Fact]
    public void AssemblerPairsEdgesAndFlagsPortals()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        // Two clockwise triangles forming the full tile square: the assembler reverses them and pairs the diagonal.
        double ox = p.TileOriginX(0), oz = p.TileOriginZ(0);
        double[] v = [ox, 0, oz, ox + 8, 0, oz, ox + 8, 0, oz + 8, ox, 0, oz + 8];
        NavTileData d = NavTileAssembler.FromPolygons(p, 0, 0, 0, 0, v, [3, 3], [0, 2, 1, 0, 3, 2]);
        NavTile t = NavTile.Create(d, p);
        int paired = 0;
        int portals = 0;
        for (int i = 0; i < t.EdgeNeighbors.Length; i++)
        {
            paired += t.EdgeNeighbors[i] != 0 ? 1 : 0;
            portals += t.EdgePortalSides[i] != NavMeshFormat.NoSide ? 1 : 0;
        }
        Assert.Equal(2, paired);
        Assert.Equal(4, portals);
    }

    [Fact]
    public void DebugGeometryIsPlainWorldData()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(9, 0, 0, 2, 2, 1, false, 1);
        NavMeshDebugGeometry g = NavMeshDebug.Collect(m);
        Assert.NotEmpty(g.Polygons);
        Assert.NotEmpty(g.Links);
        foreach (DebugPolygon poly in g.Polygons)
        {
            foreach (var v in poly.Vertices)
            {
                Assert.InRange(v.X, -16.0, 16.0);
            }
        }
    }
}
