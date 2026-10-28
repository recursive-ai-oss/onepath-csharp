using System;
using System.Collections.Generic;
using System.Security.Cryptography;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class NavMeshSnapshotTests
{
    private static TiledNavMesh World(ulong seed) => NavMeshWorlds.RandomMesh(seed, -1, -1, 2, 2, 2, detail: true, offMesh: 2);

    private static PolyRef[] AllPolyRefs(TiledNavMesh m)
    {
        var list = new List<PolyRef>();
        for (int s = 0; s < m.SlotCount; s++)
        {
            if (m.GetTileBySlot(s) is { } t)
            {
                for (int p = 0; p < t.PolyCount; p++)
                {
                    list.Add(m.GetSlotRef(s).Poly(p));
                }
            }
        }
        return list.ToArray();
    }

    // Validity of the references (and of forged neighbours of them with other generations and slots) as one bit string.
    private static string Validity(TiledNavMesh m, PolyRef[] refs)
    {
        var sb = new System.Text.StringBuilder();
        foreach (PolyRef r in refs)
        {
            sb.Append(m.IsValid(r) ? '1' : '0');
            sb.Append(m.IsValid(new PolyRef(r.Value + (1UL << 40))) ? '1' : '0');
            sb.Append(m.IsValid(new PolyRef(r.Value - (1UL << 40))) ? '1' : '0');
            sb.Append(m.IsValid(r.Tile) ? '1' : '0');
        }
        return sb.ToString();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    [InlineData(4)]
    [InlineData(5)]
    [InlineData(6)]
    [InlineData(7)]
    public void ExcursionThenRestoreEqualsNeverLeaving(int group)
    {
        int changedByExcursion = 0;
        for (int i = 0; i < 6; i++)
        {
            int seed = (group * 6) + i;
            ulong world = 100 + (ulong)seed;
            var ops = new NavMeshWorlds.Rng(1000 + (ulong)seed);
            int prefix = ops.Int(60);
            int excursion = 1 + ops.Int(40);
            int tail = 1 + ops.Int(30);

            TiledNavMesh a = World(world);
            TiledNavMesh b = World(world);
            var ra = new NavMeshWorlds.Rng((ulong)seed);
            var rb = new NavMeshWorlds.Rng((ulong)seed);
            for (int k = 0; k < prefix; k++)
            {
                NavMeshMutations.Apply(a, ra, world);
                NavMeshMutations.Apply(b, rb, world);
            }
            Assert.Equal(NavMeshMutations.Digest(b, 7), NavMeshMutations.Digest(a, 7));

            byte[] full = a.Snapshot();
            byte[] referenced = a.Snapshot(SnapshotMode.Referenced);
            var library = new TileLibrary();
            library.AddAll(a);
            PolyRef[] refs = AllPolyRefs(a);
            // A snapshot of the same state is the same bytes, and taking one does not change the state.
            Assert.Equal(full, a.Snapshot());

            // The excursion: mutate a lot, then roll back.
            var wild = new NavMeshWorlds.Rng(9000 + (ulong)seed);
            for (int k = 0; k < excursion; k++)
            {
                NavMeshMutations.Apply(a, wild, world);
            }
            changedByExcursion += a.Snapshot().AsSpan().SequenceEqual(full) ? 0 : 1;
            a.Restore(full);
            Assert.Equal(full, a.Snapshot());
            Assert.Equal(referenced, a.Snapshot(SnapshotMode.Referenced));
            Assert.Equal(NavMeshMutations.Digest(b, 7), NavMeshMutations.Digest(a, 7));
            Assert.Equal(Validity(b, refs), Validity(a, refs));
            Assert.Empty(NavMeshValidator.Validate(a));

            // Restoring through the referenced form, into a fresh mesh, gives the same state too.
            TiledNavMesh c = NavMeshSnapshot.Restore(referenced, library);
            Assert.Equal(full, c.Snapshot());
            TiledNavMesh d = NavMeshSnapshot.Restore(full);
            Assert.Equal(referenced, d.Snapshot(SnapshotMode.Referenced));

            // Carrying on from the restored state matches carrying on without the excursion, step for step.
            var ta = new NavMeshWorlds.Rng(5000 + (ulong)seed);
            var tb = new NavMeshWorlds.Rng(5000 + (ulong)seed);
            var tc = new NavMeshWorlds.Rng(5000 + (ulong)seed);
            for (int k = 0; k < tail; k++)
            {
                NavMeshMutations.Apply(a, ta, world);
                NavMeshMutations.Apply(b, tb, world);
                NavMeshMutations.Apply(c, tc, world);
                Assert.Equal(b.Snapshot(), a.Snapshot());
                Assert.Equal(b.Snapshot(), c.Snapshot());
            }
            Assert.Equal(NavMeshMutations.Digest(b, 8), NavMeshMutations.Digest(a, 8));
            Assert.Equal(NavMeshMutations.Digest(b, 8), NavMeshMutations.Digest(c, 8));
            Assert.Equal(Validity(b, refs), Validity(a, refs));
        }
        // The excursions are real: they change the state in nearly every run.
        Assert.True(changedByExcursion >= 5, $"only {changedByExcursion} of 6 excursions changed the state");
    }

    [Fact]
    public void EditsAreRuntimeStateNotPartOfTheTile()
    {
        NavMeshParams p = NavMeshWorlds.Params();
        NavTile tile = NavMeshWorlds.MakeTile(p, 3, 0, 0, 0, detail: false, offMesh: 0);
        byte area0 = tile.PolyAreas[0];
        ushort flags0 = tile.PolyFlags[0];
        TileContentId id = tile.ContentId;
        var m1 = new TiledNavMesh(p);
        var m2 = new TiledNavMesh(p);
        TileRef r1 = m1.AddTile(tile);
        TileRef r2 = m2.AddTile(tile);
        Assert.True(m1.SetPolyFlags(r1.Poly(0), (ushort)(flags0 ^ 4)));
        Assert.True(m1.SetPolyArea(r1.Poly(0), (byte)(area0 ^ 1)));
        // Neither the tile object nor the other mesh sees it, and the tile's identity is its as-built content.
        Assert.Equal(area0, tile.PolyAreas[0]);
        Assert.Equal(flags0, tile.PolyFlags[0]);
        Assert.Equal(flags0, m2.GetTile(r2)!.PolyFlags[0]);
        Assert.Equal(id, m1.GetTile(r1)!.ContentId);
        Assert.Equal((ushort)(flags0 ^ 4), m1.GetTile(r1)!.PolyFlags[0]);
        // Stale and out-of-range edits change nothing.
        Assert.False(m1.SetPolyFlags(new PolyRef(r1.Poly(0).Value + (1UL << 40)), 1));
        Assert.False(m1.SetPolyFlags(r1.Poly(tile.PolyCount), 1));
        Assert.Throws<ArgumentOutOfRangeException>(() => m1.SetPolyArea(r1.Poly(0), 64));
        // Edits reach the serialized mesh but not the snapshot's tile identity: restoring the edit needs the edit record.
        Assert.NotEqual(m1.Snapshot(), m2.Snapshot());
        TiledNavMesh back = NavMeshSnapshot.Restore(m1.Snapshot());
        Assert.Equal((ushort)(flags0 ^ 4), back.GetTile(back.GetTileRef(0, 0, 0))!.PolyFlags[0]);
        // Editing back to the as-built values leaves no trace.
        m1.SetPolyFlags(r1.Poly(0), flags0);
        m1.SetPolyArea(r1.Poly(0), area0);
        Assert.Equal(m2.Snapshot(), m1.Snapshot());
    }

    [Fact]
    public void DisabledOffMeshLinksHaveNoStitchedLinksAndComeBack()
    {
        TiledNavMesh m = World(321);
        var before = new List<long>();
        NavMeshWorlds.DumpLinks(m, before.Add);
        (TileRef tile, int link)? found = null;
        for (int s = 0; s < m.SlotCount && found is null; s++)
        {
            if (m.GetTileBySlot(s) is { OffMeshLinks.Length: > 0 })
            {
                found = (m.GetSlotRef(s), 0);
            }
        }
        Assert.NotNull(found);
        var (tr, li) = found.Value;
        Assert.True(m.SetOffMeshLinkEnabled(tr, li, false));
        Assert.False(m.IsOffMeshLinkEnabled(tr, li));
        var off = new List<long>();
        NavMeshWorlds.DumpLinks(m, off.Add);
        Assert.NotEqual(before, off);
        Assert.Empty(NavMeshValidator.Validate(m));
        byte[] snap = m.Snapshot();
        Assert.True(m.SetOffMeshLinkEnabled(tr, li, true));
        var again = new List<long>();
        NavMeshWorlds.DumpLinks(m, again.Add);
        Assert.Equal(before, again);
        m.Restore(snap);
        Assert.False(m.IsOffMeshLinkEnabled(tr, li));
        Assert.False(m.SetOffMeshLinkEnabled(tr, 99, false));
    }

    [Fact]
    public void MissingLibraryTileAndForeignParametersAreRejected()
    {
        TiledNavMesh m = World(5);
        byte[] referenced = m.Snapshot(SnapshotMode.Referenced);
        Assert.Contains("tile library", Assert.Throws<NavMeshFormatException>(() => NavMeshSnapshot.Restore(referenced)).Message);
        var partial = new TileLibrary();
        partial.Add(m.GetTileBySlot(0)!);
        Assert.Throws<NavMeshFormatException>(() => NavMeshSnapshot.Restore(referenced, partial));
        var other = new TiledNavMesh(new NavMeshParams { OriginX = -16, OriginZ = -16, TileSize = 8, CellSize = 0.25, WalkableClimb = 0.75f });
        Assert.Contains("fingerprint", Assert.Throws<NavMeshFingerprintException>(() => other.Restore(m.Snapshot())).Message);
        Assert.Equal(0, other.TileCount);
    }

    [Fact]
    public void HostileBytesNeverHalfRestore()
    {
        TiledNavMesh src = NavMeshWorlds.RandomMesh(9, 0, 0, 2, 1, 1, detail: false, offMesh: 1);
        src.SetPolyFlags(src.GetSlotRef(0).Poly(0), 6);
        byte[] full = src.Snapshot();
        byte[] referenced = src.Snapshot(SnapshotMode.Referenced);
        var library = new TileLibrary();
        library.AddAll(src);

        TiledNavMesh target = World(77);
        ulong digest = NavMeshMutations.Digest(target, 3);
        byte[] state = target.Snapshot();

        // Every proper prefix is rejected with the typed error and leaves the target alone.
        for (int n = 0; n < referenced.Length; n++)
        {
            Assert.Throws<NavMeshFormatException>(() => target.Restore(referenced.AsSpan(0, n), library));
        }
        for (int n = 0; n < full.Length; n += 1 + (n / 64))
        {
            Assert.Throws<NavMeshFormatException>(() => target.Restore(full.AsSpan(0, n)));
        }
        Assert.Equal(state, target.Snapshot());

        // Random corruption: either a typed error (target untouched) or a mesh that is internally consistent.
        var r = new NavMeshWorlds.Rng(42);
        for (int i = 0; i < 4000; i++)
        {
            byte[] bad = (byte[])(i % 2 == 0 ? full : referenced).Clone();
            for (int k = 1 + r.Int(3); k > 0; k--)
            {
                bad[r.Int(bad.Length)] = (byte)r.Int(256);
            }
            TiledNavMesh t2 = World(77);
            byte[] before = t2.Snapshot();
            try
            {
                t2.Restore(bad, library);
                Assert.Empty(NavMeshValidator.Validate(t2));
                Assert.Equal(t2.Snapshot(), NavMeshSnapshot.Restore(t2.Snapshot()).Snapshot());
            }
            catch (NavMeshFormatException)
            {
                Assert.Equal(before, t2.Snapshot());
            }
        }
        Assert.Equal(digest, NavMeshMutations.Digest(target, 3));
        Assert.Throws<NavMeshFormatException>(() => target.Restore(new byte[] { 1, 2, 3 }));
        Assert.Throws<NavMeshFormatException>(() => target.Restore(NavMeshSerializer.WriteMesh(src)));
    }

    [Fact]
    public void UnknownOrRepeatedSectionsAreRejected()
    {
        byte[] good = NavMeshWorlds.RandomMesh(2, 0, 0, 1, 1, 1, false, 0).Snapshot(SnapshotMode.Referenced);
        // Section table: count at 8 + 68, then id (u16), reserved (u16), length (u32).
        int at = 8 + 68;
        byte[] unknown = (byte[])good.Clone();
        unknown[at + 4] = 2;
        Assert.Contains("section", Assert.Throws<NavMeshFormatException>(() => NavMeshSnapshot.Restore(unknown, new TileLibrary())).Message);
        byte[] none = (byte[])good.Clone();
        none[at] = 0;
        Assert.Throws<NavMeshFormatException>(() => NavMeshSnapshot.Restore(none.AsSpan(0, at + 4)));
    }

    [Fact]
    public void ContentIdIsTheSha256OfTheSerializedTile()
    {
        // Tiles of many different byte lengths cover every padding case of the hash.
        NavMeshParams p = NavMeshWorlds.Params();
        var lengths = new HashSet<int>();
        for (ulong seed = 1; seed <= 80; seed++)
        {
            NavTile t = NavMeshWorlds.MakeTile(p, seed, (int)(seed % 3), 0, 0, seed % 2 == 0, (int)(seed % 3));
            byte[] bytes = NavMeshSerializer.WriteTile(t);
            lengths.Add(bytes.Length % 64);
            Assert.Equal(Convert.ToHexStringLower(SHA256.HashData(bytes)), t.ContentId.ToString());
        }
        Assert.True(lengths.Count > 20);
    }
}
