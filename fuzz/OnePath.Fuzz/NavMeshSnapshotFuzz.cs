using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>
/// Fuzz target for snapshot restore. The input is split in two: the first byte picks a base mesh and the rest is either
/// used as a snapshot directly or, for odd first bytes, patched into a valid snapshot of that mesh (so the fuzzer reaches
/// deep into slots, edits and disabled links instead of dying at the magic number). Restore must throw the typed
/// <see cref="NavMeshFormatException"/> and leave the target untouched, or produce a consistent mesh whose snapshot
/// restores to the same bytes. Anything else crashes the target.
/// </summary>
internal static class NavMeshSnapshotFuzz
{
    private static readonly TiledNavMesh s_base = Make();
    private static readonly byte[] s_full = s_base.Snapshot();
    private static readonly byte[] s_referenced = s_base.Snapshot(SnapshotMode.Referenced);
    private static readonly TileLibrary s_library = Library();

    private static TiledNavMesh Make()
    {
        TiledNavMesh m = NavMeshWorlds.RandomMesh(31, 0, 0, 2, 1, 2, detail: false, offMesh: 2);
        var r = new NavMeshWorlds.Rng(5);
        for (int i = 0; i < 20; i++)
        {
            NavMeshMutations.Apply(m, r, 31);
        }
        return m;
    }

    private static TileLibrary Library()
    {
        var l = new TileLibrary();
        l.AddAll(s_base);
        return l;
    }

    public static void Run(byte[] data)
    {
        if (data.Length == 0)
        {
            return;
        }
        byte[] bytes;
        if ((data[0] & 1) == 0)
        {
            bytes = data[1..];
        }
        else
        {
            bytes = (byte[])((data[0] & 2) == 0 ? s_full : s_referenced).Clone();
            for (int i = 1; i + 2 < data.Length; i += 3)
            {
                bytes[((data[i] << 8) | data[i + 1]) % bytes.Length] = data[i + 2];
            }
        }

        TiledNavMesh target = NavMeshWorlds.RandomMesh(3, 0, 0, 1, 1, 1, detail: false, offMesh: 1);
        byte[] before = target.Snapshot();
        try
        {
            target.Restore(bytes, s_library);
        }
        catch (NavMeshFormatException)
        {
            if (!target.Snapshot().AsSpan().SequenceEqual(before))
            {
                throw new InvalidOperationException("a failed restore changed the mesh");
            }
            return;
        }
        if (NavMeshValidator.Validate(target).Count != 0)
        {
            throw new InvalidOperationException("restored mesh is inconsistent");
        }
        byte[] again = target.Snapshot();
        TiledNavMesh round = NavMeshSnapshot.Restore(again);
        if (!round.Snapshot().AsSpan().SequenceEqual(again))
        {
            throw new InvalidOperationException("restored mesh does not snapshot to stable bytes");
        }
        _ = NavMeshMutations.Digest(target, 1);
    }

    public static void WriteSeeds(string dir)
    {
        System.IO.Directory.CreateDirectory(dir);
        var seedList = new List<(string, byte[])>
        {
            ("snap-ref.bin", PrependSelector(0, s_referenced)),
            ("patch-full.bin", [1, 0, 70, 3]),
            ("patch-ref.bin", [3, 0, 80, 1]),
        };
        foreach ((string name, byte[] b) in seedList)
        {
            System.IO.File.WriteAllBytes(System.IO.Path.Combine(dir, name), b);
        }
    }

    private static byte[] PrependSelector(byte sel, byte[] snapshot)
    {
        var b = new byte[snapshot.Length + 1];
        b[0] = sel;
        snapshot.CopyTo(b, 1);
        return b;
    }
}
