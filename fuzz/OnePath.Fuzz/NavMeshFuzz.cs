using System;
using System.IO;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Fuzz;

/// <summary>Fuzz targets for the navigation mesh deserializers.</summary>
internal static class NavMeshFuzz
{
    // Whatever the bytes are, the reader either throws NavMeshFormatException or returns a mesh that is
    // consistent and writes back to exactly the same bytes. Any other exception is a bug and crashes the target.
    public static void Mesh(byte[] data)
    {
        TiledNavMesh mesh;
        try
        {
            mesh = NavMeshSerializer.ReadMesh(data);
        }
        catch (NavMeshFormatException)
        {
            return;
        }
        if (NavMeshValidator.Validate(mesh).Count != 0)
        {
            throw new InvalidOperationException("accepted mesh is inconsistent");
        }
        if (!NavMeshSerializer.WriteMesh(mesh).AsSpan().SequenceEqual(data))
        {
            throw new InvalidOperationException("accepted mesh does not round-trip");
        }
        _ = RecursiveAI.OnePath.Debug.NavMeshDebug.Collect(mesh);
    }

    public static void Tile(byte[] data)
    {
        NavTile tile;
        try
        {
            tile = NavMeshSerializer.ReadTile(data, NavMeshWorlds.Params());
        }
        catch (NavMeshFormatException)
        {
            return;
        }
        if (!NavMeshSerializer.WriteTile(tile).AsSpan().SequenceEqual(data))
        {
            throw new InvalidOperationException("accepted tile does not round-trip");
        }
    }

    public static void WriteSeeds(string dir)
    {
        Directory.CreateDirectory(dir);
        NavMeshParams p = NavMeshWorlds.Params();
        for (ulong seed = 1; seed <= 6; seed++)
        {
            TiledNavMesh m = NavMeshWorlds.RandomMesh(seed, 0, 0, 2, 1, 1 + (int)(seed % 3), seed % 2 == 0, (int)(seed % 3));
            File.WriteAllBytes(Path.Combine(dir, $"mesh-{seed}.bin"), NavMeshSerializer.WriteMesh(m));
            File.WriteAllBytes(Path.Combine(dir, $"tile-{seed}.bin"), NavMeshSerializer.WriteTile(NavMeshWorlds.MakeTile(p, seed, 1, 0, 0, seed % 2 == 0, 2)));
        }
    }
}
