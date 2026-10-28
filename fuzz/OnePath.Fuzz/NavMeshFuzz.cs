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

    // The tier-1 loaders: a saved mesh or world either throws a typed format error or loads, passes the mesh validator,
    // and saves back to exactly the same bytes.
    public static void Load(byte[] data)
    {
        try
        {
            NavMesh nav = NavMesh.Load(data);
            if (NavMeshValidator.Validate(nav.Tiles).Count != 0 || !nav.Save().AsSpan().SequenceEqual(data))
            {
                throw new InvalidOperationException("accepted mesh is inconsistent or does not round-trip");
            }
            return;
        }
        catch (OnePathFormatException)
        {
        }
        try
        {
            NavWorld world = NavWorld.Load(data);
            if (!world.Save().AsSpan().SequenceEqual(data))
            {
                throw new InvalidOperationException("accepted world does not round-trip");
            }
        }
        catch (OnePathFormatException)
        {
        }
    }

    public static void WriteLoadSeeds(string dir)
    {
        Directory.CreateDirectory(dir);
        var cells = new bool[8 * 8];
        Array.Fill(cells, true);
        cells[27] = false;
        NavSettings s = NavSettings.Humanoid with { CellSize = 0.5, TileSize = 4, OffMeshLinks = [OffMeshLinkDefinition.Create(new RecursiveAI.OneMath.Vector3D(1.5, 0, 1.5), new RecursiveAI.OneMath.Vector3D(6.5, 0, 6.5), 3)] };
        NavMesh nav = NavMesh.Build2D(cells, 8, 8, 1.0, new RecursiveAI.OneMath.Vector2D(0, 0), s);
        File.WriteAllBytes(Path.Combine(dir, "mesh.bin"), nav.Save());
        File.WriteAllBytes(Path.Combine(dir, "mesh-plain.bin"), NavMesh.Build2D(cells, 8, 8, 1.0, new RecursiveAI.OneMath.Vector2D(0, 0), NavSettings.Humanoid with { CellSize = 0.5, TileSize = 4 }).Save());
        var ring = new RecursiveAI.OneMath.Vector2D[] { new(0, 0), new(8, 0), new(8, 8), new(0, 8) };
        File.WriteAllBytes(Path.Combine(dir, "world.bin"), NavWorld.Build2D([new RecursiveAI.OnePath.Mesh.Build2D.Outline(ring)], NavSettings.Humanoid with { CellSize = 0.5, TileSize = 4 }, NavSettings.Small with { CellSize = 0.5, TileSize = 4 }).Save());
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
