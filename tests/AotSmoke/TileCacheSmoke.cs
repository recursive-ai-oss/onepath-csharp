using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>Run-time obstacles and area volumes on a 3D-built mesh, with snapshot and restore, for the NativeAOT and WebAssembly smoke apps.</summary>
internal static class TileCacheSmoke
{
    public static bool Run()
    {
        double[] v = { 0, 0, 0, 0, 0, 24, 24, 0, 24, 24, 0, 0 };
        int[] ix = { 0, 1, 2, 0, 2, 3 };
        var settings = new NavSettings { CellSize = 0.5, CellHeight = 0.25, TileSize = 12 };
        TileCache cache = new Mesh3DPipeline(settings).BuildSoupCached(v, ix);
        byte[] before = Tiles(cache.Mesh);
        int wall = cache.AddObstacle(TileObstacle.OrientedBox(12, 0, 12, 4, 1, 0.5, 0.5));
        int zone = cache.AddVolume(new ConvexVolume(new double[] { 2, 2, 8, 2, 8, 8, 2, 8 }, -1, 1, 5));
        bool ok = cache.PendingCount > 0;
        cache.Update(1);
        byte[] snap = cache.Snapshot();
        cache.UpdateAll();
        byte[] changed = Tiles(cache.Mesh);
        ok &= !changed.AsSpan().SequenceEqual(before) && cache.PendingCount == 0;
        cache.Restore(snap);
        ok &= cache.Snapshot().AsSpan().SequenceEqual(snap);
        cache.UpdateAll();
        ok &= Tiles(cache.Mesh).AsSpan().SequenceEqual(changed);
        cache.RemoveObstacle(wall);
        cache.RemoveVolume(zone);
        cache.UpdateAll();
        ok &= Tiles(cache.Mesh).AsSpan().SequenceEqual(before) && NavMeshValidator.Validate(cache.Mesh).Count == 0;
        return ok;
    }

    // The tiles by position (the mesh bytes also hold slot generations, which differ after rebuilds).
    private static byte[] Tiles(TiledNavMesh mesh)
    {
        var all = new System.Collections.Generic.List<byte>();
        var refs = new TileRef[4];
        for (int z = 0; z < 2; z++)
        {
            for (int x = 0; x < 2; x++)
            {
                int n = Math.Min(mesh.GetTilesAt(x, z, refs), refs.Length);
                for (int i = 0; i < n; i++)
                {
                    all.AddRange(NavMeshSerializer.WriteTile(mesh.GetTile(refs[i])!));
                }
            }
        }
        return all.ToArray();
    }
}
