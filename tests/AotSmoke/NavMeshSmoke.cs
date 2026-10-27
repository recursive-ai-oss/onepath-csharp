using System;
using RecursiveAI.OnePath.NavMesh;

/// <summary>Builds, stitches, serializes and re-reads a tiny two-tile mesh; used by the NativeAOT and WebAssembly smoke apps.</summary>
internal static class NavMeshSmoke
{
    public static bool Run()
    {
        var p = new NavMeshParams { TileSize = 8, CellSize = 0.25 };
        var mesh = new TiledNavMesh(p);
        TileRef a = mesh.AddTile(Strip(p, 0));
        TileRef b = mesh.AddTile(Strip(p, 1));
        byte[] bytes = NavMeshSerializer.WriteMesh(mesh);
        TiledNavMesh back = NavMeshSerializer.ReadMesh(bytes);
        bool same = bytes.AsSpan().SequenceEqual(NavMeshSerializer.WriteMesh(back));
        bool linked = back.GetLinks(a.Poly(0)).Length == 1 && back.GetLinks(b.Poly(0)).Length == 1;
        return same && linked && NavMeshValidator.Validate(back).Count == 0;
    }

    private static NavTile Strip(NavMeshParams p, int tx)
    {
        double ox = p.TileOriginX(tx);
        double[] v = [ox, 0, 2, ox + 8, 0, 2, ox + 8, 0, 4, ox, 0, 4];
        return NavTile.Create(NavTileAssembler.FromPolygons(p, tx, 0, 0, 0, v, [4], [0, 1, 2, 3]), p);
    }
}
