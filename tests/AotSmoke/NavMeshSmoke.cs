using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

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
        // Queries: the path across the tile boundary is the straight line, a ray along it is not stopped, one past the end is.
        var q = new NavMeshQuery(back);
        PathResult path = q.FindPath(new Vector3D(0.5, 0, 3), new Vector3D(15.5, 0, 3));
        NearestPoly near = q.FindNearestPoly(new Vector3D(4, 1, 3), new Vector3D(1, 2, 1));
        var visited = new PolyRef[8];
        RaycastResult free = q.Raycast(a.Poly(0), new Vector3D(1, 0, 3), new Vector3D(14, 0, 3), null, visited, out int vc);
        RaycastResult blocked = q.Raycast(a.Poly(0), new Vector3D(1, 0, 3), new Vector3D(14, 0, 5), null, visited, out _);
        bool queries = path.Status == PathStatus.Success && System.Math.Abs(path.Length - 15.0) < 1e-9 && near.Poly == a.Poly(0)
            && !free.Hit && vc == 2 && blocked.Hit;
        return same && linked && queries && NavMeshValidator.Validate(back).Count == 0;
    }

    private static NavTile Strip(NavMeshParams p, int tx)
    {
        double ox = p.TileOriginX(tx);
        double[] v = [ox, 0, 2, ox + 8, 0, 2, ox + 8, 0, 4, ox, 0, 4];
        return NavTile.Create(NavTileAssembler.FromPolygons(p, tx, 0, 0, 0, v, [4], [0, 1, 2, 3]), p);
    }
}
