using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>Generated worlds for the corridor and follower tests and the hash-matrix scenarios: perfect mazes (one polygon route between any two places), scattered obstacles, and the layered random meshes with off-mesh links.</summary>
internal static class CorridorWorlds
{
    public static readonly Vector3D Extents = new(2, 4, 2);

    public struct AllPass : IQueryFilter
    {
        public readonly bool Passes(ushort flags, byte area) => true;

        public readonly double AreaCost(byte area) => 1.0;

        public readonly double MinAreaCost => 1.0;
    }

    public static NavSettings FlatSettings() => NavSettings.Humanoid with { AgentRadius = 0f, CellSize = 0.5, TileSize = 16 };

    /// <summary>A perfect maze of <paramref name="rooms"/> x <paramref name="rooms"/> rooms with one-unit corridors; returns the mesh and the room centres.</summary>
    public static TiledNavMesh Maze(int rooms, ulong seed, out List<Vector3D> centres)
    {
        int w = (2 * rooms) + 1;
        var cells = new bool[w * w];
        var rng = new NavMeshWorlds.Rng(seed);
        var seen = new bool[rooms * rooms];
        var stack = new Stack<int>();
        stack.Push(0);
        seen[0] = true;
        cells[(1 * w) + 1] = true;
        while (stack.Count > 0)
        {
            int cur = stack.Peek();
            int cx = cur % rooms, cy = cur / rooms;
            var options = new List<int>();
            if (cx > 0 && !seen[cur - 1]) { options.Add(0); }
            if (cx < rooms - 1 && !seen[cur + 1]) { options.Add(1); }
            if (cy > 0 && !seen[cur - rooms]) { options.Add(2); }
            if (cy < rooms - 1 && !seen[cur + rooms]) { options.Add(3); }
            if (options.Count == 0)
            {
                stack.Pop();
                continue;
            }
            int d = options[rng.Int(options.Count)];
            int nx = cx + (d == 0 ? -1 : d == 1 ? 1 : 0), ny = cy + (d == 2 ? -1 : d == 3 ? 1 : 0);
            cells[((2 * ny) + 1) * w + (2 * nx) + 1] = true;
            cells[((cy + ny) + 1) * w + (cx + nx) + 1] = true;
            seen[(ny * rooms) + nx] = true;
            stack.Push((ny * rooms) + nx);
        }
        NavMesh nm = NavMesh.Build2D(cells, w, w, 1.0, new Vector2D(0, 0), FlatSettings());
        centres = new List<Vector3D>();
        for (int y = 0; y < rooms; y++)
        {
            for (int x = 0; x < rooms; x++)
            {
                centres.Add(new Vector3D((2 * x) + 1.5, 0, (2 * y) + 1.5));
            }
        }
        return nm.Tiles;
    }

    /// <summary>A square field with scattered blocks (about 22% of cells), so there are loops and some unreachable pockets.</summary>
    public static TiledNavMesh Field(int size, ulong seed) => Field(size, seed, out _);

    public static TiledNavMesh Field(int size, ulong seed, out bool[] cells)
    {
        cells = new bool[size * size];
        var rng = new NavMeshWorlds.Rng(seed);
        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = rng.Int(100) >= 22;
        }
        return NavMesh.Build2D(cells, size, size, 1.0, new Vector2D(0, 0), FlatSettings()).Tiles;
    }

    public static Vector3D RandomFieldPoint(int size, NavMeshWorlds.Rng r) =>
        new(0.1 + (r.Int((size * 10) - 2) / 10.0), 0, 0.1 + (r.Int((size * 10) - 2) / 10.0));

    public static List<PolyRef> AllPolys(TiledNavMesh m)
    {
        var l = new List<PolyRef>();
        for (int s = 0; s < m.SlotCount; s++)
        {
            NavTile? t = m.GetTileBySlot(s);
            for (int p = 0; t is not null && p < t.PolyCount; p++)
            {
                l.Add(m.GetSlotRef(s).Poly(p));
            }
        }
        return l;
    }

    public static Vector3D PointIn(TiledNavMesh m, PolyRef pr, NavMeshWorlds.Rng r)
    {
        m.TryGetPoly(pr, out NavTile t, out int pi);
        int n = t.PolyVertexCounts[pi];
        int b = t.PolyFirstIndex(pi);
        var w = new double[n];
        double sum = 0;
        for (int i = 0; i < n; i++)
        {
            w[i] = 1 + r.Int(1000);
            sum += w[i];
        }
        double x = 0, y = 0, z = 0;
        for (int i = 0; i < n; i++)
        {
            int v = 3 * t.PolyIndices[b + i];
            x += w[i] / sum * t.Vertices[v];
            y += w[i] / sum * t.Vertices[v + 1];
            z += w[i] / sum * t.Vertices[v + 2];
        }
        return new Vector3D(t.OriginX + x, t.OriginY + y, t.OriginZ + z);
    }

    /// <summary>Ground floor plus a floor four units above it, linked by off-mesh links: what a bridge or a balcony looks like. The three-layer mesh above also has a floor a quarter unit over the ground that overlaps it everywhere, which no walkable world has.</summary>
    public static TiledNavMesh TwoFloors(ulong seed) => NavMeshWorlds.RandomMesh(seed, -1, -1, 2, 2, 2, detail: seed % 2 == 0, offMesh: 3);

    public static TiledNavMesh Layered(ulong seed) => NavMeshWorlds.RandomMesh(seed, -1, -1, 2, 2, 3, detail: seed % 2 == 0, offMesh: 2);

    /// <summary>The corridor's invariants: every polygon valid and connected to the previous, no polygon twice, the position inside the first polygon.</summary>
    public static string? Check(PathCorridor c, NavMeshQuery q)
    {
        if (c.Count == 0)
        {
            return "empty";
        }
        var seen = new HashSet<ulong>();
        foreach (PolyRef p in c.Path)
        {
            if (!seen.Add(p.Value))
            {
                return "polygon twice";
            }
        }
        var probe = new PathCorridor(c.Capacity);
        probe.SetPath(c.Target, c.TargetPoly, c.Path, c.IsComplete);
        if (probe.Validate(q, null, int.MaxValue) != CorridorStatus.Valid)
        {
            return "not connected";
        }
        if (!q.ClosestPointOnPoly(c.FirstPoly, c.Position, out Vector3D cp, out bool over))
        {
            return "first polygon invalid";
        }
        double dx = cp.X - c.Position.X, dz = cp.Z - c.Position.Z;
        if (!over && (dx * dx) + (dz * dz) > 1e-10)
        {
            return $"position outside the first polygon by {Math.Sqrt((dx * dx) + (dz * dz))}";
        }
        if (c.IsComplete && c.LastPoly != c.TargetPoly)
        {
            return "complete but the last polygon is not the target polygon";
        }
        return null;
    }
}
