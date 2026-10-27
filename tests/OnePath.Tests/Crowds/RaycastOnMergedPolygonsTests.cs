using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.HashMatrix;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Raycasts and surface moves on meshes built from tile maps, whose merged polygons have vertices in the middle of a side and
/// meet at vertices. The answer is checked against the tile map itself: a segment is clear exactly when it stays on walkable cells.
/// </summary>
public class RaycastOnMergedPolygonsTests
{
    private static bool Walkable(bool[] cells, int size, double x, double z)
    {
        int cx = (int)Math.Floor(x), cz = (int)Math.Floor(z);
        return cx >= 0 && cz >= 0 && cx < size && cz < size && cells[(cz * size) + cx];
    }

    // Coordinates are multiples of 0.1, so exact integer arithmetic on tenths decides whether the segment passes a cell corner.
    private static bool TouchesLatticePoint(Vector3D a, Vector3D b)
    {
        long ax = (long)Math.Round(a.X * 10), az = (long)Math.Round(a.Z * 10), bx = (long)Math.Round(b.X * 10), bz = (long)Math.Round(b.Z * 10);
        long dx = bx - ax, dz = bz - az;
        for (long i = Math.Min(ax, bx) / 10 * 10; i <= Math.Max(ax, bx) + 10; i += 10)
        {
            for (long j = Math.Min(az, bz) / 10 * 10; j <= Math.Max(az, bz) + 10; j += 10)
            {
                long cross = ((i - ax) * dz) - ((j - az) * dx);
                bool inside = i >= Math.Min(ax, bx) && i <= Math.Max(ax, bx) && j >= Math.Min(az, bz) && j <= Math.Max(az, bz);
                if (cross == 0 && inside)
                {
                    return true;
                }
            }
        }
        return false;
    }

    [Fact]
    public void Raycast_AgreesWithTheTileMap()
    {
        int clear = 0, blocked = 0, disagreements = 0;
        string log = "";
        for (ulong seed = 1; seed <= 6; seed++)
        {
            const int size = 32;
            TiledNavMesh mesh = CorridorWorlds.Field(size, seed, out bool[] cells);
            var q = new NavMeshQuery(mesh);
            var rng = new NavMeshWorlds.Rng(seed * 13);
            var visited = new PolyRef[256];
            for (int i = 0; i < 1500; i++)
            {
                Vector3D a = CorridorWorlds.RandomFieldPoint(size, rng), b = CorridorWorlds.RandomFieldPoint(size, rng);
                if (!Walkable(cells, size, a.X, a.Z))
                {
                    continue;
                }
                NearestPoly np = q.FindNearestPoly(a, new Vector3D(0.01, 4, 0.01));
                if (!np.Poly.IsSet)
                {
                    continue;
                }
                RaycastResult r = q.Raycast(np.Poly, a, b, null, visited, out _);
                Assert.Equal(RaycastStatus.Ok, r.Status);
                // Walk the segment finely; a segment that grazes a cell corner within the step is skipped as ambiguous.
                if (TouchesLatticePoint(a, b) || Math.Abs(b.X - Math.Round(b.X)) < 1e-9 || Math.Abs(b.Z - Math.Round(b.Z)) < 1e-9)
                {
                    continue; // passes through a cell corner or ends on a cell edge: whether that is clear is a convention, not a fact
                }
                bool free = true, graze = false;
                const int steps = 4000;
                for (int s = 1; s <= steps && free; s++)
                {
                    double t = s / (double)steps;
                    double x = a.X + ((b.X - a.X) * t), z = a.Z + ((b.Z - a.Z) * t);
                    free = Walkable(cells, size, x, z);
                    graze |= !free && (Math.Abs(x - Math.Round(x)) < 1e-3 || Math.Abs(z - Math.Round(z)) < 1e-3) && Math.Abs(x - Math.Round(x)) + Math.Abs(z - Math.Round(z)) < 2e-3;
                }
                if (graze)
                {
                    continue;
                }
                if (free == r.Hit)
                {
                    disagreements++;
                    log += $"seed {seed} a {a} b {b} free {free} hit {r.Hit} t {r.T} poly {np.Poly.Value:X}\n";
                }
                clear += free ? 1 : 0;
                blocked += free ? 0 : 1;
            }
        }
        Assert.True(disagreements == 0, log);
        Assert.True(clear > 200 && blocked > 300, $"{clear} clear, {blocked} blocked");
    }

    [Fact]
    public void MoveAlongSurface_ReachesTheEndExactlyWhenTheSegmentIsClear()
    {
        const int size = 32;
        TiledNavMesh mesh = CorridorWorlds.Field(size, 3, out bool[] cells);
        var q = new NavMeshQuery(mesh);
        var rng = new NavMeshWorlds.Rng(4);
        var visited = new PolyRef[256];
        int moved = 0;
        for (int i = 0; i < 2000; i++)
        {
            Vector3D a = CorridorWorlds.RandomFieldPoint(size, rng);
            Vector3D b = new(a.X + ((rng.Int(41) - 20) / 10.0), 0, a.Z + ((rng.Int(41) - 20) / 10.0));
            NearestPoly np = Walkable(cells, size, a.X, a.Z) ? q.FindNearestPoly(a, new Vector3D(0.01, 4, 0.01)) : default;
            if (!np.Poly.IsSet)
            {
                continue;
            }
            MoveResult m = q.MoveAlongSurface(np.Poly, a, b, null, visited, out _);
            Assert.Equal(RaycastStatus.Ok, m.Status);
            if (!m.Blocked)
            {
                Assert.Equal(b.X, m.Position.X);
                Assert.Equal(b.Z, m.Position.Z);
                moved++;
            }
            else
            {
                // The slide ends on a wall edge, so a cell beside it is walkable.
                bool beside = false;
                foreach (double ox in new[] { -1e-9, 1e-9 })
                {
                    foreach (double oz in new[] { -1e-9, 1e-9 })
                    {
                        beside |= Walkable(cells, size, m.Position.X + ox, m.Position.Z + oz);
                    }
                }
                Assert.True(beside);
            }
        }
        Assert.True(moved > 300);
    }
}
