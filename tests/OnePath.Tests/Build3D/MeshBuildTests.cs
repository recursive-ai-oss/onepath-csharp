using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>Small level geometry for the end-to-end builder tests.</summary>
internal sealed class Soup
{
    public List<double> V { get; } = new();

    public List<int> I { get; } = new();

    private int Vert(double x, double y, double z)
    {
        V.Add(x);
        V.Add(y);
        V.Add(z);
        return (V.Count / 3) - 1;
    }

    private void Tri(int a, int b, int c)
    {
        I.Add(a);
        I.Add(b);
        I.Add(c);
    }

    /// <summary>An up-facing horizontal rectangle.</summary>
    public Soup Floor(double x0, double z0, double x1, double z1, double y)
    {
        int a = Vert(x0, y, z0), b = Vert(x1, y, z0), c = Vert(x1, y, z1), d = Vert(x0, y, z1);
        Tri(a, d, c);
        Tri(a, c, b);
        return this;
    }

    /// <summary>A closed box with outward faces.</summary>
    public Soup Box(double x0, double y0, double z0, double x1, double y1, double z1)
    {
        int[] p = new int[8];
        for (int i = 0; i < 8; i++)
        {
            p[i] = Vert((i & 1) == 0 ? x0 : x1, (i & 4) == 0 ? y0 : y1, (i & 2) == 0 ? z0 : z1);
        }
        void Quad(int a, int b, int c, int d)
        {
            Tri(a, b, c);
            Tri(a, c, d);
        }
        Quad(p[4], p[6], p[7], p[5]);
        Quad(p[0], p[1], p[3], p[2]);
        Quad(p[0], p[4], p[5], p[1]);
        Quad(p[2], p[3], p[7], p[6]);
        Quad(p[0], p[2], p[6], p[4]);
        Quad(p[1], p[5], p[7], p[3]);
        return this;
    }

    /// <summary>A slab rising along +X from y0 at x0 to y1 at x1.</summary>
    public Soup Ramp(double x0, double z0, double x1, double z1, double y0, double y1)
    {
        int a = Vert(x0, y0, z0), b = Vert(x1, y1, z0), c = Vert(x1, y1, z1), d = Vert(x0, y0, z1);
        Tri(a, d, c);
        Tri(a, c, b);
        return this;
    }

    public NavMesh Build(NavSettings? s = null) => NavMesh.Build(V.ToArray(), I.ToArray(), s);
}

public class MeshBuildTests
{
    private static NavSettings Fine => new() { CellSize = 0.25, CellHeight = 0.1, TileSize = 16, AgentRadius = 0.5f, AgentHeight = 1.8f, WalkableClimb = 0.4f, DetailSampleDistance = 1, DetailMaxError = 0.1 };

    internal static double Area(TiledNavMesh m)
    {
        double sum = 0;
        for (int slot = 0; slot < m.SlotCount; slot++)
        {
            NavTile? t = m.GetTileBySlot(slot);
            if (t is null)
            {
                continue;
            }
            ReadOnlySpan<float> v = t.Vertices;
            int pos = 0;
            foreach (byte n in t.PolyVertexCounts)
            {
                double a2 = 0;
                for (int e = 0; e < n; e++)
                {
                    int i0 = t.PolyIndices[pos + e], i1 = t.PolyIndices[pos + ((e + 1) % n)];
                    a2 += ((double)v[3 * i0] * v[(3 * i1) + 2]) - ((double)v[3 * i1] * v[(3 * i0) + 2]);
                }
                sum += a2 / 2;
                pos += n;
            }
        }
        return sum;
    }

    [Fact]
    public void FlatFloorCoversItsAreaMinusTheRadiusMargin()
    {
        NavMesh m = new Soup().Floor(0, 0, 40, 30, 0).Build(Fine);
        double area = Area(m.Tiles);
        // The walkable area is the floor shrunk by the radius (0.5 -> two 0.25 cells), about (39)(29) minus corner effects.
        Assert.InRange(area, 36.0 * 26.0, 40.0 * 30.0);
        PathResult r = m.FindPath(new Vector3D(2, 0, 2), new Vector3D(38, 0, 28), cancellationToken: TestContext.Current.CancellationToken);
        Assert.True(r.Status == PathStatus.Found, r.Status.ToString());
    }

    private static void AssertValid(NavMesh m)
    {
        Assert.Empty(NavMeshValidator.Validate(m.Tiles));
    }

    private static double PathLength(PathResult r)
    {
        double len = 0;
        for (int i = 1; i < r.Points.Count; i++)
        {
            Vector3D a = r.Points[i - 1].Position, b = r.Points[i].Position;
            len += Math.Sqrt(((a.X - b.X) * (a.X - b.X)) + ((a.Z - b.Z) * (a.Z - b.Z)));
        }
        return len;
    }

    [Fact]
    public void BoxObstacleIsWalkedAround()
    {
        NavMesh m = new Soup().Floor(0, 0, 40, 30, 0).Box(15, 0, 10, 25, 3, 20).Build(Fine);
        AssertValid(m);
        PathResult r = m.FindPath(new Vector3D(5, 0, 15), new Vector3D(35, 0, 15), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, r.Status);
        Assert.InRange(PathLength(r), 31.0, 45.0);
        // The box top is a walkable island the agent cannot step onto: no path reaches it.
        PathResult top = m.FindPath(new Vector3D(5, 0, 15), new Vector3D(20, 3, 15), cancellationToken: TestContext.Current.CancellationToken);
        Assert.NotEqual(PathStatus.Found, top.Status);
    }

    [Fact]
    public void RampHeightsFollowTheSurfaceWithinTheDetailError()
    {
        // 0 to 3 over 30 units (about 5.7 degrees), within the climb limit per cell.
        NavMesh m = new Soup().Ramp(0, 0, 30, 12, 0, 3).Build(Fine);
        AssertValid(m);
        int checkedPoints = 0;
        var q = new NavMeshQuery(m.Tiles);
        for (double x = 3; x < 27; x += 1.7)
        {
            for (double z = 3; z < 9; z += 1.9)
            {
                NearestPoly np = q.FindNearestPoly(new Vector3D(x, x * 0.1, z), new Vector3D(1, 4, 1), null);
                Assert.True(np.Poly.IsSet, $"no polygon near ({x}, {z})");
                Assert.InRange(Math.Abs(np.Point.Y - (x * 0.1)), 0, 0.2);
                checkedPoints++;
            }
        }
        Assert.True(checkedPoints > 20);
        PathResult r = m.FindPath(new Vector3D(2, 0.2, 6), new Vector3D(28, 2.8, 6), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, r.Status);
    }

    [Fact]
    public void TilesStitchAndTheBuildIsDeterministic()
    {
        var soup = new Soup().Floor(-10, -10, 50, 40, 0).Box(18, 0, 5, 22, 3, 20);
        NavMesh a = soup.Build(Fine);
        NavMesh b = soup.Build(Fine);
        Assert.True(a.Tiles.TileCount >= 12);
        AssertValid(a);
        Assert.Equal(NavMeshSerializer.WriteMesh(a.Tiles), NavMeshSerializer.WriteMesh(b.Tiles));
        PathResult r = a.FindPath(new Vector3D(-8, 0, -8), new Vector3D(48, 0, 38), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, r.Status);
        Assert.InRange(PathLength(r), 72.0, 90.0);
    }

    [Fact]
    public void StairsWithinTheClimbLimitAreWalkable()
    {
        var soup = new Soup();
        for (int i = 0; i < 8; i++)
        {
            soup.Box(i * 1.0, -1, 0, (i + 1) * 1.0, 0.3 * (i + 1), 6);
        }
        soup.Floor(-6, 0, 0, 6, 0);
        soup.Floor(8, 0, 14, 6, 2.4);
        NavMesh m = soup.Build(Fine);
        AssertValid(m);
        PathResult r = m.FindPath(new Vector3D(-4, 0, 3), new Vector3D(11, 2.4, 3), cancellationToken: TestContext.Current.CancellationToken);
        Assert.Equal(PathStatus.Found, r.Status);
    }

    [Fact]
    public void LayersSeparateABridgeFromTheGroundBelow()
    {
        var soup = new Soup().Floor(0, 0, 30, 30, 0).Floor(8, 12, 22, 18, 4);
        NavMesh m = soup.Build(Fine with { Partition = PartitionType.Layers, AgentHeight = 1.8f });
        AssertValid(m);
        int maxLayer = 0;
        for (int slot = 0; slot < m.Tiles.SlotCount; slot++)
        {
            NavTile? t = m.Tiles.GetTileBySlot(slot);
            if (t is not null)
            {
                maxLayer = Math.Max(maxLayer, t.Layer);
            }
        }
        Assert.Equal(1, maxLayer);
        var q = new NavMeshQuery(m.Tiles);
        NearestPoly low = q.FindNearestPoly(new Vector3D(15, 0, 15), new Vector3D(1, 1, 1), null);
        NearestPoly high = q.FindNearestPoly(new Vector3D(15, 4, 15), new Vector3D(1, 1, 1), null);
        Assert.True(low.Poly.IsSet && high.Poly.IsSet);
        Assert.NotEqual(low.Poly.Value, high.Poly.Value);
    }

    [Fact]
    public void VoxelChunkAndHeightfieldInputsBuild()
    {
        var s = Fine with { CellSize = 0.25, TileSize = 16 };
        var voxels = new byte[64 * 64 * 4];
        for (int z = 0; z < 64; z++)
        {
            for (int x = 0; x < 64; x++)
            {
                for (int y = 0; y < 2; y++)
                {
                    voxels[(((z * 64) + x) * 4) + y] = 1;
                }
            }
        }
        NavMesh m = NavMesh.Build(VoxelChunk.FromDense(64, 64, 4, voxels), s);
        AssertValid(m);
        Assert.InRange(Area(m.Tiles), 13.0 * 13.0, 16.0 * 16.0);
        Assert.Equal(PathStatus.Found, m.FindPath(new Vector3D(2, 0.2, 2), new Vector3D(14, 0.2, 14), cancellationToken: TestContext.Current.CancellationToken).Status);

        var hf = new SolidHeightfield(new HeightfieldFrame(0, 0, 0, 0.25f, 0.1f, 64, 64, 30));
        for (int c = 0; c < 64 * 64; c++)
        {
            hf.AddSpan(c, 0, 3, SpanSettings.WalkableArea, 0);
        }
        NavMesh h = NavMesh.Build(hf, s);
        AssertValid(h);
        Assert.InRange(Area(h.Tiles), 13.0 * 13.0, 16.0 * 16.0);
    }

    [Fact]
    public void HostileGeometryGetsTypedErrors()
    {
        Assert.Throws<ArgumentException>(() => NavMesh.Build(new[] { 0, 0, 0, double.NaN, 0, 0, 1, 0, 1 }, new[] { 0, 1, 2 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build(new[] { 0, 0, 0, 3e6, 0, 0, 1, 0, 1 }, new[] { 0, 1, 2 }));
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build(new double[] { 0, 0, 0, 1, 0, 0, 1, 0, 1 }, new[] { 0, 1, 5 }));
        Assert.Throws<ArgumentException>(() => NavMesh.Build(new double[] { 0, 0, 0, 1, 0 }, new[] { 0, 1, 2 }));
        Assert.Equal(0, NavMesh.Build(ReadOnlySpan<double>.Empty, ReadOnlySpan<int>.Empty).Tiles.TileCount);
        // Degenerate and zero-area triangles, a sliver, and a floor too small for the agent produce an empty or tiny but valid mesh.
        var dv = new double[] { 0, 0, 0, 1, 0, 0, 2, 0, 0, 0, 0, 0, 0.001, 0, 5, 0.002, 0, 0 };
        NavMesh d = NavMesh.Build(dv, new[] { 0, 1, 2, 3, 4, 5 });
        AssertValid(d);
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build(new double[] { 0, 0, 0, 1, 0, 0, 1, 0, 1 }, new[] { 0, 1, 2 }, Fine with { CellHeight = 0 }));
    }

    private static Soup Terrain(int size)
    {
        var soup = new Soup();
        for (int z = 0; z < size; z++)
        {
            for (int x = 0; x < size; x++)
            {
                double Y(double px, double pz) => (0.02 * px) + (0.004 * (px - 20) * (px - 20) * 0.5) + (0.01 * pz);
                int a = soup.V.Count / 3;
                foreach ((double px, double pz) in new[] { (x, z), (x + 1.0, z), (x + 1.0, z + 1.0), (x, z + 1.0) })
                {
                    soup.V.Add(px);
                    soup.V.Add(Y(px, pz));
                    soup.V.Add(pz);
                }
                soup.I.AddRange(new[] { a, a + 3, a + 2, a, a + 2, a + 1 });
            }
        }
        return soup;
    }

    [Fact]
    public void DetailMeshesTileEveryPolygonAndStayWithinTheErrorOfTheSurface()
    {
        NavSettings s = Fine with { DetailSampleDistance = 1, DetailMaxError = 0.1, TileSize = 16 };
        NavMesh m = Terrain(40).Build(s);
        AssertValid(m);
        int polys = 0;
        for (int slot = 0; slot < m.Tiles.SlotCount; slot++)
        {
            NavTile t = m.Tiles.GetTileBySlot(slot)!;
            ReadOnlySpan<float> v = t.Vertices;
            ReadOnlySpan<float> dv = t.DetailVertices;
            ReadOnlySpan<byte> dt = t.DetailTriangles;
            int pos = 0;
            for (int p = 0; p < t.PolyVertexCounts.Length; p++)
            {
                int n = t.PolyVertexCounts[p];
                DetailMesh d = t.DetailMeshes[p];
                Assert.True(d.TriangleCount >= n - 2, $"tri count {d.TriangleCount} for {n}");
                double polyArea = 0, triArea = 0;
                for (int e = 0; e < n; e++)
                {
                    int i0 = t.PolyIndices[pos + e], i1 = t.PolyIndices[pos + ((e + 1) % n)];
                    polyArea += (((double)v[3 * i0] * v[(3 * i1) + 2]) - ((double)v[3 * i1] * v[(3 * i0) + 2])) / 2;
                }
                for (int k = 0; k < d.TriangleCount; k++)
                {
                    double[] x = new double[3], z = new double[3];
                    for (int c = 0; c < 3; c++)
                    {
                        int ix = dt[(4 * ((int)d.TriangleBase + k)) + c];
                        Assert.True(ix < n + d.VertexCount, "index");
                        if (ix < n)
                        {
                            int g = t.PolyIndices[pos + ix];
                            x[c] = v[3 * g];
                            z[c] = v[(3 * g) + 2];
                        }
                        else
                        {
                            int g = (int)d.VertexBase + (ix - n);
                            x[c] = dv[3 * g];
                            z[c] = dv[(3 * g) + 2];
                        }
                    }
                    double a2 = ((x[1] - x[0]) * (z[2] - z[0])) - ((z[1] - z[0]) * (x[2] - x[0]));
                    Assert.True(a2 > 0, $"detail triangle {k} of polygon {p} is not counter-clockwise");
                    triArea += a2 / 2;
                }
                Assert.Equal(polyArea, triArea, 3);
                pos += n;
                polys++;
            }
        }
        Assert.True(polys >= 9, $"polys {polys}");
        // Heights follow the surface: 0.1 detail error, plus one row of height quantization and the in-cell slope.
        var q = new NavMeshQuery(m.Tiles);
        int over = 0;
        for (double x = 3; x < 37; x += 1.3)
        {
            for (double z = 3; z < 37; z += 1.7)
            {
                double truth = (0.02 * x) + (0.004 * (x - 20) * (x - 20) * 0.5) + (0.01 * z);
                NearestPoly np = q.FindNearestPoly(new Vector3D(x, truth, z), new Vector3D(0.3, 3, 0.3), null);
                if (np.Poly.IsSet && np.OverPolygon)
                {
                    Assert.InRange(Math.Abs(np.Point.Y - truth), 0, 0.1 + 0.1 + 0.05);
                    over++;
                }
            }
        }
        Assert.True(over > 100, $"over {over}");
    }

    [Fact]
    public void TilesAreLinkedAcrossTheirBoundaries()
    {
        NavMesh m = new Soup().Floor(0, 0, 48, 48, 0).Build(Fine);
        int portals = 0, linked = 0;
        for (int slot = 0; slot < m.Tiles.SlotCount; slot++)
        {
            NavTile t = m.Tiles.GetTileBySlot(slot)!;
            foreach (byte side in t.EdgePortalSides)
            {
                if (side != 255)
                {
                    portals++;
                }
            }
        }
        var q = new NavMeshQuery(m.Tiles);
        for (double x = 1.5; x < 47; x += 3)
        {
            PathResult r = m.FindPath(new Vector3D(x, 0, 2), new Vector3D(47 - x, 0, 46), cancellationToken: TestContext.Current.CancellationToken);
            if (r.Status == PathStatus.Found)
            {
                linked++;
            }
        }
        Assert.True(portals > 8);
        Assert.Equal(16, linked);
    }
}
