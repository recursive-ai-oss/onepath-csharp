using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;
using Xunit;

namespace RecursiveAI.OnePath.Tests;

public class Additive11Tests
{
    // Two floors-with-a-wall layout: a 20x20 square with a wall slab in the middle, as a triangle soup.
    private static (float[] V, int[] I) Soup()
    {
        float[] v = [0, 0, 0, 20, 0, 0, 20, 0, 20, 0, 0, 20];
        int[] i = [0, 2, 1, 0, 3, 2];
        return (v, i);
    }

    [Fact]
    public void FloatBuild_IsByteEqualToDoubleBuild()
    {
        (float[] v, int[] ix) = Soup();
        var d = new double[v.Length];
        for (int k = 0; k < v.Length; k++)
        {
            d[k] = v[k];
        }
        NavMesh a = NavMesh.Build(v, ix);
        NavMesh b = NavMesh.Build(d, ix);
        Assert.Equal(NavMeshSerializer.WriteMesh(a.Tiles), NavMeshSerializer.WriteMesh(b.Tiles));
    }

    [Fact]
    public void FloatBuild_ValidatesLikeTheDoubleForm()
    {
        Assert.Throws<ArgumentException>(() => NavMesh.Build(new float[4], new int[3]));
        Assert.Throws<ArgumentException>(() => NavMesh.Build(new float[9], new int[2]));
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build(new float[9], [0, 1, 3]));
        Assert.Throws<ArgumentOutOfRangeException>(() => NavMesh.Build([0f, 0f, 0f, 1e7f, 0f, 0f, 0f, 0f, 1f], [0, 1, 2]));
        Assert.Throws<ArgumentException>(() => NavMesh.Build([0f, 0f, 0f, float.NaN, 0f, 0f, 0f, 0f, 1f], [0, 1, 2]));
    }

    [Fact]
    public void DefaultQueryFilter_MatchesNewQueryFilter_AndDoesNotAllocate()
    {
        var open = new bool[30 * 30];
        Array.Fill(open, true);
        for (int y = 5; y < 25; y++)
        {
            open[(y * 30) + 15] = false;
        }
        NavMesh mesh = NavMesh.Build2D(open, 30, 30, 1.0, new Vector2D(0, 0));
        var plain = new NavMeshQuery(mesh.Tiles, 65536, PathNodeModel.Polygon);
        var gen = new NavMeshQuery(mesh.Tiles, 65536, PathNodeModel.Polygon);
        var ext = new Vector3D(1, 4, 1);
        var filter = new DefaultQueryFilter();
        var qf = new QueryFilter();
        var pa = new PolyRef[1024];
        var pb = new PolyRef[1024];
        var rng = new SeededRandom(7);
        int compared = 0;
        for (int t = 0; t < 60; t++)
        {
            var a = new Vector3D(1 + (rng.NextInt(280) / 10.0), 0, 1 + (rng.NextInt(280) / 10.0));
            var b = new Vector3D(1 + (rng.NextInt(280) / 10.0), 0, 1 + (rng.NextInt(280) / 10.0));
            NearestPoly na = plain.FindNearestPoly(a, ext);
            NearestPoly nb = plain.FindNearestPoly(b, ext);
            if (!na.Poly.IsSet || !nb.Poly.IsSet)
            {
                continue;
            }
            QueryStatus s1 = plain.BeginPath(na.Poly, nb.Poly, na.Point, nb.Point, qf);
            QueryStatus s2;
            int c1 = 0, c2 = 0;
            double cost1 = 0, cost2 = 0;
            if (s1 == QueryStatus.InProgress)
            {
                plain.UpdatePath(int.MaxValue);
                s1 = plain.FinishPath(pa, out c1, out _, out cost1);
            }
            s2 = gen.BeginPath(na.Poly, nb.Poly, na.Point, nb.Point, in filter);
            if (s2 == QueryStatus.InProgress)
            {
                gen.UpdatePath(int.MaxValue, in filter);
                s2 = gen.FinishPath(pb, out c2, out _, out cost2);
            }
            Assert.Equal(s1, s2);
            Assert.Equal(c1, c2);
            Assert.Equal(BitConverter.DoubleToInt64Bits(cost1), BitConverter.DoubleToInt64Bits(cost2));
            Assert.True(pa.AsSpan(0, c1).SequenceEqual(pb.AsSpan(0, c2)));
            compared++;
        }
        Assert.True(compared > 40);

        NearestPoly p0 = gen.FindNearestPoly(new Vector3D(3, 0, 3), ext);
        NearestPoly p1 = gen.FindNearestPoly(new Vector3D(27, 0, 27), ext);
        long bytes = AllocationProbe.SteadyState(() =>
        {
            QueryStatus s = gen.FindPath(p0.Poly, p1.Poly, p0.Point, p1.Point, in filter, pb, out int n);
            return s == QueryStatus.Success && n > 0;
        });
        Assert.Equal(0, bytes);
    }

    [Fact]
    public void ValidateMessagesNameThePropertyAndTheRange()
    {
        var init = Assert.Throws<ArgumentOutOfRangeException>(() => _ = Agent.Humanoid with { SeparationWeight = 2 });
        Assert.Equal("SeparationWeight", init.ParamName);
        Assert.Contains("0 to 1", init.Message);
        Assert.Equal("ChunkSize", Assert.Throws<ArgumentOutOfRangeException>(() => new RecursiveAI.OnePath.Grids.HpaOptions { ChunkSize = 1 }).ParamName);
        Assert.Equal("AgentSize", Assert.Throws<ArgumentOutOfRangeException>(() => new RecursiveAI.OnePath.Grids.HpaOptions { AgentSize = 300 }).ParamName);
    }

    [Fact]
    public void FindPathWithPoints_AddsPointsAndChangesNothingElse()
    {
        Grid g = Grid.FromText("""
            ........
            .#####..
            .#...#..
            .#.#.#..
            ...#....
            """);
        g.Space = new GridSpace(10, 20, 2);
        GridPath plain = g.FindPath(new Cell(0, 0), new Cell(4, 3), null, TestContext.Current.CancellationToken);
        GridPath with = g.FindPathWithPoints(new Cell(0, 0), new Cell(4, 3), null, TestContext.Current.CancellationToken);
        Assert.Equal(plain.Status, with.Status);
        Assert.Equal(plain.Cost, with.Cost);
        Assert.Equal(plain.Cells.ToArray(), with.Cells.ToArray());
        Assert.Equal(0, plain.Points.Length);
        Assert.Equal(with.Cells.Length, with.Points.Length);
        WorldPoint first = with.Points[0];
        Assert.Equal(11.0, first.X);
        Assert.Equal(21.0, first.Y);
        Assert.False(double.IsNaN(with.Length));
        Assert.Equal(PathStatus.StartInvalid, GridPathStatusProbe(g));
    }

    private static PathStatus GridPathStatusProbe(Grid g) => g.FindPathWithPoints(new Cell(-1, 0), new Cell(0, 0), null, TestContext.Current.CancellationToken).Status;
}
