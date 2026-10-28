using System;
using System.Collections.Generic;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;
using Xunit;

namespace RecursiveAI.OnePath.Tests.Grids;

// Independent oracle for the voxel search: moves are enumerated from the definition (every offset in [-1,1]^3 with
// at most `kinds` non-zero components) and the corner rules are evaluated over the move's bounding box, not the way the
// production code does it. Dijkstra is the O(n^2) array form.
internal static class VoxelBruteForce
{
    internal static bool Open(VoxelGrid g, int x, int y, int z) => g.Contains(x, y, z) && g.GetArea(x, y, z) != AreaCosts.Wall;

    internal static bool LegalMove(VoxelGrid g, int kinds, VoxelCornerRule rule, VoxelCell a, VoxelCell b)
    {
        int dx = b.X - a.X;
        int dy = b.Y - a.Y;
        int dz = b.Z - a.Z;
        int k = (dx != 0 ? 1 : 0) + (dy != 0 ? 1 : 0) + (dz != 0 ? 1 : 0);
        if (k == 0 || Math.Abs(dx) > 1 || Math.Abs(dy) > 1 || Math.Abs(dz) > 1 || k > kinds)
        {
            return false;
        }
        if (!Open(g, b.X, b.Y, b.Z) || !Open(g, a.X, a.Y, a.Z))
        {
            return false;
        }
        if (k == 1 || rule == VoxelCornerRule.AllowAll)
        {
            return true;
        }
        if (rule == VoxelCornerRule.NoCornerCutting)
        {
            // Every cell of the move's bounding box must be open.
            for (int x = Math.Min(a.X, b.X); x <= Math.Max(a.X, b.X); x++)
            {
                for (int y = Math.Min(a.Y, b.Y); y <= Math.Max(a.Y, b.Y); y++)
                {
                    for (int z = Math.Min(a.Z, b.Z); z <= Math.Max(a.Z, b.Z); z++)
                    {
                        if (!Open(g, x, y, z))
                        {
                            return false;
                        }
                    }
                }
            }
            return true;
        }
        // NoSqueezing: some monotone route of face steps from a to b through open cells exists.
        return Route(g, a, b);
    }

    private static bool Route(VoxelGrid g, VoxelCell p, VoxelCell b)
    {
        if (p == b)
        {
            return true;
        }
        if (p.X != b.X && Open(g, p.X + Math.Sign(b.X - p.X), p.Y, p.Z) && Route(g, p with { X = p.X + Math.Sign(b.X - p.X) }, b))
        {
            return true;
        }
        if (p.Y != b.Y && Open(g, p.X, p.Y + Math.Sign(b.Y - p.Y), p.Z) && Route(g, p with { Y = p.Y + Math.Sign(b.Y - p.Y) }, b))
        {
            return true;
        }
        return p.Z != b.Z && Open(g, p.X, p.Y, p.Z + Math.Sign(b.Z - p.Z)) && Route(g, p with { Z = p.Z + Math.Sign(b.Z - p.Z) }, b);
    }

    internal static double MoveCost(VoxelGrid g, VoxelCell a, VoxelCell b)
    {
        int k = (a.X != b.X ? 1 : 0) + (a.Y != b.Y ? 1 : 0) + (a.Z != b.Z ? 1 : 0);
        double len = k == 1 ? 1.0 : k == 2 ? Math.Sqrt(2.0) : Math.Sqrt(3.0);
        double ma = g.Areas[g.GetArea(a.X, a.Y, a.Z)];
        double mb = g.Areas[g.GetArea(b.X, b.Y, b.Z)];
        return len * ((ma + mb) * 0.5);
    }

    internal static double[] DistancesFrom(VoxelGrid g, int kinds, VoxelCornerRule rule, VoxelCell start)
    {
        int n = g.VoxelCount;
        var dist = new double[n];
        var done = new bool[n];
        Array.Fill(dist, double.PositiveInfinity);
        dist[g.NodeIndex(start.X, start.Y, start.Z)] = 0.0;
        while (true)
        {
            int u = -1;
            for (int i = 0; i < n; i++)
            {
                if (!done[i] && !double.IsPositiveInfinity(dist[i]) && (u < 0 || dist[i] < dist[u]))
                {
                    u = i;
                }
            }
            if (u < 0)
            {
                return dist;
            }
            done[u] = true;
            VoxelCell a = g.CellOf(u);
            for (int dz = -1; dz <= 1; dz++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        var b = new VoxelCell(a.X + dx, a.Y + dy, a.Z + dz);
                        if (!LegalMove(g, kinds, rule, a, b))
                        {
                            continue;
                        }
                        int v = g.NodeIndex(b.X, b.Y, b.Z);
                        double c = dist[u] + MoveCost(g, a, b);
                        if (c < dist[v])
                        {
                            dist[v] = c;
                        }
                    }
                }
            }
        }
    }

    internal static VoxelGrid Random(ref SeededRandom rng, int sx, int sy, int sz, double density, bool areas)
    {
        var g = new VoxelGrid(sx, sy, sz);
        if (areas)
        {
            g.Areas[1] = 2.5;
            g.Areas[2] = 4.0;
            g.Areas[3] = 1.25;
        }
        for (int z = 0; z < sz; z++)
        {
            for (int y = 0; y < sy; y++)
            {
                for (int x = 0; x < sx; x++)
                {
                    if (rng.NextDouble() < density)
                    {
                        g.SetArea(x, y, z, AreaCosts.Wall);
                    }
                    else if (areas)
                    {
                        g.SetArea(x, y, z, (byte)rng.NextInt(4));
                    }
                }
            }
        }
        return g;
    }

    internal static List<VoxelCell> OpenCells(VoxelGrid g)
    {
        var list = new List<VoxelCell>();
        for (int i = 0; i < g.VoxelCount; i++)
        {
            VoxelCell c = g.CellOf(i);
            if (g.GetArea(c.X, c.Y, c.Z) != AreaCosts.Wall)
            {
                list.Add(c);
            }
        }
        return list;
    }
}

public class VoxelSearchAgainstBruteForce
{
    private const int GridsPerConnectivity = 10_000;

    private static void Run(VoxelConnectivity conn, ulong seed)
    {
        var rng = new SeededRandom(seed);
        int kinds = (int)conn == 6 ? 1 : (int)conn == 18 ? 2 : 3;
        var rules = new[] { VoxelCornerRule.NoCornerCutting, VoxelCornerRule.NoSqueezing, VoxelCornerRule.AllowAll };
        for (int t = 0; t < GridsPerConnectivity; t++)
        {
            int sx = 1 + rng.NextInt(5);
            int sy = 1 + rng.NextInt(5);
            int sz = 1 + rng.NextInt(5);
            double density = rng.NextDouble() * 0.5;
            VoxelGrid g = VoxelBruteForce.Random(ref rng, sx, sy, sz, density, (t & 1) == 1);
            List<VoxelCell> open = VoxelBruteForce.OpenCells(g);
            if (open.Count == 0)
            {
                continue;
            }
            VoxelCornerRule rule = rules[t % 3];
            VoxelCell a = open[rng.NextInt(open.Count)];
            VoxelCell b = open[rng.NextInt(open.Count)];
            double[] dist = VoxelBruteForce.DistancesFrom(g, kinds, rule, a);
            double expect = dist[g.NodeIndex(b.X, b.Y, b.Z)];
            var options = new VoxelOptions { Connectivity = conn, Corners = rule, Heuristic = (t % 5) == 0 ? GridHeuristic.Zero : GridHeuristic.Auto };
            VoxelPath p = g.FindPath(a, b, options);
            if (double.IsPositiveInfinity(expect))
            {
                Assert.Equal(PathStatus.NoPath, p.Status);
                continue;
            }
            Assert.True(p.Found, $"grid {t}: expected a path of cost {expect}");
            Assert.True(Math.Abs(p.Cost - expect) <= 1e-9 * Math.Max(1.0, expect), $"grid {t}: cost {p.Cost} but brute force says {expect}");
            ReadOnlySpan<VoxelCell> cells = p.Cells;
            Assert.Equal(a, cells[0]);
            Assert.Equal(b, cells[^1]);
            double sum = 0;
            for (int i = 1; i < cells.Length; i++)
            {
                Assert.True(VoxelBruteForce.LegalMove(g, kinds, rule, cells[i - 1], cells[i]), $"grid {t}: illegal step");
                sum += VoxelBruteForce.MoveCost(g, cells[i - 1], cells[i]);
            }
            Assert.True(Math.Abs(sum - p.Cost) <= 1e-9 * Math.Max(1.0, sum), $"grid {t}: path sums to {sum}, reported {p.Cost}");
        }
    }

    [Fact]
    public void SixConnectedMatchesDijkstra() => Run(VoxelConnectivity.Six, 0x6001);

    [Fact]
    public void EighteenConnectedMatchesDijkstra() => Run(VoxelConnectivity.Eighteen, 0x1801);

    [Fact]
    public void TwentySixConnectedMatchesDijkstra() => Run(VoxelConnectivity.TwentySix, 0x2601);

    [Fact]
    public void LargerGridsAcrossChunkBordersMatchDijkstra()
    {
        var rng = new SeededRandom(0xC4A);
        foreach (VoxelConnectivity conn in new[] { VoxelConnectivity.Six, VoxelConnectivity.Eighteen, VoxelConnectivity.TwentySix })
        {
            int kinds = (int)conn == 6 ? 1 : (int)conn == 18 ? 2 : 3;
            for (int t = 0; t < 12; t++)
            {
                VoxelGrid g = VoxelBruteForce.Random(ref rng, 17 + rng.NextInt(6), 15 + rng.NextInt(6), 14 + rng.NextInt(6), 0.1 + (rng.NextDouble() * 0.3), t % 2 == 0);
                List<VoxelCell> open = VoxelBruteForce.OpenCells(g);
                VoxelCell a = open[rng.NextInt(open.Count)];
                double[] dist = VoxelBruteForce.DistancesFrom(g, kinds, VoxelCornerRule.NoCornerCutting, a);
                for (int q = 0; q < 4; q++)
                {
                    VoxelCell b = open[rng.NextInt(open.Count)];
                    VoxelPath p = g.FindPath(a, b, new VoxelOptions { Connectivity = conn }, cancellationToken: TestContext.Current.CancellationToken);
                    double expect = dist[g.NodeIndex(b.X, b.Y, b.Z)];
                    if (double.IsPositiveInfinity(expect))
                    {
                        Assert.Equal(PathStatus.NoPath, p.Status);
                    }
                    else
                    {
                        Assert.True(p.Found);
                        Assert.True(Math.Abs(p.Cost - expect) <= 1e-9 * Math.Max(1.0, expect));
                    }
                }
            }
        }
    }

    [Theory]
    [InlineData(VoxelConnectivity.Six, VoxelHeuristicKind.Manhattan)]
    [InlineData(VoxelConnectivity.Eighteen, VoxelHeuristicKind.Octile18)]
    [InlineData(VoxelConnectivity.TwentySix, VoxelHeuristicKind.Octile26)]
    public void HeuristicIsTheExactEmptyGridDistance(VoxelConnectivity conn, VoxelHeuristicKind kind)
    {
        int kinds = (int)conn == 6 ? 1 : (int)conn == 18 ? 2 : 3;
        var g = new VoxelGrid(7, 6, 8);
        var h = new VoxelHeuristic(7, 6, kind);
        foreach (var goal in new[] { new VoxelCell(0, 0, 0), new VoxelCell(3, 2, 5), new VoxelCell(6, 5, 7) })
        {
            double[] dist = VoxelBruteForce.DistancesFrom(g, kinds, VoxelCornerRule.NoCornerCutting, goal);
            h.SetGoal(g.NodeIndex(goal.X, goal.Y, goal.Z));
            for (int i = 0; i < g.VoxelCount; i++)
            {
                Assert.True(Math.Abs(h.Estimate(i) - dist[i]) <= 1e-9, $"node {i}: estimate {h.Estimate(i)} vs distance {dist[i]}");
            }
        }
    }
}
