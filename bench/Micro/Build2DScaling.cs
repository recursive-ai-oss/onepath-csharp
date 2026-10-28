using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// Cold, single-shot scaling run of the 2D geometry stages: <c>OnePath.Micro --scale2d [sizes...]</c> prints, per size,
/// the time of each stage on a noisy tile map (about 60% solid) and on random overlapping quadrilaterals.
/// A development signal for the growth rate, not a bar number.
/// </summary>
internal static class Build2DScaling
{
    public static void Run(string[] sizes)
    {
        int[] ns = sizes.Length > 0 ? Array.ConvertAll(sizes, s => int.Parse(s, CultureInfo.InvariantCulture)) : new[] { 32, 64, 128 };
        Console.WriteLine("case\tsize\tstage\tms\tout");
        foreach (int n in ns)
        {
            var cells = new bool[n * n];
            var rng = new Random(42);
            for (int i = 0; i < cells.Length; i++)
            {
                cells[i] = rng.Next(100) < 60;
            }
            Region map = Time("tilemap", n, "FromTileMap", () => Region.FromTileMap(cells, n, n, 1.0, new Vector2D(0, 0)));
            Time("tilemap", n, "OffsetShrinkRound", () => map.Offset(-0.4));
            Time("tilemap", n, "ShrinkOneSided", () => Mesh2DPipeline.Shrink(map, 0.4));
            Time("tilemap", n, "OffsetGrowSquare", () => map.Offset(0.4, JoinStyle.Square));
            Time("tilemap", n, "Triangulate", () => map.Triangulate());
            var settings = NavSettings.Humanoid with { AgentRadius = 0.5f, CellSize = 0.25, TileSize = 32 };
            Region pipeMap = Region.FromTileMap(cells, n, n, 1.0, new Vector2D(0, 0), Mesh2DPipeline.GeometryOptions(settings));
            Region shrunk = Time("tilemap", n, "  stage Shrink", () => Mesh2DPipeline.Shrink(pipeMap, settings.AgentRadius));
            NavMeshParams np = settings.ToParams();
            Time("tilemap", n, "  shrunk.Triangulate", () => shrunk.Triangulate());
            Time("tilemap", n, "  shrunk.PartitionConvex", () => { shrunk.PartitionConvex(); return shrunk; });
            Time("tilemap", n, "  area exact/onesided", () => { Console.Error.WriteLine(map.Offset(-0.4).Area + " " + Mesh2DPipeline.Shrink(map, 0.4).Area); return map; });
            IReadOnlyList<TilePartition> parts = Array.Empty<TilePartition>();
            Time("tilemap", n, "  stage Slice", () => { parts = TileSlicer.Slice(shrunk, np); return parts; });
            var datas = new List<NavTileData>();
            Time("tilemap", n, "  stage ToTileData", () => { datas.Clear(); foreach (TilePartition t in parts) { datas.Add(Mesh2DPipeline.ToTileData(t, np, 0, 0, 1)); } return datas; });
            Time("tilemap", n, "  stage Assemble", () => Mesh2DPipeline.Assemble(np, datas));
            Time("tilemap", n, "NavMesh.Build2D", () => NavMesh.Build2D(cells, n, n, 1.0, new Vector2D(0, 0), settings));
        }
        foreach (int n in ns)
        {
            var rng = new Random(7);
            int count = n * 4;
            double side = Math.Sqrt(count) * 6;
            var outlines = new List<Outline>();
            for (int i = 0; i < count; i++)
            {
                double cx = rng.NextDouble() * side, cy = rng.NextDouble() * side;
                var pts = new Vector2D[4];
                for (int k = 0; k < 4; k++)
                {
                    pts[k] = new Vector2D(cx + ((rng.NextDouble() - 0.5) * 12), cy + ((rng.NextDouble() - 0.5) * 12));
                }
                outlines.Add(new Outline(pts));
            }
            Region u = Time("outlines", count, "FromOutlines", () => Region.FromOutlines(outlines));
            Time("outlines", count, "OffsetShrinkRound", () => u.Offset(-0.4));
        }
    }

    /// <summary>
    /// <c>--check-tilemap file.obj [size] [seed]</c>: the tiles recovered from a generated level equal the ones the
    /// benchmarks generate directly.
    /// </summary>
    public static int CheckTileMap(string obj, double size, int seed)
    {
        bool[] fromFile = RecursiveAI.OnePath.Bench.TileMapWorld.FromObj(obj, out int tiles, out _);
        bool[] generated = RecursiveAI.OnePath.Bench.TileMapWorld.Generate(size, seed, out int n);
        bool same = tiles == n && fromFile.AsSpan().SequenceEqual(generated);
        Console.WriteLine(same ? "tile maps equal" : "tile maps DIFFER");
        return same ? 0 : 1;
    }

    /// <summary>
    /// <c>--loop2d [size] [reps]</c>: builds the noisy tile map into a navmesh repeatedly (for a sampling profiler).
    /// </summary>
    public static void Loop(int n, int reps)
    {
        var cells = new bool[n * n];
        var rng = new Random(42);
        for (int i = 0; i < cells.Length; i++)
        {
            cells[i] = rng.Next(100) < 60;
        }
        var settings = NavSettings.Humanoid with { AgentRadius = 0.5f, CellSize = 0.25, TileSize = 32 };
        for (int k = 0; k < reps; k++)
        {
            _ = NavMesh.Build2D(cells, n, n, 1.0, new Vector2D(0, 0), settings);
        }
    }

    /// <summary>
    /// <c>--digest2d [cases]</c>: SHA-256 over the exact rings of booleans, offsets and tile maps for a fixed family of
    /// pseudo-random inputs. Two builds of the library that print the same digest produce identical regions.
    /// </summary>
    public static void Digest(int cases)
    {
        using var sha = System.Security.Cryptography.IncrementalHash.CreateHash(System.Security.Cryptography.HashAlgorithmName.SHA256);
        var buf = new byte[8];
        void Add(long v)
        {
            System.Buffers.Binary.BinaryPrimitives.WriteInt64LittleEndian(buf, v);
            sha.AppendData(buf);
        }
        void AddRegion(Region r)
        {
            Add(r.RingCount);
            for (int i = 0; i < r.RingCount; i++)
            {
                Vector2D[] ring = r.GetRing(i);
                Add(ring.Length);
                foreach (Vector2D v in ring)
                {
                    Add(BitConverter.DoubleToInt64Bits(v.X));
                    Add(BitConverter.DoubleToInt64Bits(v.Y));
                }
            }
        }
        for (int c = 0; c < cases; c++)
        {
            var rng = new Random(1000 + c);
            int denom = new[] { 1, 4, 64, 1024 }[rng.Next(4)];
            int extent = denom >= 64 ? 2 : 10 + (c % 20);
            if (c % 5 == 4)
            {
                // Far from the origin and long: exercises the wide-coordinate paths of the bucket grids.
                denom = 1;
                extent = 1_000_000 + (c * 1000);
            }
            List<Outline> Make(int count)
            {
                var l = new List<Outline>();
                for (int o = 0; o < count; o++)
                {
                    var pts = new Vector2D[3 + rng.Next(6)];
                    for (int i = 0; i < pts.Length; i++)
                    {
                        double x = rng.Next((extent * denom) + 1) / (double)denom, y = rng.Next((extent * denom) + 1) / (double)denom;
                        if (c % 3 == 0 && rng.Next(4) == 0)
                        {
                            x = Math.BitIncrement(x);
                        }
                        pts[i] = new Vector2D(x, y);
                    }
                    l.Add(new Outline(pts));
                }
                return l;
            }
            Region a = Region.FromOutlines(Make(1 + rng.Next(8)));
            Region b = Region.FromOutlines(Make(1 + rng.Next(8)));
            AddRegion(a);
            AddRegion(a.Union(b));
            AddRegion(a.Intersection(b));
            AddRegion(a.Difference(b));
            AddRegion(a.SymmetricDifference(b));
            double d = (1 + rng.Next(8)) * (denom >= 64 ? 0.03125 : 0.25);
            AddRegion(a.Offset(d));
            AddRegion(a.Offset(-d));
            AddRegion(a.Offset(d, JoinStyle.Square));
            AddRegion(a.Offset(-d, JoinStyle.Square));
            if (c % 4 == 0)
            {
                int n = 6 + rng.Next(14);
                var cells = new bool[n * n];
                for (int i = 0; i < cells.Length; i++)
                {
                    cells[i] = rng.Next(100) < 55;
                }
                Region m = Region.FromTileMap(cells, n, n, 1.0, new Vector2D(0, 0));
                AddRegion(m);
                AddRegion(m.Offset(-0.4));
                AddRegion(m.Offset(0.3, JoinStyle.Square));
                AddRegion(m.Offset(-0.5, JoinStyle.Square));
            }
        }
        Console.WriteLine(Convert.ToHexString(sha.GetHashAndReset()).ToLowerInvariant());
    }

    private static string Polys(NavMesh nm)
    {
        int polys = 0;
        for (int i = 0; i < nm.Tiles.SlotCount; i++)
        {
            polys += nm.Tiles.GetTileBySlot(i)?.PolyCount ?? 0;
        }
        return polys.ToString(CultureInfo.InvariantCulture);
    }

    private static Region Time(string name, int size, string stage, Func<object> f)
    {
        // Cold first run, then the best of two more: the second figure is the warm cost.
        var sw = Stopwatch.StartNew();
        object r = f();
        double cold = sw.Elapsed.TotalMilliseconds, best = double.MaxValue;
        for (int k = 0; k < 2; k++)
        {
            sw.Restart();
            r = f();
            best = Math.Min(best, sw.Elapsed.TotalMilliseconds);
        }
        string o = r is Region g ? g.RingCount + " rings" : r is Triangulation t ? t.Indices.Length / 3 + " tris" : r is NavMesh nm ? Polys(nm) + " polys" : "";
        Console.WriteLine(string.Join('\t', name, size.ToString(CultureInfo.InvariantCulture), stage, cold.ToString("F1", CultureInfo.InvariantCulture) + " cold", best.ToString("F1", CultureInfo.InvariantCulture) + " warm", o));
        return r as Region ?? Region.Empty();
    }
}
