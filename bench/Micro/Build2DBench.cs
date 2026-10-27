using System;
using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh.Build2D;

namespace RecursiveAI.OnePath.Bench;

/// <summary>
/// Rows for the 2D geometry stages on a 128 x 128 noisy tile map (about 60% solid) and on 200 overlapping random
/// quadrilaterals. Inputs are generated with a fixed seed.
/// </summary>
[MemoryDiagnoser]
public class Build2DBench
{
    private bool[] _cells = Array.Empty<bool>();
    private List<Outline> _outlines = new();
    private Region _map = Region.Empty();
    private Region _other = Region.Empty();

    [GlobalSetup]
    public void Setup()
    {
        var rng = new Random(42);
        _cells = new bool[128 * 128];
        for (int i = 0; i < _cells.Length; i++)
        {
            _cells[i] = rng.Next(100) < 60;
        }
        _outlines = new List<Outline>();
        for (int i = 0; i < 200; i++)
        {
            double cx = rng.NextDouble() * 100, cy = rng.NextDouble() * 100;
            var pts = new Vector2D[4];
            for (int k = 0; k < 4; k++)
            {
                pts[k] = new Vector2D(cx + ((rng.NextDouble() - 0.5) * 12), cy + ((rng.NextDouble() - 0.5) * 12));
            }
            _outlines.Add(new Outline(pts));
        }
        _map = Region.FromTileMap(_cells, 128, 128, 1.0, new Vector2D(0, 0));
        _other = Region.FromOutlines(_outlines);
    }

    [Benchmark]
    public Region TileMapToRegion() => Region.FromTileMap(_cells, 128, 128, 1.0, new Vector2D(0, 0));

    [Benchmark]
    public Region UnionOfOutlines() => Region.FromOutlines(_outlines);

    [Benchmark]
    public Region Difference() => _map.Difference(_other);

    [Benchmark]
    public Region OffsetShrinkRound() => _map.Offset(-0.4);

    [Benchmark]
    public Region OffsetGrowSquare() => _other.Offset(0.5, JoinStyle.Square);

    [Benchmark]
    public Triangulation Triangulate() => _map.Triangulate();

    [Benchmark]
    public ConvexPartition PartitionConvex() => _map.PartitionConvex();
}
