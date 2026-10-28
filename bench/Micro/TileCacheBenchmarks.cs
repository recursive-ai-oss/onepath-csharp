using System.Collections.Generic;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Bench;

/// <summary>
/// Run-time rebuild rows: a 64 x 64 rolling terrain (tiles of 16 units, cell 0.3) built from the soup with and without keeping the tile cache, and the cost of one
/// obstacle appearing and disappearing (two single-tile rebuilds from the cached data, plus the queueing).
/// </summary>
[MemoryDiagnoser]
public class TileCacheBenchmarks
{
    private double[] _v = System.Array.Empty<double>();
    private int[] _i = System.Array.Empty<int>();
    private Mesh3DPipeline _pipe = null!;
    private TileCache _cache = null!;
    private readonly TileObstacle _crate = TileObstacle.Box(20, -1, 20, 22, 3, 22);

    [GlobalSetup]
    public void Setup()
    {
        var v = new List<double>();
        var ix = new List<int>();
        for (int z = 0; z < 64; z++)
        {
            for (int x = 0; x < 64; x++)
            {
                int a = v.Count / 3;
                foreach ((double px, double pz) in new[] { (x, z), (x + 1.0, z), (x + 1.0, z + 1.0), (x, z + 1.0) })
                {
                    v.AddRange(new[] { px, (0.02 * px) + (0.002 * (px - 20) * (px - 20)) + (0.01 * pz) + ((((int)px * 7) + ((int)pz * 13)) % 5 * 0.01), pz });
                }
                ix.AddRange(new[] { a, a + 3, a + 2, a, a + 2, a + 1 });
            }
        }
        _v = v.ToArray();
        _i = ix.ToArray();
        _pipe = new Mesh3DPipeline(new NavSettings { CellSize = 0.3, CellHeight = 0.2, TileSize = 16, AgentRadius = 0.6f, AgentHeight = 2f, WalkableClimb = 0.9f });
        _cache = _pipe.BuildSoupCached(_v, _i);
    }

    /// <summary>The whole level, no cache kept.</summary>
    [Benchmark(Baseline = true)]
    public int BuildLevel() => _pipe.BuildSoup(_v, _i).TileCount;

    /// <summary>The whole level, keeping the per-tile cache.</summary>
    [Benchmark]
    public int BuildLevelCached() => _pipe.BuildSoupCached(_v, _i).Mesh.TileCount;

    /// <summary>An obstacle appears in one tile and goes away again: two rebuilds of that tile from the cache.</summary>
    [Benchmark]
    public int ObstacleOnAndOff()
    {
        int id = _cache.AddObstacle(_crate);
        int n = _cache.UpdateAll();
        _cache.RemoveObstacle(id);
        return n + _cache.UpdateAll();
    }
}
