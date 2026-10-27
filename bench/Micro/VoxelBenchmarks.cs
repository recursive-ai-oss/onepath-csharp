using System;
using System.IO;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// 26-connected voxel A* and Dijkstra (the same search with a zero estimate) on a Moving AI Lab voxel map and scenario file:
/// one invocation answers the first 200 scenarios with a reused search object. The scenario file comes from
/// <c>ONEPATH_VOXEL_SCEN</c> (default <c>$ONEPATH_HOME/maps/warframe/Simple.3dmap.3dscen</c>, next to its <c>.3dmap</c>);
/// without a file a generated 64x64x64 block stands in so the benchmark still runs.
/// </summary>
[MemoryDiagnoser]
public class VoxelSearchBenchmarks
{
    private const int MaxScenarios = 200;

    private VoxelGrid _grid = null!;
    private int[] _starts = [];
    private int[] _goals = [];
    private AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic> _astar = null!;
    private AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic> _dijkstra = null!;

    [GlobalSetup]
    public void Setup()
    {
        string? scenPath = Environment.GetEnvironmentVariable("ONEPATH_VOXEL_SCEN");
        if (scenPath is null)
        {
            string home = Environment.GetEnvironmentVariable("ONEPATH_HOME") ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "onepath");
            scenPath = Path.Combine(home, "maps", "warframe", "Simple.3dmap.3dscen");
        }
        if (File.Exists(scenPath) && File.Exists(scenPath[..^6]))
        {
            _grid = VoxelMap.Parse(File.ReadAllBytes(scenPath[..^6])).ToGrid();
            VoxelScenarioEntry[] scen = VoxelScenarios.Parse(File.ReadAllBytes(scenPath));
            int n = Math.Min(MaxScenarios, scen.Length);
            _starts = new int[n];
            _goals = new int[n];
            for (int i = 0; i < n; i++)
            {
                _starts[i] = _grid.NodeIndex(scen[i].StartX, scen[i].StartY, scen[i].StartZ);
                _goals[i] = _grid.NodeIndex(scen[i].GoalX, scen[i].GoalY, scen[i].GoalZ);
            }
        }
        else
        {
            _grid = new VoxelGrid(64, 64, 64);
            var rng = new SeededRandom(1);
            for (int i = 0; i < 40000; i++)
            {
                _grid.SetWalkable(rng.NextInt(64), rng.NextInt(64), rng.NextInt(64), false);
            }
            _starts = new int[MaxScenarios];
            _goals = new int[MaxScenarios];
            for (int i = 0; i < MaxScenarios; i++)
            {
                _starts[i] = RandomOpen(ref rng);
                _goals[i] = RandomOpen(ref rng);
            }
        }
        VoxelTopology topo = _grid.CreateTopology();
        var buffers = new GridSearchBuffers(_grid.VoxelCount);
        _astar = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, new VoxelHeuristic(_grid.SizeX, _grid.SizeY, VoxelHeuristicKind.Octile26), buffers);
        _dijkstra = new AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic>(topo, default, new VoxelHeuristic(_grid.SizeX, _grid.SizeY, VoxelHeuristicKind.Zero), buffers);
    }

    private int RandomOpen(ref SeededRandom rng)
    {
        while (true)
        {
            int node = rng.NextInt(_grid.VoxelCount);
            VoxelCell c = _grid.CellOf(node);
            if (_grid.IsWalkable(c.X, c.Y, c.Z))
            {
                return node;
            }
        }
    }

    [Benchmark(Baseline = true)]
    public double AStarAllScenarios() => Run(_astar);

    [Benchmark]
    public double DijkstraAllScenarios() => Run(_dijkstra);

    private double Run(AStar<VoxelTopology, DefaultStepCost, VoxelHeuristic> search)
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            search.Begin(_starts[i], _goals[i]);
            if (search.Run() == SearchStatus.Found)
            {
                sum += search.PathCost;
            }
        }
        return sum;
    }
}
