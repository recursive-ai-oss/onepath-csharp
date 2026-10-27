using System;
using System.IO;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// A* on a Moving AI Lab map and scenario file (8-connected, no corner cutting): one invocation answers every scenario
/// with a reused search object, so it measures the search itself. The scenario file is read from
/// <c>ONEPATH_GRID_SCEN</c> (default <c>$ONEPATH_HOME/maps/dao/brc000d.map.scen</c>, next to its <c>.map</c>);
/// without any file a generated 512x512 map stands in so the benchmark still runs.
/// </summary>
[MemoryDiagnoser]
public class GridAStarBenchmarks
{
    private Grid _grid = null!;
    private int[] _starts = [];
    private int[] _goals = [];
    private AStar<SquareTopology, DefaultStepCost, OctileHeuristic> _search = null!;

    [GlobalSetup]
    public void Setup()
    {
        string? scenPath = Environment.GetEnvironmentVariable("ONEPATH_GRID_SCEN");
        if (scenPath is null)
        {
            string home = Environment.GetEnvironmentVariable("ONEPATH_HOME") ?? Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.UserProfile), "onepath");
            scenPath = Path.Combine(home, "maps", "dao", "brc000d.map.scen");
        }
        if (File.Exists(scenPath) && File.Exists(scenPath[..^5]))
        {
            _grid = MovingAiMap.Parse(File.ReadAllBytes(scenPath[..^5])).ToGrid();
            ScenarioEntry[] scen = MovingAiScenarios.Parse(File.ReadAllBytes(scenPath));
            _starts = new int[scen.Length];
            _goals = new int[scen.Length];
            for (int i = 0; i < scen.Length; i++)
            {
                _starts[i] = (scen[i].StartY * _grid.Width) + scen[i].StartX;
                _goals[i] = (scen[i].GoalY * _grid.Width) + scen[i].GoalX;
            }
        }
        else
        {
            _grid = new Grid(512, 512);
            var rng = new SeededRandom(1);
            for (int i = 0; i < 40000; i++)
            {
                _grid.SetWalkable(rng.NextInt(512), rng.NextInt(512), false);
            }
            _starts = new int[500];
            _goals = new int[500];
            for (int i = 0; i < 500; i++)
            {
                do
                {
                    _starts[i] = rng.NextInt(512 * 512);
                }
                while (!_grid.IsWalkable(_starts[i] % 512, _starts[i] / 512));
                do
                {
                    _goals[i] = rng.NextInt(512 * 512);
                }
                while (!_grid.IsWalkable(_goals[i] % 512, _goals[i] / 512));
            }
        }
        _search = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(
            _grid.CreateTopology(), default, new OctileHeuristic(_grid.Width), new GridSearchBuffers(_grid.CellCount));
    }

    [Benchmark]
    public double AllScenarios()
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            _search.Begin(_starts[i], _goals[i]);
            if (_search.Run() == SearchStatus.Found)
            {
                sum += _search.PathCost;
            }
        }
        return sum;
    }
}
