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

    internal Grid GridForJps => _grid;

    internal int[] StartsForJps => _starts;

    internal int[] GoalsForJps => _goals;

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

/// <summary>
/// JPS (block scanning) and JPS+ on the same inputs as <see cref="GridAStarBenchmarks"/>, plus the JPS+ table build and an
/// incremental repair after one cell edit. Table builds are reported separately from the searches, as the reference drivers do.
/// </summary>
[MemoryDiagnoser]
public sealed class GridJpsBenchmarks : IDisposable
{
    private readonly GridAStarBenchmarks _inputs = new();
    private Grid _grid = null!;
    private int[] _starts = [];
    private int[] _goals = [];
    private JumpSearch<BlockJumper, OctileHeuristic> _jps = null!;
    private JumpSearch<TableJumper, OctileHeuristic> _jpsPlus = null!;
    private JpsPlusIndex _index = null!;
    private int _editX;
    private int _editY;

    [GlobalSetup]
    public void Setup()
    {
        _inputs.Setup();
        _grid = _inputs.GridForJps;
        _starts = _inputs.StartsForJps;
        _goals = _inputs.GoalsForJps;
        _jps = new JumpSearch<BlockJumper, OctileHeuristic>(new BlockJumper(new BlockMap(_grid)), new OctileHeuristic(_grid.Width), 1.0, new GridSearchBuffers(_grid.CellCount));
        _index = new JpsPlusIndex(_grid);
        _jpsPlus = new JumpSearch<TableJumper, OctileHeuristic>(new TableJumper(_index), new OctileHeuristic(_grid.Width), 1.0, new GridSearchBuffers(_grid.CellCount));
        // An open cell near the middle for the edit benchmark.
        _editX = _grid.Width / 2;
        _editY = _grid.Height / 2;
    }

    [GlobalCleanup]
    public void Dispose() => _index?.Dispose();

    [Benchmark]
    public double AllScenariosJps()
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            _jps.Begin(_starts[i], _goals[i]);
            if (_jps.Run() == SearchStatus.Found)
            {
                sum += _jps.PathCost;
            }
        }
        return sum;
    }

    [Benchmark]
    public double AllScenariosJpsPlus()
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            _jpsPlus.Begin(_starts[i], _goals[i]);
            if (_jpsPlus.Run() == SearchStatus.Found)
            {
                sum += _jpsPlus.PathCost;
            }
        }
        return sum;
    }

    [Benchmark]
    public int BuildJpsPlusTables()
    {
        using var index = new JpsPlusIndex(_grid);
        return index.JumpDistance(1, 0, 0);
    }

    [Benchmark]
    public int BuildBlockMap()
    {
        using var map = new BlockMap(_grid);
        return map.RowWords;
    }

    [Benchmark]
    public int RepairAfterOneEdit()
    {
        // Toggle one cell and bring the tables up to date: the cost of an incremental update.
        bool wall = !_grid.IsWalkable(_editX, _editY);
        _grid.SetWalkable(_editX, _editY, wall);
        _index.Refresh();
        return _index.JumpDistance(1, 0, 0);
    }
}
