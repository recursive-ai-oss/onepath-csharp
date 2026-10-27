using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// HPA* on the same map and scenarios as <see cref="GridAStarBenchmarks"/> (set <c>ONEPATH_GRID_SCEN</c> the same way): hierarchy
/// build, one chunk edit followed by the incremental refresh, and every scenario with and without smoothing next to A* on the
/// same search objects. Answers carry no reference; the rows compare against this library's own A*.
/// </summary>
[MemoryDiagnoser]
public sealed class HpaBenchmarks : System.IDisposable
{
    private readonly GridAStarBenchmarks _inputs = new();
    private Grid _grid = null!;
    private int[] _starts = [];
    private int[] _goals = [];
    private HpaHierarchy _hierarchy = null!;
    private HpaSearch _search = null!;
    private AStar<SquareTopology, DefaultStepCost, OctileHeuristic> _astar = null!;
    private int _toggle;

    /// <summary>Chunk side in cells.</summary>
    [Params(8, 16, 32)]
    public int Chunk { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _inputs.Setup();
        _grid = _inputs.GridForJps;
        _starts = _inputs.StartsForJps;
        _goals = _inputs.GoalsForJps;
        _hierarchy = new HpaHierarchy(_grid, new HpaOptions { ChunkSize = Chunk });
        _search = new HpaSearch(_hierarchy);
        _astar = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(_grid.CreateTopology(), default, new OctileHeuristic(_grid.Width));
    }

    [GlobalCleanup]
    public void Cleanup() => Dispose();

    public void Dispose() => _hierarchy?.Dispose();

    [Benchmark]
    public int BuildHierarchy()
    {
        using var h = new HpaHierarchy(_grid, new HpaOptions { ChunkSize = Chunk });
        return h.NodeCount;
    }

    [Benchmark]
    public int EditOneCellAndRefresh()
    {
        int x = _grid.Width / 2;
        int y = _grid.Height / 2;
        _grid.SetArea(x, y, _toggle++ % 2 == 0 ? AreaCosts.Wall : (byte)0);
        _hierarchy.Refresh();
        return _hierarchy.NodeCount;
    }

    [Benchmark(Baseline = true)]
    public double AllScenariosAStar()
    {
        double total = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            if (_grid.Clearance[_starts[i]] == 0 || _grid.Clearance[_goals[i]] == 0)
            {
                continue;
            }
            _astar.Begin(_starts[i], _goals[i]);
            if (_astar.Run() == SearchStatus.Found)
            {
                total += _astar.PathCost;
            }
        }
        return total;
    }

    [Benchmark]
    public double AllScenariosHpaSmoothed() => RunAll(true);

    [Benchmark]
    public double AllScenariosHpaRaw() => RunAll(false);

    private double RunAll(bool smooth)
    {
        _search.Smoothing = smooth;
        double total = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            if (_grid.Clearance[_starts[i]] == 0 || _grid.Clearance[_goals[i]] == 0)
            {
                continue;
            }
            _search.Begin(_starts[i], _goals[i]);
            if (_search.Run() == SearchStatus.Found)
            {
                total += _search.PathCost;
            }
        }
        return total;
    }
}
