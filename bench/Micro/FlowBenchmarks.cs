using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// Flow fields on the same map as <see cref="GridAStarBenchmarks"/> (8-connected, no corner cutting): a whole-map integration field
/// per goal, and the repair of a field after one cell edit against recomputing it. Each invocation uses the first 20 scenario goals.
/// </summary>
[MemoryDiagnoser]
public class FlowFieldBenchmarks
{
    private readonly GridAStarBenchmarks _inputs = new();
    private Grid _grid = null!;
    private int[] _goals = [];
    private FlowField<SquareTopology, DefaultStepCost> _field = null!;
    private GridFlowField _watched = null!;
    private int _editX;
    private int _editY;
    private bool _wall;

    [GlobalSetup]
    public void Setup()
    {
        _inputs.Setup();
        _grid = _inputs.GridForJps;
        _goals = _inputs.GoalsForJps[..20];
        _field = new FlowField<SquareTopology, DefaultStepCost>(_grid.CreateTopology(), default);
        _watched = _grid.CreateFlowField([new Cell(_goals[0] % _grid.Width, _goals[0] / _grid.Width)]);
        _editX = _grid.Width / 2;
        _editY = _grid.Height / 2;
        while (!_grid.IsWalkable(_editX, _editY))
        {
            _editX++;
        }
    }

    [GlobalCleanup]
    public void Cleanup() => _watched?.Dispose();

    [Benchmark]
    public double FullFieldPer20Goals()
    {
        double sum = 0;
        foreach (int g in _goals)
        {
            _field.Begin([g]);
            _field.Run();
            sum += _field.Cost(_goals[0]);
        }
        return sum;
    }

    [Benchmark]
    public double RepairAfterOneEdit()
    {
        _wall = !_wall;
        _grid.SetWalkable(_editX, _editY, !_wall);
        _watched.Refresh();
        return _watched.GetCost(new Cell(0, 0));
    }

    [Benchmark]
    public double RecomputeAfterOneEdit()
    {
        _wall = !_wall;
        _grid.SetWalkable(_editX, _editY, !_wall);
        _watched.Recompute();
        return _watched.GetCost(new Cell(0, 0));
    }
}
