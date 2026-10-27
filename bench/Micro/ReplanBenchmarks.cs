using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// Repairing a path after a wall toggle (D* Lite, LPA*) against searching again with A*, on a generated 512x512 map with scattered walls:
/// each invocation makes 100 toggles near the path between two far corners and asks for the new path after each. Rows report no reference
/// implementation; the comparison is with A* on the same instances.
/// </summary>
[MemoryDiagnoser]
public class ReplanBenchmarks
{
    private const int Toggles = 100;
    private Grid _grid = null!;
    private Cell _start;
    private Cell _goal;
    private Cell[] _sites = [];

    [Params(GridAlgorithm.AStar, GridAlgorithm.DStarLite, GridAlgorithm.LpaStar)]
    public GridAlgorithm Algorithm { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        _grid = new Grid(512, 512);
        var rng = new SeededRandom(7);
        for (int i = 0; i < 30000; i++)
        {
            _grid.SetWalkable(rng.NextInt(512), rng.NextInt(512), false);
        }
        _start = new Cell(2, 2);
        _goal = new Cell(509, 509);
        _grid.SetWalkable(_start.X, _start.Y, true);
        _grid.SetWalkable(_goal.X, _goal.Y, true);
        GridPath p = _grid.FindPath(_start, _goal);
        _sites = new Cell[Toggles];
        for (int i = 0; i < Toggles; i++)
        {
            _sites[i] = p.Found ? p.Cells[1 + rng.NextInt(p.Cells.Length - 2)] : new Cell(100, 100);
        }
    }

    [Benchmark]
    public double ToggleAndAskForPath()
    {
        double sum = 0;
        GridReplanner? r = Algorithm == GridAlgorithm.AStar ? null : _grid.CreateReplanner(_start, _goal, new GridOptions { Algorithm = Algorithm });
        r?.Replan();
        for (int i = 0; i < Toggles; i++)
        {
            Cell c = _sites[i];
            _grid.SetWalkable(c.X, c.Y, false);
            sum += (r is null ? _grid.FindPath(_start, _goal) : r.Replan()).Cost;
        }
        for (int i = 0; i < Toggles; i++)
        {
            _grid.SetWalkable(_sites[i].X, _sites[i].Y, true);
        }
        r?.Dispose();
        return sum;
    }
}
