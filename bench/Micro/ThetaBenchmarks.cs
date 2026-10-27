using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// Theta*, Lazy Theta* (as in the paper, and re-queued) and, for context, A* on the same inputs as <see cref="GridAStarBenchmarks"/>.
/// Each invocation answers every scenario with a reused search object. No reference implementation is pinned for any-angle search,
/// so the rows are read against A* and JPS on the same instances.
/// </summary>
[MemoryDiagnoser]
public class GridThetaBenchmarks
{
    private readonly GridAStarBenchmarks _inputs = new();
    private int[] _starts = [];
    private int[] _goals = [];
    private ThetaSearch<EuclideanHeuristic> _theta = null!;
    private ThetaSearch<EuclideanHeuristic> _lazy = null!;
    private ThetaSearch<EuclideanHeuristic> _lazyRequeue = null!;
    private AStar<SquareTopology, DefaultStepCost, OctileHeuristic> _astar = null!;

    [GlobalSetup]
    public void Setup()
    {
        _inputs.Setup();
        Grid grid = _inputs.GridForJps;
        _starts = _inputs.StartsForJps;
        _goals = _inputs.GoalsForJps;
        var buffers = new GridSearchBuffers(grid.CellCount);
        _theta = new ThetaSearch<EuclideanHeuristic>(grid, new EuclideanHeuristic(grid.Width), ThetaVariant.Theta, 1.0, buffers);
        _lazy = new ThetaSearch<EuclideanHeuristic>(grid, new EuclideanHeuristic(grid.Width), ThetaVariant.Lazy, 1.0, buffers);
        _lazyRequeue = new ThetaSearch<EuclideanHeuristic>(grid, new EuclideanHeuristic(grid.Width), ThetaVariant.LazyRequeue, 1.0, buffers);
        _astar = new AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(grid.CreateTopology(), default, new OctileHeuristic(grid.Width), buffers);
    }

    private double Run(ThetaSearch<EuclideanHeuristic> search)
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

    [Benchmark(Baseline = true)]
    public double AllScenariosAStar()
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            _astar.Begin(_starts[i], _goals[i]);
            if (_astar.Run() == SearchStatus.Found)
            {
                sum += _astar.PathCost;
            }
        }
        return sum;
    }

    [Benchmark]
    public double AllScenariosTheta() => Run(_theta);

    [Benchmark]
    public double AllScenariosLazyTheta() => Run(_lazy);

    [Benchmark]
    public double AllScenariosLazyThetaRequeue() => Run(_lazyRequeue);
}
