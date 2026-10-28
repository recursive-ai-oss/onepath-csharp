using System;
using BenchmarkDotNet.Attributes;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Micro;

/// <summary>
/// The specialised octile searches (A*, JPS, JPS+, each with the improved pruning of Harabor and Grastien 2014 where it applies) on the
/// inputs of <see cref="GridAStarBenchmarks"/>: one invocation answers every scenario with a reused search object.
/// </summary>
[MemoryDiagnoser]
public class GridOctileBenchmarks
{
    private readonly GridAStarBenchmarks _inputs = new();
    private int[] _starts = [];
    private int[] _goals = [];
    private OctileAStar _astar = null!;
    private OctileJump<BitFinder> _jps = null!;
    private OctileJump<BitFinder> _jpsP = null!;
    private OctileJump<TableFinder> _plus = null!;
    private OctileJump<TableFinder> _plusP = null!;

    [GlobalSetup]
    public void Setup()
    {
        _inputs.Setup();
        Grid grid = _inputs.GridForJps;
        _starts = _inputs.StartsForJps;
        _goals = _inputs.GoalsForJps;
        var map = new OctileMap(grid);
        var index = new JpsPlusIndex(grid);
        _astar = new OctileAStar(grid);
        _jps = new OctileJump<BitFinder>(new BitFinder(map, false), grid.Width, grid.Height);
        _jpsP = new OctileJump<BitFinder>(new BitFinder(map, true), grid.Width, grid.Height);
        _plus = new OctileJump<TableFinder>(new TableFinder(index, map, false), grid.Width, grid.Height);
        _plusP = new OctileJump<TableFinder>(new TableFinder(index, map, true), grid.Width, grid.Height);
    }

    [Benchmark(Baseline = true)]
    public double AStar()
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            if (_astar.Run(_starts[i], _goals[i], false, int.MaxValue, false) == SearchStatus.Found)
            {
                sum += _astar.PathCost;
            }
        }
        return sum;
    }

    [Benchmark]
    public double Jps() => SumJump(_jps);

    [Benchmark]
    public double JpsImproved() => SumJump(_jpsP);

    [Benchmark]
    public double JpsPlus() => SumJump(_plus);

    [Benchmark]
    public double JpsPlusImproved() => SumJump(_plusP);

    private double SumJump<T>(OctileJump<T> search)
        where T : struct, IJumpFinder
    {
        double sum = 0;
        for (int i = 0; i < _starts.Length; i++)
        {
            if (search.Run(_starts[i], _goals[i], false, int.MaxValue, false) == SearchStatus.Found)
            {
                sum += search.PathCost;
            }
        }
        return sum;
    }
}
