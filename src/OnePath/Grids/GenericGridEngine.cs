using System;

namespace RecursiveAI.OnePath.Grids;

// The non-generic face of AStar<SquareTopology, TCost, THeuristic> for GridSearch: the options pick one of the eight instantiations
// (cost: plain or delegates; heuristic: zero, Manhattan, Euclidean, octile) exactly as Grid.FindPath does.
internal abstract class GenericGridEngine
{
    public abstract SearchStatus Status { get; }

    public abstract int ExpandedCount { get; }

    public abstract int Iterations { get; }

    public abstract double PathCost { get; }

    public abstract void Begin(int start, int goal);

    public abstract SearchStatus Step(int iterations);

    public abstract int PathLength();

    public abstract void CopyPath(Span<int> destination);

    public abstract void CopyCells(Span<Cell> destination, int width);

    /// <summary>Writes the engine's own state blob (kind AStar or JumpSearch) into <paramref name="w"/>, reusing its buffer.</summary>
    public abstract void SaveTo(StateWriter w);

    /// <summary>The grid-side structure the engine reads (jump point engines only); the engine is rebuilt when the grid hands out another one.</summary>
    public virtual object? Token => null;

    public abstract void RestoreState(ReadOnlySpan<byte> state);

    internal static GenericGridEngine Create(Grid grid, GridOptions o)
    {
        SquareTopology topology = grid.CreateTopology(o.AgentSize, o.Connectivity, o.Diagonals);
        GridHeuristic h = o.Heuristic;
        if (h == GridHeuristic.Auto)
        {
            h = o.Connectivity == Connectivity.Four ? GridHeuristic.Manhattan : GridHeuristic.Octile;
        }
        double scale = grid.Areas.MinCost;
        int w = grid.Width;
        double weight = o.HeuristicWeight;
        if (o.StepCost is null && o.Blocked is null)
        {
            var cost = default(DefaultStepCost);
            return h switch
            {
                GridHeuristic.Zero => new Impl<DefaultStepCost, ZeroHeuristic>(topology, cost, default, w, weight),
                GridHeuristic.Manhattan => new Impl<DefaultStepCost, ManhattanHeuristic>(topology, cost, new ManhattanHeuristic(w, scale), w, weight),
                GridHeuristic.Euclidean => new Impl<DefaultStepCost, EuclideanHeuristic>(topology, cost, new EuclideanHeuristic(w, scale), w, weight),
                _ => new Impl<DefaultStepCost, OctileHeuristic>(topology, cost, new OctileHeuristic(w, scale), w, weight),
            };
        }
        else
        {
            var cost = new SearchRunner.DelegateStepCost { Width = w, Step = o.StepCost, Blocked = o.Blocked };
            return h switch
            {
                GridHeuristic.Zero => new Impl<SearchRunner.DelegateStepCost, ZeroHeuristic>(topology, cost, default, w, weight),
                GridHeuristic.Manhattan => new Impl<SearchRunner.DelegateStepCost, ManhattanHeuristic>(topology, cost, new ManhattanHeuristic(w, scale), w, weight),
                GridHeuristic.Euclidean => new Impl<SearchRunner.DelegateStepCost, EuclideanHeuristic>(topology, cost, new EuclideanHeuristic(w, scale), w, weight),
                _ => new Impl<SearchRunner.DelegateStepCost, OctileHeuristic>(topology, cost, new OctileHeuristic(w, scale), w, weight),
            };
        }
    }

    // Jump point search over the block map (JPS) or the JPS+ tables, as Grid.FindPath runs it when the specialised engines are off or
    // the estimate is not plain octile or zero.
    internal static GenericGridEngine CreateJump(Grid grid, GridOptions o, bool plus, double unitCost)
    {
        bool zero = o.Heuristic == GridHeuristic.Zero;
        if (plus)
        {
            JpsPlusIndex index = grid.SharedJpsPlus();
            var jumper = new TableJumper(index);
            return zero
                ? new JumpImpl<TableJumper, ZeroHeuristic>(jumper, default, unitCost, index)
                : new JumpImpl<TableJumper, OctileHeuristic>(jumper, new OctileHeuristic(grid.Width, unitCost), unitCost, index);
        }
        BlockMap map = grid.SharedBlockMap();
        var block = new BlockJumper(map);
        return zero
            ? new JumpImpl<BlockJumper, ZeroHeuristic>(block, default, unitCost, map)
            : new JumpImpl<BlockJumper, OctileHeuristic>(block, new OctileHeuristic(grid.Width, unitCost), unitCost, map);
    }

    private sealed class JumpImpl<TJumper, THeuristic> : GenericGridEngine
        where TJumper : struct, IJumper
        where THeuristic : struct, IHeuristic
    {
        private readonly JumpSearch<TJumper, THeuristic> _search;
        private readonly object _token;
        private int[] _scratch = [];

        public JumpImpl(TJumper jumper, THeuristic heuristic, double unitCost, object token)
        {
            _token = token;
            _search = new JumpSearch<TJumper, THeuristic>(jumper, heuristic, unitCost, new GridSearchBuffers(jumper.Width * jumper.Height));
        }

        public override object? Token => _token;

        public override SearchStatus Status => _search.Status;

        public override int ExpandedCount => _search.ExpandedCount;

        public override int Iterations => _search.Iterations;

        public override double PathCost => _search.PathCost;

        public override void Begin(int start, int goal) => _search.Begin(start, goal);

        public override SearchStatus Step(int iterations) => _search.Step(iterations);

        public override int PathLength() => _search.PathLength();

        public override void CopyPath(Span<int> destination) => _search.CopyPath(destination);

        public override void CopyCells(Span<Cell> destination, int width)
        {
            // The hops are expanded into cell indexes first; the scratch grows to the longest path seen and is then reused.
            int n = _search.PathLength();
            if (_scratch.Length < n)
            {
                _scratch = new int[n];
            }
            _search.CopyPath(_scratch);
            for (int i = 0; i < n; i++)
            {
                destination[i] = SearchRunner.ToCell(_scratch[i], width);
            }
        }

        public override void SaveTo(StateWriter w) => _search.SaveTo(w);

        public override void RestoreState(ReadOnlySpan<byte> state) => _search.RestoreState(state);
    }

    private sealed class Impl<TCost, THeuristic> : GenericGridEngine
        where TCost : struct, IStepCost
        where THeuristic : struct, IHeuristic
    {
        private readonly AStar<SquareTopology, TCost, THeuristic> _search;
        private readonly int _width;

        public Impl(SquareTopology topology, TCost cost, THeuristic heuristic, int width, double weight)
        {
            _width = width;
            _search = new AStar<SquareTopology, TCost, THeuristic>(topology, cost, heuristic, new GridSearchBuffers(topology.NodeCount))
            {
                HeuristicWeight = weight,
            };
        }

        public override SearchStatus Status => _search.Status;

        public override int ExpandedCount => _search.ExpandedCount;

        public override int Iterations => _search.Iterations;

        public override double PathCost => _search.PathCost;

        public override void Begin(int start, int goal) => _search.Begin(start, goal);

        public override SearchStatus Step(int iterations) => _search.Step(iterations);

        public override int PathLength() => _search.PathLength();

        public override void CopyPath(Span<int> destination) => _search.CopyPath(destination);

        public override void CopyCells(Span<Cell> destination, int width)
        {
            // The path is written backwards from the goal along the parent links; no scratch is needed.
            GridSearchBuffers b = _search.Buffers;
            int i = _search.PathLength();
            for (int n = _search.Goal; n >= 0; n = b.Parent[n])
            {
                destination[--i] = SearchRunner.ToCell(n, width);
            }
        }

        public override void SaveTo(StateWriter w) => _search.SaveTo(w);

        public override void RestoreState(ReadOnlySpan<byte> state) => _search.RestoreState(state);
    }
}
