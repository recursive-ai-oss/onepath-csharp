using System.Runtime.CompilerServices;
using System;
using System.Threading.Tasks;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Orca;

/// <summary>
/// A minimal fixed-step ORCA simulation: agents with preferred velocities and static segment obstacles. A step computes every
/// agent's new velocity from the state at the start of the step, then integrates all positions. Nothing depends on a clock,
/// the thread count or the order in which agents were added beyond their indices. After construction and the first steps
/// (buffers grow, never shrink) a step with <see cref="MaxThreads"/> = 1 does not allocate.
/// The neighbour and constraint order is specified in <c>docs/orca.md</c>.
/// </summary>
public sealed class OrcaSimulator
{
    private const int ChunkSize = 32;
    private const int MaxAgents = (1 << 22) - 1;

    private OrcaAgent[] _agents = new OrcaAgent[16];
    private Vector2D[] _newVelocity = new Vector2D[16];
    private OrcaSegment[] _obstacles = new OrcaSegment[4];
    private long[] _cellKeys = new long[16];
    private int _agentCount;
    private int _obstacleCount;
    private int _maxThreads = 1;
    private Worker[] _workers = new Worker[1];

    // The grid of the current step.
    private double _gridMinX;
    private double _gridMinY;
    private double _cellSize = 1;
    private long _gridWidth = 1;

    /// <summary>Number of agents.</summary>
    public int Count => _agentCount;

    /// <summary>Number of obstacle segments.</summary>
    public int ObstacleCount => _obstacleCount;

    /// <summary>
    /// How many workers compute velocities (default 1). Agents are split by index and results are written per agent, so the
    /// result is identical for every value. Values above 1 use <see cref="Parallel"/> and allocate.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is less than 1.</exception>
    public int MaxThreads
    {
        get => _maxThreads;
        set
        {
            ArgumentOutOfRangeException.ThrowIfLessThan(value, 1);
            _maxThreads = value;
        }
    }

    /// <summary>The agent with the given index; the reference is valid until the next <see cref="AddAgent"/>.</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="index"/> is out of range.</exception>
    public ref OrcaAgent this[int index]
    {
        get
        {
            if ((uint)index >= (uint)_agentCount)
            {
                throw new ArgumentOutOfRangeException(nameof(index), index, "There is no agent with this index.");
            }
            return ref _agents[index];
        }
    }

    /// <summary>Adds an agent and returns its index (indices are assigned 0, 1, 2, ... in call order).</summary>
    /// <exception cref="ArgumentException">A field is out of range or not finite, or there are too many agents.</exception>
    public int AddAgent(in OrcaAgent agent)
    {
        Validate(agent);
        if (_agentCount >= MaxAgents)
        {
            throw new ArgumentException("Too many agents.", nameof(agent));
        }
        if (_agentCount == _agents.Length)
        {
            Array.Resize(ref _agents, _agents.Length * 2);
            Array.Resize(ref _newVelocity, _agents.Length);
            Array.Resize(ref _cellKeys, _agents.Length);
        }
        _agents[_agentCount] = agent;
        return _agentCount++;
    }

    /// <summary>Adds a static obstacle segment and returns its index.</summary>
    /// <exception cref="ArgumentException">An end is not finite.</exception>
    public int AddObstacle(OrcaSegment segment)
    {
        if (!IsFinite(segment.A) || !IsFinite(segment.B))
        {
            throw new ArgumentException("Obstacle ends must be finite.", nameof(segment));
        }
        if (_obstacleCount == _obstacles.Length)
        {
            Array.Resize(ref _obstacles, _obstacles.Length * 2);
        }
        _obstacles[_obstacleCount] = segment;
        return _obstacleCount++;
    }

    /// <summary>
    /// Advances the simulation by <paramref name="dt"/>: new velocities for all agents from the current state, then
    /// <c>position += velocity * dt</c>.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="dt"/> is not finite and greater than 0.</exception>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public void Step(double dt)
    {
        if (!(dt > 0 && double.IsFinite(dt)))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "The time step must be finite and greater than 0.");
        }
        int n = _agentCount;
        if (n == 0)
        {
            return;
        }
        BuildGrid();

        int threads = Math.Min(_maxThreads, (n + ChunkSize - 1) / ChunkSize);
        if (_workers.Length < threads)
        {
            Array.Resize(ref _workers, threads);
        }
        for (int w = 0; w < threads; w++)
        {
            _workers[w] ??= new Worker();
        }

        if (threads <= 1)
        {
            ComputeRange(0, n, dt, _workers[0]);
        }
        else
        {
            ComputeParallel(n, threads, dt);
        }

        for (int i = 0; i < n; i++)
        {
            ref OrcaAgent a = ref _agents[i];
            a.Velocity = _newVelocity[i];
            a.Position += a.Velocity * dt;
        }
    }

    // Kept out of Step so the closure it needs is only allocated when threads are used.
    private void ComputeParallel(int n, int threads, double dt)
    {
        // Worker w takes chunks w, w + threads, ... Each agent's velocity depends only on the start-of-step state and
        // is written to its own slot, so the result does not depend on the number of workers or their timing.
        int chunks = (n + ChunkSize - 1) / ChunkSize;
        Parallel.For(0, threads, new ParallelOptions { MaxDegreeOfParallelism = threads }, w =>
        {
            for (int chunk = w; chunk < chunks; chunk += threads)
            {
                int from = chunk * ChunkSize;
                ComputeRange(from, Math.Min(n, from + ChunkSize), dt, _workers[w]);
            }
        });
    }

    private static bool IsFinite(Vector2D v) => double.IsFinite(v.X) && double.IsFinite(v.Y);

    private static void Validate(in OrcaAgent a)
    {
        if (!IsFinite(a.Position) || !IsFinite(a.Velocity) || !IsFinite(a.PreferredVelocity))
        {
            throw new ArgumentException("Position, velocity and preferred velocity must be finite.", nameof(a));
        }
        if (!(a.Radius >= 0 && double.IsFinite(a.Radius)) || !(a.MaxSpeed >= 0 && double.IsFinite(a.MaxSpeed)) ||
            !(a.NeighborDistance >= 0 && double.IsFinite(a.NeighborDistance)) || a.MaxNeighbors < 0 ||
            !(a.TimeHorizon > 0 && double.IsFinite(a.TimeHorizon)) || !(a.ObstacleTimeHorizon > 0 && double.IsFinite(a.ObstacleTimeHorizon)))
        {
            throw new ArgumentException("Radius, speeds and distances must be finite and at least 0, the time horizons finite and greater than 0, the neighbour count at least 0.", nameof(a));
        }
        // Positions beyond the coordinate bound are rejected, as everywhere in the library.
        const double bound = 2097152.0;
        if (Math.Abs(a.Position.X) >= bound || Math.Abs(a.Position.Y) >= bound)
        {
            throw new ArgumentOutOfRangeException(nameof(a), a.Position, "Agent positions must have magnitude below 2^21 = 2097152.");
        }
    }

    // Sorts the agents by (cell, index): cell = row * width + column. The 3 cells of one row are consecutive keys.
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void BuildGrid()
    {
        int n = _agentCount;
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity, maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        double cell = 0;
        for (int i = 0; i < n; i++)
        {
            ref OrcaAgent a = ref _agents[i];
            minX = Math.Min(minX, a.Position.X);
            minY = Math.Min(minY, a.Position.Y);
            maxX = Math.Max(maxX, a.Position.X);
            maxY = Math.Max(maxY, a.Position.Y);
            cell = Math.Max(cell, a.NeighborDistance);
        }
        double extent = Math.Max(maxX - minX, maxY - minY);
        // At least 1/2^20 of the extent so cell coordinates stay small; never below a tiny positive size.
        cell = Math.Max(cell, extent / 1048576.0);
        if (!(cell > 0))
        {
            cell = 1;
        }
        _gridMinX = minX;
        _gridMinY = minY;
        _cellSize = cell;
        _gridWidth = (long)Math.Floor((maxX - minX) / cell) + 3;
        for (int i = 0; i < n; i++)
        {
            _cellKeys[i] = (CellKey(_agents[i].Position) << 22) | (long)i;
        }
        KeySort.Sort(_cellKeys, n);
    }

    // One cell of padding on each side keeps the 3x3 neighbourhood keys non-negative.
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private long CellKey(Vector2D p)
    {
        long cx = (long)Math.Floor((p.X - _gridMinX) / _cellSize) + 1;
        long cy = (long)Math.Floor((p.Y - _gridMinY) / _cellSize) + 1;
        return (cy * _gridWidth) + cx;
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void ComputeRange(int from, int to, double dt, Worker w)
    {
        for (int i = from; i < to; i++)
        {
            _newVelocity[i] = ComputeVelocity(i, dt, w);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private Vector2D ComputeVelocity(int i, double dt, Worker w)
    {
        ref readonly OrcaAgent a = ref _agents[i];

        // Obstacles within reach, nearest first (ties by obstacle index).
        double reach = (a.ObstacleTimeHorizon * a.MaxSpeed) + a.Radius;
        double reach2 = reach * reach;
        w.ObstacleCount = 0;
        for (int o = 0; o < _obstacleCount; o++)
        {
            Vector2D sa = _obstacles[o].A - a.Position;
            Vector2D sb = _obstacles[o].B - a.Position;
            double d2 = OrcaConstraints.SegmentDistanceSquared(sa, sb);
            if (d2 <= reach2)
            {
                w.InsertObstacle(d2, o);
            }
        }

        // Agents: the MaxNeighbors nearest within NeighborDistance (ties by agent index).
        w.NeighborCount = 0;
        if (a.MaxNeighbors > 0 && _agentCount > 1)
        {
            w.Prepare(a.MaxNeighbors);
            double range2 = a.NeighborDistance * a.NeighborDistance;
            ScanNeighbors(i, a.Position, range2, a.MaxNeighbors, w);
        }

        int total = w.ObstacleCount + w.NeighborCount;
        w.EnsureLines(total);
        int k = 0;
        for (int o = 0; o < w.ObstacleCount; o++)
        {
            OrcaSegment s = _obstacles[w.ObstacleIndex[o]];
            w.Lines[k++] = OrcaConstraints.ForSegment(s.A - a.Position, s.B - a.Position, a.Radius, a.ObstacleTimeHorizon, dt);
        }
        for (int m = 0; m < w.NeighborCount; m++)
        {
            ref readonly OrcaAgent b = ref _agents[w.NeighborIndex[m]];
            w.Lines[k++] = OrcaConstraints.ForAgentUnchecked(b.Position - a.Position, a.Velocity, b.Velocity, a.Radius + b.Radius, a.TimeHorizon, dt, 0.5);
        }
        return OrcaLinearProgram.Solve(w.Lines.AsSpan(0, total), w.ObstacleCount, a.MaxSpeed, a.PreferredVelocity, w.Scratch, out _);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private void ScanNeighbors(int self, Vector2D p, double range2, int max, Worker w)
    {
        long baseKey = CellKey(p);
        long[] keys = _cellKeys;
        int n = _agentCount;
        for (int row = -1; row <= 1; row++)
        {
            long first = baseKey + (row * _gridWidth) - 1;
            long last = first + 2;
            int pos = LowerBound(keys, n, first << 22);
            for (; pos < n && (keys[pos] >> 22) <= last; pos++)
            {
                int j = (int)(keys[pos] & ((1 << 22) - 1));
                if (j == self)
                {
                    continue;
                }
                Vector2D d = _agents[j].Position - p;
                double d2 = Vector2D.Dot(d, d);
                if (d2 < range2)
                {
                    w.InsertNeighbor(d2, j, max);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static int LowerBound(long[] keys, int count, long value)
    {
        int lo = 0, hi = count;
        while (lo < hi)
        {
            int mid = (lo + hi) >> 1;
            if (keys[mid] < value)
            {
                lo = mid + 1;
            }
            else
            {
                hi = mid;
            }
        }
        return lo;
    }

    // Per-worker buffers; grow only.
    private sealed class Worker
    {
        public HalfPlane[] Lines = new HalfPlane[16];
        public HalfPlane[] ScratchLines = new HalfPlane[16];
        public double[] NeighborDist = new double[16];
        public int[] NeighborIndex = new int[16];
        public double[] ObstacleDist = new double[8];
        public int[] ObstacleIndex = new int[8];
        public int NeighborCount;
        public int ObstacleCount;

        public Span<HalfPlane> Scratch => ScratchLines;

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public void EnsureLines(int total)
        {
            if (Lines.Length < total)
            {
                int size = Math.Max(total, Lines.Length * 2);
                Lines = new HalfPlane[size];
                ScratchLines = new HalfPlane[size];
            }
        }

        public void Prepare(int max)
        {
            if (NeighborDist.Length < max)
            {
                NeighborDist = new double[max];
                NeighborIndex = new int[max];
            }
        }

        // Bounded insertion keeping (distance, index) ascending.
        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public void InsertNeighbor(double d2, int index, int max)
        {
            int pos = NeighborCount;
            if (pos == max)
            {
                // Full: the candidate must be strictly before the current last entry.
                int last = pos - 1;
                if (!(d2 < NeighborDist[last] || (d2 == NeighborDist[last] && index < NeighborIndex[last])))
                {
                    return;
                }
                pos = last;
            }
            else
            {
                NeighborCount++;
            }
            while (pos > 0 && (d2 < NeighborDist[pos - 1] || (d2 == NeighborDist[pos - 1] && index < NeighborIndex[pos - 1])))
            {
                NeighborDist[pos] = NeighborDist[pos - 1];
                NeighborIndex[pos] = NeighborIndex[pos - 1];
                pos--;
            }
            NeighborDist[pos] = d2;
            NeighborIndex[pos] = index;
        }

        [MethodImpl(MethodImplOptions.AggressiveOptimization)]
        public void InsertObstacle(double d2, int index)
        {
            if (ObstacleCount == ObstacleDist.Length)
            {
                Array.Resize(ref ObstacleDist, ObstacleCount * 2);
                Array.Resize(ref ObstacleIndex, ObstacleCount * 2);
            }
            int pos = ObstacleCount++;
            while (pos > 0 && d2 < ObstacleDist[pos - 1])
            {
                ObstacleDist[pos] = ObstacleDist[pos - 1];
                ObstacleIndex[pos] = ObstacleIndex[pos - 1];
                pos--;
            }
            ObstacleDist[pos] = d2;
            ObstacleIndex[pos] = index;
        }
    }
}
