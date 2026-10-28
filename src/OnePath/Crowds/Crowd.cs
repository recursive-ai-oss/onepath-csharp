using System;
using System.Collections.Generic;
using System.Threading.Tasks;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Crowds.Orca;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>
/// A crowd of <see cref="Agent"/>s walking one mesh. Each step, every agent keeps its path corridor valid, picks the velocity it
/// would like (toward the next corner of its path, slowing to stop at the goal, pushed apart from close neighbours when
/// separation is on), then solves for the velocity nearest to it that avoids the wall edges and neighbouring agents around it (ORCA,
/// van den Berg, Guy, Lin and Manocha 2011, with the navmesh boundary as static obstacles), and moves along the mesh surface by it.
/// <para>
/// A step needs only the time step: there is no clock, no hidden random number and no sub-stepping. The result depends only on the
/// mesh, the agents' states and settings and the order of calls, never on <see cref="CrowdOptions.MaxThreads"/>. Agents are always
/// handled in slot order (the lowest free slot is reused when one is added), and an agent's step reads only the state the step began
/// with of the others. A crowd is not thread-safe; its state is plain data (see <c>docs/crowds.md</c>).
/// </para>
/// </summary>
public sealed class Crowd
{
    private const int ChunkSize = 32;
    private const int MaxNeighborSlots = 12;
    private const byte FlagActive = 1, FlagWalking = 2, FlagCrossing = 4;

    private readonly TiledNavMesh _mesh;
    private readonly CrowdOptions _opt;
    private readonly int _wallCap;
    private readonly int _debugCap;
    private readonly ParallelOptions _parallel;
    private readonly Action<int> _body;
    private Worker[] _workers;

    private Agent?[] _agents = new Agent?[16];
    private PathFollower?[] _follow = new PathFollower?[16];
    private AgentOptions?[] _agentOpt = new AgentOptions?[16];
    private double[] _priority = new double[16];
    private byte[] _flags = new byte[16];
    private double[] _py = new double[16];
    private double[] _px = new double[16], _pz = new double[16], _vx = new double[16], _vz = new double[16];
    private double[] _prefX = new double[16], _prefZ = new double[16], _outX = new double[16], _outZ = new double[16];
    private WallSegment[] _walls;
    private int[] _wallCount = new int[16];
    private double[] _wallCx = new double[16], _wallCz = new double[16];
    private long[] _wallStamp = new long[16];
    private HalfPlane[]? _debugLines;
    private int[] _debugCount = new int[16];
    private long[] _keys = new long[16];
    private int _slotCount;
    private int _count;
    private int _lowestFree;
    private long _nextId;
    private long _steps;

    // The neighbour grid of the current step.
    private double _gridMinX, _gridMinZ, _cellSize = 1;
    private long _gridWidth = 1;
    private int _keyCount;
    private double _dt;
    private int _phase;
    private int _chunks;
    private int _threads;

    /// <summary>Creates an empty crowd over a mesh (tier 2 entry; <see cref="NavMesh.CreateCrowd"/> is the usual way).</summary>
    /// <param name="mesh">The tiled mesh. It may be edited between steps; agents repair their paths and walls.</param>
    /// <param name="options">Crowd settings; null means <see cref="CrowdOptions.Default"/>.</param>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A setting is out of range (the message names it).</exception>
    public Crowd(TiledNavMesh mesh, CrowdOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        _opt = options ?? CrowdOptions.Default;
        _opt.Validate();
        _mesh = mesh;
        _wallCap = _opt.MaxWallsPerAgent;
        _debugCap = _opt.MaxWallsAvoided + MaxNeighborSlots;
        _walls = new WallSegment[16 * _wallCap];
        _workers = new Worker[_opt.MaxThreads];
        _parallel = new ParallelOptions { MaxDegreeOfParallelism = _opt.MaxThreads };
        _body = RunWorker;
        if (_opt.RecordConstraints)
        {
            _debugLines = new HalfPlane[16 * _debugCap];
        }
    }

    /// <summary>The mesh agents walk.</summary>
    public TiledNavMesh Mesh => _mesh;

    /// <summary>The settings.</summary>
    public CrowdOptions Options => _opt;

    /// <summary>Number of agents in the crowd.</summary>
    public int Count => _count;

    /// <summary>Steps taken so far.</summary>
    public long StepCount => _steps;

    /// <summary>A query object over the mesh for the host's own queries between steps (not safe to use during <see cref="Step"/>).</summary>
    public NavMeshQuery Query => GetWorker(0).Query;

    /// <summary>
    /// Adds an agent at the walkable point nearest to <paramref name="position"/> (within the search extents of its path settings). It
    /// stands still with no destination.
    /// </summary>
    /// <param name="position">World position.</param>
    /// <param name="options">Agent settings; null means <see cref="Agent.Humanoid"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is not finite or is beyond ±2^21, or a setting is out of range.</exception>
    /// <exception cref="ArgumentException">No walkable polygon is near the position.</exception>
    /// <exception cref="InvalidOperationException">The crowd holds <see cref="CrowdOptions.MaxAgents"/> agents.</exception>
    public Agent Add(Vector3D position, AgentOptions? options = null)
    {
        options ??= Agent.Humanoid;
        options.Validate();
        NavMeshQuery.CheckPos(position, nameof(position));
        if (_count >= _opt.MaxAgents)
        {
            throw new InvalidOperationException($"The crowd already holds {_opt.MaxAgents} agents.");
        }
        var follower = new PathFollower(options.ToFollowerOptions());
        if (!follower.Place(GetWorker(0).Query, position, _opt.Filter))
        {
            throw new ArgumentException($"No walkable polygon near ({position.X}, {position.Y}, {position.Z}).", nameof(position));
        }
        int slot = _lowestFree;
        while (slot < _slotCount && (_flags[slot] & FlagActive) != 0)
        {
            slot++;
        }
        if (slot == _slotCount)
        {
            EnsureSlots(slot + 1);
            _slotCount++;
        }
        _lowestFree = slot + 1;
        var agent = new Agent(this, slot, _nextId++, options);
        _agents[slot] = agent;
        _follow[slot] = follower;
        _agentOpt[slot] = options;
        _priority[slot] = options.Priority;
        _flags[slot] = FlagActive;
        _wallStamp[slot] = -1;
        _wallCount[slot] = 0;
        _count++;
        return agent;
    }

    /// <summary>Removes an agent. Its slot is reused by the next agent added.</summary>
    /// <returns>False if the agent is not in this crowd.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="agent"/> is null.</exception>
    public bool Remove(Agent agent)
    {
        ArgumentNullException.ThrowIfNull(agent);
        int slot = agent.Slot;
        if (slot < 0 || slot >= _slotCount || !ReferenceEquals(_agents[slot], agent))
        {
            return false;
        }
        agent.Deactivate();
        _agents[slot] = null;
        _follow[slot] = null;
        _agentOpt[slot] = null;
        _flags[slot] = 0;
        _count--;
        if (slot < _lowestFree)
        {
            _lowestFree = slot;
        }
        return true;
    }

    /// <summary>Appends the agents to <paramref name="destination"/> in slot order (the order a step handles them).</summary>
    /// <exception cref="ArgumentNullException"><paramref name="destination"/> is null.</exception>
    public void CopyAgents(List<Agent> destination)
    {
        ArgumentNullException.ThrowIfNull(destination);
        for (int i = 0; i < _slotCount; i++)
        {
            if (_agents[i] is { } a)
            {
                destination.Add(a);
            }
        }
    }

    /// <summary>
    /// Advances every agent by <paramref name="dt"/> seconds. Allocation-free once the agents and meshes are in place.
    /// </summary>
    /// <param name="dt">The time step: positive and finite. The host fixes it (for example 1/30); nothing here reads a clock.</param>
    /// <exception cref="ArgumentOutOfRangeException">dt is not positive and finite.</exception>
    public void Step(double dt)
    {
        if (!(dt > 0) || !double.IsFinite(dt))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be positive and finite.");
        }
        _steps++;
        if (_count == 0)
        {
            return;
        }
        _dt = dt;
        BuildGrid();
        _chunks = (_slotCount + ChunkSize - 1) / ChunkSize;
        _threads = Math.Min(_opt.MaxThreads, _chunks);
        for (int phase = 1; phase <= 3; phase++)
        {
            _phase = phase;
            if (_threads <= 1)
            {
                RunWorker(0);
            }
            else
            {
                // Worker w takes chunks w, w + threads, ... An agent's work touches only its own slot (and reads state frozen
                // for the phase), so the result does not depend on the number of workers.
                Parallel.For(0, _threads, _parallel, _body);
            }
        }
    }

    // ---- agent operations (called by Agent) ----

    internal PathFollower FollowerAt(int slot) => _follow[slot]!;

    internal double PriorityAt(int slot) => _priority[slot];

    internal void SetPriorityAt(int slot, double value) => _priority[slot] = value;

    internal PathStatus SetDestinationAt(int slot, Vector3D destination) => _follow[slot]!.SetDestination(GetWorker(0).Query, destination, _opt.Filter);

    internal bool TeleportAt(int slot, Vector3D position)
    {
        PathFollower f = _follow[slot]!;
        if (!f.Place(GetWorker(0).Query, position, _opt.Filter))
        {
            return false;
        }
        _wallStamp[slot] = -1;
        return true;
    }

    internal void CompleteOffMeshAt(int slot) => _follow[slot]!.CompleteOffMeshLink(GetWorker(0).Query, _opt.Filter);

    internal int SlotCount => _slotCount;

    internal AgentOptions? OptionsAt(int slot) => _agentOpt[slot];

    internal (double X, double Z) PreferredAt(int slot) => (_prefX[slot], _prefZ[slot]);

    internal bool IsCrossingAt(int slot) => (_flags[slot] & FlagCrossing) != 0;

    internal Agent? AgentAt(int slot) => _agents[slot];

    internal ReadOnlySpan<HalfPlane> ConstraintsAt(int slot) =>
        _debugLines is null ? default : _debugLines.AsSpan(slot * _debugCap, _debugCount[slot]);

    internal ReadOnlySpan<WallSegment> WallsAt(int slot) => _walls.AsSpan(slot * _wallCap, _wallCount[slot]);

    // ---- storage ----

    private Worker GetWorker(int w) => _workers[w] ??= new Worker(new NavMeshQuery(_mesh, _opt.QueryNodes), _opt.MaxWallsAvoided);

    private void EnsureSlots(int needed)
    {
        int cap = _agents.Length;
        if (needed <= cap)
        {
            return;
        }
        int size = cap;
        while (size < needed)
        {
            size *= 2;
        }
        Array.Resize(ref _agents, size);
        Array.Resize(ref _follow, size);
        Array.Resize(ref _agentOpt, size);
        Array.Resize(ref _priority, size);
        Array.Resize(ref _flags, size);
        Array.Resize(ref _py, size);
        Array.Resize(ref _px, size);
        Array.Resize(ref _pz, size);
        Array.Resize(ref _vx, size);
        Array.Resize(ref _vz, size);
        Array.Resize(ref _prefX, size);
        Array.Resize(ref _prefZ, size);
        Array.Resize(ref _outX, size);
        Array.Resize(ref _outZ, size);
        Array.Resize(ref _wallCount, size);
        Array.Resize(ref _wallCx, size);
        Array.Resize(ref _wallCz, size);
        Array.Resize(ref _wallStamp, size);
        Array.Resize(ref _debugCount, size);
        Array.Resize(ref _keys, size);
        Array.Resize(ref _walls, size * _wallCap);
        if (_debugLines is not null)
        {
            Array.Resize(ref _debugLines, size * _debugCap);
        }
    }

    // ---- the step ----

    private void RunWorker(int w)
    {
        Worker worker = GetWorker(w);
        int step = Math.Max(_threads, 1);
        for (int chunk = w; chunk < _chunks; chunk += step)
        {
            int from = chunk * ChunkSize;
            int to = Math.Min(_slotCount, from + ChunkSize);
            for (int i = from; i < to; i++)
            {
                if ((_flags[i] & FlagActive) == 0)
                {
                    continue;
                }
                switch (_phase)
                {
                    case 1:
                        Plan(i, worker);
                        break;
                    case 2:
                        Avoid(i, worker);
                        break;
                    default:
                        Move(i, worker);
                        break;
                }
            }
        }
    }

    // Snapshot of positions and velocities (what every agent sees of the others) and the neighbour grid, sorted by (cell, slot).
    private void BuildGrid()
    {
        double minX = double.PositiveInfinity, minZ = double.PositiveInfinity, maxX = double.NegativeInfinity, maxZ = double.NegativeInfinity;
        double cell = 0;
        int k = 0;
        for (int i = 0; i < _slotCount; i++)
        {
            if ((_flags[i] & FlagActive) == 0)
            {
                continue;
            }
            PathFollower f = _follow[i]!;
            Vector3D p = f.Position, v = f.Velocity;
            _px[i] = p.X;
            _py[i] = p.Y;
            _pz[i] = p.Z;
            _vx[i] = v.X;
            _vz[i] = v.Z;
            minX = Math.Min(minX, p.X);
            minZ = Math.Min(minZ, p.Z);
            maxX = Math.Max(maxX, p.X);
            maxZ = Math.Max(maxZ, p.Z);
            cell = Math.Max(cell, NeighborDistance(_agentOpt[i]!));
            _keys[k++] = i;
        }
        _keyCount = k;
        double extent = Math.Max(maxX - minX, maxZ - minZ);
        cell = Math.Max(cell, extent / 1048576.0);
        if (!(cell > 0))
        {
            cell = 1;
        }
        _gridMinX = minX;
        _gridMinZ = minZ;
        _cellSize = cell;
        _gridWidth = (long)Math.Floor((maxX - minX) / cell) + 3;
        for (int n = 0; n < k; n++)
        {
            int slot = (int)_keys[n];
            _keys[n] = (CellKey(_px[slot], _pz[slot]) << 22) | (long)slot;
        }
        Array.Sort(_keys, 0, k);
    }

    private long CellKey(double x, double z)
    {
        long cx = (long)Math.Floor((x - _gridMinX) / _cellSize) + 1;
        long cz = (long)Math.Floor((z - _gridMinZ) / _cellSize) + 1;
        return (cz * _gridWidth) + cx;
    }

    private static double NeighborDistance(AgentOptions o) =>
        o.Quality == AvoidanceQuality.Off ? 0 : Math.Max(o.SeparationDistance, (2 * o.Radius) + (o.MaxSpeed * o.TimeHorizon));

    private static double ObstacleHorizon(AgentOptions o) => 0.5 * o.TimeHorizon;

    // Phase 1: corridor upkeep, wanted velocity, wall lookup.
    private void Plan(int i, Worker w)
    {
        PathFollower f = _follow[i]!;
        AgentOptions o = _agentOpt[i]!;
        QueryFilter? filter = _opt.Filter;
        if (f.State == FollowerState.Arrived)
        {
            // Pushed away from the goal by others: go back (the corridor is short, so this is cheap).
            Vector3D p = f.Position, d = f.Destination;
            double dx = p.X - d.X, dz = p.Z - d.Z;
            double limit = (2 * o.ArrivalRadius) + 0.01;
            if ((dx * dx) + (dz * dz) > limit * limit)
            {
                f.SetDestination(w.Query, d, filter);
            }
        }
        bool walking = f.PlanCrowd(w.Query, _dt, filter, out double wx, out double wz);
        byte flags = FlagActive;
        if (walking)
        {
            flags |= FlagWalking;
        }
        else
        {
            wx = wz = 0;
        }
        if (f.State == FollowerState.OffMeshLink)
        {
            flags |= FlagCrossing;
        }
        _flags[i] = flags;
        _prefX[i] = wx;
        _prefZ[i] = wz;
        if (o.Quality != AvoidanceQuality.Off && (flags & FlagCrossing) == 0)
        {
            RefreshWalls(i, f, o, w);
        }
    }

    private void RefreshWalls(int i, PathFollower f, AgentOptions o, Worker w)
    {
        Vector3D p = f.Position;
        double dx = p.X - _wallCx[i], dz = p.Z - _wallCz[i];
        double refresh = _opt.WallRefreshDistance;
        if (_wallStamp[i] == _mesh.EditCount && (dx * dx) + (dz * dz) <= refresh * refresh)
        {
            return;
        }
        double radius = (ObstacleHorizon(o) * o.MaxSpeed) + o.WallClearance + refresh;
        Span<WallSegment> span = _walls.AsSpan(i * _wallCap, _wallCap);
        AreaQueryStatus s = w.Query.FindWalls(f.Corridor.FirstPoly, p, radius, _opt.Filter, span, out int count, out _);
        if (s != AreaQueryStatus.Success)
        {
            _wallCount[i] = 0;
            _wallStamp[i] = -1;
            return;
        }
        _wallCount[i] = count;
        _wallCx[i] = p.X;
        _wallCz[i] = p.Z;
        _wallStamp[i] = _mesh.EditCount;
    }

    // Phase 2: ORCA against the walls and neighbours seen at the start of the step.
    private void Avoid(int i, Worker w)
    {
        AgentOptions o = _agentOpt[i]!;
        byte flags = _flags[i];
        double prefX = _prefX[i], prefZ = _prefZ[i];
        _debugCount[i] = 0;
        if ((flags & FlagCrossing) != 0 || o.Quality == AvoidanceQuality.Off)
        {
            _outX[i] = prefX;
            _outZ[i] = prefZ;
            return;
        }
        double posX = _px[i], posZ = _pz[i];
        int maxNb = o.MaxNeighbors;
        double range = NeighborDistance(o);
        w.NeighborCount = 0;
        ScanNeighbors(i, posX, posZ, range * range, maxNb, w);

        // Separation pushes the wanted velocity away from close neighbours.
        if (o.SeparationDistance > 0 && o.SeparationWeight > 0)
        {
            double sx = 0, sz = 0;
            for (int m = 0; m < w.NeighborCount; m++)
            {
                int j = w.NeighborIndex[m];
                double dx = posX - _px[j], dz = posZ - _pz[j];
                double d = Math.Sqrt((dx * dx) + (dz * dz));
                if (d >= o.SeparationDistance)
                {
                    continue;
                }
                double nx, nz;
                if (d > 1e-9)
                {
                    nx = dx / d;
                    nz = dz / d;
                }
                else
                {
                    nx = j < i ? 1 : -1;
                    nz = 0;
                }
                double t = 1.0 - (d / o.SeparationDistance);
                double weight = o.SeparationWeight * o.MaxSpeed * t * t;
                sx += nx * weight;
                sz += nz * weight;
            }
            prefX += sx;
            prefZ += sz;
            double len2 = (prefX * prefX) + (prefZ * prefZ);
            if (len2 > o.MaxSpeed * o.MaxSpeed)
            {
                double k = o.MaxSpeed / Math.Sqrt(len2);
                prefX *= k;
                prefZ *= k;
            }
        }

        // Walls: the nearest few of the cached pieces within reach.
        double tau = o.TimeHorizon;
        double obstTau = ObstacleHorizon(o);
        double reach = (obstTau * o.MaxSpeed) + o.WallClearance;
        double reach2 = reach * reach;
        w.WallCount = 0;
        int cached = _wallCount[i];
        int baseIdx = i * _wallCap;
        for (int k = 0; k < cached; k++)
        {
            WallSegment s = _walls[baseIdx + k];
            var wa = new Vector2D(s.AX - posX, s.AZ - posZ);
            var wb = new Vector2D(s.BX - posX, s.BZ - posZ);
            double d2 = OrcaConstraints.SegmentDistanceSquared(wa, wb);
            if (d2 <= reach2 && SameFloor(s, _py[i], posX, posZ, d2) && OrcaConstraints.ForBoundary(wa, wb, o.WallClearance, obstTau, out HalfPlane line))
            {
                w.InsertWall(d2, s, line);
            }
        }
        // A sideways bias breaks the symmetry of head-on meetings: toward the right of the heading, unless a wall within four radii
        // lies on that side (then to the left, where there is room).
        if (o.SidePreference > 0)
        {
            double len = Math.Sqrt((prefX * prefX) + (prefZ * prefZ));
            double sign = 1;
            if (len > 0)
            {
                double rxh = prefZ / len, rzh = -prefX / len;
                for (int k = 0; k < w.WallCount; k++)
                {
                    HalfPlane h = w.WallLine[k];
                    double dist = o.WallClearance - (Vector2D.Dot(h.Point, h.Normal) * obstTau);
                    if (dist < 4 * o.Radius && (h.Normal.X * rxh) + (h.Normal.Y * rzh) < -0.5)
                    {
                        sign = -1;
                        break;
                    }
                }
            }
            double bx = prefZ * o.SidePreference * sign, bz = -prefX * o.SidePreference * sign;
            prefX += bx;
            prefZ += bz;
        }
        int total = w.WallCount + w.NeighborCount;
        w.EnsureLines(total);
        int n = 0;
        for (int k = 0; k < w.WallCount; k++)
        {
            w.Lines[n++] = w.WallLine[k];
        }
        var vel = new Vector2D(_vx[i], _vz[i]);
        double pri = _priority[i];
        for (int m = 0; m < w.NeighborCount; m++)
        {
            int j = w.NeighborIndex[m];
            AgentOptions q = _agentOpt[j]!;
            // Share of the evasion: the other's weight over both; all of it when the other is crossing a link and cannot give way.
            double share = (_flags[j] & FlagCrossing) != 0 ? 1.0 : _priority[j] / (pri + _priority[j]);
            double combined = o.Radius + q.Radius;
            // Two agents at exactly the same place have no direction between them: the one with the higher slot is taken to be a little to the side.
            double rx = _px[j] - posX, rz = _pz[j] - posZ;
            if (rx == 0 && rz == 0)
            {
                rx = j > i ? 1e-3 : -1e-3;
            }
            w.Lines[n++] = OrcaConstraints.ForAgent(new Vector2D(rx, rz), vel, new Vector2D(_vx[j], _vz[j]), combined, tau, _dt, share);
        }
        ReadOnlySpan<HalfPlane> lines = w.Lines.AsSpan(0, total);
        var pref = new Vector2D(prefX, prefZ);
        Vector2D result = OrcaLinearProgram.Solve(lines, w.WallCount, o.MaxSpeed, pref, w.Scratch, out _);
        // Blocked by an agent that stands still (arrived or idle) straight ahead: the closest-velocity rule has no sideways escape, so the
        // solution is far slower than wanted. Try the wanted velocity turned 90 and 45 degrees to either side and take the fastest outcome.
        double wantedLen2 = Vector2D.Dot(pref, pref);
        if (wantedLen2 > 1e-4 && Vector2D.Dot(result, result) < 0.09 * wantedLen2 && FarFromGoal(i, o) && HasStandingNeighbor(w))
        {
            double best = Vector2D.Dot(result, result);
            Vector2D bestV = result;
            for (int k = 0; k < 4; k++)
            {
                // k: +90, -90, +45, -45 degrees; 0.7071067811865476 is the double nearest to sqrt(1/2).
                Vector2D turned = k switch
                {
                    0 => new Vector2D(-prefZ, prefX),
                    1 => new Vector2D(prefZ, -prefX),
                    2 => new Vector2D(0.7071067811865476 * (prefX - prefZ), 0.7071067811865476 * (prefX + prefZ)),
                    _ => new Vector2D(0.7071067811865476 * (prefX + prefZ), 0.7071067811865476 * (prefZ - prefX)),
                };
                Vector2D alt = OrcaLinearProgram.Solve(lines, w.WallCount, o.MaxSpeed, turned, w.Scratch, out _);
                double speed2 = Vector2D.Dot(alt, alt);
                if (speed2 > best)
                {
                    best = speed2;
                    bestV = alt;
                }
            }
            result = bestV;
        }
        _outX[i] = result.X;
        _outZ[i] = result.Y;
        if (_debugLines is not null)
        {
            int cnt = Math.Min(total, _debugCap);
            Array.Copy(w.Lines, 0, _debugLines, i * _debugCap, cnt);
            _debugCount[i] = cnt;
        }
        _prefX[i] = prefX;
        _prefZ[i] = prefZ;
    }

    // Standing agents near the goal are the queue the agent joins, not an obstacle to go around: only agents more than eight radii
    // from their destination (straight-line) look for a way around.
    private bool FarFromGoal(int i, AgentOptions o)
    {
        Vector3D d = _follow[i]!.Destination;
        double dx = d.X - _px[i], dz = d.Z - _pz[i];
        double far = 8 * o.Radius;
        return (dx * dx) + (dz * dz) > far * far;
    }

    // Walls of another floor (polygons stacked above or below, joined by a link) are not this agent's walls: compare the height of the
    // wall at its nearest point with the agent's, allowing for slopes.
    private static bool SameFloor(WallSegment s, double y, double x, double z, double d2)
    {
        double ex = s.BX - s.AX, ez = s.BZ - s.AZ;
        double len2 = (ex * ex) + (ez * ez);
        double u = len2 > 0 ? Math.Clamp((((x - s.AX) * ex) + ((z - s.AZ) * ez)) / len2, 0.0, 1.0) : 0.0;
        double wy = s.AY + ((s.BY - s.AY) * u);
        return Math.Abs(wy - y) <= 1.0 + (1.5 * Math.Sqrt(d2));
    }

    // True if one of the neighbours of the current avoidance problem is neither walking nor crossing a link.
    private bool HasStandingNeighbor(Worker w)
    {
        for (int m = 0; m < w.NeighborCount; m++)
        {
            if ((_flags[w.NeighborIndex[m]] & (FlagWalking | FlagCrossing)) == 0)
            {
                return true;
            }
        }
        return false;
    }

    // Phase 3: limit the change of velocity and move along the surface.
    private void Move(int i, Worker w)
    {
        byte flags = _flags[i];
        if ((flags & FlagCrossing) != 0)
        {
            return;
        }
        AgentOptions o = _agentOpt[i]!;
        PathFollower f = _follow[i]!;
        double ox = _outX[i], oz = _outZ[i];
        double ax = ox - _vx[i], az = oz - _vz[i];
        double alen = Math.Sqrt((ax * ax) + (az * az));
        double maxDv = o.MaxAcceleration * _dt;
        double vx = _vx[i], vz = _vz[i];
        if (alen > maxDv)
        {
            double k = maxDv / alen;
            vx += ax * k;
            vz += az * k;
        }
        else
        {
            vx = ox;
            vz = oz;
        }
        if ((flags & FlagWalking) != 0 && f.State == FollowerState.Moving)
        {
            f.ApplyCrowd(w.Query, _dt, _opt.Filter, vx, vz);
        }
        else
        {
            f.Nudge(w.Query, _dt, _opt.Filter, vx, vz);
        }
    }

    // The MaxNeighbors nearest agents within range (ties by slot), from the 3 x 3 cells around the agent.
    private void ScanNeighbors(int self, double x, double z, double range2, int max, Worker w)
    {
        if (max <= 0 || _keyCount < 2)
        {
            return;
        }
        long baseKey = CellKey(x, z);
        long[] keys = _keys;
        int n = _keyCount;
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
                double dx = _px[j] - x, dz = _pz[j] - z;
                double d2 = (dx * dx) + (dz * dz);
                if (d2 < range2)
                {
                    w.InsertNeighbor(d2, j, max);
                }
            }
        }
    }

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

    // Per-worker buffers: a query object and the lists of one agent's avoidance problem.
    private sealed class Worker
    {
        private readonly double[] _wallDist;
        private readonly int _wallMax;
        private double[] _neighborDist = new double[MaxNeighborSlots];

        public Worker(NavMeshQuery query, int maxWalls)
        {
            Query = query;
            _wallMax = maxWalls;
            _wallDist = new double[maxWalls];
            WallPiece = new WallSegment[maxWalls];
            WallLine = new HalfPlane[maxWalls];
        }

        public NavMeshQuery Query { get; }

        public HalfPlane[] Lines = new HalfPlane[16];
        public HalfPlane[] ScratchLines = new HalfPlane[16];
        public int[] NeighborIndex = new int[MaxNeighborSlots];
        public WallSegment[] WallPiece;
        public HalfPlane[] WallLine;
        public int NeighborCount;
        public int WallCount;

        public Span<HalfPlane> Scratch => ScratchLines;

        public void EnsureLines(int total)
        {
            if (Lines.Length < total)
            {
                int size = Math.Max(total, Lines.Length * 2);
                Lines = new HalfPlane[size];
                ScratchLines = new HalfPlane[size];
            }
        }

        // Bounded insertion keeping (distance, slot) ascending.
        public void InsertNeighbor(double d2, int index, int max)
        {
            int pos = NeighborCount;
            if (pos == max)
            {
                int last = pos - 1;
                if (!(d2 < _neighborDist[last] || (d2 == _neighborDist[last] && index < NeighborIndex[last])))
                {
                    return;
                }
                pos = last;
            }
            else
            {
                NeighborCount++;
            }
            while (pos > 0 && (d2 < _neighborDist[pos - 1] || (d2 == _neighborDist[pos - 1] && index < NeighborIndex[pos - 1])))
            {
                _neighborDist[pos] = _neighborDist[pos - 1];
                NeighborIndex[pos] = NeighborIndex[pos - 1];
                pos--;
            }
            _neighborDist[pos] = d2;
            NeighborIndex[pos] = index;
        }

        // Keeps the nearest pieces ordered by (distance, polygon, edge, ends): the order does not depend on how the cache was filled.
        public void InsertWall(double d2, WallSegment s, HalfPlane line)
        {
            int pos = WallCount;
            if (pos == _wallMax)
            {
                if (!Before(d2, s, _wallDist[pos - 1], WallPiece[pos - 1]))
                {
                    return;
                }
                pos--;
            }
            else
            {
                WallCount++;
            }
            while (pos > 0 && Before(d2, s, _wallDist[pos - 1], WallPiece[pos - 1]))
            {
                _wallDist[pos] = _wallDist[pos - 1];
                WallPiece[pos] = WallPiece[pos - 1];
                WallLine[pos] = WallLine[pos - 1];
                pos--;
            }
            _wallDist[pos] = d2;
            WallPiece[pos] = s;
            WallLine[pos] = line;
        }

        private static bool Before(double d2, WallSegment a, double e2, WallSegment b)
        {
            if (d2 != e2)
            {
                return d2 < e2;
            }
            if (a.Poly.Value != b.Poly.Value)
            {
                return a.Poly.Value < b.Poly.Value;
            }
            if (a.Edge != b.Edge)
            {
                return a.Edge < b.Edge;
            }
            if (a.AX != b.AX)
            {
                return a.AX < b.AX;
            }
            return a.AZ < b.AZ;
        }
    }
}
