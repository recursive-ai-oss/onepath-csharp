using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath.Crowds.Corridors;

/// <summary>What a <see cref="PathFollower"/> is doing.</summary>
public enum FollowerState : byte
{
    /// <summary>Standing on the mesh with no destination (or stopped).</summary>
    Idle = 0,

    /// <summary>Walking its corridor toward the destination (this includes waiting at the end of a corridor that a replan is still extending).</summary>
    Moving = 1,

    /// <summary>Crossing an off-mesh link: automatically over a fixed duration, or until the host calls <see cref="PathFollower.CompleteOffMeshLink"/>.</summary>
    OffMeshLink = 2,

    /// <summary>Stopped at the end of its corridor; <see cref="PathFollower.ReachedTarget"/> tells whether that is the destination.</summary>
    Arrived = 3,

    /// <summary>Gave up after making no progress through every allowed recovery.</summary>
    Stuck = 4,

    /// <summary>No route could be started or kept: no polygon near the destination, the agent was lost off the mesh, or replanning kept running out of budget.</summary>
    NoPath = 5,
}

/// <summary>
/// One agent walking a navigation mesh: a <see cref="PathCorridor"/>, a position and velocity, and the steering that moves
/// them along the corridor's corners with a fixed time step. There is no internal clock: every <see cref="Step"/> gets its dt
/// from the caller, and budgets count steps and search iterations, so the same calls give the same state on every platform.
/// The rules (steering, arrival, stuck detection, mesh-change handling, off-mesh links) are in <c>docs/crowds.md</c>.
/// <para>
/// Sequence: <see cref="Place"/> the agent on the mesh, <see cref="SetDestination"/>, then <see cref="Step"/> every tick.
/// Methods take the query object and filter to use (nothing is stored), so one query object can serve many followers. A
/// follower is not thread-safe. The state is plain data with a byte-exact form (<see cref="Write"/> and <see cref="Read"/>).
/// </para>
/// </summary>
public sealed class PathFollower
{
    private const byte FormatVersion = 1;
    private const double TinyDistance = 1e-9;
    private const int MaxHops = 4;

    private readonly FollowerOptions _opt;
    private readonly StraightPathPoint[] _corners;
    private PathCorridor _corr;
    private int _cornerCount;
    private FollowerState _state;
    private bool _reached;
    private Vector3D _position;
    private Vector3D _velocity;
    private Vector3D _destination;
    private double _remaining = double.PositiveInfinity;
    private long _steps;
    private int _rounds;
    private int _recoveries;
    private double _clock;
    private Vector3D _anchor;
    private Vector3D _offStart, _offEnd;
    private double _offElapsed, _offDuration;
    private PolyRef _offExitPoly;

    /// <summary>Creates a follower that is not on any mesh yet (call <see cref="Place"/>).</summary>
    /// <param name="options">Tuning; null means <see cref="FollowerOptions.Humanoid"/>.</param>
    /// <exception cref="ArgumentOutOfRangeException">An option is out of range (the message names it).</exception>
    public PathFollower(FollowerOptions? options = null)
    {
        _opt = options ?? FollowerOptions.Humanoid;
        _opt.Validate();
        _corr = new PathCorridor(_opt.CorridorCapacity);
        _corners = new StraightPathPoint[_opt.MaxCorners];
    }

    /// <summary>The options in use.</summary>
    public FollowerOptions Options => _opt;

    /// <summary>The corridor (raw access for tier-3 use and for drawing).</summary>
    public PathCorridor Corridor => _corr;

    /// <summary>What the agent is doing.</summary>
    public FollowerState State => _state;

    /// <summary>True while the agent has somewhere to go: walking or crossing an off-mesh link.</summary>
    public bool HasPath => _state is FollowerState.Moving or FollowerState.OffMeshLink;

    /// <summary>True if the agent stopped in <see cref="FollowerState.Arrived"/> at the destination itself (false at the end of a corridor that could not reach it).</summary>
    public bool ReachedTarget => _reached;

    /// <summary>Where the agent is (world coordinates, on the mesh surface).</summary>
    public Vector3D Position => _position;

    /// <summary>The velocity of the last step: the distance actually moved divided by dt (zero when stopped).</summary>
    public Vector3D Velocity => _velocity;

    /// <summary>The last destination given to <see cref="SetDestination"/>.</summary>
    public Vector3D Destination => _destination;

    /// <summary>
    /// Path distance still to walk to the end of the corridor (or the next off-mesh link), measured along the corners at the
    /// start of the last step minus what that step covered; positive infinity when the end is not within the corner buffer.
    /// </summary>
    public double RemainingDistance => _remaining;

    /// <summary>The corners computed by the last step (the first is the agent's position); empty before the first step. Debug data.</summary>
    public ReadOnlySpan<StraightPathPoint> Corners => _corners.AsSpan(0, _cornerCount);

    /// <summary>Where the current off-mesh crossing began.</summary>
    public Vector3D OffMeshStart => _offStart;

    /// <summary>Where the current off-mesh crossing ends.</summary>
    public Vector3D OffMeshEnd => _offEnd;

    /// <summary>Seconds of the automatic crossing done so far.</summary>
    public double OffMeshElapsed => _offElapsed;

    /// <summary>Seconds the automatic crossing takes.</summary>
    public double OffMeshDuration => _offDuration;

    /// <summary>Steps taken since <see cref="Place"/>.</summary>
    public long StepCount => _steps;

    /// <summary>
    /// Puts the agent on the mesh at the point of the nearest accepted polygon to <paramref name="position"/> (within
    /// <see cref="FollowerOptions.SearchExtents"/>), standing still with no destination.
    /// </summary>
    /// <returns>False, leaving the agent as it was, if no accepted polygon is near.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is not finite or is beyond ±2^21.</exception>
    public bool Place(NavMeshQuery query, Vector3D position, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        NearestPoly np = query.FindNearestPoly(position, _opt.SearchExtents, filter);
        if (!np.Poly.IsSet)
        {
            return false;
        }
        _corr.Reset(np.Poly, np.Point);
        _position = np.Point;
        _velocity = default;
        _destination = np.Point;
        _state = FollowerState.Idle;
        _reached = false;
        _remaining = double.PositiveInfinity;
        _cornerCount = 0;
        _steps = 0;
        ResetProgress();
        return true;
    }

    /// <summary>Stops the agent where it is (state <see cref="FollowerState.Idle"/>); a crossing in progress is abandoned at the entry.</summary>
    public void Stop()
    {
        if (_corr.Count > 0)
        {
            _corr.Reset(_corr.FirstPoly, _corr.Position);
            _position = _corr.Position;
        }
        _velocity = default;
        _state = FollowerState.Idle;
        _reached = false;
        _remaining = double.PositiveInfinity;
        ResetProgress();
    }

    /// <summary>
    /// Sends the agent to <paramref name="target"/>: finds the polygon nearest it and a route, in one replan call of
    /// <see cref="FollowerOptions.ReplanBudget"/> iterations (a route not found within it is finished by the following steps).
    /// During an off-mesh crossing the target is only recorded and the route is found after the crossing.
    /// </summary>
    /// <returns><see cref="PathStatus.Success"/> (full route), <see cref="PathStatus.Partial"/> (target unreachable: the agent will walk to the closest point), <see cref="PathStatus.InProgress"/> (route being extended), or the reason the agent cannot go (<see cref="PathStatus.InvalidStartPoly"/> when not placed, <see cref="PathStatus.NoEndNearby"/>); then the state is <see cref="FollowerState.NoPath"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The target is not finite or is beyond ±2^21.</exception>
    public PathStatus SetDestination(NavMeshQuery query, Vector3D target, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        NavMeshQuery.CheckPos(target, nameof(target));
        if (_corr.Count == 0)
        {
            return PathStatus.InvalidStartPoly;
        }
        _destination = target;
        _corr.SetTarget(target);
        _reached = false;
        _rounds = 0;
        _recoveries = 0;
        ResetProgress();
        if (_state == FollowerState.OffMeshLink)
        {
            return PathStatus.InProgress;
        }
        _state = FollowerState.Moving;
        PathStatus s = ReplanRound(query, filter);
        if (s is PathStatus.InvalidStartPoly or PathStatus.NoEndNearby)
        {
            _state = FollowerState.NoPath;
            _velocity = default;
        }
        return s;
    }

    /// <summary>
    /// Advances the agent by <paramref name="dt"/> seconds: checks the corridor against the mesh (repairing it if tiles or
    /// polygons changed), tries shortcuts on their schedule, finds the corners and steers along them.
    /// </summary>
    /// <param name="query">The query object (not stored).</param>
    /// <param name="dt">The time step, positive and finite. Fixed by the host; nothing here reads a clock.</param>
    /// <param name="filter">Polygon filter and area costs (null accepts everything).</param>
    /// <exception cref="ArgumentOutOfRangeException">dt is not positive and finite.</exception>
    public void Step(NavMeshQuery query, double dt, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        if (!(dt > 0) || !double.IsFinite(dt))
        {
            throw new ArgumentOutOfRangeException(nameof(dt), dt, "dt must be positive and finite.");
        }
        switch (_state)
        {
            case FollowerState.OffMeshLink:
                StepOffMesh(query, dt, filter);
                return;
            case FollowerState.Moving:
                StepMoving(query, dt, filter);
                return;
            default:
                _velocity = default;
                return;
        }
    }

    /// <summary>
    /// Ends the off-mesh crossing the host is driving (or cuts an automatic one short): the agent is put at the link's exit
    /// and walks on. If the exit polygon is no longer part of the corridor the agent is relocated to the mesh there.
    /// </summary>
    /// <exception cref="InvalidOperationException">The follower is not crossing an off-mesh link.</exception>
    public void CompleteOffMeshLink(NavMeshQuery query, QueryFilter? filter = null)
    {
        ArgumentNullException.ThrowIfNull(query);
        if (_state != FollowerState.OffMeshLink)
        {
            throw new InvalidOperationException("The follower is not crossing an off-mesh link.");
        }
        FinishOffMesh(query, filter);
    }

    // --- stepping ---

    private void StepMoving(NavMeshQuery query, double dt, QueryFilter? filter)
    {
        _steps++;
        Vector3D start = _position;
        // 1. The mesh may have changed under the corridor.
        if (!KeepCorridorValid(query, filter))
        {
            return;
        }
        // 2. Finish a pending replan (bounded per step).
        if (_corr.NeedsReplan)
        {
            PathStatus s = ReplanRound(query, filter);
            if (_state != FollowerState.Moving)
            {
                _velocity = default;
                return;
            }
            if (s == PathStatus.NoEndNearby)
            {
                _state = FollowerState.NoPath;
                _velocity = default;
                return;
            }
            if (s == PathStatus.InvalidStartPoly)
            {
                // The corridor's polygons were unusable: put the agent back on the mesh and plan again next step.
                Recover(query, filter);
                return;
            }
        }
        // 3. Shortcuts, on their step schedule.
        if (!FindCorners(query, filter))
        {
            _velocity = default;
            return;
        }
        bool optimized = false;
        if (_opt.OptimizeInterval > 0 && _steps % _opt.OptimizeInterval == 0 && _cornerCount > 2)
        {
            optimized = TryShortcut(query, filter);
        }
        if (_opt.TopologyInterval > 0 && _steps % _opt.TopologyInterval == 0)
        {
            optimized |= _corr.OptimizeTopology(query, filter, _opt.TopologyBudget);
        }
        if (optimized && !FindCorners(query, filter))
        {
            _velocity = default;
            return;
        }
        // 4. Arrival and off-mesh entry.
        int ci = SteeringCorner();
        double rem = RemainingToStop(out bool known);
        if (OffMeshEntry(out int entry))
        {
            BeginOffMesh(entry);
            return;
        }
        StraightPathPoint c = _corners[ci];
        bool last = ci == _cornerCount - 1;
        if (last && XzDistance(_position, c.Position) <= _opt.ArrivalRadius)
        {
            if (!_corr.IsComplete && _corr.NeedsReplan)
            {
                // The corridor ends short of the destination and a replan is still extending it: wait at its end.
                _velocity = default;
                ResetProgress();
                return;
            }
            Arrive();
            return;
        }
        // 5. Steer: Reynolds' arrival speed, limited acceleration, then walk the corners.
        double wanted = _opt.MaxSpeed;
        if (_opt.SlowRadius > 0 && known)
        {
            wanted = Math.Max(_opt.MaxSpeed * Math.Min(1.0, rem / _opt.SlowRadius), _opt.ArrivalMinSpeed);
        }
        double dx = c.Position.X - _position.X, dz = c.Position.Z - _position.Z;
        double dist = Math.Sqrt((dx * dx) + (dz * dz));
        double wx = dx / dist * wanted, wz = dz / dist * wanted;
        double ax = wx - _velocity.X, az = wz - _velocity.Z;
        double alen = Math.Sqrt((ax * ax) + (az * az));
        double maxDv = _opt.MaxAcceleration * dt;
        double vx = _velocity.X, vz = _velocity.Z;
        if (alen > maxDv)
        {
            double k = maxDv / alen;
            vx += ax * k;
            vz += az * k;
        }
        else
        {
            vx = wx;
            vz = wz;
        }
        double length = Math.Sqrt((vx * vx) + (vz * vz)) * dt;
        bool landedEnd = WalkCorners(query, filter, length, ci);
        if (_state != FollowerState.Moving)
        {
            return;
        }
        Vector3D moved = _position - start;
        _velocity = moved / dt;
        _remaining = known ? Math.Max(0.0, rem - Math.Sqrt((moved.X * moved.X) + (moved.Z * moved.Z))) : double.PositiveInfinity;
        if (landedEnd && !(!_corr.IsComplete && _corr.NeedsReplan))
        {
            Arrive();
            return;
        }
        // 6. Stuck detection.
        UpdateStuck(dt, known && rem <= _opt.SlowRadius);
    }

    // Moves along the corners for `length` of horizontal distance, landing exactly on a corner that is reached; returns true if
    // the agent landed on the end corner. A corner that is the end or an off-mesh entry is never passed.
    private bool WalkCorners(NavMeshQuery query, QueryFilter? filter, double length, int ci)
    {
        double left = length;
        for (int hop = 0; hop < MaxHops && left > 0; hop++)
        {
            StraightPathPoint c = _corners[ci];
            double dx = c.Position.X - _position.X, dz = c.Position.Z - _position.Z;
            double d = Math.Sqrt((dx * dx) + (dz * dz));
            bool stop = ci == _cornerCount - 1 || (c.Flags & StraightPathMarks.OffMeshConnection) != 0;
            Vector3D to;
            bool lands = left >= d;
            if (lands)
            {
                to = c.Position;
            }
            else
            {
                double s = left / d;
                to = new Vector3D(_position.X + (dx * s), _position.Y + ((c.Position.Y - _position.Y) * s), _position.Z + (dz * s));
            }
            // The leg to a corner lies inside the corridor up to the corner's polygon: walk the corridor, not a ray.
            if (!_corr.MoveAlong(query, to, LegLimit(c.Poly)) && !_corr.MovePosition(query, to, filter))
            {
                Recover(query, filter);
                return false;
            }
            _position = _corr.Position;
            if (!lands)
            {
                return false;
            }
            left -= d;
            if (stop)
            {
                return ci == _cornerCount - 1 && (c.Flags & StraightPathMarks.End) != 0;
            }
            // Landed on an intermediate corner with distance to spare: the corners ahead change, so find them again.
            if (!FindCorners(query, filter))
            {
                return false;
            }
            ci = SteeringCorner();
        }
        return false;
    }

    // How many corridor polygons the leg to a corner can lie in: up to and including the corner's own polygon.
    private int LegLimit(PolyRef cornerPoly)
    {
        ReadOnlySpan<PolyRef> path = _corr.Path;
        for (int i = 0; i < path.Length; i++)
        {
            if (path[i] == cornerPoly)
            {
                return i + 1;
            }
        }
        return path.Length;
    }

    // The first corner after the position that is not on top of it (or the last corner).
    private int SteeringCorner()
    {
        int last = _cornerCount - 1;
        for (int i = 1; i < last; i++)
        {
            if (XzDistance(_position, _corners[i].Position) > TinyDistance)
            {
                return i;
            }
        }
        return last;
    }

    // Path distance from the position along the corners to the end or the first off-mesh entry; `known` is false when
    // neither is within the corner buffer (the buffer ended in a corner of a long path).
    private double RemainingToStop(out bool known)
    {
        double sum = 0;
        for (int i = 1; i < _cornerCount; i++)
        {
            sum += XzDistance(_corners[i - 1].Position, _corners[i].Position);
            if ((_corners[i].Flags & (StraightPathMarks.End | StraightPathMarks.OffMeshConnection)) != 0)
            {
                known = true;
                return sum;
            }
        }
        known = _cornerCount > 0 && (_corners[_cornerCount - 1].Flags & StraightPathMarks.End) != 0;
        return sum;
    }

    // The entry corner of an off-mesh link the agent stands at: the first corner (the start itself, or the next one).
    private bool OffMeshEntry(out int entry)
    {
        entry = -1;
        for (int i = 0; i < Math.Min(2, _cornerCount - 1); i++)
        {
            if ((_corners[i].Flags & StraightPathMarks.OffMeshConnection) != 0)
            {
                if (XzDistance(_position, _corners[i].Position) <= _opt.OffMeshTriggerRadius)
                {
                    entry = i;
                    return true;
                }
                return false;
            }
        }
        return false;
    }

    private bool TryShortcut(NavMeshQuery query, QueryFilter? filter)
    {
        // Aim at the farthest corner within range that comes before any off-mesh entry; a ray cannot cross a link anyway.
        int aim = -1;
        for (int i = 2; i < _cornerCount; i++)
        {
            if ((_corners[i - 1].Flags & StraightPathMarks.OffMeshConnection) != 0)
            {
                break;
            }
            if (XzDistance(_position, _corners[i].Position) > _opt.OptimizeRange)
            {
                break;
            }
            aim = i;
        }
        return aim > 0 && _corr.OptimizeVisibility(query, _corners[aim].Position, _opt.OptimizeRange, filter, _opt.OptimizeBudget);
    }

    // Validates the corridor against the mesh; a lost first polygon relocates the agent. False: the state is final for this step.
    private bool KeepCorridorValid(NavMeshQuery query, QueryFilter? filter)
    {
        CorridorStatus v = _corr.Validate(query, filter, _opt.ValidateLookAhead);
        if (v == CorridorStatus.Invalid)
        {
            return Recover(query, filter);
        }
        return true;
    }

    private bool Recover(NavMeshQuery query, QueryFilter? filter)
    {
        if (_corr.Relocate(query, filter, _opt.SearchExtents))
        {
            _position = _corr.Position;
            _velocity = default;
            ResetProgress();
            return true;
        }
        _state = FollowerState.NoPath;
        _velocity = default;
        return false;
    }

    private PathStatus ReplanRound(NavMeshQuery query, QueryFilter? filter)
    {
        // The budget doubles with every unfinished round, so a search that restarts from the closest polygon cannot stay trapped
        // in a pocket nearer the goal than its exits: some round is large enough to leave it. The round counter is state.
        long budget = (long)_opt.ReplanBudget << Math.Min(_rounds, 20);
        PathStatus s = _corr.Replan(query, filter, (int)Math.Min(budget, int.MaxValue), _opt.SearchExtents);
        switch (s)
        {
            case PathStatus.InProgress:
                if (++_rounds >= _opt.MaxReplanRounds)
                {
                    _state = FollowerState.NoPath;
                    _velocity = default;
                }
                break;
            case PathStatus.InvalidStartPoly:
                if (++_rounds >= _opt.MaxReplanRounds)
                {
                    _state = FollowerState.NoPath;
                    _velocity = default;
                }
                break;
            default:
                _rounds = 0;
                break;
        }
        return s;
    }

    // Fills the corner buffer. If the corridor no longer holds together (a break beyond the look-ahead) it is cut or the agent
    // relocated, and the step ends without moving.
    private bool FindCorners(NavMeshQuery query, QueryFilter? filter)
    {
        if (_corr.FindCorners(query, _corners, out _cornerCount) == PathStatus.Success)
        {
            return true;
        }
        _cornerCount = 0;
        if (_corr.Validate(query, filter, int.MaxValue) == CorridorStatus.Invalid || _corr.FindCorners(query, _corners, out _cornerCount) != PathStatus.Success)
        {
            _cornerCount = 0;
            if (_state == FollowerState.Moving && !_corr.Relocate(query, filter, _opt.SearchExtents))
            {
                _state = FollowerState.NoPath;
            }
            _position = _corr.Position;
            return false;
        }
        return true;
    }

    private void Arrive()
    {
        _state = FollowerState.Arrived;
        _reached = _corr.IsComplete;
        _velocity = default;
        _remaining = 0;
        ResetProgress();
    }

    private void ResetProgress()
    {
        _clock = 0;
        _anchor = _position;
    }

    private void UpdateStuck(double dt, bool arriving)
    {
        if (_opt.StuckWindow <= 0 || arriving)
        {
            ResetProgress();
            return;
        }
        _clock += dt;
        if (_clock < _opt.StuckWindow)
        {
            return;
        }
        double moved = XzDistance(_position, _anchor);
        double expected = _opt.MaxSpeed * _clock * _opt.StuckProgress;
        ResetProgress();
        if (moved >= expected)
        {
            return;
        }
        if (++_recoveries > _opt.MaxStuckRecoveries)
        {
            _state = FollowerState.Stuck;
            _velocity = default;
            return;
        }
        _corr.RequestReplan();
    }

    // --- off-mesh links ---

    private void BeginOffMesh(int entry)
    {
        StraightPathPoint a = _corners[entry], b = _corners[entry + 1];
        _corr.TrimTo(a.Poly);
        _corr.SetPosition(a.Position);
        _position = a.Position;
        _offStart = a.Position;
        _offEnd = b.Position;
        _offExitPoly = b.Poly;
        _offElapsed = 0;
        _offDuration = Vector3D.Distance(_offStart, _offEnd) / _opt.OffMeshSpeed;
        _velocity = default;
        _state = FollowerState.OffMeshLink;
        ResetProgress();
    }

    private void StepOffMesh(NavMeshQuery query, double dt, QueryFilter? filter)
    {
        _steps++;
        if (_opt.OffMeshMode == OffMeshMode.Host)
        {
            _velocity = default;
            return;
        }
        _offElapsed += dt;
        if (_offElapsed >= _offDuration)
        {
            FinishOffMesh(query, filter);
            return;
        }
        double t = _offElapsed / _offDuration;
        Vector3D p = Vector3D.Lerp(_offStart, _offEnd, t);
        _velocity = (p - _position) / dt;
        _position = p;
    }

    private void FinishOffMesh(NavMeshQuery query, QueryFilter? filter)
    {
        _velocity = default;
        _position = _offEnd;
        if (!_corr.CrossOffMesh(_offExitPoly, _offEnd))
        {
            _corr.SetPosition(_offEnd);
            if (!_corr.Relocate(query, filter, _opt.SearchExtents))
            {
                _state = FollowerState.NoPath;
                return;
            }
        }
        _position = _corr.Position;
        _state = FollowerState.Moving;
        ResetProgress();
    }

    private static double XzDistance(Vector3D a, Vector3D b)
    {
        double dx = a.X - b.X, dz = a.Z - b.Z;
        return Math.Sqrt((dx * dx) + (dz * dz));
    }

    // --- byte-exact state ---

    /// <summary>Size in bytes of <see cref="Write"/>'s output for the current state.</summary>
    public int ByteSize => 3 + 8 + 4 + 4 + 8 + (6 * 24) + 8 + 8 + 8 + 8 + _corr.ByteSize;

    /// <summary>
    /// Writes the state (see <c>docs/crowds.md</c>, "State bytes"): everything that decides later steps, including the corridor.
    /// The options are not state (the host recreates the follower with the same options before <see cref="Read"/>); neither
    /// are the corner buffer, which every step recomputes, nor a search being sliced through <see cref="CorridorReplan"/>.
    /// </summary>
    /// <param name="destination">At least <see cref="ByteSize"/> bytes.</param>
    /// <returns>The number of bytes written.</returns>
    /// <exception cref="ArgumentException">The destination is too small.</exception>
    public int Write(Span<byte> destination)
    {
        int size = ByteSize;
        if (destination.Length < size)
        {
            throw new ArgumentException($"The destination holds {destination.Length} bytes; {size} are needed.", nameof(destination));
        }
        var w = new StateWriter(destination);
        w.U8(FormatVersion);
        w.U8((byte)_state);
        w.U8(_reached ? (byte)1 : (byte)0);
        w.U64((ulong)_steps);
        w.I32(_rounds);
        w.I32(_recoveries);
        w.F64(_clock);
        w.V3(_anchor);
        w.V3(_position);
        w.V3(_velocity);
        w.V3(_destination);
        w.V3(_offStart);
        w.V3(_offEnd);
        w.F64(_offElapsed);
        w.F64(_offDuration);
        w.U64(_offExitPoly.Value);
        w.F64(_remaining);
        _corr.WriteTo(ref w);
        return w.Position;
    }

    /// <summary>Returns the state as a new array (see <see cref="Write"/>).</summary>
    public byte[] ToBytes()
    {
        byte[] b = new byte[ByteSize];
        Write(b);
        return b;
    }

    /// <summary>
    /// Replaces this follower's state with bytes written by <see cref="Write"/> (from a follower with the same options). Nothing
    /// changes if the bytes are rejected.
    /// </summary>
    /// <exception cref="NavMeshFormatException">The bytes are truncated, of another version, hold impossible values or a corridor larger than <see cref="FollowerOptions.CorridorCapacity"/> (the message names which).</exception>
    public void Read(ReadOnlySpan<byte> source)
    {
        var r = new StateReader(source, "follower");
        r.Version(FormatVersion);
        int state = r.U8();
        if (state > (int)FollowerState.NoPath)
        {
            throw r.Error("state", state);
        }
        int reached = r.U8();
        if (reached > 1)
        {
            throw r.Error("reached flag", reached);
        }
        ulong steps = r.U64();
        if (steps > long.MaxValue)
        {
            throw r.Error("step count", steps);
        }
        int rounds = r.I32(), recoveries = r.I32();
        if (rounds < 0 || recoveries < 0)
        {
            throw r.Error("counter", Math.Min(rounds, recoveries));
        }
        double clock = r.F64("stuck clock", false);
        Vector3D anchor = r.V3("stuck anchor"), position = r.V3("position"), velocity = r.V3("velocity"), destination = r.V3("destination");
        Vector3D offStart = r.V3("off-mesh start"), offEnd = r.V3("off-mesh end");
        double offElapsed = r.F64("off-mesh elapsed", false), offDuration = r.F64("off-mesh duration", false);
        ulong exit = r.U64();
        double remaining = r.F64Remaining();
        PathCorridor corr = PathCorridor.ReadFrom(ref r);
        if (corr.Capacity != _opt.CorridorCapacity)
        {
            throw r.Error("corridor capacity (differs from the options)", corr.Capacity);
        }
        if (r.Position != source.Length)
        {
            throw r.Error("trailing bytes", source.Length - r.Position);
        }
        _corr = corr;
        _state = (FollowerState)state;
        _reached = reached == 1;
        _steps = (long)steps;
        _rounds = rounds;
        _recoveries = recoveries;
        _clock = clock;
        _anchor = anchor;
        _position = position;
        _velocity = velocity;
        _destination = destination;
        _offStart = offStart;
        _offEnd = offEnd;
        _offElapsed = offElapsed;
        _offDuration = offDuration;
        _offExitPoly = new PolyRef(exit);
        _remaining = remaining;
        _cornerCount = 0;
    }
}
