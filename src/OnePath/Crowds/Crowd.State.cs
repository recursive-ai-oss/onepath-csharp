using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Crowds.Corridors;
using RecursiveAI.OnePath.Mesh;
using CrowdStateReader = RecursiveAI.OnePath.Crowds.Corridors.StateReader;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

// Byte-exact state: see docs/crowds.md, "Crowd state bytes".
public sealed partial class Crowd
{
    private const uint StateMagic = 0x5343504F; // bytes 4F 50 43 53, "OPCS"
    private const int StateVersion = 1;
    private const int KindCrowd = 1, KindCrowdWithMesh = 2;
    private const int WallPieceBytes = (6 * 8) + 8 + 4;

    /// <summary>
    /// Saves the crowd's state as bytes: the slot table and id counter, the step count and, for every agent, its id, settings,
    /// priority, path follower (corridor included) and cached wall pieces. The same state gives the same bytes on every platform,
    /// saving twice gives the same bytes, and save → restore → save gives the same bytes.
    /// <para>
    /// Not saved: the mesh (see <see cref="SaveStateWithMesh"/>), <see cref="CrowdOptions"/> (the crowd to restore into is created
    /// with the same options), and anything the next step recomputes (neighbour grid, constraint debug lines).
    /// </para>
    /// </summary>
    /// <returns>A new array.</returns>
    public byte[] SaveState()
    {
        var w = new Sink(256 + (_count * 512));
        WriteHeader(w, KindCrowd);
        w.I32(_wallCap);
        w.I32(_slotCount);
        w.I32(_count);
        w.I32(_lowestFree);
        w.U64((ulong)_nextId);
        w.U64((ulong)_steps);
        for (int i = 0; i < _slotCount; i++)
        {
            if ((_flags[i] & FlagActive) == 0)
            {
                w.U8(0);
                continue;
            }
            w.U8(1);
            w.U64((ulong)_agents[i]!.Id);
            w.F64(_priority[i]);
            w.U8(_flags[i]);
            w.F64(_prefX[i]);
            w.F64(_prefZ[i]);
            WriteAgentOptions(w, _agentOpt[i]!);
            // A wall cache is only worth keeping while it matches the mesh; a stale one is refreshed by the next step anyway.
            bool valid = _wallStamp[i] == _mesh.EditCount;
            w.U8(valid ? (byte)1 : (byte)0);
            if (valid)
            {
                w.F64(_wallCx[i]);
                w.F64(_wallCz[i]);
                w.I32(_wallCount[i]);
                for (int k = 0; k < _wallCount[i]; k++)
                {
                    WallSegment s = _walls[(i * _wallCap) + k];
                    w.F64(s.AX);
                    w.F64(s.AY);
                    w.F64(s.AZ);
                    w.F64(s.BX);
                    w.F64(s.BY);
                    w.F64(s.BZ);
                    w.U64(s.Poly.Value);
                    w.I32(s.Edge);
                }
            }
            PathFollower f = _follow[i]!;
            w.I32(f.ByteSize);
            f.Write(w.Reserve(f.ByteSize));
        }
        return w.ToArray();
    }

    /// <summary>
    /// Puts the crowd back to a state written by <see cref="SaveState"/>. The crowd must have been created with the same mesh and
    /// <see cref="CrowdOptions"/> (the delegate in <see cref="CrowdOptions.Filter"/> cannot be saved, so supply it again); if the mesh
    /// was edited since, restore it to the snapshot's state <i>first</i> (<see cref="NavMeshSnapshot.RestoreInto"/>, or use
    /// <see cref="RestoreStateWithMesh"/>), because corridors and cached walls refer to it.
    /// <para>
    /// <see cref="Agent"/> objects of agents that are in the snapshot and still in the crowd stay valid and are updated in place;
    /// agents added since are removed (their objects become inactive); agents in the snapshot but removed since get new
    /// <see cref="Agent"/> objects with the same <see cref="Agent.Id"/> (find them with <see cref="CopyAgents"/> or
    /// <see cref="GetAgent"/>). A step after the restore gives exactly what the step after the save would have given, for any
    /// <see cref="CrowdOptions.MaxThreads"/>.
    /// </para>
    /// <para>All or nothing: if the bytes are rejected, the crowd is unchanged.</para>
    /// </summary>
    /// <param name="state">Bytes from <see cref="SaveState"/> of a crowd with the same options.</param>
    /// <exception cref="NavMeshFormatException">The bytes are truncated, not crowd state, of another version, hold an impossible value (a count beyond the crowd's limits, a NaN, a corridor larger than its capacity, a duplicate agent id) or were saved with different wall-cache settings; the message names which.</exception>
    public void RestoreState(ReadOnlySpan<byte> state) => Commit(Parse(state, KindCrowd));

    /// <summary>
    /// Saves the mesh's run-time state (<see cref="NavMeshSnapshot"/>) and the crowd's together, so a host restores both in the
    /// right order with one call.
    /// </summary>
    /// <param name="mode">How tiles are stored in the mesh part: <see cref="SnapshotMode.Full"/> (self-contained, the default) or <see cref="SnapshotMode.Referenced"/> (needs a <see cref="TileLibrary"/> to restore).</param>
    /// <returns>A new array.</returns>
    public byte[] SaveStateWithMesh(SnapshotMode mode = SnapshotMode.Full)
    {
        byte[] mesh = NavMeshSnapshot.Take(_mesh, mode);
        byte[] crowd = SaveState();
        var w = new Sink(mesh.Length + crowd.Length + 32);
        WriteHeader(w, KindCrowdWithMesh);
        w.I32(mesh.Length);
        w.Bytes(mesh);
        w.I32(crowd.Length);
        w.Bytes(crowd);
        return w.ToArray();
    }

    /// <summary>
    /// Restores what <see cref="SaveStateWithMesh"/> saved: the mesh first, then the crowd. All or nothing: the crowd part is checked
    /// before the mesh is touched, and the mesh restore is itself all or nothing.
    /// </summary>
    /// <param name="state">Bytes from <see cref="SaveStateWithMesh"/>.</param>
    /// <param name="library">Tiles for a mesh part saved in <see cref="SnapshotMode.Referenced"/> mode; null otherwise.</param>
    /// <exception cref="NavMeshFormatException">As for <see cref="RestoreState"/> and <see cref="NavMeshSnapshot.RestoreInto"/>.</exception>
    public void RestoreStateWithMesh(ReadOnlySpan<byte> state, TileLibrary? library = null)
    {
        var r = new CrowdStateReader(state, "crowd");
        ReadHeader(ref r, KindCrowdWithMesh);
        int meshLen = r.I32();
        if (meshLen < 0 || meshLen > r.Remaining)
        {
            throw r.Error("mesh snapshot length", meshLen);
        }
        ReadOnlySpan<byte> mesh = r.Bytes(meshLen);
        int crowdLen = r.I32();
        if (crowdLen < 0 || crowdLen != r.Remaining)
        {
            throw r.Error("crowd state length", crowdLen);
        }
        Prepared prepared = Parse(r.Bytes(crowdLen), KindCrowd);
        NavMeshSnapshot.RestoreInto(_mesh, mesh, library);
        Commit(prepared);
    }

    /// <summary>Finds an agent by its <see cref="Agent.Id"/>.</summary>
    /// <returns>The agent, or null if no agent with that id is in the crowd.</returns>
    public Agent? GetAgent(long id)
    {
        for (int i = 0; i < _slotCount; i++)
        {
            if (_agents[i] is { } a && a.Id == id)
            {
                return a;
            }
        }
        return null;
    }

    // ---- parsing ----

    private sealed class Slot
    {
        public long Id;
        public double Priority, PrefX, PrefZ;
        public byte Flags;
        public AgentOptions Options = null!;
        public bool WallsValid;
        public double WallCx, WallCz;
        public WallSegment[] Walls = Array.Empty<WallSegment>();
        public int FollowerStart, FollowerLength;
        public PathFollower Follower = null!;
    }

    private sealed class Prepared
    {
        public int SlotCount, Count, LowestFree;
        public long NextId, Steps;
        public Slot?[] Slots = Array.Empty<Slot?>();
        public byte[] Bytes = Array.Empty<byte>();
    }

    private static void WriteHeader(Sink w, int kind)
    {
        w.U32(StateMagic);
        w.U16(StateVersion);
        w.U16(kind);
    }

    private static void ReadHeader(ref CrowdStateReader r, int kind)
    {
        if (r.Remaining < 8)
        {
            throw r.Error("header (truncated)", r.Remaining);
        }
        if ((uint)r.I32() != StateMagic)
        {
            throw new NavMeshFormatException("Invalid crowd state: bad magic (not crowd state).");
        }
        int version = r.U16();
        if (version != StateVersion)
        {
            throw new NavMeshFormatException($"Invalid crowd state: version {version}, expected {StateVersion}.");
        }
        int k = r.U16();
        if (k != kind)
        {
            throw new NavMeshFormatException($"Invalid crowd state: kind {k}, expected {kind}.");
        }
    }

    // Reads and validates everything without touching the crowd, building the followers as it goes.
    private Prepared Parse(ReadOnlySpan<byte> state, int kind)
    {
        var r = new CrowdStateReader(state, "crowd");
        ReadHeader(ref r, kind);
        int wallCap = r.I32();
        if (wallCap != _wallCap)
        {
            throw r.Error("MaxWallsPerAgent (differs from the crowd's options)", wallCap);
        }
        int slotCount = r.I32(), count = r.I32(), lowestFree = r.I32();
        if (slotCount < 0 || slotCount > _opt.MaxAgents || slotCount > r.Remaining)
        {
            throw r.Error("slot count", slotCount);
        }
        if (count < 0 || count > slotCount || count > _opt.MaxAgents)
        {
            throw r.Error("agent count", count);
        }
        if (lowestFree < 0 || lowestFree > slotCount)
        {
            throw r.Error("lowest free slot", lowestFree);
        }
        ulong nextId = r.U64(), steps = r.U64();
        if (nextId > long.MaxValue)
        {
            throw r.Error("next id", nextId);
        }
        if (steps > long.MaxValue)
        {
            throw r.Error("step count", steps);
        }
        var p = new Prepared { SlotCount = slotCount, Count = count, LowestFree = lowestFree, NextId = (long)nextId, Steps = (long)steps, Slots = new Slot?[slotCount], Bytes = state.ToArray() };
        int active = 0;
        for (int i = 0; i < slotCount; i++)
        {
            int flag = r.U8();
            if (flag > 1)
            {
                throw r.Error("active flag", flag);
            }
            if (flag == 0)
            {
                if (i < lowestFree)
                {
                    throw r.Error("lowest free slot (a slot below it is empty)", i);
                }
                continue;
            }
            active++;
            if (active > count)
            {
                throw r.Error("agent count (fewer than the active slots)", count);
            }
            p.Slots[i] = ReadSlot(ref r, p.NextId);
        }
        if (active != count)
        {
            throw r.Error("agent count (differs from the active slots)", count);
        }
        if (r.Remaining != 0)
        {
            throw r.Error("trailing bytes", r.Remaining);
        }
        CheckDistinctIds(p, ref r);
        return p;
    }

    private Slot ReadSlot(ref CrowdStateReader r, long nextId)
    {
        var s = new Slot();
        ulong id = r.U64();
        if (id >= (ulong)nextId)
        {
            throw r.Error("agent id", id);
        }
        s.Id = (long)id;
        s.Priority = r.F64("priority", false);
        if (!(s.Priority > 0))
        {
            throw r.Error("priority", s.Priority);
        }
        s.Flags = (byte)r.U8();
        if ((s.Flags & FlagActive) == 0 || (s.Flags & ~(FlagActive | FlagWalking | FlagCrossing)) != 0)
        {
            throw r.Error("agent flags", s.Flags);
        }
        s.PrefX = r.F64("preferred velocity", false);
        s.PrefZ = r.F64("preferred velocity", false);
        s.Options = ReadAgentOptions(ref r);
        int valid = r.U8();
        if (valid > 1)
        {
            throw r.Error("wall cache flag", valid);
        }
        if (valid == 1)
        {
            s.WallsValid = true;
            s.WallCx = r.F64("wall cache centre", true);
            s.WallCz = r.F64("wall cache centre", true);
            int n = r.I32();
            if (n < 0 || n > _wallCap || (long)n * WallPieceBytes > r.Remaining)
            {
                throw r.Error("wall piece count", n);
            }
            s.Walls = new WallSegment[n];
            for (int k = 0; k < n; k++)
            {
                double ax = r.F64("wall", true), ay = r.F64("wall", true), az = r.F64("wall", true);
                double bx = r.F64("wall", true), by = r.F64("wall", true), bz = r.F64("wall", true);
                ulong poly = r.U64();
                int edge = r.I32();
                if (edge < 0)
                {
                    throw r.Error("wall edge", edge);
                }
                s.Walls[k] = new WallSegment(ax, ay, az, bx, by, bz, new PolyRef(poly), edge);
            }
        }
        int len = r.I32();
        if (len < 0 || len > r.Remaining)
        {
            throw r.Error("follower length", len);
        }
        ReadOnlySpan<byte> fb = r.Bytes(len);
        s.FollowerLength = len;
        s.FollowerStart = r.Position - len;
        s.Follower = new PathFollower(s.Options.ToFollowerOptions());
        s.Follower.Read(fb);
        return s;
    }

    private static void CheckDistinctIds(Prepared p, ref CrowdStateReader r)
    {
        long[] ids = new long[p.Count];
        int n = 0;
        for (int i = 0; i < p.SlotCount; i++)
        {
            if (p.Slots[i] is { } s)
            {
                ids[n++] = s.Id;
            }
        }
        Array.Sort(ids);
        for (int i = 1; i < n; i++)
        {
            if (ids[i] == ids[i - 1])
            {
                throw r.Error("agent id (duplicate)", ids[i]);
            }
        }
    }

    // ---- commit ----

    private void Commit(Prepared p)
    {
        // Nothing below can fail: everything was validated, and the follower bytes were read once already.
        EnsureSlots(Math.Max(p.SlotCount, 1));
        for (int i = 0; i < Math.Max(_slotCount, p.SlotCount); i++)
        {
            Agent? old = i < _slotCount ? _agents[i] : null;
            Slot? s = i < p.SlotCount ? p.Slots[i] : null;
            if (old is not null && (s is null || s.Id != old.Id))
            {
                old.Deactivate();
                old = null;
            }
            if (s is null)
            {
                _agents[i] = null;
                _follow[i] = null;
                _agentOpt[i] = null;
                _flags[i] = 0;
                _wallStamp[i] = -1;
                _wallCount[i] = 0;
                _debugCount[i] = 0;
                continue;
            }
            Agent agent = old ?? new Agent(this, i, s.Id, s.Options);
            _agents[i] = agent;
            _agentOpt[i] = agent.Options;
            if (old is not null && _follow[i] is { } keep)
            {
                // Same agent: keep its follower object, so a reference to Agent.Path stays valid.
                keep.Read(p.Bytes.AsSpan(s.FollowerStart, s.FollowerLength));
            }
            else
            {
                _follow[i] = s.Follower;
            }
            _priority[i] = s.Priority;
            _flags[i] = s.Flags;
            _prefX[i] = s.PrefX;
            _prefZ[i] = s.PrefZ;
            _debugCount[i] = 0;
            if (s.WallsValid)
            {
                s.Walls.CopyTo(_walls.AsSpan(i * _wallCap, _wallCap));
                _wallCount[i] = s.Walls.Length;
                _wallCx[i] = s.WallCx;
                _wallCz[i] = s.WallCz;
                _wallStamp[i] = _mesh.EditCount;
            }
            else
            {
                _wallCount[i] = 0;
                _wallStamp[i] = -1;
            }
        }
        _slotCount = p.SlotCount;
        _count = p.Count;
        _lowestFree = p.LowestFree;
        _nextId = p.NextId;
        _steps = p.Steps;
    }

    // ---- agent settings ----

    private static void WriteAgentOptions(Sink w, AgentOptions o)
    {
        w.F64(o.Radius);
        w.F64(o.MaxSpeed);
        w.F64(o.MaxAcceleration);
        w.F64(o.ArrivalRadius);
        w.F64(o.Priority);
        w.U8((byte)o.Quality);
        w.F64(o.WallClearance);
        w.F64(o.SeparationDistance);
        w.F64(o.SeparationWeight);
        w.F64(o.SidePreference);
        FollowerOptions? f = o.Path;
        w.U8(f is null ? (byte)0 : (byte)1);
        if (f is null)
        {
            return;
        }
        w.F64(f.MaxSpeed);
        w.F64(f.MaxAcceleration);
        w.F64(f.SlowRadius);
        w.F64(f.ArrivalMinSpeed);
        w.F64(f.ArrivalRadius);
        w.I32(f.MaxCorners);
        w.I32(f.ValidateLookAhead);
        w.I32(f.OptimizeInterval);
        w.F64(f.OptimizeRange);
        w.I32(f.OptimizeBudget);
        w.I32(f.TopologyInterval);
        w.I32(f.TopologyBudget);
        w.I32(f.ReplanBudget);
        w.I32(f.MaxReplanRounds);
        w.F64(f.StuckWindow);
        w.F64(f.StuckProgress);
        w.I32(f.MaxStuckRecoveries);
        w.U8((byte)f.OffMeshMode);
        w.F64(f.OffMeshSpeed);
        w.F64(f.OffMeshTriggerRadius);
        w.F64(f.SearchExtents.X);
        w.F64(f.SearchExtents.Y);
        w.F64(f.SearchExtents.Z);
        w.I32(f.CorridorCapacity);
    }

    private static AgentOptions ReadAgentOptions(ref CrowdStateReader r)
    {
        double radius = r.F64("radius", false), maxSpeed = r.F64("max speed", false), maxAcc = r.F64("max acceleration", false);
        double arrival = r.F64("arrival radius", false), priority = r.F64("option priority", false);
        int quality = r.U8();
        double clearance = r.F64("wall clearance", false), sepDist = r.F64("separation distance", false);
        double sepWeight = r.F64("separation weight", false), side = r.F64("side preference", false);
        int hasPath = r.U8();
        if (hasPath > 1)
        {
            throw r.Error("path settings flag", hasPath);
        }
        FollowerOptions? path = null;
        if (hasPath == 1)
        {
            path = new FollowerOptions
            {
                MaxSpeed = r.F64("path max speed", false),
                MaxAcceleration = r.F64("path max acceleration", false),
                SlowRadius = r.F64("slow radius", false),
                ArrivalMinSpeed = r.F64("arrival min speed", false),
                ArrivalRadius = r.F64("path arrival radius", false),
                MaxCorners = r.I32(),
                ValidateLookAhead = r.I32(),
                OptimizeInterval = r.I32(),
                OptimizeRange = r.F64("optimize range", false),
                OptimizeBudget = r.I32(),
                TopologyInterval = r.I32(),
                TopologyBudget = r.I32(),
                ReplanBudget = r.I32(),
                MaxReplanRounds = r.I32(),
                StuckWindow = r.F64("stuck window", false),
                StuckProgress = r.F64("stuck progress", false),
                MaxStuckRecoveries = r.I32(),
                OffMeshMode = (OffMeshMode)r.U8(),
                OffMeshSpeed = r.F64("off-mesh speed", false),
                OffMeshTriggerRadius = r.F64("off-mesh trigger radius", false),
                SearchExtents = new Vector3D(r.F64("search extents", false), r.F64("search extents", false), r.F64("search extents", false)),
                CorridorCapacity = r.I32(),
            };
        }
        var o = new AgentOptions
        {
            Radius = radius,
            MaxSpeed = maxSpeed,
            MaxAcceleration = maxAcc,
            ArrivalRadius = arrival,
            Priority = priority,
            Quality = (AvoidanceQuality)quality,
            WallClearance = clearance,
            SeparationDistance = sepDist,
            SeparationWeight = sepWeight,
            SidePreference = side,
            Path = path,
        };
        try
        {
            o.Validate();
        }
        catch (ArgumentOutOfRangeException e)
        {
            throw new NavMeshFormatException($"Invalid crowd state: agent settings: {e.Message}", e);
        }
        return o;
    }

    // Growable little-endian byte buffer; doubles go by bit pattern.
    private sealed class Sink
    {
        private byte[] _b;
        private int _n;

        public Sink(int capacity) => _b = new byte[Math.Max(capacity, 64)];

        private Span<byte> Take(int n)
        {
            if (_b.Length - _n < n)
            {
                Array.Resize(ref _b, Math.Max(_b.Length * 2, _n + n));
            }
            Span<byte> s = _b.AsSpan(_n, n);
            _n += n;
            return s;
        }

        public Span<byte> Reserve(int n) => Take(n);

        public void U8(byte v) => Take(1)[0] = v;

        public void U16(int v) => BinaryPrimitives.WriteUInt16LittleEndian(Take(2), (ushort)v);

        public void U32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Take(4), v);

        public void I32(int v) => BinaryPrimitives.WriteInt32LittleEndian(Take(4), v);

        public void U64(ulong v) => BinaryPrimitives.WriteUInt64LittleEndian(Take(8), v);

        public void F64(double v) => U64((ulong)BitConverter.DoubleToInt64Bits(v));

        public void Bytes(ReadOnlySpan<byte> v) => v.CopyTo(Take(v.Length));

        public byte[] ToArray() => _b.AsSpan(0, _n).ToArray();
    }
}
