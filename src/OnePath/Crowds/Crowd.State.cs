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
        byte[] bytes = new byte[GetStateSize()];
        bool ok = TrySaveState(bytes, out int written);
        System.Diagnostics.Debug.Assert(ok && written == bytes.Length);
        return bytes;
    }

    /// <summary>
    /// The number of bytes <see cref="SaveState()"/> would produce now (and <see cref="TrySaveState"/> would write). Allocates nothing;
    /// it walks the state once, so call it again after the crowd changes.
    /// </summary>
    public int GetStateSize()
    {
        var w = new StateSink(Span<byte>.Empty, false);
        WriteState(ref w);
        return w.Length;
    }

    /// <summary>
    /// Writes the bytes of <see cref="SaveState()"/> (OPCS v1, byte for byte) into a buffer the caller owns. Allocates nothing once the
    /// crowd's internal scratch has grown to its steady size (it is only used by <see cref="StateHash"/>).
    /// </summary>
    /// <param name="destination">The buffer; <see cref="GetStateSize"/> bytes are needed.</param>
    /// <param name="written">The number of bytes written, or 0 when <paramref name="destination"/> is too small (nothing is promised about its contents then).</param>
    /// <returns>True when the state fitted.</returns>
    public bool TrySaveState(Span<byte> destination, out int written)
    {
        var w = new StateSink(destination, false);
        WriteState(ref w);
        if (w.Length > destination.Length)
        {
            written = 0;
            return false;
        }
        written = w.Length;
        return true;
    }

    /// <summary>
    /// A 64-bit hash of exactly the state <see cref="SaveState()"/> captures, defined as FNV-1a (64-bit: offset basis
    /// 0xCBF29CE484222325, prime 0x100000001B3, over the bytes in order) of the bytes <see cref="SaveState()"/> returns. Equal states
    /// give equal hashes on every platform and for any <see cref="CrowdOptions.MaxThreads"/>; different states differ except for
    /// the usual 2^-64 collisions. It does not include the mesh or the options (see <see cref="SaveState()"/>). Allocates nothing once
    /// the crowd's internal scratch has grown to the largest follower; costs about one pass over the snapshot bytes.
    /// </summary>
    public ulong StateHash()
    {
        var w = new StateSink(Span<byte>.Empty, true);
        WriteState(ref w);
        return w.Hash;
    }

    private void WriteState(ref StateSink w)
    {
        WriteHeader(ref w, KindCrowd);
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
            WriteAgentOptions(ref w, _agentOpt[i]!);
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
            int size = f.ByteSize;
            w.I32(size);
            if (w.Hashing)
            {
                if (_stateScratch.Length < size)
                {
                    _stateScratch = new byte[Math.Max(size, _stateScratch.Length * 2)];
                }
                f.Write(_stateScratch);
                w.HashBytes(_stateScratch.AsSpan(0, size));
            }
            else if (w.Fits(size))
            {
                f.Write(w.Take(size));
            }
            else
            {
                w.Skip(size);
            }
        }
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
    /// <para>
    /// Allocation: restoring a state of the same agents (same ids and settings in the same slots, only their per-step state differing)
    /// into a crowd whose capacity already suffices allocates nothing. It allocates for each agent that is new to its slot (its
    /// <see cref="Agent"/>, settings and follower objects), when the slot arrays must grow, and when the bytes are rejected (the
    /// exception).
    /// </para>
    /// </summary>
    /// <param name="state">Bytes from <see cref="SaveState()"/> of a crowd with the same options.</param>
    /// <exception cref="NavMeshFormatException">The bytes are truncated, not crowd state, of another version, hold an impossible value (a count beyond the crowd's limits, a NaN, a corridor larger than its capacity, a duplicate agent id) or were saved with different wall-cache settings; the message names which.</exception>
    public void RestoreState(ReadOnlySpan<byte> state)
    {
        Validate(state, KindCrowd, out RestoreHeader h);
        Apply(state, in h);
    }

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
        byte[] all = new byte[8 + 4 + mesh.Length + 4 + crowd.Length];
        var w = new StateSink(all, false);
        WriteHeader(ref w, KindCrowdWithMesh);
        w.I32(mesh.Length);
        w.Bytes(mesh);
        w.I32(crowd.Length);
        w.Bytes(crowd);
        return all;
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
        ReadOnlySpan<byte> crowd = r.Bytes(crowdLen);
        Validate(crowd, KindCrowd, out RestoreHeader h);
        NavMeshSnapshot.RestoreInto(_mesh, mesh, library);
        Apply(crowd, in h);
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

    // ---- restore: validate everything first (touching nothing), then apply ----

    private struct RestoreHeader
    {
        public int SlotCount, Count, LowestFree;
        public long NextId, Steps;
    }

    // Per slot the validation pass records where the slot, its settings and its end lie in the bytes, and the objects a slot needs
    // that the crowd does not already have. A slot whose agent has the same id and byte-identical settings is reused as it is.
    private AgentOptions?[] _rsOpt = Array.Empty<AgentOptions?>();
    private PathFollower?[] _rsFollow = Array.Empty<PathFollower?>();
    private int[] _rsAt = Array.Empty<int>();
    private long[] _rsIds = Array.Empty<long>();
    private byte[] _stateScratch = Array.Empty<byte>();
    private readonly byte[] _optScratch = new byte[512];

    private static void WriteHeader(ref StateSink w, int kind)
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

    private static void ReadHead(ref CrowdStateReader r, int kind, int wallCap, int maxAgents, out RestoreHeader h)
    {
        ReadHeader(ref r, kind);
        int cap = r.I32();
        if (cap != wallCap)
        {
            throw r.Error("MaxWallsPerAgent (differs from the crowd's options)", cap);
        }
        int slotCount = r.I32(), count = r.I32(), lowestFree = r.I32();
        if (slotCount < 0 || slotCount > maxAgents || slotCount > r.Remaining)
        {
            throw r.Error("slot count", slotCount);
        }
        if (count < 0 || count > slotCount || count > maxAgents)
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
        h = new RestoreHeader { SlotCount = slotCount, Count = count, LowestFree = lowestFree, NextId = (long)nextId, Steps = (long)steps };
    }

    private void Validate(ReadOnlySpan<byte> state, int kind, out RestoreHeader h)
    {
        // Scratch left over from a rejected earlier call must not leak into this one.
        Array.Clear(_rsOpt);
        Array.Clear(_rsFollow);
        var r = new CrowdStateReader(state, "crowd");
        ReadHead(ref r, kind, _wallCap, _opt.MaxAgents, out h);
        if (_rsOpt.Length < h.SlotCount)
        {
            int n = Math.Max(h.SlotCount, 16);
            _rsOpt = new AgentOptions?[n];
            _rsFollow = new PathFollower?[n];
            _rsAt = new int[3 * n];
            _rsIds = new long[n];
        }
        int active = 0;
        for (int i = 0; i < h.SlotCount; i++)
        {
            int at = r.Position;
            int flag = r.U8();
            if (flag > 1)
            {
                throw r.Error("active flag", flag);
            }
            if (flag == 0)
            {
                if (i < h.LowestFree)
                {
                    throw r.Error("lowest free slot (a slot below it is empty)", i);
                }
                _rsAt[3 * i] = -1;
                continue;
            }
            if (active >= h.Count)
            {
                throw r.Error("agent count (fewer than the active slots)", h.Count);
            }
            _rsIds[active++] = ValidateSlot(ref r, i, h.NextId, at);
        }
        if (active != h.Count)
        {
            throw r.Error("agent count (differs from the active slots)", h.Count);
        }
        if (r.Remaining != 0)
        {
            throw r.Error("trailing bytes", r.Remaining);
        }
        Span<long> ids = _rsIds.AsSpan(0, active);
        ids.Sort();
        for (int i = 1; i < ids.Length; i++)
        {
            if (ids[i] == ids[i - 1])
            {
                throw r.Error("agent id (duplicate)", ids[i]);
            }
        }
    }

    private long ValidateSlot(ref CrowdStateReader r, int slot, long nextId, int at)
    {
        ulong id = r.U64();
        if (id >= (ulong)nextId)
        {
            throw r.Error("agent id", id);
        }
        double priority = r.F64("priority", false);
        if (!(priority > 0))
        {
            throw r.Error("priority", priority);
        }
        int flags = r.U8();
        if ((flags & FlagActive) == 0 || (flags & ~(FlagActive | FlagWalking | FlagCrossing)) != 0)
        {
            throw r.Error("agent flags", flags);
        }
        r.F64("preferred velocity", false);
        r.F64("preferred velocity", false);
        // Same agent as the crowd holds in this slot with byte-identical settings: nothing to build for it.
        AgentOptions? existing = slot < _slotCount && _agents[slot] is { } a && a.Id == (long)id ? _agentOpt[slot] : null;
        AgentOptions? built = null;
        bool reuse = false;
        if (existing is not null)
        {
            var o = new StateSink(_optScratch, false);
            WriteAgentOptions(ref o, existing);
            if (o.Length <= _optScratch.Length && r.Remaining >= o.Length && r.PeekEquals(_optScratch.AsSpan(0, o.Length)))
            {
                r.Bytes(o.Length);
                reuse = true;
            }
        }
        if (!reuse)
        {
            built = ReadAgentOptions(ref r);
        }
        int optsEnd = r.Position;
        ReadWalls(ref r, Span<WallSegment>.Empty, out _, out _, out _);
        int len = r.I32();
        if (len < 0 || len > r.Remaining)
        {
            throw r.Error("follower length", len);
        }
        ReadOnlySpan<byte> fb = r.Bytes(len);
        if (reuse)
        {
            _follow[slot]!.Check(fb);
            _rsOpt[slot] = null;
            _rsFollow[slot] = null;
        }
        else
        {
            var f = new PathFollower(built!.ToFollowerOptions());
            f.Read(fb);
            _rsOpt[slot] = built;
            _rsFollow[slot] = f;
        }
        _rsAt[3 * slot] = at;
        _rsAt[(3 * slot) + 1] = optsEnd;
        _rsAt[(3 * slot) + 2] = r.Position;
        return (long)id;
    }

    // Reads (and with a destination, stores) the wall-cache part of a slot; returns false when the slot has no valid cache.
    private bool ReadWalls(ref CrowdStateReader r, Span<WallSegment> store, out double cx, out double cz, out int count)
    {
        cx = cz = 0;
        count = 0;
        int valid = r.U8();
        if (valid > 1)
        {
            throw r.Error("wall cache flag", valid);
        }
        if (valid == 0)
        {
            return false;
        }
        cx = r.F64("wall cache centre", true);
        cz = r.F64("wall cache centre", true);
        int n = r.I32();
        if (n < 0 || n > _wallCap || (long)n * WallPieceBytes > r.Remaining)
        {
            throw r.Error("wall piece count", n);
        }
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
            if (!store.IsEmpty)
            {
                store[k] = new WallSegment(ax, ay, az, bx, by, bz, new PolyRef(poly), edge);
            }
        }
        count = n;
        return true;
    }

    // ---- apply ----

    private void Apply(ReadOnlySpan<byte> state, in RestoreHeader h)
    {
        // Nothing below can fail: everything was validated by the pass that filled the scratch.
        EnsureSlots(Math.Max(h.SlotCount, 1));
        int top = Math.Max(_slotCount, h.SlotCount);
        for (int i = 0; i < top; i++)
        {
            Agent? old = i < _slotCount ? _agents[i] : null;
            bool present = i < h.SlotCount && _rsAt[3 * i] >= 0;
            if (!present)
            {
                old?.Deactivate();
                _agents[i] = null;
                _follow[i] = null;
                _agentOpt[i] = null;
                _flags[i] = 0;
                _wallStamp[i] = -1;
                _wallCount[i] = 0;
                _debugCount[i] = 0;
                continue;
            }
            var r = new CrowdStateReader(state.Slice(_rsAt[3 * i], _rsAt[(3 * i) + 1] - _rsAt[3 * i]), "crowd");
            r.U8();
            long id = (long)r.U64();
            double priority = r.F64("priority", false);
            byte flags = (byte)r.U8();
            double prefX = r.F64("preferred velocity", false), prefZ = r.F64("preferred velocity", false);
            bool reuse = _rsOpt[i] is null;
            if (!reuse)
            {
                old?.Deactivate();
                old = null;
                var agent = new Agent(this, i, id, _rsOpt[i]!);
                _agents[i] = agent;
                _agentOpt[i] = agent.Options;
                _follow[i] = _rsFollow[i];
                _rsOpt[i] = null;
                _rsFollow[i] = null;
            }
            var t = new CrowdStateReader(state.Slice(_rsAt[(3 * i) + 1], _rsAt[(3 * i) + 2] - _rsAt[(3 * i) + 1]), "crowd");
            bool walls = ReadWalls(ref t, _walls.AsSpan(i * _wallCap, _wallCap), out double cx, out double cz, out int wallCount);
            int len = t.I32();
            ReadOnlySpan<byte> fb = t.Bytes(len);
            if (reuse)
            {
                // Same agent: its follower object (and corridor) is read in place, so a reference to Agent.Path stays valid.
                _follow[i]!.ReadStored(fb);
            }
            _priority[i] = priority;
            _flags[i] = flags;
            _prefX[i] = prefX;
            _prefZ[i] = prefZ;
            _debugCount[i] = 0;
            if (walls)
            {
                _wallCount[i] = wallCount;
                _wallCx[i] = cx;
                _wallCz[i] = cz;
                _wallStamp[i] = _mesh.EditCount;
            }
            else
            {
                _wallCount[i] = 0;
                _wallStamp[i] = -1;
            }
        }
        _slotCount = h.SlotCount;
        _count = h.Count;
        _lowestFree = h.LowestFree;
        _nextId = h.NextId;
        _steps = h.Steps;
    }

    // ---- agent settings ----

    private static void WriteAgentOptions(ref StateSink w, AgentOptions o)
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

    // Little-endian state writer over a caller's span, in one of two modes. Writing: bytes go into the span; what does not fit is
    // skipped but still counted, so an empty span measures the size. Hashing: nothing is stored; the bytes feed FNV-1a (64-bit).
    // Doubles go by bit pattern.
    private ref struct StateSink
    {
        private const ulong FnvBasis = 0xCBF29CE484222325UL, FnvPrime = 0x100000001B3UL;
        private readonly Span<byte> _d;
        private int _n;
        private ulong _h;

        public StateSink(Span<byte> destination, bool hashing)
        {
            _d = destination;
            Hashing = hashing;
            _n = 0;
            _h = FnvBasis;
        }

        public bool Hashing { get; }

        public int Length => _n;

        public ulong Hash => _h;

        public bool Fits(int n) => _d.Length - _n >= n;

        public Span<byte> Take(int n)
        {
            Span<byte> s = _d.Slice(_n, n);
            _n += n;
            return s;
        }

        public void Skip(int n) => _n += n;

        private void Feed(ulong v, int bytes)
        {
            for (int k = 0; k < bytes; k++)
            {
                _h = (_h ^ (byte)(v >> (8 * k))) * FnvPrime;
            }
        }

        public void U8(byte v)
        {
            if (Hashing)
            {
                Feed(v, 1);
            }
            else if (_n < _d.Length)
            {
                _d[_n] = v;
            }
            _n++;
        }

        public void U16(int v)
        {
            if (Hashing)
            {
                Feed((ushort)v, 2);
            }
            else if (Fits(2))
            {
                BinaryPrimitives.WriteUInt16LittleEndian(_d[_n..], (ushort)v);
            }
            _n += 2;
        }

        public void U32(uint v)
        {
            if (Hashing)
            {
                Feed(v, 4);
            }
            else if (Fits(4))
            {
                BinaryPrimitives.WriteUInt32LittleEndian(_d[_n..], v);
            }
            _n += 4;
        }

        public void I32(int v) => U32((uint)v);

        public void U64(ulong v)
        {
            if (Hashing)
            {
                Feed(v, 8);
            }
            else if (Fits(8))
            {
                BinaryPrimitives.WriteUInt64LittleEndian(_d[_n..], v);
            }
            _n += 8;
        }

        public void F64(double v) => U64((ulong)BitConverter.DoubleToInt64Bits(v));

        public void Bytes(ReadOnlySpan<byte> v)
        {
            if (Hashing)
            {
                HashBytes(v);
                return;
            }
            if (Fits(v.Length))
            {
                v.CopyTo(_d[_n..]);
            }
            _n += v.Length;
        }

        public void HashBytes(ReadOnlySpan<byte> v)
        {
            ulong h = _h;
            for (int k = 0; k < v.Length; k++)
            {
                h = (h ^ v[k]) * FnvPrime;
            }
            _h = h;
            _n += v.Length;
        }
    }
}
