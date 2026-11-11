using System;
using System.Buffers;
using System.Collections.Generic;
using System.Globalization;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.Scope;

/// <summary>A crowd on a navigation mesh, replayed deterministically. The crowd's own state bytes are the snapshot.</summary>
internal sealed partial class CrowdWorld : IScopeWorld
{
    // The entity tree lists this many polygons; a larger mesh is still drawn whole.
    internal const int MaxPolygonEntities = 4096;

    private readonly string _name;
    private readonly CrowdPlayer _player;
    private readonly MeshView _mesh;
    private readonly List<CrowdOp> _ops = [];
    private readonly List<Agent> _agents = [];

    public CrowdWorld(string name, CrowdPlayer player, MeshView mesh)
    {
        _name = name;
        _player = player;
        _mesh = mesh;
    }

    public Crowd Crowd => _player.Crowd;

    public long Step => _player.Crowd.StepCount;

    public ulong Hash() => _player.Crowd.StateHash();

    public void Advance(ReadOnlySpan<byte> input, TimelineSink sink)
    {
        try
        {
            if (!input.IsEmpty)
            {
                CrowdOps.Parse(input, _ops);
                foreach (CrowdOp op in _ops)
                {
                    bool ok = _player.Apply(op, out long id);
                    if (sink.IsRecording)
                    {
                        Report(sink, op, id, ok, _mesh.Planar);
                    }
                }
            }

            _player.Step();
        }
        catch (CrowdReplayException e)
        {
            throw new ScopeFormatException(_name, e.Message);
        }

        if (sink.IsRecording)
        {
            int moving = 0, count = 0;
            double speed = 0;
            foreach (Agent a in Crowd)
            {
                count++;
                if (a.State == FollowerState.Moving)
                {
                    moving++;
                    speed += a.Velocity.Length();
                }
            }

            sink.Sample("agent-count", count);
            sink.Sample("moving", moving);
            sink.Sample("mean-speed", moving == 0 ? 0 : speed / moving);
        }
    }

    public void AdvanceAxis(string axis, int position, ReadOnlySpan<byte> input) =>
        throw new ArgumentException($"A crowd source has only the time axis, not '{axis}'.", nameof(axis));

    public void WriteSnapshot(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        byte[] state = Crowd.SaveState();
        writer.Write(state);
    }

    public void Restore(ReadOnlySpan<byte> snapshot)
    {
        try
        {
            Crowd.RestoreState(snapshot);
        }
        catch (OnePathFormatException e)
        {
            throw new ScopeFormatException(_name, "expected a OnePath crowd snapshot: " + e.Message);
        }
    }

    public void EncodePoke(in PokeCall call, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        ScopeValue[] a = call.Args ?? [];
        switch (call.Poke)
        {
            case "set_target":
                Expect(call, a, ValueKind.Entity, ValueKind.Vector);
                CrowdOps.Write(CrowdOp.SetDestination(AgentOf(a[0].GetEntity()), World(a[1].GetVector())), into);
                break;
            case "add_agent":
                Expect(call, a, ValueKind.Vector);
                CrowdOps.Write(CrowdOp.Add(World(a[0].GetVector()), NewAgentOptions()), into);
                break;
            case "remove_agent":
                Expect(call, a, ValueKind.Entity);
                CrowdOps.Write(CrowdOp.Remove(AgentOf(a[0].GetEntity())), into);
                break;
            case "stop_agent":
                Expect(call, a, ValueKind.Entity);
                CrowdOps.Write(CrowdOp.Stop(AgentOf(a[0].GetEntity())), into);
                break;
            case "teleport_agent":
                Expect(call, a, ValueKind.Entity, ValueKind.Vector);
                CrowdOps.Write(CrowdOp.Teleport(AgentOf(a[0].GetEntity()), World(a[1].GetVector())), into);
                break;
            default:
                throw new ArgumentException($"Unknown poke '{call.Poke}'; this source declares set_target, add_agent, remove_agent, stop_agent and teleport_agent.", nameof(call));
        }
    }

    public Difference? FirstDifference(IScopeWorld other)
    {
        if (other is not CrowdWorld o)
        {
            throw new ArgumentException("The other world is not a OnePath crowd world.", nameof(other));
        }

        var left = new SortedDictionary<long, Agent>();
        var right = new SortedDictionary<long, Agent>();
        foreach (Agent a in Crowd)
        {
            left[a.Id] = a;
        }

        foreach (Agent a in o.Crowd)
        {
            right[a.Id] = a;
        }

        var ids = new SortedSet<long>(left.Keys);
        ids.UnionWith(right.Keys);
        foreach (long id in ids)
        {
            EntityId e = ScopeInfo.Agent(id);
            bool inLeft = left.TryGetValue(id, out Agent? l), inRight = right.TryGetValue(id, out Agent? r);
            if (!inLeft || !inRight)
            {
                return new Difference(e, "present", inLeft ? "yes" : "no", inRight ? "yes" : "no");
            }

            Difference? d = Compare(e, "position", l!.Position, r!.Position);
            d ??= Compare(e, "velocity", l.Velocity, r.Velocity);
            d ??= Compare(e, "destination", l.Destination, r.Destination);
            if (d is null && l.State != r.State)
            {
                d = new Difference(e, "state", l.State.ToString(), r.State.ToString());
            }

            if (d is null && !l.RemainingDistance.Equals(r.RemainingDistance))
            {
                d = new Difference(e, "remaining distance", Fmt(l.RemainingDistance), Fmt(r.RemainingDistance));
            }

            if (d is not null)
            {
                return d;
            }
        }

        if (Step != o.Step)
        {
            return new Difference(ScopeInfo.CrowdRoot, "step", Step.ToString(CultureInfo.InvariantCulture), o.Step.ToString(CultureInfo.InvariantCulture));
        }

        // Anything the listed fields do not show (corridors, wall caches, priorities) still counts: equal worlds have equal state hashes.
        return Hash() == o.Hash() ? null : new Difference(ScopeInfo.CrowdRoot, "state", Hash().ToString("x16", CultureInfo.InvariantCulture), o.Hash().ToString("x16", CultureInfo.InvariantCulture));
    }

    public void CollectEntities(List<EntityInfo> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        into.Add(new EntityInfo(ScopeInfo.CrowdRoot, EntityId.None, "crowd", "crowd"));
        var sorted = new SortedDictionary<long, Agent>();
        foreach (Agent a in Crowd)
        {
            sorted[a.Id] = a;
        }

        foreach (long id in sorted.Keys)
        {
            into.Add(new EntityInfo(ScopeInfo.Agent(id), ScopeInfo.CrowdRoot, "agent", $"agent {id}"));
        }

        into.Add(new EntityInfo(ScopeInfo.MeshRoot, EntityId.None, "navmesh", "navigation mesh"));
        _mesh.CollectEntities(into, MaxPolygonEntities);
    }

    public bool Inspect(EntityId entity, PropertyList into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        bool planar = _mesh.Planar;
        if (entity == ScopeInfo.CrowdRoot)
        {
            into.Add("Crowd", "step", ScopeValue.OfInteger(Step));
            into.Add("Crowd", "agents", ScopeValue.OfInteger(Crowd.Count));
            into.Add("Crowd", "time step (s)", ScopeValue.OfNumber(_player.Dt));
            into.Add("Crowd", "state hash", ScopeValue.OfText(Hash().ToString("x16", CultureInfo.InvariantCulture)));
            into.Add("Crowd", "capacity", ScopeValue.OfInteger(_player.Options.MaxAgents));
            return true;
        }

        if (entity == ScopeInfo.MeshRoot)
        {
            into.Add("Mesh", "polygons", ScopeValue.OfInteger(_mesh.Polygons.Count));
            into.Add("Mesh", "tiles", ScopeValue.OfInteger(_mesh.TileCount));
            into.Add("Mesh", "links", ScopeValue.OfInteger(_mesh.Links.Count));
            into.Add("Mesh", "planar", ScopeValue.OfBool(planar));
            into.Add("Mesh", "edit count", ScopeValue.OfInteger(Crowd.Mesh.EditCount));
            return true;
        }

        if (ScopeInfo.IsAgent(entity, out long id))
        {
            Agent? a = Crowd.GetAgent(id);
            if (a is null)
            {
                return false;
            }

            into.Add("Agent", "id", ScopeValue.OfInteger(a.Id));
            into.Add("Agent", "state", ScopeValue.OfText(a.State.ToString()));
            into.Add("Agent", "position", ScopeValue.OfVector(ScopeInfo.P(a.Position, planar)));
            into.Add("Agent", "height", ScopeValue.OfNumber(a.Position.Y));
            into.Add("Agent", "velocity", ScopeValue.OfVector(ScopeInfo.P(a.Velocity, planar)));
            into.Add("Agent", "speed", ScopeValue.OfNumber(a.Velocity.Length()));
            into.Add("Agent", "destination", ScopeValue.OfVector(ScopeInfo.P(a.Destination, planar)));
            into.Add("Agent", "remaining distance", ScopeValue.OfNumber(a.RemainingDistance));
            into.Add("Agent", "has path", ScopeValue.OfBool(a.HasPath));
            into.Add("Agent", "reached target", ScopeValue.OfBool(a.ReachedTarget));
            into.Add("Agent", "corners", ScopeValue.OfInteger(a.Path.Corners.Length));
            AgentOptions o = a.Options;
            into.Add("Settings", "radius", ScopeValue.OfNumber(o.Radius));
            into.Add("Settings", "max speed", ScopeValue.OfNumber(o.MaxSpeed));
            into.Add("Settings", "max acceleration", ScopeValue.OfNumber(o.MaxAcceleration));
            into.Add("Settings", "arrival radius", ScopeValue.OfNumber(o.ArrivalRadius));
            into.Add("Settings", "priority", ScopeValue.OfNumber(a.Priority));
            into.Add("Settings", "quality", ScopeValue.OfText(o.Quality.ToString()));
            into.Add("Settings", "wall clearance", ScopeValue.OfNumber(o.WallClearance));
            into.Add("Settings", "separation distance", ScopeValue.OfNumber(o.SeparationDistance));
            into.Add("Settings", "separation weight", ScopeValue.OfNumber(o.SeparationWeight));
            return true;
        }

        if (_mesh.TryGet(entity, out Debug.DebugPolygon p))
        {
            var centre = Vector3D.Zero;
            foreach (Vector3D v in p.Vertices)
            {
                centre += v;
            }

            centre = centre / p.Vertices.Count;
            into.Add("Polygon", "tile slot", ScopeValue.OfInteger(p.Poly.TileSlot));
            into.Add("Polygon", "index", ScopeValue.OfInteger(p.Poly.PolyIndex));
            into.Add("Polygon", "area", ScopeValue.OfInteger(p.Area));
            into.Add("Polygon", "flags", ScopeValue.OfInteger(p.Flags));
            into.Add("Polygon", "vertices", ScopeValue.OfInteger(p.Vertices.Count));
            into.Add("Polygon", "centre", ScopeValue.OfVector(ScopeInfo.P(centre, planar)));
            into.Add("Polygon", "links", ScopeValue.OfInteger(Crowd.Mesh.GetLinks(p.Poly).Length));
            int agents = 0;
            foreach (Agent a in Crowd)
            {
                if (a.Path.Corridor.FirstPoly == p.Poly)
                {
                    agents++;
                }
            }

            into.Add("Polygon", "agents standing on it", ScopeValue.OfInteger(agents));
            return true;
        }

        return false;
    }

    private static void Report(TimelineSink sink, in CrowdOp op, long id, bool ok, bool planar)
    {
        EntityId e = id == 0 ? EntityId.None : ScopeInfo.Agent(id);
        switch (op.Kind)
        {
            case CrowdOpKind.Add:
                sink.Event("agents", e, ok ? $"agent {id} added" : "add failed: no room or no walkable polygon");
                break;
            case CrowdOpKind.Remove:
                sink.Event("agents", e, $"agent {id} removed");
                break;
            case CrowdOpKind.SetDestination:
                sink.Event("orders", e, $"agent {id} sent to {Fmt(op.Position, planar)}");
                break;
            case CrowdOpKind.Stop:
                sink.Event("orders", e, $"agent {id} stopped");
                break;
            case CrowdOpKind.Teleport:
                sink.Event("orders", e, $"agent {id} teleported to {Fmt(op.Position, planar)}");
                break;
        }
    }

    private static void Expect(in PokeCall call, ScopeValue[] args, params ValueKind[] kinds)
    {
        if (args.Length != kinds.Length)
        {
            throw new ArgumentException($"Poke '{call.Poke}' takes {kinds.Length} arguments, not {args.Length}.", nameof(call));
        }

        for (int i = 0; i < kinds.Length; i++)
        {
            if (args[i].Kind != kinds[i])
            {
                throw new ArgumentException($"Argument {i + 1} of poke '{call.Poke}' must be {kinds[i]}, not {args[i].Kind}.", nameof(call));
            }
        }
    }

    private long AgentOf(EntityId entity)
    {
        if (!ScopeInfo.IsAgent(entity, out long id) || Crowd.GetAgent(id) is null)
        {
            throw new ArgumentException($"Entity {entity} is not an agent in the crowd at step {Step}.", nameof(entity));
        }

        return id;
    }

    // A vector from the viewer is in the view's plane for a planar source (x, z), and in the world for a spatial one.
    private Vector3D World(Vector3 v) => _mesh.Planar ? new Vector3D(v.X, _mesh.FloorY, v.Y) : new Vector3D(v.X, v.Y, v.Z);

    // A new agent copies the settings of the lowest-numbered agent, so it fits the same mesh; the default settings if there is none.
    private AgentOptions NewAgentOptions()
    {
        Agent? first = null;
        foreach (Agent a in Crowd)
        {
            if (first is null || a.Id < first.Id)
            {
                first = a;
            }
        }

        return first?.Options ?? Agent.Humanoid;
    }

    private Difference? Compare(EntityId e, string field, Vector3D l, Vector3D r) =>
        l.X.Equals(r.X) && l.Y.Equals(r.Y) && l.Z.Equals(r.Z) ? null : new Difference(e, field, Fmt(l, _mesh.Planar), Fmt(r, _mesh.Planar));

    private static string Fmt(double v) => v.ToString("R", CultureInfo.InvariantCulture);

    // Vectors read the way the inspector shows them: (x, z, 0) in a planar source, (x, y, z) otherwise.
    private static string Fmt(Vector3D v, bool planar) => planar ? $"({Fmt(v.X)}, {Fmt(v.Z)}, 0)" : $"({Fmt(v.X)}, {Fmt(v.Y)}, {Fmt(v.Z)})";
}
