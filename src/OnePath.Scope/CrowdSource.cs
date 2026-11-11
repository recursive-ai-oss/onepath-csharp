using System;
using System.Buffers;
using OneScope.Sdk;
using RecursiveAI.OnePath.Replay;

namespace RecursiveAI.OnePath.Scope;

/// <summary>An opened crowd replay (or a live crowd): setup bytes plus per-step operations.</summary>
internal sealed class CrowdSource : IScopeSource
{
    private readonly MeshView _mesh;
    private readonly CrowdOptions _options;
    private readonly double _dt;

    public CrowdSource(string name, Recording recording, MeshView mesh, CrowdOptions options, double dt)
    {
        Name = name;
        Recording = recording;
        _mesh = mesh;
        _options = options;
        _dt = dt;
        Info = Declare(name, mesh.Planar);
    }

    public string Name { get; }

    public SourceInfo Info { get; }

    public Recording Recording { get; }

    public IScopeWorld CreateWorld()
    {
        try
        {
            return new CrowdWorld(Name, new CrowdPlayer(Recording.Setup.Span), _mesh);
        }
        catch (CrowdReplayException e)
        {
            throw new ScopeFormatException(Name, e.Message);
        }
    }

    public IScopeSource WithRecording(Recording recording) => new CrowdSource(Name, recording, _mesh, _options, _dt);

    public void WriteReplay(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        var player = new CrowdPlayer(Recording.Setup.Span);
        var steps = new ReadOnlyMemory<byte>[Recording.StepCount];
        var hashes = new ulong[Recording.StepCount];
        try
        {
            for (long s = Recording.StartStep; s < Recording.EndStep; s++)
            {
                ReadOnlyMemory<byte> input = Recording.InputAt(s);
                player.Apply(input.Span);
                player.Step();
                steps[s - Recording.StartStep] = input;
                hashes[s - Recording.StartStep] = player.Crowd.StateHash();
            }
        }
        catch (CrowdReplayException e)
        {
            throw new ScopeFormatException(Name, e.Message);
        }

        CrowdReplay.Create(Recording.Setup.Span, steps, hashes).Write(writer);
    }

    private static SourceInfo Declare(string name, bool planar) => new()
    {
        ModuleId = ScopeInfo.ModuleId,
        Name = name,
        Dimensions = planar ? 2 : 3,
        Unit = LengthUnit.Metre,
        Axes = [new AxisInfo(AxisIds.Time, "Time", 0)],
        EntityKinds =
        [
            new EntityKindInfo("crowd", "Crowd"),
            new EntityKindInfo("agent", "Agent"),
            new EntityKindInfo("navmesh", "Navigation mesh"),
            new EntityKindInfo("polygon", "Polygon"),
        ],
        Overlays =
        [
            new OverlayInfo(ScopeInfo.OvMesh, "Navigation mesh", true),
            new OverlayInfo(ScopeInfo.OvAgents, "Agents", true),
            new OverlayInfo(ScopeInfo.OvPaths, "Paths", true),
            new OverlayInfo(ScopeInfo.OvVelocity, "Velocities", false),
            new OverlayInfo(ScopeInfo.OvTargets, "Targets", true),
            new OverlayInfo(ScopeInfo.OvLabels, "Agent ids", false),
            new OverlayInfo(ScopeInfo.OvLinks, "Off-mesh links", false),
        ],
        Pokes =
        [
            new PokeInfo("set_target", "Set an agent's target", [new ParamInfo("agent", ValueKind.Entity), new ParamInfo("target", ValueKind.Vector)]) { Role = PokeRole.DragToPoint },
            new PokeInfo("add_agent", "Add an agent", [new ParamInfo("position", ValueKind.Vector)]),
            new PokeInfo("remove_agent", "Remove an agent", [new ParamInfo("agent", ValueKind.Entity)]),
            new PokeInfo("stop_agent", "Stop an agent", [new ParamInfo("agent", ValueKind.Entity)]),
            new PokeInfo("teleport_agent", "Teleport an agent", [new ParamInfo("agent", ValueKind.Entity), new ParamInfo("position", ValueKind.Vector)]),
        ],
        Tracks =
        [
            new TrackInfo("agents", "Agents joining and leaving", TrackKind.Event),
            new TrackInfo("orders", "Orders", TrackKind.Event),
            new TrackInfo("agent-count", "Agents", TrackKind.Series),
            new TrackInfo("moving", "Agents moving", TrackKind.Series),
            new TrackInfo("mean-speed", "Mean speed", TrackKind.Series),
        ],
    };
}
