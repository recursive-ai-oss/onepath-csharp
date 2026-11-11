using System;
using System.Buffers;
using System.Buffers.Binary;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Scope;

/// <summary>An opened grid run: a search or flow field over a grid, one slice of work per time step.</summary>
internal sealed class GridSource : IScopeSource
{
    internal const string IterationAxis = "iteration";
    private readonly GridRun.Parsed _run;
    private readonly byte[] _setup;

    private GridSource(string name, GridRun.Parsed run, byte[] setup, Recording recording)
    {
        Name = name;
        _run = run;
        _setup = setup;
        Recording = recording;
        bool flow = run.Kind == 2;
        Info = new SourceInfo
        {
            ModuleId = ScopeInfo.ModuleId,
            Name = name,
            Dimensions = 2,
            Unit = new LengthUnit("cell", 0.0),
            Axes = [new AxisInfo(AxisIds.Time, "Time", 0), new AxisInfo(IterationAxis, flow ? "Flow-field iteration" : "Search iteration", run.Slice)],
            EntityKinds = [new EntityKindInfo("grid", "Grid"), new EntityKindInfo("cell", "Cell"), new EntityKindInfo("marker", "Start or goal")],
            Overlays =
            [
                new OverlayInfo(ScopeInfo.GridCells, "Cells", true),
                new OverlayInfo(ScopeInfo.GridClosed, flow ? "Settled cells" : "Closed set", true),
                new OverlayInfo(ScopeInfo.GridOpen, "Open set", true),
                new OverlayInfo(ScopeInfo.GridPath, "Path", true),
                new OverlayInfo(ScopeInfo.GridMarkers, flow ? "Goals" : "Start and goal", true),
                new OverlayInfo(ScopeInfo.GridCosts, "Costs", flow),
                new OverlayInfo(ScopeInfo.GridArrows, "Flow directions", flow),
            ],
            Pokes =
            [
                new PokeInfo("move_goal", flow ? "Make a cell the goal" : "Move the goal", [new ParamInfo("marker", ValueKind.Entity), new ParamInfo("target", ValueKind.Vector)]) { Role = PokeRole.DragToPoint },
                .. flow ? Array.Empty<PokeInfo>() : [new PokeInfo("move_start", "Move the start", [new ParamInfo("target", ValueKind.Vector)])],
                new PokeInfo("set_wall", "Set or clear a wall", [new ParamInfo("cell", ValueKind.Vector), new ParamInfo("wall", ValueKind.Bool)]),
                new PokeInfo("set_area", "Set a cell's area", [new ParamInfo("cell", ValueKind.Vector), new ParamInfo("area", ValueKind.Integer)]),
            ],
            Tracks =
            [
                new TrackInfo("edits", "Edits", TrackKind.Event),
                new TrackInfo("search", flow ? "Flow field" : "Search", TrackKind.Event),
                new TrackInfo(flow ? "settled" : "expanded", flow ? "Cells settled" : "Cells expanded", TrackKind.Series),
                new TrackInfo("open", "Open cells", TrackKind.Series),
            ],
        };
    }

    public string Name { get; }

    public SourceInfo Info { get; }

    public Recording Recording { get; }

    public static GridSource Open(ReadOnlyMemory<byte> data, string name)
    {
        byte[] file = data.ToArray();
        GridRun.Parsed run = GridRun.Read(file, name);
        // The grid state is checked when the first world is made; do it now so a bad file is refused at open.
        var source = new GridSource(name, run, file.AsSpan(0, run.SetupLength).ToArray(), Recording.Create(file.AsMemory(0, run.SetupLength), Array.ConvertAll(run.Steps, s => (ReadOnlyMemory<byte>)s)));
        _ = source.CreateWorld();
        return source;
    }

    public IScopeWorld CreateWorld() => new GridWorld(Name, _run);

    public IScopeSource WithRecording(Recording recording) => new GridSource(Name, _run, _setup, recording);

    public void WriteReplay(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.Write(Recording.Setup.Span);
        Span<byte> count = writer.GetSpan(4);
        BinaryPrimitives.WriteInt32LittleEndian(count, (int)Recording.StepCount);
        writer.Advance(4);
        for (long s = Recording.StartStep; s < Recording.EndStep; s++)
        {
            ReadOnlyMemory<byte> input = Recording.InputAt(s);
            Span<byte> len = writer.GetSpan(4);
            BinaryPrimitives.WriteInt32LittleEndian(len, input.Length);
            writer.Advance(4);
            writer.Write(input.Span);
        }
    }
}
