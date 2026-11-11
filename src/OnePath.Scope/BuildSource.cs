using System;
using System.Buffers;
using System.Collections.Generic;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePath.Debug;

namespace RecursiveAI.OnePath.Scope;

/// <summary>A navigation-mesh build input. It has one empty time step so the stage axis has a step to run inside.</summary>
internal sealed class BuildSource : IScopeSource
{
    private readonly BuildStages _stages;
    private readonly Lazy<MeshView> _final;
    private readonly byte[] _file;

    private BuildSource(string name, byte[] file, BuildStages stages, Recording recording)
    {
        Name = name;
        _file = file;
        _stages = stages;
        Recording = recording;
        _final = new Lazy<MeshView>(() => new MeshView(NavMesh.Build(stages.Input.Vertices, stages.Input.Indices, stages.Input.Settings).Tiles));
        Info = new SourceInfo
        {
            ModuleId = ScopeInfo.ModuleId,
            Name = name,
            Dimensions = 3,
            Unit = LengthUnit.Metre,
            Axes = [new AxisInfo(AxisIds.Time, "Time", 0), new AxisInfo(ScopeInfo.StageAxis, "Build stage", ScopeInfo.StageNames.Length)],
            EntityKinds = [new EntityKindInfo("build", "Build"), new EntityKindInfo("stage", "Build stage")],
            Overlays =
            [
                new OverlayInfo(ScopeInfo.OvInput, "Input triangles", true),
                new OverlayInfo(ScopeInfo.OvStage, "Stage output", true),
                new OverlayInfo(ScopeInfo.OvFinal, "Finished mesh", false),
            ],
        };
    }

    public string Name { get; }

    public SourceInfo Info { get; }

    public Recording Recording { get; }

    public static BuildSource Open(ReadOnlyMemory<byte> data, string name)
    {
        BuildInput.Parsed parsed = BuildInput.Read(data.Span, name);
        byte[] file = data.ToArray();
        var stages = new BuildStages(parsed);
        return new BuildSource(name, file, stages, Recording.Create(file.AsMemory(0, 0), [ReadOnlyMemory<byte>.Empty]));
    }

    public IScopeWorld CreateWorld() => new BuildWorld(Name, _stages, _final);

    public IScopeSource WithRecording(Recording recording) => new BuildSource(Name, _file, _stages, recording);

    public void WriteReplay(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        writer.Write(_file);
    }
}

/// <summary>The world of a build source: only a step counter. The stage being viewed is view state and never changes the hash.</summary>
internal sealed class BuildWorld : IScopeWorld
{
    private static readonly Color32 InputColor = new(150, 155, 165), FinalFill = new(70, 130, 200), FinalEdge = new(210, 230, 250);
    private readonly string _name;
    private readonly BuildStages _stages;
    private readonly Lazy<MeshView> _final;
    private int _view;

    public BuildWorld(string name, BuildStages stages, Lazy<MeshView> final)
    {
        _name = name;
        _stages = stages;
        _final = final;
    }

    public long Step { get; private set; }

    /// <summary>The stage position the last <see cref="AdvanceAxis"/> selected; 0 for none.</summary>
    public int ViewStage => _view;

    /// <summary>The hash of a stage's output (positions 1 to 7), the same value calling the library's stage directly and hashing its result gives.</summary>
    public ulong StageHash(int position) => _stages.All[position - 1].Hash;

    public string StageSummary(int position) => _stages.All[position - 1].Summary;

    public ulong Hash() => unchecked((0xCBF29CE484222325UL ^ (ulong)Step) * 0x100000001B3UL);

    public void Advance(ReadOnlySpan<byte> input, TimelineSink sink)
    {
        if (!input.IsEmpty)
        {
            throw new ScopeFormatException(_name, "expected an empty input: a build source takes no pokes.");
        }

        Step++;
        _view = 0;
    }

    public void AdvanceAxis(string axis, int position, ReadOnlySpan<byte> input)
    {
        if (axis != ScopeInfo.StageAxis)
        {
            throw new ArgumentException($"A build source has the time axis and '{ScopeInfo.StageAxis}', not '{axis}'.", nameof(axis));
        }

        if (position < 1 || position > ScopeInfo.StageNames.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(position), position, $"The stage position must be 1 to {ScopeInfo.StageNames.Length}.");
        }

        _ = _stages.All;
        _view = position;
    }

    public void WriteSnapshot(IBufferWriter<byte> writer)
    {
        ArgumentNullException.ThrowIfNull(writer);
        System.Buffers.Binary.BinaryPrimitives.WriteInt64LittleEndian(writer.GetSpan(8), Step);
        writer.Advance(8);
    }

    public void Restore(ReadOnlySpan<byte> snapshot)
    {
        if (snapshot.Length != 8)
        {
            throw new ScopeFormatException(_name, $"expected an 8-byte build snapshot, got {snapshot.Length} bytes.");
        }

        Step = System.Buffers.Binary.BinaryPrimitives.ReadInt64LittleEndian(snapshot);
        _view = 0;
    }

    public void CollectEntities(List<EntityInfo> into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        into.Add(new EntityInfo(ScopeInfo.StageRoot, EntityId.None, "build", "navigation mesh build"));
        for (int i = 0; i < ScopeInfo.StageNames.Length; i++)
        {
            into.Add(new EntityInfo(new EntityId(ScopeInfo.RootTag | (ulong)(uint)(16 + i)), ScopeInfo.StageRoot, "stage", $"{i + 1}. {ScopeInfo.StageNames[i]}"));
        }
    }

    public bool Inspect(EntityId entity, PropertyList into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        NavSettings s = _stages.Input.Settings;
        if (entity == ScopeInfo.StageRoot)
        {
            into.Add("Input", "vertices", ScopeValue.OfInteger(_stages.Input.Vertices.Length / 3));
            into.Add("Input", "triangles", ScopeValue.OfInteger(_stages.Input.Indices.Length / 3));
            into.Add("Settings", "agent radius", ScopeValue.OfNumber(s.AgentRadius));
            into.Add("Settings", "agent height", ScopeValue.OfNumber(s.AgentHeight));
            into.Add("Settings", "walkable climb", ScopeValue.OfNumber(s.WalkableClimb));
            into.Add("Settings", "max slope (degrees)", ScopeValue.OfNumber(s.AgentMaxSlopeDegrees));
            into.Add("Settings", "cell size", ScopeValue.OfNumber(s.CellSize));
            into.Add("Settings", "cell height", ScopeValue.OfNumber(s.CellHeight));
            into.Add("Settings", "partition", ScopeValue.OfText(s.Partition.ToString()));
            return true;
        }

        long k = (long)(entity.Value & ~(3UL << 62)) - 16;
        if ((entity.Value >> 62) == 3 && k >= 0 && k < ScopeInfo.StageNames.Length)
        {
            StageResult r = _stages.All[k];
            into.Add("Stage", "name", ScopeValue.OfText(r.Name));
            into.Add("Stage", "position", ScopeValue.OfInteger(k + 1));
            into.Add("Stage", "hash", ScopeValue.OfText(BuildStages.Hex(r.Hash)));
            into.Add("Stage", "summary", ScopeValue.OfText(r.Summary));
            into.Add("Stage", "elements", ScopeValue.OfInteger(r.Total));
            into.Add("Stage", "drawn", ScopeValue.OfInteger(r.Prims.Count));
            return true;
        }

        return false;
    }

    public void Draw(DrawList into, in DrawRequest request)
    {
        ArgumentNullException.ThrowIfNull(into);
        if (request.IsOn(ScopeInfo.OvInput))
        {
            double[] v = _stages.Input.Vertices;
            int[] idx = _stages.Input.Indices;
            var style = new DrawStyle(InputColor, ScopeInfo.OvInput, 0, DrawFlags.None);
            for (int t = 0; t + 2 < idx.Length; t += 3)
            {
                into.Triangle(Pt(v, idx[t]), Pt(v, idx[t + 1]), Pt(v, idx[t + 2]), style);
            }
        }

        if (request.IsOn(ScopeInfo.OvStage) && _view > 0)
        {
            StageResult r = _stages.All[_view - 1];
            foreach (StagePrim p in r.Prims)
            {
                var style = new DrawStyle(p.Color, ScopeInfo.OvStage, 2, p.Kind == BuildStages.PrimLine ? DrawFlags.OnTop : DrawFlags.None);
                switch (p.Kind)
                {
                    case BuildStages.PrimLine:
                        into.Line(p.A, p.B, style);
                        break;
                    case BuildStages.PrimBox:
                        into.Box(p.A, p.B, Quaternion.Identity, style);
                        break;
                    default:
                        into.Triangle(p.A, p.B, p.C, style);
                        break;
                }
            }
        }

        if (request.IsOn(ScopeInfo.OvFinal))
        {
            MeshView m = _final.Value;
            into.Mesh(m.Vertices, m.Indices, new DrawStyle(FinalFill, ScopeInfo.OvFinal, 1, DrawFlags.None));
            ReadOnlySpan<Vector3> e = m.Edges;
            var edge = new DrawStyle(FinalEdge, ScopeInfo.OvFinal, 3, DrawFlags.OnTop);
            for (int i = 0; i + 1 < e.Length; i += 2)
            {
                into.Line(e[i], e[i + 1], edge);
            }
        }
    }

    public void EncodePoke(in PokeCall call, IBufferWriter<byte> into) =>
        throw new ArgumentException($"A build source declares no pokes, so '{call.Poke}' is unknown.", nameof(call));

    public Difference? FirstDifference(IScopeWorld other)
    {
        if (other is not BuildWorld o)
        {
            throw new ArgumentException("The other world is not a OnePath build world.", nameof(other));
        }

        return Step == o.Step ? null : new Difference(ScopeInfo.StageRoot, "step", Step.ToString(System.Globalization.CultureInfo.InvariantCulture), o.Step.ToString(System.Globalization.CultureInfo.InvariantCulture));
    }

    private static Vector3 Pt(double[] v, int i) => new((float)v[i * 3], (float)v[(i * 3) + 1], (float)v[(i * 3) + 2]);
}
