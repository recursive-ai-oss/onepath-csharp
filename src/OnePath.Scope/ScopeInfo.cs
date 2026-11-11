using System;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Scope;

/// <summary>What a host program tells a viewer about OnePath, and the ids the module declares.</summary>
public static class ScopeInfo
{
    /// <summary>The module id a probe's handshake names: <c>onepath</c>.</summary>
    public const string ModuleId = "onepath";

    /// <summary>The version of the crowd replay format a host announces (see docs/replay.md).</summary>
    public const int FormatVersion = 1;

    /// <summary>The id of the build-stage axis of a navigation-mesh build source.</summary>
    public const string StageAxis = "stage";

    /// <summary>The stages of a build, in the order the axis steps through them: voxels, filtered spans, compact spans, regions, contours, polygons, detail.</summary>
    public static ReadOnlySpan<string> StageNames => s_stageNames;

    private static readonly string[] s_stageNames = ["Voxels", "Filtered spans", "Compact spans", "Regions", "Contours", "Polygons", "Detail"];

    /// <summary>The library version string for a probe's handshake.</summary>
    public static string LibraryVersion => typeof(Crowd).Assembly.GetName().Version?.ToString(3) ?? "1";

    // Overlay ids (bit positions in the draw request).
    internal const int OvMesh = 0, OvAgents = 1, OvPaths = 2, OvVelocity = 3, OvTargets = 4, OvWalls = 5, OvLabels = 6, OvLinks = 7;
    internal const int OvInput = 0, OvStage = 1, OvFinal = 2;
    internal const int GridCells = 0, GridClosed = 1, GridOpen = 2, GridPath = 3, GridMarkers = 4, GridCosts = 5, GridArrows = 6;
    internal static readonly EntityId GridRoot = new(RootTag | 4), StartMarker = new(RootTag | 5), GoalMarker = new(RootTag | 6);
    internal const ulong CellTag = 2UL << 62;

    internal static EntityId CellEntity(int index) => new(CellTag | (ulong)(uint)index);

    // Entity ids: the top two bits say what an id names.
    internal const ulong KindShift = 62;
    internal const ulong AgentTag = 1UL << 62, PolyTag = 2UL << 62, RootTag = 3UL << 62;
    internal static readonly EntityId CrowdRoot = new(RootTag | 1), MeshRoot = new(RootTag | 2), StageRoot = new(RootTag | 3);

    internal static EntityId Agent(long id) => new(AgentTag | (ulong)id);

    internal static EntityId Poly(ulong polyRefValue) => new(PolyTag | (polyRefValue & 0xFF_FFFF_FFFFUL));

    internal static bool IsAgent(EntityId e, out long id)
    {
        id = (long)(e.Value & ~(3UL << (int)KindShift));
        return (e.Value >> (int)KindShift) == 1;
    }

    internal static Vector3 P(Vector3D v, bool planar) => planar ? new Vector3((float)v.X, (float)v.Z, 0) : new Vector3((float)v.X, (float)v.Y, (float)v.Z);
}
