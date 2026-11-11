using System;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Scope;

internal sealed partial class CrowdWorld
{
    private static readonly Color32 MeshFill = new(58, 72, 92), MeshEdge = new(120, 150, 190), LinkColor = new(220, 90, 200), PathColor = new(240, 210, 70), TargetColor = new(240, 150, 60), VelocityColor = new(90, 220, 230), LabelColor = new(235, 235, 235);

    public void Draw(DrawList into, in DrawRequest request)
    {
        ArgumentNullException.ThrowIfNull(into);
        bool planar = _mesh.Planar;
        if (request.IsOn(ScopeInfo.OvMesh))
        {
            into.Mesh(_mesh.Vertices, _mesh.Indices, new DrawStyle(MeshFill, ScopeInfo.OvMesh, 0, DrawFlags.None));
            ReadOnlySpan<Vector3> e = _mesh.Edges;
            var edge = new DrawStyle(MeshEdge, ScopeInfo.OvMesh, 1, DrawFlags.None);
            for (int i = 0; i + 1 < e.Length; i += 2)
            {
                into.Line(e[i], e[i + 1], edge);
            }
        }

        if (request.IsOn(ScopeInfo.OvLinks))
        {
            var style = new DrawStyle(LinkColor, ScopeInfo.OvLinks, 2, DrawFlags.None);
            foreach (Debug.DebugLink l in _mesh.Links)
            {
                if (l.Kind == Mesh.NavLinkKind.OffMesh)
                {
                    into.Line(ScopeInfo.P(l.A, planar), ScopeInfo.P(l.B, planar), style);
                }
            }
        }

        Span<char> text = stackalloc char[24];
        foreach (Agent a in Crowd)
        {
            EntityId entity = ScopeInfo.Agent(a.Id);
            Vector3 at = ScopeInfo.P(a.Position, planar);
            if (request.IsOn(ScopeInfo.OvAgents))
            {
                var style = new DrawStyle(StateColor(a.State), ScopeInfo.OvAgents, 3, DrawFlags.None, entity);
                float r = (float)a.Options.Radius;
                if (planar)
                {
                    into.Sphere(at, r, style);
                }
                else
                {
                    into.Cylinder(at, at + new Vector3(0, r * 2.5f, 0), r, style);
                }
            }

            if (request.IsOn(ScopeInfo.OvPaths) && a.HasPath)
            {
                var style = new DrawStyle(PathColor, ScopeInfo.OvPaths, 4, DrawFlags.OnTop);
                Vector3 prev = at;
                foreach (StraightPathPoint c in a.Path.Corners)
                {
                    Vector3 next = ScopeInfo.P(c.Position, planar);
                    into.Line(prev, next, style);
                    prev = next;
                }
            }

            if (request.IsOn(ScopeInfo.OvTargets) && a.HasPath)
            {
                into.Point(ScopeInfo.P(a.Destination, planar), 6, new DrawStyle(TargetColor, ScopeInfo.OvTargets, 4, DrawFlags.OnTop));
            }

            if (request.IsOn(ScopeInfo.OvVelocity))
            {
                Vector3 v = ScopeInfo.P(a.Velocity, planar);
                if (v != Vector3.Zero)
                {
                    into.Arrow(at, at + v, 0.2f, new DrawStyle(VelocityColor, ScopeInfo.OvVelocity, 5, DrawFlags.OnTop));
                }
            }

            if (request.IsOn(ScopeInfo.OvLabels))
            {
                a.Id.TryFormat(text, out int n, default, System.Globalization.CultureInfo.InvariantCulture);
                into.Label(at + (planar ? new Vector3(0, (float)a.Options.Radius * 1.4f, 0) : new Vector3(0, (float)a.Options.Radius * 3.2f, 0)), text[..n], new DrawStyle(LabelColor, ScopeInfo.OvLabels, 6, DrawFlags.OnTop));
            }
        }
    }

    private static Color32 StateColor(FollowerState s) => s switch
    {
        FollowerState.Moving => new Color32(90, 200, 110),
        FollowerState.OffMeshLink => new Color32(170, 120, 230),
        FollowerState.Arrived => new Color32(80, 140, 235),
        FollowerState.Stuck => new Color32(230, 80, 70),
        FollowerState.NoPath => new Color32(240, 150, 60),
        _ => new Color32(170, 170, 170),
    };
}
