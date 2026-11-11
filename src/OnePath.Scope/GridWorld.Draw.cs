using System;
using System.Collections.Generic;
using System.Numerics;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Scope;

internal sealed partial class GridWorld
{
    private static readonly Color32 Floor = new(52, 60, 74), Wall = new(18, 20, 26), ClosedColor = new(70, 120, 200), OpenColor = new(230, 190, 70), PathColor = new(90, 230, 120), StartColor = new(80, 220, 120), GoalColor = new(235, 90, 90), ArrowColor = new(235, 235, 235);

    public void Draw(DrawList into, in DrawRequest request)
    {
        ArgumentNullException.ThrowIfNull(into);
        int w = _grid.Width, h = _grid.Height;
        ReadOnlySpan<byte> cells = _grid.Cells;
        bool state = !IsFlow && (request.IsOn(ScopeInfo.GridClosed) || request.IsOn(ScopeInfo.GridOpen)) && FrontierKnown;
        double maxCost = 0;
        ReadOnlySpan<double> costs = default;
        ReadOnlySpan<int> next = default;
        if (IsFlow)
        {
            costs = _flow!.Costs;
            next = _flow.Next;
            foreach (double c in costs)
            {
                if (double.IsFinite(c) && c > maxCost)
                {
                    maxCost = c;
                }
            }
        }

        for (int i = 0; i < cells.Length; i++)
        {
            var centre = new Vector3((i % w) + 0.5f, (i / w) + 0.5f, 0);
            if (request.IsOn(ScopeInfo.GridCells))
            {
                Color32 colour = cells[i] == AreaCosts.Wall ? Wall : Floor;
                into.Box(centre, new Vector3(0.48f, 0.48f, 0.02f), Quaternion.Identity, new DrawStyle(colour, ScopeInfo.GridCells, 0, DrawFlags.None, ScopeInfo.CellEntity(i)));
            }

            if (cells[i] == AreaCosts.Wall)
            {
                continue;
            }

            if (state)
            {
                byte s = _frontier[i];
                if (s == SearchFrontier.Closed && request.IsOn(ScopeInfo.GridClosed))
                {
                    into.Box(centre, new Vector3(0.4f, 0.4f, 0.02f), Quaternion.Identity, new DrawStyle(ClosedColor, ScopeInfo.GridClosed, 1, DrawFlags.None, ScopeInfo.CellEntity(i)));
                }
                else if (s == SearchFrontier.Open && request.IsOn(ScopeInfo.GridOpen))
                {
                    into.Box(centre, new Vector3(0.4f, 0.4f, 0.02f), Quaternion.Identity, new DrawStyle(OpenColor, ScopeInfo.GridOpen, 1, DrawFlags.None, ScopeInfo.CellEntity(i)));
                }
            }

            if (IsFlow)
            {
                double c = costs[i];
                if (double.IsFinite(c))
                {
                    if (request.IsOn(ScopeInfo.GridCosts))
                    {
                        float t = maxCost > 0 ? (float)(c / maxCost) : 0f;
                        into.Box(centre, new Vector3(0.45f, 0.45f, 0.02f), Quaternion.Identity, new DrawStyle(ColorMaps.Evaluate(ColorMap.Viridis, t), ScopeInfo.GridCosts, 1, DrawFlags.None, ScopeInfo.CellEntity(i)));
                    }

                    if (request.IsOn(ScopeInfo.GridClosed))
                    {
                        into.Point(centre, 3, new DrawStyle(ClosedColor, ScopeInfo.GridClosed, 2, DrawFlags.OnTop, ScopeInfo.CellEntity(i)));
                    }
                }

                if (request.IsOn(ScopeInfo.GridArrows) && next[i] >= 0)
                {
                    var to = new Vector3((next[i] % w) + 0.5f, (next[i] / w) + 0.5f, 0);
                    into.Arrow(centre, centre + ((to - centre) * 0.7f), 0.15f, new DrawStyle(ArrowColor, ScopeInfo.GridArrows, 3, DrawFlags.OnTop));
                }
            }
        }

        if (!IsFlow && request.IsOn(ScopeInfo.GridPath) && _search!.Status == PathStatus.Found)
        {
            var path = new Cell[_search.PathLength];
            _search.CopyPath(path);
            var style = new DrawStyle(PathColor, ScopeInfo.GridPath, 4, DrawFlags.OnTop);
            for (int i = 0; i + 1 < path.Length; i++)
            {
                into.Line(new Vector3(path[i].X + 0.5f, path[i].Y + 0.5f, 0), new Vector3(path[i + 1].X + 0.5f, path[i + 1].Y + 0.5f, 0), style);
            }
        }
        else if (IsFlow && request.IsOn(ScopeInfo.GridPath))
        {
            // The route from the farthest settled cell, following the field's directions.
            int far = -1;
            for (int i = 0; i < costs.Length; i++)
            {
                if (double.IsFinite(costs[i]) && (far < 0 || costs[i] > costs[far]))
                {
                    far = i;
                }
            }

            var style = new DrawStyle(PathColor, ScopeInfo.GridPath, 4, DrawFlags.OnTop);
            for (int guard = 0; far >= 0 && next[far] >= 0 && guard < costs.Length; guard++)
            {
                int to = next[far];
                into.Line(new Vector3((far % w) + 0.5f, (far / w) + 0.5f, 0), new Vector3((to % w) + 0.5f, (to / w) + 0.5f, 0), style);
                far = to;
            }
        }

        if (request.IsOn(ScopeInfo.GridMarkers))
        {
            if (!IsFlow)
            {
                into.Sphere(new Vector3(_start.X + 0.5f, _start.Y + 0.5f, 0), 0.35f, new DrawStyle(StartColor, ScopeInfo.GridMarkers, 5, DrawFlags.OnTop, ScopeInfo.StartMarker));
            }

            foreach (Cell g in IsFlow ? _goals : [_goal])
            {
                into.Sphere(new Vector3(g.X + 0.5f, g.Y + 0.5f, 0), 0.35f, new DrawStyle(GoalColor, ScopeInfo.GridMarkers, 5, DrawFlags.OnTop, ScopeInfo.GoalMarker));
            }
        }
    }

    public bool Inspect(EntityId entity, PropertyList into)
    {
        ArgumentNullException.ThrowIfNull(into);
        into.Clear();
        int w = _grid.Width;
        if (entity == ScopeInfo.GridRoot)
        {
            into.Add("Grid", "width", ScopeValue.OfInteger(w));
            into.Add("Grid", "height", ScopeValue.OfInteger(_grid.Height));
            into.Add("Grid", "walls", ScopeValue.OfInteger(CountWalls()));
            into.Add("Run", "step", ScopeValue.OfInteger(_step));
            into.Add("Run", "iterations per step", ScopeValue.OfInteger(_run.Slice));
            into.Add("Run", IsFlow ? "status" : "status", ScopeValue.OfText(Status.ToString()));
            into.Add("Run", IsFlow ? "cells settled" : "cells expanded", ScopeValue.OfInteger(ExpandedCount));
            into.Add("Run", "algorithm", ScopeValue.OfText(IsFlow ? "flow field" : _run.Algorithm.ToString()));
            into.Add("Run", "connectivity", ScopeValue.OfInteger((int)_run.Connectivity));
            into.Add("Run", "agent size", ScopeValue.OfInteger(_run.AgentSize));
            if (!IsFlow)
            {
                into.Add("Run", "iterations", ScopeValue.OfInteger(_search!.Iterations));
                if (_search.Status == PathStatus.Found)
                {
                    into.Add("Path", "cost", ScopeValue.OfNumber(_search.PathCost));
                    into.Add("Path", "cells", ScopeValue.OfInteger(_search.PathLength));
                }

                if (FrontierKnown)
                {
                    into.Add("Search", "open cells", ScopeValue.OfInteger(_open));
                    into.Add("Search", "closed cells", ScopeValue.OfInteger(_closed));
                }
            }

            return true;
        }

        if (entity == ScopeInfo.StartMarker || entity == ScopeInfo.GoalMarker)
        {
            Cell c = entity == ScopeInfo.StartMarker ? _start : _goal;
            if (entity == ScopeInfo.StartMarker && IsFlow)
            {
                return false;
            }

            into.Add("Marker", "cell", ScopeValue.OfVector(new Vector3(c.X, c.Y, 0)));
            into.Add("Marker", "area", ScopeValue.OfInteger(_grid.GetArea(c.X, c.Y)));
            return true;
        }

        if ((entity.Value >> 62) == 2 && (entity.Value & ~(3UL << 62)) < (ulong)_grid.CellCount)
        {
            int i = (int)(entity.Value & ~(3UL << 62));
            int x = i % w, y = i / w;
            byte area = _grid.GetArea(x, y);
            into.Add("Cell", "x", ScopeValue.OfInteger(x));
            into.Add("Cell", "y", ScopeValue.OfInteger(y));
            into.Add("Cell", "area", ScopeValue.OfInteger(area));
            into.Add("Cell", "walkable", ScopeValue.OfBool(area != AreaCosts.Wall));
            into.Add("Cell", "clearance", ScopeValue.OfInteger(_grid.GetClearance(x, y)));
            if (area != AreaCosts.Wall)
            {
                into.Add("Cell", "step cost", ScopeValue.OfNumber(_grid.Areas[area]));
            }

            if (IsFlow)
            {
                double c = _flow!.Costs[i];
                into.Add("Flow", "cost", ScopeValue.OfNumber(c));
                int nx = _flow.Next[i];
                if (nx >= 0)
                {
                    into.Add("Flow", "next", ScopeValue.OfVector(new Vector3(nx % w, nx / w, 0)));
                }
            }
            else if (FrontierKnown)
            {
                into.Add("Search", "state", ScopeValue.OfText(StateName(_frontier[i])));
            }

            return true;
        }

        return false;
    }

    private int CountWalls()
    {
        int n = 0;
        foreach (byte b in _grid.Cells)
        {
            if (b == AreaCosts.Wall)
            {
                n++;
            }
        }

        return n;
    }
}
