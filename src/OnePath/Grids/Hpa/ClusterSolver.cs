using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Dijkstra's algorithm restricted to one rectangle of cells (a chunk), used to price and refine the edges inside a
/// chunk. Moves are the topology's own (so the diagonal rule and agent size apply); a move whose target lies outside the
/// rectangle is ignored. Every move between two cells of a rectangle looks only at cells of that rectangle, so a
/// chunk's results depend on nothing else. Nodes pop in <see cref="OpenSet"/> order with local indices as the final tie-break,
/// which makes both distances and parent chains identical on every platform.
/// </summary>
internal sealed class ClusterSolver
{
    private readonly SquareTopology _topology;
    private readonly int _width;
    private readonly double[] _g;
    private readonly int[] _parent;
    private readonly uint[] _state;
    private readonly uint[] _target;
    private readonly OpenSet _open;
    private readonly List<int> _scratch = [];
    private uint _epoch;
    private int _x0;
    private int _y0;
    private int _x1;
    private int _y1;
    private int _cw;

    internal ClusterSolver(SquareTopology topology, int maxCells)
    {
        _topology = topology;
        _width = topology.Width;
        _g = new double[maxCells];
        _parent = new int[maxCells];
        _state = new uint[maxCells];
        _target = new uint[maxCells];
        _open = new OpenSet(maxCells);
    }

    /// <summary>
    /// Runs from <paramref name="source"/> over the rectangle [x0..x1] x [y0..y1]. With targets the run stops as soon as
    /// every target inside the rectangle has been settled (their distances are final); without, it settles the whole rectangle.
    /// </summary>
    internal void Run(int x0, int y0, int x1, int y1, int source, ReadOnlySpan<int> targets, int extraTarget = -1)
    {
        if (_epoch >= uint.MaxValue - 4)
        {
            Array.Clear(_state);
            Array.Clear(_target);
            _epoch = 0;
        }
        _epoch += 2;
        _x0 = x0;
        _y0 = y0;
        _x1 = x1;
        _y1 = y1;
        _cw = x1 - x0 + 1;
        int remaining = 0;
        for (int i = 0; i <= targets.Length; i++)
        {
            int t = i < targets.Length ? targets[i] : extraTarget;
            if (t < 0)
            {
                continue;
            }
            int ty = t / _width;
            int tx = t - (ty * _width);
            if (tx < x0 || tx > x1 || ty < y0 || ty > y1)
            {
                continue;
            }
            int lt = ((ty - y0) * _cw) + (tx - x0);
            if (_target[lt] != _epoch)
            {
                _target[lt] = _epoch;
                remaining++;
            }
        }
        bool bounded = remaining > 0;
        _open.Clear();
        int sy = source / _width;
        int l0 = ((sy - y0) * _cw) + (source - (sy * _width) - x0);
        _g[l0] = 0.0;
        _parent[l0] = -1;
        _state[l0] = _epoch;
        _open.Push(l0, 0.0, 0.0);
        var relaxer = new Relaxer { Owner = this };
        while (_open.Pop(out int l, out _, out double g))
        {
            _state[l] = _epoch + 1;
            if (bounded && _target[l] == _epoch && --remaining == 0)
            {
                return;
            }
            int ly = l / _cw;
            relaxer.From = l;
            relaxer.GFrom = g;
            _topology.VisitNeighbors(((y0 + ly) * _width) + x0 + (l - (ly * _cw)), ref relaxer);
        }
    }

    /// <summary>Distance from the last run's source to a cell of the rectangle; +∞ if it was not settled.</summary>
    internal double Dist(int cell)
    {
        int y = cell / _width;
        int x = cell - (y * _width);
        if (x < _x0 || x > _x1 || y < _y0 || y > _y1)
        {
            return double.PositiveInfinity;
        }
        int l = ((y - _y0) * _cw) + (x - _x0);
        return _state[l] == _epoch + 1 ? _g[l] : double.PositiveInfinity;
    }

    /// <summary>Appends the cells from just after the source up to and including <paramref name="cell"/>; false if it was not settled.</summary>
    internal bool AppendPath(int cell, List<int> destination)
    {
        int y = cell / _width;
        int x = cell - (y * _width);
        if (x < _x0 || x > _x1 || y < _y0 || y > _y1)
        {
            return false;
        }
        int l = ((y - _y0) * _cw) + (x - _x0);
        if (_state[l] != _epoch + 1)
        {
            return false;
        }
        _scratch.Clear();
        for (; _parent[l] >= 0; l = _parent[l])
        {
            int ly = l / _cw;
            _scratch.Add(((_y0 + ly) * _width) + _x0 + (l - (ly * _cw)));
        }
        for (int i = _scratch.Count - 1; i >= 0; i--)
        {
            destination.Add(_scratch[i]);
        }
        return true;
    }

    private void Relax(int from, double gFrom, int neighbor, double step)
    {
        int ny = neighbor / _width;
        int nx = neighbor - (ny * _width);
        if (nx < _x0 || nx > _x1 || ny < _y0 || ny > _y1)
        {
            return;
        }
        int l = ((ny - _y0) * _cw) + (nx - _x0);
        double g = gFrom + step;
        if (_state[l] < _epoch)
        {
            _state[l] = _epoch;
        }
        else if (_state[l] != _epoch || !(g < _g[l]))
        {
            return; // closed, or not strictly cheaper: the earlier route keeps the node
        }
        _g[l] = g;
        _parent[l] = from;
        _open.Push(l, g, g);
    }

    private struct Relaxer : INeighborVisitor
    {
        public ClusterSolver Owner;
        public int From;
        public double GFrom;

        public readonly void Visit(int neighbor, double baseCost) => Owner.Relax(From, GFrom, neighbor, baseCost);
    }
}
