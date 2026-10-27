using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

public sealed partial class NavMeshQuery
{
    private double[] _portals = new double[6 * 64];

    // How consecutive corridor polygons connect.
    private enum Connection { None, Edge, OffMesh }

    /// <summary>
    /// String-pulls a corridor into the shortest path inside it (the simple funnel algorithm: Lee and Preparata 1984,
    /// Hershberger and Snoeyink). Start and end are first moved to the closest points of the first and last polygons.
    /// Corners are portal endpoints (their heights are the heights along the shared edge). Off-mesh links in the corridor
    /// give two points: the entry (flag <see cref="StraightPathMarks.OffMeshConnection"/>) and the exit, and the funnel restarts there.
    /// </summary>
    /// <param name="start">Path start (world).</param>
    /// <param name="end">Path end (world).</param>
    /// <param name="corridor">Polygons from the start polygon to the end polygon; each pair must touch.</param>
    /// <param name="result">Receives the points.</param>
    /// <param name="count">Number of points written.</param>
    /// <param name="truncated">True if <paramref name="result"/> was too small; the points written are a prefix.</param>
    /// <returns>Success, or <see cref="PathStatus.InvalidCorridor"/> (nothing written).</returns>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21.</exception>
    public PathStatus FindStraightPath(Vector3D start, Vector3D end, ReadOnlySpan<PolyRef> corridor, Span<StraightPathPoint> result, out int count, out bool truncated)
    {
        CheckPos(start, nameof(start));
        CheckPos(end, nameof(end));
        count = 0;
        truncated = false;
        if (corridor.Length == 0 || !ClosestPointOnPoly(corridor[0], start, out Vector3D s, out _) || !ClosestPointOnPoly(corridor[^1], end, out Vector3D e, out _))
        {
            return PathStatus.InvalidCorridor;
        }
        // First pass: validate the whole corridor so nothing is half-written.
        for (int i = 0; i + 1 < corridor.Length; i++)
        {
            if (!_mesh.IsValid(corridor[i + 1]) || Connect(corridor[i], corridor[i + 1], out _, out _, out _, out _) == Connection.None)
            {
                return PathStatus.InvalidCorridor;
            }
        }
        var w = new Writer(result);
        w.Add(new StraightPathPoint(s, StraightPathMarks.Start, corridor[0]));
        Vector3D segStart = s;
        int segBegin = 0;
        int portalCount = 0;
        for (int i = 0; i + 1 < corridor.Length; i++)
        {
            Connection c = Connect(corridor[i], corridor[i + 1], out Vector3D left, out Vector3D right, out Vector3D offStart, out Vector3D offEnd);
            if (c == Connection.Edge)
            {
                if ((6 * (portalCount + 1)) > _portals.Length)
                {
                    Array.Resize(ref _portals, _portals.Length * 2);
                }
                int o = 6 * portalCount++;
                _portals[o] = left.X;
                _portals[o + 1] = left.Y;
                _portals[o + 2] = left.Z;
                _portals[o + 3] = right.X;
                _portals[o + 4] = right.Y;
                _portals[o + 5] = right.Z;
            }
            else
            {
                Funnel(segStart, offStart, portalCount, corridor, segBegin, ref w);
                w.AddDistinct(new StraightPathPoint(offStart, StraightPathMarks.OffMeshConnection, corridor[i]));
                w.AddDistinct(new StraightPathPoint(offEnd, StraightPathMarks.None, corridor[i + 1]));
                segStart = offEnd;
                segBegin = i + 1;
                portalCount = 0;
            }
        }
        Funnel(segStart, e, portalCount, corridor, segBegin, ref w);
        w.AddDistinct(new StraightPathPoint(e, StraightPathMarks.End, corridor[^1]));
        count = w.Count;
        truncated = w.Truncated;
        return PathStatus.Success;
    }

    private ref struct Writer
    {
        private readonly Span<StraightPathPoint> _r;

        public Writer(Span<StraightPathPoint> r)
        {
            _r = r;
            Count = 0;
            Truncated = false;
        }

        public int Count { get; private set; }

        public bool Truncated { get; private set; }

        public void Add(StraightPathPoint p)
        {
            if (Count < _r.Length)
            {
                _r[Count++] = p;
            }
            else
            {
                Truncated = true;
            }
        }

        // Skips a point at the same position as the last one (keeps the earlier point's flags unless the new one is End or off-mesh).
        public void AddDistinct(StraightPathPoint p)
        {
            if (Count > 0 && Count <= _r.Length && _r[Count - 1].Position == p.Position)
            {
                StraightPathPoint last = _r[Count - 1];
                _r[Count - 1] = new StraightPathPoint(last.Position, last.Flags | p.Flags, p.Flags == StraightPathMarks.None ? last.Poly : p.Poly);
                return;
            }
            Add(p);
        }
    }

    // Twice the signed area of (a, b, c) in X/Z: positive when c lies to the left of a->b (X right, Z up).
    private static double Cross(double ax, double az, double bx, double bz, double cx, double cz) =>
        ((bx - ax) * (cz - az)) - ((bz - az) * (cx - ax));

    // Simple stupid funnel over the portals in _portals[0..pc): appends the corners between `from` and `to`.
    // A corner on portal k is entered when the path passes from corridor[polyBase + k] to corridor[polyBase + k + 1].
    private void Funnel(Vector3D from, Vector3D to, int pc, ReadOnlySpan<PolyRef> corridor, int polyBase, ref Writer w)
    {
        double ax = from.X, ay = from.Y, az = from.Z; // apex
        double lx = ax, ly = ay, lz = az;
        double rx = ax, ry = ay, rz = az;
        int apexIdx = 0, leftIdx = 0, rightIdx = 0;
        for (int i = 0; i <= pc; i++)
        {
            double nlx, nly, nlz, nrx, nry, nrz;
            if (i < pc)
            {
                int o = 6 * i;
                nlx = _portals[o];
                nly = _portals[o + 1];
                nlz = _portals[o + 2];
                nrx = _portals[o + 3];
                nry = _portals[o + 4];
                nrz = _portals[o + 5];
            }
            else
            {
                nlx = nrx = to.X;
                nly = nry = to.Y;
                nlz = nrz = to.Z;
            }
            // Tighten the right side.
            if (Cross(ax, az, rx, rz, nrx, nrz) >= 0)
            {
                if ((ax == rx && az == rz) || Cross(ax, az, lx, lz, nrx, nrz) < 0)
                {
                    rx = nrx;
                    ry = nry;
                    rz = nrz;
                    rightIdx = i;
                }
                else
                {
                    // The right edge passed the left edge: the left point is a corner.
                    w.AddDistinct(new StraightPathPoint(new Vector3D(lx, ly, lz), StraightPathMarks.None, corridor[polyBase + Math.Min(leftIdx + 1, pc)]));
                    ax = lx;
                    ay = ly;
                    az = lz;
                    apexIdx = leftIdx;
                    rx = lx;
                    ry = ly;
                    rz = lz;
                    rightIdx = apexIdx;
                    leftIdx = apexIdx;
                    i = apexIdx;
                    continue;
                }
            }
            // Tighten the left side.
            if (Cross(ax, az, lx, lz, nlx, nlz) <= 0)
            {
                if ((ax == lx && az == lz) || Cross(ax, az, rx, rz, nlx, nlz) > 0)
                {
                    lx = nlx;
                    ly = nly;
                    lz = nlz;
                    leftIdx = i;
                }
                else
                {
                    w.AddDistinct(new StraightPathPoint(new Vector3D(rx, ry, rz), StraightPathMarks.None, corridor[polyBase + Math.Min(rightIdx + 1, pc)]));
                    ax = rx;
                    ay = ry;
                    az = rz;
                    apexIdx = rightIdx;
                    lx = rx;
                    ly = ry;
                    lz = rz;
                    leftIdx = apexIdx;
                    rightIdx = apexIdx;
                    i = apexIdx;
                    continue;
                }
            }
        }
        _ = ay;
    }

    // Finds how corridor polygon a connects to b. For an edge: the shared part as a (left, right) pair seen when walking
    // from a to b. For an off-mesh link: its entry and exit points.
    private Connection Connect(PolyRef a, PolyRef b, out Vector3D left, out Vector3D right, out Vector3D offStart, out Vector3D offEnd)
    {
        left = right = offStart = offEnd = default;
        if (!_mesh.TryGetPoly(a, out NavTile ta, out int pa) || !_mesh.TryGetPoly(b, out NavTile tb, out int pb))
        {
            return Connection.None;
        }
        ReadOnlySpan<float> v = ta.Vertices;
        ReadOnlySpan<ushort> idx = ta.PolyIndices;
        int n = ta.PolyVertexCounts[pa];
        int pBase = ta.PolyStart[pa];
        if (a.TileSlot == b.TileSlot)
        {
            for (int e = 0; e < n; e++)
            {
                if (ta.EdgeNeighbors[pBase + e] == pb + 1)
                {
                    int i0 = 3 * idx[pBase + e], i1 = 3 * idx[pBase + ((e + 1) % n)];
                    // Interior is left of the edge direction, so walking out of the edge the start vertex is on the right.
                    right = new Vector3D(ta.OriginX + v[i0], ta.OriginY + v[i0 + 1], ta.OriginZ + v[i0 + 2]);
                    left = new Vector3D(ta.OriginX + v[i1], ta.OriginY + v[i1 + 1], ta.OriginZ + v[i1 + 2]);
                    return Connection.Edge;
                }
            }
        }
        ReadOnlySpan<NavLink> links = _mesh.GetLinks(a);
        for (int j = 0; j < links.Length; j++)
        {
            NavLink l = links[j];
            if (l.Target != b || l.Kind != NavLinkKind.Portal)
            {
                continue;
            }
            int axis = (l.Side & 1) == 0 ? 2 : 0;
            int e = l.Index;
            int i0 = 3 * idx[pBase + e], i1 = 3 * idx[pBase + ((e + 1) % n)];
            double u0 = v[i0 + axis], u1 = v[i1 + axis];
            double lo = Math.Min(u0, u1), hi = Math.Max(u0, u1);
            // Exact shared part: this edge's span clipped to the hull of the other polygon's boundary edges on the facing side.
            int opp = NavMeshFormat.OppositeSide(l.Side);
            double hl = double.PositiveInfinity, hh = double.NegativeInfinity;
            for (int k = 0; k < tb.PortalCount(opp); k++)
            {
                if (tb.PortalPoly(opp, k) != pb)
                {
                    continue;
                }
                int ge = tb.PortalEdge(opp, k);
                int nb = tb.PolyVertexCounts[pb];
                int f = ge - tb.PolyStart[pb];
                int j0 = 3 * tb.PolyIndices[ge], j1 = 3 * tb.PolyIndices[tb.PolyStart[pb] + ((f + 1) % nb)];
                double w0 = tb.Vertices[j0 + axis], w1 = tb.Vertices[j1 + axis];
                double elo = Math.Max(lo, Math.Min(w0, w1)), ehi = Math.Min(hi, Math.Max(w0, w1));
                if (ehi > elo)
                {
                    hl = Math.Min(hl, elo);
                    hh = Math.Max(hh, ehi);
                }
            }
            if (!(hh > hl))
            {
                hl = lo + ((l.RangeMin * (hi - lo)) / 255.0);
                hh = lo + ((l.RangeMax * (hi - lo)) / 255.0);
                if (u1 < u0)
                {
                    hl = hi - ((l.RangeMax * (hi - lo)) / 255.0);
                    hh = hi - ((l.RangeMin * (hi - lo)) / 255.0);
                }
            }
            double d = u1 - u0;
            double tl = d == 0 ? 0 : (hl - u0) / d, th = d == 0 ? 0 : (hh - u0) / d;
            Vector3D p0 = new(ta.OriginX + v[i0], ta.OriginY + v[i0 + 1], ta.OriginZ + v[i0 + 2]);
            Vector3D p1 = new(ta.OriginX + v[i1], ta.OriginY + v[i1 + 1], ta.OriginZ + v[i1 + 2]);
            Vector3D pl = Vector3D.Lerp(p0, p1, tl), ph = Vector3D.Lerp(p0, p1, th);
            // tl/th are measured from the edge's first vertex; the first vertex is the right side.
            if (u1 >= u0)
            {
                right = pl;
                left = ph;
            }
            else
            {
                right = ph;
                left = pl;
            }
            return Connection.Edge;
        }
        for (int j = 0; j < links.Length; j++)
        {
            NavLink l = links[j];
            if (l.Target != b || l.Kind == NavLinkKind.Portal)
            {
                continue;
            }
            NavTile owner = _mesh.GetTileBySlot(l.OwnerSlot)!;
            OffMeshLink o = owner.OffMeshLinks[l.Index];
            bool fwd = l.Kind == NavLinkKind.OffMesh;
            offStart = new Vector3D(owner.OriginX + (fwd ? o.StartX : o.EndX), owner.OriginY + (fwd ? o.StartY : o.EndY), owner.OriginZ + (fwd ? o.StartZ : o.EndZ));
            offEnd = new Vector3D(owner.OriginX + (fwd ? o.EndX : o.StartX), owner.OriginY + (fwd ? o.EndY : o.StartY), owner.OriginZ + (fwd ? o.EndZ : o.StartZ));
            return Connection.OffMesh;
        }
        return Connection.None;
    }
}
