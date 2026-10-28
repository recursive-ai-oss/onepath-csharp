using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// A contour vertex on the cell grid: <see cref="X"/> and <see cref="Z"/> are cell-corner coordinates (0..Width, 0..Depth), <see cref="Y"/> the
/// floor row at the corner. <see cref="Neighbor"/> describes the edge from this vertex to the next one: the region across it,
/// 0 for a wall (no walkable region there), <see cref="Contours.TileBorder"/> where the edge lies on the grid boundary, or the contour's own region
/// for the bridge edges that join a hole to its outline.
/// </summary>
/// <param name="X">Corner x in cells.</param>
/// <param name="Y">Floor row.</param>
/// <param name="Z">Corner z in cells.</param>
/// <param name="Neighbor">Region across the edge to the next vertex; see above.</param>
public readonly record struct ContourVertex(int X, int Y, int Z, int Neighbor);

/// <summary>
/// The outline of one region (or one connected piece of it): a closed loop of vertices, counter-clockwise seen with x to the right and z up, the region on its left.
/// Holes are joined to the outline by a zero-width bridge, so the loop visits the two bridge ends twice.
/// </summary>
public sealed class Contour
{
    internal Contour(int region, byte area, ContourVertex[] vertices, ContourVertex[]? raw, ContourVertex[][]? rawHoles)
    {
        Region = region;
        Area = area;
        Vertices = vertices;
        RawVertices = raw;
        RawHoles = rawHoles;
    }

    /// <summary>The region this contour bounds.</summary>
    public int Region { get; }

    /// <summary>Area id of the region.</summary>
    public byte Area { get; }

    /// <summary>The simplified loop (at least 3 vertices).</summary>
    public ContourVertex[] Vertices { get; }

    /// <summary>The traced loop before simplification, one vertex per unit boundary edge; only when <see cref="ContourSettings.KeepRawVertices"/> was set.</summary>
    public ContourVertex[]? RawVertices { get; }

    /// <summary>The raw loops of the holes joined into this contour (clockwise); only when raw vertices were kept.</summary>
    public ContourVertex[][]? RawHoles { get; }
}

/// <summary>The contours of a compact heightfield: the hand-off to the polygon stage.</summary>
public sealed class ContourSet
{
    internal ContourSet(HeightfieldFrame frame, List<Contour> contours, int regionCount, int droppedContours, int droppedHoles, int rawFallbacks)
    {
        Frame = frame;
        Contours = contours;
        RegionCount = regionCount;
        DroppedContours = droppedContours;
        DroppedHoles = droppedHoles;
        RawFallbacks = rawFallbacks;
    }

    /// <summary>The grid the vertex coordinates refer to.</summary>
    public HeightfieldFrame Frame { get; }

    /// <summary>Contours ordered by region id, then by where tracing found them.</summary>
    public IReadOnlyList<Contour> Contours { get; }

    /// <summary>Number of regions in the source heightfield.</summary>
    public int RegionCount { get; }

    /// <summary>Loops dropped as degenerate. Always 0 at present: a degenerate simplification falls back to a stricter one, then to the raw loop.</summary>
    public int DroppedContours { get; }

    /// <summary>Holes dropped because no valid bridge to the outline existed or no outline contained them.</summary>
    public int DroppedHoles { get; }

    /// <summary>Loops that fell back to the raw loop because simplification crossed the outline over itself or erased a tiny region even without an error allowance.</summary>
    public int RawFallbacks { get; }
}

/// <summary>
/// Traces region outlines on the span grid and simplifies them. Definitions (loop orientation, vertex heights, the simplification rules and their
/// tie-breaks, hole bridging) are in docs/build3d-regions.md.
/// </summary>
public static class Contours
{
    /// <summary>The <see cref="ContourVertex.Neighbor"/> value of an edge on the boundary of the grid.</summary>
    public const int TileBorder = -1;

    /// <summary>Traces and simplifies the contours of every region.</summary>
    /// <exception cref="ArgumentException">A span's region id is negative.</exception>
    /// <exception cref="ArgumentOutOfRangeException">Invalid settings.</exception>
    public static ContourSet Build(CompactHeightfield chf, ContourSettings settings)
    {
        ArgumentNullException.ThrowIfNull(chf);
        ArgumentNullException.ThrowIfNull(settings);
        settings.Validate();
        var g = SpanGraph.For(chf);
        try
        {
            return BuildCore(chf, settings, g);
        }
        finally
        {
            g.Release();
        }
    }

    private static ContourSet BuildCore(CompactHeightfield chf, ContourSettings settings, SpanGraph g)
    {
        Span<CompactSpan> spans = chf.Spans;
        int n = g.Count;
        int[] region = Scratch.Rent<int>(n);
        int maxRegion = 0;
        for (int i = 0; i < n; i++)
        {
            int ri = spans[i].Region;
            if (ri < 0)
            {
                throw new ArgumentException($"Span {i} has a negative region id {ri}.", nameof(chf));
            }
            region[i] = ri;
            if (ri > maxRegion)
            {
                maxRegion = ri;
            }
        }
        var regionArea = new byte[maxRegion + 1];
        for (int i = 0; i < n; i++)
        {
            if (region[i] > 0)
            {
                regionArea[region[i]] = spans[i].Area;
            }
        }

        byte[] visited = Scratch.RentZeroed<byte>(n);
        var traceBuffer = new List<ContourVertex>(256);
        var outers = new List<(ContourVertex[] Raw, long Area)>[maxRegion + 1];
        var holes = new List<ContourVertex[]>[maxRegion + 1];
        int guard = (4 * n) + 8;
        int[] nbrs = g.Nbr;
        for (int i = 0; i < n; i++)
        {
            int r = region[i];
            if (r == 0)
            {
                continue;
            }
            int b = 4 * i;
            // Most spans are inside a region: all four links lead to spans of the same region. Check that before the general test.
            int t0 = nbrs[b], t1 = nbrs[b + 1], t2 = nbrs[b + 2], t3 = nbrs[b + 3];
            if (t0 >= 0 && t1 >= 0 && t2 >= 0 && t3 >= 0 && region[t0] == r && region[t1] == r && region[t2] == r && region[t3] == r)
            {
                continue;
            }
            for (int d = 0; d < 4; d++)
            {
                if ((visited[i] & (1 << d)) != 0 || NeighborValue(g, region, i, d) == r)
                {
                    continue;
                }
                ContourVertex[] loop = Trace(chf, g, region, visited, i, d, guard, traceBuffer);
                long area = SignedArea2(loop);
                if (area > 0)
                {
                    (outers[r] ??= new List<(ContourVertex[], long)>()).Add((loop, area));
                }
                else
                {
                    (holes[r] ??= new List<ContourVertex[]>()).Add(loop);
                }
            }
        }
        Scratch.Return(region);
        Scratch.Return(visited);
        var result = new List<Contour>();
        int dropped = 0;
        int droppedHoles = 0;
        int fallbacks = 0;
        for (int r = 1; r <= maxRegion; r++)
        {
            if (outers[r] == null)
            {
                droppedHoles += holes[r]?.Count ?? 0;
                continue;
            }
            var polys = new List<List<ContourVertex>?>();
            var rawOf = new List<ContourVertex[]>();
            foreach (var (raw, area) in outers[r]!)
            {
                List<ContourVertex>? p = Simplify(raw, settings, ref fallbacks, regionArea, r);
                if (p == null)
                {
                    dropped++;
                }
                polys.Add(p);
                rawOf.Add(raw);
            }
            var holesOf = new List<ContourVertex[]>[polys.Count];
            var rawHolesOf = new List<ContourVertex[]>[polys.Count];
            if (holes[r] != null)
            {
                foreach (ContourVertex[] rawHole in holes[r]!)
                {
                    List<ContourVertex>? h = Simplify(rawHole, settings, ref fallbacks, regionArea, r);
                    if (h == null)
                    {
                        // A hole too small to survive simplification: the outline stays solid there.
                        droppedHoles++;
                        continue;
                    }
                    int owner = Owner(polys, h);
                    if (owner < 0)
                    {
                        droppedHoles++;
                        continue;
                    }
                    (holesOf[owner] ??= new List<ContourVertex[]>()).Add(h.ToArray());
                    (rawHolesOf[owner] ??= new List<ContourVertex[]>()).Add(rawHole);
                }
            }
            for (int k = 0; k < polys.Count; k++)
            {
                if (polys[k] == null)
                {
                    droppedHoles += holesOf[k]?.Count ?? 0;
                    continue;
                }
                List<ContourVertex> poly = polys[k]!;
                if (holesOf[k] != null)
                {
                    var outlineOnly = new List<ContourVertex>(poly);
                    int failed = MergeHoles(poly, holesOf[k], r);
                    // Each loop is simplified on its own, so a simplified hole can cross the simplified outline. Then join the raw loops instead.
                    if (HasProperCrossing(poly))
                    {
                        fallbacks++;
                        poly = new List<ContourVertex>(rawOf[k]);
                        failed = MergeHoles(poly, new List<ContourVertex[]>(rawHolesOf[k]!), r);
                    }
                    droppedHoles += failed;
                    // A region that overlaps itself seen from above (a spiral ramp) can have holes that cover the outline; keep the outline then.
                    if (SignedArea2(poly.ToArray()) <= 0)
                    {
                        droppedHoles += holesOf[k].Count - failed;
                        poly = outlineOnly;
                    }
                }
                result.Add(new Contour(r, regionArea[r], poly.ToArray(), settings.KeepRawVertices ? rawOf[k] : null, settings.KeepRawVertices ? rawHolesOf[k]?.ToArray() ?? Array.Empty<ContourVertex[]>() : null));
            }
        }
        return new ContourSet(chf.Frame, result, maxRegion, dropped, droppedHoles, fallbacks);
    }

    // Joins the holes to the outline in order of their leftmost vertex; returns how many could not be joined.
    private static int MergeHoles(List<ContourVertex> poly, List<ContourVertex[]> holes, int region)
    {
        holes.Sort(static (a, b) =>
        {
            int ia = Leftmost(a), ib = Leftmost(b);
            int c = a[ia].X.CompareTo(b[ib].X);
            return c != 0 ? c : a[ia].Z.CompareTo(b[ib].Z);
        });
        var pending = new List<ContourVertex[]>(holes);
        int failed = 0;
        for (int h = 0; h < holes.Count; h++)
        {
            ContourVertex[] hole = holes[h];
            pending.Remove(hole);
            if (!MergeHole(poly, hole, region, pending))
            {
                failed++;
            }
        }
        return failed;
    }

    private static int NeighborValue(SpanGraph g, int[] region, int i, int d)
    {
        int t = g.Nbr[(4 * i) + d];
        if (t >= 0)
        {
            return region[t];
        }
        return g.OutsideGrid(i, d) ? TileBorder : 0;
    }

    // Walks one boundary loop with the region on the left. State (span, dir) is "at the start of edge dir of span".
    private static ContourVertex[] Trace(CompactHeightfield chf, SpanGraph g, int[] region, byte[] visited, int start, int startDir, int guard, List<ContourVertex> loop)
    {
        Span<CompactSpan> spans = chf.Spans;
        loop.Clear();
        int s = start;
        int dir = startDir;
        int r = region[start];
        int steps = 0;
        do
        {
            if (++steps > guard)
            {
                throw new InvalidOperationException("Contour tracing did not close; the compact heightfield is inconsistent.");
            }
            int nv = NeighborValue(g, region, s, dir);
            if (nv != r)
            {
                visited[s] |= (byte)(1 << dir);
                int x = g.CellX(s);
                int z = g.CellZ(s);
                // Start corner of the edge: bottom (3) at (x,z), right (2) at (x+1,z), top (1) at (x+1,z+1), left (0) at (x,z+1).
                int cx = dir == 2 || dir == 1 ? x + 1 : x;
                int cz = dir == 1 || dir == 0 ? z + 1 : z;
                loop.Add(new ContourVertex(cx, CornerHeight(spans, g, s, dir), cz, nv));
                dir = (dir + 3) & 3;
            }
            else
            {
                s = g.Nbr[(4 * s) + dir];
                dir = (dir + 1) & 3;
            }
        }
        while (s != start || dir != startDir);
        return loop.ToArray();
    }

    // Highest floor among the spans around the start corner of edge dir: this span, the two linked across the corner's edges, and the diagonal one.
    private static int CornerHeight(Span<CompactSpan> spans, SpanGraph g, int s, int dir)
    {
        int h = spans[s].Y;
        int d2 = (dir + 1) & 3;
        int a = g.Nbr[(4 * s) + dir];
        int b = g.Nbr[(4 * s) + d2];
        if (a >= 0)
        {
            h = Math.Max(h, spans[a].Y);
            int diag = g.Nbr[(4 * a) + d2];
            if (diag >= 0)
            {
                h = Math.Max(h, spans[diag].Y);
            }
        }
        if (b >= 0)
        {
            h = Math.Max(h, spans[b].Y);
            int diag = g.Nbr[(4 * b) + dir];
            if (diag >= 0)
            {
                h = Math.Max(h, spans[diag].Y);
            }
        }
        return h;
    }

    internal static long SignedArea2(ReadOnlySpan<ContourVertex> p)
    {
        long a = 0;
        for (int i = 0, j = p.Length - 1; i < p.Length; j = i++)
        {
            a += ((long)p[j].X * p[i].Z) - ((long)p[i].X * p[j].Z);
        }
        return a;
    }

    // ---- simplification ----

    // A loop that simplifies to nothing (a tiny region) is retried without the error allowance, then kept raw: a region never disappears.
    // Straight region borders can make a loop cross itself; it is then simplified again with every stretch refined before the raw loop is used.
    private static List<ContourVertex>? Simplify(ContourVertex[] raw, ContourSettings st, ref int fallbacks, byte[] regionArea, int region)
    {
        bool straight = st.StraightRegionBorders;
        List<ContourVertex>? p = SimplifyCore(raw, st.MaxError, st.MaxEdgeLength, ref fallbacks, straight ? regionArea : null, region, st.BorderRegion);
        if (p == null && st.MaxError > 0f)
        {
            p = SimplifyCore(raw, 0f, st.MaxEdgeLength, ref fallbacks, straight ? regionArea : null, region, st.BorderRegion);
        }
        if (p == null)
        {
            fallbacks++;
            p = new List<ContourVertex>(raw);
        }
        return p;
    }

    // Returns null when the loop degenerates.
    private static List<ContourVertex>? SimplifyCore(ContourVertex[] raw, float maxError, int maxEdgeLength, ref int fallbacks, byte[]? straightAreas, int region, int borderRegion)
    {
        int n = raw.Length;
        if (maxError <= 0f)
        {
            straightAreas = null;
        }
        var keep = new bool[n];
        bool portal = false;
        for (int i = 0; i < n; i++)
        {
            if (raw[i].Neighbor != raw[(i + n - 1) % n].Neighbor)
            {
                keep[i] = true;
                portal = true;
            }
        }
        if (!portal)
        {
            // Seeds chosen by position, so both sides of a shared boundary pick the same ones.
            int lo = 0, hi = 0;
            for (int i = 1; i < n; i++)
            {
                if (Less(raw[i], raw[lo]))
                {
                    lo = i;
                }
                if (Less(raw[hi], raw[i]))
                {
                    hi = i;
                }
            }
            keep[lo] = true;
            keep[hi] = true;
        }
        double maxErr2 = (double)maxError * maxError;
        long maxLen2 = (long)maxEdgeLength * maxEdgeLength;
        var xs = new int[n];
        var zs = new int[n];
        for (int i = 0; i < n; i++)
        {
            xs[i] = raw[i].X;
            zs[i] = raw[i].Z;
        }
        var stack = new Stack<(int A, int B)>();
        var kept = new List<int>();
        for (int i = 0; i < n; i++)
        {
            if (keep[i])
            {
                kept.Add(i);
            }
        }
        for (int k = 0; k < kept.Count; k++)
        {
            int a = kept[k];
            int b = kept[(k + 1) % kept.Count];
            stack.Push((a, b <= a ? b + n : b));
        }
        while (stack.Count > 0)
        {
            var (ia, ib) = stack.Pop();
            if (ib - ia < 2)
            {
                continue;
            }
            ContourVertex pa = raw[ia % n], pb = raw[ib % n];
            int best = -1;
            double bestD = -1;
            // The segment's constants are the same for every point between its ends, so they are worked out once; the distance is SegmentDistance2.
            long sx = pb.X - pa.X, sz = pb.Z - pa.Z;
            long len2 = (sx * sx) + (sz * sz);
            int bestIdx = 0;
            for (int k = ia + 1; k < ib; k++)
            {
                int kk = k >= n ? k - n : k;
                long px = xs[kk] - pa.X, pz = zs[kk] - pa.Z;
                double d;
                if (len2 == 0)
                {
                    d = (px * px) + (pz * pz);
                }
                else
                {
                    long dot = (px * sx) + (pz * sz);
                    if (dot <= 0)
                    {
                        d = (px * px) + (pz * pz);
                    }
                    else if (dot >= len2)
                    {
                        long qx = xs[kk] - pb.X, qz = zs[kk] - pb.Z;
                        d = (qx * qx) + (qz * qz);
                    }
                    else
                    {
                        long cross = (sx * pz) - (sz * px);
                        d = (double)cross * cross / len2;
                    }
                }
                // Ties go to the lexicographically smaller point (x, then z).
                if (d > bestD || (d == bestD && (xs[kk] != xs[bestIdx] ? xs[kk] < xs[bestIdx] : zs[kk] < zs[bestIdx])))
                {
                    bestD = d;
                    best = k;
                    bestIdx = kk;
                }
            }
            // Split where the outline deviates too much, or (wall edges only) where the edge is too long: at the raw vertex halfway along it.
            long cdx = pb.X - pa.X, cdz = pb.Z - pa.Z;
            // Douglas & Peucker refinement. A stretch shared with another region of the same area is left as the straight chord between its ends: both
            // regions see the same chord, nothing is lost at a wall, and it saves the vertices a staircase-shaped border would otherwise keep.
            bool straight = straightAreas != null && pa.Neighbor > 0 && pa.Neighbor != borderRegion && straightAreas[pa.Neighbor] == straightAreas[region];
            if (bestD > maxErr2 && !straight)
            {
                keep[best % n] = true;
                stack.Push((ia, best));
                stack.Push((best, ib));
            }
            else if (maxLen2 > 0 && pa.Neighbor == 0 && (cdx * cdx) + (cdz * cdz) > maxLen2)
            {
                int mid = (ia + ib) / 2;
                keep[mid % n] = true;
                stack.Push((ia, mid));
                stack.Push((mid, ib));
            }
        }
        var outLoop = new List<ContourVertex>();
        int firstKept = Array.IndexOf(keep, true);
        for (int k = 0; k < n; k++)
        {
            int i = (firstKept + k) % n;
            if (keep[i])
            {
                outLoop.Add(raw[i]);
            }
        }
        // Drop consecutive vertices at the same position.
        for (int i = outLoop.Count - 1; i >= 0 && outLoop.Count > 1; i--)
        {
            ContourVertex cur = outLoop[i];
            ContourVertex prev = outLoop[(i + outLoop.Count - 1) % outLoop.Count];
            if (cur.X == prev.X && cur.Z == prev.Z)
            {
                outLoop.RemoveAt(i);
            }
        }
        long rawArea = SignedArea2(raw);
        if (outLoop.Count < 3 || SignedArea2(outLoop.ToArray()) == 0 || Math.Sign(SignedArea2(outLoop.ToArray())) != Math.Sign(rawArea))
        {
            return null;
        }
        if (HasProperCrossing(outLoop))
        {
            if (straightAreas != null)
            {
                return SimplifyCore(raw, maxError, maxEdgeLength, ref fallbacks, null, region, borderRegion);
            }
            fallbacks++;
            return new List<ContourVertex>(raw);
        }
        return outLoop;
    }

    private static bool Less(ContourVertex a, ContourVertex b) => a.X != b.X ? a.X < b.X : a.Z < b.Z;

    private static double SegmentDistance2(ContourVertex a, ContourVertex b, ContourVertex p)
    {
        long dx = b.X - a.X, dz = b.Z - a.Z;
        long px = p.X - a.X, pz = p.Z - a.Z;
        long len2 = (dx * dx) + (dz * dz);
        if (len2 == 0)
        {
            return (px * px) + (pz * pz);
        }
        long dot = (px * dx) + (pz * dz);
        if (dot <= 0)
        {
            return (px * px) + (pz * pz);
        }
        if (dot >= len2)
        {
            long qx = p.X - b.X, qz = p.Z - b.Z;
            return (qx * qx) + (qz * qz);
        }
        long cross = (dx * pz) - (dz * px);
        return (double)cross * cross / len2;
    }

    // ---- exact integer geometry (coordinates are grid corners, so long arithmetic is exact) ----

    private static long Orient(long ax, long az, long bx, long bz, long cx, long cz) => ((bx - ax) * (cz - az)) - ((bz - az) * (cx - ax));

    private static bool OnSegment(long ax, long az, long bx, long bz, long px, long pz) =>
        Orient(ax, az, bx, bz, px, pz) == 0 && Math.Min(ax, bx) <= px && px <= Math.Max(ax, bx) && Math.Min(az, bz) <= pz && pz <= Math.Max(az, bz);

    private static bool ProperCross(ContourVertex a, ContourVertex b, ContourVertex c, ContourVertex d)
    {
        long o1 = Orient(a.X, a.Z, b.X, b.Z, c.X, c.Z);
        long o2 = Orient(a.X, a.Z, b.X, b.Z, d.X, d.Z);
        long o3 = Orient(c.X, c.Z, d.X, d.Z, a.X, a.Z);
        long o4 = Orient(c.X, c.Z, d.X, d.Z, b.X, b.Z);
        return ((o1 > 0 && o2 < 0) || (o1 < 0 && o2 > 0)) && ((o3 > 0 && o4 < 0) || (o3 < 0 && o4 > 0));
    }

    private static bool ClosedIntersect(ContourVertex a, ContourVertex b, ContourVertex c, ContourVertex d)
    {
        if (ProperCross(a, b, c, d))
        {
            return true;
        }
        return OnSegment(a.X, a.Z, b.X, b.Z, c.X, c.Z) || OnSegment(a.X, a.Z, b.X, b.Z, d.X, d.Z)
            || OnSegment(c.X, c.Z, d.X, d.Z, a.X, a.Z) || OnSegment(c.X, c.Z, d.X, d.Z, b.X, b.Z);
    }

    private static bool HasProperCrossing(List<ContourVertex> p)
    {
        int m = p.Count;
        for (int i = 0; i < m; i++)
        {
            ContourVertex a = p[i], b = p[(i + 1) % m];
            for (int j = i + 2; j < m; j++)
            {
                if (i == 0 && j == m - 1)
                {
                    continue;
                }
                ContourVertex c = p[j], d = p[(j + 1) % m];
                if (Math.Max(a.X, b.X) < Math.Min(c.X, d.X) || Math.Max(c.X, d.X) < Math.Min(a.X, b.X)
                    || Math.Max(a.Z, b.Z) < Math.Min(c.Z, d.Z) || Math.Max(c.Z, d.Z) < Math.Min(a.Z, b.Z))
                {
                    continue;
                }
                if (ProperCross(a, b, c, d))
                {
                    return true;
                }
            }
        }
        return false;
    }

    // ---- holes ----

    private static int Leftmost(ContourVertex[] p)
    {
        int best = 0;
        for (int i = 1; i < p.Length; i++)
        {
            if (Less(p[i], p[best]))
            {
                best = i;
            }
        }
        return best;
    }

    // The smallest outline containing the hole's leftmost vertex (boundary counts as inside), or -1.
    private static int Owner(List<List<ContourVertex>?> polys, List<ContourVertex> hole)
    {
        ContourVertex q = hole[Leftmost(hole.ToArray())];
        int owner = -1;
        long ownerArea = long.MaxValue;
        for (int k = 0; k < polys.Count; k++)
        {
            List<ContourVertex>? p = polys[k];
            if (p == null || !Contains(p, q))
            {
                continue;
            }
            long a = SignedArea2(p.ToArray());
            if (a < ownerArea)
            {
                ownerArea = a;
                owner = k;
            }
        }
        return owner;
    }

    private static bool Contains(List<ContourVertex> p, ContourVertex q)
    {
        bool inside = false;
        int m = p.Count;
        for (int i = 0, j = m - 1; i < m; j = i++)
        {
            ContourVertex a = p[j], b = p[i];
            if (OnSegment(a.X, a.Z, b.X, b.Z, q.X, q.Z))
            {
                return true;
            }
            if ((a.Z > q.Z) != (b.Z > q.Z))
            {
                // x of the edge at height q.Z compared with q.X, in exact integer form.
                long lhs = (long)(q.X - a.X) * (b.Z - a.Z);
                long rhs = (long)(b.X - a.X) * (q.Z - a.Z);
                if (b.Z > a.Z ? lhs < rhs : lhs > rhs)
                {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    // Is q inside the cone at vertex j of the polygon (interior on the left)?
    private static bool InCone(IList<ContourVertex> p, int j, ContourVertex q)
    {
        int m = p.Count;
        ContourVertex prev = p[(j + m - 1) % m], v = p[j], next = p[(j + 1) % m];
        if (Orient(prev.X, prev.Z, v.X, v.Z, next.X, next.Z) > 0)
        {
            return Orient(v.X, v.Z, next.X, next.Z, q.X, q.Z) > 0 && Orient(v.X, v.Z, q.X, q.Z, prev.X, prev.Z) > 0;
        }
        return !(Orient(v.X, v.Z, prev.X, prev.Z, q.X, q.Z) >= 0 && Orient(v.X, v.Z, q.X, q.Z, next.X, next.Z) >= 0);
    }

    private static bool BridgeFree(ContourVertex p, ContourVertex q, IList<ContourVertex> poly)
    {
        int m = poly.Count;
        for (int i = 0; i < m; i++)
        {
            ContourVertex a = poly[i], b = poly[(i + 1) % m];
            bool touchesEnd = (a.X == p.X && a.Z == p.Z) || (a.X == q.X && a.Z == q.Z) || (b.X == p.X && b.Z == p.Z) || (b.X == q.X && b.Z == q.Z);
            if (!touchesEnd && ClosedIntersect(p, q, a, b))
            {
                return false;
            }
        }
        return true;
    }

    // Joins the hole to the outline through the shortest valid bridge from the hole's leftmost vertex. False when none is valid.
    private static bool MergeHole(List<ContourVertex> outer, ContourVertex[] hole, int region, List<ContourVertex[]> others)
    {
        int k = Leftmost(hole);
        ContourVertex p = hole[k];
        var cand = new List<(long D, int J)>();
        for (int j = 0; j < outer.Count; j++)
        {
            long dx = outer[j].X - p.X, dz = outer[j].Z - p.Z;
            cand.Add(((dx * dx) + (dz * dz), j));
        }
        cand.Sort(static (a, b) => a.D != b.D ? a.D.CompareTo(b.D) : a.J.CompareTo(b.J));
        foreach (var (_, j) in cand)
        {
            ContourVertex q = outer[j];
            if (!InCone(outer, j, p) || !InCone(hole, k, q) || !BridgeFree(p, q, outer) || !BridgeFree(p, q, hole))
            {
                continue;
            }
            bool free = true;
            foreach (ContourVertex[] o in others)
            {
                free &= BridgeFree(p, q, o);
            }
            if (!free)
            {
                continue;
            }
            var merged = new List<ContourVertex>(outer.Count + hole.Length + 2);
            for (int i = 0; i <= j; i++)
            {
                merged.Add(i == j ? outer[j] with { Neighbor = region } : outer[i]);
            }
            for (int i = 0; i <= hole.Length; i++)
            {
                ContourVertex h = hole[(k + i) % hole.Length];
                merged.Add(i == hole.Length ? h with { Neighbor = region } : h);
            }
            merged.Add(outer[j]);
            for (int i = j + 1; i < outer.Count; i++)
            {
                merged.Add(outer[i]);
            }
            outer.Clear();
            outer.AddRange(merged);
            return true;
        }
        return false;
    }
}
