using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// What the first build of a layer leaves behind so that a rebuild with obstacles can repair it instead of starting over (docs/build3d-runtime.md,
/// "Repairing a layer"): per span the region it belongs to, per region its size and first span, every traced loop with its simplified form, which loop
/// owns which edge, and the contours of every region.
/// </summary>
internal sealed class LayerBase
{
    private bool _overflow;

    /// <summary>True when the first build finished and left a complete record.</summary>
    public bool Ready { get; private set; }

    /// <summary>Number of regions; the tile border region is <see cref="BorderId"/> (one more).</summary>
    public int Count { get; private set; }

    public int BorderId => Count + 1;

    /// <summary>Per span: the region of its connected same-area component, whether the span is in the tile's own cells or in the border (0: none).</summary>
    public int[] Comp { get; private set; } = Array.Empty<int>();

    /// <summary>Per span: its region id as the contour stage sees it (border spans carry <see cref="BorderId"/>).</summary>
    public int[] Region { get; private set; } = Array.Empty<int>();

    /// <summary>Per region: spans in its component (border spans included).</summary>
    public int[] Size { get; private set; } = Array.Empty<int>();

    /// <summary>Per region: the first span of its component, which decides its number.</summary>
    public int[] First { get; private set; } = Array.Empty<int>();

    public byte[] RegionArea { get; private set; } = Array.Empty<byte>();

    /// <summary>Every traced loop, in the order of their smallest edge keys.</summary>
    public List<TracedLoop> Loops { get; } = new();

    /// <summary>Per span and direction (key = 4 × span + direction): the index in <see cref="Loops"/> of the loop that has this edge, or 0xFFFF.</summary>
    public ushort[] EdgeLoop { get; private set; } = Array.Empty<ushort>();

    /// <summary>Per region: indices into <see cref="Loops"/>, in order.</summary>
    public int[][] RegionLoops { get; private set; } = Array.Empty<int[]>();

    /// <summary>Per region: the contours of the first build.</summary>
    public Contour[][] RegionContours { get; private set; } = Array.Empty<Contour[]>();

    public int[] Begin(int n, int count)
    {
        Count = count;
        Comp = new int[n];
        Size = new int[count + 2];
        First = new int[count + 2];
        Array.Fill(First, -1);
        EdgeLoop = new ushort[4 * n];
        Array.Fill(EdgeLoop, ushort.MaxValue);
        Loops.Clear();
        _overflow = false;
        Ready = false;
        return Comp;
    }

    public void Note(int region, int span)
    {
        if (region > 0)
        {
            Size[region]++;
            if (First[region] < 0)
            {
                First[region] = span;
            }
        }
    }

    public void AddLoop(TracedLoop loop, int index)
    {
        if (index >= ushort.MaxValue)
        {
            _overflow = true;
            return;
        }

        Loops.Add(loop);
        int[] keys = loop.Keys!;
        for (int k = 0; k < keys.Length; k++)
        {
            EdgeLoop[keys[k]] = (ushort)index;
        }
    }

    public void End(ContourSet contours, byte[] regionArea, List<TracedLoop>?[] outers, List<TracedLoop>?[] holes, int loopCount, bool keepRaw, int[] region)
    {
        RegionArea = regionArea;
        Region = region.AsSpan(0, Comp.Length).ToArray();
        if (_overflow || keepRaw)
        {
            return;
        }

        var perRegion = new List<int>[Count + 2];
        for (int i = 0; i < Loops.Count; i++)
        {
            (perRegion[Loops[i].Region] ??= new List<int>()).Add(i);
        }

        RegionLoops = new int[Count + 2][];
        for (int r = 0; r < RegionLoops.Length; r++)
        {
            RegionLoops[r] = perRegion[r]?.ToArray() ?? Array.Empty<int>();
        }

        var contoursOf = new List<Contour>[Count + 2];
        foreach (Contour c in contours.Contours)
        {
            (contoursOf[c.Region] ??= new List<Contour>()).Add(c);
        }

        RegionContours = new Contour[Count + 2][];
        for (int r = 0; r < RegionContours.Length; r++)
        {
            RegionContours[r] = contoursOf[r]?.ToArray() ?? Array.Empty<Contour>();
        }

        Ready = true;
    }
}

/// <summary>
/// Repairs the contours of a layer whose areas differ from the first build's only by spans an obstacle made unwalkable.
/// The result is exactly what the from-scratch stages give for those areas (same regions, same loops starting at the same edges, same simplification);
/// it is only cheaper because the work follows the changed spans, not the layer. When a shortcut's precondition fails the caller builds from scratch.
/// </summary>
internal static class LayerRepair
{
    // A search for a way round a removed blob stops after this many spans (a quarter of the layer at least 256): then the layer is built from scratch.
    private const int MinSearch = 256;

    /// <summary>Contours for the areas in <paramref name="area"/>, or null when the repair does not apply (the caller builds from scratch).</summary>
    public static ContourSet? Repair(Mesh3DPipeline p, LayerSpans ls, LayerBase b, byte[] area, LayerWork w)
    {
        int n = ls.Count;
        int dn = w.DeltaCount;
        int[] delta = w.Delta;
        int[] nbr = ls.Nbr;
        int[] comp = b.Comp;
        int[] size = b.Size;
        int count = b.Count;
        int borderId = b.BorderId;
        int minArea = p.Regions.RegionOptions.MinRegionArea;

        // Regions as the contour stage sees them now: the first build's, with the removed spans out.
        int[] region = w.Region;
        Array.Copy(b.Region, region, n);
        var removed = new int[count + 2];
        for (int k = 0; k < dn; k++)
        {
            int s = delta[k];
            region[s] = 0;
            removed[comp[s]]++;
        }

        // A region that shrinks below the minimum size, or vanishes, is dropped or kept by rules that look at its neighbours: build from scratch.
        for (int c = 1; c <= count; c++)
        {
            if (removed[c] > 0 && (size[c] - removed[c] <= 0 || (minArea > 0 && size[c] - removed[c] < minArea)))
            {
                return null;
            }
        }

        // One pass over the removed spans' neighbours that are still walkable. Each is next to a removed span, so it is
        //  - a span of a small region that may have been kept only for a neighbour of another area (then: from scratch);
        //  - when of the removed span's own region, a "ring" span, for the connectivity check;
        //  - when in a region the contour stage traces, a start for the loop round the new boundary: the loop through its edge towards the removed span is traced
        //    again. An old loop that runs through any such loop's edges is gone. (An old loop that loses an edge of a removed span is found this way too: it
        //    reaches the removed span through a test that now fails, and that test is one of these edges.)
        int loops = b.Loops.Count;
        var gone = new bool[loops];
        var touched = new bool[count + 2];
        byte[] visited = w.Visited;
        Array.Clear(visited, 0, n);
        int guard = (4 * n) + 8;
        int[] sx = ls.X, sz = ls.Z, sy = ls.Y;
        ushort[] edgeLoop = b.EdgeLoop;
        int[] ringStamp = w.RingStamp;
        int id = ++w.StampId;
        w.RingCount = 0;
        List<TracedLoop>? fresh = null;
        for (int k = 0; k < dn; k++)
        {
            int s = delta[k];
            int cs = comp[s];
            int b4 = 4 * s;
            for (int d = 0; d < 4; d++)
            {
                int t = nbr[b4 + d];
                if (t < 0 || area[t] == 0)
                {
                    continue;
                }

                int ct = comp[t];
                if (ct > 0)
                {
                    if (minArea > 0 && size[ct] < minArea)
                    {
                        return null;
                    }

                    if (ct == cs && ringStamp[t] != id)
                    {
                        ringStamp[t] = id;
                        w.AddRing(t);
                    }
                }

                int r = region[t];
                int dir = (d + 2) & 3;
                if (r <= 0 || r == borderId || (visited[t] & (1 << dir)) != 0)
                {
                    continue;
                }

                ContourVertex[] raw = LayerTileBuilder.Trace(nbr, region, sx, sz, sy, visited, t, dir, guard, w);
                int[] keys = w.Keys;
                int lo = 0;
                for (int v = 1; v < raw.Length; v++)
                {
                    if (keys[v] < keys[lo])
                    {
                        lo = v;
                    }
                }

                // The loop starts at its smallest edge key, as a scan of the whole layer would find it.
                if (lo != 0)
                {
                    var rotated = new ContourVertex[raw.Length];
                    Array.Copy(raw, lo, rotated, 0, raw.Length - lo);
                    Array.Copy(raw, 0, rotated, raw.Length - lo, lo);
                    raw = rotated;
                }

                var loop = new TracedLoop(raw, Contours.SignedArea2(raw)) { Region = r, MinKey = keys[lo] };
                for (int v = 0; v < raw.Length; v++)
                {
                    int ol = edgeLoop[keys[v]];
                    if (ol != ushort.MaxValue)
                    {
                        gone[ol] = true;
                    }
                }

                touched[r] = true;
                (fresh ??= new List<TracedLoop>()).Add(loop);
            }
        }

        if (!StaysConnected(ls, b, area, w, removed, id))
        {
            return null;
        }

        // The first span of a region decides its number; if a removed span was the first one, the regions must still come in the same order.
        int[] first = b.First;
        int prevFirst = -1;
        for (int c = 1; c <= count; c++)
        {
            int f = first[c];
            if (removed[c] > 0 && area[f] == 0)
            {
                do
                {
                    f++;
                }
                while (f < n && !(comp[f] == c && area[f] != 0));
            }

            if (f <= prevFirst)
            {
                return null;
            }

            prevFirst = f;
        }

        ContourSettings settings = new() { MaxError = p.Regions.ContourOptions.MaxError, MaxEdgeLength = p.Regions.ContourOptions.MaxEdgeLength, KeepRawVertices = false, StraightRegionBorders = p.Regions.ContourOptions.StraightRegionBorders, BorderRegion = borderId };
        var result = new List<Contour>();
        int dropped = 0, droppedHoles = 0, fallbacks = 0;
        for (int r = 1; r <= count; r++)
        {
            if (!touched[r])
            {
                result.AddRange(b.RegionContours[r]);
                continue;
            }

            var outers = new List<TracedLoop>();
            var holes = new List<TracedLoop>();
            foreach (int li in b.RegionLoops[r])
            {
                if (!gone[li])
                {
                    TracedLoop l = b.Loops[li];
                    (l.Area2 > 0 ? outers : holes).Add(l);
                }
            }

            if (fresh is not null)
            {
                foreach (TracedLoop l in fresh)
                {
                    if (l.Region == r)
                    {
                        (l.Area2 > 0 ? outers : holes).Add(l);
                    }
                }
            }

            // Loops come in the order a scan would find them.
            outers.Sort(static (x, y) => x.MinKey.CompareTo(y.MinKey));
            holes.Sort(static (x, y) => x.MinKey.CompareTo(y.MinKey));
            Contours.AssembleRegion(r, outers.Count > 0 ? outers : null, holes.Count > 0 ? holes : null, b.RegionArea, settings, false, result, ref dropped, ref droppedHoles, ref fallbacks);
        }

        return new ContourSet(ls.Frame, result, borderId, dropped, droppedHoles, fallbacks);
    }

    // Removing spans from a region keeps it one piece when the walkable spans next to the removed ones can still reach each other inside the region: every
    // other span of the region reaches one of them (a path from it to a removed span passes one first). A search from one of them that finds all the others
    // proves it; a search that runs out of budget or of region says "perhaps not", and the layer is built from scratch. The ring spans of this repair are those
    // stamped with <paramref name="id"/> and listed in the work buffer.
    private static bool StaysConnected(LayerSpans ls, LayerBase b, byte[] area, LayerWork w, int[] removed, int id)
    {
        int[] nbr = ls.Nbr;
        int[] comp = b.Comp;
        int n = ls.Count;
        int[] stamp = w.Stamp, ringStamp = w.RingStamp, queue = w.Queue;
        int limit = Math.Max(MinSearch, n / 4);
        int[] ring = w.Ring;
        int rn = w.RingCount;
        for (int c = 1; c <= b.Count; c++)
        {
            if (removed[c] == 0)
            {
                continue;
            }

            int cnt = 0, firstRing = -1;
            for (int k = 0; k < rn; k++)
            {
                if (comp[ring[k]] == c)
                {
                    if (cnt++ == 0)
                    {
                        firstRing = ring[k];
                    }
                }
            }

            if (cnt <= 1)
            {
                continue;
            }

            // First through the band of spans next to the ring (two cells from a removed span): a blob's ring is nearly always joined there, and the search stays
            // small. If that fails the region may still be whole by a longer way round: search it all, within the budget.
            int visit = ++w.StampId;
            int found = Search(nbr, comp, area, stamp, ringStamp, queue, firstRing, c, id, visit, cnt, true, n);
            if (found < cnt)
            {
                visit = ++w.StampId;
                found = Search(nbr, comp, area, stamp, ringStamp, queue, firstRing, c, id, visit, cnt, false, limit);
            }

            if (found < cnt)
            {
                return false;
            }
        }

        return true;
    }

    // Breadth-first search inside region c from one ring span, counting the ring spans it reaches (the start counts). With band set it only enters spans that are
    // ring spans or next to one; otherwise it stops after entering more than limit spans.
    private static int Search(int[] nbr, int[] comp, byte[] area, int[] stamp, int[] ringStamp, int[] queue, int start, int c, int id, int visitId, int ring, bool band, int limit)
    {
        int head = 0, tail = 0;
        queue[tail++] = start;
        stamp[start] = visitId;
        int found = 1;
        while (head < tail && found < ring)
        {
            if (!band && tail > limit)
            {
                return 0;
            }

            int s = queue[head++];
            for (int d = 0; d < 4; d++)
            {
                int t = nbr[(4 * s) + d];
                if (t < 0 || stamp[t] == visitId || area[t] == 0 || comp[t] != c)
                {
                    continue;
                }

                if (band && ringStamp[t] != id && !NextToRing(nbr, ringStamp, t, id))
                {
                    continue;
                }

                stamp[t] = visitId;
                queue[tail++] = t;
                if (ringStamp[t] == id)
                {
                    found++;
                }
            }
        }

        return found;
    }

    private static bool NextToRing(int[] nbr, int[] ringStamp, int t, int id)
    {
        for (int d = 0; d < 4; d++)
        {
            int u = nbr[(4 * t) + d];
            if (u >= 0 && ringStamp[u] == id)
            {
                return true;
            }
        }

        return false;
    }
}
