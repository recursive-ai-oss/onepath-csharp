using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>Erosion of the walkable area by the agent radius.</summary>
public static class Erosion
{
    /// <summary>Cost of a step to an orthogonal neighbour. Distances are in these units: a cell is 2, a diagonal step is 3 (≈ 2·√2 rounded).</summary>
    public const int OrthogonalCost = 2;

    /// <summary>Cost of a diagonal step (an orthogonal link followed by a perpendicular one).</summary>
    public const int DiagonalCost = 3;

    /// <summary>
    /// Sets the area of every span closer than <paramref name="radius"/> cells to a boundary to 0. A span is on the boundary
    /// when its area is already 0 or it lacks a link, or links to a span with area 0, in any of the four directions. The
    /// distance of a span is the exact shortest path to a boundary span over the span links, with orthogonal steps costing
    /// <see cref="OrthogonalCost"/> and diagonal steps (two perpendicular links, in either order) <see cref="DiagonalCost"/>;
    /// a span is eroded when that distance is below <c>2 * radius</c>. The result does not depend on any traversal order.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">radius is negative.</exception>
    public static void ErodeWalkableArea(CompactHeightfield chf, int radius)
    {
        ArgumentNullException.ThrowIfNull(chf);
        if (radius < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(radius), radius, "The radius must not be negative.");
        }
        if (radius == 0)
        {
            return;
        }
        int[] dist = ComputeBoundaryDistance(chf, 2L * radius > int.MaxValue / 2 ? int.MaxValue / 2 : 2 * radius);
        int cap = 2 * radius;
        Span<CompactSpan> spans = chf.Spans;
        for (int i = 0; i < spans.Length; i++)
        {
            if (dist[i] < cap)
            {
                spans[i].Area = 0;
            }
        }
    }

    /// <summary>
    /// The distance of every span to the nearest boundary span, capped at <paramref name="cap"/> (spans farther than the
    /// cap report the cap). See <see cref="ErodeWalkableArea"/> for the definition.
    /// </summary>
    public static int[] ComputeBoundaryDistance(CompactHeightfield chf, int cap)
    {
        ArgumentNullException.ThrowIfNull(chf);
        if (cap < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(cap), cap, "The cap must be at least 1.");
        }
        return ComputeBoundaryDistance(chf, SpanLinks.Build(chf), cap);
    }

    /// <summary>The distance field from the raw links (<see cref="SpanLinks.Build"/>), so a caller that needs both can build them once.</summary>
    internal static int[] ComputeBoundaryDistance(CompactHeightfield chf, int[] link, int cap)
    {
        Span<CompactSpan> spans = chf.Spans;
        int n = spans.Length;
        int[] dist = GC.AllocateUninitializedArray<int>(n);
        for (int i = 0; i < n; i++)
        {
            dist[i] = IsBoundary(spans, link, i) ? 0 : cap;
        }

        // Pull relaxation to a fixed point. The shortest-path distance (the unique fixed point) does not depend on the order of the passes,
        // so a forward and a backward pass over half of the neighbours (the chamfer masks) do most of the work, and passes over all of them,
        // alternating direction, finish it and prove it: the loop ends with a pass that changes nothing.
        Relax(dist, link, forward: true, half: true);
        Relax(dist, link, forward: false, half: true);
        bool forwardPass = true;
        while (Relax(dist, link, forwardPass, half: false))
        {
            forwardPass = !forwardPass;
        }
        return dist;
    }

    // One pass of dist[i] = min(dist[i], dist[n] + 2 for a link i → n, dist[m] + 3 for a link n → m perpendicular to i → n). Returns whether anything changed.
    // Half masks: the forward pass looks at -x, -z and the diagonals of them, the backward pass at +x, +z and theirs (the other half is not yet final).
    private static bool Relax(int[] dist, int[] link, bool forward, bool half)
    {
        bool changed = false;
        int count = dist.Length;
        for (int k = 0; k < count; k++)
        {
            int i = forward ? k : count - 1 - k;
            int cur = dist[i];
            if (cur == 0)
            {
                continue;
            }
            int best = cur;
            int b = 4 * i;
            if (half)
            {
                if (forward)
                {
                    // -x (0) with the -z hop; -z (3) with the -x and +x hops.
                    int n0 = link[b];
                    if (n0 >= 0)
                    {
                        best = Math.Min(best, dist[n0] + OrthogonalCost);
                        int m = link[(4 * n0) + 3];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                    }
                    int n3 = link[b + 3];
                    if (n3 >= 0)
                    {
                        best = Math.Min(best, dist[n3] + OrthogonalCost);
                        int m = link[4 * n3];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                        m = link[(4 * n3) + 2];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                    }
                }
                else
                {
                    // +x (2) with the +z hop; +z (1) with the +x and -x hops.
                    int n2 = link[b + 2];
                    if (n2 >= 0)
                    {
                        best = Math.Min(best, dist[n2] + OrthogonalCost);
                        int m = link[(4 * n2) + 1];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                    }
                    int n1 = link[b + 1];
                    if (n1 >= 0)
                    {
                        best = Math.Min(best, dist[n1] + OrthogonalCost);
                        int m = link[(4 * n1) + 2];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                        m = link[4 * n1];
                        if (m >= 0)
                        {
                            best = Math.Min(best, dist[m] + DiagonalCost);
                        }
                    }
                }
            }
            else
            {
                for (int d = 0; d < 4; d++)
                {
                    int nb = link[b + d];
                    if (nb < 0)
                    {
                        continue;
                    }
                    best = Math.Min(best, dist[nb] + OrthogonalCost);
                    int m1 = link[(4 * nb) + ((d + 1) & 3)];
                    if (m1 >= 0)
                    {
                        best = Math.Min(best, dist[m1] + DiagonalCost);
                    }
                    int m3 = link[(4 * nb) + ((d + 3) & 3)];
                    if (m3 >= 0)
                    {
                        best = Math.Min(best, dist[m3] + DiagonalCost);
                    }
                }
            }
            if (best < cur)
            {
                dist[i] = best;
                changed = true;
            }
        }
        return changed;
    }

    private static bool IsBoundary(Span<CompactSpan> spans, int[] link, int i)
    {
        if (spans[i].Area == 0)
        {
            return true;
        }
        for (int d = 0; d < 4; d++)
        {
            int n = link[(4 * i) + d];
            if (n < 0 || spans[n].Area == 0)
            {
                return true;
            }
        }
        return false;
    }
}

/// <summary>The links of a compact heightfield as a flat array: for span i and direction d, <c>link[4 * i + d]</c> is the index of the linked span or -1.</summary>
internal static class SpanLinks
{
    /// <summary>Resolves every span's per-cell link indices to span indices. The links may be one-way; see <see cref="SpanGraph"/> for the mutual subset.</summary>
    public static int[] Build(CompactHeightfield chf)
    {
        HeightfieldFrame f = chf.Frame;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> count = chf.CellCount;
        Span<CompactSpan> spans = chf.Spans;
        int[] link = GC.AllocateUninitializedArray<int>(4 * spans.Length);
        int w = f.Width;
        int cols = f.ColumnCount;
        for (int c = 0; c < cols; c++)
        {
            int end = first[c] + count[c];
            for (int i = first[c]; i < end; i++)
            {
                int conn = spans[i].Connections;
                int b = 4 * i;
                int con = conn & 0x3f;
                link[b] = con == CompactHeightfield.NotConnected ? -1 : first[c - 1] + con;
                con = (conn >> 6) & 0x3f;
                link[b + 1] = con == CompactHeightfield.NotConnected ? -1 : first[c + w] + con;
                con = (conn >> 12) & 0x3f;
                link[b + 2] = con == CompactHeightfield.NotConnected ? -1 : first[c + 1] + con;
                con = (conn >> 18) & 0x3f;
                link[b + 3] = con == CompactHeightfield.NotConnected ? -1 : first[c - w] + con;
            }
        }
        return link;
    }
}
