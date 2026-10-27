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
        HeightfieldFrame f = chf.Frame;
        Span<CompactSpan> spans = chf.Spans;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> count = chf.CellCount;
        var dist = new int[spans.Length];
        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                for (int i = first[c]; i < first[c] + count[c]; i++)
                {
                    dist[i] = IsBoundary(chf, x, z, i) ? 0 : cap;
                }
            }
        }

        // Pull relaxation to a fixed point. A shortest path below the cap has at most cap / 2 steps, so the
        // loop ends within that many passes; the fixed point is unique, hence independent of the sweep order.
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (int z = 0; z < f.Depth; z++)
            {
                for (int x = 0; x < f.Width; x++)
                {
                    int c = f.Column(x, z);
                    for (int i = first[c]; i < first[c] + count[c]; i++)
                    {
                        if (dist[i] == 0)
                        {
                            continue;
                        }
                        int best = dist[i];
                        for (int d = 0; d < 4; d++)
                        {
                            int con = CompactHeightfield.GetConnection(spans[i], d);
                            if (con == CompactHeightfield.NotConnected)
                            {
                                continue;
                            }
                            int nx = x + CompactHeightfield.DirX[d];
                            int nz = z + CompactHeightfield.DirZ[d];
                            int n = first[f.Column(nx, nz)] + con;
                            best = Math.Min(best, Math.Min(cap, dist[n] + OrthogonalCost));
                            for (int e = 1; e < 4; e += 2)
                            {
                                int d2 = (d + e) & 3;
                                int con2 = CompactHeightfield.GetConnection(spans[n], d2);
                                if (con2 == CompactHeightfield.NotConnected)
                                {
                                    continue;
                                }
                                int mx = nx + CompactHeightfield.DirX[d2];
                                int mz = nz + CompactHeightfield.DirZ[d2];
                                int m = first[f.Column(mx, mz)] + con2;
                                best = Math.Min(best, Math.Min(cap, dist[m] + DiagonalCost));
                            }
                        }
                        if (best < dist[i])
                        {
                            dist[i] = best;
                            changed = true;
                        }
                    }
                }
            }
        }
        return dist;
    }

    private static bool IsBoundary(CompactHeightfield chf, int x, int z, int i)
    {
        Span<CompactSpan> spans = chf.Spans;
        if (spans[i].Area == 0)
        {
            return true;
        }
        HeightfieldFrame f = chf.Frame;
        for (int d = 0; d < 4; d++)
        {
            int con = CompactHeightfield.GetConnection(spans[i], d);
            if (con == CompactHeightfield.NotConnected)
            {
                return true;
            }
            int n = chf.CellFirst[f.Column(x + CompactHeightfield.DirX[d], z + CompactHeightfield.DirZ[d])] + con;
            if (spans[n].Area == 0)
            {
                return true;
            }
        }
        return false;
    }
}
