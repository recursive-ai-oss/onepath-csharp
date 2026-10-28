using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// The span filters, applied to the solid heightfield before it is compacted. Each filter only ever changes a
/// span's area id (to 0 or, for low obstacles, to the area of the span below). Definitions are in docs/build3d-spans.md.
/// </summary>
public static class SpanFilters
{
    /// <summary>The ceiling above a column's top span: effectively unbounded.</summary>
    public const int Unbounded = 1 << 30;

    /// <summary>
    /// Low-hanging obstacles: a non-walkable span whose top is at most <paramref name="walkableClimb"/> rows above the top
    /// of the walkable span directly below it becomes walkable with that span's area (kerbs, stair edges, clutter an agent steps over).
    /// Decisions use the areas as they were before the filter ran.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">walkableClimb is negative.</exception>
    public static void FilterLowHangingWalkableObstacles(SolidHeightfield hf, int walkableClimb)
    {
        ArgumentNullException.ThrowIfNull(hf);
        if (walkableClimb < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(walkableClimb), walkableClimb, "The walkable climb must not be negative.");
        }
        ReadOnlySpan<int> heads = hf.Heads;
        for (int col = 0; col < heads.Length; col++)
        {
            bool prevWalkable = false;
            int prevTop = 0;
            byte prevArea = 0;
            for (int i = heads[col]; i != -1;)
            {
                ref SolidSpan s = ref hf.Span(i);
                byte original = s.Area;
                if (original == 0 && prevWalkable && s.Max - prevTop <= walkableClimb)
                {
                    s.Area = prevArea;
                }
                prevWalkable = original != 0;
                prevTop = s.Max;
                prevArea = original;
                i = s.Next;
            }
        }
    }

    /// <summary>
    /// Ledge spans: a walkable span becomes non-walkable when an agent standing on it could fall off, or when the
    /// ground around it is too uneven to be one surface. For each of the four neighbour columns, every gap an agent of
    /// <paramref name="walkableHeight"/> could stand in next to the span (sharing at least that much clearance) is a
    /// neighbour floor; a floor more than <paramref name="walkableClimb"/> below, or a column edge with no floor at all
    /// (outside the grid, or empty), makes the span a ledge; so does a spread of accessible neighbour floors
    /// (within the climb of this span) larger than the climb.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">walkableHeight is below 1 or walkableClimb is negative.</exception>
    public static void FilterLedgeSpans(SolidHeightfield hf, int walkableHeight, int walkableClimb)
    {
        ArgumentNullException.ThrowIfNull(hf);
        CheckHeightClimb(walkableHeight, walkableClimb);
        HeightfieldFrame f = hf.Frame;
        int[] heads = hf.RawHeads;
        SolidSpan[] spans = hf.RawSpans;
        int width = f.Width;
        int depth = f.Depth;
        for (int z = 0; z < depth; z++)
        {
            for (int x = 0; x < width; x++)
            {
                for (int i = heads[x + (z * width)]; i != -1;)
                {
                    ref SolidSpan s = ref spans[i];
                    int next = s.Next;
                    if (s.Area != 0)
                    {
                        int floor = s.Max;
                        int ceiling = next == -1 ? Unbounded : spans[next].Min;
                        if (IsLedge(heads, spans, width, depth, x, z, floor, ceiling, walkableHeight, walkableClimb))
                        {
                            s.Area = 0;
                        }
                    }
                    i = next;
                }
            }
        }
    }

    /// <summary>
    /// Low-height spans: a walkable span with less than <paramref name="walkableHeight"/> rows of clearance between its top and the
    /// bottom of the next span above becomes non-walkable. The top span of a column has unbounded clearance.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">walkableHeight is below 1.</exception>
    public static void FilterWalkableLowHeightSpans(SolidHeightfield hf, int walkableHeight)
    {
        ArgumentNullException.ThrowIfNull(hf);
        CheckHeightClimb(walkableHeight, 0);
        ReadOnlySpan<int> heads = hf.Heads;
        for (int col = 0; col < heads.Length; col++)
        {
            for (int i = heads[col]; i != -1;)
            {
                ref SolidSpan s = ref hf.Span(i);
                int next = s.Next;
                if (s.Area != 0 && next != -1 && hf.Span(next).Min - s.Max < walkableHeight)
                {
                    s.Area = 0;
                }
                i = next;
            }
        }
    }

    // Raw accesses: a column index is checked against the grid before it is used, and the chain indices come from the heads and the Next links, which are
    // below the used count of the span store.
    private static bool IsLedge(int[] heads, SolidSpan[] spans, int width, int depth, int x, int z, int floor, int ceiling, int height, int climb)
    {
        int minAccessible = Unbounded;
        int maxAccessible = -Unbounded;
        for (int dir = 0; dir < 4; dir++)
        {
            // Direction order does not matter: any drop below the climb (or a missing column) makes a ledge, and the spread is a minimum and a maximum.
            int col = dir switch
            {
                0 => x > 0 ? (x - 1) + (z * width) : -1,
                1 => z < depth - 1 ? x + ((z + 1) * width) : -1,
                2 => x < width - 1 ? (x + 1) + (z * width) : -1,
                _ => z > 0 ? x + ((z - 1) * width) : -1,
            };
            if (col < 0)
            {
                // Beyond the grid there is nothing to stand on.
                return true;
            }
            int first = Raw.At(heads, col);
            // The gap below the column's first span is a floor at -climb: it counts as a drop for anything that stands higher.
            int nFloor = -climb;
            int nCeil = first == -1 ? Unbounded : Raw.At(spans, first).Min;
            int cur = first;
            while (true)
            {
                if (Math.Min(ceiling, nCeil) - Math.Max(floor, nFloor) >= height)
                {
                    int diff = nFloor - floor;
                    if (diff < -climb)
                    {
                        return true;
                    }
                    if (Math.Abs(diff) <= climb)
                    {
                        minAccessible = Math.Min(minAccessible, nFloor);
                        maxAccessible = Math.Max(maxAccessible, nFloor);
                    }
                }
                if (cur == -1)
                {
                    break;
                }
                ref readonly SolidSpan ns = ref Raw.At(spans, cur);
                nFloor = ns.Max;
                nCeil = ns.Next == -1 ? Unbounded : Raw.At(spans, ns.Next).Min;
                cur = ns.Next;
            }
        }
        return minAccessible != Unbounded && maxAccessible - minAccessible > climb;
    }

    private static void CheckHeightClimb(int height, int climb)
    {
        if (height < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(height), height, "The walkable height must be at least 1 row.");
        }
        if (climb < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(climb), climb, "The walkable climb must not be negative.");
        }
    }
}
