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
        ReadOnlySpan<int> heads = hf.Heads;
        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                for (int i = heads[f.Column(x, z)]; i != -1;)
                {
                    ref SolidSpan s = ref hf.Span(i);
                    int next = s.Next;
                    if (s.Area != 0)
                    {
                        int floor = s.Max;
                        int ceiling = next == -1 ? Unbounded : hf.Span(next).Min;
                        if (IsLedge(hf, x, z, floor, ceiling, walkableHeight, walkableClimb))
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

    private static bool IsLedge(SolidHeightfield hf, int x, int z, int floor, int ceiling, int height, int climb)
    {
        HeightfieldFrame f = hf.Frame;
        int lowestDrop = Unbounded;
        int minAccessible = Unbounded;
        int maxAccessible = -Unbounded;
        for (int dir = 0; dir < 4; dir++)
        {
            int nx = x + CompactHeightfield.DirX[dir];
            int nz = z + CompactHeightfield.DirZ[dir];
            if ((uint)nx >= (uint)f.Width || (uint)nz >= (uint)f.Depth)
            {
                // Beyond the grid there is nothing to stand on.
                lowestDrop = Math.Min(lowestDrop, -climb - 1);
                continue;
            }
            int first = hf.Heads[f.Column(nx, nz)];
            // The gap below the column's first span is a floor at -climb: it counts as a drop for anything that stands higher.
            int nFloor = -climb;
            int nCeil = first == -1 ? Unbounded : hf.Span(first).Min;
            int cur = first;
            while (true)
            {
                if (Math.Min(ceiling, nCeil) - Math.Max(floor, nFloor) >= height)
                {
                    int diff = nFloor - floor;
                    lowestDrop = Math.Min(lowestDrop, diff);
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
                ref readonly SolidSpan ns = ref hf.Span(cur);
                nFloor = ns.Max;
                nCeil = ns.Next == -1 ? Unbounded : hf.Span(ns.Next).Min;
                cur = ns.Next;
            }
        }
        if (lowestDrop < -climb)
        {
            return true;
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
