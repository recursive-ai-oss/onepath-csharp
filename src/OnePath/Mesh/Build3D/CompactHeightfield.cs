using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>One open span of the compact heightfield: the free space above a walkable floor.</summary>
public struct CompactSpan
{
    /// <summary>Row of the floor (the top of the solid span below). The agent stands at this row.</summary>
    public int Y { get; set; }

    /// <summary>Rows of free space above the floor, capped at 65535.</summary>
    public int Height { get; set; }

    /// <summary>Area id of the floor; 0 means no longer walkable (erosion and area marking write here).</summary>
    public byte Area { get; set; }

    /// <summary>
    /// Neighbour links, 6 bits per direction (direction d in bits 6d..6d+5): the index of the target span within its cell, or
    /// <see cref="CompactHeightfield.NotConnected"/>. Read with <see cref="CompactHeightfield.GetConnection"/>.
    /// </summary>
    public int Connections { get; set; }

    /// <summary>Region id, written by the region stage; 0 until then.</summary>
    public int Region { get; set; }
}

/// <summary>
/// The compact open-span heightfield: only the walkable floors, stored cell by cell from the bottom up, each with the free
/// height above it and links to the reachable span in each of the four neighbour columns. This is the hand-off to the region
/// stage. Directions are 0 = −x, 1 = +z, 2 = +x, 3 = −z.
/// </summary>
public sealed class CompactHeightfield
{
    private static readonly int[] s_dirX = { -1, 0, 1, 0 };
    private static readonly int[] s_dirZ = { 0, 1, 0, -1 };

    /// <summary>Link value meaning "no neighbour".</summary>
    public const int NotConnected = 0x3f;

    /// <summary>x offset of each direction.</summary>
    public static ReadOnlySpan<int> DirX => s_dirX;

    /// <summary>z offset of each direction.</summary>
    public static ReadOnlySpan<int> DirZ => s_dirZ;

    private CompactHeightfield(HeightfieldFrame frame, int walkableHeight, int walkableClimb, int[] cellFirst, int[] cellCount, CompactSpan[] spans)
    {
        Frame = frame;
        WalkableHeight = walkableHeight;
        WalkableClimb = walkableClimb;
        CellFirstArray = cellFirst;
        CellCountArray = cellCount;
        SpansArray = spans;
    }

    /// <summary>The grid.</summary>
    public HeightfieldFrame Frame { get; }

    /// <summary>Clearance rows the links were built for.</summary>
    public int WalkableHeight { get; }

    /// <summary>Climb rows the links were built for.</summary>
    public int WalkableClimb { get; }

    /// <summary>Number of regions written by the region stage (ids 1..RegionCount); 0 before it ran.</summary>
    public int RegionCount { get; set; }

    /// <summary>Total number of spans.</summary>
    public int SpanCount => SpansArray.Length;

    /// <summary>Per column, the index of its first span in <see cref="Spans"/>.</summary>
    public ReadOnlySpan<int> CellFirst => CellFirstArray;

    /// <summary>Per column, how many spans it has.</summary>
    public ReadOnlySpan<int> CellCount => CellCountArray;

    /// <summary>All spans, column by column, each column from the bottom up. Modifiable: the later stages write areas and regions here.</summary>
    public Span<CompactSpan> Spans => SpansArray;

    internal int[] CellFirstArray { get; }

    internal int[] CellCountArray { get; }

    internal CompactSpan[] SpansArray { get; }

    /// <summary>An independent copy: the same grid and spans, so the later stages can write areas and regions without touching this one.</summary>
    public CompactHeightfield Clone() => new(Frame, WalkableHeight, WalkableClimb, CellFirstArray, CellCountArray, (CompactSpan[])SpansArray.Clone()) { RegionCount = RegionCount };

    /// <summary>The link of <paramref name="span"/> in direction <paramref name="dir"/> (0..3): the target's index within its cell, or <see cref="NotConnected"/>.</summary>
    public static int GetConnection(in CompactSpan span, int dir) => (span.Connections >> (dir * 6)) & 0x3f;

    /// <summary>
    /// Builds the compact heightfield from the filtered solid heightfield: every span with a non-zero area becomes an open
    /// span above it; two spans are linked when they are in orthogonally adjacent columns, their floors differ by at most
    /// <paramref name="walkableClimb"/> rows and the free space they share is at least <paramref name="walkableHeight"/> rows. When
    /// several spans of a neighbour cell qualify, the lowest is linked. A link to a cell index of 63 or more is dropped.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">walkableHeight is below 1 or walkableClimb is negative.</exception>
    public static CompactHeightfield Build(SolidHeightfield solid, int walkableHeight, int walkableClimb)
    {
        ArgumentNullException.ThrowIfNull(solid);
        if (walkableHeight < 1)
        {
            throw new ArgumentOutOfRangeException(nameof(walkableHeight), walkableHeight, "The walkable height must be at least 1 row.");
        }
        if (walkableClimb < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(walkableClimb), walkableClimb, "The walkable climb must not be negative.");
        }
        HeightfieldFrame f = solid.Frame;
        ReadOnlySpan<int> heads = solid.Heads;
        int cols = f.ColumnCount;
        int total = solid.CountWalkable();
        var first = new int[cols];
        var count = new int[cols];
        var spans = new CompactSpan[total];
        SolidSpan[] raw = solid.RawSpans;
        int n = 0;
        for (int c = 0; c < cols; c++)
        {
            first[c] = n;
            for (int i = heads[c]; i != -1;)
            {
                ref SolidSpan s = ref raw[i];
                int next = s.Next;
                if (s.Area != 0)
                {
                    int h = next == -1 ? 65535 : Math.Min(65535, raw[next].Min - s.Max);
                    ref CompactSpan o = ref spans[n++];
                    o.Y = s.Max;
                    o.Height = h;
                    o.Area = s.Area;
                    o.Connections = AllUnconnected;
                }
                i = next;
            }
            count[c] = n - first[c];
        }

        int width = f.Width;
        int depth = f.Depth;
        for (int z = 0; z < depth; z++)
        {
            for (int x = 0; x < width; x++)
            {
                int c = (z * width) + x;
                int end = first[c] + count[c];
                for (int k = first[c]; k < end; k++)
                {
                    ref CompactSpan s = ref spans[k];
                    int conn = AllUnconnected;
                    // Columns beyond the grid have no spans. Directions: 0 = -x, 1 = +z, 2 = +x, 3 = -z.
                    if (x > 0)
                    {
                        conn = Link(conn, 0, spans, first, count, c - 1, in s, walkableHeight, walkableClimb);
                    }
                    if (z < depth - 1)
                    {
                        conn = Link(conn, 1, spans, first, count, c + width, in s, walkableHeight, walkableClimb);
                    }
                    if (x < width - 1)
                    {
                        conn = Link(conn, 2, spans, first, count, c + 1, in s, walkableHeight, walkableClimb);
                    }
                    if (z > 0)
                    {
                        conn = Link(conn, 3, spans, first, count, c - width, in s, walkableHeight, walkableClimb);
                    }
                    s.Connections = conn;
                }
            }
        }
        return new CompactHeightfield(f, walkableHeight, walkableClimb, first, count, spans);
    }

    // The lowest span of column nc that span s can step to, written into direction dir of conn; conn unchanged when there is none.
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private static int Link(int conn, int dir, CompactSpan[] spans, int[] first, int[] count, int nc, in CompactSpan s, int walkableHeight, int walkableClimb)
    {
        int limit = Math.Min(count[nc], NotConnected);
        int nf = first[nc];
        int sTop = s.Y + s.Height;
        for (int m = 0; m < limit; m++)
        {
            ref readonly CompactSpan t = ref spans[nf + m];
            int bottom = Math.Max(s.Y, t.Y);
            int top = Math.Min(sTop, t.Y + t.Height);
            if (top - bottom >= walkableHeight && Math.Abs(t.Y - s.Y) <= walkableClimb)
            {
                return (conn & ~(0x3f << (dir * 6))) | (m << (dir * 6));
            }
        }
        return conn;
    }

    private const int AllUnconnected = NotConnected | (NotConnected << 6) | (NotConnected << 12) | (NotConnected << 18);
}
