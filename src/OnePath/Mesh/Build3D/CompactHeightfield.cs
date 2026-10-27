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
        int total = 0;
        for (int c = 0; c < cols; c++)
        {
            for (int i = heads[c]; i != -1; i = solid.Span(i).Next)
            {
                if (solid.Span(i).Area != 0)
                {
                    total++;
                }
            }
        }
        var first = new int[cols];
        var count = new int[cols];
        var spans = new CompactSpan[total];
        int n = 0;
        for (int c = 0; c < cols; c++)
        {
            first[c] = n;
            for (int i = heads[c]; i != -1;)
            {
                ref SolidSpan s = ref solid.Span(i);
                if (s.Area != 0)
                {
                    int h = s.Next == -1 ? 65535 : Math.Min(65535, solid.Span(s.Next).Min - s.Max);
                    spans[n++] = new CompactSpan { Y = s.Max, Height = h, Area = s.Area, Connections = AllUnconnected, Region = 0 };
                    count[c]++;
                }
                i = s.Next;
            }
        }

        for (int z = 0; z < f.Depth; z++)
        {
            for (int x = 0; x < f.Width; x++)
            {
                int c = f.Column(x, z);
                for (int k = first[c]; k < first[c] + count[c]; k++)
                {
                    ref CompactSpan s = ref spans[k];
                    for (int dir = 0; dir < 4; dir++)
                    {
                        int nx = x + DirX[dir];
                        int nz = z + DirZ[dir];
                        if ((uint)nx >= (uint)f.Width || (uint)nz >= (uint)f.Depth)
                        {
                            continue;
                        }
                        int nc = f.Column(nx, nz);
                        for (int m = 0; m < count[nc] && m < NotConnected; m++)
                        {
                            ref readonly CompactSpan t = ref spans[first[nc] + m];
                            int bottom = Math.Max(s.Y, t.Y);
                            int top = Math.Min(s.Y + s.Height, t.Y + t.Height);
                            if (top - bottom >= walkableHeight && Math.Abs(t.Y - s.Y) <= walkableClimb)
                            {
                                s.Connections = (s.Connections & ~(0x3f << (dir * 6))) | (m << (dir * 6));
                                break;
                            }
                        }
                    }
                }
            }
        }
        return new CompactHeightfield(f, walkableHeight, walkableClimb, first, count, spans);
    }

    private const int AllUnconnected = NotConnected | (NotConnected << 6) | (NotConnected << 12) | (NotConnected << 18);
}
