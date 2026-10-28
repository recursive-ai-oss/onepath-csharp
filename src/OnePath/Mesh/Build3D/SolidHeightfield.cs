using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>One run of solid voxels in a column: rows <see cref="Min"/> up to but not including <see cref="Max"/>.</summary>
public struct SolidSpan
{
    /// <summary>First solid row (inclusive).</summary>
    public int Min { get; set; }

    /// <summary>One past the last solid row. The span's top surface, the walkable floor, is at row <see cref="Max"/>.</summary>
    public int Max { get; set; }

    /// <summary>Area id of the span's top surface; 0 means not walkable.</summary>
    public byte Area { get; set; }

    /// <summary>Index of the next span up in the same column, or -1.</summary>
    public int Next { get; set; }
}

/// <summary>
/// The solid heightfield: for every column of the frame, a list of disjoint solid spans sorted from the
/// bottom up. Spans that overlap or touch are merged when added (see docs/build3d-spans.md). This is the raw
/// output of rasterization and the input of the span filters.
/// </summary>
public sealed class SolidHeightfield
{
    private readonly int[] _heads;
    private SolidSpan[] _spans;
    private int _used;
    private int _freeList = -1;
    private int _freeCount;

    /// <summary>Creates an empty heightfield on <paramref name="frame"/>.</summary>
    public SolidHeightfield(HeightfieldFrame frame)
    {
        ArgumentNullException.ThrowIfNull(frame);
        Frame = frame;
        _heads = new int[frame.ColumnCount];
        // A walkable level has about one span per column: start there instead of doubling up to it.
        _spans = new SolidSpan[Math.Clamp(frame.ColumnCount, 64, 1 << 20)];
        Array.Fill(_heads, -1);
    }

    /// <summary>The grid this heightfield lives on.</summary>
    public HeightfieldFrame Frame { get; }

    /// <summary>Number of live spans.</summary>
    public int SpanCount => _used - _freeCount;

    /// <summary>Per column, the index of its lowest span or -1. Indexed by <see cref="HeightfieldFrame.Column"/>.</summary>
    public ReadOnlySpan<int> Heads => _heads;

    /// <summary>Reference to span <paramref name="index"/>. Only indices reachable from <see cref="Heads"/> and <see cref="SolidSpan.Next"/> are live.</summary>
    public ref SolidSpan Span(int index)
    {
        if ((uint)index >= (uint)_used)
        {
            throw new ArgumentOutOfRangeException(nameof(index), index, "No such span.");
        }
        return ref _spans[index];
    }

    // The span store and column heads for the filters' inner loops: indices reachable from the heads are below _used by construction.
    internal SolidSpan[] RawSpans => _spans;

    /// <summary>The number of live spans with a non-zero area.</summary>
    internal int CountWalkable()
    {
        int total = 0;
        SolidSpan[] spans = _spans;
        for (int i = 0; i < _used; i++)
        {
            if (spans[i].Area != 0)
            {
                total++;
            }
        }
        return total;
    }

    /// <summary>Removes every span, keeping the allocated storage.</summary>
    public void Clear()
    {
        Array.Fill(_heads, -1);
        _used = 0;
        _freeList = -1;
        _freeCount = 0;
    }

    /// <summary>
    /// Adds the solid rows [<paramref name="min"/>, <paramref name="max"/>) to a column, merging with every span it
    /// overlaps or touches.
    /// The merged span's area is that of whichever input has the higher top; when the two tops differ by at most
    /// <paramref name="mergeThreshold"/> rows the larger area id wins instead (0 is "not walkable", so a walkable
    /// surface survives a thin obstacle).
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The column or the rows are outside the grid, or the span is empty.</exception>
    public void AddSpan(int column, int min, int max, byte area, int mergeThreshold)
    {
        if ((uint)column >= (uint)_heads.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(column), column, "No such column.");
        }
        if (min < 0 || max > Frame.Height || min >= max)
        {
            throw new ArgumentOutOfRangeException(nameof(min), $"Span rows [{min}, {max}) must satisfy 0 <= min < max <= {Frame.Height}.");
        }
        AddSpanUnchecked(column, min, max, area, mergeThreshold);
    }

    /// <summary>AddSpan for callers that have already established <c>0 &lt;= column &lt; ColumnCount</c> and <c>0 &lt;= min &lt; max &lt;= Frame.Height</c>.</summary>
    internal void AddSpanUnchecked(int column, int min, int max, byte area, int mergeThreshold)
    {
        int newMin = min;
        int newMax = max;
        byte newArea = area;
        int prev = -1;
        int cur = _heads[column];
        while (cur != -1)
        {
            ref SolidSpan s = ref _spans[cur];
            if (s.Min > newMax)
            {
                break;
            }
            if (s.Max < newMin)
            {
                prev = cur;
                cur = s.Next;
                continue;
            }
            // The span with the higher top decides the area: the top is the walkable surface.
            if (s.Max > newMax)
            {
                newArea = s.Max - newMax <= mergeThreshold ? Math.Max(newArea, s.Area) : s.Area;
                newMax = s.Max;
            }
            else if (newMax - s.Max <= mergeThreshold)
            {
                newArea = Math.Max(newArea, s.Area);
            }
            if (s.Min < newMin)
            {
                newMin = s.Min;
            }
            int next = s.Next;
            s.Next = _freeList;
            s.Area = 0; // a freed span never counts as walkable, so CountWalkable can scan the whole store
            _freeList = cur;
            _freeCount++;
            if (prev == -1)
            {
                _heads[column] = next;
            }
            else
            {
                _spans[prev].Next = next;
            }
            cur = next;
        }

        int idx;
        if (_freeList != -1)
        {
            idx = _freeList;
            _freeList = _spans[idx].Next;
            _freeCount--;
        }
        else
        {
            if (_used == _spans.Length)
            {
                Array.Resize(ref _spans, _spans.Length * 2);
            }
            idx = _used++;
        }
        _spans[idx] = new SolidSpan { Min = newMin, Max = newMax, Area = newArea, Next = cur };
        if (prev == -1)
        {
            _heads[column] = idx;
        }
        else
        {
            _spans[prev].Next = idx;
        }
    }
}
