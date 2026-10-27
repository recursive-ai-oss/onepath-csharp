using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Mesh.Build3D;

namespace RecursiveAI.OnePath.Tests.Build3D;

/// <summary>Small deterministic generator for tests (xorshift64*).</summary>
internal sealed class TestRng
{
    private ulong _s;

    public TestRng(ulong seed) => _s = (seed * 0x9E3779B97F4A7C15UL) | 1UL;

    public ulong Next()
    {
        _s ^= _s >> 12;
        _s ^= _s << 25;
        _s ^= _s >> 27;
        return _s * 0x2545F4914F6CDD1DUL;
    }

    public int Int(int n) => (int)((Next() >> 33) % (ulong)n);

    public double Double() => (Next() >> 11) / (double)(1UL << 53);

    public double Range(double lo, double hi) => lo + ((hi - lo) * Double());
}

/// <summary>Plain-data copy of a solid heightfield for comparisons.</summary>
internal static class Dump
{
    public static List<(int Min, int Max, byte Area)>[] Columns(SolidHeightfield hf)
    {
        var cols = new List<(int, int, byte)>[hf.Frame.ColumnCount];
        for (int c = 0; c < cols.Length; c++)
        {
            cols[c] = new List<(int, int, byte)>();
            for (int i = hf.Heads[c]; i != -1; i = hf.Span(i).Next)
            {
                cols[c].Add((hf.Span(i).Min, hf.Span(i).Max, hf.Span(i).Area));
            }
        }
        return cols;
    }

    public static void Expect(List<(int Min, int Max, byte Area)> col, params (int Min, int Max, int Area)[] expected)
    {
        Xunit.Assert.Equal(expected.Length, col.Count);
        for (int i = 0; i < expected.Length; i++)
        {
            Xunit.Assert.Equal(expected[i], (col[i].Min, col[i].Max, (int)col[i].Area));
        }
    }

    public static string Text(SolidHeightfield hf)
    {
        var sb = new System.Text.StringBuilder();
        var cols = Columns(hf);
        for (int c = 0; c < cols.Length; c++)
        {
            foreach (var (a, b, ar) in cols[c])
            {
                sb.Append(c).Append(':').Append(a).Append('-').Append(b).Append('/').Append(ar).Append(' ');
            }
        }
        return sb.ToString();
    }

    public static string Text(CompactHeightfield c)
    {
        var sb = new System.Text.StringBuilder();
        for (int i = 0; i < c.SpanCount; i++)
        {
            CompactSpan s = c.Spans[i];
            sb.Append(s.Y).Append(',').Append(s.Height).Append(',').Append(s.Area).Append(',').Append(s.Connections).Append(';');
        }
        for (int i = 0; i < c.CellCount.Length; i++)
        {
            sb.Append(c.CellCount[i]).Append(',');
        }
        return sb.ToString();
    }
}
