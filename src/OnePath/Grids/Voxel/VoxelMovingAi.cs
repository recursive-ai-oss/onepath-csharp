using System;
using System.Collections.Generic;
using System.Globalization;

namespace RecursiveAI.OnePath.Grids.Voxel;

/// <summary>
/// A parsed Moving AI Lab voxel map (<c>.3dmap</c>, https://movingai.com/benchmarks/voxels.html): a header line
/// <c>voxel X Y Z</c> with the dimensions, then one line <c>x y z</c> for every filled (solid) voxel. Every voxel not
/// listed is open space. Coordinates are zero-based and must lie inside the dimensions.
/// </summary>
public sealed class VoxelMap
{
    private readonly int[] _filled;

    private VoxelMap(int sizeX, int sizeY, int sizeZ, int[] filled)
    {
        SizeX = sizeX;
        SizeY = sizeY;
        SizeZ = sizeZ;
        _filled = filled;
    }

    /// <summary>Voxels along x.</summary>
    public int SizeX { get; }

    /// <summary>Voxels along y.</summary>
    public int SizeY { get; }

    /// <summary>Voxels along z.</summary>
    public int SizeZ { get; }

    /// <summary>Number of lines that listed a filled voxel (a voxel listed twice counts twice).</summary>
    public int FilledCount => _filled.Length;

    /// <summary>Parses map text.</summary>
    /// <param name="text">The file contents.</param>
    /// <returns>The map.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="GridFormatException">The text is not a valid voxel map.</exception>
    public static VoxelMap Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        return Parse(VoxelScenarios.Ascii(text));
    }

    /// <summary>Parses map bytes (ASCII).</summary>
    /// <param name="data">The file contents.</param>
    /// <returns>The map.</returns>
    /// <exception cref="GridFormatException">The data is not a valid voxel map.</exception>
    public static VoxelMap Parse(ReadOnlySpan<byte> data)
    {
        int pos = 0;
        int line = 0;
        if (!MovingAiMap.NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> head))
        {
            throw new GridFormatException(0, "the file is empty.");
        }
        Span<Range> f = stackalloc Range[5];
        ReadOnlySpan<byte> h = MovingAiMap.Trim(head);
        int n = VoxelScenarios.Split(h, f);
        if (n != 4 || !h[f[0]].SequenceEqual("voxel"u8))
        {
            throw new GridFormatException(line, "expected the first line 'voxel X Y Z'.");
        }
        int sx = VoxelScenarios.Int(h[f[1]], line, "size x");
        int sy = VoxelScenarios.Int(h[f[2]], line, "size y");
        int sz = VoxelScenarios.Int(h[f[3]], line, "size z");
        if (sx < 1 || sy < 1 || sz < 1)
        {
            throw new GridFormatException(line, "every dimension must be at least 1.");
        }
        if ((long)sx * sy * sz > VoxelGrid.MaxVoxels)
        {
            throw new GridFormatException(line, $"a {sx}x{sy}x{sz} map is larger than the {VoxelGrid.MaxVoxels} voxel limit.");
        }
        var filled = new List<int>();
        while (MovingAiMap.NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> l))
        {
            ReadOnlySpan<byte> t = MovingAiMap.Trim(l);
            if (t.IsEmpty)
            {
                continue;
            }
            if (VoxelScenarios.Split(t, f) != 3)
            {
                throw new GridFormatException(line, "expected a voxel line 'x y z'.");
            }
            int x = VoxelScenarios.Int(t[f[0]], line, "x");
            int y = VoxelScenarios.Int(t[f[1]], line, "y");
            int z = VoxelScenarios.Int(t[f[2]], line, "z");
            if (x >= sx || y >= sy || z >= sz)
            {
                throw new GridFormatException(line, $"voxel ({x}, {y}, {z}) is outside the {sx}x{sy}x{sz} map.");
            }
            filled.Add(x + (sx * (y + (sy * z))));
        }
        return new VoxelMap(sx, sy, sz, filled.ToArray());
    }

    /// <summary>Builds the grid: listed voxels are solid, all others open (area 0).</summary>
    /// <returns>A new grid with unit voxels.</returns>
    public VoxelGrid ToGrid()
    {
        var grid = new VoxelGrid(SizeX, SizeY, SizeZ);
        foreach (int node in _filled)
        {
            VoxelCell c = VoxelGrid.Decode(node, SizeX, SizeY);
            grid.SetArea(c.X, c.Y, c.Z, AreaCosts.Wall);
        }
        return grid;
    }
}

/// <summary>One line of a Moving AI voxel scenario file: start, goal, the published optimal length and the ratio of that length to the heuristic.</summary>
/// <param name="StartX">Start x.</param>
/// <param name="StartY">Start y.</param>
/// <param name="StartZ">Start z.</param>
/// <param name="GoalX">Goal x.</param>
/// <param name="GoalY">Goal y.</param>
/// <param name="GoalZ">Goal z.</param>
/// <param name="OptimalLength">The published optimal length, printed with eight decimals.</param>
/// <param name="Ratio">The published ratio of the optimal length to the heuristic distance.</param>
public readonly record struct VoxelScenarioEntry(int StartX, int StartY, int StartZ, int GoalX, int GoalY, int GoalZ, double OptimalLength, double Ratio);

/// <summary>Reads Moving AI Lab voxel scenario files (<c>.3dscen</c>, version 1).</summary>
public static class VoxelScenarios
{
    /// <summary>Most entries accepted in one file.</summary>
    public const int MaxEntries = 1 << 24;

    /// <summary>Parses scenario text.</summary>
    /// <param name="text">The file contents.</param>
    /// <returns>The entries in file order.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="GridFormatException">The text is not a valid version 1 voxel scenario file.</exception>
    public static VoxelScenarioEntry[] Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        return Parse(Ascii(text));
    }

    /// <summary>Parses scenario bytes (ASCII). Format: <c>version 1</c>, the map name, then per line
    /// <c>sx sy sz gx gy gz length ratio</c>.</summary>
    /// <param name="data">The file contents.</param>
    /// <returns>The entries in file order.</returns>
    /// <exception cref="GridFormatException">The data is not a valid version 1 voxel scenario file.</exception>
    public static VoxelScenarioEntry[] Parse(ReadOnlySpan<byte> data)
    {
        int pos = 0;
        int line = 0;
        if (!MovingAiMap.NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> first))
        {
            throw new GridFormatException(0, "the file is empty.");
        }
        ReadOnlySpan<byte> head = MovingAiMap.Trim(first);
        if (!(head.SequenceEqual("version 1"u8) || head.SequenceEqual("version 1.0"u8)))
        {
            throw new GridFormatException(line, "expected the first line 'version 1'.");
        }
        if (!MovingAiMap.NextLine(data, ref pos, ref line, out _))
        {
            throw new GridFormatException(line, "expected the map name on the second line.");
        }
        var entries = new List<VoxelScenarioEntry>();
        Span<Range> f = stackalloc Range[10];
        while (MovingAiMap.NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> l))
        {
            ReadOnlySpan<byte> t = MovingAiMap.Trim(l);
            if (t.IsEmpty)
            {
                continue;
            }
            if (entries.Count >= MaxEntries)
            {
                throw new GridFormatException(line, $"more than {MaxEntries} scenarios.");
            }
            if (Split(t, f) != 8)
            {
                throw new GridFormatException(line, "expected 8 fields (start x/y/z, goal x/y/z, length, ratio).");
            }
            entries.Add(new VoxelScenarioEntry(
                Int(t[f[0]], line, "start x"),
                Int(t[f[1]], line, "start y"),
                Int(t[f[2]], line, "start z"),
                Int(t[f[3]], line, "goal x"),
                Int(t[f[4]], line, "goal y"),
                Int(t[f[5]], line, "goal z"),
                Real(t[f[6]], line, "length"),
                Real(t[f[7]], line, "ratio")));
        }
        return entries.ToArray();
    }

    internal static byte[] Ascii(string text)
    {
        byte[] bytes = new byte[text.Length];
        for (int i = 0; i < text.Length; i++)
        {
            if (text[i] > 127)
            {
                throw new GridFormatException($"character {i}: non-ASCII character in text.");
            }
            bytes[i] = (byte)text[i];
        }
        return bytes;
    }

    internal static int Int(ReadOnlySpan<byte> s, int line, string what)
    {
        long v = MovingAiMap.ParseDigits(s, line, what);
        if (v > int.MaxValue)
        {
            throw new GridFormatException(line, $"{what} is too large.");
        }
        return (int)v;
    }

    private static double Real(ReadOnlySpan<byte> s, int line, string what)
    {
        if (s.Length > 32)
        {
            throw new GridFormatException(line, $"{what} is too long.");
        }
        Span<char> chars = stackalloc char[32];
        for (int i = 0; i < s.Length; i++)
        {
            chars[i] = (char)s[i];
        }
        chars = chars[..s.Length];
        if (!double.TryParse(chars, NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out double v) || double.IsNaN(v) || double.IsInfinity(v))
        {
            throw new GridFormatException(line, $"{what} is not a number.");
        }
        return v;
    }

    // Splits on runs of spaces and tabs. Returns the field count; stores at most fields.Length ranges.
    internal static int Split(ReadOnlySpan<byte> s, Span<Range> fields)
    {
        int count = 0;
        int i = 0;
        while (i < s.Length)
        {
            while (i < s.Length && s[i] is (byte)' ' or (byte)'\t')
            {
                i++;
            }
            if (i >= s.Length)
            {
                break;
            }
            int start = i;
            while (i < s.Length && !(s[i] is (byte)' ' or (byte)'\t'))
            {
                i++;
            }
            if (count < fields.Length)
            {
                fields[count] = start..i;
            }
            count++;
        }
        return count;
    }
}
