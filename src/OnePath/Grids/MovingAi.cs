using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace RecursiveAI.OnePath.Grids;

/// <summary>Thrown when a map or scenario file does not follow its format; the message says what was wrong and where.</summary>
public sealed class GridFormatException : FormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public GridFormatException()
    {
    }

    /// <summary>Creates the exception with a message.</summary>
    /// <param name="message">What was wrong.</param>
    public GridFormatException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and an inner exception.</summary>
    /// <param name="message">What was wrong.</param>
    /// <param name="innerException">The cause.</param>
    public GridFormatException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    internal GridFormatException(int line, string message)
        : base($"line {line}: {message}")
    {
        Line = line;
    }

    /// <summary>The 1-based line of the problem, or 0 when not tied to a line.</summary>
    public int Line { get; }
}

/// <summary>How terrain letters of a Moving AI map become cells.</summary>
public sealed class MovingAiOptions
{
    /// <summary>
    /// Whether 'S' (swamp) is walkable. The format page calls swamp passable from regular terrain; the published
    /// benchmark optimal lengths decide the default. Default true.
    /// </summary>
    public bool SwampWalkable { get; set; } = true;

    /// <summary>Whether 'W' (water) is walkable. Water is not passable from ordinary terrain on the format page. Default false.</summary>
    public bool WaterWalkable { get; set; }
}

/// <summary>
/// A parsed Moving AI Lab <c>.map</c> file (format: https://movingai.com/benchmarks/formats.html): a header
/// (<c>type</c>, <c>height</c>, <c>width</c>, then a line <c>map</c>) followed by <c>height</c> rows of <c>width</c>
/// characters. Walkable letters are '.' and 'G'; '@', 'O' and 'T' are blocked; 'S' (swamp) and 'W' (water) depend on
/// <see cref="MovingAiOptions"/>. Anything else is an error.
/// </summary>
public sealed class MovingAiMap
{
    /// <summary>Largest width or height accepted.</summary>
    public const int MaxDimension = 1 << 16;

    private readonly byte[] _tiles;

    private MovingAiMap(int width, int height, byte[] tiles)
    {
        Width = width;
        Height = height;
        _tiles = tiles;
    }

    /// <summary>Columns.</summary>
    public int Width { get; }

    /// <summary>Rows.</summary>
    public int Height { get; }

    /// <summary>The terrain letters, row-major (<c>y * Width + x</c>).</summary>
    public ReadOnlySpan<byte> Tiles => _tiles;

    /// <summary>Parses map text.</summary>
    /// <param name="text">The file contents.</param>
    /// <returns>The map.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="GridFormatException">The text is not a valid map.</exception>
    public static MovingAiMap Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        byte[] bytes = new byte[text.Length];
        for (int i = 0; i < text.Length; i++)
        {
            if (text[i] > 127)
            {
                throw new GridFormatException($"character {i}: non-ASCII character in map text.");
            }
            bytes[i] = (byte)text[i];
        }
        return Parse(bytes);
    }

    /// <summary>Parses map bytes (ASCII).</summary>
    /// <param name="data">The file contents.</param>
    /// <returns>The map.</returns>
    /// <exception cref="GridFormatException">The data is not a valid map.</exception>
    public static MovingAiMap Parse(ReadOnlySpan<byte> data)
    {
        int pos = 0;
        int line = 0;
        int width = -1;
        int height = -1;
        bool sawMap = false;
        while (!sawMap)
        {
            if (!NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> l))
            {
                throw new GridFormatException(line, "the header ended before a 'map' line.");
            }
            if (l.IsEmpty)
            {
                continue;
            }
            int sp = l.IndexOfAny((byte)' ', (byte)'\t');
            ReadOnlySpan<byte> key = sp < 0 ? l : l[..sp];
            ReadOnlySpan<byte> value = sp < 0 ? default : Trim(l[(sp + 1)..]);
            if (key.SequenceEqual("map"u8) && value.IsEmpty)
            {
                sawMap = true;
            }
            else if (key.SequenceEqual("type"u8))
            {
                // The type (normally 'octile') does not change how the cells are read.
            }
            else if (key.SequenceEqual("width"u8))
            {
                width = ParseDimension(value, line, "width");
            }
            else if (key.SequenceEqual("height"u8))
            {
                height = ParseDimension(value, line, "height");
            }
            else
            {
                throw new GridFormatException(line, "unknown header line (expected type, height, width or map).");
            }
        }
        if (width < 0)
        {
            throw new GridFormatException(line, "the header has no width.");
        }
        if (height < 0)
        {
            throw new GridFormatException(line, "the header has no height.");
        }
        if ((long)width * height > Grid.MaxCells)
        {
            throw new GridFormatException(line, $"a {width}x{height} map is larger than the {Grid.MaxCells} cell limit.");
        }
        // Never allocate more than the input can fill: every row needs width bytes.
        if ((long)width * height > data.Length - pos)
        {
            throw new GridFormatException(line, $"the header declares {width}x{height} cells but the file is too short.");
        }
        var tiles = new byte[width * height];
        for (int y = 0; y < height; y++)
        {
            if (!NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> row))
            {
                throw new GridFormatException(line, $"expected {height} map rows but the file has {y}.");
            }
            if (row.Length != width)
            {
                throw new GridFormatException(line, $"row {y} has {row.Length} characters; the header says width {width}.");
            }
            for (int x = 0; x < width; x++)
            {
                byte c = row[x];
                if (!IsKnownTerrain(c))
                {
                    throw new GridFormatException(line, $"column {x + 1}: unknown terrain character 0x{c:X2}.");
                }
                tiles[(y * width) + x] = c;
            }
        }
        while (NextLine(data, ref pos, ref line, out ReadOnlySpan<byte> rest))
        {
            if (!Trim(rest).IsEmpty)
            {
                throw new GridFormatException(line, "unexpected text after the last map row.");
            }
        }
        return new MovingAiMap(width, height, tiles);
    }

    /// <summary>Turns the map into a grid of walkable and blocked cells (area 0 and wall).</summary>
    /// <param name="options">Terrain rules; null for the defaults.</param>
    /// <returns>A new grid with unit cells.</returns>
    public Grid ToGrid(MovingAiOptions? options = null)
    {
        options ??= new MovingAiOptions();
        var walkable = new bool[_tiles.Length];
        for (int i = 0; i < _tiles.Length; i++)
        {
            walkable[i] = _tiles[i] switch
            {
                (byte)'.' or (byte)'G' => true,
                (byte)'S' => options.SwampWalkable,
                (byte)'W' => options.WaterWalkable,
                _ => false,
            };
        }
        return Grid.From(walkable, Width, Height);
    }

    private static bool IsKnownTerrain(byte c) => c is (byte)'.' or (byte)'G' or (byte)'@' or (byte)'O' or (byte)'T' or (byte)'S' or (byte)'W';

    private static int ParseDimension(ReadOnlySpan<byte> value, int line, string name)
    {
        long v = ParseDigits(value, line, name);
        if (v < 1 || v > MaxDimension)
        {
            throw new GridFormatException(line, $"{name} {v} is outside 1..{MaxDimension}.");
        }
        return (int)v;
    }

    internal static long ParseDigits(ReadOnlySpan<byte> value, int line, string what)
    {
        if (value.IsEmpty || value.Length > 10)
        {
            throw new GridFormatException(line, $"{what} is missing or too long.");
        }
        long v = 0;
        foreach (byte c in value)
        {
            if (c < (byte)'0' || c > (byte)'9')
            {
                throw new GridFormatException(line, $"{what} must be a non-negative integer.");
            }
            v = (v * 10) + (c - (byte)'0');
        }
        return v;
    }

    internal static ReadOnlySpan<byte> Trim(ReadOnlySpan<byte> s)
    {
        int a = 0;
        int b = s.Length;
        while (a < b && s[a] is (byte)' ' or (byte)'\t' or (byte)'\r')
        {
            a++;
        }
        while (b > a && s[b - 1] is (byte)' ' or (byte)'\t' or (byte)'\r')
        {
            b--;
        }
        return s[a..b];
    }

    // Reads the next line without its '\n' (and without a trailing '\r'); false at the end of the data.
    internal static bool NextLine(ReadOnlySpan<byte> data, ref int pos, ref int line, out ReadOnlySpan<byte> result)
    {
        if (pos >= data.Length)
        {
            result = default;
            return false;
        }
        ReadOnlySpan<byte> rest = data[pos..];
        int nl = rest.IndexOf((byte)'\n');
        ReadOnlySpan<byte> l = nl < 0 ? rest : rest[..nl];
        pos += nl < 0 ? rest.Length : nl + 1;
        if (!l.IsEmpty && l[^1] == (byte)'\r')
        {
            l = l[..^1];
        }
        line++;
        result = l;
        return true;
    }
}

/// <summary>One line of a Moving AI <c>.scen</c> file: a start, a goal and the published optimal path length.</summary>
/// <param name="Bucket">The difficulty bucket (path length class).</param>
/// <param name="Map">The map file name.</param>
/// <param name="MapWidth">Map width as printed in the scenario.</param>
/// <param name="MapHeight">Map height as printed in the scenario.</param>
/// <param name="StartX">Start column.</param>
/// <param name="StartY">Start row.</param>
/// <param name="GoalX">Goal column.</param>
/// <param name="GoalY">Goal row.</param>
/// <param name="OptimalLength">The published optimal length (octile metric, no corner cutting).</param>
public readonly record struct ScenarioEntry(int Bucket, string Map, int MapWidth, int MapHeight, int StartX, int StartY, int GoalX, int GoalY, double OptimalLength);

/// <summary>Reads Moving AI Lab scenario files, version 1 (https://movingai.com/benchmarks/formats.html).</summary>
public static class MovingAiScenarios
{
    /// <summary>Most entries accepted in one file.</summary>
    public const int MaxEntries = 1 << 24;

    /// <summary>Parses scenario text.</summary>
    /// <param name="text">The file contents.</param>
    /// <returns>The entries in file order.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="text"/> is null.</exception>
    /// <exception cref="GridFormatException">The text is not a valid version 1 scenario file.</exception>
    public static ScenarioEntry[] Parse(string text)
    {
        ArgumentNullException.ThrowIfNull(text);
        byte[] bytes = new byte[text.Length];
        for (int i = 0; i < text.Length; i++)
        {
            if (text[i] > 127)
            {
                throw new GridFormatException($"character {i}: non-ASCII character in scenario text.");
            }
            bytes[i] = (byte)text[i];
        }
        return Parse(bytes);
    }

    /// <summary>Parses scenario bytes (ASCII).</summary>
    /// <param name="data">The file contents.</param>
    /// <returns>The entries in file order.</returns>
    /// <exception cref="GridFormatException">The data is not a valid version 1 scenario file.</exception>
    public static ScenarioEntry[] Parse(ReadOnlySpan<byte> data)
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
        var entries = new List<ScenarioEntry>();
        Span<Range> fields = stackalloc Range[10];
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
            int n = SplitFields(t, fields);
            if (n != 9)
            {
                throw new GridFormatException(line, $"expected 9 fields (bucket, map, width, height, start x/y, goal x/y, length) but found {(n > 9 ? "more" : n.ToString(CultureInfo.InvariantCulture))}.");
            }
            int bucket = Int(t[fields[0]], line, "bucket");
            string map = Encoding.ASCII.GetString(t[fields[1]]);
            int mw = Int(t[fields[2]], line, "map width");
            int mh = Int(t[fields[3]], line, "map height");
            int sx = Int(t[fields[4]], line, "start x");
            int sy = Int(t[fields[5]], line, "start y");
            int gx = Int(t[fields[6]], line, "goal x");
            int gy = Int(t[fields[7]], line, "goal y");
            string lenText = Encoding.ASCII.GetString(t[fields[8]]);
            if (!double.TryParse(lenText, NumberStyles.AllowDecimalPoint, CultureInfo.InvariantCulture, out double len) || double.IsNaN(len) || double.IsInfinity(len))
            {
                throw new GridFormatException(line, "the optimal length is not a number.");
            }
            entries.Add(new ScenarioEntry(bucket, map, mw, mh, sx, sy, gx, gy, len));
        }
        return entries.ToArray();
    }

    private static int Int(ReadOnlySpan<byte> s, int line, string what)
    {
        long v = MovingAiMap.ParseDigits(s, line, what);
        if (v > int.MaxValue)
        {
            throw new GridFormatException(line, $"{what} is too large.");
        }
        return (int)v;
    }

    // Splits on runs of spaces and tabs. Returns the field count; stores at most fields.Length - 1 ranges.
    private static int SplitFields(ReadOnlySpan<byte> s, Span<Range> fields)
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
