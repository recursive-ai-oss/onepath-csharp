using System;
using System.Buffers.Binary;
using System.Globalization;

namespace RecursiveAI.OnePath;

/// <summary>
/// Thrown when grid, graph, search or flow-field state cannot be restored: the bytes are truncated, were written by another
/// kind of object or format version, name nodes outside the object they are restored into, or contain impossible values.
/// The message names the byte offset and what was wrong. The object being restored may be partly changed when this is thrown;
/// restore again from good bytes or discard it.
/// </summary>
public sealed class GridStateException : FormatException
{
    /// <summary>Creates the exception with a message.</summary>
    /// <param name="message">What was wrong.</param>
    public GridStateException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and a cause.</summary>
    /// <param name="message">What was wrong.</param>
    /// <param name="innerException">The cause.</param>
    public GridStateException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    /// <summary>Creates an empty exception.</summary>
    public GridStateException()
    {
    }
}

/// <summary>Object kinds in a state blob's header (docs/grids-state.md).</summary>
internal enum StateKind : ushort
{
    Grid = 1,
    HexGrid = 2,
    VoxelGrid = 3,
    Graph = 4,
    GridReplanner = 5,
    HexReplanner = 6,
    FlowField = 7,
    AStar = 8,
    JumpSearch = 9,
    ThetaSearch = 10,
    GraphAStar = 11,
    GridSearch = 12,
}

/// <summary>Writes the little-endian state format into a growing byte array.</summary>
internal sealed class StateWriter
{
    internal const uint Magic = 0x5347504F; // the bytes O P G S, little-endian
    internal const ushort FormatVersion = 1;

    private byte[] _buffer = new byte[256];
    private int _length;

    internal StateWriter(StateKind kind)
    {
        U32(Magic);
        U16(FormatVersion);
        U16((ushort)kind);
    }

    private Span<byte> Reserve(int count)
    {
        if (_buffer.Length - _length < count)
        {
            long need = (long)_length + count;
            long size = Math.Max(need, (long)_buffer.Length * 2);
            if (size > Array.MaxLength)
            {
                size = Array.MaxLength;
            }
            if (need > size)
            {
                throw new InvalidOperationException("The state is too large to hold in one byte array.");
            }
            Array.Resize(ref _buffer, (int)size);
        }
        Span<byte> s = _buffer.AsSpan(_length, count);
        _length += count;
        return s;
    }

    internal void U8(byte v) => Reserve(1)[0] = v;

    internal void U16(ushort v) => BinaryPrimitives.WriteUInt16LittleEndian(Reserve(2), v);

    internal void U32(uint v) => BinaryPrimitives.WriteUInt32LittleEndian(Reserve(4), v);

    internal void I32(int v) => BinaryPrimitives.WriteInt32LittleEndian(Reserve(4), v);

    internal void I64(long v) => BinaryPrimitives.WriteInt64LittleEndian(Reserve(8), v);

    internal void F64(double v) => BinaryPrimitives.WriteInt64LittleEndian(Reserve(8), BitConverter.DoubleToInt64Bits(v));

    internal void Bool(bool v) => U8(v ? (byte)1 : (byte)0);

    internal void Bytes(ReadOnlySpan<byte> v) => v.CopyTo(Reserve(v.Length));

    internal byte[] ToArray() => _buffer.AsSpan(0, _length).ToArray();
}

/// <summary>Reads the state format with bounds checks; every failure is a <see cref="GridStateException"/>.</summary>
internal ref struct StateReader
{
    private readonly ReadOnlySpan<byte> _data;
    private int _pos;

    internal StateReader(ReadOnlySpan<byte> data, StateKind expected)
    {
        _data = data;
        _pos = 0;
        if (data.Length < 8)
        {
            throw new GridStateException(string.Create(CultureInfo.InvariantCulture, $"The state has {data.Length} bytes; the header alone is 8."));
        }
        uint magic = U32();
        if (magic != StateWriter.Magic)
        {
            throw new GridStateException("The bytes are not onepath grid state (bad magic number).");
        }
        ushort version = U16();
        if (version != StateWriter.FormatVersion)
        {
            throw new GridStateException(string.Create(CultureInfo.InvariantCulture, $"State format version {version} is not supported (this build reads {StateWriter.FormatVersion})."));
        }
        ushort kind = U16();
        if (kind != (ushort)expected)
        {
            throw new GridStateException(string.Create(CultureInfo.InvariantCulture, $"The state holds object kind {kind} but kind {(ushort)expected} ({expected}) was expected."));
        }
    }

    internal readonly int Position => _pos;

    internal readonly int Remaining => _data.Length - _pos;

    internal readonly GridStateException Fail(string message) =>
        new(string.Create(CultureInfo.InvariantCulture, $"Bad state at byte {_pos}: {message}"));

    private ReadOnlySpan<byte> Take(int count)
    {
        if (count < 0 || _data.Length - _pos < count)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"the state ends early ({count} more bytes needed, {_data.Length - _pos} left)."));
        }
        ReadOnlySpan<byte> s = _data.Slice(_pos, count);
        _pos += count;
        return s;
    }

    internal byte U8() => Take(1)[0];

    internal ushort U16() => BinaryPrimitives.ReadUInt16LittleEndian(Take(2));

    internal uint U32() => BinaryPrimitives.ReadUInt32LittleEndian(Take(4));

    internal int I32() => BinaryPrimitives.ReadInt32LittleEndian(Take(4));

    internal long I64() => BinaryPrimitives.ReadInt64LittleEndian(Take(8));

    internal double F64() => BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(Take(8)));

    internal bool Bool()
    {
        byte b = U8();
        if (b > 1)
        {
            throw Fail($"a flag byte holds {b}; it must be 0 or 1.");
        }
        return b == 1;
    }

    internal ReadOnlySpan<byte> Bytes(int count) => Take(count);

    /// <summary>Reads a count that must lie in [0, <paramref name="max"/>] and be backed by at least <paramref name="minBytesEach"/> bytes per item.</summary>
    internal int Count(int max, int minBytesEach, string what)
    {
        int n = I32();
        if (n < 0 || n > max)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"{what} is {n}; it must be between 0 and {max}."));
        }
        if ((long)n * minBytesEach > Remaining)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"{what} is {n} but only {Remaining} bytes follow."));
        }
        return n;
    }

    /// <summary>Reads an int that must equal <paramref name="expected"/> (the restored object's own size).</summary>
    internal void Expect(int expected, string what)
    {
        int v = I32();
        if (v != expected)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"{what} is {v} in the state but {expected} in the object being restored."));
        }
    }

    internal int Node(int nodeCount, string what)
    {
        int v = I32();
        if ((uint)v >= (uint)nodeCount)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"{what} {v} is outside the {nodeCount} nodes of the object being restored."));
        }
        return v;
    }

    internal double NotNaN(string what)
    {
        double v = F64();
        if (double.IsNaN(v))
        {
            throw Fail($"{what} is NaN.");
        }
        return v;
    }

    internal readonly void End()
    {
        if (_pos != _data.Length)
        {
            throw Fail(string.Create(CultureInfo.InvariantCulture, $"{_data.Length - _pos} unexpected bytes follow the state."));
        }
    }
}
