using System;
using System.Buffers.Binary;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;

namespace RecursiveAI.OnePath.Crowds.Corridors;

// Little-endian writer for state bytes; doubles go by bit pattern so the same state gives the same bytes everywhere.
internal ref struct StateWriter
{
    private readonly Span<byte> _d;

    public StateWriter(Span<byte> destination)
    {
        _d = destination;
        Position = 0;
    }

    public int Position { get; private set; }

    public readonly Span<byte> Remaining => _d[Position..];

    public void Skip(int n) => Position += n;

    public void U8(byte v) => _d[Position++] = v;

    public void I32(int v)
    {
        BinaryPrimitives.WriteInt32LittleEndian(_d[Position..], v);
        Position += 4;
    }

    public void U64(ulong v)
    {
        BinaryPrimitives.WriteUInt64LittleEndian(_d[Position..], v);
        Position += 8;
    }

    public void F64(double v) => U64((ulong)BitConverter.DoubleToInt64Bits(v));

    public void V3(Vector3D v)
    {
        F64(v.X);
        F64(v.Y);
        F64(v.Z);
    }
}

// Reader that validates as it goes: a short buffer, a wrong version or an impossible value throws a typed error naming it.
internal ref struct StateReader
{
    private const double Bound = 2097152.0; // 2^21, the coordinate rule
    private readonly ReadOnlySpan<byte> _s;
    private readonly string _what;

    public StateReader(ReadOnlySpan<byte> source, string what)
    {
        _s = source;
        _what = what;
        Position = 0;
    }

    public int Position { get; private set; }

    public readonly NavMeshFormatException Error(string field, double value) =>
        new($"Invalid {_what} state: {field} = {value} at byte {Position}.");

    private void Need(int n)
    {
        if (_s.Length - Position < n)
        {
            throw new NavMeshFormatException($"Invalid {_what} state: truncated at byte {Position}, {n} more bytes needed, {_s.Length - Position} left.");
        }
    }

    public int U8()
    {
        Need(1);
        return _s[Position++];
    }

    public void Version(int expected)
    {
        int v = U8();
        if (v != expected)
        {
            throw new NavMeshFormatException($"Invalid {_what} state: version {v}, expected {expected}.");
        }
    }

    public int I32()
    {
        Need(4);
        int v = BinaryPrimitives.ReadInt32LittleEndian(_s[Position..]);
        Position += 4;
        return v;
    }

    public ulong U64()
    {
        Need(8);
        ulong v = BinaryPrimitives.ReadUInt64LittleEndian(_s[Position..]);
        Position += 8;
        return v;
    }

    public double F64(string field, bool coordinate)
    {
        double v = BitConverter.Int64BitsToDouble((long)U64());
        if (double.IsNaN(v) || double.IsInfinity(v) || (coordinate && Math.Abs(v) >= Bound))
        {
            throw Error(field, v);
        }
        return v;
    }

    // The remaining distance: not negative, positive infinity allowed (unknown).
    public double F64Remaining()
    {
        double v = BitConverter.Int64BitsToDouble((long)U64());
        if (double.IsNaN(v) || v < 0)
        {
            throw Error("remaining distance", v);
        }
        return v;
    }

    public Vector3D V3(string field)
    {
        double x = F64(field, true), y = F64(field, true), z = F64(field, true);
        return new Vector3D(x, y, z);
    }
}
