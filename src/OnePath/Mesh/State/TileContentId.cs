using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// The 256-bit identity of a tile's as-built content: the SHA-256 of its serialized form with the polygon areas and
/// flags it was built with. A snapshot can name a tile by this id when the host already holds the tile.
/// </summary>
public readonly struct TileContentId : IEquatable<TileContentId>, IComparable<TileContentId>
{
    private readonly ulong _a, _b, _c, _d;

    private TileContentId(ulong a, ulong b, ulong c, ulong d)
    {
        _a = a;
        _b = b;
        _c = c;
        _d = d;
    }

    internal static TileContentId Compute(ReadOnlySpan<byte> tileBytes)
    {
        Span<byte> h = stackalloc byte[32];
        Sha256.Hash(tileBytes, h);
        return FromBytes(h);
    }

    /// <summary>Creates an id from its 32-byte form.</summary>
    /// <exception cref="ArgumentException">The span is not 32 bytes long.</exception>
    public static TileContentId FromBytes(ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length != 32)
        {
            throw new ArgumentException("A tile content id is 32 bytes.", nameof(bytes));
        }
        return new TileContentId(
            BinaryPrimitives.ReadUInt64BigEndian(bytes), BinaryPrimitives.ReadUInt64BigEndian(bytes[8..]),
            BinaryPrimitives.ReadUInt64BigEndian(bytes[16..]), BinaryPrimitives.ReadUInt64BigEndian(bytes[24..]));
    }

    /// <summary>Writes the 32-byte form.</summary>
    /// <exception cref="ArgumentException">The destination is shorter than 32 bytes.</exception>
    public void CopyTo(Span<byte> destination)
    {
        if (destination.Length < 32)
        {
            throw new ArgumentException("The destination must hold 32 bytes.", nameof(destination));
        }
        BinaryPrimitives.WriteUInt64BigEndian(destination, _a);
        BinaryPrimitives.WriteUInt64BigEndian(destination[8..], _b);
        BinaryPrimitives.WriteUInt64BigEndian(destination[16..], _c);
        BinaryPrimitives.WriteUInt64BigEndian(destination[24..], _d);
    }

    /// <inheritdoc />
    public bool Equals(TileContentId o) => _a == o._a && _b == o._b && _c == o._c && _d == o._d;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is TileContentId o && Equals(o);

    /// <inheritdoc />
    public override int GetHashCode() => (int)_a ^ (int)(_a >> 32);

    /// <summary>Ordering by the big-endian bytes.</summary>
    public int CompareTo(TileContentId o)
    {
        int c = _a.CompareTo(o._a);
        if (c != 0)
        {
            return c;
        }
        c = _b.CompareTo(o._b);
        if (c != 0)
        {
            return c;
        }
        c = _c.CompareTo(o._c);
        return c != 0 ? c : _d.CompareTo(o._d);
    }

    /// <summary>Equality.</summary>
    public static bool operator ==(TileContentId a, TileContentId b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(TileContentId a, TileContentId b) => !a.Equals(b);

    /// <summary>Ordering.</summary>
    public static bool operator <(TileContentId a, TileContentId b) => a.CompareTo(b) < 0;

    /// <summary>Ordering.</summary>
    public static bool operator >(TileContentId a, TileContentId b) => a.CompareTo(b) > 0;

    /// <summary>Ordering.</summary>
    public static bool operator <=(TileContentId a, TileContentId b) => a.CompareTo(b) <= 0;

    /// <summary>Ordering.</summary>
    public static bool operator >=(TileContentId a, TileContentId b) => a.CompareTo(b) >= 0;

    /// <summary>The id as 64 lower-case hexadecimal digits.</summary>
    public override string ToString()
    {
        Span<byte> b = stackalloc byte[32];
        CopyTo(b);
        return Convert.ToHexStringLower(b);
    }
}
