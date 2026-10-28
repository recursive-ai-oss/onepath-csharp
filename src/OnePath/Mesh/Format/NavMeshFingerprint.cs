using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePath.Mesh;

/// <summary>
/// A fixed 128-bit fingerprint of every setting that changes the content of a tile: the mesh parameters, the digest of the
/// builder settings they do not cover, and the format version. Every tile carries its mesh's fingerprint in its header, so a tile
/// built with other settings is rejected when it is added, streamed in or restored. The encoding is in <c>docs/navmesh-format.md</c>.
/// </summary>
public readonly record struct NavMeshFingerprint(ulong Low, ulong High)
{
    /// <summary>The size of the fingerprint in bytes.</summary>
    public const int Size = 16;

    /// <summary>Writes the 16 bytes, little-endian, low word first.</summary>
    public void CopyTo(Span<byte> destination)
    {
        BinaryPrimitives.WriteUInt64LittleEndian(destination, Low);
        BinaryPrimitives.WriteUInt64LittleEndian(destination.Slice(8), High);
    }

    /// <summary>Reads 16 bytes written by <see cref="CopyTo"/>.</summary>
    public static NavMeshFingerprint From(ReadOnlySpan<byte> source) =>
        new(BinaryPrimitives.ReadUInt64LittleEndian(source), BinaryPrimitives.ReadUInt64LittleEndian(source.Slice(8)));

    /// <summary>The 32 lower-case hex digits of the 16 bytes in order.</summary>
    public override string ToString()
    {
        Span<byte> b = stackalloc byte[Size];
        CopyTo(b);
        return Convert.ToHexString(b).ToLowerInvariant();
    }
}
