using System;

namespace RecursiveAI.OnePath.NavMesh;

/// <summary>
/// A stable reference to a polygon: 24 bits of tile-slot generation (salt), 20 bits of tile slot and 20 bits of
/// polygon index. A reference stays valid while its tile stays in its slot; removing or replacing the tile
/// bumps the slot's salt, so old references are rejected instead of silently meaning another polygon.
/// The all-zero value is <see cref="Invalid"/>.
/// </summary>
public readonly struct PolyRef : IEquatable<PolyRef>
{
    /// <summary>The raw 64-bit value (salt in the top 24 bits, then the slot, then the polygon index).</summary>
    public ulong Value { get; }

    /// <summary>Wraps a raw value.</summary>
    public PolyRef(ulong value) => Value = value;

    internal PolyRef(uint salt, int slot, int poly) =>
        Value = ((ulong)salt << 40) | ((ulong)(uint)slot << 20) | (uint)poly;

    /// <summary>The reference that refers to nothing.</summary>
    public static PolyRef Invalid => default;

    /// <summary>True unless this is <see cref="Invalid"/>.</summary>
    public bool IsSet => Value != 0;

    /// <summary>Slot generation, 1 to 2^24-1 for a real reference.</summary>
    public uint Salt => (uint)(Value >> 40);

    /// <summary>Tile slot index.</summary>
    public int TileSlot => (int)((Value >> 20) & 0xFFFFF);

    /// <summary>Polygon index within the tile.</summary>
    public int PolyIndex => (int)(Value & 0xFFFFF);

    /// <summary>The tile reference this polygon belongs to.</summary>
    public TileRef Tile => new(Salt, TileSlot);

    /// <inheritdoc />
    public bool Equals(PolyRef other) => Value == other.Value;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is PolyRef r && Equals(r);

    /// <inheritdoc />
    public override int GetHashCode() => (int)Value ^ (int)(Value >> 32);

    /// <summary>Equality.</summary>
    public static bool operator ==(PolyRef a, PolyRef b) => a.Value == b.Value;

    /// <summary>Inequality.</summary>
    public static bool operator !=(PolyRef a, PolyRef b) => a.Value != b.Value;

    /// <inheritdoc />
    public override string ToString() => IsSet ? $"poly(salt {Salt}, tile {TileSlot}, poly {PolyIndex})" : "poly(invalid)";
}

/// <summary>A stable reference to a tile: slot plus slot generation. Invalid after the tile is removed or replaced.</summary>
public readonly struct TileRef : IEquatable<TileRef>
{
    /// <summary>Slot generation; 0 means invalid.</summary>
    public uint Salt { get; }

    /// <summary>Tile slot index.</summary>
    public int Slot { get; }

    internal TileRef(uint salt, int slot)
    {
        Salt = salt;
        Slot = slot;
    }

    /// <summary>The reference that refers to nothing.</summary>
    public static TileRef Invalid => default;

    /// <summary>True unless this is <see cref="Invalid"/>.</summary>
    public bool IsSet => Salt != 0;

    /// <summary>The reference of polygon <paramref name="polyIndex"/> of this tile.</summary>
    public PolyRef Poly(int polyIndex) => new(Salt, Slot, polyIndex);

    /// <inheritdoc />
    public bool Equals(TileRef other) => Salt == other.Salt && Slot == other.Slot;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is TileRef r && Equals(r);

    /// <inheritdoc />
    public override int GetHashCode() => (int)(Salt * 31u) ^ Slot;

    /// <summary>Equality.</summary>
    public static bool operator ==(TileRef a, TileRef b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(TileRef a, TileRef b) => !a.Equals(b);

    /// <inheritdoc />
    public override string ToString() => IsSet ? $"tile(salt {Salt}, slot {Slot})" : "tile(invalid)";
}
