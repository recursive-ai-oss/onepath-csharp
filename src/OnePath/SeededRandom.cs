namespace RecursiveAI.OnePath;

/// <summary>
/// A small seeded pseudo-random generator (SplitMix64, Steele, Lea &amp; Flood 2014). The same seed gives the
/// same sequence on every platform, so tests, generated maps and random queries are reproducible.
/// Not suitable for cryptography.
/// </summary>
public struct SeededRandom
{
    private ulong _state;

    /// <summary>Creates a generator from a seed. Any 64-bit value is valid.</summary>
    public SeededRandom(ulong seed) => _state = seed;

    /// <summary>The next 64 random bits.</summary>
    public ulong NextUInt64()
    {
        _state += 0x9E3779B97F4A7C15UL;
        ulong z = _state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return z ^ (z >> 31);
    }

    /// <summary>A value in [0, <paramref name="bound"/>). Uses a multiply-shift reduction (a bias below 2^-32 per value, fixed and platform independent).</summary>
    /// <param name="bound">Exclusive upper bound; must be positive.</param>
    /// <exception cref="System.ArgumentOutOfRangeException"><paramref name="bound"/> is not positive.</exception>
    public int NextInt(int bound)
    {
        System.ArgumentOutOfRangeException.ThrowIfNegativeOrZero(bound);
        return (int)(((NextUInt64() >> 32) * (ulong)bound) >> 32);
    }

    /// <summary>A double in [0, 1) with 53 random bits.</summary>
    public double NextDouble() => (NextUInt64() >> 11) * (1.0 / 9007199254740992.0);
}
