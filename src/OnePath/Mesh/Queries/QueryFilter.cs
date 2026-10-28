using System;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>
/// Decides which polygons a query may use and what crossing them costs. Struct implementations passed to the
/// generic query methods are inlined; tier-1 code uses <see cref="QueryFilter"/> instead.
/// </summary>
public interface IQueryFilter
{
    /// <summary>True if a polygon (or off-mesh link) with these flags and this area may be used.</summary>
    bool Passes(ushort flags, byte area);

    /// <summary>
    /// Cost multiplier for travelling a unit distance inside a polygon (or along an off-mesh link) of this area.
    /// Must be positive and finite for areas that pass.
    /// </summary>
    double AreaCost(byte area);

    /// <summary>
    /// A positive lower bound on <see cref="AreaCost"/> over all areas that can pass. The search heuristic is
    /// distance times this bound; a bound that is too high makes paths non-optimal, 0 turns the heuristic off.
    /// </summary>
    double MinAreaCost { get; }
}

/// <summary>
/// The plain filter: include and exclude flag masks, one cost multiplier per area, and an optional predicate.
/// A polygon passes when <c>(flags &amp; IncludeFlags) != 0</c> (or <see cref="IncludeFlags"/> is the default 0xFFFF, which accepts polygons with no flags set), <c>(flags &amp; ExcludeFlags) == 0</c>, its area cost
/// is finite and the predicate (if any) accepts it. Defaults: every flag included, none excluded, every area costs 1.
/// A filter is immutable once constructed, so one instance can be shared by queries, crowds and threads: set the masks and
/// predicate in an object initializer and derive area costs with <see cref="WithAreaCost"/>, which returns a new filter.
/// </summary>
public sealed class QueryFilter
{
    private readonly double[] _cost = new double[NavMeshFormatLimits.AreaCount];

    /// <summary>Creates the default filter.</summary>
    public QueryFilter() => Array.Fill(_cost, 1.0);

    private QueryFilter(QueryFilter from)
    {
        _cost = (double[])from._cost.Clone();
        _minCost = from._minCost;
        IncludeFlags = from.IncludeFlags;
        ExcludeFlags = from.ExcludeFlags;
        Predicate = from.Predicate;
    }

    /// <summary>Polygons need at least one of these flag bits. Default 0xFFFF. Settable only in an object initializer.</summary>
    public ushort IncludeFlags { get; init; } = 0xFFFF;

    /// <summary>Polygons with any of these flag bits are skipped. Default 0. Settable only in an object initializer.</summary>
    public ushort ExcludeFlags { get; init; }

    /// <summary>Extra test applied after the masks, called with a polygon's flags and area; null accepts everything. Settable only in an object initializer.</summary>
    public Func<ushort, byte, bool>? Predicate { get; init; }

    /// <summary>
    /// Returns a copy of this filter with the cost multiplier of one area changed; this filter is not modified.
    /// Use <see cref="double.PositiveInfinity"/> to forbid the area.
    /// </summary>
    /// <param name="area">The area, 0..63.</param>
    /// <param name="cost">The multiplier: positive, infinity allowed.</param>
    /// <returns>The new filter, with the same masks and predicate.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The area is outside 0..63 or the cost is not positive (infinity allowed) or NaN.</exception>
    public QueryFilter WithAreaCost(int area, double cost)
    {
        if ((uint)area >= NavMeshFormatLimits.AreaCount)
        {
            throw new ArgumentOutOfRangeException(nameof(area), area, "Area must be 0 to 63.");
        }
        if (!(cost > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(cost), cost, "Area cost must be positive (infinity forbids the area).");
        }
        var copy = new QueryFilter(this);
        copy._cost[area] = cost;
        double m = double.PositiveInfinity;
        foreach (double c in copy._cost)
        {
            m = Math.Min(m, c);
        }
        copy._minCost = double.IsInfinity(m) ? 1.0 : m;
        return copy;
    }

    /// <summary>The cost multiplier of an area (1 for areas outside 0..63 are not valid; callers pass real areas).</summary>
    public double GetAreaCost(byte area) => _cost[area & 63];

    // Computed by WithAreaCost so that preparing a filter for a query (done by every plain-filter call) costs nothing.
    private double _minCost = 1.0;

    internal double MinCost => _minCost;

    internal bool Accepts(ushort flags, byte area) =>
        (IncludeFlags == 0xFFFF || (flags & IncludeFlags) != 0) && (flags & ExcludeFlags) == 0 && !double.IsPositiveInfinity(_cost[area & 63]) && (Predicate is null || Predicate(flags, area));
}

internal static class NavMeshFormatLimits
{
    public const int AreaCount = 64;
}

/// <summary>Adapts the plain <see cref="QueryFilter"/> to the struct-generic core.</summary>
internal readonly struct PlainFilter : IQueryFilter
{
    private readonly QueryFilter _f;
    private readonly double _min;

    public PlainFilter(QueryFilter f)
    {
        _f = f;
        _min = f.MinCost;
    }

    public bool Passes(ushort flags, byte area) => _f.Accepts(flags, area);

    public double AreaCost(byte area) => _f.GetAreaCost(area);

    public double MinAreaCost => _min;
}
