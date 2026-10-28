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
