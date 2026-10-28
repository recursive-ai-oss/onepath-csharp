using System;

namespace RecursiveAI.OnePath;

/// <summary>
/// Cost multipliers per area type. A grid cell stores an area id (0..254); <see cref="Wall"/> (255) is
/// impassable. Moving between two cells costs the step length times the mean of the two cells' multipliers
/// (see docs/grids.md). All multipliers start at 1.
/// </summary>
public sealed partial class AreaCosts
{
    /// <summary>The reserved area id of impassable cells.</summary>
    public const byte Wall = 255;

    /// <summary>The number of usable area ids (0..254).</summary>
    public const int Count = 255;

    private readonly double[] _multipliers = new double[256];
    private double _min = 1.0;

    /// <summary>Creates a table with every multiplier 1.</summary>
    public AreaCosts()
    {
        Array.Fill(_multipliers, 1.0);
        _multipliers[Wall] = double.PositiveInfinity;
    }

    /// <summary>Incremented whenever a multiplier changes.</summary>
    public int Version { get; private set; }

    /// <summary>The smallest multiplier of any usable area: the scale that keeps distance heuristics admissible.</summary>
    public double MinCost => _min;

    internal double[] Multipliers => _multipliers;

    /// <summary>True when every usable area has the same multiplier (what the jump point searches need); that multiplier is returned.</summary>
    internal bool TryGetUniform(out double multiplier)
    {
        multiplier = _multipliers[0];
        for (int i = 1; i < Count; i++)
        {
            if (_multipliers[i] != multiplier)
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>Gets or sets the multiplier of an area. The getter returns +∞ for <see cref="Wall"/>.</summary>
    /// <param name="area">Area id.</param>
    /// <exception cref="ArgumentOutOfRangeException">Setting <see cref="Wall"/>, or a value that is not positive and finite.</exception>
    public double this[byte area]
    {
        get => _multipliers[area];
        set
        {
            if (area == Wall)
            {
                throw new ArgumentOutOfRangeException(nameof(area), area, "Area 255 is the wall and has no multiplier.");
            }
            if (!(value > 0.0) || double.IsPositiveInfinity(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "An area multiplier must be positive and finite.");
            }
            _multipliers[area] = value;
            double min = double.PositiveInfinity;
            for (int i = 0; i < Count; i++)
            {
                if (_multipliers[i] < min)
                {
                    min = _multipliers[i];
                }
            }
            _min = min;
            Version++;
        }
    }

    /// <summary>Copies the multipliers of another table.</summary>
    /// <param name="other">The table to copy.</param>
    /// <exception cref="ArgumentNullException"><paramref name="other"/> is null.</exception>
    public void CopyFrom(AreaCosts other)
    {
        ArgumentNullException.ThrowIfNull(other);
        Array.Copy(other._multipliers, _multipliers, _multipliers.Length);
        _min = other._min;
        Version++;
    }
}

/// <summary>Which neighbour moves a square-grid search may make.</summary>
public enum Connectivity
{
    /// <summary>North, east, south, west.</summary>
    Four = 4,

    /// <summary>The four cardinal moves plus the four diagonals (subject to the <see cref="DiagonalRule"/>).</summary>
    Eight = 8,
}

/// <summary>When a diagonal move is allowed on an eight-connected grid.</summary>
public enum DiagonalRule
{
    /// <summary>Both cells beside the move must be walkable (the Moving AI benchmark convention). The default.</summary>
    NoCornerCutting = 0,

    /// <summary>At least one of the two cells beside the move must be walkable (no squeezing between two diagonal walls).</summary>
    NoSqueezing = 1,

    /// <summary>The destination is all that matters; the move may cut between two blocked cells.</summary>
    AllowAll = 2,
}

/// <summary>The distance estimate used by a grid search.</summary>
public enum GridHeuristic
{
    /// <summary>Manhattan for four-connected grids, octile for eight-connected grids, hex distance on hex grids.</summary>
    Auto = 0,

    /// <summary>No estimate: Dijkstra's algorithm.</summary>
    Zero = 1,

    /// <summary>Manhattan distance (|dx| + |dy|).</summary>
    Manhattan = 2,

    /// <summary>Octile distance (diagonal steps cost √2).</summary>
    Octile = 3,

    /// <summary>Straight-line distance; the estimate any-angle searches (Theta*, Lazy Theta*) use.</summary>
    Euclidean = 4,
}
