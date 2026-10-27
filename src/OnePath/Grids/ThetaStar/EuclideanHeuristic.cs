using System;

namespace RecursiveAI.OnePath.Grids;

/// <summary>
/// Straight-line distance <c>scale * sqrt(dx² + dy²)</c> between cell centres on a square grid of the given width: the
/// admissible, consistent estimate for any-angle search (and still admissible, if loose, for grid moves). The square root
/// is applied to an exactly represented integer, so the value is identical on every platform.
/// </summary>
public struct EuclideanHeuristic : IHeuristic
{
    private readonly int _width;
    private readonly double _scale;
    private int _gx;
    private int _gy;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="width">Grid width in cells; positive.</param>
    /// <param name="scale">Multiplier applied to the distance, normally the cost of a straight step.</param>
    public EuclideanHeuristic(int width, double scale = 1.0)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(width);
        _width = width;
        _scale = scale;
        _gx = _gy = 0;
    }

    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
        _gy = goal / _width;
        _gx = goal - (_gy * _width);
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node)
    {
        int y = node / _width;
        long dx = (node - (y * _width)) - _gx;
        long dy = y - _gy;
        return _scale * Math.Sqrt((double)((dx * dx) + (dy * dy)));
    }
}
