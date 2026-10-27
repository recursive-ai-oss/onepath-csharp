using System;
using System.Globalization;

namespace RecursiveAI.OnePath;

/// <summary>A directed edge of a weighted graph.</summary>
/// <param name="From">Source node id.</param>
/// <param name="To">Target node id.</param>
/// <param name="Weight">Cost of the edge: finite and not negative.</param>
public readonly record struct GraphEdge(int From, int To, double Weight);

/// <summary>A position in 3D (or 2D with <c>Z = 0</c>) for point graphs. Coordinates must be finite and within ±2^21.</summary>
public readonly record struct Point3(double X, double Y, double Z)
{
    /// <summary>Creates a point in the plane (<c>Z = 0</c>).</summary>
    public Point3(double x, double y) : this(x, y, 0.0)
    {
    }

    /// <summary>The straight-line distance to <paramref name="other"/>: <c>sqrt(dx*dx + dy*dy + dz*dz)</c> summed in that order, IEEE-exact on every platform.</summary>
    /// <param name="other">The other point.</param>
    public double DistanceTo(Point3 other)
    {
        double dx = X - other.X;
        double dy = Y - other.Y;
        double dz = Z - other.Z;
        return Math.Sqrt(((dx * dx) + (dy * dy)) + (dz * dz));
    }
}
