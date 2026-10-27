using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Crowds.Orca;

/// <summary>
/// A half-plane of permitted velocities: <c>(v - Point) . Normal &gt;= 0</c>, with a unit <see cref="Normal"/>.
/// Only the point and the unit normal matter. Because the normal has unit length,
/// <see cref="SignedDistance"/> is the Euclidean distance of a velocity to the delimiting line.
/// </summary>
public readonly struct HalfPlane
{
    /// <summary>A point on the delimiting line.</summary>
    public Vector2D Point { get; }

    /// <summary>The unit normal pointing into the permitted side.</summary>
    public Vector2D Normal { get; }

    /// <summary>Creates a half-plane. <paramref name="normal"/> must have unit length (not checked).</summary>
    public HalfPlane(Vector2D point, Vector2D normal)
    {
        Point = point;
        Normal = normal;
    }

    /// <summary>
    /// <c>(v - Point) . Normal</c>: positive inside the permitted side, negative when <paramref name="v"/> violates the
    /// half-plane (then its magnitude is the penetration depth).
    /// </summary>
    public double SignedDistance(Vector2D v) => Vector2D.Dot(v - Point, Normal);
}
