using System;

namespace RecursiveAI.OnePath.NavMesh.Build3D;

/// <summary>
/// A convex prism that sets the area id of the walkable spans inside it: a convex polygon in the xz plane between two heights.
/// Use it to mark water, roads, no-go zones and the like before the region stage.
/// </summary>
public sealed class ConvexVolume
{
    private readonly double[] _xz;
    private readonly double _orientation;

    /// <summary>Creates a volume.</summary>
    /// <param name="xz">Polygon corners as x, z pairs in world units, in either winding.</param>
    /// <param name="minY">Lowest world y included.</param>
    /// <param name="maxY">Highest world y included.</param>
    /// <param name="area">Area id to assign (0 makes spans non-walkable).</param>
    /// <exception cref="ArgumentException">Fewer than 3 corners, a NaN coordinate, minY above maxY, zero area, or a polygon that is not convex and simple.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21.</exception>
    public ConvexVolume(ReadOnlySpan<double> xz, double minY, double maxY, byte area)
    {
        if (xz.Length % 2 != 0 || xz.Length < 6 || xz.Length > 2 * 256)
        {
            throw new ArgumentException($"A convex volume needs 3..256 corners given as x, z pairs; got {xz.Length} numbers.", nameof(xz));
        }
        for (int i = 0; i < xz.Length; i++)
        {
            HeightfieldFrame.CheckCoordinate(xz[i], nameof(xz));
        }
        HeightfieldFrame.CheckCoordinate(minY, nameof(minY));
        HeightfieldFrame.CheckCoordinate(maxY, nameof(maxY));
        if (minY > maxY)
        {
            throw new ArgumentException($"minY {minY} is above maxY {maxY}.");
        }
        int n = xz.Length / 2;
        double areaSum = 0;
        for (int i = 0; i < n; i++)
        {
            int j = (i + 1) % n;
            areaSum += (xz[2 * i] * xz[(2 * j) + 1]) - (xz[2 * j] * xz[(2 * i) + 1]);
        }
        if (areaSum == 0)
        {
            throw new ArgumentException("The polygon has zero area.", nameof(xz));
        }
        _orientation = areaSum > 0 ? 1.0 : -1.0;
        // Convex and simple: every corner lies on the inner side of every edge.
        for (int e = 0; e < n; e++)
        {
            int e2 = (e + 1) % n;
            for (int k = 0; k < n; k++)
            {
                if (Side(xz, e, e2, xz[2 * k], xz[(2 * k) + 1]) * _orientation < 0)
                {
                    throw new ArgumentException($"The polygon is not convex: corner {k} lies outside edge {e}.", nameof(xz));
                }
            }
        }
        _xz = xz.ToArray();
        MinY = minY;
        MaxY = maxY;
        Area = area;
    }

    /// <summary>Lowest world y included.</summary>
    public double MinY { get; }

    /// <summary>Highest world y included.</summary>
    public double MaxY { get; }

    /// <summary>The area id assigned.</summary>
    public byte Area { get; }

    /// <summary>True when the point (x, z) is inside or on the polygon.</summary>
    public bool Contains(double x, double z)
    {
        int n = _xz.Length / 2;
        for (int e = 0; e < n; e++)
        {
            if (Side(_xz, e, (e + 1) % n, x, z) * _orientation < 0)
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Sets <see cref="Area"/> on every span of <paramref name="chf"/> that has a non-zero area, whose cell centre is inside the
    /// polygon and whose floor height lies within [MinY, MaxY]. Volumes applied later overwrite earlier ones.
    /// </summary>
    /// <returns>The number of spans changed or set.</returns>
    public int Apply(CompactHeightfield chf)
    {
        ArgumentNullException.ThrowIfNull(chf);
        HeightfieldFrame f = chf.Frame;
        Span<CompactSpan> spans = chf.Spans;
        int marked = 0;
        for (int z = 0; z < f.Depth; z++)
        {
            double cz = f.CenterZ(z);
            for (int x = 0; x < f.Width; x++)
            {
                if (!Contains(f.CenterX(x), cz))
                {
                    continue;
                }
                int c = f.Column(x, z);
                for (int i = chf.CellFirst[c]; i < chf.CellFirst[c] + chf.CellCount[c]; i++)
                {
                    double wy = f.WorldY(spans[i].Y);
                    if (spans[i].Area != 0 && wy >= MinY && wy <= MaxY)
                    {
                        spans[i].Area = Area;
                        marked++;
                    }
                }
            }
        }
        return marked;
    }

    // Cross product of (corner e2 - corner e) with (p - corner e): positive when p is left of the edge.
    private static double Side(ReadOnlySpan<double> xz, int e, int e2, double px, double pz)
    {
        double ax = xz[2 * e];
        double az = xz[(2 * e) + 1];
        return ((xz[2 * e2] - ax) * (pz - az)) - ((xz[(2 * e2) + 1] - az) * (px - ax));
    }
}
