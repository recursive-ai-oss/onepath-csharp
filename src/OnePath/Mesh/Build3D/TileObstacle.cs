using System;
using System.Diagnostics.CodeAnalysis;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>The shape of a <see cref="TileObstacle"/>.</summary>
public enum ObstacleShape : byte
{
    /// <summary>A vertical cylinder: base centre, radius, height.</summary>
    Cylinder = 1,

    /// <summary>A box with sides parallel to the axes.</summary>
    Box = 2,

    /// <summary>A box turned about the vertical axis by a yaw angle.</summary>
    OrientedBox = 3,
}

/// <summary>
/// A temporary obstacle: a solid that blocks walking while it exists (a barricade, a parked vehicle, a fallen tree). After erosion, every
/// walkable span whose cell centre lies inside the shape grown by the agent radius and whose floor lies in the shape's height range
/// [<see cref="MinY"/>, <see cref="MaxY"/>] is made non-walkable, so the agent radius stays clear of it like any other wall without eroding the tile again. Obstacles are immutable; add them to a <see cref="TileCache"/>
/// (or list them on <see cref="Mesh3DPipeline.Obstacles"/> to bake them into a build).
/// </summary>
public sealed class TileObstacle
{
    private readonly double[] _p;
    private readonly double _sin;
    private readonly double _cos;

    private TileObstacle(ObstacleShape shape, double[] p)
    {
        Shape = shape;
        _p = p;
        switch (shape)
        {
            case ObstacleShape.Cylinder:
                MinX = p[0] - p[3];
                MaxX = p[0] + p[3];
                MinZ = p[2] - p[3];
                MaxZ = p[2] + p[3];
                MinY = p[1];
                MaxY = p[1] + p[4];
                break;
            case ObstacleShape.Box:
                MinX = p[0];
                MinY = p[1];
                MinZ = p[2];
                MaxX = p[3];
                MaxY = p[4];
                MaxZ = p[5];
                break;
            default:
                // The yaw's sine and cosine come from the deterministic library, so every platform turns the box identically.
                (_sin, _cos) = Math64.SinCos(p[6]);
                double ex = (p[3] * Math.Abs(_cos)) + (p[5] * Math.Abs(_sin));
                double ez = (p[3] * Math.Abs(_sin)) + (p[5] * Math.Abs(_cos));
                MinX = p[0] - ex;
                MaxX = p[0] + ex;
                MinZ = p[2] - ez;
                MaxZ = p[2] + ez;
                MinY = p[1] - p[4];
                MaxY = p[1] + p[4];
                break;
        }
    }

    /// <summary>Which kind of shape this is; <see cref="Parameters"/> lists its numbers.</summary>
    public ObstacleShape Shape { get; }

    /// <summary>Lowest world y affected (world units).</summary>
    public double MinY { get; }

    /// <summary>Highest world y affected (world units).</summary>
    public double MaxY { get; }

    /// <summary>Smallest x of the shape's bounding box.</summary>
    public double MinX { get; }

    /// <summary>Largest x of the shape's bounding box.</summary>
    public double MaxX { get; }

    /// <summary>Smallest z of the shape's bounding box.</summary>
    public double MinZ { get; }

    /// <summary>Largest z of the shape's bounding box.</summary>
    public double MaxZ { get; }

    /// <summary>The defining numbers: cylinder (x, y, z of the base centre, radius, height); box (min x, y, z, max x, y, z); oriented box (centre x, y, z, half sizes x, y, z, yaw).</summary>
    public ReadOnlySpan<double> Parameters => _p;

    /// <summary>A vertical cylinder.</summary>
    /// <param name="x">Base centre x (world units).</param>
    /// <param name="y">Base centre y: the lowest floor height affected.</param>
    /// <param name="z">Base centre z.</param>
    /// <param name="radius">Radius, above 0.</param>
    /// <param name="height">Height above the base, 0 or more.</param>
    /// <exception cref="ArgumentException">A value is NaN or infinite, the radius is not above 0 or the height is negative.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate or its extent is beyond ±2^21.</exception>
    public static TileObstacle Cylinder(double x, double y, double z, double radius, double height)
    {
        Check(x, nameof(x));
        Check(y, nameof(y));
        Check(z, nameof(z));
        CheckPositive(radius, nameof(radius));
        CheckNonNegative(height, nameof(height));
        Check(x - radius, nameof(radius));
        Check(x + radius, nameof(radius));
        Check(z - radius, nameof(radius));
        Check(z + radius, nameof(radius));
        Check(y + height, nameof(height));
        return new TileObstacle(ObstacleShape.Cylinder, new[] { x, y, z, radius, height });
    }

    /// <summary>An axis-aligned box. All values are world units; the corners are inclusive.</summary>
    /// <exception cref="ArgumentException">A value is NaN or infinite, or a minimum is above its maximum.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is beyond ±2^21.</exception>
    public static TileObstacle Box(double minX, double minY, double minZ, double maxX, double maxY, double maxZ)
    {
        Check(minX, nameof(minX));
        Check(minY, nameof(minY));
        Check(minZ, nameof(minZ));
        Check(maxX, nameof(maxX));
        Check(maxY, nameof(maxY));
        Check(maxZ, nameof(maxZ));
        if (minX > maxX || minY > maxY || minZ > maxZ)
        {
            throw new ArgumentException("A box minimum is above its maximum.");
        }
        return new TileObstacle(ObstacleShape.Box, new[] { minX, minY, minZ, maxX, maxY, maxZ });
    }

    /// <summary>A box turned about the vertical axis.</summary>
    /// <param name="x">Centre x.</param>
    /// <param name="y">Centre y.</param>
    /// <param name="z">Centre z.</param>
    /// <param name="halfX">Half size along the box's own x axis, above 0.</param>
    /// <param name="halfY">Half height, 0 or more.</param>
    /// <param name="halfZ">Half size along the box's own z axis, above 0.</param>
    /// <param name="yaw">Rotation in radians from the world x axis towards the world z axis; any finite value.</param>
    /// <exception cref="ArgumentException">A value is NaN or infinite, or a half size is not positive.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate or the box's extent is beyond ±2^21.</exception>
    public static TileObstacle OrientedBox(double x, double y, double z, double halfX, double halfY, double halfZ, double yaw)
    {
        Check(x, nameof(x));
        Check(y, nameof(y));
        Check(z, nameof(z));
        CheckPositive(halfX, nameof(halfX));
        CheckNonNegative(halfY, nameof(halfY));
        CheckPositive(halfZ, nameof(halfZ));
        if (double.IsNaN(yaw) || double.IsInfinity(yaw))
        {
            throw new ArgumentException($"yaw is {yaw}; it must be finite.", nameof(yaw));
        }
        var o = new TileObstacle(ObstacleShape.OrientedBox, new[] { x, y, z, halfX, halfY, halfZ, yaw });
        Check(o.MinX, nameof(halfX));
        Check(o.MaxX, nameof(halfX));
        Check(o.MinZ, nameof(halfZ));
        Check(o.MaxZ, nameof(halfZ));
        Check(o.MinY, nameof(halfY));
        Check(o.MaxY, nameof(halfY));
        return o;
    }

    /// <summary>Rebuilds an obstacle from its shape and <see cref="Parameters"/> (what a snapshot stores); the factory rules apply.</summary>
    internal static TileObstacle FromParameters(ObstacleShape shape, ReadOnlySpan<double> p)
    {
        if (p.Length != ParameterCount(shape))
        {
            throw new ArgumentException($"{shape} takes {ParameterCount(shape)} numbers, got {p.Length}.", nameof(p));
        }
        return shape switch
        {
            ObstacleShape.Cylinder => Cylinder(p[0], p[1], p[2], p[3], p[4]),
            ObstacleShape.Box => Box(p[0], p[1], p[2], p[3], p[4], p[5]),
            ObstacleShape.OrientedBox => OrientedBox(p[0], p[1], p[2], p[3], p[4], p[5], p[6]),
            _ => throw new ArgumentException($"Unknown obstacle shape {(int)shape}.", nameof(shape)),
        };
    }

    internal static int ParameterCount(ObstacleShape shape) => shape switch
    {
        ObstacleShape.Cylinder => 5,
        ObstacleShape.Box => 6,
        ObstacleShape.OrientedBox => 7,
        _ => -1,
    };

    /// <summary>True when the point (x, z) is inside or on the shape's footprint.</summary>
    public bool ContainsXZ(double x, double z)
    {
        switch (Shape)
        {
            case ObstacleShape.Cylinder:
            {
                double dx = x - _p[0], dz = z - _p[2];
                return (dx * dx) + (dz * dz) <= _p[3] * _p[3];
            }
            case ObstacleShape.Box:
                return x >= MinX && x <= MaxX && z >= MinZ && z <= MaxZ;
            default:
            {
                double dx = x - _p[0], dz = z - _p[2];
                double u = (dx * _cos) + (dz * _sin);
                double v = (dz * _cos) - (dx * _sin);
                return Math.Abs(u) <= _p[3] && Math.Abs(v) <= _p[5];
            }
        }
    }

    // The footprint test with the shape grown by `grow` on every side (a round shape grows round, a box square).
    internal bool ContainsGrown(double x, double z, double grow)
    {
        switch (Shape)
        {
            case ObstacleShape.Cylinder:
            {
                double dx = x - _p[0], dz = z - _p[2], r = _p[3] + grow;
                return (dx * dx) + (dz * dz) <= r * r;
            }
            case ObstacleShape.Box:
                return x >= MinX - grow && x <= MaxX + grow && z >= MinZ - grow && z <= MaxZ + grow;
            default:
            {
                double dx = x - _p[0], dz = z - _p[2];
                double u = (dx * _cos) + (dz * _sin);
                double v = (dz * _cos) - (dx * _sin);
                return Math.Abs(u) <= _p[3] + grow && Math.Abs(v) <= _p[5] + grow;
            }
        }
    }

    /// <summary>
    /// Makes the spans of <paramref name="chf"/> (already eroded) that this obstacle covers non-walkable (area 0). The footprint is grown by
    /// <paramref name="grow"/>, the agent radius, because erosion already ran: it stands in for the erosion the obstacle would have caused.
    /// </summary>
    /// <returns>The number of spans changed.</returns>
    internal int Mark(CompactHeightfield chf, double grow)
    {
        HeightfieldFrame f = chf.Frame;
        Span<CompactSpan> spans = chf.Spans;
        ReadOnlySpan<int> first = chf.CellFirst;
        ReadOnlySpan<int> count = chf.CellCount;
        int marked = 0;
        double reach = grow + f.CellSize;
        for (int z = 0; z < f.Depth; z++)
        {
            double cz = f.CenterZ(z);
            if (cz < MinZ - reach || cz > MaxZ + reach)
            {
                continue;
            }
            for (int x = 0; x < f.Width; x++)
            {
                double cx = f.CenterX(x);
                if (cx < MinX - reach || cx > MaxX + reach || !ContainsGrown(cx, cz, grow))
                {
                    continue;
                }
                int c = f.Column(x, z);
                for (int i = first[c]; i < first[c] + count[c]; i++)
                {
                    double wy = f.WorldY(spans[i].Y);
                    if (spans[i].Area != 0 && wy >= MinY && wy <= MaxY)
                    {
                        spans[i].Area = 0;
                        marked++;
                    }
                }
            }
        }
        return marked;
    }

    private static void Check(double v, string name) => HeightfieldFrame.CheckCoordinate(v, name);

    private static void CheckPositive(double v, string name)
    {
        if (!(v > 0) || double.IsInfinity(v))
        {
            throw new ArgumentException($"{name} is {v}; it must be above 0 and finite.", name);
        }
    }

    private static void CheckNonNegative(double v, string name)
    {
        if (!(v >= 0) || double.IsInfinity(v))
        {
            throw new ArgumentException($"{name} is {v}; it must be 0 or more and finite.", name);
        }
    }
}
