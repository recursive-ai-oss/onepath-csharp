using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>The convex polygons of one tile, in world-plane coordinates (plane x is world X, plane y is world Z).</summary>
public sealed class TilePartition
{
    internal TilePartition(int tileX, int tileZ, ConvexPartition partition)
    {
        TileX = tileX;
        TileZ = tileZ;
        Partition = partition;
    }

    /// <summary>Tile column.</summary>
    public int TileX { get; }

    /// <summary>Tile row.</summary>
    public int TileZ { get; }

    /// <summary>The tile's polygons and in-tile adjacency; edges on a tile line have no neighbour.</summary>
    public ConvexPartition Partition { get; }
}

/// <summary>
/// Cuts a <see cref="Region"/> along the tile lines of a <see cref="NavMeshParams"/> tiling and partitions each
/// tile into convex polygons. See <c>docs/build2d-tiles.md</c> for the rule.
/// </summary>
public static class TileSlicer
{
    /// <summary>The grid unit of a region used for tiles is at least the cell size divided by this (so grid-distinct points stay distinct as tile-local float32).</summary>
    public const int MinUnitDivisor = 256;

    /// <summary>The finest <see cref="Geometry2DOptions.SnapBits"/> (at most 16) a region may use with the given cell size.</summary>
    public static int FinestSnapBits(double cellSize)
    {
        int bits = Geometry2DOptions.MaxSnapBits;
        while (bits > 0 && Region.Pow2(-bits) < cellSize / MinUnitDivisor)
        {
            bits--;
        }
        return bits;
    }

    /// <summary>Most crossings of vertical with horizontal tile lines accepted inside one region.</summary>
    public const int MaxCrossings = 1 << 20;

    /// <summary>Most tile lines accepted along one axis.</summary>
    public const int MaxLinesPerAxis = 1 << 12;

    /// <summary>
    /// Slices the region. The tile grid starts at <c>OriginX</c>/<c>OriginZ</c> of <paramref name="p"/> and has square
    /// tiles of <c>TileSize</c>; a tile line is the grid-snapped position of its world coordinate. Tiles are returned in
    /// ascending (row, column) order and only when they hold area.
    /// </summary>
    /// <param name="region">The walkable area.</param>
    /// <param name="p">The tiling and the polygon vertex limit (<c>MaxVertsPerPoly</c>).</param>
    /// <exception cref="ArgumentOutOfRangeException">The mesh parameters are invalid.</exception>
    /// <exception cref="ArgumentException">The tile lines are too many or do not sit on the region's snap grid closely enough
    /// for tile edges to match (use a finer snap grid or a tile size and origin on the grid).</exception>
    public static IReadOnlyList<TilePartition> Slice(Region region, NavMeshParams p)
    {
        ArgumentNullException.ThrowIfNull(region);
        ArgumentNullException.ThrowIfNull(p);
        p.Validate();
        var result = new List<TilePartition>();
        if (region.SnapBits > 0 && Region.Pow2(-region.SnapBits) < p.CellSize / MinUnitDivisor)
        {
            throw new ArgumentException($"The region's snap grid (2^-{region.SnapBits}) is finer than a {MinUnitDivisor}th of the cell size {p.CellSize:R}: features that thin would collapse in the tile format. Build the region with SnapBits {FinestSnapBits(p.CellSize)} or less.", nameof(region));
        }
        if (!region.TryGetGridBounds(out long minX, out long minY, out long maxX, out long maxY))
        {
            return result;
        }
        double scale = Region.Pow2(region.SnapBits);
        Axis ax = MakeAxis(p.OriginX, p.TileSize, minX, maxX, scale, p.CellSize, "X");
        Axis az = MakeAxis(p.OriginZ, p.TileSize, minY, maxY, scale, p.CellSize, "Z");
        List<long> xCuts = ax.Cuts(minX, maxX), yCuts = az.Cuts(minY, maxY);
        if ((long)xCuts.Count * yCuts.Count > MaxCrossings)
        {
            throw new ArgumentException($"The region is cut by {xCuts.Count} x {yCuts.Count} tile lines, more than {MaxCrossings} crossings; use a larger tile size.");
        }
        Triangulation tri = region.TriangulateWithCuts(xCuts, yCuts);
        if (tri.TriangleCount == 0)
        {
            return result;
        }
        // Every triangle lies inside one tile cell, so its centroid decides the tile. Compare 3 × line with the
        // coordinate sum to stay in integers.
        ReadOnlySpan<Vector2D> vv = tri.Vertices;
        ReadOnlySpan<int> idx = tri.Indices;
        var keyed = new List<(int Z, int X, int T)>(tri.TriangleCount);
        for (int t = 0; t < tri.TriangleCount; t++)
        {
            long sx = 0, sy = 0;
            for (int k = 0; k < 3; k++)
            {
                Vector2D v = vv[idx[(3 * t) + k]];
                sx += (long)Math.Round(v.X * scale);
                sy += (long)Math.Round(v.Y * scale);
            }
            keyed.Add((az.TileOf(sy), ax.TileOf(sx), t));
        }
        keyed.Sort(static (a, b) =>
        {
            int c = a.Z.CompareTo(b.Z);
            if (c != 0)
            {
                return c;
            }
            c = a.X.CompareTo(b.X);
            return c != 0 ? c : a.T.CompareTo(b.T);
        });
        for (int i = 0; i < keyed.Count;)
        {
            int j = i;
            var tris = new List<int>();
            while (j < keyed.Count && keyed[j].Z == keyed[i].Z && keyed[j].X == keyed[i].X)
            {
                tris.Add(keyed[j].T);
                j++;
            }
            ConvexPartition part = ConvexPartition.Create(tri.Subset(tris), p.MaxVertsPerPoly);
            result.Add(new TilePartition(keyed[i].X, keyed[i].Z, part));
            i = j;
        }
        return result;
    }

    // The tile lines along one axis: line k (k = first .. first + count - 1) is the snapped world coordinate
    // origin + k × size; tile k lies between line k and line k + 1.
    private readonly struct Axis
    {
        private readonly long[] _lines;
        private readonly int _first;

        public Axis(long[] lines, int first)
        {
            _lines = lines;
            _first = first;
        }

        public List<long> Cuts(long lo, long hi)
        {
            var cuts = new List<long>();
            foreach (long l in _lines)
            {
                if (l > lo && l < hi)
                {
                    cuts.Add(l);
                }
            }
            return cuts;
        }

        // Tile index containing the point whose coordinate sum over three vertices is sum3: the last line with 3 × line <= sum3.
        public int TileOf(long sum3)
        {
            int lo = 0, hi = _lines.Length - 1;
            while (lo < hi)
            {
                int mid = lo + ((hi - lo + 1) / 2);
                if (3 * _lines[mid] <= sum3)
                {
                    lo = mid;
                }
                else
                {
                    hi = mid - 1;
                }
            }
            return _first + lo;
        }
    }

    private static Axis MakeAxis(double origin, double size, long lo, long hi, double scale, double cellSize, string name)
    {
        double wlo = lo / scale, whi = hi / scale;
        double first = Math.Floor((wlo - origin) / size) - 1;
        double last = Math.Floor((whi - origin) / size) + 1;
        if (!(last - first < MaxLinesPerAxis))
        {
            throw new ArgumentException($"The region spans more than {MaxLinesPerAxis} tiles along {name} (tile size {size:R}); use a larger tile size.");
        }
        int f = (int)first;
        int n = (int)(last - first) + 2;
        var lines = new long[n];
        double tol = cellSize * (1.0 / 1024.0);
        for (int k = 0; k < n; k++)
        {
            double world = origin + ((f + k) * size);
            long l = (long)Math.Round(world * scale);
            if (Math.Abs((l / scale) - world) > tol && l > lo && l < hi)
            {
                throw new ArgumentException($"The tile line {f + k} along {name} at {world:R} is {Math.Abs((l / scale) - world):R} from the snap grid, more than a cell/1024 ({tol:R}); use a finer snap grid or place the tile origin and size on it.");
            }
            lines[k] = l;
        }
        return new Axis(lines, f);
    }
}
