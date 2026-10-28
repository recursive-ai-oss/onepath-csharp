using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Build2D;

/// <summary>Options shared by the 2D geometry stages.</summary>
public sealed class Geometry2DOptions
{
    /// <summary>The largest allowed <see cref="SnapBits"/>.</summary>
    public const int MaxSnapBits = 16;

    private int _snapBits = 16;

    /// <summary>
    /// Every coordinate is rounded to the nearest multiple of 2^-SnapBits before any computation, and every
    /// constructed point (intersections, offsets) is rounded to the same grid. Default 16 (about 1.5e-5 units),
    /// range 0 to 16. All topology is decided exactly on the snapped points.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">The value is outside 0 to 16.</exception>
    public int SnapBits
    {
        get => _snapBits;
        set
        {
            if (value < 0 || value > MaxSnapBits)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "SnapBits must be between 0 and " + MaxSnapBits + ".");
            }
            _snapBits = value;
        }
    }

    internal static Geometry2DOptions Default { get; } = new();
}

/// <summary>How offset boundaries are joined around convex corners.</summary>
public enum JoinStyle
{
    /// <summary>
    /// Circular arcs, approximated by a circumscribed regular polygon whose vertices lie at most the arc tolerance
    /// outside the true circle (so an inflated obstacle never undershoots the true offset).
    /// </summary>
    Round = 0,

    /// <summary>The offset is the Minkowski sum with an axis-aligned square of half-side equal to the distance.</summary>
    Square = 1,
}

/// <summary>A polygon with holes as given by the caller: rings of any orientation, filled by the even-odd rule.</summary>
public sealed class Outline
{
    private readonly Vector2D[][] _rings;

    /// <summary>Creates an outline from its rings; each ring is a closed polygon (the last vertex connects to the first).</summary>
    /// <param name="rings">Every ring of the polygon (outer boundary and holes), any order and orientation. At least one ring.</param>
    /// <exception cref="ArgumentNullException"><paramref name="rings"/> or a ring is null.</exception>
    /// <exception cref="ArgumentException">No rings, or a ring has fewer than 3 vertices.</exception>
    public Outline(IEnumerable<IReadOnlyList<Vector2D>> rings)
    {
        ArgumentNullException.ThrowIfNull(rings);
        var list = new List<Vector2D[]>();
        foreach (IReadOnlyList<Vector2D> r in rings)
        {
            if (r == null)
            {
                throw new ArgumentNullException(nameof(rings), "A ring is null.");
            }
            if (r.Count < 3)
            {
                throw new ArgumentException("Ring " + list.Count + " has " + r.Count + " vertices; a ring needs at least 3.", nameof(rings));
            }
            var a = new Vector2D[r.Count];
            for (int i = 0; i < a.Length; i++)
            {
                a[i] = r[i];
            }
            list.Add(a);
        }
        if (list.Count == 0)
        {
            throw new ArgumentException("An outline needs at least one ring.", nameof(rings));
        }
        _rings = list.ToArray();
    }

    /// <summary>Creates an outline from an outer ring and optional holes (orientation of each is irrelevant).</summary>
    public Outline(IReadOnlyList<Vector2D> outer, IReadOnlyList<IReadOnlyList<Vector2D>>? holes = null)
        : this(Join(outer, holes))
    {
    }

    /// <summary>Number of rings.</summary>
    public int RingCount => _rings.Length;

    /// <summary>The vertices of ring <paramref name="index"/> (a copy).</summary>
    public Vector2D[] GetRing(int index) => (Vector2D[])_rings[index].Clone();

    internal Vector2D[][] RawRings => _rings;

    private static IEnumerable<IReadOnlyList<Vector2D>> Join(IReadOnlyList<Vector2D> outer, IReadOnlyList<IReadOnlyList<Vector2D>>? holes)
    {
        ArgumentNullException.ThrowIfNull(outer);
        yield return outer;
        if (holes != null)
        {
            foreach (IReadOnlyList<Vector2D> h in holes)
            {
                yield return h;
            }
        }
    }
}

/// <summary>
/// A planar region with exact, non-crossing boundary rings on the snap grid. Outer boundaries run counter-clockwise
/// and holes clockwise, so the interior is always on the left. Regions are immutable.
/// </summary>
/// <remarks>
/// <para>All constructions snap to the grid of <see cref="Geometry2DOptions.SnapBits"/> and decide topology exactly on
/// it, so the result never self-intersects. Rings may touch at single vertices. The result depends only on the set of
/// input edges: reordering outlines, rings or start vertices gives an identical region, ring order and start vertices.</para>
/// <para>Coordinates must lie within ±2^21 (the range in which a Q31.32 fixed-point value converts to double exactly).</para>
/// </remarks>
public sealed class Region
{
    private const double CoordinateBound = 2097152.0; // 2^21

    private readonly P[][] _rings;
    private Vector2D[][]? _public;

    internal Region(List<P[]> rings, int bits)
    {
        _rings = rings.ToArray();
        SnapBits = bits;
    }

    /// <summary>The snap grid exponent this region lives on.</summary>
    public int SnapBits { get; }

    /// <summary>True when the region has no area.</summary>
    public bool IsEmpty => _rings.Length == 0;

    /// <summary>Number of boundary rings (outer boundaries and holes).</summary>
    public int RingCount => _rings.Length;

    internal P[][] Rings => _rings;

    /// <summary>The vertices of ring <paramref name="index"/>, exact on the snap grid (a copy).</summary>
    public Vector2D[] GetRing(int index)
    {
        _public ??= new Vector2D[_rings.Length][];
        Vector2D[] r = _public[index] ??= ToPublic(_rings[index], SnapBits);
        return (Vector2D[])r.Clone();
    }

    /// <summary>Total area (outer rings minus holes), computed exactly in integers and converted once.</summary>
    public double Area
    {
        get
        {
            Int128 twice = 0;
            foreach (P[] r in _rings)
            {
                for (int i = 0; i < r.Length; i++)
                {
                    P a = r[i], b = r[(i + 1) % r.Length];
                    twice += ((Int128)a.X * b.Y) - ((Int128)a.Y * b.X);
                }
            }
            return (double)twice * 0.5 * UnitSquared(SnapBits);
        }
    }

    /// <summary>An empty region.</summary>
    public static Region Empty(Geometry2DOptions? options = null) => new(new List<P[]>(), (options ?? Geometry2DOptions.Default).SnapBits);

    // ---- construction ----

    /// <summary>
    /// The union of outlines. Each outline is first resolved on its own by the even-odd rule over its rings (so
    /// holes work in any orientation, and self-intersecting or overlapping rings are cleaned), then the outlines are united.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="outlines"/> or an outline is null.</exception>
    /// <exception cref="ArgumentException">No outlines.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN, infinite or beyond ±2^21; the message names the outline, ring and vertex.</exception>
    public static Region FromOutlines(IEnumerable<Outline> outlines, Geometry2DOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(outlines);
        int bits = (options ?? Geometry2DOptions.Default).SnapBits;
        var resolved = new List<Region>();
        int oi = 0;
        foreach (Outline o in outlines)
        {
            if (o == null)
            {
                throw new ArgumentNullException(nameof(outlines), "Outline " + oi + " is null.");
            }
            var edges = new List<InEdge>();
            for (int ri = 0; ri < o.RawRings.Length; ri++)
            {
                Vector2D[] ring = o.RawRings[ri];
                var pts = new P[ring.Length];
                for (int vi = 0; vi < ring.Length; vi++)
                {
                    pts[vi] = Snap(ring[vi], bits, "outline " + oi + ", ring " + ri + ", vertex " + vi);
                }
                for (int vi = 0; vi < pts.Length; vi++)
                {
                    edges.Add(new InEdge(pts[vi], pts[(vi + 1) % pts.Length], 0));
                }
            }
            resolved.Add(Resolve(edges, BoolOp.EvenOdd, bits));
            oi++;
        }
        if (oi == 0)
        {
            throw new ArgumentException("At least one outline is required.", nameof(outlines));
        }
        return resolved.Count == 1 ? resolved[0] : Union(resolved);
    }

    /// <summary>
    /// The region covered by the true cells of a tile map. <paramref name="cells"/> holds <c>width * height</c> flags in
    /// row-major order (index <c>y * width + x</c>); cell (x, y) spans <c>origin + (x, y) * cellSize</c> to
    /// <c>origin + (x + 1, y + 1) * cellSize</c>. Adjacent cells merge into one outline exactly.
    /// </summary>
    /// <exception cref="ArgumentException">The span length is not <c>width * height</c>, or a size is not positive, or the cell size is smaller than the snap grid unit.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A grid line falls beyond ±2^21 or a number is not finite.</exception>
    public static Region FromTileMap(ReadOnlySpan<bool> cells, int width, int height, double cellSize, Vector2D origin, Geometry2DOptions? options = null)
    {
        int bits = (options ?? Geometry2DOptions.Default).SnapBits;
        if (width <= 0 || height <= 0)
        {
            throw new ArgumentException("Tile map width and height must be positive.");
        }
        if (cells.Length != (long)width * height)
        {
            throw new ArgumentException("Tile map has " + cells.Length + " cells; expected " + ((long)width * height) + ".");
        }
        if (!(cellSize > 0) || !double.IsFinite(cellSize))
        {
            throw new ArgumentOutOfRangeException(nameof(cellSize), cellSize, "Cell size must be positive and finite.");
        }
        if (cellSize * Pow2(bits) < 1.0)
        {
            throw new ArgumentException("Cell size " + cellSize + " is smaller than the snap grid unit 2^-" + bits + ".");
        }
        // Grid lines are computed once per index, so neighbouring cells share exactly the same snapped coordinates.
        var xs = new long[width + 1];
        var ys = new long[height + 1];
        for (int i = 0; i <= width; i++)
        {
            xs[i] = Snap(new Vector2D(origin.X + (i * cellSize), 0), bits, "tile map column line " + i).X;
        }
        for (int j = 0; j <= height; j++)
        {
            ys[j] = Snap(new Vector2D(0, origin.Y + (j * cellSize)), bits, "tile map row line " + j).Y;
        }
        bool[] solid = cells.ToArray();
        bool Solid(int x, int y) => x >= 0 && y >= 0 && x < width && y < height && solid[(y * width) + x];
        var edges = new List<(P, P)>();
        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (!Solid(x, y))
                {
                    continue;
                }
                // Interior on the left of each directed boundary edge: counter-clockwise around solid cells.
                if (!Solid(x, y - 1)) { edges.Add((new P(xs[x], ys[y]), new P(xs[x + 1], ys[y]))); }
                if (!Solid(x + 1, y)) { edges.Add((new P(xs[x + 1], ys[y]), new P(xs[x + 1], ys[y + 1]))); }
                if (!Solid(x, y + 1)) { edges.Add((new P(xs[x + 1], ys[y + 1]), new P(xs[x], ys[y + 1]))); }
                if (!Solid(x - 1, y)) { edges.Add((new P(xs[x], ys[y + 1]), new P(xs[x], ys[y]))); }
            }
        }
        return new Region(Labeling.TraceRings(edges), bits);
    }

    // ---- boolean operations ----

    /// <summary>The union of this region and <paramref name="other"/>.</summary>
    public Region Union(Region other) => Combine(this, other, BoolOp.Union);

    /// <summary>The intersection of this region and <paramref name="other"/>.</summary>
    public Region Intersection(Region other) => Combine(this, other, BoolOp.Intersection);

    /// <summary>This region minus <paramref name="other"/>.</summary>
    public Region Difference(Region other) => Combine(this, other, BoolOp.Difference);

    /// <summary>The points in exactly one of this region and <paramref name="other"/>.</summary>
    public Region SymmetricDifference(Region other) => Combine(this, other, BoolOp.Xor);

    /// <summary>The union of several regions (all on the same snap grid).</summary>
    /// <exception cref="ArgumentException">No regions, or regions on different grids.</exception>
    public static Region Union(IEnumerable<Region> regions)
    {
        ArgumentNullException.ThrowIfNull(regions);
        var edges = new List<InEdge>();
        int bits = -1;
        foreach (Region r in regions)
        {
            ArgumentNullException.ThrowIfNull(r);
            if (bits >= 0 && r.SnapBits != bits)
            {
                throw new ArgumentException("Regions use different snap grids (" + bits + " and " + r.SnapBits + ").", nameof(regions));
            }
            bits = r.SnapBits;
            r.AppendEdges(edges, 0);
        }
        if (bits < 0)
        {
            throw new ArgumentException("At least one region is required.", nameof(regions));
        }
        return Resolve(edges, BoolOp.Positive, bits);
    }

    private static Region Combine(Region a, Region b, BoolOp op)
    {
        ArgumentNullException.ThrowIfNull(b);
        if (a.SnapBits != b.SnapBits)
        {
            throw new ArgumentException("Regions use different snap grids (" + a.SnapBits + " and " + b.SnapBits + ").", nameof(b));
        }
        var edges = new List<InEdge>();
        a.AppendEdges(edges, 0);
        b.AppendEdges(edges, 1);
        return Resolve(edges, op, a.SnapBits);
    }

    internal void AppendEdges(List<InEdge> edges, int op)
    {
        foreach (P[] r in _rings)
        {
            for (int i = 0; i < r.Length; i++)
            {
                edges.Add(new InEdge(r[i], r[(i + 1) % r.Length], op));
            }
        }
    }

    internal static Region Resolve(List<InEdge> edges, BoolOp op, int bits)
    {
        Arrangement arr = Arrangement.Build(edges);
        List<(P A, P B)> boundary = Labeling.Boundary(arr, op);
        return new Region(Labeling.TraceRings(boundary), bits);
    }

    // ---- offsetting ----

    /// <summary>
    /// Grows (positive distance) or shrinks (negative) the region by moving its boundary. Growing unites the region
    /// with everything within the distance of its boundary; shrinking removes it, which is how a walkable area is
    /// eroded by an agent radius. The offset is the exact Minkowski construction (sweeping a disc or square along every
    /// boundary edge), so thin parts vanish and narrow gaps close correctly.
    /// </summary>
    /// <param name="distance">Signed distance in world units; |distance| at most 2^20. Zero returns this region.</param>
    /// <param name="join">The shape swept along the boundary; see <see cref="JoinStyle"/>.</param>
    /// <param name="arcTolerance">For round joins, the largest distance the polygonal arc may lie outside the true circle.
    /// Values not above zero choose |distance| / 128; the effective minimum is one grid unit.</param>
    /// <exception cref="ArgumentOutOfRangeException">The distance is not finite or too large, or the result would leave ±2^21.</exception>
    public Region Offset(double distance, JoinStyle join = JoinStyle.Round, double arcTolerance = 0) =>
        OffsetCore(distance, join, arcTolerance, oneSided: false);

    /// <summary>
    /// Shrinks by <paramref name="distance"/> (positive, round joins) with only the pieces that can change the result:
    /// a rectangle on the interior side of each edge and, at each reflex vertex, the circular sector between the two edge
    /// normals. Every point of the region nearest to the boundary at a convex vertex is nearer to an adjacent edge, so the
    /// removed set is the same in exact arithmetic; the polygonal arcs and snap rounding see fewer, differently placed
    /// pieces, so the rings can differ from <see cref="Offset"/> by up to the arc tolerance. About a tenth of the edges, so
    /// the noding is that much cheaper.
    /// </summary>
    internal Region ShrinkOneSided(double distance, double arcTolerance = 0)
    {
        if (!(distance > 0))
        {
            throw new ArgumentOutOfRangeException(nameof(distance), distance, "Shrink distance must be positive.");
        }
        return OffsetCore(-distance, JoinStyle.Round, arcTolerance, oneSided: true);
    }

    private Region OffsetCore(double distance, JoinStyle join, double arcTolerance, bool oneSided)
    {
        if (!double.IsFinite(distance) || Math.Abs(distance) > 1048576.0)
        {
            throw new ArgumentOutOfRangeException(nameof(distance), distance, "Offset distance must be finite and at most 2^20 in magnitude.");
        }
        if (!double.IsFinite(arcTolerance))
        {
            throw new ArgumentOutOfRangeException(nameof(arcTolerance), arcTolerance, "Arc tolerance must be finite.");
        }
        if (distance == 0.0 || IsEmpty)
        {
            return this;
        }
        double unit = Pow2(SnapBits);
        double radius = Math.Abs(distance) * unit; // in grid units
        var edges = new List<InEdge>();
        if (oneSided)
        {
            AppendShrinkEdges(edges, radius, arcTolerance * unit, distance);
            return Resolve(edges, BoolOp.Difference, SnapBits);
        }
        AppendEdges(edges, 0);
        List<P[]> pieces = join == JoinStyle.Square ? SquarePieces(radius)
            : RoundPieces(radius, arcTolerance * unit);
        foreach (P[] piece in pieces)
        {
            for (int i = 0; i < piece.Length; i++)
            {
                P a = piece[i], b = piece[(i + 1) % piece.Length];
                if (Math.Abs(a.X) > Geo.MaxGrid / 2 || Math.Abs(a.Y) > Geo.MaxGrid / 2)
                {
                    throw new ArgumentOutOfRangeException(nameof(distance), distance, "The offset would move the boundary beyond ±2^21.");
                }
                edges.Add(new InEdge(a, b, 1));
            }
        }
        return Resolve(edges, distance > 0 ? BoolOp.Union : BoolOp.Difference, SnapBits);
    }

    // Edges of the shrink pieces: the interior side (left) of each edge. At each vertex that is reflex for the interior,
    // the swept disc only matters between the two edge normals, so the piece is that circular sector (its two straight
    // sides are the rectangles' end sides, rounded by the same expression, so the pieces meet exactly).
    //
    // The pieces are not emitted as closed polygons. A rectangle's side along the region's own edge is the region edge
    // itself, so that edge is added once, counting for both operands. Where a sector exists, its two straight sides
    // and the rectangles' end sides they meet run over the same points in opposite directions, so they cancel in the
    // arrangement; leaving them out gives the same planar subdivision with far fewer edges to node.
    private void AppendShrinkEdges(List<InEdge> edges, double radius, double tol, double distance)
    {
        double[] disc = DiscUnitVectors(radius, tol, out int m, equalArea: true);
        var selected = new List<int>(m);
        var sector = new List<P>(m + 3);
        foreach (P[] r in _rings)
        {
            int n = r.Length;
            // Per vertex: the arc chain (tip on the outgoing normal, arc vertices, tip on the incoming normal), or null.
            var arcs = new P[n][];
            for (int i = 0; i < n; i++)
            {
                P prev = r[(i + n - 1) % n], a = r[i], b = r[(i + 1) % n];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double len = Math.Sqrt((dx * dx) + (dy * dy));
                if (len == 0)
                {
                    continue;
                }
                double nx = -dy / len * radius, ny = dx / len * radius;

                // Interior on the left: a right turn is reflex for the interior.
                if (Geo.Orient(prev, a, b) >= 0)
                {
                    continue;
                }
                double pdx = a.X - prev.X, pdy = a.Y - prev.Y;
                double plen = Math.Sqrt((pdx * pdx) + (pdy * pdy));
                if (plen == 0)
                {
                    continue;
                }
                double inx = -pdy / plen * radius, iny = pdx / plen * radius; // normal of the edge arriving at a
                // Sector directions, counter-clockwise from the outgoing normal to the incoming one (less than a half turn).
                double fx = nx, fy = ny, tx = inx, ty = iny;
                selected.Clear();
                for (int k = 0; k < m; k++)
                {
                    double wx = disc[2 * k], wy = disc[(2 * k) + 1];
                    if (((fx * wy) - (fy * wx)) > 0 && ((wx * ty) - (wy * tx)) > 0)
                    {
                        selected.Add(k);
                    }
                }
                // The selected vectors are consecutive around the circle; start at the one after the gap.
                int first = 0;
                for (int q = 0; q < selected.Count; q++)
                {
                    if (((selected[(q + selected.Count - 1) % selected.Count] + 1) % m) != selected[q])
                    {
                        first = q;
                        break;
                    }
                }
                sector.Clear();
                sector.Add(a);
                void Push(P q)
                {
                    if (sector[^1] != q)
                    {
                        sector.Add(q);
                    }
                }
                Push(Round(a.X + fx, a.Y + fy));
                for (int q = 0; q < selected.Count; q++)
                {
                    int k = selected[(first + q) % selected.Count];
                    Push(Round(a.X + disc[2 * k], a.Y + disc[(2 * k) + 1]));
                }
                Push(Round(a.X + tx, a.Y + ty));
                if (sector.Count > 1 && sector[^1] == sector[0])
                {
                    sector.RemoveAt(sector.Count - 1);
                }
                if (sector.Count >= 3)
                {
                    arcs[i] = sector.GetRange(1, sector.Count - 1).ToArray();
                }
            }

            for (int i = 0; i < n; i++)
            {
                P a = r[i], b = r[(i + 1) % n];
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double len = Math.Sqrt((dx * dx) + (dy * dy));
                if (len == 0)
                {
                    continue;
                }
                double nx = -dy / len * radius, ny = dx / len * radius;
                P bq = Round(b.X + nx, b.Y + ny), aq = Round(a.X + nx, a.Y + ny);
                CheckOffsetRange(a, distance);
                CheckOffsetRange(bq, distance);
                CheckOffsetRange(aq, distance);
                edges.Add(new InEdge(a, b, 2));
                if (arcs[(i + 1) % n] == null)
                {
                    edges.Add(new InEdge(b, bq, 1));
                }
                edges.Add(new InEdge(bq, aq, 1));
                if (arcs[i] == null)
                {
                    edges.Add(new InEdge(aq, a, 1));
                }
                else
                {
                    P[] chain = arcs[i];
                    for (int q = 0; q + 1 < chain.Length; q++)
                    {
                        CheckOffsetRange(chain[q], distance);
                        edges.Add(new InEdge(chain[q], chain[q + 1], 1));
                    }
                }
            }
        }
    }

    private static void CheckOffsetRange(P a, double distance)
    {
        if (Math.Abs(a.X) > Geo.MaxGrid / 2 || Math.Abs(a.Y) > Geo.MaxGrid / 2)
        {
            throw new ArgumentOutOfRangeException(nameof(distance), distance, "The offset would move the boundary beyond ±2^21.");
        }
    }

    private static void AddDiscs(List<P[]> pieces, List<P> verts, double radius, double tol)
    {
        verts.Sort(static (x, y) => x.CompareTo(y));
        double[] disc = DiscUnitVectors(radius, tol, out int m);
        for (int i = 0; i < verts.Count; i++)
        {
            if (i > 0 && verts[i] == verts[i - 1])
            {
                continue;
            }
            var poly = new List<P>(m);
            for (int k = 0; k < m; k++)
            {
                P q = Round(verts[i].X + disc[2 * k], verts[i].Y + disc[(2 * k) + 1]);
                if (poly.Count == 0 || poly[^1] != q)
                {
                    poly.Add(q);
                }
            }
            if (poly.Count > 1 && poly[0] == poly[^1])
            {
                poly.RemoveAt(poly.Count - 1);
            }
            if (poly.Count >= 3)
            {
                pieces.Add(poly.ToArray());
            }
        }
    }

    private List<P[]> RoundPieces(double radius, double tol)
    {
        var pieces = new List<P[]>();
        // Rectangles along each edge, discs at each vertex: together the stadium swept by a disc along the edge.
        var verts = new List<P>();
        foreach (P[] r in _rings)
        {
            for (int i = 0; i < r.Length; i++)
            {
                P a = r[i], b = r[(i + 1) % r.Length];
                verts.Add(a);
                double dx = b.X - a.X, dy = b.Y - a.Y;
                double len = Math.Sqrt((dx * dx) + (dy * dy));
                if (len == 0)
                {
                    continue;
                }
                double nx = -dy / len * radius, ny = dx / len * radius;
                pieces.Add(new[]
                {
                    Round(a.X - nx, a.Y - ny), Round(b.X - nx, b.Y - ny), Round(b.X + nx, b.Y + ny), Round(a.X + nx, a.Y + ny),
                });
            }
        }
        AddDiscs(pieces, verts, radius, tol);
        return pieces;
    }

    // Vertex offsets (x0, y0, x1, y1, ...) of a counter-clockwise regular polygon circumscribed about a circle of the
    // given radius, with m = 8, 16, ... 256 vertices chosen so the vertices stay within tol of the circle. Uses only
    // + - * / and sqrt (half-angle recurrences), so it is identical everywhere.
    private static double[] DiscUnitVectors(double radius, double tol, out int m, bool equalArea = false)
    {
        if (!(tol >= 1.0))
        {
            tol = Math.Max(1.0, radius / 128.0);
        }
        double c = 0.0; // cos of the full step angle for m = 4
        double s = 1.0;
        m = 4;
        double ch, sh;
        while (true)
        {
            m *= 2;
            s = Math.Sqrt((1.0 - c) * 0.5);
            c = Math.Sqrt((1.0 + c) * 0.5);
            ch = Math.Sqrt((1.0 + c) * 0.5);
            sh = Math.Sqrt((1.0 - c) * 0.5);
            if (radius / ch - radius <= tol || m >= 256)
            {
                break;
            }
        }
        double scale = radius / ch;
        var v = new double[2 * m];
        double x = ch, y = sh; // direction at half a step: the polygon's first vertex
        for (int k = 0; k < m; k++)
        {
            v[2 * k] = x * scale;
            v[(2 * k) + 1] = y * scale;
            double nx = (x * c) - (y * s);
            double ny = (x * s) + (y * c);
            x = nx;
            y = ny;
        }
        if (equalArea)
        {
            EqualAreaQuadrant(v, m, radius, ch, sh);
        }
        return v;
    }

    // Reshapes the circumscribed polygon so that a quarter sector (the two tips on the circle at axis directions, the
    // polygon's vertices between them) has exactly the area of a quarter disc, instead of the circumscribed polygon's
    // excess. The first and last vertex of a quadrant slide along the tangent lines through the tips (so walls that run
    // along the axes continue straight, and those vertices stay collinear and are dropped); the vertices between are
    // scaled towards the centre by the same factor lambda. The area is quadratic in lambda; it is found by a fixed 60
    // bisection steps on [0, 1] (the circumscribed polygon is larger than the disc at 1, the bare triangle smaller at 0),
    // so the result is the same everywhere. The other three quadrants are 90-degree rotations of the first.
    private static void EqualAreaQuadrant(double[] v, int m, double r, double ch, double sh)
    {
        int q = m / 4;
        double t = r * sh / ch; // tangent offset of the circumscribed polygon's end vertices
        double lo = 0.0, hi = 1.0;
        for (int it = 0; it < 60; it++)
        {
            double mid = (lo + hi) * 0.5;
            if (QuadrantTwiceArea(v, q, r, t, mid) < Math.PI * r * r * 0.5)
            {
                lo = mid;
            }
            else
            {
                hi = mid;
            }
        }
        double lambda = hi;
        for (int k = 0; k < q; k++)
        {
            double x, y;
            if (k == 0)
            {
                x = r;
                y = lambda * t;
            }
            else if (k == q - 1)
            {
                x = lambda * t;
                y = r;
            }
            else
            {
                x = lambda * v[2 * k];
                y = lambda * v[(2 * k) + 1];
            }
            v[2 * k] = x;
            v[(2 * k) + 1] = y;
        }
        for (int k = q; k < m; k++)
        {
            int src = k - q; // rotate the vertex q places earlier by a quarter turn
            v[2 * k] = -v[(2 * src) + 1];
            v[(2 * k) + 1] = v[2 * src];
        }
    }

    // Twice the area of the polygon centre, (r, 0), the quadrant's vertices (reshaped by lambda), (0, r).
    private static double QuadrantTwiceArea(double[] ring, int q, double r, double t, double lambda)
    {
        double px = r, py = 0, twice = 0;
        for (int k = 0; k <= q; k++)
        {
            double x, y;
            if (k == q)
            {
                x = 0;
                y = r;
            }
            else if (k == 0)
            {
                x = r;
                y = lambda * t;
            }
            else if (k == q - 1)
            {
                x = lambda * t;
                y = r;
            }
            else
            {
                x = lambda * ring[2 * k];
                y = lambda * ring[(2 * k) + 1];
            }
            twice += (px * y) - (py * x);
            px = x;
            py = y;
        }
        return twice;
    }

    private List<P[]> SquarePieces(double radius)
    {
        long r = Math.Max(1, (long)Math.Round(radius));
        var pieces = new List<P[]>();
        foreach (P[] ring in _rings)
        {
            for (int i = 0; i < ring.Length; i++)
            {
                P a = ring[i], b = ring[(i + 1) % ring.Length];
                var pts = new List<P>(8)
                {
                    new(a.X - r, a.Y - r), new(a.X + r, a.Y - r), new(a.X + r, a.Y + r), new(a.X - r, a.Y + r),
                    new(b.X - r, b.Y - r), new(b.X + r, b.Y - r), new(b.X + r, b.Y + r), new(b.X - r, b.Y + r),
                };
                pieces.Add(ConvexHull(pts));
            }
        }
        return pieces;
    }

    // Andrew's monotone chain with exact orientation; strictly convex, counter-clockwise.
    internal static P[] ConvexHull(List<P> pts)
    {
        pts.Sort(static (x, y) => x.CompareTo(y));
        var u = new List<P>();
        foreach (P p in pts)
        {
            if (u.Count > 0 && u[^1] == p)
            {
                continue;
            }
            u.Add(p);
        }
        if (u.Count < 3)
        {
            return u.ToArray();
        }
        var h = new List<P>(u.Count * 2);
        foreach (P p in u)
        {
            while (h.Count >= 2 && Geo.Orient(h[^2], h[^1], p) <= 0) { h.RemoveAt(h.Count - 1); }
            h.Add(p);
        }
        int lower = h.Count + 1;
        for (int i = u.Count - 2; i >= 0; i--)
        {
            P p = u[i];
            while (h.Count >= lower && Geo.Orient(h[^2], h[^1], p) <= 0) { h.RemoveAt(h.Count - 1); }
            h.Add(p);
        }
        h.RemoveAt(h.Count - 1);
        return h.ToArray();
    }

    private static P Round(double x, double y) => new((long)Math.Round(x), (long)Math.Round(y));

    // ---- triangulation ----

    /// <summary>
    /// Constrained Delaunay triangulation of the region: every boundary edge is a triangle edge, and the triangles
    /// exactly cover the region. Vertices are the boundary vertices in (x, y) order; triangles are counter-clockwise
    /// and sorted, so the result depends only on the region.
    /// </summary>
    public Triangulation Triangulate()
    {
        var pts = new List<P>();
        foreach (P[] r in _rings)
        {
            pts.AddRange(r);
        }
        pts.Sort(static (x, y) => x.CompareTo(y));
        var verts = new List<P>(pts.Count);
        foreach (P p in pts)
        {
            if (verts.Count == 0 || verts[^1] != p)
            {
                verts.Add(p);
            }
        }
        if (verts.Count < 3)
        {
            return Triangulation.CreateEmpty();
        }
        P[] va = verts.ToArray();
        var cmp = Comparer<P>.Create(static (x, y) => x.CompareTo(y));
        var segs = new List<(int A, int B)>();
        var sign = new Dictionary<long, int>(EdgeKeyComparer.Instance);
        foreach (P[] r in _rings)
        {
            for (int i = 0; i < r.Length; i++)
            {
                int a = Array.BinarySearch(va, r[i], cmp), b = Array.BinarySearch(va, r[(i + 1) % r.Length], cmp);
                int d = 1;
                if (a > b)
                {
                    (a, b) = (b, a);
                    d = -1;
                }
                long key = ((long)a << 32) | (uint)b;
                sign[key] = sign.TryGetValue(key, out int old) ? old + d : d;
            }
        }
        var keys = new List<long>(sign.Keys);
        keys.Sort();
        foreach (long k in keys)
        {
            if (sign[k] != 0)
            {
                segs.Add(((int)(k >> 32), (int)(k & 0xFFFFFFFFL)));
            }
        }
        var cdt = new Cdt(va, segs);
        (int, int)[] w = cdt.Winding((u, v) =>
            u < v ? (sign[((long)u << 32) | (uint)v], 0) : (-sign[((long)v << 32) | (uint)u], 0));
        return Triangulation.FromCdt(cdt, w, va, SnapBits);
    }

    /// <summary>The bounding box of all ring vertices in grid units; false for an empty region.</summary>
    internal bool TryGetGridBounds(out long minX, out long minY, out long maxX, out long maxY)
    {
        minX = minY = long.MaxValue;
        maxX = maxY = long.MinValue;
        bool any = false;
        foreach (P[] r in _rings)
        {
            foreach (P q in r)
            {
                any = true;
                minX = Math.Min(minX, q.X);
                maxX = Math.Max(maxX, q.X);
                minY = Math.Min(minY, q.Y);
                maxY = Math.Max(maxY, q.Y);
            }
        }
        return any;
    }

    /// <summary>
    /// Triangulates the region with extra constraint lines: the vertical lines x = <paramref name="xLines"/> and the
    /// horizontal lines y = <paramref name="yLines"/> (grid units) are cut through the region, so no triangle crosses
    /// one. The lines are noded with the boundary by the same snap rounding as every other construction: where a line
    /// crosses the boundary off the grid, the crossing is rounded once and the boundary passes through the rounded point.
    /// </summary>
    internal Triangulation TriangulateWithCuts(IReadOnlyList<long> xLines, IReadOnlyList<long> yLines)
    {
        if (!TryGetGridBounds(out long x0, out long y0, out long x1, out long y1))
        {
            return Triangulation.CreateEmpty();
        }
        var edges = new List<InEdge>();
        AppendEdges(edges, 0);
        foreach (long x in xLines)
        {
            if (x > x0 && x < x1)
            {
                edges.Add(new InEdge(new P(x, y0), new P(x, y1), 1));
            }
        }
        foreach (long y in yLines)
        {
            if (y > y0 && y < y1)
            {
                edges.Add(new InEdge(new P(x0, y), new P(x1, y), 1));
            }
        }
        Arrangement arr = Arrangement.Build(edges);
        var constraints = new List<(int, int)>(arr.SegA.Length);
        var index = new Dictionary<long, int>(arr.SegA.Length, EdgeKeyComparer.Instance);
        for (int i = 0; i < arr.SegA.Length; i++)
        {
            constraints.Add((arr.SegA[i], arr.SegB[i]));
            index[((long)arr.SegA[i] << 32) | (uint)arr.SegB[i]] = i;
        }
        var cdt = new Cdt(arr.Verts, constraints);
        (int, int)[] w = cdt.Winding((u, v) =>
        {
            // Only the region boundary (operand 0) changes the winding; the cut lines are plain constraints.
            if (u < v)
            {
                return (arr.D0[index[((long)u << 32) | (uint)v]], 0);
            }
            return (-arr.D0[index[((long)v << 32) | (uint)u]], 0);
        });
        return Triangulation.FromCdt(cdt, w, arr.Verts, SnapBits);
    }

    /// <summary>Triangulates the region and merges the triangles into convex polygons; see <see cref="ConvexPartition"/>.</summary>
    /// <param name="maxVerticesPerPolygon">Upper bound on polygon size, 3 to 16.</param>
    public ConvexPartition PartitionConvex(int maxVerticesPerPolygon = 6) =>
        ConvexPartition.Create(Triangulate(), maxVerticesPerPolygon);

    // ---- helpers ----

    internal static double Pow2(int e)
    {
        double r = 1.0;
        for (int i = 0; i < Math.Abs(e); i++)
        {
            r = e > 0 ? r * 2.0 : r * 0.5;
        }
        return r;
    }

    private static double UnitSquared(int bits) => Pow2(-2 * bits);

    internal static P Snap(Vector2D v, int bits, string what)
    {
        CheckCoordinate(v.X, what, "x");
        CheckCoordinate(v.Y, what, "y");
        double scale = Pow2(bits);
        return new P((long)Math.Round(v.X * scale), (long)Math.Round(v.Y * scale));
    }

    private static void CheckCoordinate(double v, string what, string axis)
    {
        if (!double.IsFinite(v))
        {
            throw new ArgumentOutOfRangeException(what, v, "The " + axis + " coordinate of " + what + " is not finite.");
        }
        if (Math.Abs(v) > CoordinateBound)
        {
            throw new ArgumentOutOfRangeException(what, v, "The " + axis + " coordinate of " + what + " is " + v.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", beyond the supported range of ±2^21.");
        }
    }

    internal static Vector2D[] ToPublic(P[] ring, int bits)
    {
        double inv = Pow2(-bits);
        var r = new Vector2D[ring.Length];
        for (int i = 0; i < r.Length; i++)
        {
            r[i] = new Vector2D(ring[i].X * inv, ring[i].Y * inv);
        }
        return r;
    }
}
