using System;
using System.Collections.Generic;
using System.Globalization;
using System.Threading;
using RecursiveAI.OnePath.Graphs;

namespace RecursiveAI.OnePath;

/// <summary>
/// How a <see cref="PointGraph"/> links its points. Links from every source are merged into one set of undirected pairs,
/// de-duplicated, and every pair becomes two edges weighted with the straight-line distance. Without any link source the
/// graph has no edges.
/// </summary>
public sealed record PointGraphOptions
{
    /// <summary>Explicit links as pairs of point indices (always kept, never passed to <see cref="LinkValid"/>). Null or empty means none.</summary>
    public IReadOnlyList<(int A, int B)>? Links { get; init => field = value is null ? null : FrozenList<(int A, int B)>.Of(value); }

    /// <summary>
    /// Link every pair of points at most this far apart (0, the default, means no distance rule). Combined with
    /// <see cref="NearestCount"/> it limits each point's nearest neighbours to those within this distance.
    /// Finite; 0 or at least 2^-30.
    /// </summary>
    public double MaxDistance { get; init; }

    /// <summary>
    /// Link each point to its nearest <c>k</c> other points (0, the default, means no such rule). Distance ties break
    /// towards the smaller point index; the resulting links are made mutual. With <see cref="LinkValid"/> set, the
    /// <c>k</c> nearest points that pass the test.
    /// </summary>
    public int NearestCount { get; init; }

    /// <summary>
    /// Optional validation of rule-generated links, typically a line-of-sight raycast: called with the two positions
    /// (the lower point index first) and the link is kept when it returns true. Called at most once per candidate pair per
    /// requesting point, in a fixed order, so a deterministic delegate gives a deterministic graph.
    /// </summary>
    public Func<Point3, Point3, bool>? LinkValid { get; init; }
}

/// <summary>
/// A graph of positioned nodes. Edge weights are straight-line distances, so the straight-line distance is a consistent
/// A* estimate and <see cref="GraphAlgorithm.Auto"/> uses A*. Immutable once built; safe to query from several threads
/// (each query rents its own working memory). Positions obey the coordinate rule: finite and within ±2^21.
/// <para>
/// <b>Link building</b> uses a uniform spatial hash with lookups only (no enumeration order matters). The set of links
/// depends only on the positions and options: it is the same under any permutation of the input points, except that
/// <see cref="PointGraphOptions.NearestCount"/> may pick differently between points at exactly equal distance (it prefers the smaller index).
/// Edge order is by (smaller index, larger index) of each pair.
/// </para>
/// </summary>
public sealed class PointGraph
{
    /// <summary>The largest coordinate magnitude accepted (2^21), so positions convert exactly to and from a 64-bit fixed-point host (32 fractional bits).</summary>
    public const double CoordinateLimit = 2097152.0;

    private readonly Point3[] _positions;
    private readonly CsrGraph _graph;
    private SearchPair? _spare;

    private PointGraph(Point3[] positions, CsrGraph graph)
    {
        _positions = positions;
        _graph = graph;
    }

    /// <summary>The point positions, indexed by node id.</summary>
    public ReadOnlySpan<Point3> Positions => _positions;

    /// <summary>The compiled graph (tier 3 access for searches and landmarks).</summary>
    public CsrGraph Graph => _graph;

    /// <summary>Number of points.</summary>
    public int NodeCount => _positions.Length;

    /// <summary>Builds a point graph in the plane (<c>Z = 0</c>).</summary>
    /// <param name="points">The points; node id = position in the sequence.</param>
    /// <param name="options">Linking options; null means no links.</param>
    /// <returns>The graph.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="points"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN, infinite or beyond ±2^21 (the message names the value and the bound), or an option is out of range.</exception>
    /// <exception cref="ArgumentException">An explicit link names a point that does not exist.</exception>
    public static PointGraph From(IEnumerable<WorldPoint> points, PointGraphOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(points);
        var list = new List<Point3>();
        foreach (WorldPoint p in points)
        {
            list.Add(new Point3(p.X, p.Y, 0.0));
        }
        return From(list, options);
    }

    /// <summary>Builds a point graph.</summary>
    /// <param name="points">The points; node id = position in the sequence.</param>
    /// <param name="options">Linking options; null means no links.</param>
    /// <returns>The graph.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="points"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN, infinite or beyond ±2^21 (the message names the value and the bound), or an option is out of range.</exception>
    /// <exception cref="ArgumentException">An explicit link names a point that does not exist.</exception>
    public static PointGraph From(IEnumerable<Point3> points, PointGraphOptions? options = null)
    {
        ArgumentNullException.ThrowIfNull(points);
        options ??= new PointGraphOptions();
        if (!(options.MaxDistance >= 0.0) || double.IsPositiveInfinity(options.MaxDistance))
        {
            throw new ArgumentOutOfRangeException(nameof(options), options.MaxDistance, "MaxDistance must be finite and not negative.");
        }
        if (options.MaxDistance > 0.0 && options.MaxDistance < PointLinks.MinCell)
        {
            throw new ArgumentOutOfRangeException(nameof(options), options.MaxDistance, "MaxDistance must be 0 (no rule) or at least 2^-30.");
        }
        if (options.NearestCount < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(options), options.NearestCount, "NearestCount must not be negative.");
        }
        var pts = new List<Point3>();
        foreach (Point3 p in points)
        {
            CheckCoordinate(p.X, "X", pts.Count, nameof(points));
            CheckCoordinate(p.Y, "Y", pts.Count, nameof(points));
            CheckCoordinate(p.Z, "Z", pts.Count, nameof(points));
            pts.Add(p);
        }
        if (pts.Count > CsrGraph.MaxNodes)
        {
            throw new ArgumentException($"A point graph holds at most {CsrGraph.MaxNodes} points.", nameof(points));
        }
        Point3[] positions = pts.ToArray();
        var pairs = new List<(int, int)>();
        if (options.Links is not null)
        {
            foreach ((int a, int b) in options.Links)
            {
                if ((uint)a >= (uint)positions.Length || (uint)b >= (uint)positions.Length)
                {
                    throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Link ({a}, {b}) names a point outside 0..{positions.Length - 1}."), nameof(options));
                }
                if (a != b)
                {
                    pairs.Add((Math.Min(a, b), Math.Max(a, b)));
                }
            }
        }
        var generated = new List<(int, int)>();
        if (positions.Length > 1)
        {
            if (options.NearestCount > 0)
            {
                PointLinks.Nearest(positions, options.NearestCount, options.MaxDistance, options.LinkValid, generated);
            }
            else if (options.MaxDistance > 0.0)
            {
                PointLinks.Within(positions, options.MaxDistance, options.LinkValid, generated);
            }
        }
        pairs.AddRange(generated);
        pairs.Sort(static (x, y) => x.Item1 != y.Item1 ? x.Item1.CompareTo(y.Item1) : x.Item2.CompareTo(y.Item2));
        var edges = new List<GraphEdge>(pairs.Count * 2);
        (int, int) last = (-1, -1);
        foreach ((int a, int b) in pairs)
        {
            if ((a, b) == last)
            {
                continue;
            }
            last = (a, b);
            double w = positions[a].DistanceTo(positions[b]);
            edges.Add(new GraphEdge(a, b, w));
            edges.Add(new GraphEdge(b, a, w));
        }
        return new PointGraph(positions, CsrGraph.Create(positions.Length, edges.ToArray()));
    }

    /// <summary>The node nearest to <paramref name="position"/> by straight-line distance (a linear scan; ties go to the smaller id).</summary>
    /// <param name="position">A position within ±2^21.</param>
    /// <returns>The node id, or −1 for an empty graph.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN, infinite or beyond ±2^21.</exception>
    public int NearestNode(Point3 position)
    {
        CheckCoordinate(position.X, "X", -1, nameof(position));
        CheckCoordinate(position.Y, "Y", -1, nameof(position));
        CheckCoordinate(position.Z, "Z", -1, nameof(position));
        int best = -1;
        double bestD = double.PositiveInfinity;
        for (int i = 0; i < _positions.Length; i++)
        {
            double d = _positions[i].DistanceTo(position);
            if (d < bestD)
            {
                bestD = d;
                best = i;
            }
        }
        return best;
    }

    /// <summary>Builds landmark tables for this graph (see <see cref="Landmarks.Build"/>).</summary>
    /// <param name="options">Count, selection and seed; null means <see cref="LandmarkOptions"/> defaults (16 landmarks, avoid, seed 1).</param>
    /// <returns>The table.</returns>
    /// <exception cref="ArgumentException">The graph has no nodes.</exception>
    public Landmarks BuildLandmarks(LandmarkOptions? options = null) => Landmarks.Build(_graph, options?.Count ?? 16, options?.Selection ?? LandmarkSelection.Avoid, options?.Seed ?? 1);

    /// <summary>Finds the cheapest path between two nodes (A* with the straight-line estimate by default).</summary>
    /// <param name="start">Start node id.</param>
    /// <param name="goal">Goal node id.</param>
    /// <param name="options">Options; null means defaults.</param>
    /// <param name="cancellationToken">Checked every 256 iterations.</param>
    /// <returns>The result, with <see cref="GraphPath.Points"/> filled.</returns>
    /// <exception cref="InvalidOperationException">The landmarks in <paramref name="options"/> were built for a different graph.</exception>
    /// <exception cref="OperationCanceledException">Cancellation was requested.</exception>
    public GraphPath FindPath(int start, int goal, GraphOptions? options = null, CancellationToken cancellationToken = default)
    {
        SearchPair pair = Interlocked.Exchange(ref _spare, null) ?? new SearchPair(_graph.NodeCount);
        try
        {
            return GraphRunner.Run(_graph, _positions, pair, start, goal, options ?? new GraphOptions(), cancellationToken);
        }
        finally
        {
            _spare = pair;
        }
    }

    /// <summary>Finds the cheapest path between the nodes nearest to two positions (see <see cref="NearestNode"/>).</summary>
    /// <param name="start">Start position.</param>
    /// <param name="goal">Goal position.</param>
    /// <param name="options">Options; null means defaults.</param>
    /// <param name="cancellationToken">Checked every 256 iterations.</param>
    /// <returns>The result; <see cref="PathStatus.StartInvalid"/> or <see cref="PathStatus.GoalInvalid"/> for an empty graph.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN, infinite or beyond ±2^21.</exception>
    public GraphPath FindPath(Point3 start, Point3 goal, GraphOptions? options = null, CancellationToken cancellationToken = default)
        => FindPath(NearestNode(start), NearestNode(goal), options, cancellationToken);

    private static void CheckCoordinate(double value, string axis, int index, string paramName)
    {
        if (!(Math.Abs(value) <= CoordinateLimit))
        {
            string where = index >= 0 ? $"Point {index}" : "The position";
            throw new ArgumentOutOfRangeException(
                paramName,
                value,
                string.Create(CultureInfo.InvariantCulture, $"{where} has {axis} = {value:R}, outside the supported range of ±2^21 (±{CoordinateLimit:R})."));
        }
    }
}

// Spatial hashing for point links. Cells are looked up by exact integer coordinates, never enumerated, so nothing
// depends on dictionary order; candidate lists inside a cell are in point-index order.
internal static class PointLinks
{
    internal const double MinCell = 1.0 / 1073741824.0;

    private readonly record struct CellKey(long X, long Y, long Z);

    private sealed class Index
    {
        internal readonly Dictionary<CellKey, List<int>> Cells = [];
        internal readonly double CellSize;
        internal readonly Point3[] Points;

        internal Index(Point3[] points, double cellSize)
        {
            Points = points;
            CellSize = cellSize;
            for (int i = 0; i < points.Length; i++)
            {
                CellKey k = KeyOf(points[i]);
                if (!Cells.TryGetValue(k, out List<int>? list))
                {
                    list = [];
                    Cells[k] = list;
                }
                list.Add(i);
            }
        }

        internal CellKey KeyOf(Point3 p) => new((long)Math.Floor(p.X / CellSize), (long)Math.Floor(p.Y / CellSize), (long)Math.Floor(p.Z / CellSize));
    }

    // All pairs within maxDistance (inclusive), by comparing squared distances.
    internal static void Within(Point3[] pts, double maxDistance, Func<Point3, Point3, bool>? valid, List<(int, int)> output)
    {
        var index = new Index(pts, maxDistance);
        double limit = maxDistance * maxDistance;
        for (int i = 0; i < pts.Length; i++)
        {
            CellKey c = index.KeyOf(pts[i]);
            for (long dx = -1; dx <= 1; dx++)
            {
                for (long dy = -1; dy <= 1; dy++)
                {
                    for (long dz = -1; dz <= 1; dz++)
                    {
                        if (!index.Cells.TryGetValue(new CellKey(c.X + dx, c.Y + dy, c.Z + dz), out List<int>? list))
                        {
                            continue;
                        }
                        foreach (int j in list)
                        {
                            if (j <= i)
                            {
                                continue;
                            }
                            double ex = pts[i].X - pts[j].X;
                            double ey = pts[i].Y - pts[j].Y;
                            double ez = pts[i].Z - pts[j].Z;
                            if ((((ex * ex) + (ey * ey)) + (ez * ez)) <= limit && (valid is null || valid(pts[i], pts[j])))
                            {
                                output.Add((i, j));
                            }
                        }
                    }
                }
            }
        }
    }

    // Each point's k nearest valid neighbours by rings of cells around its own cell. After scanning rings 0..r every point
    // closer than r * cell is known (a point outside the block is at least r cells away), so candidates strictly closer
    // than that bound are final and are examined in (distance, index) order.
    internal static void Nearest(Point3[] pts, int k, double maxDistance, Func<Point3, Point3, bool>? valid, List<(int, int)> output)
    {
        double minX = double.PositiveInfinity, minY = minX, minZ = minX, maxX = double.NegativeInfinity, maxY = maxX, maxZ = maxX;
        foreach (Point3 p in pts)
        {
            minX = Math.Min(minX, p.X);
            maxX = Math.Max(maxX, p.X);
            minY = Math.Min(minY, p.Y);
            maxY = Math.Max(maxY, p.Y);
            minZ = Math.Min(minZ, p.Z);
            maxZ = Math.Max(maxZ, p.Z);
        }
        double extent = Math.Max(maxX - minX, Math.Max(maxY - minY, maxZ - minZ));
        // Cells never get smaller than 2^-30, so cell coordinates stay far inside a 64-bit integer.
        double cell = extent > 0.0 ? Math.Max(MinCell, extent / Math.Max(1, (int)Math.Sqrt(pts.Length))) : 1.0;
        var index = new Index(pts, cell);
        bool flatZ = maxZ == minZ;
        bool flatY = maxY == minY;
        long maxRing = (long)Math.Ceiling(extent / cell) + 1;
        var pool = new List<(double D2, int J)>();
        for (int i = 0; i < pts.Length; i++)
        {
            pool.Clear();
            CellKey c = index.KeyOf(pts[i]);
            int done = 0;
            int accepted = 0;
            for (long r = 0; r <= maxRing + 1 && accepted < k; r++)
            {
                for (long dx = -r; dx <= r; dx++)
                {
                    for (long dy = flatY ? 0 : -r; dy <= (flatY ? 0 : r); dy++)
                    {
                        for (long dz = flatZ ? 0 : -r; dz <= (flatZ ? 0 : r); dz++)
                        {
                            if (Math.Max(Math.Abs(dx), Math.Max(Math.Abs(dy), Math.Abs(dz))) != r)
                            {
                                continue;
                            }
                            if (!index.Cells.TryGetValue(new CellKey(c.X + dx, c.Y + dy, c.Z + dz), out List<int>? list))
                            {
                                continue;
                            }
                            foreach (int j in list)
                            {
                                if (j == i)
                                {
                                    continue;
                                }
                                double ex = pts[i].X - pts[j].X;
                                double ey = pts[i].Y - pts[j].Y;
                                double ez = pts[i].Z - pts[j].Z;
                                pool.Add(((((ex * ex) + (ey * ey)) + (ez * ez)), j));
                            }
                        }
                    }
                }
                pool.Sort(static (a, b) => a.D2 != b.D2 ? a.D2.CompareTo(b.D2) : a.J.CompareTo(b.J));
                double safe = r * cell;
                bool everything = r >= maxRing + 1 || (maxDistance > 0.0 && safe > maxDistance);
                double safe2 = safe * safe;
                while (done < pool.Count && accepted < k && (everything || pool[done].D2 < safe2))
                {
                    (double d2, int j) = pool[done++];
                    if (maxDistance > 0.0 && d2 > maxDistance * maxDistance)
                    {
                        // Sorted by distance: everything after this is farther still.
                        done = pool.Count;
                        accepted = k;
                        break;
                    }
                    int lo = Math.Min(i, j);
                    int hi = Math.Max(i, j);
                    if (valid is null || valid(pts[lo], pts[hi]))
                    {
                        output.Add((lo, hi));
                        accepted++;
                    }
                }
            }
        }
    }
}
