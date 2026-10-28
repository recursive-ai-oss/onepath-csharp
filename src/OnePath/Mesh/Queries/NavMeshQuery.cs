using System;
using RecursiveAI.OneMath;

namespace RecursiveAI.OnePath.Mesh.Queries;

/// <summary>
/// The query object for a <see cref="TiledNavMesh"/>: nearest polygon, A* over polygons (whole, sliced or async),
/// string pulling and raycasts along the surface. One object holds pooled work buffers, so steady-state queries
/// allocate nothing; use one object per thread (it is not thread-safe) and keep it for the life of the mesh.
/// The mesh may be edited between queries but not during one; polygon references into removed or replaced tiles are
/// rejected rather than misread. Normative behaviour is in <c>docs/navmesh-queries.md</c>.
/// </summary>
public sealed partial class NavMeshQuery
{
    private readonly TiledNavMesh _mesh;
    private readonly int _maxNodes;
    private int[] _bvhHits = new int[256];
    private readonly int[] _bvhStack = new int[64];
    private TileRef[] _tileBuf = new TileRef[16];

    /// <summary>Creates a query object.</summary>
    /// <param name="mesh">The mesh to query.</param>
    /// <param name="maxNodes">Most search nodes one path search may create; a search that needs more ends as Partial. Default 65536; 2 to 8 000 000.</param>
    /// <exception cref="ArgumentNullException"><paramref name="mesh"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="maxNodes"/> is out of range.</exception>
    /// <param name="nodeModel">What one search node stands for; see <see cref="PathNodeModel"/>. Default <see cref="PathNodeModel.Crossing"/>.</param>
    public NavMeshQuery(TiledNavMesh mesh, int maxNodes = 65536, PathNodeModel nodeModel = PathNodeModel.Crossing)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        if (maxNodes < 2 || maxNodes > 8_000_000)
        {
            throw new ArgumentOutOfRangeException(nameof(maxNodes), maxNodes, "maxNodes must be 2 to 8000000.");
        }
        if (nodeModel is not (PathNodeModel.Crossing or PathNodeModel.Polygon))
        {
            throw new ArgumentOutOfRangeException(nameof(nodeModel), nodeModel, "Unknown path node model.");
        }
        _mesh = mesh;
        _polygonNodes = nodeModel == PathNodeModel.Polygon;
        _maxNodes = maxNodes;
        InitPathBuffers();
    }

    /// <summary>What one path-search node stands for in this query object.</summary>
    public PathNodeModel NodeModel => _polygonNodes ? PathNodeModel.Polygon : PathNodeModel.Crossing;

    /// <summary>The mesh being queried.</summary>
    public TiledNavMesh Mesh => _mesh;

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    internal static void CheckPos(Vector3D v, string name)
    {
        if (!(Math.Abs(v.X) < NavMeshParams.MaxWorld && Math.Abs(v.Y) < NavMeshParams.MaxWorld && Math.Abs(v.Z) < NavMeshParams.MaxWorld))
        {
            throw new ArgumentOutOfRangeException(name, v, $"Position {v} must be finite and within ±{NavMeshParams.MaxWorld:R} on every axis.");
        }
    }

    /// <summary>
    /// The nearest polygon to a position among those overlapping the box <c>center ± halfExtents</c> that the filter
    /// accepts, over all tiles and layers. Distance is the 3D distance to the closest point of the polygon surface
    /// (inside the outline seen from above that is the point straight above or below, so on a multi-layer mesh the layer
    /// whose floor is vertically closest wins; outside, the nearest outline point in X/Z at its edge height). Ties go to
    /// the lowest (tile column, tile row, layer, polygon index).
    /// </summary>
    /// <returns>The result; <see cref="NearestPoly.Poly"/> is <see cref="PolyRef.Invalid"/> if nothing qualifies.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A position is non-finite or beyond ±2^21, or an extent is negative or non-finite.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public NearestPoly FindNearestPoly<TFilter>(Vector3D center, Vector3D halfExtents, in TFilter filter)
        where TFilter : struct, IQueryFilter
    {
        CheckPos(center, nameof(center));
        if (!(halfExtents.X >= 0 && halfExtents.Y >= 0 && halfExtents.Z >= 0 && double.IsFinite(halfExtents.X) && double.IsFinite(halfExtents.Y) && double.IsFinite(halfExtents.Z)))
        {
            throw new ArgumentOutOfRangeException(nameof(halfExtents), halfExtents, "Half extents must be finite and not negative.");
        }
        NavMeshParams p = _mesh.Params;
        double minX = center.X - halfExtents.X, maxX = center.X + halfExtents.X;
        double minZ = center.Z - halfExtents.Z, maxZ = center.Z + halfExtents.Z;
        double fx0 = Math.Floor((minX - p.OriginX) / p.TileSize), fx1 = Math.Floor((maxX - p.OriginX) / p.TileSize);
        double fz0 = Math.Floor((minZ - p.OriginZ) / p.TileSize), fz1 = Math.Floor((maxZ - p.OriginZ) / p.TileSize);
        var best = new Candidate(PolyRef.Invalid, default, false, double.PositiveInfinity, 0, 0, 0, 0);
        double columns = (fx1 - fx0 + 1) * (fz1 - fz0 + 1);
        if (columns <= _mesh.SlotCount)
        {
            for (long tx = (long)fx0; tx <= (long)fx1; tx++)
            {
                for (long tz = (long)fz0; tz <= (long)fz1; tz++)
                {
                    int cnt = _mesh.GetTilesAt((int)tx, (int)tz, _tileBuf);
                    if (cnt > _tileBuf.Length)
                    {
                        _tileBuf = new TileRef[cnt];
                        _mesh.GetTilesAt((int)tx, (int)tz, _tileBuf);
                    }
                    for (int i = 0; i < cnt; i++)
                    {
                        ScanTile(_tileBuf[i], center, halfExtents, in filter, ref best);
                    }
                }
            }
        }
        else
        {
            // A huge box over a small mesh: look at every tile instead of every grid cell. Ties are resolved by explicit key, so order is irrelevant.
            for (int s = 0; s < _mesh.SlotCount; s++)
            {
                NavTile? t = _mesh.GetTileBySlot(s);
                if (t is not null && t.TileX >= fx0 && t.TileX <= fx1 && t.TileZ >= fz0 && t.TileZ <= fz1)
                {
                    ScanTile(_mesh.GetSlotRef(s), center, halfExtents, in filter, ref best);
                }
            }
        }
        return new NearestPoly(best.Poly, best.Point, best.Over, best.D2);
    }

    /// <summary>Nearest polygon with the plain filter (null accepts every polygon).</summary>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public NearestPoly FindNearestPoly(Vector3D center, Vector3D halfExtents, QueryFilter? filter = null)
    {
        var f = new PlainFilter(filter ?? s_defaultFilter);
        return FindNearestPoly(center, halfExtents, in f);
    }

    private static readonly QueryFilter s_defaultFilter = new();

    private readonly struct Candidate
    {
        public Candidate(PolyRef poly, Vector3D point, bool over, double d2, int tx, int tz, int layer, int index)
        {
            Poly = poly;
            Point = point;
            Over = over;
            D2 = d2;
            Tx = tx;
            Tz = tz;
            Layer = layer;
            Index = index;
        }

        public PolyRef Poly { get; }

        public Vector3D Point { get; }

        public bool Over { get; }

        public double D2 { get; }

        public int Tx { get; }

        public int Tz { get; }

        public int Layer { get; }

        public int Index { get; }

        [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
        public bool BeatenBy(double d2, int tx, int tz, int layer, int index)
        {
            if (d2 != D2)
            {
                return d2 < D2;
            }
            if (tx != Tx)
            {
                return tx < Tx;
            }
            if (tz != Tz)
            {
                return tz < Tz;
            }
            return layer != Layer ? layer < Layer : index < Index;
        }
    }

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    private void ScanTile<TFilter>(TileRef tr, Vector3D center, Vector3D ext, in TFilter filter, ref Candidate best)
        where TFilter : struct, IQueryFilter
    {
        NavTile? t = _mesh.GetTile(tr);
        if (t is null || t.PolyCount == 0)
        {
            return;
        }
        TileBvh bvh = t.GetBvh();
        int cnt = 0;
        double lx = center.X - t.OriginX, ly = center.Y - t.OriginY, lz = center.Z - t.OriginZ;
        while (true)
        {
            cnt = 0;
            bvh.Query(lx - ext.X, ly - ext.Y, lz - ext.Z, lx + ext.X, ly + ext.Y, lz + ext.Z, _bvhHits, ref cnt, _bvhStack);
            if (cnt <= _bvhHits.Length)
            {
                break;
            }
            _bvhHits = new int[cnt];
        }
        for (int i = 0; i < cnt; i++)
        {
            int pi = _bvhHits[i];
            if (!filter.Passes(t.PolyFlags[pi], t.PolyAreas[pi]))
            {
                continue;
            }
            PolyGeometry.Closest(t, pi, lx, ly, lz, out double cx, out double cy, out double cz, out bool over);
            double dx = cx - lx, dy = cy - ly, dz = cz - lz;
            double d2 = (dx * dx) + (dy * dy) + (dz * dz);
            if (best.Poly.IsSet && !best.BeatenBy(d2, t.TileX, t.TileZ, t.Layer, pi))
            {
                continue;
            }
            best = new Candidate(tr.Poly(pi), new Vector3D(t.OriginX + cx, t.OriginY + cy, t.OriginZ + cz), over, d2, t.TileX, t.TileZ, t.Layer, pi);
        }
    }

    /// <summary>
    /// The closest point of a polygon to a position (see <see cref="FindNearestPoly{TFilter}"/> for the rule).
    /// </summary>
    /// <returns>False if the polygon reference is invalid or stale.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The position is non-finite or beyond ±2^21.</exception>
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveOptimization)]
    public bool ClosestPointOnPoly(PolyRef poly, Vector3D position, out Vector3D closest, out bool overPolygon)
    {
        CheckPos(position, nameof(position));
        closest = default;
        overPolygon = false;
        if (!_mesh.TryGetPoly(poly, out NavTile t, out int pi))
        {
            return false;
        }
        PolyGeometry.Closest(t, pi, position.X - t.OriginX, position.Y - t.OriginY, position.Z - t.OriginZ, out double cx, out double cy, out double cz, out overPolygon);
        closest = new Vector3D(t.OriginX + cx, t.OriginY + cy, t.OriginZ + cz);
        return true;
    }
}
