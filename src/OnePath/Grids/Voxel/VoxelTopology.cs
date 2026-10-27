using System;

namespace RecursiveAI.OnePath.Grids.Voxel;

// The 26 neighbour offsets in the normative order: faces (one non-zero component), then edges (two), then corners
// (three); inside each group z, then y, then x ascending from -1 to 1. A search with fewer moves uses a prefix.
internal static class VoxelOffsets
{
    internal static readonly sbyte[] Dx = new sbyte[26];
    internal static readonly sbyte[] Dy = new sbyte[26];
    internal static readonly sbyte[] Dz = new sbyte[26];
    internal static readonly byte[] Kind = new byte[26];

    static VoxelOffsets()
    {
        int n = 0;
        for (int kind = 1; kind <= 3; kind++)
        {
            for (int z = -1; z <= 1; z++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int x = -1; x <= 1; x++)
                    {
                        int nz = (x != 0 ? 1 : 0) + (y != 0 ? 1 : 0) + (z != 0 ? 1 : 0);
                        if (nz != kind)
                        {
                            continue;
                        }
                        Dx[n] = (sbyte)x;
                        Dy[n] = (sbyte)y;
                        Dz[n] = (sbyte)z;
                        Kind[n] = (byte)kind;
                        n++;
                    }
                }
            }
        }
    }
}

/// <summary>
/// A view of a <see cref="VoxelGrid"/> for one movement rule: the topology A* searches. Create it with
/// <see cref="VoxelGrid.CreateTopology"/>. It reads the grid's chunk table, so edits show through, but do not edit the
/// grid while a search runs.
/// <para>
/// Neighbour order (normative): the 6 face moves, then the 12 edge moves, then the 8 corner moves, each group by
/// z, then y, then x ascending (-1, 0, 1); a view with 6 or 18 moves uses the first 6 or 18. Open voxels only: a voxel is open
/// when its area is not <see cref="AreaCosts.Wall"/>. The cost of a move is <c>length * ((m[from] + m[to]) * 0.5)</c>,
/// with length 1 for a face move, <see cref="GridMath.Sqrt2"/> for an edge move and <see cref="VoxelMath.Sqrt3"/> for a
/// corner move and <c>m</c> the area multiplier of the voxel's area. Edge and corner moves obey the
/// <see cref="VoxelCornerRule"/>.
/// </para>
/// </summary>
public readonly struct VoxelTopology : IGridTopology
{
    private readonly byte[]?[] _chunks;
    private readonly double[] _multipliers;
    private readonly int _sizeX;
    private readonly int _sizeY;
    private readonly int _sizeZ;
    private readonly int _chunksX;
    private readonly int _chunksY;
    private readonly byte _fill;
    private readonly int _moves;
    private readonly VoxelCornerRule _rule;

    internal VoxelTopology(VoxelGrid grid, VoxelConnectivity connectivity, VoxelCornerRule rule)
    {
        _chunks = grid.ChunkTable;
        _multipliers = grid.Areas.Multipliers;
        _sizeX = grid.SizeX;
        _sizeY = grid.SizeY;
        _sizeZ = grid.SizeZ;
        _chunksX = grid.ChunksX;
        _chunksY = grid.ChunksY;
        _fill = grid.DefaultArea;
        _moves = (int)connectivity;
        _rule = rule;
    }

    /// <summary>Voxels along x.</summary>
    public int SizeX => _sizeX;

    /// <summary>Voxels along y.</summary>
    public int SizeY => _sizeY;

    /// <summary>Voxels along z.</summary>
    public int SizeZ => _sizeZ;

    /// <summary>The number of moves per voxel: 6, 18 or 26.</summary>
    public int Moves => _moves;

    /// <inheritdoc/>
    public int NodeCount => _sizeX * _sizeY * _sizeZ;

    /// <inheritdoc/>
    public bool IsWalkable(int node)
    {
        if ((uint)node >= (uint)NodeCount)
        {
            return false;
        }
        VoxelCell c = VoxelGrid.Decode(node, _sizeX, _sizeY);
        return Area(c.X, c.Y, c.Z) != AreaCosts.Wall;
    }

    /// <inheritdoc/>
    public void VisitNeighbors<TVisitor>(int node, ref TVisitor visitor)
        where TVisitor : struct, INeighborVisitor
    {
        VoxelCell c = VoxelGrid.Decode(node, _sizeX, _sizeY);
        int x = c.X;
        int y = c.Y;
        int z = c.Z;
        double mFrom = _multipliers[Area(x, y, z)];
        sbyte[] dxs = VoxelOffsets.Dx;
        sbyte[] dys = VoxelOffsets.Dy;
        sbyte[] dzs = VoxelOffsets.Dz;
        byte[] kinds = VoxelOffsets.Kind;
        for (int i = 0; i < _moves; i++)
        {
            int dx = dxs[i];
            int dy = dys[i];
            int dz = dzs[i];
            int nx = x + dx;
            int ny = y + dy;
            int nz = z + dz;
            if ((uint)nx >= (uint)_sizeX || (uint)ny >= (uint)_sizeY || (uint)nz >= (uint)_sizeZ)
            {
                continue;
            }
            byte area = Area(nx, ny, nz);
            if (area == AreaCosts.Wall)
            {
                continue;
            }
            int kind = kinds[i];
            double length = 1.0;
            if (kind > 1)
            {
                if (_rule != VoxelCornerRule.AllowAll && !SweptOk(x, y, z, dx, dy, dz, kind))
                {
                    continue;
                }
                length = kind == 2 ? GridMath.Sqrt2 : VoxelMath.Sqrt3;
            }
            visitor.Visit(nx + (_sizeX * (ny + (_sizeY * nz))), length * ((mFrom + _multipliers[area]) * 0.5));
        }
    }

    // Both endpoints are inside the grid, so every swept cell (it lies between them on each axis) is too.
    private byte Area(int x, int y, int z) => VoxelGrid.Read(_chunks, _chunksX, _chunksY, _fill, x, y, z);

    private bool Open(int x, int y, int z) => Area(x, y, z) != AreaCosts.Wall;

    private bool SweptOk(int x, int y, int z, int dx, int dy, int dz, int kind)
    {
        if (kind == 2)
        {
            // The two face neighbours the edge move passes between.
            bool a;
            bool b;
            if (dx == 0)
            {
                a = Open(x, y + dy, z);
                b = Open(x, y, z + dz);
            }
            else if (dy == 0)
            {
                a = Open(x + dx, y, z);
                b = Open(x, y, z + dz);
            }
            else
            {
                a = Open(x + dx, y, z);
                b = Open(x, y + dy, z);
            }
            return _rule == VoxelCornerRule.NoCornerCutting ? a && b : a || b;
        }

        // Corner move: three face cells and three edge cells inside the move's cube.
        bool fx = Open(x + dx, y, z);
        bool fy = Open(x, y + dy, z);
        bool fz = Open(x, y, z + dz);
        bool exy = Open(x + dx, y + dy, z);
        bool exz = Open(x + dx, y, z + dz);
        bool eyz = Open(x, y + dy, z + dz);
        if (_rule == VoxelCornerRule.NoCornerCutting)
        {
            return fx && fy && fz && exy && exz && eyz;
        }

        // A monotone route of three face steps: first face, then an edge cell containing it.
        return (fx && (exy || exz)) || (fy && (exy || eyz)) || (fz && (exz || eyz));
    }
}

/// <summary>Which distance a <see cref="VoxelHeuristic"/> computes.</summary>
public enum VoxelHeuristicKind
{
    /// <summary>Always zero (Dijkstra).</summary>
    Zero = 0,

    /// <summary><c>|dx| + |dy| + |dz|</c>: exact for six moves.</summary>
    Manhattan = 1,

    /// <summary>Exact empty-grid distance for 18 moves (faces cost 1, edges √2).</summary>
    Octile18 = 2,

    /// <summary>Exact empty-grid distance for 26 moves (faces 1, edges √2, corners √3).</summary>
    Octile26 = 3,
}

/// <summary>
/// The distance estimate for voxel searches: the exact shortest cost on an empty grid with the given moves, times a scale
/// (normally <see cref="AreaCosts.MinCost"/>), so it is admissible and consistent whenever every area multiplier is at
/// least the scale. With sorted absolute offsets a ≤ b ≤ c: 26 moves cost <c>a√3 + (b−a)√2 + (c−b)</c>; 18 moves cost <c>e√2 + (m−2e)</c> with <c>m = a+b+c</c> and
/// <c>e = min(⌊m/2⌋, a+b)</c> edge moves, each of which covers two axis units.
/// </summary>
public struct VoxelHeuristic : IHeuristic
{
    private readonly int _sizeX;
    private readonly int _sizeY;
    private readonly VoxelHeuristicKind _kind;
    private readonly double _scale;
    private int _gx;
    private int _gy;
    private int _gz;

    /// <summary>Creates the heuristic.</summary>
    /// <param name="sizeX">Grid size along x; positive.</param>
    /// <param name="sizeY">Grid size along y; positive.</param>
    /// <param name="kind">The distance.</param>
    /// <param name="scale">Multiplier on the distance, normally <see cref="AreaCosts.MinCost"/>.</param>
    public VoxelHeuristic(int sizeX, int sizeY, VoxelHeuristicKind kind, double scale = 1.0)
    {
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sizeX);
        ArgumentOutOfRangeException.ThrowIfNegativeOrZero(sizeY);
        _sizeX = sizeX;
        _sizeY = sizeY;
        _kind = kind;
        _scale = scale;
        _gx = _gy = _gz = 0;
    }

    /// <inheritdoc/>
    public void SetGoal(int goal)
    {
        VoxelCell c = VoxelGrid.Decode(goal, _sizeX, _sizeY);
        _gx = c.X;
        _gy = c.Y;
        _gz = c.Z;
    }

    /// <inheritdoc/>
    public readonly double Estimate(int node)
    {
        if (_kind == VoxelHeuristicKind.Zero)
        {
            return 0.0;
        }
        VoxelCell n = VoxelGrid.Decode(node, _sizeX, _sizeY);
        int dx = n.X > _gx ? n.X - _gx : _gx - n.X;
        int dy = n.Y > _gy ? n.Y - _gy : _gy - n.Y;
        int dz = n.Z > _gz ? n.Z - _gz : _gz - n.Z;

        // Sort ascending: a <= b <= c.
        int a = dx;
        int b = dy;
        int c = dz;
        if (a > b)
        {
            (a, b) = (b, a);
        }
        if (b > c)
        {
            (b, c) = (c, b);
        }
        if (a > b)
        {
            (a, b) = (b, a);
        }
        switch (_kind)
        {
            case VoxelHeuristicKind.Manhattan:
                return _scale * (a + b + c);
            case VoxelHeuristicKind.Octile18:
            {
                int m = a + b + c;
                int e = Math.Min(m / 2, a + b);
                return _scale * ((e * GridMath.Sqrt2) + (m - (2 * e)));
            }
            default:
                // 3D octile: a corner moves, then b-a edge moves, then c-b face moves.
                return _scale * ((a * VoxelMath.Sqrt3) + ((b - a) * GridMath.Sqrt2) + (c - b));
        }
    }
}
