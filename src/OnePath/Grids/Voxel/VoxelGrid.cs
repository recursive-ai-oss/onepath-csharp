using System;
using System.Collections.Generic;
using System.Threading;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath;

/// <summary>
/// A bounded block of voxels, one byte each: an area id (0..254, see <see cref="AreaCosts"/>) or <see cref="AreaCosts.Wall"/>
/// for a solid voxel. This is the tier-1 entry point (<c>grid.FindPath(a, b)</c>); tier 2 and 3 build on
/// <see cref="CreateTopology"/>, <see cref="AStar{TTopology, TCost, THeuristic}"/> and <see cref="VoxelHeuristic"/>.
/// <para>
/// <b>Storage layout (normative).</b> The block is cut into chunks of 16 × 16 × 16 voxels. The chunk table is a dense
/// array of <c>ceil(SizeX/16) * ceil(SizeY/16) * ceil(SizeZ/16)</c> slots, indexed
/// <c>cx + ChunksX * (cy + ChunksY * cz)</c>. A slot is either empty, meaning every voxel of the chunk has the grid's
/// <see cref="DefaultArea"/>, or a 4096-byte array indexed <c>(x &amp; 15) | ((y &amp; 15) &lt;&lt; 4) | ((z &amp; 15) &lt;&lt; 8)</c>.
/// A chunk is allocated by the first edit that differs from the default and then stays allocated. Voxels beyond the
/// block's size inside an edge chunk exist in storage but are never read as part of the grid. A world that is mostly
/// open (or mostly solid) costs memory only for the chunks that differ.
/// </para>
/// <para>
/// The voxel index used as a search node is <c>x + SizeX * (y + SizeY * z)</c>. Search buffers hold 20 bytes per voxel (cost, parent, mark and open-set position),
/// so the largest searchable block is <see cref="MaxVoxels"/> voxels.
/// </para>
/// <para>Reading is safe from several threads once no thread edits the grid. Edits must not overlap queries.</para>
/// </summary>
public sealed class VoxelGrid
{
    /// <summary>The largest number of voxels a grid may have (2^30).</summary>
    public const int MaxVoxels = 1 << 30;

    /// <summary>Edge length of a storage chunk in voxels.</summary>
    public const int ChunkSize = 16;

    private const int ChunkVolume = ChunkSize * ChunkSize * ChunkSize;
    private const int Shift = 4;
    private const int Mask = ChunkSize - 1;

    private static readonly VoxelOptions DefaultOptions = new();

    private readonly byte[]?[] _chunks;
    private readonly byte _fill;
    private readonly List<IVoxelChangeListener> _listeners = [];
    private int _allocated;
    private GridSearchBuffers? _spare;

    /// <summary>Creates a grid where every voxel has the same area.</summary>
    /// <param name="sizeX">Voxels along x; at least 1.</param>
    /// <param name="sizeY">Voxels along y; at least 1.</param>
    /// <param name="sizeZ">Voxels along z; at least 1.</param>
    /// <param name="defaultArea">Area of every voxel until edited: 0 for open space (the default) or <see cref="AreaCosts.Wall"/> for a solid block to carve.</param>
    /// <exception cref="ArgumentOutOfRangeException">A dimension is below 1 or the grid would exceed <see cref="MaxVoxels"/> voxels.</exception>
    public VoxelGrid(int sizeX, int sizeY, int sizeZ, byte defaultArea = 0)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(sizeX, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(sizeY, 1);
        ArgumentOutOfRangeException.ThrowIfLessThan(sizeZ, 1);
        if ((long)sizeX * sizeY * sizeZ > MaxVoxels)
        {
            throw new ArgumentOutOfRangeException(nameof(sizeX), $"A {sizeX}x{sizeY}x{sizeZ} grid has more than the supported {MaxVoxels} voxels.");
        }
        SizeX = sizeX;
        SizeY = sizeY;
        SizeZ = sizeZ;
        ChunksX = (sizeX + Mask) >> Shift;
        ChunksY = (sizeY + Mask) >> Shift;
        ChunksZ = (sizeZ + Mask) >> Shift;
        _chunks = new byte[ChunksX * ChunksY * ChunksZ][];
        _fill = defaultArea;
        Areas = new AreaCosts();
        Space = VoxelSpace.Unit;
    }

    /// <summary>Voxels along x.</summary>
    public int SizeX { get; }

    /// <summary>Voxels along y.</summary>
    public int SizeY { get; }

    /// <summary>Voxels along z.</summary>
    public int SizeZ { get; }

    /// <summary>Chunks along x.</summary>
    public int ChunksX { get; }

    /// <summary>Chunks along y.</summary>
    public int ChunksY { get; }

    /// <summary>Chunks along z.</summary>
    public int ChunksZ { get; }

    /// <summary>Number of voxels.</summary>
    public int VoxelCount => SizeX * SizeY * SizeZ;

    /// <summary>The area every voxel has until it is edited.</summary>
    public byte DefaultArea => _fill;

    /// <summary>Number of chunks that hold their own storage (the rest read as <see cref="DefaultArea"/>).</summary>
    public int AllocatedChunks => _allocated;

    /// <summary>The area cost table. Edit it before querying; changes must not overlap a running search.</summary>
    public AreaCosts Areas { get; }

    /// <summary>The world-space mapping. Default: unit voxels, origin (0, 0, 0).</summary>
    public VoxelSpace Space { get; set; }

    /// <summary>Incremented by every voxel edit that changes a voxel.</summary>
    public int Version { get; private set; }

    internal byte[]?[] ChunkTable => _chunks;

    /// <summary>Creates a grid from flags, x varying fastest then y then z; <see langword="true"/> is open space.</summary>
    /// <param name="walkable">Voxels in index order (<c>x + sizeX * (y + sizeY * z)</c>).</param>
    /// <param name="sizeX">Voxels along x.</param>
    /// <param name="sizeY">Voxels along y.</param>
    /// <param name="sizeZ">Voxels along z.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The size is invalid.</exception>
    /// <exception cref="ArgumentException">The span length is not <c>sizeX * sizeY * sizeZ</c>.</exception>
    public static VoxelGrid From(ReadOnlySpan<bool> walkable, int sizeX, int sizeY, int sizeZ)
    {
        var grid = new VoxelGrid(sizeX, sizeY, sizeZ);
        if (walkable.Length != grid.VoxelCount)
        {
            throw new ArgumentException($"Expected {grid.VoxelCount} voxels for a {sizeX}x{sizeY}x{sizeZ} grid but got {walkable.Length}.", nameof(walkable));
        }
        int i = 0;
        for (int z = 0; z < sizeZ; z++)
        {
            for (int y = 0; y < sizeY; y++)
            {
                for (int x = 0; x < sizeX; x++)
                {
                    if (!walkable[i++])
                    {
                        grid.Store(x, y, z, AreaCosts.Wall);
                    }
                }
            }
        }
        return grid;
    }

    /// <summary>Creates a grid from a 3D array indexed <c>[x, y, z]</c>; <see langword="true"/> is open space.</summary>
    /// <param name="walkable">Voxels.</param>
    /// <returns>The grid.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="walkable"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The array is empty or too large.</exception>
    public static VoxelGrid From(bool[,,] walkable)
    {
        ArgumentNullException.ThrowIfNull(walkable);
        var grid = new VoxelGrid(walkable.GetLength(0), walkable.GetLength(1), walkable.GetLength(2));
        for (int x = 0; x < grid.SizeX; x++)
        {
            for (int y = 0; y < grid.SizeY; y++)
            {
                for (int z = 0; z < grid.SizeZ; z++)
                {
                    if (!walkable[x, y, z])
                    {
                        grid.Store(x, y, z, AreaCosts.Wall);
                    }
                }
            }
        }
        return grid;
    }

    /// <summary>Whether the voxel lies inside the grid.</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <returns>True if inside.</returns>
    public bool Contains(int x, int y, int z) => (uint)x < (uint)SizeX && (uint)y < (uint)SizeY && (uint)z < (uint)SizeZ;

    /// <summary>The search node index of a voxel: <c>x + SizeX * (y + SizeY * z)</c>.</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <returns>The index.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The voxel is outside the grid.</exception>
    public int NodeIndex(int x, int y, int z)
    {
        Check(x, y, z);
        return x + (SizeX * (y + (SizeY * z)));
    }

    /// <summary>The voxel of a search node index.</summary>
    /// <param name="node">Index in 0..VoxelCount-1.</param>
    /// <returns>The voxel.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The index is outside the grid.</exception>
    public VoxelCell CellOf(int node)
    {
        if ((uint)node >= (uint)VoxelCount)
        {
            throw new ArgumentOutOfRangeException(nameof(node), node, "The node index is outside the grid.");
        }
        return Decode(node, SizeX, SizeY);
    }

    /// <summary>The area id of a voxel.</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <returns>0..254, or <see cref="AreaCosts.Wall"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The voxel is outside the grid.</exception>
    public byte GetArea(int x, int y, int z)
    {
        Check(x, y, z);
        return Read(_chunks, ChunksX, ChunksY, _fill, x, y, z);
    }

    /// <summary>True if the voxel is not solid.</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <returns>True if open.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The voxel is outside the grid.</exception>
    public bool IsWalkable(int x, int y, int z) => GetArea(x, y, z) != AreaCosts.Wall;

    /// <summary>Sets the area of a voxel (use <see cref="AreaCosts.Wall"/> for solid) and notifies listeners if it changed.</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <param name="area">New area id.</param>
    /// <exception cref="ArgumentOutOfRangeException">The voxel is outside the grid.</exception>
    public void SetArea(int x, int y, int z, byte area)
    {
        Check(x, y, z);
        if (Read(_chunks, ChunksX, ChunksY, _fill, x, y, z) == area)
        {
            return;
        }
        Store(x, y, z, area);
        Version++;
        for (int l = 0; l < _listeners.Count; l++)
        {
            _listeners[l].OnVoxelChanged(x, y, z);
        }
    }

    /// <summary>Makes a voxel solid or open (area 0).</summary>
    /// <param name="x">Position along x.</param>
    /// <param name="y">Position along y.</param>
    /// <param name="z">Position along z.</param>
    /// <param name="walkable">True for open space, false for solid.</param>
    /// <exception cref="ArgumentOutOfRangeException">The voxel is outside the grid.</exception>
    public void SetWalkable(int x, int y, int z, bool walkable) => SetArea(x, y, z, walkable ? (byte)0 : AreaCosts.Wall);

    /// <summary>Registers a listener for voxel edits.</summary>
    /// <param name="listener">The listener.</param>
    /// <exception cref="ArgumentNullException">The listener is null.</exception>
    public void AddListener(IVoxelChangeListener listener)
    {
        ArgumentNullException.ThrowIfNull(listener);
        _listeners.Add(listener);
    }

    /// <summary>Unregisters a listener.</summary>
    /// <param name="listener">The listener.</param>
    /// <returns>True if it was registered.</returns>
    public bool RemoveListener(IVoxelChangeListener listener) => _listeners.Remove(listener);

    /// <summary>
    /// The 4096 area ids of one chunk (indexed as described on the class), or an empty span when the chunk holds only
    /// <see cref="DefaultArea"/>. Voxels past the grid's size in an edge chunk are padding.
    /// </summary>
    /// <param name="cx">Chunk position along x.</param>
    /// <param name="cy">Chunk position along y.</param>
    /// <param name="cz">Chunk position along z.</param>
    /// <returns>The chunk contents or an empty span.</returns>
    /// <exception cref="ArgumentOutOfRangeException">The chunk is outside the table.</exception>
    public ReadOnlySpan<byte> GetChunk(int cx, int cy, int cz)
    {
        if ((uint)cx >= (uint)ChunksX || (uint)cy >= (uint)ChunksY || (uint)cz >= (uint)ChunksZ)
        {
            throw new ArgumentOutOfRangeException(nameof(cx), $"Chunk ({cx}, {cy}, {cz}) is outside the {ChunksX}x{ChunksY}x{ChunksZ} chunk table.");
        }
        byte[]? c = _chunks[cx + (ChunksX * (cy + (ChunksY * cz)))];
        return c is null ? default : c;
    }

    /// <summary>The centre of a voxel in world space.</summary>
    /// <param name="cell">The voxel.</param>
    /// <returns>The world position.</returns>
    public VoxelPoint CellCenter(VoxelCell cell)
    {
        VoxelSpace s = Space;
        double size = s.SizeOrOne;
        return new VoxelPoint(s.OriginX + ((cell.X + 0.5) * size), s.OriginY + ((cell.Y + 0.5) * size), s.OriginZ + ((cell.Z + 0.5) * size));
    }

    /// <summary>The voxel containing a world position.</summary>
    /// <param name="point">The position; each coordinate within ±2^21.</param>
    /// <param name="cell">The voxel, when inside the grid.</param>
    /// <returns>False if the position lies outside the grid.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    public bool TryGetCell(VoxelPoint point, out VoxelCell cell)
    {
        GridSpace.CheckCoordinate(point.X, nameof(point));
        GridSpace.CheckCoordinate(point.Y, nameof(point));
        GridSpace.CheckCoordinate(point.Z, nameof(point));
        VoxelSpace s = Space;
        double size = s.SizeOrOne;
        double fx = Math.Floor((point.X - s.OriginX) / size);
        double fy = Math.Floor((point.Y - s.OriginY) / size);
        double fz = Math.Floor((point.Z - s.OriginZ) / size);
        if (fx >= 0.0 && fx < SizeX && fy >= 0.0 && fy < SizeY && fz >= 0.0 && fz < SizeZ)
        {
            cell = new VoxelCell((int)fx, (int)fy, (int)fz);
            return true;
        }
        cell = default;
        return false;
    }

    /// <summary>Creates the search topology for a movement rule.</summary>
    /// <param name="connectivity">Six, eighteen or twenty-six moves.</param>
    /// <param name="corners">Corner rule for edge and corner moves.</param>
    /// <returns>A view of this grid. Edits show through (it reads the chunk table).</returns>
    /// <exception cref="ArgumentOutOfRangeException">An enum value is undefined.</exception>
    public VoxelTopology CreateTopology(VoxelConnectivity connectivity = VoxelConnectivity.TwentySix, VoxelCornerRule corners = VoxelCornerRule.NoCornerCutting)
    {
        if (connectivity is not (VoxelConnectivity.Six or VoxelConnectivity.Eighteen or VoxelConnectivity.TwentySix))
        {
            throw new ArgumentOutOfRangeException(nameof(connectivity), connectivity, "Connectivity must be Six, Eighteen or TwentySix.");
        }
        if (corners is not (VoxelCornerRule.NoCornerCutting or VoxelCornerRule.NoSqueezing or VoxelCornerRule.AllowAll))
        {
            throw new ArgumentOutOfRangeException(nameof(corners), corners, "Undefined corner rule.");
        }
        return new VoxelTopology(this, connectivity, corners);
    }

    /// <summary>Finds a least-cost path between two voxels.</summary>
    /// <param name="start">Start voxel.</param>
    /// <param name="goal">Goal voxel.</param>
    /// <param name="options">Query options; null means <see cref="VoxelOptions.Default"/>.</param>
    /// <returns>The result; check <see cref="VoxelPath.Status"/>. Voxels outside the grid or solid give <see cref="PathOutcome.StartInvalid"/> or <see cref="PathOutcome.GoalInvalid"/>.</returns>
    public VoxelPath FindPath(VoxelCell start, VoxelCell goal, VoxelOptions? options = null) => FindPathCore(start, goal, options ?? DefaultOptions, false);

    /// <summary>Finds a least-cost path between two world positions (their containing voxels); the result carries world-space voxel centres.</summary>
    /// <param name="start">Start position; each coordinate within ±2^21.</param>
    /// <param name="goal">Goal position; each coordinate within ±2^21.</param>
    /// <param name="options">Query options; null means <see cref="VoxelOptions.Default"/>.</param>
    /// <returns>The result. A position outside the grid gives <see cref="PathOutcome.StartInvalid"/> or <see cref="PathOutcome.GoalInvalid"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException">A coordinate is NaN or beyond ±2^21.</exception>
    public VoxelPath FindPath(VoxelPoint start, VoxelPoint goal, VoxelOptions? options = null)
    {
        bool a = TryGetCell(start, out VoxelCell s);
        bool b = TryGetCell(goal, out VoxelCell g);
        if (!a)
        {
            return VoxelPath.Invalid(PathOutcome.StartInvalid);
        }
        if (!b)
        {
            return VoxelPath.Invalid(PathOutcome.GoalInvalid);
        }
        return FindPathCore(s, g, options ?? DefaultOptions, true);
    }

    private VoxelPath FindPathCore(VoxelCell start, VoxelCell goal, VoxelOptions o, bool world)
    {
        if (!Contains(start.X, start.Y, start.Z) || Read(_chunks, ChunksX, ChunksY, _fill, start.X, start.Y, start.Z) == AreaCosts.Wall)
        {
            return VoxelPath.Invalid(PathOutcome.StartInvalid);
        }
        if (!Contains(goal.X, goal.Y, goal.Z) || Read(_chunks, ChunksX, ChunksY, _fill, goal.X, goal.Y, goal.Z) == AreaCosts.Wall)
        {
            return VoxelPath.Invalid(PathOutcome.GoalInvalid);
        }
        if (o.Blocked is not null && (o.Blocked(start) || o.Blocked(goal)))
        {
            return VoxelPath.Invalid(o.Blocked(start) ? PathOutcome.StartInvalid : PathOutcome.GoalInvalid);
        }
        VoxelTopology topology = CreateTopology(o.Connectivity, o.Corners);
        int a = start.X + (SizeX * (start.Y + (SizeY * start.Z)));
        int b = goal.X + (SizeX * (goal.Y + (SizeY * goal.Z)));
        GridHeuristic h = o.Heuristic == GridHeuristic.Auto ? (o.Connectivity == VoxelConnectivity.Six ? GridHeuristic.Manhattan : GridHeuristic.Octile) : o.Heuristic;
        double scale = Areas.MinCost;
        Func<VoxelCell, VoxelPoint>? centre = world ? CellCenter : null;

        GridSearchBuffers? buffers = Interlocked.Exchange(ref _spare, null);
        buffers ??= new GridSearchBuffers(VoxelCount);
        try
        {
            VoxelHeuristic heuristic = h switch
            {
                GridHeuristic.Zero => new VoxelHeuristic(SizeX, SizeY, VoxelHeuristicKind.Zero, scale),
                GridHeuristic.Manhattan => new VoxelHeuristic(SizeX, SizeY, VoxelHeuristicKind.Manhattan, scale),
                _ => new VoxelHeuristic(SizeX, SizeY, o.Connectivity == VoxelConnectivity.Six ? VoxelHeuristicKind.Manhattan : o.Connectivity == VoxelConnectivity.Eighteen ? VoxelHeuristicKind.Octile18 : VoxelHeuristicKind.Octile26, scale),
            };
            if (o.StepCost is null && o.Blocked is null)
            {
                return VoxelSearchRunner.Run(topology, default(DefaultStepCost), heuristic, buffers, SizeX, SizeY, a, b, o, centre);
            }
            var cost = new VoxelSearchRunner.DelegateStepCost { SizeX = SizeX, SizeY = SizeY, Step = o.StepCost, Blocked = o.Blocked };
            return VoxelSearchRunner.Run(topology, cost, heuristic, buffers, SizeX, SizeY, a, b, o, centre);
        }
        finally
        {
            _spare = buffers;
        }
    }

    internal static VoxelCell Decode(int node, int sizeX, int sizeY)
    {
        int layer = sizeX * sizeY;
        int z = node / layer;
        int rest = node - (z * layer);
        int y = rest / sizeX;
        return new VoxelCell(rest - (y * sizeX), y, z);
    }

    // Callers guarantee 0 <= x < SizeX etc., so the chunk index is inside the table and the in-chunk index inside 4096.
    internal static byte Read(byte[]?[] chunks, int chunksX, int chunksY, byte fill, int x, int y, int z)
    {
        byte[]? c = chunks[(x >> Shift) + (chunksX * ((y >> Shift) + (chunksY * (z >> Shift))))];
        return c is null ? fill : c[(x & Mask) | ((y & Mask) << Shift) | ((z & Mask) << (2 * Shift))];
    }

    private void Store(int x, int y, int z, byte area)
    {
        int ci = (x >> Shift) + (ChunksX * ((y >> Shift) + (ChunksY * (z >> Shift))));
        byte[]? c = _chunks[ci];
        if (c is null)
        {
            c = new byte[ChunkVolume];
            Array.Fill(c, _fill);
            _chunks[ci] = c;
            _allocated++;
        }
        c[(x & Mask) | ((y & Mask) << Shift) | ((z & Mask) << (2 * Shift))] = area;
    }

    private void Check(int x, int y, int z)
    {
        if (!Contains(x, y, z))
        {
            throw new ArgumentOutOfRangeException(nameof(x), $"Voxel ({x}, {y}, {z}) is outside the {SizeX}x{SizeY}x{SizeZ} grid.");
        }
    }
}
