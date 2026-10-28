using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Graphs;
using RecursiveAI.OnePath.Grids;
using RecursiveAI.OnePath.Grids.Voxel;

namespace RecursiveAI.OnePath.Debug;

/// <summary>
/// Plain-data copy of a grid path query for a host to draw: the path, the cells the search expanded and the world points.
/// Immutable; nothing in it refers back to the grid. The expansion list is filled only when the query ran with
/// <c>RecordExpansions</c> set. For a replanner result it holds the cells that replan settled or reopened.
/// </summary>
public sealed class GridPathDebug
{
    internal GridPathDebug(GridPath path)
    {
        Algorithm = path.Algorithm;
        Status = path.Status;
        Cost = path.Cost;
        Cells = path.Cells.ToArray();
        Points = path.Points.ToArray();
        ExpandedCells = path.ExpandedCells.ToArray();
        ExpandedCount = path.ExpandedCount;
    }

    /// <summary>The algorithm that ran (A*, a jump point search, Theta*, HPA*, or a replanner's).</summary>
    public GridAlgorithm Algorithm { get; }

    /// <summary>The outcome of the query.</summary>
    public PathStatus Status { get; }

    /// <summary>Path cost; +∞ unless found.</summary>
    public double Cost { get; }

    /// <summary>The path's cells, start first: every cell for A*, the jump points or waypoints for jump point and any-angle searches, the refined cells for HPA*.</summary>
    public IReadOnlyList<Cell> Cells { get; }

    /// <summary>The path's cell centres in world units; empty unless the query took world points.</summary>
    public IReadOnlyList<WorldPoint> Points { get; }

    /// <summary>The cells expanded, in order (jump points for JPS, abstract-graph nodes' cells for HPA*); empty unless recorded. Its length equals <see cref="ExpandedCount"/> when recorded.</summary>
    public IReadOnlyList<Cell> ExpandedCells { get; }

    /// <summary>Number of expansions, recorded or not.</summary>
    public int ExpandedCount { get; }
}

/// <summary>Plain-data copy of a voxel path query; see <see cref="GridPathDebug"/>.</summary>
public sealed class VoxelPathDebug
{
    internal VoxelPathDebug(VoxelPath path)
    {
        Status = path.Status;
        Cost = path.Cost;
        Cells = path.Cells.ToArray();
        Points = path.Points.ToArray();
        ExpandedCells = path.ExpandedCells.ToArray();
        ExpandedCount = path.ExpandedCount;
    }

    /// <summary>The outcome of the query.</summary>
    public PathStatus Status { get; }

    /// <summary>Path cost; +∞ unless found.</summary>
    public double Cost { get; }

    /// <summary>The path's voxels, start first.</summary>
    public IReadOnlyList<VoxelCell> Cells { get; }

    /// <summary>The path's voxel centres in world units; empty unless the query took world points.</summary>
    public IReadOnlyList<VoxelPoint> Points { get; }

    /// <summary>The voxels expanded, in order; empty unless recorded.</summary>
    public IReadOnlyList<VoxelCell> ExpandedCells { get; }

    /// <summary>Number of expansions, recorded or not.</summary>
    public int ExpandedCount { get; }
}

/// <summary>Plain-data copy of a graph path query; see <see cref="GridPathDebug"/>.</summary>
public sealed class GraphPathDebug
{
    internal GraphPathDebug(GraphPath path)
    {
        Algorithm = path.Algorithm;
        Status = path.Status;
        Cost = path.Cost;
        Nodes = path.Nodes.ToArray();
        Points = path.Points.ToArray();
        ExpandedNodes = path.ExpandedNodes.ToArray();
        ExpandedCount = path.ExpandedCount;
    }

    /// <summary>The algorithm that ran.</summary>
    public GraphAlgorithm Algorithm { get; }

    /// <summary>The outcome of the query.</summary>
    public PathStatus Status { get; }

    /// <summary>Path cost; +∞ unless found.</summary>
    public double Cost { get; }

    /// <summary>The path's nodes, start first.</summary>
    public IReadOnlyList<int> Nodes { get; }

    /// <summary>The nodes' positions for point graphs; empty otherwise.</summary>
    public IReadOnlyList<Point3> Points { get; }

    /// <summary>The nodes expanded, in order; empty unless <c>GraphOptions.RecordExpansions</c> was set. A bidirectional search lists both directions in the order they ran.</summary>
    public IReadOnlyList<int> ExpandedNodes { get; }

    /// <summary>Number of expansions, recorded or not.</summary>
    public int ExpandedCount { get; }
}

/// <summary>
/// Plain-data copy of a flow field: the integration field (least cost to the nearest goal) and the direction field (the cell each
/// cell steps to), row-major, <c>index = y * Width + x</c>.
/// </summary>
public sealed class FlowFieldDebug
{
    internal FlowFieldDebug(GridFlowField field, int width, int height, Cell[] goals)
    {
        Width = width;
        Height = height;
        Status = field.Status;
        Settled = field.Settled;
        Costs = field.Costs.ToArray();
        NextCell = field.Next.ToArray();
        Goals = goals;
    }

    /// <summary>Columns.</summary>
    public int Width { get; }

    /// <summary>Rows.</summary>
    public int Height { get; }

    /// <summary>Whether the field is complete or a sliced run or repair is still going.</summary>
    public FlowFieldStatus Status { get; }

    /// <summary>Cells settled by the current run or repair.</summary>
    public int Settled { get; }

    /// <summary>Integration cost per cell; +∞ for unreachable cells and for cells not reached yet.</summary>
    public IReadOnlyList<double> Costs { get; }

    /// <summary>Index of the cell each cell steps to, or -1 for goals, unreachable cells and cells not settled yet.</summary>
    public IReadOnlyList<int> NextCell { get; }

    /// <summary>The goal cells the field was created with.</summary>
    public IReadOnlyList<Cell> Goals { get; }

    /// <summary>The step of a cell as a cell offset.</summary>
    /// <param name="x">Column.</param>
    /// <param name="y">Row.</param>
    /// <param name="dx">Column offset of the step; 0 if none.</param>
    /// <param name="dy">Row offset of the step; 0 if none.</param>
    /// <returns>False when the cell has no step or lies outside the grid.</returns>
    public bool TryGetStep(int x, int y, out int dx, out int dy)
    {
        dx = 0;
        dy = 0;
        if ((uint)x >= (uint)Width || (uint)y >= (uint)Height)
        {
            return false;
        }
        int next = NextCell[(y * Width) + x];
        if (next < 0)
        {
            return false;
        }
        int ny = next / Width;
        dx = next - (ny * Width) - x;
        dy = ny - y;
        return true;
    }
}

/// <summary>One edge of the HPA* abstract graph.</summary>
/// <param name="From">Cell the edge leaves.</param>
/// <param name="To">Cell the edge reaches.</param>
/// <param name="Cost">Edge cost.</param>
/// <param name="IsInter"><see langword="true"/> for an edge between two chunks (one step across a border), <see langword="false"/> for a cost inside a chunk.</param>
public readonly record struct HpaDebugEdge(Cell From, Cell To, double Cost, bool IsInter);

/// <summary>Plain-data copy of an HPA* abstract graph: the entrance cells and the edges between them.</summary>
public sealed class HpaGraphDebug
{
    internal HpaGraphDebug(int chunkSize, int clustersX, int clustersY, Cell[] nodes, int[] nodeCluster, HpaDebugEdge[] edges)
    {
        ChunkSize = chunkSize;
        ClustersX = clustersX;
        ClustersY = clustersY;
        Nodes = nodes;
        NodeCluster = nodeCluster;
        Edges = edges;
    }

    /// <summary>Chunk side in cells.</summary>
    public int ChunkSize { get; }

    /// <summary>Chunk columns.</summary>
    public int ClustersX { get; }

    /// <summary>Chunk rows.</summary>
    public int ClustersY { get; }

    /// <summary>Abstract nodes (entrance cells), chunk by chunk in chunk order.</summary>
    public IReadOnlyList<Cell> Nodes { get; }

    /// <summary>Chunk index of each node in <see cref="Nodes"/>.</summary>
    public IReadOnlyList<int> NodeCluster { get; }

    /// <summary>All edges: per chunk, the finite intra costs between its nodes (both directions) then the inter edges, in index order.</summary>
    public IReadOnlyList<HpaDebugEdge> Edges { get; }
}

/// <summary>Builds the plain-data debug views of grid, voxel, graph, flow-field and HPA* results. Allocates; meant for drawing, not for the hot path.</summary>
public static class GridDebug
{
    /// <summary>Copies a grid path result (also a hex or replanner result).</summary>
    /// <param name="path">The result.</param>
    /// <returns>The copy.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    public static GridPathDebug Describe(GridPath path)
    {
        ArgumentNullException.ThrowIfNull(path);
        return new GridPathDebug(path);
    }

    /// <summary>Copies a voxel path result.</summary>
    /// <param name="path">The result.</param>
    /// <returns>The copy.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    public static VoxelPathDebug Describe(VoxelPath path)
    {
        ArgumentNullException.ThrowIfNull(path);
        return new VoxelPathDebug(path);
    }

    /// <summary>Copies a graph path result.</summary>
    /// <param name="path">The result.</param>
    /// <returns>The copy.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    public static GraphPathDebug Describe(GraphPath path)
    {
        ArgumentNullException.ThrowIfNull(path);
        return new GraphPathDebug(path);
    }

    /// <summary>Copies a flow field's integration and direction fields as they stand (a sliced run shows its progress).</summary>
    /// <param name="field">The field.</param>
    /// <returns>The copy.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="field"/> is null.</exception>
    public static FlowFieldDebug Describe(GridFlowField field)
    {
        ArgumentNullException.ThrowIfNull(field);
        return new FlowFieldDebug(field, field.Width, field.Height, field.GoalCells());
    }

    /// <summary>Copies an HPA* hierarchy's abstract graph (refreshing it first if edits are pending).</summary>
    /// <param name="hierarchy">The hierarchy.</param>
    /// <returns>The copy.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="hierarchy"/> is null.</exception>
    public static HpaGraphDebug Describe(HpaHierarchy hierarchy)
    {
        ArgumentNullException.ThrowIfNull(hierarchy);
        hierarchy.Refresh();
        int w = hierarchy.Grid.Width;
        var nodes = new List<Cell>();
        var nodeCluster = new List<int>();
        var edges = new List<HpaDebugEdge>();
        for (int c = 0; c < hierarchy.ClusterCount; c++)
        {
            ReadOnlySpan<int> cells = hierarchy.ClusterNodes(c);
            for (int i = 0; i < cells.Length; i++)
            {
                nodes.Add(ToCell(cells[i], w));
                nodeCluster.Add(c);
            }
            for (int i = 0; i < cells.Length; i++)
            {
                for (int j = 0; j < cells.Length; j++)
                {
                    double cost = hierarchy.IntraCost(c, i, j);
                    if (i != j && !double.IsPositiveInfinity(cost))
                    {
                        edges.Add(new HpaDebugEdge(ToCell(cells[i], w), ToCell(cells[j], w), cost, false));
                    }
                }
            }
            for (int i = 0; i < cells.Length; i++)
            {
                hierarchy.GetInterEdges(c, i, out ReadOnlySpan<int> targets, out ReadOnlySpan<double> costs);
                for (int k = 0; k < targets.Length; k++)
                {
                    edges.Add(new HpaDebugEdge(ToCell(cells[i], w), ToCell(targets[k], w), costs[k], true));
                }
            }
        }
        return new HpaGraphDebug(hierarchy.ChunkSize, hierarchy.ClustersX, hierarchy.ClustersY, nodes.ToArray(), nodeCluster.ToArray(), edges.ToArray());
    }

    private static Cell ToCell(int index, int width)
    {
        int y = index / width;
        return new Cell(index - (y * width), y);
    }
}
