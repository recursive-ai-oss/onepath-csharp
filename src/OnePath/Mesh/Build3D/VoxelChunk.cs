using System;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>A run of solid voxels of one material in a chunk column: rows <see cref="Start"/> up to <see cref="Start"/> + <see cref="Length"/>.</summary>
/// <param name="Start">First solid row (0 or more).</param>
/// <param name="Length">Number of solid rows (at least 1).</param>
/// <param name="Material">Material id, 1..255 (0 is empty and never stored).</param>
public readonly record struct VoxelRun(int Start, int Length, byte Material);

/// <summary>
/// A block of solid voxels given as run-length columns, the form a voxel world supplies: for every (x, z) column the
/// sorted, disjoint runs of solid voxels with their material ids. <see cref="Voxelizer.AddChunk"/> turns it into
/// spans without any triangles.
/// </summary>
public sealed class VoxelChunk
{
    private readonly int[] _columnStarts;
    private readonly VoxelRun[] _runs;

    private VoxelChunk(int width, int depth, int height, int[] starts, VoxelRun[] runs)
    {
        Width = width;
        Depth = depth;
        Height = height;
        _columnStarts = starts;
        _runs = runs;
    }

    /// <summary>Columns along x.</summary>
    public int Width { get; }

    /// <summary>Columns along z.</summary>
    public int Depth { get; }

    /// <summary>Rows along y; every run ends at or below this.</summary>
    public int Height { get; }

    /// <summary>For column <c>x + z * Width</c>, the index of its first run in <see cref="Runs"/>; one extra entry ends the last column.</summary>
    public ReadOnlySpan<int> ColumnStarts => _columnStarts;

    /// <summary>All runs, column by column, each column sorted by start.</summary>
    public ReadOnlySpan<VoxelRun> Runs => _runs;

    /// <summary>
    /// Builds a chunk from a dense array of material ids (0 = empty), indexed <c>(z * width + x) * height + y</c>.
    /// Adjacent voxels of the same material form one run.
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">A dimension is below 1 or too large.</exception>
    /// <exception cref="ArgumentException">The array length is not width * depth * height.</exception>
    public static VoxelChunk FromDense(int width, int depth, int height, ReadOnlySpan<byte> voxels)
    {
        CheckDims(width, depth, height);
        if (voxels.Length != (long)width * depth * height)
        {
            throw new ArgumentException($"voxels has {voxels.Length} entries for a {width} x {depth} x {height} chunk.", nameof(voxels));
        }
        int cols = width * depth;
        var starts = new int[cols + 1];
        var runs = new System.Collections.Generic.List<VoxelRun>();
        for (int c = 0; c < cols; c++)
        {
            starts[c] = runs.Count;
            ReadOnlySpan<byte> col = voxels.Slice(c * height, height);
            int y = 0;
            while (y < height)
            {
                byte m = col[y];
                if (m == 0)
                {
                    y++;
                    continue;
                }
                int e = y + 1;
                while (e < height && col[e] == m)
                {
                    e++;
                }
                runs.Add(new VoxelRun(y, e - y, m));
                y = e;
            }
        }
        starts[cols] = runs.Count;
        return new VoxelChunk(width, depth, height, starts, runs.ToArray());
    }

    /// <summary>Builds a chunk from run-length columns, validating them.</summary>
    /// <param name="width">Columns along x.</param>
    /// <param name="depth">Columns along z.</param>
    /// <param name="height">Rows along y.</param>
    /// <param name="columnStarts">width * depth + 1 non-decreasing offsets into <paramref name="runs"/>, starting at 0 and ending at runs.Length.</param>
    /// <param name="runs">The runs. Within a column they must be sorted, non-overlapping, inside 0..height, of length at least 1, with material at least 1.</param>
    /// <exception cref="ArgumentException">The offsets or runs violate the rules above; the message names the column and run.</exception>
    public static VoxelChunk FromRuns(int width, int depth, int height, ReadOnlySpan<int> columnStarts, ReadOnlySpan<VoxelRun> runs)
    {
        CheckDims(width, depth, height);
        int cols = width * depth;
        if (columnStarts.Length != cols + 1 || columnStarts[0] != 0 || columnStarts[cols] != runs.Length)
        {
            throw new ArgumentException($"columnStarts must have {cols + 1} entries, begin at 0 and end at {runs.Length}.", nameof(columnStarts));
        }
        for (int c = 0; c < cols; c++)
        {
            int a = columnStarts[c];
            int b = columnStarts[c + 1];
            if (b < a)
            {
                throw new ArgumentException($"columnStarts decreases at column {c}.", nameof(columnStarts));
            }
            int prevEnd = 0;
            for (int r = a; r < b; r++)
            {
                VoxelRun run = runs[r];
                if (run.Material == 0 || run.Length < 1 || run.Start < prevEnd || (long)run.Start + run.Length > height)
                {
                    throw new ArgumentException($"Run {r} in column {c} is invalid: start {run.Start}, length {run.Length}, material {run.Material} (runs must be sorted, disjoint, within 0..{height}, non-empty, with material >= 1).", nameof(runs));
                }
                prevEnd = run.Start + run.Length;
            }
        }
        return new VoxelChunk(width, depth, height, columnStarts.ToArray(), runs.ToArray());
    }

    private static void CheckDims(int width, int depth, int height)
    {
        if (width < 1 || width > HeightfieldFrame.MaxSide)
        {
            throw new ArgumentOutOfRangeException(nameof(width), width, $"The width must be in 1..{HeightfieldFrame.MaxSide}.");
        }
        if (depth < 1 || depth > HeightfieldFrame.MaxSide)
        {
            throw new ArgumentOutOfRangeException(nameof(depth), depth, $"The depth must be in 1..{HeightfieldFrame.MaxSide}.");
        }
        if (height < 1 || height > HeightfieldFrame.MaxHeight)
        {
            throw new ArgumentOutOfRangeException(nameof(height), height, $"The height must be in 1..{HeightfieldFrame.MaxHeight}.");
        }
        if ((long)width * depth > HeightfieldFrame.MaxColumns)
        {
            throw new ArgumentOutOfRangeException(nameof(width), width, $"Width times depth must not exceed {HeightfieldFrame.MaxColumns}.");
        }
    }
}
