using System;
using System.Buffers;
using System.Buffers.Binary;
using System.Collections.Generic;
using OneScope.Sdk;
using RecursiveAI.OnePath.Grids;

namespace RecursiveAI.OnePath.Scope;

/// <summary>What an edit in a grid run does.</summary>
public enum GridRunOpKind : byte
{
    /// <summary>Sets a cell's area (255 is a wall) and restarts the search, or recomputes the flow field.</summary>
    SetArea = 1,

    /// <summary>Moves a search's start and restarts it. Not valid in a flow-field run.</summary>
    SetStart = 2,

    /// <summary>Moves a search's goal and restarts it; in a flow-field run, makes this cell the only goal and recomputes.</summary>
    SetGoal = 3,
}

/// <summary>One edit applied before a step of a grid run. See docs/replay.md, "Grid runs".</summary>
/// <param name="Kind">What the edit does.</param>
/// <param name="X">The cell's column, 0 to width - 1.</param>
/// <param name="Y">The cell's row, 0 to height - 1.</param>
/// <param name="Area">For <see cref="GridRunOpKind.SetArea"/>, the new area id (255 = wall); otherwise 0.</param>
public readonly record struct GridRunOp(GridRunOpKind Kind, int X, int Y, byte Area)
{
    /// <summary>An edit that sets a cell's area.</summary>
    public static GridRunOp SetArea(int x, int y, byte area) => new(GridRunOpKind.SetArea, x, y, area);

    /// <summary>An edit that moves the start.</summary>
    public static GridRunOp SetStart(int x, int y) => new(GridRunOpKind.SetStart, x, y, 0);

    /// <summary>An edit that moves the goal.</summary>
    public static GridRunOp SetGoal(int x, int y) => new(GridRunOpKind.SetGoal, x, y, 0);
}

/// <summary>
/// Writes grid runs for the viewer: a grid, search or flow-field settings and per-step edits (<c>OPGR</c>, see docs/replay.md). Each time step
/// of the run advances the search (or the flow field) by <c>iterationsPerStep</c> queue pops, after applying that step's edits.
/// </summary>
public static class GridRun
{
    /// <summary>The file magic, <c>OPGR</c>.</summary>
    public static ReadOnlySpan<byte> Magic => "OPGR"u8;

    /// <summary>Most cells a viewer opens (the grid is drawn cell by cell).</summary>
    public const int MaxCells = 1 << 22;

    /// <summary>Most steps a run may hold.</summary>
    public const int MaxSteps = 1 << 22;

    /// <summary>Most flow-field goals.</summary>
    public const int MaxGoals = 1024;

    /// <summary>Bytes of one encoded edit.</summary>
    public const int OpBytes = 10;

    /// <summary>Most edits in one step.</summary>
    public const int MaxOpsPerStep = 1 << 16;

    /// <summary>Writes a search run: A*, JPS or JPS+ from <paramref name="start"/> to <paramref name="goal"/>.</summary>
    /// <param name="grid">The grid at step 0 (its state is saved into the file).</param>
    /// <param name="options">Connectivity, diagonal rule, agent size and algorithm are used (A*, JPS or JPS+); the rest stay default. Null for the defaults.</param>
    /// <param name="start">The start cell.</param>
    /// <param name="goal">The goal cell.</param>
    /// <param name="iterationsPerStep">Queue pops per step: 1 to 65536.</param>
    /// <param name="steps">The edits of each step; the count is the number of steps.</param>
    /// <param name="into">Receives the bytes.</param>
    /// <exception cref="ArgumentException">An argument is outside the limits above, or a cell is outside the grid.</exception>
    public static void WriteSearch(Grid grid, GridOptions? options, Cell start, Cell goal, int iterationsPerStep, IReadOnlyList<IReadOnlyList<GridRunOp>> steps, IBufferWriter<byte> into) =>
        Write(1, grid, options, [start, goal], iterationsPerStep, steps, into);

    /// <summary>Writes a flow-field run towards <paramref name="goals"/>.</summary>
    /// <param name="grid">The grid at step 0.</param>
    /// <param name="options">Connectivity, diagonal rule and agent size are used.</param>
    /// <param name="goals">1 to 1024 goal cells.</param>
    /// <param name="iterationsPerStep">Queue pops per step: 1 to 65536.</param>
    /// <param name="steps">The edits of each step; the count is the number of steps.</param>
    /// <param name="into">Receives the bytes.</param>
    /// <exception cref="ArgumentException">An argument is outside the limits above, or a cell is outside the grid.</exception>
    public static void WriteFlow(Grid grid, GridOptions? options, ReadOnlySpan<Cell> goals, int iterationsPerStep, IReadOnlyList<IReadOnlyList<GridRunOp>> steps, IBufferWriter<byte> into) =>
        Write(2, grid, options, goals, iterationsPerStep, steps, into);

    private static void Write(byte kind, Grid grid, GridOptions? options, ReadOnlySpan<Cell> points, int slice, IReadOnlyList<IReadOnlyList<GridRunOp>> steps, IBufferWriter<byte> into)
    {
        ArgumentNullException.ThrowIfNull(grid);
        ArgumentNullException.ThrowIfNull(steps);
        ArgumentNullException.ThrowIfNull(into);
        options ??= GridOptions.Default;
        if (slice is < 1 or > 65536 || steps.Count > MaxSteps || points.Length is < 1 or > MaxGoals || (long)grid.Width * grid.Height > MaxCells)
        {
            throw new ArgumentException($"Expected 1 to 65536 iterations per step, at most {MaxSteps} steps, 1 to {MaxGoals} goals and at most {MaxCells} cells.");
        }

        if (kind == 1 && options.Algorithm is not (GridAlgorithm.AStar or GridAlgorithm.Jps or GridAlgorithm.JpsPlus))
        {
            throw new ArgumentException($"A search run uses A*, JPS or JPS+, not {options.Algorithm}.", nameof(options));
        }

        foreach (Cell c in points)
        {
            if (!grid.Contains(c.X, c.Y))
            {
                throw new ArgumentException($"Cell ({c.X}, {c.Y}) is outside the {grid.Width} x {grid.Height} grid.");
            }
        }

        byte[] state = grid.SaveState();
        var w = new ArrayBufferWriter<byte>(64 + state.Length);
        Span<byte> h = w.GetSpan(20);
        Magic.CopyTo(h);
        h[4] = 1;
        h[5] = kind;
        h[6] = 0;
        h[7] = 0;
        BinaryPrimitives.WriteInt32LittleEndian(h[8..], slice);
        h[12] = (byte)options.Algorithm;
        h[13] = (byte)options.Connectivity;
        h[14] = (byte)options.Diagonals;
        h[15] = 0;
        BinaryPrimitives.WriteInt32LittleEndian(h[16..], options.AgentSize);
        w.Advance(20);
        Span<byte> g = w.GetSpan(4 + (points.Length * 8));
        BinaryPrimitives.WriteInt32LittleEndian(g, points.Length);
        for (int i = 0; i < points.Length; i++)
        {
            BinaryPrimitives.WriteInt32LittleEndian(g[(4 + (i * 8))..], points[i].X);
            BinaryPrimitives.WriteInt32LittleEndian(g[(8 + (i * 8))..], points[i].Y);
        }

        w.Advance(4 + (points.Length * 8));
        BinaryPrimitives.WriteInt32LittleEndian(w.GetSpan(4), state.Length);
        w.Advance(4);
        w.Write(state);
        BinaryPrimitives.WriteInt32LittleEndian(w.GetSpan(4), steps.Count);
        w.Advance(4);
        var ops = new ArrayBufferWriter<byte>();
        foreach (IReadOnlyList<GridRunOp> step in steps)
        {
            ops.ResetWrittenCount();
            foreach (GridRunOp op in step)
            {
                if (!grid.Contains(op.X, op.Y) || op.Kind is < GridRunOpKind.SetArea or > GridRunOpKind.SetGoal || (kind == 2 && op.Kind == GridRunOpKind.SetStart))
                {
                    throw new ArgumentException($"Edit {op} is not valid for this run and grid.");
                }

                Encode(op, ops);
            }

            if (step.Count > MaxOpsPerStep)
            {
                throw new ArgumentException($"A step may hold at most {MaxOpsPerStep} edits.");
            }

            BinaryPrimitives.WriteInt32LittleEndian(w.GetSpan(4), ops.WrittenCount);
            w.Advance(4);
            w.Write(ops.WrittenSpan);
        }

        into.Write(w.WrittenSpan);
    }

    internal static void Encode(in GridRunOp op, IBufferWriter<byte> into)
    {
        Span<byte> d = into.GetSpan(OpBytes);
        d[0] = (byte)op.Kind;
        BinaryPrimitives.WriteInt32LittleEndian(d[1..], op.X);
        BinaryPrimitives.WriteInt32LittleEndian(d[5..], op.Y);
        d[9] = op.Area;
        into.Advance(OpBytes);
    }

    internal sealed record Parsed(byte Kind, int Slice, GridAlgorithm Algorithm, Connectivity Connectivity, DiagonalRule Diagonals, int AgentSize, Cell[] Points, byte[] GridState, int SetupLength, byte[][] Steps);

    internal static Parsed Read(ReadOnlySpan<byte> bytes, string name)
    {
        ScopeFormatException Fail(string what) => new(name, what);
        if (bytes.Length < 24 || !bytes.StartsWith(Magic) || bytes[4] != 1 || bytes[6] != 0 || bytes[7] != 0 || bytes[5] is not (1 or 2) || bytes[15] != 0)
        {
            throw Fail("expected a OnePath grid run (bytes 'OPGR', version 1, kind 1 or 2, reserved bytes 0) of at least 24 bytes.");
        }

        byte kind = bytes[5];
        int slice = BinaryPrimitives.ReadInt32LittleEndian(bytes[8..]);
        int algorithm = bytes[12], conn = bytes[13], diag = bytes[14];
        int agent = BinaryPrimitives.ReadInt32LittleEndian(bytes[16..]);
        if (slice is < 1 or > 65536 || (kind == 1 && algorithm is not (int)GridAlgorithm.AStar and not (int)GridAlgorithm.Jps and not (int)GridAlgorithm.JpsPlus)
            || conn is not (4 or 8) || diag > 2 || agent is < 1 or > 255)
        {
            throw Fail($"expected 1 to 65536 iterations per step, algorithm A*/JPS/JPS+, connectivity 4 or 8, diagonal rule 0 to 2 and agent size 1 to 255 in bytes 8 to 19; got {slice}, {algorithm}, {conn}, {diag}, {agent}.");
        }

        int n = 20;
        int points = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
        n += 4;
        if (points < (kind == 1 ? 2 : 1) || points > (kind == 1 ? 2 : MaxGoals) || bytes.Length - n < (points * 8) + 8)
        {
            throw Fail($"expected {(kind == 1 ? "2 points" : $"1 to {MaxGoals} goals")} that fit in the bytes, got {points} at byte 20.");
        }

        var cells = new Cell[points];
        for (int i = 0; i < points; i++)
        {
            cells[i] = new Cell(BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]), BinaryPrimitives.ReadInt32LittleEndian(bytes[(n + 4)..]));
            n += 8;
        }

        int gridLength = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
        n += 4;
        if (gridLength < 0 || gridLength > bytes.Length - n - 4 || gridLength > (64 << 20))
        {
            throw Fail($"expected a grid state of at most 64 MiB that fits in the bytes, got {gridLength} at byte {n - 4}.");
        }

        byte[] gridState = bytes.Slice(n, gridLength).ToArray();
        n += gridLength;
        int setupLength = n;
        int stepCount = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
        n += 4;
        // Each step takes at least 4 bytes.
        if (stepCount < 0 || stepCount > MaxSteps || (long)stepCount * 4 > bytes.Length - n)
        {
            throw Fail($"expected 0 to {MaxSteps} steps that fit in the bytes, got {stepCount} at byte {n - 4}.");
        }

        var steps = new byte[stepCount][];
        for (int i = 0; i < stepCount; i++)
        {
            if (bytes.Length - n < 4)
            {
                throw Fail($"expected the length of step {i} at byte {n}.");
            }

            int length = BinaryPrimitives.ReadInt32LittleEndian(bytes[n..]);
            n += 4;
            if (length < 0 || length % OpBytes != 0 || length > MaxOpsPerStep * OpBytes || length > bytes.Length - n)
            {
                throw Fail($"expected step {i} to hold a whole number of {OpBytes}-byte edits that fit in the bytes, got {length} bytes at byte {n - 4}.");
            }

            steps[i] = bytes.Slice(n, length).ToArray();
            n += length;
        }

        if (n != bytes.Length)
        {
            throw Fail($"expected the run to end at byte {n}, found {bytes.Length - n} more bytes.");
        }

        return new Parsed(kind, slice, (GridAlgorithm)algorithm, (Connectivity)conn, (DiagonalRule)diag, agent, cells, gridState, setupLength, steps);
    }
}
