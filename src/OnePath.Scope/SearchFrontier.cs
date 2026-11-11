using System;
using System.Buffers.Binary;

namespace RecursiveAI.OnePath.Scope;

/// <summary>
/// Reads the open and closed cells out of a <see cref="GridSearch"/>'s saved state (the documented byte layout in docs/grids-state.md)
/// for drawing. The state is the library's own output, but it is still parsed with explicit length checks; anything unexpected means
/// "no frontier available", never an exception.
/// </summary>
internal static class SearchFrontier
{
    public const byte Unseen = 0, Open = 1, Closed = 2;

    /// <summary>
    /// Fills <paramref name="cells"/> (width x height entries) with Unseen/Open/Closed and returns true, or returns false when the state
    /// is not in a form this reader knows (the generic engines other than A*).
    /// </summary>
    public static bool TryRead(ReadOnlySpan<byte> state, int width, int height, byte[] cells, out int open, out int closed)
    {
        open = closed = 0;
        Array.Clear(cells);
        if (state.Length < 17 || state[8] == 0)
        {
            return state.Length >= 17 && state[8] == 0;
        }

        byte mode = state[8];
        int n = 17;
        long total = (long)width * height;
        if (mode == 1 || mode == 3)
        {
            if (mode == 3)
            {
                n += 1;
            }

            n += 1 + 1 + 4 * 6; // status, zeroEstimate, start, goal, expanded, iterations, pathA, pathB
            if (state.Length < n + 4)
            {
                return false;
            }

            int visited = BinaryPrimitives.ReadInt32LittleEndian(state[n..]);
            n += 4;
            int entry = mode == 1 ? 13 : 17;
            if (visited < 0 || (long)visited * entry > state.Length - n)
            {
                return false;
            }

            for (int i = 0; i < visited; i++)
            {
                int cell = BinaryPrimitives.ReadInt32LittleEndian(state[n..]);
                byte flags = state[n + entry - 1];
                n += entry;
                if ((uint)cell >= (uint)total)
                {
                    return false;
                }

                cells[cell] = (flags & 1) != 0 ? Closed : Open;
                if ((flags & 1) != 0)
                {
                    closed++;
                }
                else
                {
                    open++;
                }
            }

            return true;
        }

        return false;
    }
}
