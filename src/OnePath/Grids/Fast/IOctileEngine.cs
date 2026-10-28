using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePath.Grids;

/// <summary>What the one-line queries read from a finished octile search.</summary>
internal interface IOctileEngine
{
    int ExpandedCount { get; }

    List<int> Expanded { get; }

    double PathCost { get; }

    int PathLength();

    void CopyPath(Span<int> destination);
}

/// <summary>A jump point search that can be begun, stepped and saved (<see cref="GridSearch"/>'s specialised engines).</summary>
internal interface IOctileJumpEngine : IOctileEngine
{
    SearchStatus Status { get; }

    int Iterations { get; }

    int Start { get; }

    int Goal { get; }

    void Begin(int start, int goal, bool zeroEstimate);

    SearchStatus Step(int budget);

    int StateLength();

    int WriteState(Span<byte> destination);

    void ReadState(ref StateReader r, byte[] cells);
}
