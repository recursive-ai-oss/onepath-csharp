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
