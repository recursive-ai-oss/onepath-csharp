using System;
using System.Threading;

namespace RecursiveAI.OnePath.Grids;

// Runs a resumable search in 256-iteration slices so a cancellation token is observed between them. Any sequence of
// budgets gives the result of one unsliced run, so a token that is never cancelled changes nothing; callers skip the
// slicing altogether when the token cannot be cancelled.
internal static class SearchDrive
{
    internal const int Slice = 256;

    internal static SearchStatus Run(int budget, Func<int, SearchStatus> step, CancellationToken token)
    {
        int remaining = budget;
        SearchStatus s = SearchStatus.Running;
        while (s == SearchStatus.Running && remaining > 0)
        {
            token.ThrowIfCancellationRequested();
            int slice = Math.Min(remaining, Slice);
            s = step(slice);
            remaining -= slice;
        }
        return s;
    }
}
