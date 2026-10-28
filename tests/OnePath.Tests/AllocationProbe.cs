using System;
using System.Threading;

namespace RecursiveAI.OnePath.Tests;

/// <summary>
/// Measures what a piece of work allocates on the calling thread once it is in steady state.
/// </summary>
/// <remarks>
/// Tiered compilation promotes hot methods on a background timer that restarts whenever new code is
/// being compiled, so on a busy machine running the whole suite in parallel the measured loop can still
/// be executing tier-0 code (or code that was just swapped) after any fixed number of warm-up rounds.
/// A real per-call allocation shows up in every window; a one-off runtime cost does not. So the work is
/// measured in several consecutive windows, pausing between windows to let tier-up finish, and the
/// smallest window is the steady-state figure.
/// </remarks>
internal static class AllocationProbe
{
    public static long SteadyState(Action work, int attempts = 10)
    {
        return SteadyState(() =>
        {
            work();
            return true;
        }, attempts);
    }

    /// <summary>
    /// As above, for work that can run dry (agents arriving, queues emptying): a window whose work returns
    /// false did too little to count, so an idle window can't pass the check trivially. Returns
    /// <see cref="long.MaxValue"/> when no window did enough. <paramref name="prepare"/> runs before each
    /// window, unmeasured, to give the work something to do again.
    /// </summary>
    public static long SteadyState(Func<bool> work, int attempts = 10, Action? prepare = null)
    {
        long best = long.MaxValue;
        for (int a = 0; a < attempts; a++)
        {
            prepare?.Invoke();
            long before = GC.GetAllocatedBytesForCurrentThread();
            bool counted = work();
            long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
            if (counted && allocated < best)
            {
                best = allocated;
            }

            if (best == 0)
            {
                break;
            }

            Thread.Sleep(200);
        }

        return best;
    }
}
