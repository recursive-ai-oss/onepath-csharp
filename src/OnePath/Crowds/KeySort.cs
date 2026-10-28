using System;
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.Crowds;

/// <summary>
/// Sorts the neighbour-grid keys. The keys are all different (the low bits hold the agent index), so any correct sort gives the
/// same order. It is our own introsort rather than <see cref="Array.Sort{T}(T[], int, int)"/> because the framework's generic
/// instantiation for <see cref="long"/> starts in unoptimized code in every process, which costs a small crowd much of its first
/// hundred steps.
/// </summary>
internal static class KeySort
{
    private const int InsertionLimit = 16;

    /// <summary>Sorts <c>keys[0..count)</c> ascending.</summary>
    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    public static void Sort(long[] keys, int count)
    {
        if (count < 2)
        {
            return;
        }
        if ((uint)count > (uint)keys.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(count));
        }
        int depth = 2 * (32 - System.Numerics.BitOperations.LeadingZeroCount((uint)count));
        Quick(keys, 0, count - 1, depth);
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void Quick(long[] a, int lo, int hi, int depth)
    {
        // Recurse into the smaller part and loop on the larger: stack depth is at most log2(n).
        while (hi - lo + 1 > InsertionLimit)
        {
            if (depth-- == 0)
            {
                Heap(a, lo, hi - lo + 1);
                return;
            }
            int mid = lo + ((hi - lo) >> 1);
            if (a[mid] < a[lo]) { (a[mid], a[lo]) = (a[lo], a[mid]); }
            if (a[hi] < a[lo]) { (a[hi], a[lo]) = (a[lo], a[hi]); }
            if (a[hi] < a[mid]) { (a[hi], a[mid]) = (a[mid], a[hi]); }
            long pivot = a[mid];
            int i = lo, j = hi;
            while (i <= j)
            {
                while (a[i] < pivot) { i++; }
                while (a[j] > pivot) { j--; }
                if (i <= j)
                {
                    (a[i], a[j]) = (a[j], a[i]);
                    i++;
                    j--;
                }
            }
            if (j - lo < hi - i)
            {
                Quick(a, lo, j, depth);
                lo = i;
            }
            else
            {
                Quick(a, i, hi, depth);
                hi = j;
            }
        }
        for (int i = lo + 1; i <= hi; i++)
        {
            long v = a[i];
            int j = i - 1;
            while (j >= lo && a[j] > v)
            {
                a[j + 1] = a[j];
                j--;
            }
            a[j + 1] = v;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    private static void Heap(long[] a, int start, int n)
    {
        for (int i = (n >> 1) - 1; i >= 0; i--)
        {
            Down(a, start, i, n);
        }
        for (int end = n - 1; end > 0; end--)
        {
            (a[start], a[start + end]) = (a[start + end], a[start]);
            Down(a, start, 0, end);
        }
    }

    private static void Down(long[] a, int start, int i, int n)
    {
        while (true)
        {
            int c = (2 * i) + 1;
            if (c >= n)
            {
                return;
            }
            if (c + 1 < n && a[start + c + 1] > a[start + c])
            {
                c++;
            }
            if (a[start + i] >= a[start + c])
            {
                return;
            }
            (a[start + i], a[start + c]) = (a[start + c], a[start + i]);
            i = c;
        }
    }
}
