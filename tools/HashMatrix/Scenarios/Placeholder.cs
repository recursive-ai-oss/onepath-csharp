using System;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;
using System.Threading.Tasks;

namespace RecursiveAI.OnePath.HashMatrix;

/// <summary>
/// Placeholder scenarios that make the columns meaningful before real parts exist: a float-heavy
/// computation using only IEEE-exact operations (+ - * / sqrt floor), run by chunk so the thread
/// count can vary, and a Vector128 loop checked against its scalar twin.
/// </summary>
internal static class PlaceholderScenarios
{
    private const int Chunks = 64;
    private const int Steps = 4096;

    [ModuleInitializer]
    internal static void Init()
    {
        Scenarios.Register("meta/float-chunks", FloatChunks);
        Scenarios.Register("meta/simd-vs-scalar", SimdVsScalar);
    }

    // Each chunk is an independent iteration of a chaotic map mixing float32, double, division and
    // sqrt. Chunks run on ctx.Threads workers and are written in chunk order, so the bytes must not
    // depend on the thread count.
    private static void FloatChunks(ScenarioContext ctx)
    {
        var results = new (float F, double D)[Chunks];
        var options = new ParallelOptions { MaxDegreeOfParallelism = Math.Max(1, ctx.Threads) };
        Parallel.For(0, Chunks, options, i => results[i] = RunChunk(i));
        foreach (var (f, d) in results)
        {
            ctx.Write(f);
            ctx.Write(d);
        }
    }

    private static (float, double) RunChunk(int chunk)
    {
        float x = 0.1f + (chunk * 0.013f);
        double y = 0.3 + (chunk * 0.00731);
        for (int s = 0; s < Steps; s++)
        {
            x = 3.9f * x * (1f - x);
            y = (y * y) + (1.0 / (2.0 + y)) - (0.25 * Math.Sqrt(y + 1.0));
            y -= Math.Floor(y);
            x += (float)(y * 1e-3);
            x -= MathF.Floor(x);
        }
        return (x, y);
    }

    // Vector128 multiply and add as separate operations (no fused multiply-add), against the scalar loop.
    private static void SimdVsScalar(ScenarioContext ctx)
    {
        const int N = 1003; // not a multiple of 4: exercises the scalar tail
        var a = new float[N];
        var b = new float[N];
        for (int i = 0; i < N; i++)
        {
            a[i] = 1f / (i + 1);
            b[i] = (i * 0.37f) - 100f;
        }
        var vec = new float[N];
        int j = 0;
        for (; j + 4 <= N; j += 4)
        {
            var va = Vector128.LoadUnsafe(ref a[j]);
            var vb = Vector128.LoadUnsafe(ref b[j]);
            (Vector128.Multiply(va, vb) + va).StoreUnsafe(ref vec[j]);
        }
        for (; j < N; j++)
        {
            vec[j] = (a[j] * b[j]) + a[j];
        }
        var scalar = new float[N];
        for (int i = 0; i < N; i++)
        {
            scalar[i] = (a[i] * b[i]) + a[i];
        }
        // Both are written: a SIMD/scalar mismatch also shows as a different hash between columns.
        ctx.Write(vec.AsSpan().SequenceEqual(scalar) ? 1 : 0);
        for (int i = 0; i < N; i++)
        {
            ctx.Write(vec[i]);
            ctx.Write(scalar[i]);
        }
    }
}
