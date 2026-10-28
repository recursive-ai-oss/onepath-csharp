using System;

namespace RecursiveAI.OnePath;

/// <summary>Init-time checks for options: a bad value throws at the property that was set, naming it and the allowed range.</summary>
internal static class Require
{
    internal static T That<T>(bool ok, T value, string property, string rule) =>
        ok ? value : throw new ArgumentOutOfRangeException(property, value, $"{property} must be {rule}.");

    internal static bool Finite(double v) => double.IsFinite(v);
}
