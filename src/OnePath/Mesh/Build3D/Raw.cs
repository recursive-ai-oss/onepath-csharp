using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace RecursiveAI.OnePath.Mesh.Build3D;

/// <summary>
/// Element access without a bounds check for the few loops of the build stages that profiling showed to be bound by it. Every call site states why
/// its index is in range; under the <c>Checked</c> configuration the access is an ordinary checked one, and the tests and the fuzz targets run there.
/// </summary>
internal static class Raw
{
    /// <summary>Reference to <paramref name="array"/>[<paramref name="index"/>]. The caller guarantees <c>0 &lt;= index &lt; array.Length</c>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref T At<T>(T[] array, int index)
    {
#if CHECKED
        return ref array[index];
#else
        return ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(array), index);
#endif
    }
}
