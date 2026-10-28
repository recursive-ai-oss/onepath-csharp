using System;
using System.Collections;
using System.Collections.Generic;

namespace RecursiveAI.OnePath;

/// <summary>
/// A read-only list that owns a private copy of the elements it was made from, so later edits to the caller's array or list never
/// show through, and that compares by element so options records holding one compare by value.
/// </summary>
internal sealed class FrozenList<T> : IReadOnlyList<T>, IEquatable<FrozenList<T>>
{
    private static readonly FrozenList<T> s_empty = new([]);

    private readonly T[] _items;

    private FrozenList(T[] items) => _items = items;

    internal static FrozenList<T> Empty => s_empty;

    /// <summary>Copies <paramref name="source"/>; null means empty. Returns the same instance when it is already frozen.</summary>
    internal static FrozenList<T> Of(IEnumerable<T>? source)
    {
        if (source is FrozenList<T> f)
        {
            return f;
        }
        if (source is null)
        {
            return s_empty;
        }
        T[] items = [.. source];
        return items.Length == 0 ? s_empty : new FrozenList<T>(items);
    }

    public int Count => _items.Length;

    public T this[int index] => _items[index];

    public bool Equals(FrozenList<T>? other)
    {
        if (other is null || other._items.Length != _items.Length)
        {
            return false;
        }
        for (int i = 0; i < _items.Length; i++)
        {
            if (!EqualityComparer<T>.Default.Equals(_items[i], other._items[i]))
            {
                return false;
            }
        }
        return true;
    }

    public override bool Equals(object? obj) => obj is FrozenList<T> o && Equals(o);

    // Equal lists need equal hashes; the count is enough and keeps the hash independent of element hash codes.
    public override int GetHashCode() => _items.Length;

    public IEnumerator<T> GetEnumerator() => ((IEnumerable<T>)_items).GetEnumerator();

    IEnumerator IEnumerable.GetEnumerator() => _items.GetEnumerator();
}
