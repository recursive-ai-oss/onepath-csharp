using System;
using System.Collections.Generic;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath.Mesh;
using RecursiveAI.OnePath.Mesh.Queries;

namespace RecursiveAI.OnePath;

/// <summary>One point of a straight (string-pulled) path.</summary>
public readonly struct StraightPathPoint : IEquatable<StraightPathPoint>
{
    internal StraightPathPoint(Vector3D position, StraightPathMarks flags, PolyRef poly)
    {
        Position = position;
        Flags = flags;
        Poly = poly;
    }

    /// <summary>World position.</summary>
    public Vector3D Position { get; }

    /// <summary>Start, end and off-mesh markers.</summary>
    public StraightPathMarks Flags { get; }

    /// <summary>The corridor polygon the path is in after this point (for the last point, the end polygon).</summary>
    public PolyRef Poly { get; }

    /// <inheritdoc />
    public bool Equals(StraightPathPoint other) => Position == other.Position && Flags == other.Flags && Poly == other.Poly;

    /// <inheritdoc />
    public override bool Equals(object? obj) => obj is StraightPathPoint p && Equals(p);

    /// <inheritdoc />
    public override int GetHashCode() => (int)Poly.Value;

    /// <summary>Equality.</summary>
    public static bool operator ==(StraightPathPoint a, StraightPathPoint b) => a.Equals(b);

    /// <summary>Inequality.</summary>
    public static bool operator !=(StraightPathPoint a, StraightPathPoint b) => !a.Equals(b);
}
