using System;

namespace RecursiveAI.OnePath;

/// <summary>
/// The base of every error caused by data: bytes or text that are corrupt, truncated, foreign or from an unsupported version
/// (saved meshes, grid and crowd state, glTF files, map files, landmark tables), and geometry a builder cannot turn into a mesh.
/// A host that loads untrusted saves, mods or network state catches this one type. Each subclass names its own subject, and
/// the message says what was wrong and where. Misuse of the API (a bad argument, a call in the wrong order) is never reported
/// with this type: it throws <see cref="ArgumentException"/> or <see cref="InvalidOperationException"/>.
/// </summary>
public class OnePathFormatException : FormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public OnePathFormatException()
        : base("The data is malformed.")
    {
    }

    /// <summary>Creates the exception with a message.</summary>
    /// <param name="message">What was wrong and where.</param>
    public OnePathFormatException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and a cause.</summary>
    /// <param name="message">What was wrong and where.</param>
    /// <param name="innerException">The cause.</param>
    public OnePathFormatException(string message, Exception? innerException)
        : base(message, innerException)
    {
    }
}

/// <summary>Thrown when serialized graph state or landmark data is malformed, of another kind or version, or was built for a different graph.</summary>
public sealed class GraphStateException : OnePathFormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public GraphStateException()
        : base("The graph data is malformed.")
    {
    }

    /// <summary>Creates the exception.</summary>
    /// <param name="message">What was wrong and where.</param>
    public GraphStateException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a cause.</summary>
    /// <param name="message">What was wrong and where.</param>
    /// <param name="innerException">The cause.</param>
    public GraphStateException(string message, Exception? innerException)
        : base(message, innerException)
    {
    }
}

/// <summary>
/// Thrown when crowd, path corridor or path follower state cannot be restored: the bytes are truncated, were written by another
/// kind of object or version, or hold impossible values. The message names the byte offset and what was wrong.
/// </summary>
public sealed class CrowdStateException : OnePathFormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public CrowdStateException()
        : base("The crowd state is malformed.")
    {
    }

    /// <summary>Creates the exception.</summary>
    /// <param name="message">What was wrong and where.</param>
    public CrowdStateException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a cause.</summary>
    /// <param name="message">What was wrong and where.</param>
    /// <param name="innerException">The cause.</param>
    public CrowdStateException(string message, Exception? innerException)
        : base(message, innerException)
    {
    }
}
