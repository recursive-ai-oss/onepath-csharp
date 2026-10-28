using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;

namespace RecursiveAI.OnePath;

/// <summary>Thrown when a map or scenario file does not follow its format; the message says what was wrong and where.</summary>
public sealed class GridFormatException : OnePathFormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public GridFormatException()
    {
    }

    /// <summary>Creates the exception with a message.</summary>
    /// <param name="message">What was wrong.</param>
    public GridFormatException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and an inner exception.</summary>
    /// <param name="message">What was wrong.</param>
    /// <param name="innerException">The cause.</param>
    public GridFormatException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    internal GridFormatException(int line, string message)
        : base($"line {line}: {message}")
    {
        Line = line;
    }

    /// <summary>The 1-based line of the problem, or 0 when not tied to a line.</summary>
    public int Line { get; }
}
