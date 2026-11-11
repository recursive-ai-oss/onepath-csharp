using System;

namespace RecursiveAI.OnePath.Replay;

/// <summary>
/// Crowd replay bytes (a file, a setup block, one step's operations or a live record) are malformed, or an operation cannot apply to the crowd it
/// meets. The message names what was wrong and the byte position or step.
/// </summary>
public sealed class CrowdReplayException : OnePathFormatException
{
    /// <summary>Creates the exception with a default message.</summary>
    public CrowdReplayException()
        : base("The crowd replay is malformed.")
    {
    }

    /// <summary>Creates the exception with a message.</summary>
    public CrowdReplayException(string message)
        : base(message)
    {
    }

    /// <summary>Creates the exception with a message and the error that caused it.</summary>
    public CrowdReplayException(string message, Exception? innerException)
        : base(message, innerException)
    {
    }
}
