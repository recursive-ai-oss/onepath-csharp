using System;
using System.Collections.Generic;
using System.Threading;

namespace RecursiveAI.OnePath;

/// <summary>Where a (possibly sliced) flow-field computation stands.</summary>
public enum FlowFieldStatus
{
    /// <summary>No computation has begun.</summary>
    NotStarted = 0,

    /// <summary>Work is left; call <c>Step</c> again.</summary>
    Running = 1,

    /// <summary>The integration field and the direction field are complete.</summary>
    Complete = 2,
}
