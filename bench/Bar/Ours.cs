using System.Collections.Generic;

namespace RecursiveAI.OnePath.Bar;

/// <summary>
/// Our side of a bar row. Each part registers the scenarios it can run here as it lands; a suite row with no
/// registered scenario is reported as "no ours yet" next to the reference's numbers.
/// </summary>
internal static class OurScenarios
{
    /// <summary>The side name our rows carry.</summary>
    public static string Side => "onepath-" + OnePathInfo.Version;

    /// <summary>
    /// Runs our implementation of a suite row once, returning the same metrics the reference driver prints, or
    /// <c>null</c> when we have nothing for that row yet. The scenario of every returned row is the suite row id.
    /// </summary>
    public static List<Row>? Run(SuiteRow row, Paths paths)
    {
        _ = row;
        _ = paths;
        return null; // nothing to measure yet
    }
}
