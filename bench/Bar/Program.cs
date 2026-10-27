using System;
using RecursiveAI.OnePath;

// Bar runner: one row per scenario (time, memory, quality against the reference), as CSV.
// Scenarios register here as the parts they measure arrive.
Console.WriteLine("part,scenario,ours_ms,reference_ms,ratio,notes");
Console.Error.WriteLine($"OnePath {OnePathInfo.Version}: no scenarios yet");
return 0;
