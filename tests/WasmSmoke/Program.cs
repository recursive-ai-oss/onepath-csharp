using System;
using RecursiveAI.OnePath;

// Exercises the public surface under NativeAOT and full trimming. Exit code 0 means it ran.
Console.WriteLine($"OnePath {OnePathInfo.Version}");
return OnePathInfo.Version.Major >= 0 ? 0 : 1;
