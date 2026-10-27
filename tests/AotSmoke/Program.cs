using System;
using RecursiveAI.OnePath;

// Exercises the public surface under NativeAOT and full trimming. Exit code 0 means it ran.
Console.WriteLine($"OnePath {OnePathInfo.Version}");
bool grids = GridSmoke.Run();
Console.WriteLine($"Grids run: {(grids ? "ok" : "FAILED")}");
return grids && OnePathInfo.Version.Major >= 0 ? 0 : 1;
