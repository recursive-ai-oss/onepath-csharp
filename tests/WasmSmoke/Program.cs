using System;
using RecursiveAI.OnePath;

// Exercises the public surface under NativeAOT and full trimming. Exit code 0 means it ran.
Console.WriteLine($"OnePath {OnePathInfo.Version}");
Console.WriteLine($"OneMath {OnePathInfo.OneMathVersion}");
bool ok = OnePathInfo.FixedToDouble(3L << 31) == 1.5;
try
{
    OnePathInfo.FixedToDouble(long.MaxValue);
    ok = false;
}
catch (ArgumentOutOfRangeException)
{
}
bool nav = NavMeshSmoke.Run();
Console.WriteLine($"Navmesh format: {(nav ? "ok" : "FAILED")}");
bool orca = OrcaSmoke.Run();
Console.WriteLine($"ORCA run: {(orca ? "ok" : "FAILED")}");
bool grids = GridSmoke.Run();
Console.WriteLine($"Grids run: {(grids ? "ok" : "FAILED")}");
bool corridors = CorridorSmoke.Run();
Console.WriteLine($"Corridor follow: {(corridors ? "ok" : "FAILED")}");
return ok && orca && nav && grids && corridors && OnePathInfo.Version.Major >= 0 ? 0 : 1;
