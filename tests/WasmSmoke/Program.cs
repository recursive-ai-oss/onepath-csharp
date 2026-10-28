using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;

// Exercises the public surface under NativeAOT and full trimming. Exit code 0 means it ran.
Console.WriteLine($"OnePath {OnePathInfo.Version}");
Console.WriteLine($"OneMath {OnePathInfo.OneMathVersion}");
bool ok = FixedBridge.ToDouble(3L << 31) == 1.5;
try
{
    FixedBridge.ToDouble(long.MaxValue);
    ok = false;
}
catch (ArgumentOutOfRangeException)
{
}
bool nav = NavMeshSmoke.Run();
Console.WriteLine($"Navmesh format: {(nav ? "ok" : "FAILED")}");
bool orca = OrcaSmoke.Run();
Console.WriteLine($"ORCA run: {(orca ? "ok" : "FAILED")}");
bool tileCache = TileCacheSmoke.Run();
Console.WriteLine($"Tile cache (obstacles, volumes, state): {(tileCache ? "ok" : "FAILED")}");
bool world = WorldSmoke.Run();
Console.WriteLine($"World (links, classes, streaming, glTF): {(world ? "ok" : "FAILED")}");
bool grids = GridSmoke.Run();
Console.WriteLine($"Grids run: {(grids ? "ok" : "FAILED")}");
bool corridors = CorridorSmoke.Run();
Console.WriteLine($"Corridor follow: {(corridors ? "ok" : "FAILED")}");
bool state = StateSmoke.Run();
Console.WriteLine($"Grid state run: {(state ? "ok" : "FAILED")}");
bool crowds = CrowdSmoke.Run();
Console.WriteLine($"Crowd run: {(crowds ? "ok" : "FAILED")}");
return ok && world && tileCache && orca && nav && grids && corridors && state && crowds && OnePathInfo.Version.Major >= 0 ? 0 : 1;
