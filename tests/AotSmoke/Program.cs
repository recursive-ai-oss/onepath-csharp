using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.NavMesh.Build2D;

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
// 2D geometry pipeline: outlines -> boolean -> offset -> triangulation -> convex polygons.
var a = Region.FromOutlines(new[] { new Outline(new[] { new Vector2D(0, 0), new Vector2D(10, 0), new Vector2D(10, 10), new Vector2D(0, 10) }) });
var b = Region.FromOutlines(new[] { new Outline(new[] { new Vector2D(5, 5), new Vector2D(15, 5), new Vector2D(15, 15), new Vector2D(5, 15) }) });
var u = a.Union(b);
var shrunk = u.Offset(-1.0);
var poly = shrunk.PartitionConvex();
Console.WriteLine($"Region area {u.Area}, eroded area {shrunk.Area:F3}, {poly.PolygonCount} convex polygons");
ok = ok && u.Area == 175.0 && poly.PolygonCount > 0 && shrunk.Area < u.Area;
return ok && OnePathInfo.Version.Major >= 0 ? 0 : 1;
