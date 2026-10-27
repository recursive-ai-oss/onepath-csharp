using System;
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Mesh.Build3D;
using RecursiveAI.OnePath.Mesh.Build2D;

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

// The 3D builder's span stage: a floor as triangles and as a voxel chunk, through the pipeline.
var frame = new HeightfieldFrame(0, 0, 0, 1f, 1f, 12, 12, 8);
double[] floor = { 0, 2, 0, 0, 2, 12, 12, 2, 12, 12, 2, 0 };
int[] quad = { 0, 1, 2, 0, 2, 3 };
var settings = new SpanSettings { WalkableHeight = 2, WalkableClimb = 1, WalkableRadius = 1 };
var fromSoup = new SpanPipeline(settings).BuildFromTriangles(frame, floor, quad);
var dense = new byte[12 * 12 * 8];
for (int c = 0; c < 144; c++)
{
    dense[c * 8] = 1;
    dense[(c * 8) + 1] = 1;
}
var fromChunk = new SpanPipeline(settings).BuildFromChunk(frame, VoxelChunk.FromDense(12, 12, 8, dense));
int walkable = 0;
for (int i = 0; i < fromSoup.SpanCount; i++)
{
    walkable += fromSoup.Spans[i].Area != 0 ? 1 : 0;
}
ok &= fromSoup.SpanCount == 100 && fromChunk.SpanCount == 100 && walkable == 64;
try
{
    Voxelizer.Rasterize(new SolidHeightfield(frame), new[] { 0, 0, double.NaN, 1, 0, 0, 0, 0, 1 }, quad.AsSpan(0, 3), settings);
    ok = false;
}
catch (ArgumentException)
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
bool grids = GridSmoke.Run();
Console.WriteLine($"Grids run: {(grids ? "ok" : "FAILED")}");
return ok && orca && nav && grids && OnePathInfo.Version.Major >= 0 ? 0 : 1;
