using System;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.NavMesh.Build3D;

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

return ok && orca && OnePathInfo.Version.Major >= 0 ? 0 : 1;
