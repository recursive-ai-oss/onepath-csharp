using System;
using System.Collections.Generic;
using RecursiveAI.OnePath.Mesh.Queries;
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

// Regions and contours: the eroded floor is one 8 x 8 square under every partition.
var floorChunk = VoxelChunk.FromDense(12, 12, 8, dense);
foreach (PartitionType type in Enum.GetValues<PartitionType>())
{
    var rp = new RegionPipeline(settings, new RegionSettings { Partition = type, MinRegionArea = 0, MergeRegionArea = 0 }, new ContourSettings { MaxEdgeLength = 0 });
    ContourSet set = rp.BuildFromChunk(frame, floorChunk);
    ok &= set.Contours.Count == 1 && set.Contours[0].Vertices.Length == 4;
}


// 2D geometry pipeline: outlines -> boolean -> offset -> triangulation -> convex polygons.
var a = Region.FromOutlines(new[] { new Outline(new[] { new Vector2D(0, 0), new Vector2D(10, 0), new Vector2D(10, 10), new Vector2D(0, 10) }) });
var b = Region.FromOutlines(new[] { new Outline(new[] { new Vector2D(5, 5), new Vector2D(15, 5), new Vector2D(15, 15), new Vector2D(5, 15) }) });
var u = a.Union(b);
var shrunk = u.Offset(-1.0);
var poly = shrunk.PartitionConvex();
Console.WriteLine($"Region area {u.Area}, eroded area {shrunk.Area:F3}, {poly.PolygonCount} convex polygons");
ok = ok && u.Area == 175.0 && poly.PolygonCount > 0 && shrunk.Area < u.Area;
// Tier 1: outlines -> tiled navigation mesh -> path.
var navMesh = NavMesh.Build2D(new[] { new Outline(new[] { new Vector2D(0, 0), new Vector2D(100, 0), new Vector2D(100, 50), new Vector2D(0, 50) }) }, NavSettings.Humanoid with { TileSize = 32 });
Console.WriteLine($"NavMesh.Build2D: {navMesh.Tiles.TileCount} tiles");
ok = ok && navMesh.Tiles.TileCount == 8 && RecursiveAI.OnePath.Mesh.NavMeshValidator.Validate(navMesh.Tiles).Count == 0;
var navPath = navMesh.FindPath(new Vector3D(5, 0, 5), new Vector3D(95, 0, 45));
Console.WriteLine($"NavMesh.FindPath: {navPath.Status}, {navPath.Points.Count} points");
ok = ok && navPath.Points.Count >= 2;
// Tier 1, 3D: triangle soup -> tiled mesh with height detail -> path.
var levelTris = new List<Triangle3D>
{
    new(new Vector3D(0, 0, 0), new Vector3D(0, 0, 40), new Vector3D(40, 0, 40)),
    new(new Vector3D(0, 0, 0), new Vector3D(40, 0, 40), new Vector3D(40, 0, 0)),
};
NavMesh level = NavMesh.Build(levelTris, NavSettings.Humanoid with { TileSize = 16 });
PathResult levelPath = level.FindPath(new Vector3D(3, 0, 3), new Vector3D(37, 0, 37));
Console.WriteLine($"NavMesh.Build (3D): {level.Tiles.TileCount} tiles, path {levelPath.Status}");
ok = ok && level.Tiles.TileCount == 9 && levelPath.Status == PathStatus.Found && RecursiveAI.OnePath.Mesh.NavMeshValidator.Validate(level.Tiles).Count == 0;
bool tileCache = TileCacheSmoke.Run();
Console.WriteLine($"Tile cache (obstacles, volumes, state): {(tileCache ? "ok" : "FAILED")}");
bool grids = GridSmoke.Run();
Console.WriteLine($"Grids run: {(grids ? "ok" : "FAILED")}");
bool flow = FlowSmoke.Run();
Console.WriteLine($"Flow fields run: {(flow ? "ok" : "FAILED")}");
bool graphs = GraphSmoke.Run();
Console.WriteLine($"Graphs run: {(graphs ? "ok" : "FAILED")}");
bool corridors = CorridorSmoke.Run();
Console.WriteLine($"Corridor follow: {(corridors ? "ok" : "FAILED")}");
bool state = StateSmoke.Run();
Console.WriteLine($"Grid state run: {(state ? "ok" : "FAILED")}");
bool crowds = CrowdSmoke.Run();
Console.WriteLine($"Crowd run: {(crowds ? "ok" : "FAILED")}");
return ok && tileCache && orca && nav && grids && flow && graphs && corridors && state && crowds && OnePathInfo.Version.Major >= 0 ? 0 : 1;
