# onepath

Pathfinding and navigation for .NET, written from first principles: grids, graphs, navmeshes and
crowds in one package, `RecursiveAI.OnePath`. Results are bit-identical on every platform.

**Status:** preview (`0.1.0-preview.1`). The API can still change between previews.

- **Grids and graphs:** square, hex and 3D voxel grids; A\*, JPS, JPS+, Theta\*, Lazy Theta\*, HPA\*, LPA\*/D\* Lite replanning, flow fields, area costs, agent sizes; weighted and point graphs with bidirectional search and ALT landmarks.
- **Navigation meshes:** an exact 2D builder (outlines, tile maps) and a 3D builder (triangle soup, glTF, voxel chunks), with path, string-pulling, raycast, nearest-point and random-point queries, run-time tile rebuild and obstacles, off-mesh links.
- **Crowds:** path corridors and navmesh-aware ORCA avoidance, with byte-exact snapshot and restore.
- **Still being tuned:** speed against the native references on some builders, queries and crowd sizes.

**Determinism.** Results depend only on the inputs and options, never on the operating system, CPU, SIMD width,
runtime version, JIT tier, WASM versus native, or thread count. The library uses only IEEE-exact `+ - * /` and `sqrt`,
its own transcendental functions (from `RecursiveAI.OneMath`), fixed reduction orders and index-ordered parallel work, and
the same inputs give the same bits on x64, arm64 and WASM (see `docs/hash-matrix.md`). Time slicing counts iterations, never
clock time, so a deterministic game host can run it in lockstep or rollback. **Coordinate rule:** world positions cross
the API as `double` and are exact within ±2^21 (a Q31.32 fixed-point value converts exactly); a coordinate beyond that
throws `ArgumentOutOfRangeException`. Inside, meshes store float32 coordinates relative to a double tile origin.

**Platforms.** `net10.0` on Windows, Linux and macOS (x64 and arm64), NativeAOT and trimming (an app that only uses grids
trims the navmesh builders away), and browser WASM. The only runtime dependency is `RecursiveAI.OneMath`.

Licensed under either of MIT ([LICENSE-MIT](LICENSE-MIT)) or Apache-2.0
([LICENSE-APACHE](LICENSE-APACHE)) at your option. © Recursive AI, LLC.

## Quickstart

One `using RecursiveAI.OnePath;` reaches every grid and graph type below (the navmesh examples also use OneMath's
vectors and the query result types). Options are plain objects with named presets and optional
delegates; you never write a struct to use the happy path (the struct-generic search cores are there for tuning, in
`RecursiveAI.OnePath.Grids` and `.Graphs`). World positions are doubles within ±2^21.

```csharp
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
using RecursiveAI.OnePath.Mesh.Queries;
```

**Square grid.** `.` is open, `#` a wall, `1`-`9` are area ids (give an area a cost with `grid.Areas[id] = 3.0`). You can
also build a grid from a `bool[,]` of walkable cells, or from row-major area bytes.

```csharp
// quickstart: grid
var grid = Grid.FromText("""
    ........
    .#####..
    .#...#..
    .#.#.#..
    ...#....
    """);
GridPath path = grid.FindPath(new Cell(0, 0), new Cell(4, 3));
if (path.Found)
{
    // path.Cost is exact for eight-connected octile moves and path.Cells lists the cells; query with
    // WorldPoint arguments (after setting grid.Space) to also get path.Points.
}
```

Every tunable is a property of `GridOptions`: connectivity, corner rule, agent size, algorithm (A\*, JPS, JPS+, Theta\*,
Lazy Theta\*, HPA\*), heuristic, extra costs and blocked cells, and an iteration budget for time slicing.

```csharp
// quickstart: grid-options
var wide = new Grid(64, 64);
GridPath fast = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Algorithm = GridAlgorithm.JpsPlus });
GridPath straight = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Algorithm = GridAlgorithm.ThetaStar });
GridPath fat = wide.FindPath(new Cell(1, 1), new Cell(60, 50), GridOptions.ForAgentSize(3));
GridPath avoid = wide.FindPath(new Cell(1, 1), new Cell(60, 50), new GridOptions { Blocked = c => c.X == 30 && c.Y < 60 });
```

**Hex grid.** The same calls, with a layout for the offset array.

```csharp
// quickstart: hex
var hex = HexGrid.FromText("""
    ......
    ..##..
    ..#...
    ......
    """, HexLayout.PointyOddRow);
GridPath hexPath = hex.FindPath(new Cell(0, 0), new Cell(5, 3));
```

**Voxel grid.** Six-, 18- or 26-connected moves in a dense or mostly-empty block.

```csharp
// quickstart: voxel
var voxels = new VoxelGrid(32, 32, 32);
for (int y = 0; y < 32; y++)
{
    for (int z = 0; z < 32; z++)
    {
        voxels.SetWalkable(16, y, z, y == 5 && z == 5);   // a wall with one hole
    }
}
VoxelPath flight = voxels.FindPath(new VoxelCell(0, 0, 0), new VoxelCell(31, 31, 31), VoxelOptions.SixConnected);
```

**Graphs.** Weighted graphs, and point graphs that link nearby points. Landmarks (ALT) speed up repeated queries.

```csharp
// quickstart: graph
var graph = Graph.From(new[] { new GraphEdge(0, 1, 4.0), new GraphEdge(1, 2, 1.5), new GraphEdge(0, 2, 7.0) });
GraphPath route = graph.FindPath(0, 2);   // route.Nodes is 0, 1, 2 and route.Cost is 5.5
Landmarks landmarks = graph.BuildLandmarks();
GraphPath fastRoute = graph.FindPath(0, 2, GraphOptions.WithLandmarks(landmarks));

var points = PointGraph.From(new[] { new Point3(0, 0), new Point3(1, 0), new Point3(2, 0), new Point3(2, 1) }, new PointGraphOptions { MaxDistance = 1.1 });
GraphPath walk = points.FindPath(new Point3(0, 0), new Point3(2, 1));
```

**Replanning.** A replanner keeps its search state between edits and repairs only what changed (D\* Lite by default).

```csharp
// quickstart: replan
var yard = new Grid(8, 8);
using GridReplanner replanner = yard.CreateReplanner(new Cell(0, 0), new Cell(7, 0));
GridPath straightRoute = replanner.Replan();
yard.SetRect(3, 0, 1, 7, AreaCosts.Wall);   // a wall appears
GridPath detour = replanner.Replan();
replanner.MoveStart(new Cell(1, 1));        // the agent moved
GridPath onward = replanner.Replan();
```

**Flow fields.** One field serves any number of agents heading for the same goal.

```csharp
// quickstart: flow-field
var field = new Grid(16, 16);
using GridFlowField toGoal = field.CreateFlowField(new Cell(15, 15));
double cost = toGoal.GetCost(new Cell(0, 0));          // least cost from a cell to the goal
if (toGoal.TryGetNext(new Cell(0, 0), out Cell next))  // the step a unit on that cell takes
{
    // move from (0, 0) to next
}
```

**Navigation mesh from a tile map** (a mesh from outlines, `NavMesh.Build2D(outlines)`, works the same way).

```csharp
// quickstart: navmesh-2d
var open = new bool[40 * 40];
Array.Fill(open, true);
NavMesh mesh = NavMesh.Build2D(open, 40, 40, 1.0, new Vector2D(0, 0));
PathResult route2D = mesh.FindPath(new Vector3D(3, 0, 3), new Vector3D(36, 0, 36));
// route2D.HasPath, route2D.Points (the string-pulled path), route2D.Length
```

**Navigation mesh from level geometry** (triangles, Y up; `NavMesh.BuildGltf` reads a glTF file). `NavSettings` presets carry
the agent's radius, height, step and slope.

```csharp
// quickstart: navmesh-3d
// A 40 x 40 floor from two triangles (Y is up); real levels pass their triangle soup the same way.
var floor = new[]
{
    new Triangle3D(new Vector3D(0, 0, 0), new Vector3D(0, 0, 40), new Vector3D(40, 0, 0)),
    new Triangle3D(new Vector3D(40, 0, 0), new Vector3D(0, 0, 40), new Vector3D(40, 0, 40)),
};
NavMesh level = NavMesh.Build(floor, NavSettings.Humanoid);
PathResult route3D = level.FindPath(new Vector3D(3, 0, 3), new Vector3D(36, 0, 36));
```

**Crowds.** Agents follow corridors to their destinations and avoid each other with ORCA; you call `Step` with a fixed time step.

```csharp
// quickstart: crowd
Crowd crowd = level.CreateCrowd();
Agent walker = crowd.Add(new Vector3D(3, 0, 3), Agent.Humanoid);
PathStatus status = walker.SetDestination(new Vector3D(36, 0, 36));
for (int i = 0; i < 20; i++)
{
    crowd.Step(0.05);   // fixed time step; the crowd has no clock of its own
}
// status.HasPath(), walker.HasPath, walker.RemainingDistance, walker.Velocity, walker.Position
```

The snippets above are compiled and run by `tests/OnePath.Tests/ReadmeQuickstartTests.cs`.

## Layout

| Path | What |
|---|---|
| `src/OnePath/` | the package (`RecursiveAI.OnePath`, target `net10.0`) |
| `tests/OnePath.Tests/` | unit tests (xUnit v3) |
| `tests/AotSmoke/` | NativeAOT + full-trim smoke app |
| `tests/WasmSmoke/` | browser-wasm smoke app, run headless under Node |
| `bench/Bar/` | the bar runner: time, memory and quality rows as CSV |
| `bench/Micro/` | BenchmarkDotNet micro-benchmarks |
| `fuzz/OnePath.Fuzz/` | SharpFuzz / libFuzzer targets |
| `tools/HashMatrix/` | hash-matrix runner and `tools/hash-matrix.sh` (see `docs/hash-matrix.md`) |
| `ci/` | the build, test and smoke scripts CI runs; run them locally the same way |

## Building

Requires the .NET 10 SDK (`global.json` pins 10.0.100 and rolls forward to later 10.0 feature bands).

```sh
ci/build.sh            # Release and Checked builds of the core projects
ci/test.sh             # unit tests, Release and Checked
ci/aot-smoke.sh        # NativeAOT + trimmed publish of the smoke app, then runs it
ci/wasm-smoke.sh       # browser-wasm build, run headless under Node (installs the wasm-tools workload if missing)
```

Each script takes configurations as arguments (`ci/test.sh Debug`). `dotnet build OnePath.slnx -c Release`
builds everything, including the WASM smoke app, which needs the `wasm-tools` workload.

- **Configurations:** `Debug`, `Release`, and `Checked` (optimized, defines `CHECKED`; the internal
  unchecked-access helpers become bounds-checked, and tests and fuzzing run in it).
- **Output location:** `bin/` and `obj/` never appear in the tree. They go to `$ONEPATH_ARTIFACTS`
  if set, else `<temp>/onepath-artifacts/<checkout folder name>`.
- **Package ID:** `$(PackagePrefix).OnePath`, with `PackagePrefix` defined once in `Directory.Build.props`.
- **Warnings are errors**, analyzers run at `latest-recommended`, and `BannedSymbols.txt` forbids in
  `src/` the calls that could make results machine-dependent (platform maths functions, fused
  multiply-add, `Vector<T>`, clocks, `Random`, processor count).
- **Math package:** `src/OnePath` depends on `RecursiveAI.OneMath` (version in `Directory.Packages.props`),
  its only runtime dependency. Until it is published, set `ONEMATH_FEED` to a folder containing the
  `.nupkg` and restore picks it up; the `ci/` scripts default it to `~/onepath/feed` when that exists.
  On a machine that cannot see the folder, `tools/install-feed.sh <folder-with-nupkg>` copies the package
  into `~/onepath/feed`. (For a remote job runner that only syncs the git tree, ship the package with the
  runner's per-file include option and run that script on the remote machine.)
- `ONEPATH_DOTNET_ROOT`: scripts put this user-local SDK folder first on `PATH`.
