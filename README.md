# onepath

**Pathfinding and navigation for .NET, written from first principles: grids, graphs, navigation meshes and crowds in one package, with results that are bit-identical on every platform.** Square, hex and 3D voxel grids with A\*, JPS, JPS+, Theta\*, HPA\* and incremental replanning; weighted and point graphs; an exact 2D navmesh builder and a Recast-class 3D builder into one tiled format; path, raycast and nearest-point queries; crowds with navmesh-aware ORCA. Pure C# with one runtime dependency, our own deterministic math library.

[![CI](https://github.com/recursive-ai-oss/onepath-csharp/actions/workflows/ci.yml/badge.svg)](https://github.com/recursive-ai-oss/onepath-csharp/actions/workflows/ci.yml)
[![.NET 10](https://img.shields.io/badge/.NET-10.0-512BD4)](#install)
[![License](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](#licence)

**Status: 1.1.0.** Every part meets its speed target on x64 and arm64 (see [the table](#speed-against-the-references)). The public API follows semantic versioning, and results and saved formats are stable across every 1.x release (see [Determinism](#determinism)). See [CHANGELOG.md](CHANGELOG.md).

## Packages

| Package | What it is |
|---|---|
| `RecursiveAI.OnePath` | Grids (square, hex, voxel), graphs, the navmesh builders, the query layer, the run-time tile cache and crowds. Trimming drops the parts an app does not use: an app that only uses grids trims the navmesh builders away. |
| [`RecursiveAI.OneMath`](https://www.nuget.org/packages/RecursiveAI.OneMath) | The only dependency (1.0 or later): vectors, our own transcendental functions, exact geometric predicates and the fixed-point bridge. Installed automatically. |

## Install

```
dotnet add package RecursiveAI.OnePath
```

The library targets `net10.0` on Windows, Linux and macOS (x64 and arm64), is NativeAOT- and trim-compatible, runs in browser WASM, and uses only the .NET base class library and `RecursiveAI.OneMath` at runtime. It never reads files or the network: glTF files, saved meshes and snapshots come in as bytes you supply.

## The three tiers

Each tier is built on the one below. **The happy path** (this page's snippets) is options objects with presets (`GridOptions`, `NavSettings.Humanoid`, `Agent.Humanoid`) and plain delegates; you never write a struct. **The middle tier** exposes every build stage as a replaceable static method, one `NavMeshQuery` with filters, area costs and sync, sliced and span forms, and struct-generic cost and heuristic hooks for the grid and graph searches. **The raw tier** is voxel spans, regions, contours, polygons, tile data, the open and closed sets and the ORCA solver. The [documentation index](docs/README.md) is organised the same way.

## Quick start

One `using RecursiveAI.OnePath;` reaches every type below, grids, graphs, navmeshes and crowds alike (the navmesh examples also
use OneMath's vectors); `PathResult`, `PathOptions`, `QueryFilter`, `FollowerState` and `PartitionType` live there too. Options are plain objects with named presets and optional
delegates; you never write a struct to use the happy path (the struct-generic search cores are there for tuning, in
`RecursiveAI.OnePath.Grids` and `.Graphs`). World positions are doubles within ±2^21.

```csharp
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;
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

## Determinism

Results depend only on the inputs and options, never on the operating system, CPU, SIMD width, runtime version, JIT tier, WASM versus native, or thread count. The library uses only IEEE-exact `+ - * /` and `sqrt`, its own transcendental functions (from `RecursiveAI.OneMath`), fixed reduction orders and index-ordered parallel work; ties break on node or polygon index, never on hash order. The same inputs give the same bits on x64, arm64 and WASM, checked by a hash matrix of fixed scenarios across JIT, scalar, NativeAOT, thread-count and WASM runs ([docs/hash-matrix.md](docs/hash-matrix.md)).

For a deterministic game host (lockstep, rollback) that means:

- **Coordinate rule.** World positions cross the API as `double` and are exact within ±2^21 (a Q31.32 fixed-point value converts exactly); a coordinate beyond that throws `ArgumentOutOfRangeException` naming the value and the bound. Inside, meshes store float32 coordinates relative to a double tile origin.
- **No clock.** `Crowd.Step(dt)` takes the step from you; sliced searches, rebuilds and replans count iterations, never time.
- **Snapshot and restore.** Grid searches, replanners, flow fields, dynamic meshes (rebuilt tiles, obstacles) and crowds save to bytes and restore to the same state; the same state writes the same bytes on every platform.
- **Debug geometry** (polygons, links, paths, corridors, agent velocities) comes out as plain data.

- **Across versions.** Within a major version, every release gives the same bits for the same inputs and options. No 1.x minor or patch release changes an existing path, mesh build, query, search or crowd step. A change that would alter an existing result, even a better one, does one of two things. It ships behind a new option whose default keeps the old result. Or it waits for the next major version and is listed in the changelog. Features added in a minor release then keep their results for the rest of that major version. The math underneath, `RecursiveAI.OneMath`, makes the same promise, so updating either package within its major version never moves a result. Peers in a lockstep session, and replays recorded on another 1.x release, stay in sync. The hash matrix checks this: every line of the previous release's manifest must appear unchanged in the next one's.
- **Saved data.** Every 1.x release reads every format written by an earlier 1.x release: navigation meshes, tile caches and every saved state (grids, searches, replanners, flow fields, dynamic meshes, crowds). This promise is backward reading only. A later release may write a newer format version, and an earlier release need not read it. Fixtures written by 1.0.0 are kept in the tests and loaded by every later release.

The formats, tie-breaks and budget units are written down in [docs/](docs/README.md) so a port can reproduce the same bits from the text.

## What it does not do

- **No rendering or engine glue.** Debug geometry is data; drawing it and moving your game objects is yours.
- **No steering beyond ORCA avoidance.** Formations and general steering behaviours are not included.
- **No volume pathfinding for flying agents** beyond the voxel grid search.
- **Memory is higher than the native references'** (see the speed section): the library trades memory for speed and safety.

## Documentation

Coming from Recast, Detour, DotRecast or Unity? [docs/migrating.md](docs/migrating.md) maps the types, settings and units. The normative documents in [docs/](docs/README.md) hold every choice the papers leave open. API reference is the XML documentation shipped with the package.

## Speed against the references

Every row is **our time divided by the reference's** (below 1.00 is faster than the reference), warm, in-process, best of
three, on the same inputs and settings, with grid costs checked against the optimal costs. The bar is 1.15 for navmesh parts and 1.00 for grids. Grid, build and tile-cache rows are from one locked run; the path-query rows ("later run") are from a short locked run after the partial-path fix; the crowd rows are from the locked run before, because crowd stepping has not changed since. Rows refreshed after each
run. Millisecond-scale rows (queries, tile cache) warm up for at least two seconds before timing. Method: [docs/benchmarking.md](docs/benchmarking.md).

- **x64:** AMD Ryzen 9 7950X3D (16 cores, 32 threads), Linux, .NET 10.0.13.
- **arm64:** Apple M2 (8 cores), macOS, .NET 10.0.11.
- **References:** Recast and Detour v1.6.0 (with DetourCrowd, DetourTileCache), RVO2 v2.0.2 (C++), RVO2-CS v2.0.1, DotRecast 2026.3.1, Warthog v0.6.2 (warthog-core and warthog-jps). All are used as black boxes through their documented APIs; none of their code is in this repository or package.

| Part | Scenario | Reference | x64 | arm64 | Notes |
|---|---|---|---:|---:|---|
| Grid A\* | 9 Moving AI maps | Warthog | 0.83-0.98 | 0.86-1.00 | optimal costs match on every instance |
| Grid JPS | 9 Moving AI maps | Warthog | 0.49-0.79 | 0.46-0.70 | |
| Grid JPS+ | 9 Moving AI maps | Warthog | 0.64-0.85 | 0.45-0.82 | |
| Grid JPS, improved pruning | 9 Moving AI maps | Warthog | 0.71-0.88 | 0.57-0.88 | |
| Grid JPS+, improved pruning | 9 Moving AI maps | Warthog | 0.80-1.00 | 0.58-0.95 | |
| Path query | city, tiled | Detour | 0.53 | 0.45 | later run |
| Path query | terraces, tiled | Detour | 0.50 | 0.40 | later run |
| Path query | terraces, single tile | Detour | 0.73 | 0.57 | later run |
| Path query | overlapping floors, single tile | Detour | 0.14 | 0.13 | later run |
| Path query | city, single tile | Detour | 0.92 | 0.91 | later run |
| Path query | tile map, tiled | Detour | 1.04 | 0.94 | later run (1.40 / 1.26 before the partial-path fix) |
| 3D build | 8 levels, solo and tiled | Recast | 0.23-0.89 | 0.24-1.06 | same settings; walkable coverage within 0.35% of Recast's on every level |
| 2D build | tile map, solo and tiled, radius 0.5 / 0.75 / 1.0 | Recast | 0.42-0.89 | 0.43-0.95 | see the coverage note below |
| Crowd | 100-5000 agents on a city mesh | DetourCrowd | 0.47-0.72 | 0.48-0.66 | earlier run (crowd code unchanged) |
| Crowd, ORCA only | circles of 250, 1000, 5000 agents, one thread | RVO2 (C++) | 0.71-1.11 | 0.69-1.01 | earlier run |
| Crowd, ORCA only | circles of 250, 1000, 5000 agents, all cores | RVO2-CS | 0.05-0.25 | 0.08-0.35 | earlier run |
| Tile cache | obstacle add and remove, city and terraces, tiled | DetourTileCache | 0.78-0.93 | 0.68-0.91 | |
| Tile cache | obstacle add and remove, city and terraces, tiled | DotRecast | 0.33-0.45 | 0.31-0.43 | |

DotRecast is faster than ours on none of the build, query, crowd and tile-cache rows above.

**2D coverage.** The 2D builder is checked against the exact eroded area of the input (walkable coverage within 1%): ours is
within 0.01% of it on every tile-map row. Recast's coverage on the same settings is 31-44% below that area (DotRecast 31-51%),
because its walkable area ends at the agent radius plus one cell (ledge filtering and minimum region size); we report it
as information, not as a target.

**Memory.** Peak resident memory is higher than the native references': 35-49 MB against 5-24 MB on the grid rows, 61-1060 MB
against 5-85 MB on the build rows (Recast).

**WASM** speed is measured and reported, not gated. Every other row of the full run (including DotRecast on each build,
query and crowd row) is in the bar runner's CSV output (`bench/Bar`).

The maps and scenarios are the Moving AI Lab benchmarks, used under the Open Data Commons Attribution License; see
[THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md) and cite N. Sturtevant, *Benchmarks for Grid-Based Pathfinding*, IEEE TCIAIG
4(2), 2012, for any result computed from them.

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
  its only runtime dependency (1.0.0, restored from nuget.org). To use another build, set `ONEMATH_FEED` to a folder containing the
  `.nupkg` and restore picks it up; the `ci/` scripts default it to `~/onepath/feed` when that exists.
  On a machine that cannot see the folder, `tools/install-feed.sh <folder-with-nupkg>` copies the package
  into `~/onepath/feed`. (For a remote job runner that only syncs the git tree, ship the package with the
  runner's per-file include option and run that script on the remote machine.)
- `ONEPATH_DOTNET_ROOT`: scripts put this user-local SDK folder first on `PATH`.

## Licence

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE) at your option. Copyright Recursive AI, LLC. Third-party material (benchmark maps and the reference implementations used as black-box comparisons) is listed with its terms in [THIRD-PARTY-NOTICES.md](THIRD-PARTY-NOTICES.md).
