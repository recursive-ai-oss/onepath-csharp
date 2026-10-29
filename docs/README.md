# onepath documentation

The documents here are normative: where a paper leaves a choice open (tie-breaks, open-list order, rounding, byte layouts,
budget units, preset values) the choice is written down so that a port to another language can reproduce the same bits from
the text alone. API reference is in the XML documentation that ships with the package; the [README](../README.md) has the
quickstart and the speed table.

The library has three tiers, each built on the one below:

1. **Happy path.** `Grid.FromText(...)` or `Grid.From(...)` then `.FindPath(a, b)`, `HexGrid`, `VoxelGrid`, `Graph`, `PointGraph`, `NavMesh.Build(...)`,
   `NavMesh.Build2D(...)`, `NavMesh.BuildGltf(...)`, `mesh.FindPath(a, b)`, `mesh.CreateCrowd()`, `crowd.Add(...)`,
   `agent.SetDestination(...)`, `crowd.Step(dt)`. Everything is an options object with presets (`GridOptions`, `NavSettings.Humanoid`,
   `Agent.Humanoid`) or a plain delegate. Namespace `RecursiveAI.OnePath`. You never write a struct.
2. **Middle.** Each build stage as a visible, replaceable static method; one `NavMeshQuery` with filters, area costs and
   sync, span and sliced forms; grid and graph searches with struct-generic cost and heuristic hooks; corridors and path
   followers; tile cache; tile streaming. Namespaces `.Grids`, `.Graphs`, `.Mesh.*`, `.Crowds.Corridors`.
3. **Raw.** Voxel spans, regions, contours, polygons, tile data, the open and closed sets, the search loop, the ORCA solver
   (`.Mesh.Build3D`, `.Mesh.Build2D`, `.Mesh.Format`, `.Grids` engines, `.Crowds.Orca`).

## Where to start

| You want to | Read |
|---|---|
| Move from Recast, Detour, DotRecast or Unity's navigation | [migrating.md](migrating.md) |
| Know what a path result contains and how statuses map across grids, graphs and meshes | [path-results.md](path-results.md) |
| Search a tile map, hex map or voxel world | [grids.md](grids.md) |
| Route over a weighted or point graph, with landmarks | [graphs.md](graphs.md) |
| Build a navmesh from a tile map or outlines | [build2d-tiles.md](build2d-tiles.md) |
| Build a navmesh from level geometry | [build3d-polys.md](build3d-polys.md) |
| Query a mesh (path, raycast, nearest, random point) | [navmesh-queries.md](navmesh-queries.md) |
| Run a crowd | [crowds.md](crowds.md), then [orca.md](orca.md) for the solver |
| Run inside a lockstep or rollback host | [Host hooks](#host-hooks-for-deterministic-hosts) below |

## By area

### Grids and graphs (tier 1 and 2)

| Document | Covers |
|---|---|
| [grids.md](grids.md) | Square (4- and 8-connected), hex and 3D voxel grids; A\*, JPS, JPS+, Theta\*, Lazy Theta\*, HPA\*; LPA\* and D\* Lite replanners; flow fields; area costs; agent clearance; tie-breaking and open-list order; incremental tile updates; cost and heuristic hooks |
| [graphs.md](graphs.md) | Weighted and point graphs, A\*, bidirectional search, ALT landmarks and their serialization |
| [grids-state.md](grids-state.md) | Byte-exact save and restore of grid, graph, replanner, flow-field and sliced-search state |

### Navigation mesh: format and queries (tiers 2 and 3)

| Document | Covers |
|---|---|
| [navmesh-format.md](navmesh-format.md) | The tiled mesh (format version 2, frozen): tile layout, polygons, detail heights, areas and flags, off-mesh links, several agent sizes, streaming, serialization, build fingerprint |
| [navmesh-queries.md](navmesh-queries.md) | `NavMeshQuery`: path (polygon and crossing node models), string pulling, raycast, nearest polygon and point, random point, distance to wall, shape queries, sliced queries |
| [navmesh-snapshot.md](navmesh-snapshot.md) | Byte-exact snapshot and restore of a dynamic mesh (rebuilt tiles, obstacles) |

### Navigation mesh: builders

| Document | Covers |
|---|---|
| [build2d-geometry.md](build2d-geometry.md) | The exact 2D builder's geometry: snap grid, regions, boolean operations, offsets and shrink, constrained Delaunay, convex merge, exact predicates |
| [build2d-tiles.md](build2d-tiles.md) | Tile slicing, plane-to-mesh mapping and rounding, `NavMesh.Build2D` |
| [build3d-spans.md](build3d-spans.md) | 3D builder, front half: voxelization of triangle soup and voxel chunks, span filters, compaction, erosion |
| [build3d-regions.md](build3d-regions.md) | Regions (watershed, monotone, layers) and contours |
| [build3d-polys.md](build3d-polys.md) | Polygons, per-polygon height detail, tiles, `NavMesh.Build` |
| [build3d-gltf.md](build3d-gltf.md) | The glTF 2.0 reader and `NavMesh.BuildGltf` |
| [build3d-runtime.md](build3d-runtime.md) | Run-time tile cache: tile rebuild, temporary obstacles, area volumes |

### Crowds

| Document | Covers |
|---|---|
| [crowds.md](crowds.md) | Path corridors, path following, off-mesh links, `Crowd` and `Agent`, separation, priorities, avoidance quality, crowd snapshot bytes |
| [orca.md](orca.md) | The ORCA solver: constraints, linear program, neighbour order, where the paper is silent |

## Conventions across the tiers

- **Options are sealed records, immutable once constructed** (`GridOptions`, `VoxelOptions`, `PathOptions`, `NavSettings`,
  `CrowdOptions`, `AgentOptions`, `GraphOptions`, `PointGraphOptions`, `LandmarkOptions`, `FollowerOptions`): properties are `init`-only, so
  a shared preset cannot be changed by one caller under another, and `with` derives a changed copy (`GridOptions.FourConnected with { Algorithm = GridAlgorithm.Jps }`).
  There is no `Clone()`. Two options with the same values are equal (delegates compare by reference). A list or array an option
  holds (`NavSettings.OffMeshLinks`, `PointGraphOptions.Links`, `Mesh3DPipeline.Obstacles`) is copied when it is set, so editing the
  caller's array afterwards changes nothing. `QueryFilter` is immutable too (`WithAreaCost` copies). Settings classes of the middle
  and raw tiers (stage settings of the 3D builder, for example) stay mutable builders.
- **One validation point.** A value is checked where it is set: `new GridOptions { AgentSize = 0 }` or `Agent.Humanoid with { MaxSpeed = -5 }`
  throws `ArgumentOutOfRangeException` right there, with `ParamName` the property and a message that states the allowed range.
  Rules that tie several values together (a tile size against the cell size, a filter against a mesh) are checked when the
  options are used and name the properties involved. `Validate()` on the options types repeats the field checks for values built another way.
- **Cancellation:** every tier-1 `FindPath` takes an optional `CancellationToken` and throws `OperationCanceledException`;
  a token that is never cancelled changes no result.
- **`FindPathAsync` means one thing everywhere** (`Grid`, `NavMesh`, `NavMeshQuery`, `NavWorld`): the signature is
  `(start, goal, options, cancellationToken)` and the search runs cooperatively on the caller's synchronization context in slices
  of `GridOptions.IterationsPerSlice` / `PathOptions.IterationsPerSlice` iterations, yielding between slices. The slices count
  iterations, never time, and any slicing returns what `FindPath` returns. Nothing moves to the thread pool, so `Blocked`, `StepCost`
  and filter delegates always run on the thread that awaits. To search on another thread, call `FindPath` inside `Task.Run`
  (and do not edit the world meanwhile). Hex and voxel grids have no async form for that reason. `Grid.FindPathAsync` slices
  A\*, JPS and JPS+ (the algorithms `GridSearch` can resume) and refuses the others and `RecordExpansions` with an `ArgumentException`.
- **Errors.** Misuse (a bad argument, a call in the wrong order) throws `ArgumentException` (`ArgumentOutOfRangeException`,
  `ArgumentNullException`) or `InvalidOperationException`. Bad *data* (bytes, text, files, geometry the builder cannot use) throws a
  subclass of `OnePathFormatException` (itself a `FormatException`), so a host that loads untrusted saves catches one type:
  `NavMeshFormatException` (with `NavMeshVersionException` and `NavMeshFingerprintException`) for meshes, tiles and snapshots,
  `GridStateException` and `GraphStateException` for saved grid, search, replanner, flow-field and graph state and landmark tables,
  `CrowdStateException` for crowd, corridor and follower state, `GridFormatException` for map and scenario text,
  `GltfException` for glTF input and `GeometryException` for a 2D builder invariant failure. Each message names its subject, the
  offset or field, and what was wrong. A data error is never an `InvalidOperationException` or `ArgumentException`.
  Expected outcomes of a query (no path, a start off the mesh) are statuses, not exceptions.
- **Where the types live.** Everything the happy path names, including `PathResult`, `PathOptions`, `QueryFilter`, `FollowerState`,
  `PartitionType`, `PathNodeModel`, `StraightPathPoint` and the status enums, is in `RecursiveAI.OnePath`: one `using` (plus OneMath's
  vectors) is enough for the quickstart. Tier 2 and 3 types are in the sub-namespaces.
- **Enumerating a crowd:** `foreach (Agent a in crowd)` and `crowd.CopyAgents(Span<Agent>)` allocate nothing; both go in slot order.
- **Listeners:** `AddListener(IGridChangeListener)` and `AddListener(IVoxelChangeListener)` on the tier-1 grids are observer
  registration, not search hooks: a class implements the interface, never a struct, and nothing on the happy path needs one. They
  are the only library interfaces the tier-1 surface test allows in the root namespace. Replanners and flow fields implement the
  callbacks explicitly, so `OnCellChanged` is not a method you can call on them.

## Host hooks for deterministic hosts

A lockstep or rollback host needs five things; each has its own tests and is documented where it lives.

| Hook | What | Document |
|---|---|---|
| Coordinate rule | World positions are `double`, exact within ±2^21 (a Q31.32 fixed-point value converts exactly); beyond that `ArgumentOutOfRangeException`. Meshes store tile-local `float32` relative to a double tile origin | [navmesh-format.md](navmesh-format.md), [grids.md](grids.md) |
| Snapshot and restore | Same state, same bytes, on every platform: grids, searches, replanners and flow fields; dynamic meshes; crowds | [grids-state.md](grids-state.md), [navmesh-snapshot.md](navmesh-snapshot.md), [crowds.md](crowds.md) |
| Fixed-dt stepping | `Crowd.Step(dt)` with a caller-supplied step; no internal clock | [crowds.md](crowds.md) |
| Iteration budgets | Sliced grid, graph, mesh and tile-cache work counts iterations, never time; the unit of one iteration per family is stated in each document | [grids.md](grids.md), [grids-state.md](grids-state.md), [navmesh-queries.md](navmesh-queries.md), [build3d-runtime.md](build3d-runtime.md) |
| Debug geometry | Polygons, links, paths, corridors, agent velocities and grid state as plain data (namespace `RecursiveAI.OnePath.Debug`) | [navmesh-format.md](navmesh-format.md), [crowds.md](crowds.md), [grids-state.md](grids-state.md) |

## Saved formats and their fixtures

Every 1.x release reads every format written by an earlier 1.x release: navigation meshes and tiles, mesh and tile-cache snapshots, crowd state (with and without the mesh), path corridors and followers, grid, hex and voxel state, search, replanner and flow-field state, graphs and landmark tables. Reading is backward only; a later release may write a newer version marker.

The rule that keeps it true:

- A change to a format bumps that format's version marker. The reader keeps reading every earlier 1.x version.
- Each release has a directory `tests/OnePath.Tests/Fixtures/<release>/` of small files its own code wrote (made by `tools/FormatFixtures` from generated scenes, with a `fixtures.tsv` manifest). A fixture is never regenerated or edited. A release that adds or changes a format adds a directory of its own and appends it to the list in `FormatFixtureTests`.
- The expectation recorded beside each fixture is what the loaded state does, not its bytes: queries, searches, steps and edits after the load are run and hashed (SHA-256 of a transcript written only through the public API). Re-writing and comparing bytes would break the first time a release writes a newer marker.
- `FormatFixtureTests` loads every fixture of every release with the current readers and compares. It also lists every public method that returns a byte array or fills a `Span<byte>`, and fails for one that has no fixture, so a new saved format cannot ship without one.

## Determinism and measurement

| Document | Covers |
|---|---|
| [hash-matrix.md](hash-matrix.md) | The manifest format and the columns (JIT, scalar, threads, NativeAOT, WASM, ...) that must produce equal hashes |
| [benchmarking.md](benchmarking.md) | How the speed rows are measured: warm-up, best of N, references and their versions, quality metrics, memory |
