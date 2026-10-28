# Changelog

All notable changes are listed here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- **Results are bit-identical across every release of a major version:** no 1.x minor or patch release changes an existing path, build, query, search or crowd result (README, "Determinism").
- **Every 1.x release reads every format written by an earlier 1.x release** (navigation meshes, tile caches, saved states); backward reading only.

## [1.0.0-rc.1]

Release candidate for 1.0: the public API is frozen from here unless a release candidate finds a defect. Every part measures within its speed target against the native references on x64 and arm64 (see the README table); results are bit-identical across x64, arm64, WebAssembly, SIMD and scalar, and any thread count.

### Breaking

- **`QueryFilter` is immutable after construction.** `IncludeFlags`, `ExcludeFlags` and `Predicate` are `init`-only (set them in an object initializer), and `SetAreaCost` is replaced by `WithAreaCost(area, cost)`, which returns a new filter and leaves the original alone. A filter inside `PathOptions` or `CrowdOptions` can therefore be shared by queries, crowds and threads without one caller changing it under another. Migrate `f.SetAreaCost(3, x);` to `f = f.WithAreaCost(3, x);`.

### Added

- `HexGrid.FindPathAsync` and `VoxelGrid.FindPathAsync` (cell and world-position forms), shaped like `Grid.FindPathAsync`: the result equals the synchronous one, cancellation and argument errors come through the task, and the grid must not be edited while the task runs. On single-threaded WebAssembly, await the task; never block on it.

## [0.5.0-preview.1]

### Breaking

- **Options are immutable after construction.** `GridOptions`, `VoxelOptions` and `PathOptions` properties are `init`-only like `NavSettings`, `CrowdOptions` and `GraphOptions` already were, so a shared preset (`GridOptions.Default`, the defaults used when you pass `null`) cannot be changed by one caller under another. Code that assigned a property after construction (`o.Algorithm = GridAlgorithm.Jps;`) must set it in an object initializer (`new GridOptions { Algorithm = GridAlgorithm.Jps }`). Binary-incompatible with 0.4.0-preview.3 (setters are replaced by init accessors); recompiling is enough. `Clone()` is kept.
- **`FindPath` gains a trailing `CancellationToken cancellationToken = default`** on `Grid`, `HexGrid`, `VoxelGrid`, `NavMesh`, `NavWorld`, `NavMeshQuery` and the `TiledNavMesh` extension, so existing source compiles unchanged but already-built callers must be recompiled (the old signatures are gone).

### Added

- **Cancellation on every tier-1 `FindPath`:** the token is checked every 256 iterations; cancelling throws `OperationCanceledException` and leaves the grid or mesh untouched. A token that is never cancelled returns exactly the result of no token (slicing never changes a search).
- **`NavMesh.FindPathAsync` and `Grid.FindPathAsync`** (cells and world points): same results as the synchronous forms. The mesh form is `NavMeshQuery.FindPathAsync` (slices of expansions with a yield between them); the grid form runs the synchronous search on the thread pool.
- Documentation of the cross-tier conventions (immutable options, cancellation, observer-style listeners) in `docs/README.md`; the 3D builder and mesh format public members got a documentation pass.
- **`GridSearch` runs JPS and JPS+:** `Grid.CreateSearch` accepts `GridAlgorithm.Jps` and `GridAlgorithm.JpsPlus` on the engines `FindPath` uses (the specialised octile jump point search, the generic block-map and JPS+ table searches, or plain A\* where `FindPath` falls back to it), so a budgeted jump point search resumes with `Step(budget)`, returns cell for cell the path and cost bits `FindPath` returns for every slicing, and saves and restores byte-exactly. One iteration is one pop of a jump point. State kind 12 gains mode 3 (specialised jump point engine), documented in `docs/grids-state.md`; existing states are unchanged.

### Changed

- `GridSearch.SaveState(Span<byte>)`, `GetStateSize` and `RestoreState` allocate nothing in steady state on the generic engine too (the handle keeps one serialisation buffer); `GridSearch.UsesSpecializedEngine` now answers from the options and the grid before the first search.

## [0.4.0-preview.3]

### Fixed

- **Path queries with an unreachable goal are fast again.** In the polygon node model (the default), a path whose goal cannot be reached ran a second search toward the nearest reachable polygon; on tiled tile maps, where most such queries end partial, that made path queries 1.40x (x64) and 1.26x (arm64) Detour's time. Nodes are now relaxed on cost plus the remaining straight-line estimate from their entry point, which gives the same corridors without the second search: that row now measures 1.04 / 0.94, and every path-query row is within the bar on both machines. Partial paths still end at the reachable polygon nearest the goal; path lengths on the 3D quality levels move by at most 0.4%, all toward shorter.

## [0.4.0-preview.2]

### Added

- **Grids for deterministic hosts:** `Grid.CreateSearch` returns a non-generic resumable `GridSearch` (`TryBegin`, `Step`, `CopyPath`, byte-exact `SaveState`/`RestoreState`, allocation-free on the specialised engine) that returns the path `FindPath` returns; `GridReplanner.TryReplan`/`CopyPath` (no allocation); `AStar.TryBegin`, `Topology`, `Start`, `Goal`; `Grid.ContentHash`; `Grid.TryFindNearestWalkable`, `CanFit`, `MaxAgentSizeAt`; `Grid.TryGetFixedCost` (Q31.32) and `GridPath.TryGetStepCounts`; options `SpecializedEngines` (force the generic engine so tier-1 and tier-2 A\* break ties alike) and `NearestRadius`. Documented in `docs/grids.md` and `docs/grids-state.md` (state kind 12).
- **Crowd state without allocation:** `Crowd.StateHash()` (FNV-1a 64 of the snapshot bytes), `Crowd.GetStateSize()` and `Crowd.TrySaveState(Span<byte>, out int)` write the same OPCS v1 bytes as `SaveState()` into caller memory; `RestoreState` restores the same agents without allocating (followers and corridors are read in place, so a held `Agent.Path` stays valid). Documented in `docs/crowds.md`.

### Changed

- **The run-time tile cache is no longer experimental:** the `[Experimental]` marker is removed from `TileCache`, `TileObstacle`, `ObstacleShape`, `Mesh3DPipeline.Build*Cached` and `Mesh3DPipeline.Obstacles`. An obstacle add-and-remove cycle now measures 0.78-0.93 (x64) and 0.68-0.91 (arm64) of DetourTileCache's time on the city and terraces levels. Rebuilds repair the contours of the changed layer instead of redoing them, and the result is byte-identical to a fresh build of the same state.

### Fixed

- A crowd whose settings (speed, horizon, wall clearance) were huge but valid, for example restored from a saved state, could ask the wall query for a radius beyond its limit; the radius is now bounded.
- Restoring a sliced search whose open list named a node the search never reached is now a `GridStateException` (it would have popped the node with stale parent links, and `PathLength` could loop). A state saved before `Begin` restores as `NotStarted`; check `Status` before `Step`.
- Restoring a sliced search (`AStar`, jump point, Theta\*, graph A\*) from a state that claimed *found* with a goal the search never closed could be accepted and then hang in `PathLength`/`CopyPath` by following leftover parent links in a loop; it is now a `GridStateException`.

## [0.4.0-preview.1]

First public preview. The API can still change between previews. The only runtime dependency is `RecursiveAI.OneMath`.

### Added

- **Grids:** square (4- and 8-connected), hex and 3D voxel grids; A\*, JPS and JPS+ (with improved-pruning variants), Theta\*, Lazy Theta\*, HPA\* over chunks, LPA\* and D\* Lite replanning, flow fields, area costs, agent clearance for several sizes, incremental tile updates, iteration-budgeted (sliced) searches. Costs are exact for the metric used (octile, hex distance, 3D octile).
- **Graphs:** weighted and point graphs, A\* and Dijkstra, bidirectional search, ALT landmarks.
- **Navigation mesh:** one tiled format (tiles with a double origin and float32 tile-local coordinates, multi-layer tiles, per-polygon height detail, areas, flags, off-mesh link slots, 128-bit build fingerprint per tile) with versioned, byte-exact serialization, streaming of tiles in and out, and several agent sizes (`NavWorld`). Layout documented in `docs/navmesh-format.md`; the first public version of the format is version 2.
- **Builders:** an exact 2D builder (outlines and tile maps; exact predicates, constrained Delaunay, convex merge) and a Recast-class 3D builder (triangle soup, glTF, height fields and voxel chunks entering at the span stage; watershed, monotone and layer regions; contours, polygons, height detail), both behind `NavMesh.Build2D` and `NavMesh.Build`.
- **Queries:** path (polygon-node or edge-crossing model, filters and area costs; sync and sliced), string pulling, raycast, nearest polygon and point, random point, distance to wall, polygons in a circle or shape, off-mesh links.
- **Crowds:** path corridors that follow and repair paths, path following, navmesh-aware ORCA (closed-form truncated-cone constraints), separation, priorities, fixed-step `Crowd.Step`, byte-exact crowd snapshot and restore.
- **Deterministic-host hooks:** snapshot and restore of crowd, grid, search and dynamic-mesh state; iteration budgets for sliced queries and rebuilds; debug geometry as plain data.
- **Determinism:** results are bit-identical across x64, arm64 and WASM, scalar and SIMD, and any thread count; world coordinates are `double`, exact within ±2^21, and anything beyond throws `ArgumentOutOfRangeException`.
- NativeAOT, trimming and browser-WASM smoke apps; the package is trim- and AOT-compatible.

### Experimental

These types carry `[Experimental]` and may change or move in any preview:

- the run-time tile cache: `TileCache`, `TileObstacle`, `ObstacleShape`, `Mesh3DPipeline.BuildSoupCached`, `BuildChunkCached`, `BuildHeightfieldCached` and `Mesh3DPipeline.Obstacles`. A tile rebuild still costs several times what a comparable native tile cache spends (see the speed table in the README).

### Known limits

- Peak memory is higher than the native references'.
- Path queries on one large single-tile city mesh ran 1.6-1.8x slower than Detour in the last locked run (a later change to the mesh contours is not yet re-measured), and the tile cache is 4-6x slower than DetourTileCache; the README table lists the rows.

[Unreleased]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v1.0.0-rc.1...HEAD
[1.0.0-rc.1]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v0.5.0-preview.1...v1.0.0-rc.1
[0.5.0-preview.1]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v0.4.0-preview.3...v0.5.0-preview.1
[0.4.0-preview.3]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v0.4.0-preview.2...v0.4.0-preview.3
[0.4.0-preview.2]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v0.4.0-preview.1...v0.4.0-preview.2
[0.4.0-preview.1]: https://github.com/recursive-ai-oss/onepath-csharp/releases/tag/v0.4.0-preview.1
