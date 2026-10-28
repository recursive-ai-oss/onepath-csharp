# Changelog

All notable changes are listed here. The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project follows [Semantic Versioning](https://semver.org/).

## [Unreleased]

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

These types carry `[Experimental("ONEPATH0002")]` and may change or move in any preview:

- the run-time tile cache: `TileCache`, `TileObstacle`, `ObstacleShape`, `Mesh3DPipeline.BuildSoupCached`, `BuildChunkCached`, `BuildHeightfieldCached` and `Mesh3DPipeline.Obstacles`. A tile rebuild still costs several times what a comparable native tile cache spends (see the speed table in the README).

### Known limits

- Peak memory is higher than the native references'.
- Path queries on one large single-tile city mesh ran 1.6-1.8x slower than Detour in the last locked run (a later change to the mesh contours is not yet re-measured), and the tile cache is 4-6x slower than DetourTileCache; the README table lists the rows.

[Unreleased]: https://github.com/recursive-ai-oss/onepath-csharp/compare/v0.4.0-preview.1...HEAD
[0.4.0-preview.1]: https://github.com/recursive-ai-oss/onepath-csharp/releases/tag/v0.4.0-preview.1
