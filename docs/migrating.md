# Migrating to onepath

For code that uses Recast and Detour (C++), DotRecast (C#) or Unity's built-in navigation. It maps concepts and types, and calls out
the places where a number means something different. The mapping comes from the three projects' published API documentation; where
you depend on an exact behaviour of the old library, check it against that library's own documentation.

Three differences run through everything:

- **Units are world units.** Recast's build settings count in voxels (cells); onepath's `NavSettings` are metres-style world units, and
  the library does the conversion. Pass `AgentRadius = 0.6`, not `walkableRadius = 2`.
- **Coordinates are `double`** at the public boundary (`Vector3D` from `RecursiveAI.OneMath`), exact within ±2^21, with Y up. Positions
  beyond that bound throw `ArgumentOutOfRangeException`. Convert engine vectors with `Vector3D.FromSystemNumerics(v)` and
  `vector3D.ToSystemNumerics()`; `NavMesh.Build(ReadOnlySpan<float> vertices, ReadOnlySpan<int> indices, settings)` takes the
  float arrays engines hold (the float to double step is exact).
- **Mesh bytes are not interchangeable.** A Detour or DotRecast navigation-mesh file is not read by onepath (it fails with a typed
  `NavMeshFormatException`, a kind of `OnePathFormatException`: bad magic), and onepath's files are not read by them. **Re-bake from your geometry** with `NavMesh.Build(...)`;
  there is no converter, because the polygon sets differ (see [navmesh-format.md](navmesh-format.md)).

## Building a mesh

One call replaces the Recast pipeline (heightfield, compact heightfield, regions, contours, polygon mesh, detail mesh, mesh data, nav mesh):

```csharp
NavMesh mesh = NavMesh.Build(vertices, indices, NavSettings.Humanoid with { AgentRadius = 0.4, CellSize = 0.25 });
```

Each stage is also callable on its own in tier 2 (`Mesh3DPipeline`, [build3d-spans.md](build3d-spans.md) onward).

| Recast / DotRecast build setting | Unit there | onepath (`NavSettings`) | Unit here |
|---|---|---|---|
| `cs` (cell size) | world units | `CellSize` (default 0.3) | world units |
| `ch` (cell height) | world units | `CellHeight` (0.2) | world units |
| `walkableSlopeAngle` | degrees | `AgentMaxSlopeDegrees` (45) | degrees, 0 to 90 |
| `walkableHeight` | voxels | `AgentHeight` (2) | world units |
| `walkableClimb` | voxels | `WalkableClimb` (0.9) | world units |
| `walkableRadius` | voxels | `AgentRadius` (0.6) | world units |
| `maxEdgeLen` | voxels | `MaxEdgeLength` (3.6) | **world units**; 0 disables |
| `maxSimplificationError` | voxels | `ContourMaxError` (1.3) | cells (same meaning) |
| `minRegionArea` | cells; the samples pass a side length and square it | `MinRegionArea` (64) | **cells, an area**: pass `8 * 8`, not `8` |
| `mergeRegionArea` | cells; same convention | `MergeRegionArea` (400) | **cells, an area**: pass `20 * 20` |
| `maxVertsPerPoly` | vertices | `MaxVertsPerPoly` (6) | vertices, 3 to 16 |
| `detailSampleDist` | the samples multiply by `cs` | `DetailSampleDistance` (1.8) | **world units**; 0 turns height detail off |
| `detailSampleMaxError` | the samples multiply by `ch` | `DetailMaxError` (0.2) | **world units** |
| `tileSize`, `borderSize` | voxels | `TileSize` (64) | **world units**; the border is internal |
| partition (watershed, monotone, layers) | enum | `Partition` | `PartitionType` |
| `bmin`, `bmax` | world | `OriginX`, `OriginZ` | tile grid origin; the heightfield bounds come from the input |

Flags and areas: Detour polygons carry a 16-bit flags value and a 6-bit area id; so do onepath's (`NavSettings.Flags`, `Area` for a 2D
build; areas from `SpanSettings` and volumes in 3D; 64 areas). Area costs are multipliers in a `QueryFilter` (`WithAreaCost(area, cost)`),
not a per-area array on the query. Off-mesh connections are `NavSettings.OffMeshLinks` (`OffMeshLinkDefinition`).

| Concept | Recast / Detour / DotRecast | onepath |
|---|---|---|
| Triangle input | `rcRasterizeTriangles` over `float[]` | `NavMesh.Build(float or double vertices, indices)`, `Triangle3D`, `NavMesh.BuildGltf` |
| Voxel input | height field built by hand | `NavMesh.Build(VoxelChunk)`, `NavMesh.Build(SolidHeightfield)` |
| Tile map / 2D outlines | not supported | `NavMesh.Build2D(...)` (exact, no voxels) |
| Tiled mesh | `dtNavMesh` plus `dtNavMeshCreateParams` per tile | `NavMesh` is tiled by `NavSettings.TileSize`; `TileStreamer` and `NavWorld` stream and mix agent sizes |
| Several agent sizes | one mesh per size, by hand | `NavWorld` (one mesh per agent class) |
| Run-time obstacles and area volumes | tile cache | `TileCache` |
| Save and load | `dtNavMesh` data blobs | `NavMeshSerializer`, snapshots in [navmesh-snapshot.md](navmesh-snapshot.md) |

## Queries

| Detour / DotRecast | onepath | Notes |
|---|---|---|
| `findNearestPoly(center, halfExtents, filter)` | `NavMeshQuery.FindNearestPoly(center, halfExtents, filter)` | Returns `NearestPoly` (poly reference and point) |
| `findPath` then `findStraightPath` | `NavMesh.FindPath(start, end, options)` | One call, returns `PathResult` with `Points`, `Status`, `IsPartial`; options carry the filter and search extents (default half-extents 2, 4, 2). Tier 2: `BeginPath`/`UpdatePath`/`FinishPath` and `FindStraightPath` |
| sliced path query (`initSlicedFindPath`, `updateSlicedFindPath`, `finalizeSlicedFindPath`) | `BeginPath`, `UpdatePath(maxIterations)`, `FinishPath` | The budget counts iterations, never time; slicing gives the same result as one call |
| `raycast` | `NavMesh.Raycast(from, to, out NavMeshHit, filter)`; `NavMeshQuery.Raycast` | `NavMeshHit` has position, normal and distance |
| `moveAlongSurface` | `NavMeshQuery.MoveAlongSurface` | |
| `findRandomPoint`, `findRandomPointAroundCircle` | `NavMesh.TryGetRandomPoint(ref SeededRandom, ...)`, `NavMeshQuery.RandomPoint`/`RandomPointAround` | The generator is passed in and seeded: there is no hidden random state |
| `findDistanceToWall`, `findPolysAroundCircle`, `findLocalNeighbourhood` | `DistanceToWall`, `FindPolysInCircle`, `FindPolysInShape`, `FindLocalNeighbourhood` | |
| `closestPointOnPoly` | `ClosestPointOnPoly` | |
| `dtQueryFilter` (include/exclude flags, area costs) | `QueryFilter` | Immutable; `new QueryFilter { ExcludeFlags = 2 }.WithAreaCost(3, 5)`. Struct form for the zero-allocation span queries: `DefaultQueryFilter`, or your own `IQueryFilter` |
| `dtStatus` bit flags | `QueryStatus`, `PathStatus` | Enums, not bit masks; see [path-results.md](path-results.md) |
| Poly reference `dtPolyRef` | `PolyRef` | 64-bit (24 bits salt, 20 slot, 20 polygon) |

A path that cannot reach the goal returns the corridor to the nearest reachable polygon with `IsPartial = true`, as Detour does.

## Crowds

`dtCrowd`/`DtCrowd` ≈ `Crowd` (`mesh.CreateCrowd()`); `dtCrowdAgentParams` ≈ `AgentOptions` (`Agent.Humanoid`, `Small`, `Large`).

| Detour crowd parameter | onepath | Notes |
|---|---|---|
| `radius` | `AgentOptions.Radius` | Body radius between agents; the mesh is already shrunk by its build radius |
| `height` | `NavSettings.AgentHeight` | Build time, not per agent |
| `maxSpeed`, `maxAcceleration` | `MaxSpeed`, `MaxAcceleration` | units per second, per second squared |
| `collisionQueryRange` | `CrowdOptions.MaxWallsPerAgent`, `WallRefreshDistance`; neighbours by `Quality` | |
| `pathOptimizationRange` | `FollowerOptions.OptimizeRange` (through `AgentOptions.Path`) | |
| `separationWeight` | `SeparationDistance` and `SeparationWeight` | **Not the same scale.** Detour's weight is a non-negative number (the samples use values such as 2); onepath's `SeparationWeight` is **0 to 1**, the fraction of `MaxSpeed` a touching neighbour adds to the wanted velocity (default 0.3), and separation is off until `SeparationDistance` is above 0. Values above 1 throw. Start with `SeparationDistance = 2 * Radius` and tune |
| `obstacleAvoidanceType` | `Quality`: `Off`, `Low`, `Medium`, `High` | Presets for neighbour count and time horizon |
| `queryFilterType` | `CrowdOptions.Filter` (one filter per crowd) | |
| `updateFlags` (anticipate turns, optimize visibility, optimize topology, obstacle avoidance, separation) | `FollowerOptions` intervals and budgets; `Quality`; `SeparationDistance` | A feature is off when its interval, distance or quality is 0 |
| `requestMoveTarget` | `Agent.SetDestination(position)` → `PathStatus` | Returns `GoalInvalid` if the goal is not near the mesh |
| `update(dt)` | `Crowd.Step(dt)` | Fixed `dt` is the supported use; the crowd has no clock |
| `getActiveAgents` | `foreach (Agent a in crowd)`, `CopyAgents(span)` | Slot order |
| `addAgent` returning an index | `Crowd.Add(position, options)` returns an `Agent` | Throws if the position is off the mesh or the crowd is full |

The velocity solver is ORCA (van den Berg et al.), made aware of mesh edges; it is not the sampled velocity-obstacle search Detour uses,
so agent paths differ in detail even with matching numbers. Details in [crowds.md](crowds.md) and [orca.md](orca.md).

## From Unity's navigation

| Unity | onepath |
|---|---|
| `NavMeshSurface` / `NavMeshBuilder.BuildNavMeshData` | `NavMesh.Build(vertices, indices, settings)`, or `NavMesh.BuildGltf`; Unity's `float[]` and `Vector3[]` meshes convert with `Vector3D.FromSystemNumerics` |
| Agent Type (radius, height, step height, max slope) | `NavSettings` (`AgentRadius`, `AgentHeight`, `WalkableClimb`, `AgentMaxSlopeDegrees`); one mesh per size through `NavWorld` |
| `NavMesh.SamplePosition(point, out hit, maxDistance, areaMask)` | `NavMesh.TrySamplePosition(position, maxDistance, out point, filter)` |
| `NavMesh.CalculatePath(a, b, areaMask, path)` | `NavMesh.FindPath(a, b, options)`; the path is `PathResult.Points` and `Status` tells complete from partial |
| `NavMesh.Raycast` | `NavMesh.Raycast(from, to, out NavMeshHit, filter)` (returns true when the ray is blocked, like Unity) |
| Area costs and `areaMask` | `QueryFilter.WithAreaCost` and `IncludeFlags`/`ExcludeFlags` |
| `NavMeshAgent.speed`, `acceleration`, `radius`, `avoidancePriority` | `AgentOptions.MaxSpeed`, `MaxAcceleration`, `Radius`, `Priority` (**greater priority gives way less**, the opposite direction to Unity's lower-number-wins `avoidancePriority`) |
| `NavMeshAgent.stoppingDistance` | `AgentOptions.ArrivalRadius` |
| `NavMeshAgent.SetDestination`, `remainingDistance`, `velocity`, `hasPath` | `Agent.SetDestination`, `RemainingDistance`, `Velocity`, `HasPath` |
| `NavMeshAgent.Warp` | `Agent.Teleport(position)` (returns false when off the mesh) |
| `NavMeshAgent` moving the transform | You move your own objects: read `Agent.Position` after `crowd.Step(dt)` |
| `NavMeshObstacle` (carving) | `TileCache` obstacles |
| `OffMeshLink` | `NavSettings.OffMeshLinks`; with `OffMeshMode.Host` the host drives the crossing and calls `Agent.CompleteOffMeshLink()` |
| Per-frame `Update` with `Time.deltaTime` | `crowd.Step(fixedDt)`: use a fixed step, not the frame time, if you need identical results on every machine |

Unity's navigation is not deterministic across platforms; onepath's is. If your game relied on float quirks of the old library (a
path that happened to cut a corner), expect small differences and re-tune against the settings above.
