# Path results

Every one-call path query reports through the same vocabulary, whatever the world is made of.

## `PathStatus` (namespace `RecursiveAI.OnePath`)

| Value | Number | Meaning |
|---|---|---|
| `None` | 0 | No query has produced a result. This is `default(PathStatus)`: an unassigned result, a zeroed array slot or a default struct field never reads as success. |
| `Found` | 1 | A path to the goal exists in the result. |
| `StartInvalid` | 2 | Start outside the world, blocked, or no walkable surface near it. |
| `GoalInvalid` | 3 | Same for the goal. |
| `NoPath` | 4 | The reachable set was exhausted; grid and graph queries return no best-effort path. |
| `BudgetExhausted` | 5 | The iteration budget ran out; a sliced query may be continued. |
| `Partial` | 6 | Goal unreachable (or node limit hit); the path leads to the closest reachable place. Navmesh queries and crowd agents. |
| `InProgress` | 7 | Not finished; later crowd steps continue it. |

The numbers are fixed (hash scenarios and logs write them). `HasPath()` is true for `Found` and `Partial` and false for `None`.

**Zero is never success.** Every status enum a result can carry starts with a neutral value at 0 (`None`, or `NotStarted` for a search that
has not begun), so `default` means "nothing happened". The success value is 1 in `PathStatus` (`Found`), `QueryStatus` (`Success`),
`AreaQueryStatus` (`Success`) and `RaycastStatus` (`Ok`).

## Result members

| Member | `GridPath` (square, hex) | `VoxelPath` | `GraphPath` (graph, point graph) | `PathResult` (navmesh) |
|---|---|---|---|---|
| `Status` | `PathStatus` | same | same | same (`Detail` holds the `QueryStatus`) |
| `Found`, `HasPath`, `IsPartial` | yes | yes | yes | yes |
| `Cost` | search cost, +∞ without a path | same | same | corridor cost under the filter |
| `Length` | polyline length of `Points`; 0 without a path; NaN when a path exists but the query made no points | same | same (plain graphs: NaN) | straight-path length, 0 without a path |
| Discrete path | `Cells` | `Cells` | `Nodes` | `Corridor` (polygons) |
| World-space path | `Points` (`WorldPoint`) | `Points` (`VoxelPoint`) | `Points` (`Point3`, point graphs) | `Points` (`StraightPathPoint`) |

`GridSearch.Status` and `GridSearch.Step` speak the same vocabulary: `None` before a search has begun, `InProgress` while it has work left,
then `Found` or `NoPath`. The raw engines (`AStar`, `JumpSearch`, `ThetaSearch`, `GraphAStar`) keep `SearchStatus`
(`NotStarted`, `Running`, `Found`, `NotFound`).

`Agent.SetDestination` returns a `PathStatus`: `Found`, `Partial`, `InProgress`, `GoalInvalid` (nothing walkable near the destination)
or `StartInvalid` (the agent is not on the mesh).

## Positions that are not on the mesh

One rule across the crowd members: a position with no walkable polygon near it is an expected outcome, reported by the member's result, except
where the member returns the new agent itself.

| Member | Off the mesh |
|---|---|
| `NavMesh.FindPath`, `NavWorld.FindPath` | `PathStatus.StartInvalid` / `GoalInvalid` in the result |
| `Agent.SetDestination` | `GoalInvalid` (or `StartInvalid` when the agent itself is off the mesh) |
| `Agent.Teleport` | returns `false`, nothing changes |
| `NavMesh.TrySamplePosition`, `Raycast` | `false` / blocked |
| `Crowd.TryAdd` | returns `false`, `agent` is null |
| `Crowd.Add` | throws `ArgumentException` naming `position` (its behaviour since 1.0.0) |

`Crowd.Add` and `Crowd.TryAdd` also refuse when the crowd holds `CrowdOptions.MaxAgents` agents: `Add` throws `InvalidOperationException`, `TryAdd`
returns `false` (compare `Count` with `Options.MaxAgents` to tell the two refusals apart). Coordinates that are not finite or beyond ±2^21 and
settings out of range are mistakes in the call, not outcomes: both members throw `ArgumentOutOfRangeException`.

## Tier 2 and 3 keep their detail

`QueryStatus` (namespace `RecursiveAI.OnePath.Mesh.Queries`) is the polygon-level outcome of `NavMeshQuery`'s sliced and span forms,
straight paths and path corridors: `None` (0), `Success` (1), `Partial` (2), `InProgress` (3), `InvalidStartPoly` (4), `InvalidEndPoly` (5),
`NoStartNearby` (6), `NoEndNearby` (7), `InvalidCorridor` (8). `QueryStatus.ToPathStatus()` maps it: None→None, Success→Found,
Partial→Partial, InProgress→InProgress, the start errors→StartInvalid, the end errors→GoalInvalid, InvalidCorridor→NoPath.
`AreaQueryStatus` is `None` (0), `Success` (1), `InProgress` (2), `InvalidStartPoly` (3), `InvalidShape` (4); `RaycastStatus` is `None` (0), `Ok` (1),
`InvalidStartPoly` (2), `StartOutsidePolygon` (3). `SearchStatus` (the raw engines), `CorridorStatus` and `FlowFieldStatus` describe their own operations and stay as they are.

