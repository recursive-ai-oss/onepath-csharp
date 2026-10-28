# Path results

Every one-call path query reports through the same vocabulary, whatever the world is made of.

## `PathStatus` (namespace `RecursiveAI.OnePath`)

| Value | Number | Meaning |
|---|---|---|
| `Found` | 0 | A path to the goal exists in the result. |
| `StartInvalid` | 1 | Start outside the world, blocked, or no walkable surface near it. |
| `GoalInvalid` | 2 | Same for the goal. |
| `NoPath` | 3 | The reachable set was exhausted; grid and graph queries return no best-effort path. |
| `BudgetExhausted` | 4 | The iteration budget ran out; a sliced query may be continued. |
| `Partial` | 5 | Goal unreachable (or node limit hit); the path leads to the closest reachable place. Navmesh queries and crowd agents. |
| `InProgress` | 6 | Not finished; later crowd steps continue it. |

The numbers are fixed (hash scenarios and logs write them). `HasPath()` is true for `Found` and `Partial`.

## Result members

| Member | `GridPath` (square, hex) | `VoxelPath` | `GraphPath` (graph, point graph) | `PathResult` (navmesh) |
|---|---|---|---|---|
| `Status` | `PathStatus` | same | same | same (`Detail` holds the `QueryStatus`) |
| `Found`, `HasPath`, `IsPartial` | yes | yes | yes | yes |
| `Cost` | search cost, +∞ without a path | same | same | corridor cost under the filter |
| `Length` | polyline length of `Points`; 0 without a path; NaN when a path exists but the query made no points | same | same (plain graphs: NaN) | straight-path length, 0 without a path |
| Discrete path | `Cells` | `Cells` | `Nodes` | `Corridor` (polygons) |
| World-space path | `Points` (`WorldPoint`) | `Points` (`VoxelPoint`) | `Points` (`Point3`, point graphs) | `Points` (`StraightPathPoint`) |

`Agent.SetDestination` returns a `PathStatus`: `Found`, `Partial`, `InProgress`, `GoalInvalid` (nothing walkable near the destination)
or `StartInvalid` (the agent is not on the mesh).

## Tier 2 and 3 keep their detail

`QueryStatus` (namespace `RecursiveAI.OnePath.Mesh.Queries`) is the polygon-level outcome of `NavMeshQuery`'s sliced and span forms,
straight paths and path corridors: `Success`, `Partial`, `InProgress`, `InvalidStartPoly`, `InvalidEndPoly`, `NoStartNearby`,
`NoEndNearby`, `InvalidCorridor`. `QueryStatus.ToPathStatus()` maps it: Success→Found, Partial→Partial, InProgress→InProgress,
the start errors→StartInvalid, the end errors→GoalInvalid, InvalidCorridor→NoPath. `SearchStatus` (grid sliced search), `RaycastStatus`,
`AreaQueryStatus`, `CorridorStatus` and `FlowFieldStatus` describe their own operations and stay as they are.

