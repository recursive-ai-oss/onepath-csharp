# Grid, graph and search state: snapshot and restore

A host that rolls back (lockstep with prediction, rollback netcode, replays) needs to put the mutable pathfinding state back exactly and
get the results it would have got had it never left. This document is the normative description of what is saved, the byte format,
what is *not* saved and why, and what each family counts as one iteration of a budget. A port must reproduce the same bytes.

## What is covered

| Object | Saved | Rebuilt after restore |
|---|---|---|
| `Grid`, `HexGrid` | cells, the 255 area multipliers and the area table's version, `Version`, `Space` | clearance (recomputed in place at once if it had been computed), jump point indexes, HPA\* hierarchies (dropped; built on the next query) |
| `VoxelGrid` | which chunks are allocated and their 4096 bytes (padding included), area table and version, `Version`, `Space` | nothing |
| `Graph` | node count and every edge ever added in id order (from, to, weight, alive) | the compiled CSR graph, on demand |
| `GridReplanner`, `HexReplanner` (LPA\*, D\* Lite) | start, goal, g and rhs of every planned node, the queue in heap order, km, pending edits and the overflow flag, counters, the area version it was built against | the search object itself (so heuristic scale follows the restored area table) |
| `GridFlowField` (square and hex) | goals, integration costs, direction steps, the open heap of an unfinished run or repair, the touched list of a repair in progress, `Settled`, status, pending edits, area version | repair scratch stamps |
| `AStar`, `JumpSearch`, `ThetaSearch`, `GraphAStar` (the sliced `Begin`/`Step` forms) | status, goal (and start for A\*), counters, path cost, every visited node (closed flag, g, parent), the open heap in heap order, the recorded expansion list | the heuristic's goal, the jumper's index |

Not covered, and why: **HPA\*** and **bidirectional** sliced searches (an HPA\* search works on temporary abstract nodes of the
hierarchy it was built on, a bidirectional search has two buffer sets and a meeting node; both are cheap to restart and neither is
what a host holds across frames, but their *grids* roll back like any other), `PointGraph` and `CsrGraph` and `Landmarks` (immutable),
the navmesh (its own snapshot: `docs/navmesh-snapshot.md`). Tier-1 one-shot queries keep no state between calls.

## Restoring: the contract

* `SaveState()` returns a `byte[]`; `RestoreState(ReadOnlySpan<byte>)` puts it back in place; `FromState(...)` on `Grid`, `HexGrid`,
  `VoxelGrid` and `Graph` creates a new object. Saving twice without a change gives the same bytes; save → restore → save gives
  the same bytes; the same history gives the same bytes on every platform.
* Objects that are *built over* something (a replanner, a flow field, a sliced search) are restored in place into an object that was
  created the same way: same grid (restored first), same options (agent size, connectivity, diagonal rule, algorithm, delegate hooks,
  heuristic weight). Delegates cannot be saved, so the caller supplies them again. What can be checked is checked: sizes, algorithm,
  mode, goals and the heuristic weight must match, otherwise `GridStateException` names the mismatch.
* **Order:** restore the grid, then everything built on it. Restoring a grid tells its listeners: flow fields, replanners and
  HPA\* hierarchies drop what they derived and rebuild on next use; other listeners see each cell that differs as an ordinary edit
  (voxel listeners likewise per voxel, in ascending node order). Restoring a dependent afterwards replaces that rebuild with its
  exact saved state.
* **Versions can repeat after a rollback** (restore to version 7, edit once, and an object cached at "version 8" from the other
  timeline matches by number only). Nothing derived is therefore trusted across a restore; this is why the caches above are dropped.
* A restore validates before it changes anything where it can: `Grid`, `HexGrid`, `VoxelGrid`, `Graph`, `GridReplanner` and
  `HexReplanner` are unchanged when `GridStateException` is thrown. Flow fields and sliced searches are read in place and are
  unusable after a failed restore until restored again, recomputed (`Recompute`) or begun again.
* Hostile bytes: every count is bounded by the object's size and by the bytes left, every node index and parent is range-checked,
  heaps may not list a node twice, NaN is rejected, and the parent links of a search must lead only to visited nodes that cost no more and contain no cycle. The
  state is *structurally* validated, not semantically: a restored cost field (replanner g/rhs, flow integration costs) is trusted to
  be consistent, as a snapshot of your own memory is. An inconsistent one cannot index out of range or hang a bounded call, but
  it can make a path query report `InvalidOperationException` ("direction field contains a cycle", "has a cost but no path").
* A graph state names its node count outright (isolated nodes cost no bytes), up to the 2^26 `new Graph(n)` allows: restoring allocates nothing for them, but the first compile or query does.
* Must not overlap a running query or an edit on the same object (as for edits in general).

## Byte format (version 1)

Little-endian throughout (`BinaryPrimitives`); `i32` is a signed 32-bit integer, `f64` a binary64 written by its bit pattern, `u8 flag`
is 0 or 1 (any other value is an error). Counts are `i32` and must be in range.

Header (8 bytes): `u32 magic = 0x5347504F` (the bytes `4F 50 47 53`, "OPGS"), `u16 format version = 1`, `u16 kind`.

| kind | object |
|---|---|
| 1 | Grid |
| 2 | HexGrid |
| 3 | VoxelGrid |
| 4 | Graph |
| 5 | GridReplanner |
| 6 | HexReplanner |
| 7 | GridFlowField |
| 8 | AStar |
| 9 | JumpSearch |
| 10 | ThetaSearch |
| 11 | GraphAStar |

After the payload nothing may follow.

### Pieces

* **Space** (GridSpace): `f64 originX, f64 originY, f64 cellSize`. All three zero is the unassigned default (behaves as unit cells);
  otherwise the same range rules as the constructor apply (|origin| ≤ 2^21, 0 < cellSize ≤ 2^21). VoxelSpace has four values, `originX, originY, originZ, cellSize`.
* **Areas**: `i32 version` (≥ 0), then 255 × `f64` multiplier for areas 0..254 (each > 0 and finite). Area 255 is the wall and is not stored.
* **Heap `OpenSet`** (F, G, node): `i32 count`, then per slot in heap order `i32 node, f64 F, f64 G`. Slots are written as they lie in
  the array so a restored heap behaves exactly as the original; nodes must be distinct and F, G not NaN.
* **Heap `ReplanOpenSet`** (key1, key2, node): `i32 count`, then per slot in heap order `i32 node, f64 key1, f64 key2`.
* **Search buffers** (over `nodeCount` nodes): `i32 visited`, then per visited node in increasing index `i32 node, u8 closed, f64 g, i32 parent`
  (parent −1 or a node); then the open heap (`OpenSet`); then `i32 expandedCount` and that many `i32` nodes. "Visited" means seen by the
  current search; the buffers' epoch counter is not saved because it never affects a result.

### Objects

* **Grid** (1): `i32 width, i32 height, i32 version, Space, Areas, u8[width*height] cells` (row-major). Restore requires the same width and height.
* **HexGrid** (2): `i32 width, i32 height, i32 layout, i32 version, Space, Areas, cells`. Same width, height and layout required.
* **VoxelGrid** (3): `i32 sizeX, sizeY, sizeZ, u8 defaultArea, i32 version, VoxelSpace, Areas`, then for each of `chunksX*chunksY*chunksZ`
  chunks (x fastest, then y, then z) `u8 flag` and, when 1, 4096 bytes (index `x | y<<4 | z<<8` within the chunk, padding past the
  grid edge included). Same size and default area required.
* **Graph** (4): `i32 nodeCount` (0..2^26), `i32 edgeCount`, then per edge in id order `i32 from, i32 to, f64 weight, u8 alive`
  (endpoints < nodeCount, weight ≥ 0 and finite).
* **Replan search**: `i32 nodeCount, u8 mode (0 LPA*, 1 D* Lite), i32 start, i32 goal, u8 initialized, u8 found, i64 iterations, i64 expanded, i32 lastExpanded`.
  When initialized: `f64 km, u8 overflow, i32 pendingCount` and the pending entries (`i32`: node, or the bitwise complement of the node when its neighbours are to be re-examined too;
  none when the overflow flag is set), `i32 plannedCount` and per node with a finite g or rhs, in increasing index, `i32 node, f64 g, f64 rhs`, then the `ReplanOpenSet`.
  An uninitialised search (never planned, or restarted by `InvalidateAll`, `SetGoal`, or a start move under LPA\*) stores nothing more, and reads back with g and rhs at +∞: its old values are never used because planning overwrites them.
* **GridReplanner / HexReplanner** (5, 6): `i32 width, i32 height, u8 algorithm (GridAlgorithm value), i32 areaVersion, i32 start, i32 goal`, then the replan search.
* **Flow field search**: `i32 nodeCount, u8 status (0 not started, 1 running, 2 complete), i32 settled, u8 repairing, i32 goalCount` and goal nodes (strictly increasing),
  `i32 reachedCount` and per node with a finite cost in increasing index `i32 node, f64 cost`, `i32 stepCount` and per node with a step, in increasing index, `i32 node, i32 next`
  (the next node must be reached and cost no more than the node), the `OpenSet`, and when repairing `i32 touchedCount` and that many distinct nodes.
* **GridFlowField** (7): `i32 width, i32 height, i32 areaVersion, i32 goalCount` and the goal nodes in the order given at creation (must equal the field's own), `i32 pendingCount` and the pending edited cells, then the flow field search.
* **A\*, jump point, Theta\*, graph A\*** (8–11): `i32 nodeCount, u8 status (SearchStatus), f64 heuristicWeight, i32 start (−1 if unused), i32 goal, i32 expanded, i32 iterations, f64 pathCost`;
  unless the status is NotStarted, the search buffers. The weight must equal the restoring search's (A\* and Theta\*; 1.0 for the others). Every visited node's parent must be -1 or a visited node that costs no more, with no cycles.

## What one iteration is

Every sliced form counts iterations, never time; a budget of *n* does the same work on every machine, and sliced = unsliced for every budget (tested).

| Family | One iteration | Not counted |
|---|---|---|
| A\*, voxel A\*, Theta\*, Lazy Theta\* | one open-list pop (the goal's pop included) | line-of-sight checks, relaxing neighbours |
| JPS, JPS+ | one pop of a jump point | the scans that find jump points |
| HPA\* | one pop in the abstract graph (temporary start and goal nodes included) | building or refreshing the hierarchy, refining the path, smoothing |
| LPA\*, D\* Lite (`Replan(budget)`) | one queue pop, including a pop that only re-keys a stale entry | applying pending edits |
| Flow field (`Step(n)`, `IterationBudget`) | one settled cell | the invalidation walk that starts a repair; the final direction pass |
| Graph A\*, Dijkstra | one pop | |
| Bidirectional graph search | one pop from one side, sides alternating | |

Tier-1 `GridOptions.IterationBudget`, `VoxelOptions.IterationBudget` and `GraphOptions.IterationBudget` (0 = unlimited) are a single
cap: the query stops with `PathStatus.BudgetExhausted` and is not resumable. To resume, use the tier-2 `Begin`/`Step` form, whose state can also be saved.

## Debug geometry

`RecursiveAI.OnePath.Debug.GridDebug.Describe(...)` copies a result into immutable plain data:

| Input | Data |
|---|---|
| `GridPath` (A\*, JPS, JPS+, Theta\*, Lazy Theta\*, HPA\*, hex, replanner results) | algorithm, status, cost, path cells and world points, expanded cells in order, expanded count |
| `VoxelPath` | the same for voxels |
| `GraphPath` | nodes, positions, expanded nodes, cost, algorithm |
| `GridFlowField` | integration costs, next cell per cell, goals, status, settled (a sliced run shows its progress) |
| `HpaHierarchy` | abstract nodes (entrance cells) with their chunk, intra-chunk and inter-chunk edges with costs |

Expansion lists are opt-in (`RecordExpansions` on `GridOptions`, `VoxelOptions`, `GraphOptions`), so the happy path pays nothing; when
recorded, the list's length equals `ExpandedCount`. What an "expanded cell" is: the cell popped and expanded (A\*, voxel, Theta\*); the jump point popped (JPS);
the abstract node's cell (HPA\*); for a replanner result, each node that replan settled (overconsistent) or reset (underconsistent), in order.
