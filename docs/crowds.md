# Path corridors and path following

Normative behaviour of `PathCorridor`, `CorridorReplan`, `PathFollower` and `FollowerOptions` (namespace
`RecursiveAI.OnePath.Crowds.Corridors`) over a `TiledNavMesh` (`navmesh-format.md`) and a `NavMeshQuery`
(`navmesh-queries.md`). Everything here is deterministic: results depend only on the mesh, the inputs and the options, never on
platform, thread count or time. Arithmetic is IEEE `+ − × ÷ sqrt` on double (positions are world doubles). There is no clock:
the caller passes dt, and every budget counts steps or search iterations.

The corridor idea (an agent keeps the polygon route still ahead of it, trimmed as it moves and repaired as the mesh changes, with
corners from the funnel algorithm) follows the public talks and writing of M. Mononen; the funnel algorithm is Lee and Preparata
1984 (simple stupid funnel as in `navmesh-queries.md`); arrival steering is Reynolds, *Steering Behaviors for Autonomous
Characters*, 1999. Every rule below that those sources leave open is decided here.

## Inputs and errors

- Positions are `Vector3D` doubles. A non-finite coordinate or one with |v| ≥ 2^21 throws `ArgumentOutOfRangeException` naming the
  argument. dt must be positive and finite (same exception). Options out of range throw the same exception naming the field.
- Stale, forged or rejected polygon references never throw: the call returns false or a status and changes nothing (or, for
  `Validate`, trims or reports).
- Objects are not thread-safe. The query object and filter are arguments of every call and are never stored. A search in progress
  inside a query (`CorridorReplan`) must not be disturbed by other uses of that query.
- Steady-state steps allocate nothing (checked by a test).

## Corridor state

A corridor holds: a capacity (2 to 4096, default 256), a list of polygon references (`Path`, at most capacity, no polygon twice,
each connected to the previous), the agent position (on the surface of the first polygon), the target (world point), the target
polygon (or invalid), and three flags: `IsComplete` (the last polygon is the target polygon), `NeedsReplan`, and a private
*replan from start* flag (set by `SetTarget`, `Relocate` and `RequestReplan`).

"Connected" means: the next polygon is entered through a shared in-tile edge (`EdgeNeighbors`), a tile-portal link, or an
off-mesh link whose own flags and area the filter accepts; and the filter accepts the next polygon.

## Moving the agent

`MovePosition(to)` traces `MoveAlongSurface` from the first polygon at the current position to `to` (sliding along a wall, so the
result is always on the mesh). Let *V* be the polygons it visited (first = the first polygon, last = the polygon holding the new
position). The new corridor is `reverse(V[j..last]) ++ path[i+1..]` where *i* is the largest corridor index whose polygon is in
*V* and *V[j]* is that polygon. So polygons walked are trimmed, a step into a side polygon leads back through the polygons walked,
and the result has no repeated polygon (no polygon of `path[i+1..]` is in *V*). A merge that crosses more polygons than the capacity
is refused (nothing changes, returns false); an end that no longer fits is cut and the corridor becomes incomplete with a replan
requested.

`MoveTarget(to)` does the same at the other end (on the reversed corridor) from the target polygon; it is refused if the merge would
not fit. If the corridor is not complete the target is only recorded (target polygon cleared, replan requested).

`MoveAlong(to, limit)` places the agent at a point known to lie inside the first `limit` corridor polygons (the leg to a corner
does, by construction of the funnel): of the polygons whose outline holds the point horizontally (within 1e-7), the one whose surface
there is vertically closest to `to.Y` wins, and the furthest along on a tie (within 1e-9). The corridor is trimmed to start there
and the position becomes the surface point (`ClosestPointOnPoly`). It follows the corridor instead of tracing a ray, so a leg
that grazes polygon vertices or runs along a wall cannot be blocked by rounding at a vertex. False if no polygon in range holds it.

## Corners

`FindCorners` is `FindStraightPath(position, target, corridor)`: the first point is the position (flag `Start`), then portal
corners, the last point has flag `End` (the target when `IsComplete`, else the nearest point of the last polygon to the target).
An off-mesh link in the corridor gives an entry point (flag `OffMeshConnection`) and an exit point; the funnel restarts at the exit.
A point equal to the previous one merges its flags into it. `InvalidCorridor` is returned (nothing written) if a polygon is stale or
two neighbours no longer touch.

## Shortcuts (each bounded by a budget)

**Visibility** (`OptimizeVisibility(point, range, filter, budget)`). Cast a ray along the surface from the position toward `point`,
cut to `range` horizontally. The polygons crossed, at most `budget` of them (at least 2, at most the capacity), form a chain *C*
starting at the first polygon. Let *i* be the largest corridor index ≥ 1 whose polygon is in *C* at index *j* ≥ 1. Replace
`path[0..i]` by `C[0..j]`. Nothing changes if there is no such *i*, if the result equals the corridor, or if it would exceed the
capacity (a detour through more polygons than the corridor holds is not a shortcut). The shortcut trusts a straight ray, so it can
rarely lengthen the string-pulled path (a test counts: it shortens well over four times more often than it lengthens).

**Topology** (`OptimizeTopology(filter, budget, window)`). Run a path search (the query's A\*, see `navmesh-queries.md`) from the
first polygon at the position to polygon `path[t]`, *t* = min(count−1, window−1), ending at the nearest point of that polygon to the
target, with at most `budget` node expansions. If it finishes with the route, the corridor becomes `route ++ path[t+1..]` with loops
cut (a polygon met again cuts everything between its two occurrences, the first stays) and cut to the capacity (then
incomplete, replan requested). An unfinished search changes nothing. It needs at least 3 polygons; `window` ≥ 3.

## Mesh changes

`Validate(filter, lookAhead)` checks polygons `0 .. min(count, lookAhead)−1`: each must be a valid reference, accepted by the filter,
and connected to the previous one. The first failing index *k*: *k* = 0 → `Invalid` (nothing changes; relocate); *k* > 0 → the
corridor is cut to its first *k* polygons, marked incomplete, replan requested (`from start` flag cleared, so the replan splices at
the end of the valid part), `Trimmed`. If everything checked is fine and the whole corridor was checked while complete, a stale
target polygon makes it incomplete with a replan requested; otherwise `Valid`.

`Relocate(filter, extents)` replaces the corridor by the single nearest accepted polygon to the position within `extents` (default
(2, 4, 2)), moves the position onto it, keeps the target, requests a replan from start. False if none is near.

## Replanning

The target polygon is found afresh at the start of each replan: the accepted polygon nearest to `Target` within the search extents
(default (2, 4, 2)); the target itself is **not** moved (so a point on a boundary resolves the same way every time and a later
replan asks the same question a new query would). None → `NoEndNearby`.

Search 1 starts at the last polygon of the corridor (the end of the still-valid part) at the nearest point of it to the agent, unless
the *from start* flag is set (then search 1 is search 2). If search 1 cannot start or ends `Partial` (no route), **search 2** runs from
the first polygon at the position. A search is the query's sliced A\* (`BeginPath`/`UpdatePath`). The route of the search that ended
is spliced in: `path[0..e−1] ++ route` for search 1 (*e* = its start index), `route` for search 2, loops cut, cut to capacity.
Outcome:

| Search result | Corridor | Flags |
|---|---|---|
| `Success` | leads to the target polygon | complete, no replan |
| `Partial` (open list empty: no route) | leads to the expanded polygon closest to the target | incomplete, no replan |
| `InProgress` (budget used up) | leads to the closest polygon so far | incomplete, replan requested |
| cut to capacity | as above | incomplete, replan requested |

`CorridorReplan` splits one replan over calls: `Begin` then `Update(budget)` until not `InProgress`, then `Apply`. Total
iterations across both searches are counted; the result after any split of the same work equals one unlimited call (tested for
budgets 1, 2, 3, 5, 17, 100 against `int.MaxValue` on meshes with one-way links and edits, byte-exact). `PathCorridor.Replan(budget)`
is `Begin`, `Update(budget)`, `Apply`: when the budget runs out it applies the partial route (above).

## Following

`PathFollower` owns a corridor and the agent state: state, position, velocity, destination, counters. `Place(position)` puts it on
the nearest accepted polygon (state `Idle`). `SetDestination(target)` sets the target and replans from start in one call of
`ReplanBudget` iterations; the state becomes `Moving` (or `NoPath` when no polygon is near the target or the agent was not placed).

### Step(dt)

Only in state `Moving` (an off-mesh crossing has its own rule; other states set velocity to zero). In order:

1. **Mesh check.** `Validate(lookAhead = ValidateLookAhead)`. `Invalid` → `Relocate`; failing that, `NoPath`.
2. **Replan.** If the corridor needs one: one `Replan` call of budget `ReplanBudget · 2^rounds` (rounds = unfinished calls so far
   for this destination; capped at 2^20), then rounds += 1 if still `InProgress`. Reaching `MaxReplanRounds` → `NoPath`. The doubling
   guarantees that some call is large enough to leave a pocket that is closer to the target than its exits (a restarted search from
   the closest polygon cannot otherwise). A failed start (stale first polygon) relocates and the step ends; `NoEndNearby` → `NoPath`. Otherwise
   the step goes on with the corridor as the replan left it (a partial corridor is walked to its end, where the agent waits).
3. **Corners.** `FindCorners` into a buffer of `MaxCorners`; if the corridor does not hold together beyond the look-ahead it is
   validated in full (cut or relocated) and the step ends without moving.
4. **Shortcuts.** Every `OptimizeInterval` steps (counted since `Place`, 0 = off) a visibility shortcut aimed at the farthest corner
   within `OptimizeRange` that comes before an off-mesh entry (budget `OptimizeBudget`); every `TopologyInterval` steps a topology
   optimization (budget `TopologyBudget`). Corners are recomputed if the corridor changed.
5. **Off-mesh entry.** If the first corner (the position itself) or the next one is flagged `OffMeshConnection` and is within
   `OffMeshTriggerRadius` horizontally, the crossing begins (below) and the step ends.
6. **Arrival.** The steering corner is the first corner after the position that is more than 1e-9 away (or the last). If it is the
   last and within `ArrivalRadius`: if the corridor is incomplete and a replan is pending, the agent waits (velocity 0, stuck clock
   reset); otherwise the state is `Arrived`, `ReachedTarget` = `IsComplete`, velocity 0.
7. **Steering.** The wanted speed is `MaxSpeed`, or, if `SlowRadius` > 0 and the end (or an off-mesh entry) is among the corners,
   `max(MaxSpeed · min(1, d / SlowRadius), ArrivalMinSpeed)` with *d* the corner-polyline length to it (Reynolds' arrival). The
   wanted velocity points at the steering corner. The velocity moves toward it by at most `MaxAcceleration · dt` (in both speeding
   up and slowing down); the step length is |v| · dt.
8. **Walking.** Up to 4 hops: move toward the steering corner; if the step length reaches it, land exactly on it, subtract the
   distance, and (unless the corner is the end or an off-mesh entry, which are never passed) recompute the corners and continue
   with the rest. Each move is `MoveAlong(…, limit = index of the corner's polygon + 1)`, falling back to `MovePosition`; if both
   fail the agent is relocated. Landing on the end corner arrives (unless a replan is pending).
9. **Report.** `Velocity` = displacement / dt (3D). `RemainingDistance` = the length computed in step 7 minus the horizontal
   displacement, or +∞ when the end was not within the corners.
10. **Stuck.** Only at full speed (not while slowing). The clock accumulates dt; each `StuckWindow` seconds the horizontal
    displacement over the window must be at least `MaxSpeed · window · StuckProgress`. If not: recoveries += 1; more than
    `MaxStuckRecoveries` → state `Stuck` (velocity 0, until a new destination); otherwise the corridor is asked to replan from
    start. `StuckWindow` = 0 turns detection off.

### Off-mesh links

At the entry the corridor is trimmed to the entry's polygon and the position set to the entry point; the state is `OffMeshLink`;
start, end, exit polygon and duration (3D length ÷ `OffMeshSpeed`) are recorded. `Automatic`: each step adds dt to the elapsed time;
position = entry + (exit − entry) · elapsed / duration (velocity = displacement / dt); when elapsed ≥ duration the crossing finishes
(so it takes ⌈duration / dt⌉ steps). `Host`: nothing moves; the host calls `CompleteOffMeshLink` when its animation ends. Finishing
puts the agent at the exit point and trims the corridor to the exit polygon (if it is no longer in the corridor the agent is
relocated there), and the state returns to `Moving`. `SetDestination` during a crossing only records the target (a replan follows
the crossing).

### Defaults (`FollowerOptions.Humanoid`; `Small` = speed 2.5, acceleration 6, slow radius 0.5, off-mesh speed 2.5)

`MaxSpeed` 3.5, `MaxAcceleration` 8, `SlowRadius` 1, `ArrivalMinSpeed` 0.25, `ArrivalRadius` 0.01, `MaxCorners` 8,
`ValidateLookAhead` 8, `OptimizeInterval` 4, `OptimizeRange` 10, `OptimizeBudget` 16, `TopologyInterval` 16, `TopologyBudget` 32,
`ReplanBudget` 2048, `MaxReplanRounds` 16, `StuckWindow` 1, `StuckProgress` 0.1, `MaxStuckRecoveries` 2, `OffMeshMode` Automatic,
`OffMeshSpeed` 3.5, `OffMeshTriggerRadius` 0.05, `SearchExtents` (2, 4, 2), `CorridorCapacity` 256. They are starting points chosen
for a one-unit-per-metre world; none was tuned against a reference.

### What the tests establish (stated bounds)

- A follower reached every reachable goal in 750 random pairs on scattered-obstacle fields and 480 on two-floor meshes with
  off-mesh links, and ended exactly on the end point (within `ArrivalRadius`). The distance it walked was at most 1.08 × the funnel
  length of the fresh route (worst measured 1.043 on fields and 1.054 on two floors; 1.17 with budgets of 40 iterations per replan and
  shortcuts off). The excess comes from shortcuts and replans choosing other corridors; walking one corridor is exact.
- The corridor after any number of moves equals a fresh path query from the same polygon and position on perfect mazes (where the
  polygon route is unique), polygons and corners.
- Repair after tile removal, replacement and polygon edits reaches the target whenever a fresh query from the same polygon does.

### Limits

- Funnel corners are computed in the horizontal plane. Where two floors overlap horizontally and are joined at a point, the route can
  hairpin through the joint; the three-layer test mesh with a floor a quarter unit over the ground is such a world and is used for
  corridor invariants only. Floors a body-height apart (bridges, balconies) work.
- Budgeted replans that end unfinished restart from the closest polygon found; the budget doubles each round so they cannot stall,
  at the price of a longer search in a trap.

## State bytes

Little-endian; doubles by IEEE bit pattern; the same state gives the same bytes on every platform. Written by `Write`/`ToBytes`, read
by `Read` (validated: truncation, version, ranges, finite coordinates within ±2^21, flag bits and trailing bytes are errors,
`NavMeshFormatException`; a follower is unchanged if its read fails).

**Corridor** (`1 + 1 + 4 + 4 + 48 + 8 + 8·n` bytes): version u8 (= 1); flags u8 (1 complete, 2 needs replan, 4 replan from start); capacity
i32; count *n* i32; position 3 × f64; target 3 × f64; target polygon u64; *n* × polygon u64.

**Follower** (203 bytes + the corridor): version u8 (= 1); state u8; reached u8 (0/1); step count u64; replan rounds i32; stuck
recoveries i32; stuck clock f64; stuck anchor 3 × f64; position 3 × f64; velocity 3 × f64; destination 3 × f64; off-mesh start
3 × f64; off-mesh end 3 × f64; off-mesh elapsed f64; off-mesh duration f64; off-mesh exit polygon u64; remaining distance f64 (≥ 0,
+∞ allowed); then the corridor.

Not state: the options (the host rebuilds the follower with the same options before `Read`; a corridor of another capacity is
rejected), the corner buffer (recomputed at the start of every step), and a search being sliced through `CorridorReplan` (take
snapshots between steps; unfinished sessions are dropped). Because a step's replan round is complete within the step and its
counters are state, a snapshot taken at any step boundary resumes identically (the hash scenario `crowds/corridor-follow-edits`
restores and reruns and writes both).

A host that snapshots the mesh (`navmesh-snapshot.md`) and its followers together can roll both back and resimulate.

## Debug geometry

`FollowerDebug.Collect(query, follower)` returns plain data: state, position, velocity, destination, the corridor polygon outlines,
the corners (position, flags, polygon) and the current off-mesh crossing.
