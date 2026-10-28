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

The route is taken only if it is *shorter to walk*: the string-pulled length from the position to the same end point through the new
polygons must be less than the old length times (1 − 10⁻⁴). A search that costs less is not always shorter to walk (a polygon-node
search charges polygon centres, and which way round an obstacle it prefers can change with the agent's place in its polygon): without
this test the corridor flipped between two ways round and the agent stood between them (4 of the 1000 seeded crowd runs).

**Local query.** When the follower's query is a polygon-node query, topology searches (and the refine below) run on a crossing-node
query of the same mesh instead, created once per follower (per worker in a `Crowd`) with `clamp(8 · max(RefineBudget, TopologyBudget), 1024, 2²⁰)` nodes
(8192 in a `Crowd`). A crossing-node query is used as is. The routes of the two models are the same search with different node costs, so the
choice is only about which route is shorter to walk.

**Refine.** When a replan on a polygon-node query ends `Success`, one topology search with budget `RefineBudget` (default 128, 0 = off) and
window 64 runs at once, under the same shorter-to-walk test. Over 725 random pairs on scattered obstacles the polygon-node route was up
to 1.41 × the shortest walk (worst of the three pairs over 1.08); after the refine none is over 1.08. Two-floor meshes need a budget of 2000.

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
| `InProgress` (budget used up) | leads to the closest polygon so far | incomplete, replan from start requested |
| cut to capacity | as above | incomplete, replan requested |

**An unfinished search is not walked.** The closest polygon so far is a guess, and the next round does not continue from its end (that would
splice two searches and the route would not be the shortest): the next round searches again from the agent's polygon with twice the budget
(`ReplanBudget << rounds`), and the agent stands still while a search from the start is unfinished (`rounds > 0`, corridor incomplete,
replan requested). With 40 iterations per replan the worst walked/route over 100 pairs fell from 1.33 to under 1.2; with the default budget of
2048 the case does not arise on small meshes.

In a `Crowd` step (not in `PathFollower.Step`), a replan requested only because the corridor was cut to capacity waits until the corridor holds at most three quarters of its capacity (the agent has walked a quarter of it; corridors of fewer than 16 polygons always extend at once); it is not repeated every step. A replan from start, or one after a mesh change that left a short corridor, runs at once.

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
`OffMeshSpeed` 3.5, `OffMeshTriggerRadius` 0.05, `SearchExtents` (2, 4, 2), `CorridorCapacity` 256, `RefineBudget` 128, `CornerClearance` 0. They are starting points chosen
for a one-unit-per-metre world; none was tuned against a reference.

### What the tests establish (stated bounds)

- A follower reached every reachable goal in 750 random pairs on scattered-obstacle fields and 480 on two-floor meshes with
  off-mesh links, and ended exactly on the end point (within `ArrivalRadius`). The distance it walked was at most 1.08 × the funnel
  length of the fresh crossing-node route, on both node models (polygon-node with `RefineBudget` 2000), and at most 1.2 × with budgets of 40 iterations per replan and
  shortcuts off. The excess comes from shortcuts and replans choosing other corridors; walking one corridor is exact.
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

# Crowds: `Crowd` and `Agent`

Normative behaviour of `RecursiveAI.OnePath.Crowd`, `Agent`, `AgentOptions`, `CrowdOptions` (root namespace) and `CrowdDebug`. A crowd walks one
`TiledNavMesh`; every agent owns a `PathFollower` (above) and is steered by ORCA (van den Berg, Guy, Lin and Manocha, *Reciprocal n-body
collision avoidance*, 2011; `orca.md`) against the neighbouring agents and the boundary edges of the mesh around it. Arithmetic is
IEEE `+ − × ÷ sqrt` on double. There is no clock, no random number and no sub-stepping: `Step(dt)` is one step of `dt`.

## Agents and slots

Agents live in *slots* 0, 1, 2, …; `Add` takes the lowest free slot (so the slot order is a function of the sequence of `Add` and `Remove` calls),
`Remove` frees it. `Agent.Id` counts agents ever added (0, 1, …) and is never reused. Everything below is evaluated in slot order. `Add` places the
agent with `PathFollower.Place` (the nearest accepted polygon within the follower's search extents; an agent with no polygon near is
refused with `ArgumentException`), standing still with no destination. The crowd's `Filter` (null = everything, cost 1) is the filter of every
follower and wall query.

## Step

Three phases, each over the slots in fixed groups of 32 (worker *w* takes groups *w*, *w* + threads, …). In a phase an agent writes only its own
slot and reads the state the phase began with, so the result is the same for any `MaxThreads` (and for a worker that happens to run late):

0. **Snapshot** (single thread): every agent's position and velocity (the velocity it ended the last step with); the neighbour grid.
1. **Plan** (per agent): an `Arrived` agent that was pushed farther than `2 · ArrivalRadius + 0.01` from its destination is sent back (`SetDestination`). Then
   `PathFollower.PlanCrowd`: the corridor upkeep of `Step` (steps 1 to 6 of the follower's step: validation, replans, shortcuts, off-mesh entry, arrival),
   and the wanted velocity in X/Z, `MaxSpeed` toward the steering corner slowed by the same arrival rule as step 7 (no acceleration limit).
   An intermediate corner is aimed at `CornerClearance` past the corner: from the corner, along (unit vector to the corner − unit vector from the
   corner to the next one) normalised, skipped when that is shorter than 10⁻⁶ (no bend) or the corner is an off-mesh entry. A corner is a wall
   vertex and wall avoidance keeps `WallClearance` from it, so an agent aimed exactly at it slowed toward it and stopped short. A crowd sets
   `CornerClearance = 2 · WallClearance + 0.01` (0.11); a lone follower leaves it 0. An agent in an off-mesh
   crossing runs the whole crossing step here (no avoidance while crossing). An agent that is not walking (idle, arrived, stuck, no path) wants 0. Then the wall
   lookup (below).
2. **Avoid** (per agent; `AvoidanceQuality.Off` and crossing agents skip it and keep the wanted velocity): neighbours, separation, side bias, constraints, solve.
3. **Move** (per agent): the new velocity is limited to a change of `MaxAcceleration · dt` from the start-of-step velocity; a walking agent then moves with
   `PathFollower.ApplyCrowd`, others with `Nudge`. `ApplyCrowd` moves by velocity · dt along the surface: `MoveAlong` within the corridor up to the steering
   corner's polygon, else `MovePosition` (a ray along the surface that slides along walls), so the agent is always on the mesh. It never walks past the
   end or an off-mesh entry (it lands on it), reports velocity = displacement / dt and the remaining distance, and arrives or counts stuck time as `Step` does.
   `Nudge` pushes a standing agent the same way (`MovePosition`) and reports the velocity that resulted.

## Neighbours

Agent *i* considers the `MaxNeighbors` nearest agents *j* ≠ *i* with squared centre distance < `NeighborDistance²`, ordered by (squared distance, slot) —
`Low` 4, `Medium` 8, `High` 12 neighbours; `NeighborDistance = max(SeparationDistance, 2 · Radius + MaxSpeed · TimeHorizon)`; `TimeHorizon` 1.5, 2, 2.5 s. The
set does not depend on the structure that finds it (the implementation sorts agents by (grid cell, slot) with cells at least as wide as the largest
`NeighborDistance` and scans the 3 × 3 cells around the agent; the grid is rebuilt every step and cell keys are 64-bit integers, never hashes).
Two agents at exactly the same place get a relative position of (±1e-3, 0), plus for the one with the higher slot, so they separate.

Constraint of *i* against neighbour *j*: `OrcaConstraints.ForAgent(pj − pi, vi, vj, ri + rj, TimeHorizon, dt, share)` with the start-of-step velocities and
`share = priority_j / (priority_i + priority_j)` (the other's weight over both, so the two agents' shares add to 1; `Priority` is a weight above 0 and a
higher priority gives way less), or `share = 1` when *j* is crossing an off-mesh link and cannot give way.

## Separation and side bias

If `SeparationDistance` > 0: each neighbour *j* of *i* with centre distance *d* < `SeparationDistance` adds `SeparationWeight · MaxSpeed · t²` to the wanted velocity
in the direction away from *j* (`t = 1 − d / SeparationDistance`; when *d* ≤ 1e-9 the direction is +X for *j* below *i* in slot order and −X otherwise); the sum is
clamped to `MaxSpeed`. Separation acts on the agent's wanted velocity before avoidance, on standing agents too.

The *side bias* (`SidePreference`, default 0.05) adds to the wanted velocity *p* the vector `(p.z, −p.x) · SidePreference`, a turn to one fixed side that breaks
the symmetry of head-on meetings (exact avoidance resolves a perfectly head-on pair by stopping). If a wall constraint of the agent lies within four radii of it
on that side (its normal points back against the bias direction by more than a half), the bias goes to the other side.

## Walls: the mesh boundary as obstacles

Wall pieces come from `NavMeshQuery.FindWalls` (`navmesh-queries.md`): the edges with nothing walkable behind them, within a radius of the agent, from the polygons
the circle search reaches. Each agent keeps a list of at most `MaxWallsPerAgent` pieces (the nearest to where it was looked up), looked up with radius
`reach + WallRefreshDistance` where `reach = ObstacleHorizon · MaxSpeed + WallClearance` and `ObstacleHorizon = TimeHorizon / 2`. The lookup is repeated when the agent
is farther than `WallRefreshDistance` from where it was done, when `TiledNavMesh.EditCount` changed since, or when the last lookup failed (the lookup is a function of
the mesh and the position it was done at; that position is state, see below).

In phase 2 the agent takes the pieces within `reach` of it that face it, nearest first (ties by polygon reference, edge, then the first end's X and Z), at most
`MaxWallsAvoided`. A piece faces the agent when `OrcaConstraints.ForBoundary` gives a half-plane (the agent is on its walkable side, within 1e-6, and projects on it more
than `WallClearance` + 1e-6 from both ends) and the piece is on the agent's floor (the height of the piece at its nearest point differs from the agent's by at most
`1 + 1.5 · distance`, so the walls of a floor above or below, joined by a link, are left out). The half-planes are the **hard** constraints of the linear
program (they come first); neighbour half-planes follow in neighbour order. The program is `OrcaLinearProgram.Solve` (`orca.md`).

The agent's centre may touch the boundary: the mesh was built for an agent radius and is already shrunk by it, so a wall constraint keeps only `WallClearance` (0.05)
from the line, and a path that grazes a corner (the taut string does) is not held back by the wall it passes. Because agents are moved *along the surface*, they
stay on the mesh even where the constraints let a corner be clipped.

**Blocked by a standing agent.** If the solved velocity is shorter than 0.3 of the wanted speed, the agent is more than eight radii from its destination and a neighbour is not walking, the
program is also solved for the wanted velocity turned +90°, −90°, +45°, −45° (constants `(−z, x)`, `(z, −x)`, `√½·(x − z, x + z)`, `√½·(x + z, z − x)`, `√½ = 0.7071067811865476`)
and the fastest result (greatest squared length, first on a tie) is taken. Near its goal an agent joins the queue instead.

## Off-mesh links in crowds

`AgentOptions.ToFollowerOptions` sets `OffMeshTriggerRadius = max(follower default, 2 · Radius)`: a link end is attached to the mesh within a few radii, and agents queue
at it instead of having to stand within a hair of the point. The crossing itself is the follower's.

## Presets and defaults

`Agent.Humanoid`: radius 0.6, speed 3.5, acceleration 8, arrival radius 0.25, priority 1, quality `Medium`, wall clearance 0.05, separation off (distance 0, weight 0.3),
side bias 0.05. `Agent.Small`: radius 0.3, speed 2.5, acceleration 6, arrival radius 0.3. `Agent.Large`: radius 1.5, speed 3, acceleration 5, arrival radius 0.6. The follower options of
an agent are `FollowerOptions.Humanoid` (or `AgentOptions.Path`) with the agent's speed, acceleration and arrival radius, corridor capacity 64 and stuck detection
off (a crowd stands in queues; `Path` can turn it on). `CrowdOptions`: 65536 agents, 1 thread, no filter, 16384 query nodes per worker, 48 walls kept, refresh 1.0,
8 avoided, constraints not recorded. None of these was tuned against a reference.

## What the tests establish (stated bounds)

- 1000 seeded runs of 6 agents each on scattered-obstacle fields (passages at least 2 units wide), agents removed on arrival: every agent arrived within
  3× the time to walk the longest route at full speed plus 10 s (worst measured ratio of time taken to straight-route time at full speed 3.7); no agent position was ever off
  the mesh (checked every step); the deepest agent–agent overlap was 0.18 (combined radius 0.6), under 0.05 in all but 8 runs.
- Crossing streams (20, 60, 150 agents), crowds through a two-cell door (20, 40, 60) and 160 agents on two-floor meshes with off-mesh links (72 crossings): all arrive and stay on the
  mesh; overlap at most 0.034 (open scenes) and 0.30 for two-floor meshes (an agent leaving a link lands where it lands; crossing agents do not avoid).
- 1, 2, 5 and 16 threads give byte-identical agent states after 150 steps of 300 agents. 60 steady-state steps of 120 agents allocate nothing beyond 4 KB.

## Limits

- Standing agents are obstacles. In a passage less than about two body diameters wide an agent standing still (arrived) can block others, and two agents meeting
  head-on in such a passage can crawl; the tests that must arrive remove agents on arrival or keep goals at least 1.2 apart in passages at least 2 wide.
- Avoidance is on the mesh plane; agents on stacked floors are separated only by the height tolerance of walls and by distance (an agent above another on a different
  floor is still a neighbour in X/Z).
- `ApplyCrowd` follows the corridor, not the neighbours: an agent pushed far off its corridor is brought back by `MovePosition` and the follower's repair, not by a new search.

## State and snapshots (`Crowd.SaveState`, `RestoreState`, `SaveStateWithMesh`, `RestoreStateWithMesh`)

A host that rolls back gets the same crowd, bit for bit, as one that never left: the step after a restore equals the step after the
save, for any thread count, and saving again gives the same bytes.

**What is state.** Per crowd: the slot count (slots are never given back), the agent count, the lowest-free-slot hint, the next
agent id and the step count. Per active slot: the agent's id, its options, its priority (changeable at any time), the flags of the last
step (walking, crossing) and wanted velocity (both only for debug views: every step sets them before use), the follower's own bytes
(above; the corridor and a replan in progress are in them), and the wall cache.

**The wall cache is state, not an optimisation.** A cache is refreshed when the agent has moved more than `WallRefreshDistance` from
where it was filled or the mesh was edited, so the pieces an agent avoids depend on *where the cache was filled*. A restore that
dropped it would refill at the current position and the next steps would differ (a test that does this fails within a few steps).
The cache is written only while it is valid for the mesh as it is now (stamp equal to `Mesh.EditCount`): its centre and the pieces in
cache order. A stale cache is written as "stale" and refilled by the next step, exactly as the run that was saved would have done. On
restore a valid cache is stamped with the restored mesh's current edit count. **The mesh must therefore be restored to the saved
state first**; `RestoreStateWithMesh` does it in that order.

**Not state** (rebuilt by the step, never read before being written): the neighbour grid and its sort keys, the snapshot of positions and
velocities every agent sees, the solved velocities, the workers' query objects and buffers, the recorded constraint lines (they read
as empty until the next step). `CrowdOptions` are not saved; the crowd restored into is created with the same options (the filter is a
delegate and cannot be saved). `MaxWallsPerAgent` is checked against the bytes; `MaxAgents` bounds the slot count.
There is no clock, random number or other hidden state: a step reads only the above, the mesh and `dt`.

**Byte format, version 1** (little-endian, doubles by bit pattern; a flag is 0 or 1, anything else is an error):

| Field | Type |
|---|---|
| magic `4F 50 43 53` ("OPCS"), format version | u32, u16 = 1 |
| kind: 1 crowd, 2 crowd with mesh | u16 |
| `MaxWallsPerAgent` | i32 |
| slot count, agent count, lowest free slot | i32 ×3 |
| next id, step count | u64 ×2 (at most 2^63 − 1) |
| per slot, in slot order: active flag | u8 |
| — if active: id (below the next id, no two equal) | u64 |
| priority (above 0), flags (bit 0 active must be set; bit 1 walking, bit 2 crossing), preferred velocity x and z | f64, u8, f64 ×2 |
| agent options: radius, max speed, max acceleration, arrival radius, priority, quality (u8), wall clearance, separation distance, separation weight, side preference, path-settings flag, and if set every `FollowerOptions` field in declaration order (doubles as f64, ints as i32, off-mesh mode as u8, search extents as three f64) | see order |
| wall cache valid flag; if 1: centre x, z (f64 ×2), piece count (≤ `MaxWallsPerAgent`), per piece: ax ay az bx by bz (f64 ×6), polygon ref (u64), edge (i32) | u8, … |
| follower byte length, follower bytes (`PathFollower.Write`) | i32, bytes |

Kind 2 is: header, i32 mesh length, the mesh snapshot (`navmesh-snapshot.md`, in full or referenced mode), i32 crowd length, a kind-1 crowd state.

**Reading.** Every count is bounded by the options and the bytes left; coordinates are finite and within ±2^21; the options pass the
same validation as `Crowd.Add`; each follower is read by `PathFollower.Read` (its corridor must fit the capacity the options give);
slots below the lowest-free hint must be active; the active slots must number the agent count. The reader is structural: positions,
corridors and cached walls are not checked against the mesh (a snapshot of your own memory; stale ones are repaired by the steps),
but nothing indexes out of range. Failures throw `NavMeshFormatException` naming the field and byte offset. **A restore is all or
nothing**: everything is parsed into new objects first and nothing changes if any part is rejected (`RestoreStateWithMesh` also
checks the crowd part before touching the mesh, and the mesh restore is itself all or nothing).

**Span forms, size and hash.** `Crowd.GetStateSize()` is the length of the bytes above; `TrySaveState(Span<byte>, out int)` writes
exactly the bytes `SaveState()` returns (both run one code path, so they cannot drift) and reports `false`, writing nothing it
promises, when the buffer is short; `RestoreState` takes a `ReadOnlySpan<byte>` (a slice of any buffer). `Crowd.StateHash()` is
**FNV-1a, 64-bit** (offset basis `CBF29CE484222325`, prime `100000001B3`, per byte `h = (h xor byte) · prime` mod 2^64) over the kind-1
bytes, computed as they are produced without storing them. Equal hashes mean equal snapshots up to 64-bit collisions; the hash covers
what the snapshot covers (not the mesh, not `CrowdOptions`), is the same on every platform and for any thread count, and a host can
check it against `Fnv1a64(SaveState())`.

**Allocation.** `GetStateSize`, `TrySaveState` and `StateHash` allocate nothing, except that the first `StateHash` grows an internal
scratch buffer to the largest follower (it is reused afterwards). `RestoreState` validates the whole input first and then applies it,
so a rejected restore changes nothing. It allocates nothing when every active slot of the bytes holds, in the crowd, the agent with the
same id and byte-identical settings (the rollback case: same agents, different dynamic state) and the slot arrays already suffice; its
followers are read in place, corridor included. It allocates for each agent that is new to its slot (the `Agent`, its settings and its
follower), when the slot arrays must grow, and for the exception when bytes are rejected. `StateHash`, `GetStateSize` and
`TrySaveState` use crowd-owned scratch and must not run at the same time as each other or a step. `WorldCrowd` has no snapshot of its
own: a host calls these on each crowd of `WorldCrowd.Crowds`, in class order.

**Agent objects.** Agents that are in the snapshot and still in the crowd keep their `Agent` object and their follower object (read in
place). Agents added since are deactivated. Agents in the snapshot but removed since come back as new `Agent` objects with the same id
(`Crowd.GetAgent(id)`, `CopyAgents`): a removed `Agent` is never revived. The next added agent gets the rewound id and the lowest
free slot, as in the saved run.

**Order for a host** that also holds other state: restore the mesh (or use the combined call), then the crowd, then anything built
on them. Mesh edits made after the snapshot are undone by the mesh restore; agents' wall caches are valid again because the cache
stamp is set from the restored mesh.

## Debug geometry

`CrowdDebug.Collect(crowd)` returns, per agent in slot order, plain data: id, radius, the last wanted velocity, the follower's `FollowerDebugGeometry` (state, position,
velocity, destination, corridor outlines, corners, off-mesh crossing), the wall pieces looked up, and (with `RecordConstraints`) the half-planes solved in the last step
(walls first, then neighbours), each as a point and unit normal in velocity space.
