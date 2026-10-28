# Graphs: generic and point graphs, A*, bidirectional search, ALT

The tier-1 types (`Graph`, `PointGraph`, `GraphOptions`, `GraphPath`, `Landmarks`, `LandmarkOptions`, `GraphEdge`, `Point3`) live in the root namespace
`RecursiveAI.OnePath`; the CSR form, searches and heuristics (tiers 2 and 3) live in `RecursiveAI.OnePath.Graphs`. Graphs are their own family: grids do not route through these types.
Everything here follows the same determinism rules as the grids: IEEE-exact `+ - * / sqrt` on double, ties broken on
node ids, no hash-collection order, no clocks. A port that follows this text reproduces the same paths, costs and
expansion orders bit for bit.

## Layout

- `GraphEdge(from, to, weight)`: a directed edge. Weights are finite doubles, 0 or more (zero-weight edges are allowed).
  `Point3(x, y, z)`: a position; `Z = 0` for plane graphs.
- `CsrGraph` (tier 3): immutable compressed sparse rows. Node ids `0..N-1`, at most 2^26 nodes. Out-edges of `v` are slots
  `offsets[v]..offsets[v+1]-1` of `targets`/`weights`; the reverse adjacency (`reverseOffsets`, `sources`, `reverseWeights`)
  is stored alongside. **Order:** a stable counting sort, so within one node the edges appear in input-list order, in both
  directions. Parallel edges and self-loops are kept. Construction validates endpoints and weights (typed
  `ArgumentException` naming the edge index).
- `CsrGraph.Fingerprint()`: 64-bit FNV-1a (offset 14695981039346656037, prime 1099511628211) over the little-endian bytes,
  low byte first, of: node count; every offset; then for every edge slot its target followed by the IEEE bit pattern of its weight.
- `Graph` (tier 1): a mutable edge list that compiles to a `CsrGraph`. Node ids are stable (nodes are never removed);
  every edge gets an id (its position in the list of all edges ever added) that is never reused. Removing an edge or changing
  a weight marks the compiled form stale; the next query, or `Compile()`, rebuilds it in O(nodes + edges) from the live
  edges in id order. `Graph.From(edges, nodeCount = -1, directed = false)` adds each listed edge in both directions unless
  `directed`; the reverse of edge `i` has id `2i+1`.
- `PointGraph` (tier 1): positions plus a `CsrGraph` whose weights are straight-line distances; immutable.

## Search

`GraphAStar<TCost, THeuristic>` is A* (Hart, Nilsson & Raphael 1968); with the zero estimate it is Dijkstra. It is the
grid search's loop transplanted onto CSR, and it uses the grid core's `OpenSet` and `GridSearchBuffers` unchanged:

- Open list order: smaller `f`, then larger `g`, then smaller node id (see `grids.md`, *Open set*).
- `g[start] = 0`; push `(h(start, goal), 0)`. An iteration is one pop. The goal test happens at pop.
- Expanding `n`: for each out-edge slot in CSR order, `c = cost(n, v, weight)`; skip the edge if `c` is NaN, negative or
  +infinity; `g' = g[n] + c`; if `v` was never reached or `g' < g[v]` strictly, then compute `h = estimate(v, goal)`; if `h`
  is +infinity (the goal cannot be reached from `v`) skip `v`; otherwise set `g[v] = g'`, `parent[v] = n`, push
  `(g' + h, g')`, reopening `v` if it was closed. Equal-cost alternatives keep the first parent found.
- `ExpandedCount` counts expansions (the goal's pop is not one); `Iterations` counts pops.
- Sliced search: `Step(n)` does at most `n` iterations; any sequence of budgets gives the same result as one run.
  Tier-1 `GraphOptions.IterationBudget` ends the query with `PathStatus.BudgetExhausted` after that many iterations.
  `Run(token)` checks cancellation every 256 iterations.

### Estimates

`IGraphHeuristic.Estimate(source, target)` is a lower bound on the distance from `source` to `target`; +infinity means
"unreachable". Admissible and consistent estimates give optimal paths. Tier-1 combines every estimate that exists by taking
the maximum (a maximum of consistent estimates is consistent): the straight-line distance of a point graph, the ALT bound
of `GraphOptions.Landmarks`, and `GraphOptions.Heuristic`. Straight-line distance is consistent only while every edge
weight is at least the distance between its endpoints (true for the weights `PointGraph` builds, not for an `EdgeCost` hook
that lowers them). `Algorithm = Auto` runs A* when an estimate exists and Dijkstra otherwise; asking for an A* variant
without any estimate runs the Dijkstra variant, and `GraphPath.Algorithm` reports what ran.

## Bidirectional search

`BidirectionalGraphSearch` runs a forward search from the start over the out-edges and a backward search from the goal
over the in-edges (cost hooks are always called with the edge's true direction), each in its own `GridSearchBuffers`.

- **Potentials** (Ikeda et al. 1994; Goldberg & Harrelson 2005): `pf(v) = (h(v, goal) - h(start, v)) / 2`, computed as
  `0.5 * (ht - hs)`. The forward key is `g + pf`, the backward key is `g - pf`; with the zero estimate both are plain
  Dijkstra keys (bidirectional Dijkstra, Pohl 1969). A node for which either estimate is +infinity cannot lie on a
  start-goal path and is never pushed. Both heuristics are evaluated, so the estimate must bound distances in both directions.
- **Turns:** strict alternation, forward first. One iteration is one expansion of either side. A side expands as in the
  unidirectional search (same open set order, same strict-improvement rule, same reopening).
- **Meeting:** whenever a relaxation improves `g[v]` on one side and the other side has reached `v`, the candidate
  `mu' = g_this[v] + g_other[v]` replaces the best cost `mu` if strictly smaller (so the first-found of equal candidates
  stays); `meet = v`.
- **Stopping rule:** before every expansion, stop when either open set is empty (every node that side can reach is
  settled), or when `peekF(forward) + peekF(backward) >= mu`. For consistent estimates this is exact: no unsettled path can be
  cheaper than `mu`. Stopping at the first meeting would be wrong and is not done. If `mu` is infinite at the stop there is no path.
  The check precedes the budget test, so finishing never consumes budget and slicing cannot change the result.
- **Path:** the forward parent chain from `meet` back to the start, then the backward parent chain from `meet` to the goal.
- `start == goal` is found at once with cost 0.
- With fractional weights the sums `g + p` and the stopping comparison are floating-point: costs equal Dijkstra's to within
  rounding (a few ulps). With weights that are integers (or any dyadic values) the results are exactly equal.

## ALT

Goldberg & Harrelson, *Computing the Shortest Path: A\* Search Meets Graph Theory*, SODA 2005. `Landmarks` stores, for
each landmark `L`, `d(L, v)` (forward Dijkstra) and `d(v, L)` (Dijkstra over the reverse edges) for all `v`, as doubles
(+infinity when unreachable). Layout `table[l * N + v]`.

- **Bound:** `LowerBound(u, v) = max(0, max over L of max(d(L,v) - d(L,u) [if d(L,u) finite], d(u,L) - d(v,L) [if d(v,L) finite]))`.
  Admissible and consistent; works for directed graphs; the result is +infinity when the table proves `v` unreachable from `u`.
  Landmarks are used in table order and all of them are always used.
- **Shortest-path trees** use `OpenSet` with key `(d, d)`, so equal distances settle by smaller node id; a node's parent is
  the first predecessor that gave it its final distance (strict improvement only).
- **Selection** (`SeededRandom(seed)` provides the roots; `NextInt(N)` per landmark, drawn even when not needed):
  - *Farthest:* the first landmark is the node with the greatest finite distance from a random root (ties: smaller id); each
    next one is the node (not yet a landmark, finite distance) with the greatest distance to its nearest landmark in the
    forward tables (ties: smaller id).
  - *Avoid* (the default): grow the forward shortest-path tree from a random root `r`; weight of `v` is
    `d(r, v) - LowerBound_current(r, v)`; `size(v)` is the sum of weights in the subtree of `v`, or 0 (blocked) if the
    subtree holds a landmark; the heaviest unblocked subtree root (ties: smaller id) is chosen, then descend to the child
    with the greatest size (ties: smaller id) until a leaf; the leaf is the landmark.
  - If no candidate exists (nothing reachable, all weights zero) the smallest-id node that is not yet a landmark is used.
  - The count is capped at the node count; landmarks are distinct.
- **Serialization** (`Serialize`/`Deserialize`), little-endian: `"OPLM"`, u32 format version `1`, i32 node count, i32
  landmark count, u64 graph fingerprint, i32 landmark ids, forward table, backward table (f64 bit patterns, landmark-major).
  `Deserialize(bytes, graph)` is strict: exact length, count in `1..N`, ids in range, fingerprint equal to the graph's,
  every distance `>= 0` or +infinity (NaN and negatives rejected), landmark-to-itself distance 0. Anything else is a `GraphStateException`.
- **Staleness:** a table belongs to the graph it was built for. A query with `GraphOptions.Landmarks` compares the table's
  graph (same object, or same node count and fingerprint) with the queried graph and throws `InvalidOperationException` on a
  mismatch, so edits to a `Graph` after building landmarks are caught. Tables assume edge costs are not below the stored
  weights; an `EdgeCost` hook that lowers costs voids the guarantee.

## Point graphs

- **Coordinates** obey the coordinate rule: finite and within ±2^21 on every axis, otherwise `ArgumentOutOfRangeException`
  naming the point, the axis, the value and the bound. Distance is `sqrt((dx*dx + dy*dy) + dz*dz)` evaluated exactly in that order.
- **Link sources** (all optional, merged): explicit pairs; every pair within `MaxDistance` (inclusive; compared as squared
  distance); each point's `NearestCount` nearest points (with `MaxDistance` also set: only those within it). Self-links
  and duplicate pairs are dropped; every pair becomes two edges weighted with the distance. Pairs are sorted by
  (smaller index, larger index) and emitted as `a -> b` then `b -> a`, so edge order is canonical.
- **Nearest:** by (squared distance, index), ties to the smaller index, found by rings of grid cells; cell size is
  `max(2^-30, extent / max(1, floor(sqrt(N))))` (1 for zero extent). After scanning rings `0..r` everything strictly closer
  than `r * cell` is known, so only such candidates are examined, in order. Links are made mutual.
- **Validation delegate** `LinkValid(a, b)` (a line-of-sight raycast, say) is applied to generated links only, with the
  lower point index first. With `NearestCount` it filters candidates in (distance, index) order until `k` valid ones are
  found, so a point near a wall links to its nearest visible points. It is called in a fixed order, so a deterministic delegate yields a deterministic graph.
- **Permutation:** the set of links (as pairs of positions) is the same for any ordering of the input points when
  `MaxDistance` or explicit links are used; with `NearestCount` it is the same unless two candidates for a point's last slot
  are at exactly equal distance, where the smaller index wins. Spatial hashing uses exact integer cell coordinates and
  only looks cells up, never enumerates them. `MaxDistance` is 0 (off) or at least 2^-30.
- `NearestNode(p)`: linear scan, smaller id on ties. `FindPath(Point3, Point3)` snaps both ends with it.

## API tiers

1. `Graph.From(edges).FindPath(a, b)`, `PointGraph.From(points, options).FindPath(a, b)` (or by position), with
   `GraphOptions` carrying plain delegates: `EdgeCost(from, to, weight)`, `Blocked(node)`, `Heuristic(from, to)`, plus
   `Algorithm`, `Landmarks`, `IterationBudget`, `RecordExpansions`; presets `GraphOptions.Default`, `Bidirectional` and
   `WithLandmarks(table)`; `graph.BuildLandmarks(LandmarkOptions?)` (count 16, avoid, seed 1 by default). No struct callbacks. The options are adapted onto one
   struct adapter per hook, so the happy path instantiates each search once.
2. `GraphAStar<TCost, THeuristic>` and `BidirectionalGraphSearch<TCost, THeuristic>` with `IGraphCost` / `IGraphHeuristic`
   structs (`DefaultGraphCost`, `ZeroGraphHeuristic`, `EuclideanGraphHeuristic`, `AltGraphHeuristic`), caller-owned buffers
   and `Begin`/`Step`/`Run`.
3. `CsrGraph`, `Landmarks`, `ShortestPathTree.Compute`, `OpenSet`, `GridSearchBuffers`.

Tier-1 `FindPath` reuses pooled working memory (one pair of buffers per graph, taken and returned atomically), so a
steady stream of queries allocates only the result. Concurrent queries on an unedited graph are safe; edits must not overlap queries.

## Verification

- Every variant (Dijkstra, A*, bidirectional Dijkstra, bidirectional A*) with no estimate, landmarks, a geometric estimate and
  both equals an independent O(V^2) Dijkstra on 12,000 generated graphs (sparse directed with zero weights, dense,
  grid-like, road-like), with paths checked edge by edge; fractional-weight point graphs agree to 1e-9 relative.
- Landmark bounds never exceed true distances and are consistent on 200 graphs; serialization round-trips and hostile bytes
  give `GraphStateException`.
- Sliced equals unsliced for several budgets; link building equals brute force and is permutation-stable.
- Hash-matrix scenarios `graphs/*`; fuzz targets `graph-query`, `graph-landmarks`, `graph-points`.
- `tools/GraphRoad` prints expansion counts for the four searches on a DIMACS road graph (`--gr`) or a generated planar road-like graph (`--generate`).
