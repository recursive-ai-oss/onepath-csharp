# Grids: square and hex, open set, A*, area costs, clearance

Normative. A port must reproduce the same expansion order, costs and paths from this text. Code lives in
`src/OnePath/Grids/`. The tier-1 types (`Grid`, `HexGrid`, `VoxelGrid`, their options and results, `Cell`, `WorldPoint`, `VoxelCell`,
`VoxelPoint`, `GridSpace`, `VoxelSpace`, `AreaCosts`, `HexCoord`, `GridAlgorithm`, `PathStatus`, `GridFlowField` and the replanners) live in
the root namespace `RecursiveAI.OnePath`; the search cores, topologies, heuristics, buffers and indexes (tiers 2 and 3) live in
`RecursiveAI.OnePath.Grids` and `.Grids.Voxel`.

## Layout

- **Square grid** (`Grid`): `Width` x `Height` cells, one byte each, indexed row-major: `index = y * Width + x`.
  `x` is the column (0 at the left), `y` the row (0 at the top). At most 2^28 cells.
- **Area ids.** A cell byte is an area id: 0..254 are walkable areas, **255 is a wall**. `AreaCosts` holds one
  multiplier per walkable area (default 1; must be positive and finite). Edits go through `Grid.SetArea`, which bumps
  `Version` and tells registered `IGridChangeListener`s the changed cell.
- **Hex grid** (`HexGrid`): a rectangular offset array (`Width` columns, `Height` rows), same indexing, same area ids.
  `HexLayout` says which rows/columns are shifted: `PointyOddRow`, `PointyEvenRow`, `FlatOddColumn`, `FlatEvenColumn`.
  Axial coordinates (q, r) from offset (col, row):
  - pointy odd: `q = col - (row - (row & 1)) / 2`, `r = row`;
  - pointy even: `q = col - (row + (row & 1)) / 2`, `r = row`;
  - flat odd: `q = col`, `r = row - (col - (col & 1)) / 2`;
  - flat even: `q = col`, `r = row - (col + (col & 1)) / 2`.
  Hex distance: with `dq, dr, ds = -dq - dr`, `(|dq| + |dr| + |ds|) / 2`.

## World mapping and the coordinate rule

`GridSpace(originX, originY, cellSize)`. Square cell (x, y) has its centre at
`origin + ((x + 0.5) * size, (y + 0.5) * size)`; a world point maps to `floor((p - origin) / size)` per axis and is
outside the grid unless both results lie in range. Hex: the origin is the **centre of cell (0, 0)** and `size` is the
distance between adjacent centres; pointy layouts place axial (q, r) at `size * (q + r/2, (√3/2) r)`, flat layouts at
`size * ((√3/2) q, r + q/2)`; the inverse divides, then rounds fractional cube coordinates: round each of q, r, s = -q-r
with `floor(v + 0.5)`, then repair the coordinate with the largest rounding error so the three sum to 0 (q if its error is strictly larger than both others, else r if strictly larger than the error of s, else s, which is implied). `(√3/2)` is the double 0.8660254037844386.

World coordinates (origin, query points) cross the boundary as double and must satisfy `|v| <= 2^21` (exact for
Q31.32 fixed point). NaN, infinities and larger values throw `ArgumentOutOfRangeException` naming the value and the
bound. Internally grids are integer cell indices and costs are double; there is no float32 on the grid paths.
A point outside the grid (but within the bound) is not an exception: the query returns `StartInvalid`/`GoalInvalid`.

## Movement

**Square, neighbour order** (normative; it decides which of two equal-cost parents wins): north (0,-1), east (1,0),
south (0,1), west (-1,0), then, if eight-connected, north-east (1,-1), south-east (1,1), south-west (-1,1),
north-west (-1,-1). **Hex**: axial directions (+1,0), (+1,-1), (0,-1), (-1,0), (-1,+1), (0,+1).

**Diagonal rule** (square, eight-connected). For a diagonal move the two *beside* cells are the two cells that share an
edge with both the start and the target (for north-east: north and east of the start).
- `NoCornerCutting` (default; the Moving AI benchmark convention, Sturtevant 2012): both beside cells walkable.
- `NoSqueezing`: at least one beside cell walkable.
- `AllowAll`: no condition beyond the target being walkable.

**Step cost** of a move from cell `a` to cell `b`: `length * ((m[a] + m[b]) * 0.5)` evaluated in exactly that order
in double, where `m` is the area multiplier of the cell's area id and `length` is 1 for a straight step, `GridMath.Sqrt2`
for a diagonal (hex: always 1). A caller's cost hook may replace the result (see Tier 2). The cost of a path is the sum of its
step costs added in path order starting from 0.

**√2.** `Sqrt2 = 1.4142135623730951` (bits `0x3FF6A09E667F3BCD`), the correctly rounded double of √2; it equals
`Math.Sqrt(2.0)` because IEEE-754 square root is correctly rounded. Octile costs `a + b√2` are therefore exact up to
ordinary double rounding and identical on every platform. (The published Moving AI lengths differ from exact sums by up to
1.7e-9 relative; see Verification.)

## Clearance and agent sizes

An agent of size `s` (1..255 cells) occupies the `s x s` block of cells whose **top-left cell is the agent's position**
(extending towards +x and +y). **True clearance** of a cell is the side of the largest square of walkable cells with that
cell as its top-left corner (0 for walls; cells outside the grid count as walls; capped at 255). It is computed by
`c(x,y) = 0` for a wall, else `1 + min(c(x+1,y), c(x,y+1), c(x+1,y+1))` from the bottom-right corner, in O(cells).
A cell is walkable for size `s` iff `clearance >= s`. All movement rules above are then applied to the "walkable for
`s`" predicate: for a diagonal move the beside cells are tested for size `s`, which is exactly "the swept blocks are
free". The step cost uses the area ids of the two position cells only. Clearance is recomputed lazily after edits;
create a new `SquareTopology` after editing cells. Hex grids support only size 1.

## Open set

`OpenSet` is an indexed 4-ary min-heap over node indices with decrease-key. **Order: smaller `f` first; on equal `f`
larger `g` first; on equal `g` smaller node index first**, comparing doubles with `==`/`<`/`>`; NaN keys are rejected.
A node is in the set at most once (pushing an existing node replaces its keys and re-sifts), so the pop order depends
only on the keys. The arity is an implementation detail.

## A*

Hart, Nilsson & Raphael 1968. `f = g + w * h` with weight `w` (default 1); `h` from `IHeuristic` after `SetGoal(goal)`.

1. `g[start] = 0`, push start with `f = w * h(start)`.
2. **Iteration**: pop the best node `n` (an iteration is one pop); mark it closed; if `n == goal` finish (cost `g[goal]`);
   otherwise expand: visit the topology's neighbours in order; for each, `step = hook(n, m, baseCost)`; skip it if `step`
   is NaN, negative or +infinity; `g' = g[n] + step`; if the neighbour was never reached, or `g' < g[m]` strictly, set
   `g[m] = g'`, `parent[m] = n`, mark it open (reopening it if closed) and push `(g' + w*h(m), g')`.
   Equal-cost alternatives therefore keep the **first** parent found.
3. If the open set empties first, there is no path.

The goal test happens at pop. With `ZeroHeuristic` this is Dijkstra. Heuristics: Manhattan `scale*(|dx|+|dy|)`;
octile `scale*((max-min) + min*√2)` (with `dx, dy` integer cell distances); hex distance `scale*steps`; `scale` is the
smallest area multiplier (`AreaCosts.MinCost`), which keeps them admissible when multipliers are below 1. Heuristics are
admissible and consistent for the default cost model; hooks that lower costs, or weights above 1, give up optimality.
`Auto` picks Manhattan for four-connected, octile for eight-connected, hex distance for hex.

**Sliced search.** `AStar.Step(n)` performs at most `n` iterations and returns `Running` while work remains;
`Run()` is `Step(int.MaxValue)` with cooperative cancellation; time is never consulted. Any sequence of budgets gives
the same expansion order, path and cost as one unsliced run. The tier-1 `GridOptions.IterationBudget` ends the query with
`PathStatus.BudgetExhausted` after that many iterations.

## API tiers

1. **Happy path.** `Grid.From(bool[,])` (or spans, `Grid.FromText(picture)` with `.` open, `#` wall and `1`-`9` areas, or `MovingAiMap.ToGrid()`; `HexGrid.FromText` likewise), `grid.FindPath(a, b[, GridOptions])` with
   cells or world points; `GridOptions` (connectivity, diagonal rule, agent size, heuristic, weight, `StepCost` and `Blocked`
   delegates, expansion recording, iteration budget) with presets `Default`, `FourConnected`, `NoSqueezing`, `ForAgentSize(n)`.
   Start or goal outside the grid, blocked or too narrow for the agent give `StartInvalid`/`GoalInvalid`, not exceptions. The
   delegates are adapted onto the struct-generic core; nothing here needs a struct callback.
2. **Middle.** `grid.CreateTopology(agentSize, connectivity, diagonals)` gives a `SquareTopology` (hex: `HexGrid.CreateTopology()`);
   `new AStar<TTopology, TCost, THeuristic>(topology, cost, heuristic, buffers)` with `IStepCost` / `IHeuristic` structs;
   `GridSearchBuffers` are reusable working memory (the search allocates nothing in steady state; marks carry a generation
   number so nothing is cleared between searches).
3. **Raw.** `OpenSet`, `IGridTopology` + `INeighborVisitor` (implement your own graph), `GridSearchBuffers`, `AreaCosts`,
   `IGridChangeListener`; for jump point search `JumpRules` (pruning), `IJumper` with `OnlineJumper`, `BlockJumper` and
   `TableJumper`, `BlockMap`, `JpsPlusIndex` (the tables), and `JumpSearch<TJumper, THeuristic>` (tier 2: struct-generic over the jumper and heuristic).
   Tier 1 picks them with `GridOptions.Algorithm`.

Debug data: `GridOptions.RecordExpansions` / `GridSearchBuffers.RecordExpansions` record the expanded nodes in order
(`GridPath.ExpandedCells`, `GridSearchBuffers.ExpandedNodes`); the path is `GridPath.Cells` / `Points`.

## Jump point search (JPS, JPS+)

Papers: Harabor & Grastien 2011 (*Online Graph Pruning for Pathfinding on Grid Maps*) for JPS; Harabor & Grastien 2014
(*Improving Jump Point Search*) for block-based scanning and the offline (JPS+) variant, with Rabin's formulation of the
preprocessed distances (Game AI Pro 2). Code: `src/OnePath/Grids/Jps/`.

**When it applies.** Eight-connected moves, `DiagonalRule.NoCornerCutting`, agent size 1, every area with the same cost
multiplier `m` (straight step `m`, diagonal step `m * √2`, the same values A* uses when all multipliers are equal), no
step-cost or blocking callbacks, heuristic weight 1, heuristic octile or zero. The one-line API (`GridOptions.Algorithm`)
falls back to A* when a query does not qualify (and `GridPath.Algorithm` says which ran); JPS+ also falls back to JPS when a
side of the grid exceeds 32766 cells. Other rules (squeezing, allow-all, four moves) and agent sizes above 1 are A*'s. The
default is A*.

**Pruned successors** (`JumpRules.Successors`). Let a node be reached by a step of sign (dx, dy) from its parent (the
parent may be many cells away; only the direction matters). The start node searches all eight directions.
- Diagonal arrival: the diagonal and its two cardinal components. A diagonal step into a cell needs both side cells
  free (no corner cutting), so no neighbour is forced.
- Straight arrival: straight on; and on each side s (perpendicular to the step), if the cell beside the node on side s is free and
  the cell beside the *previous* cell on that side is a wall, that side cell is reachable at the optimal cost only through this
  node: the side's cardinal direction and the diagonal (step + s) are searched too. This is the forced neighbour of the 2011
  paper moved one step along, because a diagonal step out of the previous cell is illegal when its side cell is a wall.

**Jump** (`IJumper.Jump`). From a node in a direction, walk until a cell is reached that is the goal or (straight) has a
forced neighbour as above, or (diagonal) is the goal or has a straight scan in one of its two cardinal directions that finds
such a cell; a wall stops the walk with no result. A diagonal step needs both side cells and the target free. The result is the
cell and the number of unit steps.

**Search** (`JumpSearch`). A* over jump points with the grid core's `OpenSet`: smaller f, then larger g, then smaller cell
index. f = g + octile(m) (or g for the zero heuristic); g adds `steps * m` or `steps * (m * √2)` per jump (so a path
cost is a sum of one product per segment, not one addition per cell, and may differ from A*'s cell-by-cell sum in the last
bits). A cheaper route reopens a node. Successors are generated in direction order 0..7 (north, east, south, west,
north-east, south-east, south-west, north-west); on equal g the first parent found stays. The goal is tested when popped. One
iteration pops and expands one node, so a sliced search equals an unsliced one; the iteration budget and expansion
counts follow A*'s rules. The returned path lists every cell between the jump points (consecutive cells are neighbours).
Jump points are not unique to the search: any valid optimal path is returned, and A*, JPS and JPS+ may return different
paths of equal cost.

**Block-based scanning** (`BlockMap`, `BlockJumper`). The walkable cells are kept as bits, 64 per word, once by rows and
once by columns (bit i of word k of a line is position 64k+i; positions past the line end are blocked). A straight scan
reads whole words: stop bits are `~walkable | forced`, where forced = `side & ~(side shifted by one position along the
scan)` for the two neighbouring lines (a missing line counts as blocked), masked to positions beyond the start, and the first
set bit (lowest bit forwards, highest backwards) is the stop; a goal on the line before the stop wins. The map follows
grid edits immediately (a bit per edit in each orientation). It finds exactly the cells the cell-by-cell scan
(`OnlineJumper`) finds.

**JPS+ tables** (`JpsPlusIndex`). Per cell and direction one signed 16-bit value, stored cell by cell (`table[cell * 8 +
direction]`, so the eight values of a cell share a cache line; `Tables` returns the same numbers as a direction-major copy,
`[direction * cellCount + cell]`), direction order as above, cells row-major. Value `d > 0`: the next jump point is `d` steps away; `d <= 0`: no
jump point before a wall, and `-d` is the number of walkable cells before it. A wall cell has all zeros. Straight direction:
look at the next cell `q`; if `q` is a wall, 0; if `q` has a forced neighbour for this arrival, 1; else the value of `q`
moved one step further (`v + 1` if `v > 0`, else `v - 1`). Diagonal: if the step is illegal (side cell or target a wall), 0; if
`q`'s table for either cardinal component is positive, 1; else the same chain rule on the diagonal table. Build order: the
four straight tables, then the diagonals, each from the far end. Size 16 bytes per cell.
- *Goal bounding at query time.* Straight: the goal on the line ahead within `|d|` steps is returned directly (cost =
  distance). Diagonal: if the goal lies strictly inside the quadrant and `min(|gx-x|,|gy-y|) <= |d|`, the run stops
  at that many steps (the cell level with the goal in one axis), even if the stored jump point lies further. Otherwise a positive
  `d` returns the stored jump point and a non-positive one nothing. Stopping early adds nodes that plain JPS would
  not create; it never loses an optimal path (a superset of the nodes JPS visits) and never adds a cheaper one.
- *Incremental update.* The index subscribes to the grid's `IGridChangeListener`; `Refresh()` (called by every query) applies
  the pending edits: for each edited cell recompute the east and west tables of its row and the rows above and below, the
  north and south tables of its column and the columns beside it, recording the cells whose sign (positive or not)
  changed; then for every edited cell and sign-changed cell recompute the four diagonal entries that read it (the cell itself,
  the cells one step behind it horizontally, vertically and diagonally), and follow each changed diagonal entry backwards
  along its diagonal until an entry comes out unchanged. The tables then equal a fresh build (checked on >1000 random
  edit sequences and in the fuzz target). More than max(256, cells/64) pending edits are answered by a full rebuild.

**Cost with areas.** JPS assumes one cost for every step. Distinct area multipliers make a query fall back to A*; the
table index is independent of costs, so changing a multiplier needs no rebuild.

**Cost model of preprocessing.** O(8 · cells) time to build, 16 bytes per cell; the block map takes cells/4 bytes.

## The specialised octile searches (one-line A*, JPS, JPS+; `Grids/Fast/`)

`Grid.FindPath` runs these when the query qualifies for JPS above (eight moves, no corner cutting, agent size 1, one cost for every
area, no callbacks, weight 1, octile or zero estimate) with `GridAlgorithm.AStar`, `Jps` or `JpsPlus`. Everything else, sliced
searches (`AStar<,,>`, `JumpSearch<,>`), snapshots and replanning use the generic cores, which are unchanged. Costs are optimal
and equal to the generic cores'; the returned path may be another path of that cost.

**Exact costs.** A path costs `a + b·√2` for `a` straight and `b` diagonal steps. The searches keep the integer pair `(a, b)` per
node. Keys are computed from pairs: `g = a + b·√2`, `f = (a + (max − min)) + (b + min)·√2` with `min`/`max` of the absolute
column and row distance to the goal, each evaluated as one product and sums of doubles. Equal pairs give identical doubles, so
nodes with equal `f` in exact arithmetic tie exactly (cell-by-cell double sums would differ in the last bit and break ties by
noise); distinct pairs differ by more than 1e-6 for any grid that fits in memory, far above rounding, so double comparison orders
them exactly. A path's reported cost is `m · (a + b·√2)` for the cell cost `m`. The estimate is consistent, so a closed node is
final and never reopened.

**Open list order.** Smaller `f`; on equal `f` larger `g` (rounded to float32, which is IEEE-exact), then smaller node index
(A*, with decrease-key in a 4-ary heap). The jump point searches use a 4-ary heap without decrease-key (an improved node is
pushed again; the older entry is skipped when it pops, the node being closed) with one addition: entries whose `f` equals the `f`
of the entry popped last go on a stack and pop first, last in first out, before the heap resumes (a heap entry with a smaller `f`
than the current level wins). The order is therefore fully determined by the sequence of pushes. With this tie-break A*
expands exactly the nodes that a (g descending) open list does: 3 594 911 expansions on dao-brc000d.

**A\*** pads the map with a wall all round and keeps wall cells in the node array as permanently closed records, so a neighbour
test is a load of the record the relaxation needs anyway. Node record 16 bytes: the pair, the heap slot, and a tag
`generation << 4 | arrival direction << 1 | closed`; the generation increases per search so nothing is cleared between queries.

**Jump point searches** (`OctileJump<TFinder>`, finders `BitFinder` and `TableFinder`). Node record 16 bytes (pair, parent, tag).
A neighbour byte per cell (`OctileMap.Neighbors`, bit d = neighbour in direction d walkable) and a table of 8 × 256 entries
(`OctileMap.Successors`, `JumpRules.Successors` evaluated once for every arrival direction and neighbour byte) give the pruned
successor set without cell lookups; a diagonal step is allowed when the neighbour byte has the step's two components and the
diagonal itself. The bit rows and columns of `OctileMap` are padded with 64 blocked positions before each line and a blocked word
after it, so scans never leave the array and the carry between words needs no test. A goal on the scan line is a stop bit of its own.
To make a scan step one word read, the map also keeps four arrays in the layout of the bit rows and columns (`RowStopForward`,
`RowStopBackward`, `ColStopForward`, `ColStopBackward`): bit i of word k is set when a scan along that line direction stops at that
cell, that is when the cell is a wall (`~w`) or has a forced neighbour (a cell beside the line is free while the one beside the previous line cell is
not; the neighbouring lines' words and the carry bit of the adjacent word are read once, when the array is built). Padding words and
padding lines are all ones. A cell edit recomputes the words of the three lines around it and the words before and after the edited
word. A scan masks the stop word with the positions beyond the start, adds the goal bit, and takes the first set bit; the result is a
wall (no jump point, 0) or the step count, as before.
The JPS+ finder reads `JpsPlusIndex` with the goal bounding above.

**Improved pruning** (`BitFinder`/`TableFinder` with `improvedPruning`; Harabor & Grastien 2014, section "Improved Pruning
Rules"; not reachable from the one-line API). Without corner cutting every diagonal jump point is an intermediate turning point.
A diagonal run from a node therefore does not stop at such cells: at each cell of the run it offers the straight jump points
(or the goal) reached from it, with the node as their parent and the straight direction as their arrival direction, and it
continues until a wall; the run's last cell is not a node. A hop from a node to such a successor is walked diagonal steps first, then
straight, when the path is written out, which is obstacle free because it is the run followed by the scan. In a test on dao-brc000d
this expands 83 678 nodes against 210 340 for JPS and 83 501 for the black-box reference's improved JPS.

The improved-pruning search is internal: the bar runner and `GridOptima` construct it directly, and no public option selects it. Should an
option expose it, it ships marked `[Experimental("ONEPATH0001")]` (`System.Diagnostics.CodeAnalysis.ExperimentalAttribute`, diagnostic id
`ONEPATH0001`) until a locked comparison confirms it is as fast as the black-box reference on every Moving AI row.

**Bar driver.** `bench/Bar/OursGrid.cs` loads the map and builds the bit maps and tables outside `search_time` (`preprocess_time`
reports them), runs 40 untimed warm-up instances, then times one pass over the scenario (`ONEPATH_GRID_PASSES=N` takes the
best of N passes in the process). The search loops are marked `AggressiveOptimization` so that short queries run fully optimised
code from the first call.

## Any-angle search (Theta*, Lazy Theta*)

Papers: Nash, Daniel, Koenig & Felner 2007 (*Theta\*: Any-Angle Path Planning on Grids*; JAIR 2010) and Nash, Koenig & Tovey 2010
(*Lazy Theta\**). Code: `src/OnePath/Grids/ThetaStar/`.

**Model.** Nodes are cell centres (the same nodes as A*), not cell corners as in the papers, so every algorithm of this document
shares one node set, start and goal definition and clearance rule. A waypoint list is a path; the agent moves in straight segments
between consecutive centres.

**Line of sight** (`GridLineOfSight`). Two walkable cells see each other iff the closed segment between their centres touches no
wall cell, where a wall is the *closed* unit square (touching a wall's corner blocks). Diagonal neighbours therefore see each other
exactly when both beside cells are walkable: every legal `NoCornerCutting` move is a line of sight. Cells outside the grid are walls.
Integers only. With `dx = x1-x0`, `dy = y1-y0`, `nx = |dx|`, `ny = |dy|`, steps `sx, sy = sign`:
- `nx = 0` or `ny = 0`: all cells on the row/column between the endpoints.
- else walk from (x0, y0) with counters `ix = iy = 0` until `ix = nx` and `iy = ny`; `d = (2ix+1)*ny - (2iy+1)*nx` (64-bit). `d < 0`:
  step in x (`ix++`); `d > 0`: step in y (`iy++`); `d = 0`: the segment passes through a lattice corner; the two cells beside it
  `(x+sx, y)` and `(x, y+sy)` must be walkable, then step in both. Every cell entered must be walkable.

**Applies when** (otherwise the query runs as A* and `GridPath.Algorithm` says so): eight-connected, `NoCornerCutting`, agent size 1,
one cost multiplier `m` for every area, no `StepCost`/`Blocked` callbacks, heuristic `Auto`, `Euclidean` or `Zero`.
The cost of a segment is `m * sqrt(dx*dx + dy*dy)` (`dx`, `dy` integer, the sum exact in 64 bits, one correctly rounded square root),
so a straight cell step costs `m`, a diagonal one `m * Sqrt2` as in A*. A path's cost is the sum of its segment costs added in path order
from 0 (`GridPath.Cost`; `Cells` and `Points` list the waypoints only). Heuristic: `m * sqrt(dx*dx + dy*dy)` to the goal
(`EuclideanHeuristic`), weight `GridOptions.HeuristicWeight`.

**Search** (`ThetaSearch<THeuristic>`, tier 2). The open set is the grid core's `OpenSet` (smaller f, larger g, smaller cell index); one
iteration pops one node; the goal is tested when popped; budgets and slicing as in A*. Neighbours of an expanded node `s` are its legal
grid moves in the order north, east, south, west, north-east, south-east, south-west, north-west; closed nodes are never reopened
(as in the papers) and a neighbour's cost is replaced only when strictly lower, so the first parent found wins ties. The start has no parent
(the papers make it its own parent). Let `p` be the parent of `s`, `c(a, b)` the segment cost.
- *Theta\** (`ThetaVariant.Theta`): for each neighbour `t` that is not closed: if `p` exists and `LOS(p, t)`, candidate `g(p) + c(p, t)` with
  parent `p`; otherwise `g(s) + c(s, t)` with parent `s`.
- *Lazy Theta\** (`ThetaVariant.Lazy`, the paper): the candidate is always `g(p) + c(p, t)` with parent `p` (no test; the start
  uses `s`). When a node is popped, `SetVertex`: if it has a parent `p` and not `LOS(p, node)`, the parent becomes the closed grid neighbour `u`
  (neighbour order, first on ties) minimising `g(u) + c(u, node)`, and `g(node)` is set to that sum; then the node is closed and expanded.
- *Lazy Theta\* with re-queue* (`ThetaVariant.LazyRequeue`, the tier-1 `GridAlgorithm.LazyThetaStar`): as `Lazy`, but a node whose `SetVertex`
  changed its parent is pushed back into the open set under its corrected `g` and `f` and not expanded in this iteration (the pop still counts).
  After the repair the parent is an adjacent cell, so the next pop does not repair again. **Why:** the paper's order expands such a node under
  its optimistic cost, and on random grids that occasionally returns a path longer than the A* path (15 of about 50,000 queries, at most 13%
  longer); with the re-queue none was seen in the same checks. It costs a few more iterations.

The papers prove no optimality; paths are short, not shortest. Length lies between the visibility-graph optimum (shortest path turning
only at cell centres) and, in all checks, the A* length.
## Flow fields (many agents, one goal set)

Integration step of flow-field pathfinding (Emerson 2013, *Crowd Pathfinding and Steering Using Flow Field Tiles*;
Pentheny 2013): Dijkstra's algorithm run outwards from the goals. Code: `Grids/Flow/` (`FlowField<TTopology, TCost>` tier 2/3,
`GridFlowField` tier 1 via `Grid.CreateFlowField` and `HexGrid.CreateFlowField`).

- **Integration field** `cost[n]` (double, +∞ = unreachable or not walkable). Goals (sorted, deduplicated; a goal the agent
  cannot stand on is inactive) cost 0. Every other node costs `min over neighbours u of (cost[u] + move(n, u))`, `move(n, u)` being
  the cost of the move *out of* `n` (the topology's move cost passed through the cost hook; the same cost in both directions
  is required because the wave runs backwards along the neighbour lists). Sums are IEEE doubles formed as `cost[u] + move`.
  Square grids use the same neighbour set, diagonal rule, agent clearance and `(m[from] + m[to]) / 2` costs as A*; hex grids
  the six hex moves. The tier-1 blocked filter isolates a cell (no move into or out of it); the step-cost delegate sees `(from, to, grid cost)`.
- **Settling order.** The open list is `OpenSet` with `f = g = cost`, so ties on cost pop by smaller node index. The
  field does not depend on that order: a node's final cost is the minimum, over neighbours, of the same rounded sums of their
  final costs (rounding is monotone and moves cost ≥ 0, so a neighbour settled later never beats one settled earlier).
- **Direction field** `next[n]` (node index, −1 for goals and unreachable nodes): the **first** neighbour in the topology's neighbour
  order (square: N, E, S, W, NE, SE, SW, NW; hex: axial order) that attains the minimum of `cost[u] + move(n, u)`. It is eight-way
  (six-way on hex), not a continuous gradient: `GetDirection` returns the unit vector from this cell's centre to the next one's,
  and following `next` from any reachable cell reaches a goal with total move cost equal to `cost[n]` (exactly, in the same
  arithmetic, up to the order of addition: a sum taken from the cell towards the goal can differ from the wave's sum in the last bits
  when move costs are not dyadic, because √2 is rounded). A continuous gradient and a line-of-sight shortcut near the goal are not provided.
- **Iteration budget.** One iteration pops one node and relaxes its neighbours. `Step(n)` does at most `n`; a sliced field
  equals an unsliced one and reports the same `Settled`. `GridOptions.IterationBudget` bounds the constructor's run; call `Step`/`Run` to finish.
- **Incremental repair.** `Refresh()` after cell edits (found through `IGridChangeListener`) recomputes only the affected region
  (Ramalingam & Reps 1996 style): the *dirty* nodes — the 3x3 block around an edited cell, widened up and left by the agent size − 1
  for agents wider than one cell (hex: the cell and its neighbours) — are invalidated together with every node whose chosen next node is invalid, then each invalid node is seeded
  from its still-valid neighbours and the wave refills the region (improvements spread beyond it). Directions are recomputed for every node whose cost changed
  and its neighbours. The result is bit-identical to a field computed from scratch (cost bits and `next`). A change to an area
  multiplier, or `Recompute()`, recomputes everything. Changes the grid cannot report (a delegate answering differently) need `Recompute()`.
- **Threads.** A field is computed on one thread. Fields for different goal sets may be computed concurrently over one grid that is
  not being edited (listener registration is locked); results do not depend on the thread count. Nothing in the flow-field code uses
  thread-count-dependent scheduling.
- **Verification.** Integration field equals a per-cell O(n²) Dijkstra over 10^4 random square grids (4- and 8-connected, all three
  diagonal rules, agent sizes 1 and 2, area costs, 1–3 goals; bit-exact for four-connected where the costs are dyadic, 1e-9 relative with √2) and 600 hex grids (bit-exact); following the direction field from
  every cell reaches a goal at the field's cost; repair equals recompute (bit-exact cost and `next`) over 600 edit sequences including agent sizes 2–3, hex and area-cost
  changes; sliced equals unsliced for budgets 1–50; fields on 8 threads equal sequential ones. On Moving AI maps `GridOptima --alg flow` (`ALG=flow LIMIT=n sweep.sh`)
  computes the whole-map field to each scenario's goal and checks the start's cost against the published optimum (relative 1e-6): dao, sc1, bg512 and room, 4000 scenarios each, 0 mismatches.
  Hash scenarios `grids/flow/*`; fuzz target `grid-flow`.
## Tile edits and incremental clearance

`Grid.SetRect(x, y, w, h, area)` and `SetRect(x, y, w, h, areas)` (and the same on `HexGrid`) change a rectangle as one edit:
cells are written in row-major order, `Version` goes up once if any cell changed (nothing happens otherwise), and the listeners hear
about it once. A listener implementing `IGridBatchListener` gets `OnRectChanged` with the smallest rectangle containing every changed
cell (cells inside it may be unchanged); any other listener gets `OnCellChanged` for each changed cell, row-major. The grid already holds
the final state in both cases. A rectangle outside the grid throws `ArgumentOutOfRangeException` before any cell is written; a source of
the wrong length throws `ArgumentException`.

**Clearance repair.** Clearance of a cell reads only the cells right of, below and diagonally below-right of it, so it can change only
if the cell was edited or one of those three changed. The grid keeps the rectangles edited since the clearance was last current (more
than 32 are merged into their bounding box). The next read repairs rows from the lowest edited row upwards, each row right to left:
the cells of the row that lie in an edited rectangle, plus the cells one step left of and below the changed interval of the row
below, are recomputed, and the sweep continues left past that range only while values keep changing. Edits made before the clearance
was ever read cost nothing. The result equals the full recompute (checked on random edit sequences, `SetRect` included).
`JpsPlusIndex` implements `IGridBatchListener`: a rectangle adds its cells to the pending edits (or schedules a full rebuild past the
usual threshold). `BlockMap` updates per changed cell.

## Incremental replanning (LPA\*, D\* Lite)

Papers: Koenig, Likhachev & Furcy, *Lifelong Planning A\** (2004); Koenig & Likhachev, *D\* Lite* (2002), the version with the key
modifier `km`. One engine (`ReplanSearch<TTopology, TCost, THeuristic>`) serves both. It searches from a **root** (LPA\*: the start;
D\* Lite: the goal) towards a **focus** (LPA\*: the goal; D\* Lite: the start, which moves). `g` is the cost from the root and `rhs` the
one-step lookahead, `rhs(root) = 0` and otherwise the minimum over walkable neighbours `v` of `g(v) + cost` where the move cost is the
move `v -> node` (LPA\*) or `node -> v` (D\* Lite) through the cost hook. Unreachable and non-walkable nodes have `rhs = +inf`.

- *Queue.* Nodes with `g != rhs` are queued with key `[min(g, rhs) + h(focus, node) + km, min(g, rhs)]`. Order (`ReplanOpenSet`): first
  key, then second key, then node index; two keys within a relative 1e-11 count as equal.
- *Loop.* While the top key is smaller than the focus's key, or the focus is inconsistent (`g != rhs`): pop `u`; if its stored key
  is smaller than its current key, re-queue it with the current key; else if `g(u) > rhs(u)` set `g(u) = rhs(u)`, remove it and
  recompute `rhs` of its neighbours; else set `g(u) = +inf` and recompute `rhs` of `u` and its neighbours. Recomputing a node's `rhs` is a full
  minimum over its neighbours (not the paper's per-edge shortcut), then queue it if inconsistent, remove it if consistent.
  One pop is one **iteration** (the unit of `IterationBudget`/`Replan(budget)`); pops that change a `g` are **expansions**.
- *Tolerance.* Costs are sums of doubles added in different orders by different repairs, so costs equal on paper can differ in the
  last bits. "Smaller key", "equal" and "consistent" therefore use a relative tolerance of 1e-11 (absolute 1e-11 below 1), and a node
  whose key ties the focus's key on paper is processed when its second key is smaller. Stopping late is harmless, stopping early leaves stale costs.
  Reported costs are exact sums along the returned path, in start-to-goal order.
- *Moving the start* (D\* Lite): `km += h(old start, new start)` and the heuristic's goal becomes the new start; the queue keeps
  all its work. For LPA\* a moved start or changed goal restarts the search. D\* Lite restarts on a changed goal.
- *Changes.* The node set to re-examine for an edit at cell `(x, y)` with agent size `s` on a square grid is the cells
  `[x - s, x + 1] x [y - s, y + 1]` (for a rectangle edit `[x - s, x + w] x [y - s, y + h]`): a move between two nodes reads the two nodes'
  areas, the cells beside a diagonal move, and the `s x s` blocks both nodes occupy, all within one cell of both nodes, and the walkability
  of a node for size `s` reads the block starting at it. Both ends of every changed move are in the set, so recomputing `rhs` there is
  complete. On a hex grid the changed cell and its neighbours are re-examined. A change touching more than a quarter of the cells (or
  more pending nodes than the grid has) restarts the search instead. A change to `AreaCosts` (a multiplier) rebuilds the search
  (the heuristic's scale depends on the smallest multiplier). Changes the grid cannot see (a delegate that now answers differently)
  are announced with `Invalidate`.
- *Path.* From the focus, repeatedly step to the neighbour minimising `g + move cost`, preferring neighbours with `g == rhs` (settled; a
  queued neighbour may hold an out-of-date cost), ties broken by neighbour order (see Movement); LPA\* walks goal to start and reverses.
  Many equal-cost paths exist, so the path may differ from A\*'s, but never the cost.
- *Requirements.* The move graph must be undirected in structure (true for the square and hex topologies); the heuristic must be
  consistent and symmetric (octile, Manhattan, hex distance, zero); the heuristic weight must be 1; costs of +inf/NaN remove a move.
- *Slicing.* `Replan(budget)` pops at most `budget` times; on `BudgetExhausted` the next call resumes, and any sequence of budgets
  ends with the result, expansion count and path of one unsliced call.

Tier 1: `grid.CreateReplanner(start, goal, options)` (`GridOptions.Algorithm` = `LpaStar`, else D\* Lite) returns a `GridReplanner`
(`HexReplanner`) that listens to the grid: `Replan()`, `MoveStart`, `SetGoal`, `Invalidate`, `Dispose`. Tier 2: `ReplanSearch<,,>` over any `IGridTopology`
with struct cost and heuristic. Tier 3: `ReplanOpenSet`, the node-level `MarkChanged`/`Replan`/`ExtractPath`. A one-off `FindPath` with
`LpaStar` or `DStarLite` runs A\*.
## HPA* (hierarchical path-finding over chunks)

Botea, Müller & Schaeffer 2004, one level. `GridAlgorithm.Hpa` (tier 1; `HpaChunkSize` default 16, `HpaSmoothing` default true),
`HpaHierarchy` and `HpaSearch` (tier 2), the per-chunk data through `ClusterNodes`, `IntraCost`, `GetInterEdges` (tier 3).

**Scope.** Square grids, four or eight moves, rules `NoCornerCutting` and `NoSqueezing`, any agent size, area costs. Not supported
(the query runs as A* and `GridPath.Algorithm` says so): step-cost or blocking callbacks, `AllowAll` with eight moves. The
heuristic options and weight are ignored. Hex grids are not covered. No multi-level hierarchy. Clearance: one hierarchy is
built over the cells walkable for one agent size (true clearance); tier 1 caches a hierarchy per (chunk size, agent size, moves, rule), at most eight, oldest dropped first.
The paper's annotated variant (one graph for all sizes) is not used.

**Chunks.** Side C; chunk `(cx, cy)` holds cells `[cx·C, min(W, cx·C+C)) × [cy·C, min(H, cy·C+C))`, indexed `cy · ClustersX + cx`.

**Entrances and transitions.** For two chunks sharing a border, border position `p` is open when the two cells facing each other are
both walkable. A maximal run `[lo, hi]` of open positions is an entrance. Width `hi-lo+1 ≥ T` (`EntranceThreshold`, default 6) and
`hi ≠ lo`: two transitions at `lo` and `hi`; otherwise one at `lo + (width-1)/2`. A transition is the pair (cell in chunk A, cell
in chunk B); both cells are abstract nodes joined by inter-edges in both directions at the move cost `(m[a] + m[b]) · 0.5`. Diagonal border moves
get no entrances of their own: under both supported rules a diagonal crossing implies a walkable straight crossing beside it, and the
cells of one run are connected inside each chunk, so abstract connectivity equals real connectivity.

**Intra-edges.** For every ordered pair of nodes of a chunk, the cost of the cheapest path that stays inside the chunk (Dijkstra over the
topology's own moves, targets settled early; open order as in `OpenSet`, with the chunk-local cell index as last tie-break). +∞ when
none. Moves between two cells of a chunk read only cells of that chunk, so a chunk's edges depend on its cells alone.

**Canonical form.** A chunk's nodes are sorted by cell index; its inter-edges by (node, target cell). The graph is a function of cells,
costs and options only: `ContentEquals` and `GetContentHash` (FNV-1a 64 over node cells, cost bits and inter-edges in chunk order) compare it.

**Query.** Insertion: from the start, one chunk-restricted Dijkstra gives the cost to every node of its chunk (and to the goal when
in the same chunk); the same from the goal gives the cost of the edges into the goal (move costs are symmetric). Abstract A* runs with
cells as node ids (open order: smaller f, larger g, smaller cell index), the octile (eight moves) or Manhattan heuristic times the
smallest area cost, and edges expanded in the order: start edges or intra-edges by node index, edge to the goal, inter-edges by target. One iteration
is one pop of the abstract open set; `IterationBudget` and `Step(n)` count these. The abstract path is refined by repeating the
chunk-restricted search that priced each intra-edge (so the cells match the edge's cost); a border crossing is one step.

**Smoothing** (paper section 5, on by default). Walking from path position `i` in each allowed direction (order: N, E, S, W, NE, SE, SW, NW) while moves are legal,
a walk that meets a later path cell `j > i+1` and costs less than the path from `i` to `j` by more than 1e-9 is a candidate. The
candidate with the largest `j` wins, then the lower cost, then the lower direction; the path continues from `j`. A cell visited twice by the path maps to its last position.
`GridPath.Cost` is always the sum of the final cells' move costs.

**Guarantees.** A path is found exactly when A* finds one; each step is a legal move; the cost is never below the optimum. It is not optimal:
the paper reports a few percent over; Moving AI measurements are under Verification.

**Incremental update.** The hierarchy is an `IGridChangeListener`. A cell edit at `(x, y)` marks every chunk intersecting
`[x-(S-1)-1, x+1] × [y-(S-1)-1, y+1]` (`S` the agent size: the edit changes clearance up and left by `S-1`, and entrances read the
neighbouring chunk's cell), clipped to the grid. `Refresh` (called by every query) rebuilds marked chunks in index order; the result equals a full rebuild. Edits to
the area cost table (`AreaCosts.Version`) mark everything. `Grid` notifies listeners on every edit, so batches of edits cost one `Refresh`.

**Cost model.** Build: one Dijkstra per node over at most C² cells; node count per chunk is roughly `4·C/T` to `4·C/2`. Memory: `n²` doubles per chunk.
## 3D voxel grids

`VoxelGrid` (root namespace) is a bounded block of voxels, one byte each: an area id 0..254 or
255 for solid, the same ids and `AreaCosts` table as the square grid. It is searched by the same `AStar` through
`VoxelTopology`.

**Storage (normative).** The block is cut into 16 x 16 x 16 chunks. The chunk table is dense, `ceil(sx/16) * ceil(sy/16) *
ceil(sz/16)` slots, slot index `cx + ChunksX * (cy + ChunksY * cz)`. A slot is null (every voxel of the chunk has the grid's
default area, fixed at construction: 0 for open space, 255 for a solid block to carve) or a 4096-byte array indexed
`(x & 15) | ((y & 15) << 4) | ((z & 15) << 8)`. An edit that differs from the default allocates the chunk, which then stays.
Padding voxels past the size in an edge chunk are never read. At most 2^30 voxels.

**Search nodes.** A voxel's node is `x + sx * (y + sy * z)`; it is the final open-list tie-break. State per node is cost 8,
parent 4, mark 4, open-set position 4 (20 bytes). The one-line query (`grid.FindPath`) keeps that state in pages of 8 x 8 x 8
voxels (512 nodes, 10 KB), allocated when the search first reaches a voxel of the block and found through a dense table with one
int per block (4 MB for 5x10^8 voxels); the parent is stored as a slot (page << 9 | in-page index), the in-page index is
`(x & 7) | ((y & 7) << 3) | ((z & 7) << 6)`, and the search reads each voxel's coordinates from its page, so no node index is
ever divided. Search memory is therefore proportional to the blocks the search touches. The paged search is the same algorithm
as `AStar` (same open-list order, strict-improvement relaxation with reopening, neighbour order), so cost, path, expansion order
and iteration counts are equal (tested against the dense search). Pages are pooled between queries; a pool above 4096 pages
(about 40 MB) is dropped after the query. The tier-2 `AStar<VoxelTopology, ...>` with `GridSearchBuffers` stays dense, 20 bytes per voxel
of the block (a 518-million-voxel block needs about 10 GB).

**Moves and costs.** 6 (faces), 18 (+ edges) or 26 (+ corners) neighbours; the order is faces, then edges, then corners, each
by z, then y, then x ascending from -1 to 1 (an 18- or 6-move view uses a prefix). Face moves have length 1, edge moves
`GridMath.Sqrt2`, corner moves `VoxelMath.Sqrt3` (the double nearest the true root, 1.7320508075688772). The cost of a move is
`length * ((m[from] + m[to]) * 0.5)`. The destination must be open.

**Corner rule** (`VoxelCornerRule`) for edge and corner moves. The swept cells are those inside the move's cube other than
the start and the destination: two face cells for an edge move; three face cells and three edge cells for a corner move.
`NoCornerCutting` (default, the Moving AI voxel convention): all swept cells open. `NoSqueezing`: some monotone route of face
steps from start to destination through open cells exists (an edge move needs one of its two face cells; a corner move needs
a face cell and an edge cell containing it). `AllowAll`: the destination only.

**Heuristic.** The exact empty-grid distance of the connectivity times `AreaCosts.MinCost`, so it is admissible and
consistent. With sorted absolute offsets a <= b <= c and m = a + b + c: six moves, m; 26 moves, `a*sqrt3 + (b-a)*sqrt2 + (c-b)`;
18 moves, `e*sqrt2 + (m - 2e)` with `e = min(floor(m/2), a + b)`. A* tie-breaking and the open list are those of the square grid
(`OpenSet`: smaller f, then larger g, then smaller node index, as documented above).

**Changes.** `SetArea`/`SetWalkable` notify `IVoxelChangeListener`s and bump `Version`; topologies read the live chunk table,
so a query after an edit equals a query on a grid built from scratch with the final contents (tested). There is no cached
state to repair.

**API tiers.** Tier 1: `grid.FindPath(VoxelCell|VoxelPoint, ..., VoxelOptions?)` with plain options and delegates
(`StepCost`, `Blocked`), presets `VoxelOptions.Default/SixConnected/EighteenConnected`. Tier 2:
`AStar<VoxelTopology, TCost, VoxelHeuristic>` with struct cost hooks, resumable `Step(budget)` equal to the unsliced run.
Tier 3: `VoxelGrid.GetChunk`, `VoxelTopology`, `VoxelHeuristic`, `GridSearchBuffers`. World positions are doubles within
+-2^21 (`VoxelSpace`: origin and cell size, voxel (i,j,k) spans `origin + (i,j,k) * size`); anything beyond throws
`ArgumentOutOfRangeException` naming the value; a position outside the block gives `StartInvalid`/`GoalInvalid`.

**Moving AI voxel files.** `VoxelMap.Parse` reads `.3dmap` (`voxel X Y Z`, then one `x y z` line per *solid* voxel);
`VoxelScenarios.Parse` reads `.3dscen` (`version 1`, the map name, then `sx sy sz gx gy gz length ratio`). Malformed input
throws `GridFormatException` with the line. The published lengths of the Warframe set equal 26-move, no-corner-cutting
costs with the constants above: the check allows an absolute 1e-6 (observed differences are below 1e-7: the files print eight
decimals); the two looser rules give different costs on 95% of the first 2000 scenarios of Simple.

## Moving AI files

`MovingAiMap.Parse` reads the `.map` format (`type`, `height`, `width`, `map`, then rows). `.` and `G` are walkable;
`@`, `O`, `T` blocked; `S` (swamp) walkable and `W` (water) blocked by default (`MovingAiOptions`; the published
optimal lengths of the wc3maps512 set need exactly this). `MovingAiScenarios.Parse` reads version 1 `.scen` files
(9 whitespace separated fields per line). Both reject malformed input with `GridFormatException` (line number in the
message); sizes are bounded (65536 per side, 2^28 cells) and a map's cells must be present in the file before they are
allocated.

## Verification

- Costs equal an independent O(n²) Dijkstra written from the definitions on 10^4 random grids per variant (4-/8-connected,
  the three diagonal rules, area costs, agent sizes 1-3, the four hex layouts); returned paths are checked move by move.
- Published optimal lengths of the Moving AI 2D sets (dao, da2, bg512, sc1, wc3maps512, street, maze, random, room),
  8-connected, no corner cutting, via `tools/GridOptima`. The published numbers differ from exact octile sums by at most
  1.7e-9 of the length (consistent with some being summed with a diagonal cost of 1.41421356); the check allows a relative 1e-6.
- Hash-matrix scenarios `grids/*` write status, cost bits, expansion counts and paths for generated maps.
- Fuzz targets `movingai-map`, `movingai-scen`, `grid-query`, `grid-jps`, `grid-theta` (`fuzz/run.sh`).
- JPS and JPS+: costs equal Dijkstra on 10^4 + 4·10^3 + 600 random grids, on rooms with gaps, and A* on grids wider than a
  bitmap word; table entries equal the cell-by-cell scan; incremental tables equal a rebuild over 1200 edit sequences; the
  block scan equals the cell scan with goals anywhere; every Moving AI 2D set above equals the published optimal costs with
  both (`ALG=jps sweep.sh`, `ALG=jps+ sweep.sh`, or `GridOptima --alg jps|jps+`).

### Any-angle verification

- Line of sight equals an independent exact segment-versus-closed-square test (separating axes, integers) on 3,000 grids x 60 pairs
  and 300 grids up to 40 x 40, and is symmetric.
- On 10^4 random grids up to 12 x 12 (and 2,000 with a unit cost of 2.5, 300 up to 30 x 30): same reachability as A*; every segment passes the
  exact test; the cost equals the segment lengths added in order, bit for bit; Theta* and `LazyRequeue` are never longer than A*; every
  variant is at least the visibility-graph optimum (Dijkstra over all line-of-sight links between walkable centres).
- `GridOptima --alg theta|lazytheta` reports, on Moving AI scenarios, the ratio to the published eight-connected optimum (at most 1), re-verifies
  every 64th path with the independent test, and compares with the visibility-graph optimum on eight deterministic 40 x 40 windows per map
  (`--random N` does the same on random 24 x 24 grids).
- Replanning: after every change in 10.8k random change/move sequences (LPA\* and D\* Lite; 4- and 8-connected, three diagonal rules, agent sizes 1-3,
  area costs, cell and tile edits, start moves) the replanned cost equals an independent O(n^2) Dijkstra and the library's A\*, and paths are legal
  and sum to their cost; hex layouts the same; sliced equals unsliced; incremental clearance and JPS+ tables after tile edits equal a full
  recompute. On Moving AI dao and room (`GridOptima --replan lpa|dstar`): random wall toggles along the current path, costs equal fresh A\*.
- HPA*: refined paths are checked move by move against the brute-force move rules and priced independently on 10^4+ random grids
  (chunk 2-7, thresholds 1-8, both connectivities and rules, agent sizes 1-2, area costs): found exactly when Dijkstra reaches
  the goal, never cheaper than it, smoothing never longer; the hierarchy after random edit batches (agent sizes 1-3, area
  edits) equals a rebuild bit for bit; sliced equals whole for budgets 1-17. `GridOptima --alg hpa [--chunk N] [--smooth 0|1]` checks every
  Moving AI scenario's path and reports the mean and maximum percentage over the published optimum (chunk 16, smoothing on, 8-connected: dao 0.40% mean,
  bg512 0.33%, sc1 0.22%, room 0.08%; unsmoothed 4.9%, 4.1%, 3.2%, 1.6%; zero invalid or missing paths). Maxima are large only for very short
  scenarios (a few cells), where one detour through a transition is a big fraction. Fuzz target `grid-hpa`.
- Voxel grids: costs equal an independent O(n²) Dijkstra (moves enumerated from the definition, corner rules evaluated
  over the move's bounding box) on 10^4 random blocks per connectivity (all three corner rules, area costs, both heuristics),
  on larger blocks across chunk borders, and the heuristic equals the empty-grid distance; edits equal rebuilds; sliced equals
  unsliced for budgets 1, 2, 7 and 50. Fuzz targets `voxel-map`, `voxel-scen`, `voxel-query`; hash scenarios `grids/voxel/*`;
  the Warframe set via `tools/GridOptima/voxel-sweep.sh` (`GridOptima --voxel`).

## Host API: resumable handle, tick-path calls, hash, snapping, exact costs

For a lockstep or rollback host whose simulation ticks must not allocate.

**`GridSearch`** (`Grid.CreateSearch(options)`) is a non-generic resumable A\*, JPS or JPS+: `TryBegin(start, goal)` returns
`InProgress`, `StartInvalid` or `GoalInvalid` (the checks `FindPath` makes: inside the grid, clearance for `AgentSize`, `Blocked`; a refusal leaves the
running search alone; `Begin` is the throwing twin), `Step(n)` pops at most `n` times and returns `InProgress`, `Found` or `NoPath`, `CopyPath(Span<Cell>)` /
`CopyPath(Span<int>)` write the path, `SaveState`/`RestoreState` snapshot it (`grids-state.md`, kind 12). It supports `GridAlgorithm.AStar`, `Jps` and `JpsPlus` (the others throw `ArgumentException`). It
chooses its engine exactly as `FindPath` does: for A\* the specialised octile engine when the options allow it, otherwise the generic
`AStar<SquareTopology, ...>` instantiation `FindPath` would build; for JPS and JPS+ the specialised octile jump point engine when the options allow it,
otherwise the generic `JumpSearch` over the block map (JPS) or the JPS+ tables, and options a jump point search cannot honour (four moves, agent
size above 1, area costs that differ, callbacks, a weighted or Manhattan estimate) run plain generic A\*, as `FindPath` does. One iteration of a
jump point search is one pop of a jump point (the scans that find jump points are not counted). `FindPath(start, goal, o)` is `TryBegin` then
`Step(o.IterationBudget)`, so for every slicing the handle returns cell for cell the path `FindPath` returns, with the same
`ExpandedCount` and cost bits (tested on generated random, room, maze, cave and area-cost maps and eight option variants for A\*, nine for JPS and JPS+). The grid must not be edited
while a search is in progress (`Step` throws `InvalidOperationException`). After the first search `TryBegin`, `Step`, `CopyPath`, `GetStateSize`, `SaveState(Span<byte>)`
and `RestoreState` allocate nothing on every engine; a state save costs a pass over the cells plus the visited cells (the generic engines serialise
their own state into a buffer the handle keeps, which grows to the largest state seen).

**Which engine breaks ties how.** The specialised engine (`Grids/Fast`) keeps costs as exact integer pairs
(straight steps, diagonal steps), orders its open list by `f`, then larger `g` rounded to float32, then smaller cell index, and
relaxes neighbours in the order N, E, S, W, NE, SE, SW, NW. The generic engine sums double step costs and orders by `f`, then
larger double `g`, then smaller node index, in the same neighbour order. Both are optimal, but on a tie between equal-cost paths they can
choose different cells. We did not make the generic engine reproduce the pairs: its cost type is an arbitrary `double`
(hooks, area multipliers), so no exact pair exists, and the specialised engine's speed depends on them. Instead
**`GridOptions.SpecializedEngines = false`** forces the generic engine in `FindPath` and `CreateSearch`, and that is then
cell for cell what a hand-built `AStar<SquareTopology, DefaultStepCost, OctileHeuristic>(topology, default, new OctileHeuristic(width, Areas.MinCost))` returns (tested). A host that mixes tier-1 and tier-2
searches and needs one answer per request sets it for all of them; otherwise use one entry point per request kind.
JPS and JPS+ honour the switch too (off means the generic jump search).

**`AStar.TryBegin(start, goal)`** (generic form) returns the same statuses as an integer-node check instead of throwing; `AStar.Topology`, `Start` and `Goal` are read-only views.

**`GridReplanner.TryReplan(budget)`** repairs like `Replan(budget)` but returns a `PathStatus`; on `Found` read `PathLength`, `PathCost` (summed from the start in
path order, bit-equal to `Replan().Cost`) and `CopyPath`. Path buffers are reused, so a steady-state call allocates nothing. `RecordExpansions` is not kept by this form.

**`Grid.ContentHash()`** (64 bit): feed words `w` into `h = rotl(h ^ (w * P2), 31) * P1 + P3`, from `h = P1 ^ P2`, with
`P1 = 0x9E3779B185EBCA87`, `P2 = 0xC2B2AE3D27D4EB4F`, `P3 = 0x165667B19E3779F9` (all arithmetic mod 2^64). Words, in order:
`width | height << 32`; the cells row-major, eight per little-endian word, the last zero-padded; the 255 area multipliers 0..254 as binary64 bit patterns.
Finish: `h ^= h >> 33; h *= P2; h ^= h >> 29; h *= P3; h ^= h >> 32`. Version, world mapping and edit history are not part of it. Known answers are in the tests.
The definition changes only with a note in the changelog.

**Large agents.** An agent of side *s* occupies the *s* × *s* block whose top-left cell is the position. `CanFit(cell, s)` is `clearance >= s`, the test
`FindPath` applies to its end cells; `MaxAgentSizeAt(cell)` is the clearance without exceptions (0 outside the grid).

**`Grid.TryFindNearestWalkable(cell | world point, out cell, options)`** returns the cell that fits `AgentSize` and is not refused by `Blocked`, nearest by squared Euclidean
distance between cell indexes, ties to the smaller row and then smaller column. The query may be outside the grid. `NearestRadius` bounds the search in Chebyshev rings.
It scans rings outward and stops once a ring is farther than the best hit.

**Exact integer costs.** `Grid.TryGetFixedCost(cells, out long)` prices a path of adjacent cells in Q31.32: every step is the double the default search charges
(the mean of the two cells' area multipliers; times `1.4142135623730951` for a diagonal), multiplied by 2^32 (exact) and rounded half to even, and the integers are added.
`GridPath.TryGetStepCounts` gives the exact numbers of straight and diagonal steps. Both refuse a path with a non-adjacent step (any-angle, abstract) and
`StepCost` hooks are not priced.

## Host hooks

Snapshot and restore of grids, voxel grids, graphs, replanners, flow fields and sliced searches, the byte format, what one
iteration of each family's budget is, and the debug geometry types are specified in `grids-state.md`.
