# Grids: square and hex, open set, A*, area costs, clearance

Normative. A port must reproduce the same expansion order, costs and paths from this text. Code lives in
`src/OnePath/Grids/`; the namespace is `RecursiveAI.OnePath.Grids`.

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

1. **Happy path.** `Grid.From(bool[,])` (or spans, or `MovingAiMap.ToGrid()`), `grid.FindPath(a, b[, GridOptions])` with
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
(`GridPath.ExpandedCells`, `GridSearchBuffers.ExpandedNodes`); the path is `GridPath.Cells` / `WorldPoints`.

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

**JPS+ tables** (`JpsPlusIndex`). Per cell and direction one signed 16-bit value, layout `table[direction * cellCount +
cell]`, direction order as above, cells row-major. Value `d > 0`: the next jump point is `d` steps away; `d <= 0`: no
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

## 3D voxel grids

`VoxelGrid` (`RecursiveAI.OnePath.Grids.Voxel`) is a bounded block of voxels, one byte each: an area id 0..254 or
255 for solid, the same ids and `AreaCosts` table as the square grid. It is searched by the same `AStar` through
`VoxelTopology`.

**Storage (normative).** The block is cut into 16 x 16 x 16 chunks. The chunk table is dense, `ceil(sx/16) * ceil(sy/16) *
ceil(sz/16)` slots, slot index `cx + ChunksX * (cy + ChunksY * cz)`. A slot is null (every voxel of the chunk has the grid's
default area, fixed at construction: 0 for open space, 255 for a solid block to carve) or a 4096-byte array indexed
`(x & 15) | ((y & 15) << 4) | ((z & 15) << 8)`. An edit that differs from the default allocates the chunk, which then stays.
Padding voxels past the size in an edge chunk are never read. At most 2^30 voxels.

**Search nodes.** A voxel's node is `x + sx * (y + sy * z)`. Search buffers are dense: 20 bytes per voxel (cost 8,
parent 4, mark 4, open-set position 4), so a 518-million-voxel block needs about 10 GB for a search; storage of a sparse
world is cheap, searching one is not (a sparse search state is a possible later change).

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
- Fuzz targets `movingai-map`, `movingai-scen`, `grid-query`, `grid-jps` (`fuzz/run.sh`).
- JPS and JPS+: costs equal Dijkstra on 10^4 + 4·10^3 + 600 random grids, on rooms with gaps, and A* on grids wider than a
  bitmap word; table entries equal the cell-by-cell scan; incremental tables equal a rebuild over 1200 edit sequences; the
  block scan equals the cell scan with goals anywhere; every Moving AI 2D set above equals the published optimal costs with
  both (`ALG=jps sweep.sh`, `ALG=jps+ sweep.sh`, or `GridOptima --alg jps|jps+`).
- Voxel grids: costs equal an independent O(n²) Dijkstra (moves enumerated from the definition, corner rules evaluated
  over the move's bounding box) on 10^4 random blocks per connectivity (all three corner rules, area costs, both heuristics),
  on larger blocks across chunk borders, and the heuristic equals the empty-grid distance; edits equal rebuilds; sliced equals
  unsliced for budgets 1, 2, 7 and 50. Fuzz targets `voxel-map`, `voxel-scen`, `voxel-query`; hash scenarios `grids/voxel/*`;
  the Warframe set via `tools/GridOptima/voxel-sweep.sh` (`GridOptima --voxel`).
