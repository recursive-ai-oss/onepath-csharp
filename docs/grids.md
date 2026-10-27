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
ordinary double rounding and identical on every platform. (The published Moving AI lengths carry about 1e-9 relative
error themselves; see Verification.)

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
   `IGridChangeListener`.

Debug data: `GridOptions.RecordExpansions` / `GridSearchBuffers.RecordExpansions` record the expanded nodes in order
(`GridPath.ExpandedCells`, `GridSearchBuffers.ExpandedNodes`); the path is `GridPath.Cells` / `WorldPoints`.

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
  8-connected, no corner cutting, via `tools/GridOptima`. The published numbers differ from exact octile sums by up to
  about 2e-9 of the length; the check allows a relative 1e-6.
- Hash-matrix scenarios `grids/*` write status, cost bits, expansion counts and paths for generated maps.
- Fuzz targets `movingai-map`, `movingai-scen`, `grid-query` (`fuzz/run.sh`).
