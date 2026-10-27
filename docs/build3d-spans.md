# 3D builder, front half: voxelization and the span stage

Normative. This describes what `RecursiveAI.OnePath.Mesh.Build3D` computes up to the compact open-span
heightfield, which is the hand-off to the region stage. A port must reproduce the same spans from these rules.
Everything is IEEE-754 float32 (`+ − × ÷ sqrt` only) unless a type says double; nothing here uses a transcendental function.

## 1. The grid (`HeightfieldFrame`)

- World origin `(ox, oy, oz)` (double), cell size `cs` (x and z), cell height `ch` (y), both float32 > 0, and the
  dimensions `W × D × H` in cells. A column is `(x, z)`, index `x + z·W`; a row is `y`.
- Cell size and cell height must lie in [2^-20, 2^20].
- Limits: `W, D ≤ 65536`, `W·D ≤ 2^24`, `H ≤ 65535`. Tile-local coordinates then stay below 2^16 cells, where a float32 resolves a cell to 1/256.
- Coordinate rule: every world coordinate (vertices, bounds, origin, volume corners) must be finite and within ±2^21.
  NaN is an `ArgumentException`, a value beyond the bound an `ArgumentOutOfRangeException` naming the value and the bound.
- `FromBounds(min, max, cs, ch)`: origin = min; `W = ceil((maxx − minx)/cs)` and likewise for `D` (with z) and `H` (with y, `ch`), computed in double. Empty or inverted bounds are an `ArgumentException`.
- A vertex becomes tile-local as `(float)(x − ox)` (double subtraction, rounded once to float32), then scaled to cell units:
  `u = lx / cs`, `v = ly / ch`, `w = lz / cs` (float32 division). All clipping happens in `(u, v, w)`, so every cell boundary is an integer.

## 2. Triangle rasterization (`Voxelizer.Rasterize`)

Triangles are processed in index order. Triangle `(a, b, c)` is wound counter-clockwise seen from the side it faces; its normal
is `n = (b − a) × (c − a)` computed on the tile-local float32 coordinates (not the scaled ones).

**Validation (before anything is added).** Index arrays must have a length divisible by 3 and every index must address a vertex
(`ArgumentOutOfRangeException`); every referenced vertex must be finite and within ±2^21. On failure nothing is added.

**Degenerate triangles.** `len = sqrt(nx² + ny² + nz²)` in float32. If `len` is 0 or infinite the triangle is skipped and counted in
`VoxelizeStats.Degenerate`.

**Walkability.** Triangle area id = 63 when `ny > slopeCosine · len`, else 0 (`ClassifyTriangles`; the caller can supply any area ids instead).

**Cells touched.** A cell is the half-open footprint `[i, i+1) × [j, j+1)` in `(u, w)`; the triangle is clipped to the *closed* footprint
(Sutherland–Hodgman against axis-aligned planes at integers; vertices exactly on a plane belong to both sides; the intersection point
on a crossed edge is `p + (q − p)·t` with `t = dp/(dp − dq)` in float32 and its clipped coordinate set exactly to the plane).
Splitting runs rows first (planes `w = j+1`, from low `j` upwards), then cells within a row (planes `u = i+1`), after first clipping to the grid rectangle `[0,W]×[0,D]`.

- A triangle whose footprint has positive area touches a cell exactly when its clipped piece has positive area. The area is
  `Σ (p_i − p_0) × (p_{i+1} − p_0)` over the piece's fan, in float32; pieces whose points share an axis-aligned line have area exactly 0.
  A triangle that only touches a cell's edge or corner therefore does not touch the cell.
- A **vertical** triangle (footprint cross product exactly 0 but `len > 0`) touches a cell when its clipped piece has positive length
  in `(u, w)` (`max u − min u > 0` or `max w − min w > 0`). If all three `u` are equal to an integer `k` (a wall exactly on a grid line)
  the wall belongs to column `k − 1` if `nx > 0`, else `k`; likewise for `w` with `nz`: it belongs to the side *behind* the surface, so a closed
  solid's walls stay inside it. Out-of-grid columns drop the wall.

**Span of one piece.** `lo = floor(min v)`, `hi = ceil(max v)` over the piece's vertices. If `lo == hi` (a horizontal piece exactly on a row boundary):
`ny > 0` → `lo = hi − 1` (the solid is below an upward-facing surface), else `hi = lo + 1`. The piece is dropped if `hi ≤ 0` or `lo ≥ H`;
otherwise rows are clamped to `[0, H]` and the span `[lo, hi)` with the triangle's area id is added to the column (§3). A surface exactly at the bottom of the grid facing up has no row to occupy and is dropped.

The surface of a span is its top: row `Max`, world height `oy + Max·ch`.

## 3. Spans and merging (`SolidHeightfield.AddSpan`)

A column holds disjoint, non-touching spans sorted by `Min` (a span `[Min, Max)`, `Min < Max`). Adding `[min, max)` with area `a` and
threshold `T` merges with every existing span that overlaps **or touches** it (`s.Min ≤ max` and `s.Max ≥ min`), one span at a time from the bottom up:

- if `s.Max > max`: `a = (s.Max − max ≤ T) ? maxArea(a, s.Area) : s.Area`, `max = s.Max`;
- else if `max − s.Max ≤ T`: `a = maxArea(a, s.Area)` (otherwise `a` is kept);
- `min = min(min, s.Min)`.

The result depends on the order triangles are added (the rule is not associative) but on nothing else. `T` is `SpanSettings.MergeThreshold`, default the walkable climb.
Soup of *surfaces* gives shells: the interior of a closed box has a floor span and a ceiling span with a gap between; overlapping or vertically
stacked closed meshes leave interior floors, as with any surface voxelizer.

## 4. Height grids

`RasterizeHeightGrid` turns `countX × countZ` heights (x fastest, spacing `s`, origin `(x0, z0)`) into two triangles per square, `(a, c, d)` and `(a, d, b)` with
`a = (i,j)`, `b = (i+1,j)`, `c = (i,j+1)`, `d = (i+1,j+1)`, facing up, and rasterizes them as triangles.

## 5. Voxel chunks (`VoxelChunk`, `Voxelizer.AddChunk`)

A chunk is run-length columns `(x, z) → runs (start, length, material)`, sorted, disjoint, non-empty, material 1..255, inside `0..Height`. `FromDense` takes
`(z·W + x)·H + y` indexing with 0 as empty and joins equal neighbours into runs. `AddChunk` places the chunk at an offset in the grid, drops what falls outside,
and adds each run as span `[start, start+length)` with area `materialAreas[material]` (default 63 for every material) and merge threshold 0, columns in z-then-x order, runs bottom-up.
A solid box built as triangles and the same box as voxels give the same spans except for the shell gap of §3; after the filters (which cannot see the gap from a walkable floor) the compact heightfields are identical.

## 6. Filters (on the solid heightfield; each only changes area ids)

All lengths in rows. `Unbounded = 2^30`. The ceiling of a span is the `Min` of the next span up, or `Unbounded` for the top one.

1. **Low-hanging obstacles** `(climb)`: for consecutive spans `p` below `s` in a column, if `s.Area = 0`, `p.Area ≠ 0` (original values) and `s.Max − p.Max ≤ climb` then `s.Area := p.Area`.
2. **Ledge spans** `(height, climb)`: for every span `s` with `Area ≠ 0`, floor `f = s.Max`, ceiling `c`. For each of the four neighbour columns:
   outside the grid → a drop of `−climb − 1`. Otherwise list the neighbour's gaps: `(floor −climb, ceiling first.Min)` (`Unbounded` if empty), and for every neighbour span `n`
   `(floor n.Max, ceiling next.Min or Unbounded)`. A gap is *passable* when `min(c, gapCeiling) − max(f, gapFloor) ≥ height`. For each passable gap: `drop = gapFloor − f`; track the
   lowest drop; if `|drop| ≤ climb` track min and max gapFloor. `s` is a ledge (`Area := 0`) if the lowest drop `< −climb`, or (max − min of accessible floors) `> climb`. Geometry only is read, so the result is independent of visiting order.
3. **Low height** `(height)`: `s.Area := 0` when `s.Area ≠ 0`, a span above exists and its `Min − s.Max < height`.

`SpanPipeline` applies 1, 2, 3 in that order.

## 7. Compact open-span heightfield (`CompactHeightfield.Build(height, climb)`)

For every solid span with `Area ≠ 0`, in column-major, bottom-up order, one `CompactSpan`: `Y = s.Max`, `Height = min(65535, nextMin − s.Max)` (65535 for the top span), `Area`, `Region = 0`.
`CellFirst/CellCount` index the spans of each column. Directions: 0 = −x, 1 = +z, 2 = +x, 3 = −z. A span links in direction `d` to the **lowest** span `t` (index `m` inside the neighbour cell) with
`min(s.Y + s.Height, t.Y + t.Height) − max(s.Y, t.Y) ≥ height` and `|t.Y − s.Y| ≤ climb`; the link stores `m` in 6 bits at `6d` (63 = none). Cells with 63 or more spans cannot be linked beyond index 62.

## 8. Erosion (`Erosion.ErodeWalkableArea(radius)`)

A span is a *boundary* span when its area is 0 or in any direction it has no link or its link target has area 0. `dist(s)` is the least cost of a path of steps from `s` to a boundary span,
where a step is an orthogonal link (cost 2) or two perpendicular links in either order (a diagonal, cost 3); boundary spans have distance 0 and distances are capped at `2·radius`.
Every span with `dist < 2·radius` gets area 0. The distance is the exact shortest path (a fixed point, independent of sweep order), not a two-pass approximation. `radius` is in cells, 0 disables, at most 2^20.

## 9. Area volumes (`ConvexVolume.Apply`)

The polygon (3 to 256 corners, either winding, convex and simple, non-zero area; checked by testing every corner against every edge) and the interval `[MinY, MaxY]`. A compact span with area ≠ 0 gets the volume's area
when the cell centre `(ox + (x+0.5)·cs, oz + (z+0.5)·cs)` (double) is inside or on the polygon and the floor height `oy + Y·ch` (double) lies in `[MinY, MaxY]`. Volumes are applied in list order after erosion; later ones overwrite.

## 10. Settings defaults (`SpanSettings.Humanoid`)

Slope cosine 0.70710677 (45°), height 10 rows, climb 4 rows, radius 2 cells, merge threshold = climb. For cell size 0.3 and cell height 0.2 these mean about 2.0 tall, 0.8 step, 0.6 radius.

## 11. Determinism

Single-threaded; no result depends on thread count, SIMD or platform. The hash-matrix scenarios `build3d/*` cover §2–§9.
