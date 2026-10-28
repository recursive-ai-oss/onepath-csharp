# 3D builder: polygons, height detail, tiles

Normative text for the last stages of the voxel pipeline and for `NavMesh.Build`. Earlier stages: `build3d-spans.md`
(voxelization, filters, compaction, erosion) and `build3d-regions.md` (regions, contours). The mesh format is in `navmesh-format.md`.
Everything is integer arithmetic, exact predicates, or IEEE `+ − × ÷ sqrt` in float32/double; nothing depends on thread count or hash order.
Tiles are built one after the other in row-major order (tile row, then column), so slot numbers are fixed.

## 1. Tile grid and settings (`Mesh3DPipeline`)

- `TileCells = max(1, ceil(TileSize / CellSize − 1e-4))`; the cell size used is `cs = TileSize / TileCells` (double). Cells therefore divide a tile exactly;
  `cs` is at most the configured cell size plus the 1e-4 tolerance. Rows are `CellHeight` high.
- Span settings (all in the nominal float32 sizes `cellSize = (float)CellSize`, `ch = (float)CellHeight`, float32 division):
  `WalkableHeight = max(1, ceil(AgentHeight / ch))`, `WalkableClimb = floor(WalkableClimb / ch)`, `WalkableRadius = ceil(AgentRadius / cellSize)`,
  slope cosine = `SlopeMath.CosineOfDegrees(AgentMaxSlopeDegrees)`: the Taylor series of cos in double with 14 terms (`t₀ = 1`, `t_k = −t_{k−1}·x² / ((2k−1)(2k))`),
  summed from the smallest term up, `x = degrees · (π/180)`, rounded to float32. (45° gives 0.70710677f.)
- Region settings come from `NavSettings` (`Partition`, `MinRegionArea`, `MergeRegionArea`); contour settings: `MaxError = ContourMaxError`, `MaxEdgeLength = (int)(MaxEdgeLength / CellSize + 0.5)` cells.
- **Border** `B = WalkableRadius + 3` cells on each side. The build grid of tile `(tx, tz)` is `(TileCells + 2B)²` cells with its minimum corner at
  `(TileOriginX(tx) − B·cs, y₀, TileOriginZ(tz) − B·cs)` (double products and sums in that order).
- `y₀` and the row count: for a triangle soup `y₀ = max(minY − CellHeight, −2^21 + 1)` over the tile's triangles, rows = `ceil((maxY − y₀) / CellHeight) + WalkableHeight + 2` (error beyond 65535 rows);
  for a heightfield `y₀` is the source frame's origin Y and rows are its rows + `WalkableHeight + 2`; for a chunk `y₀ = 0` and rows are chunk rows + `WalkableHeight + 2`.
- **Triangle selection (soup).** A triangle belongs to every tile whose grid box (the tile expanded by `B·cs`) its XZ bounding box touches, found as
  `floor((v − origin) / TileSize)` of the box expanded by `B·cs`, clamped to the soup's tile range. Triangles are passed in input order.
  A tile none of whose triangles `MayBeWalkable` is skipped: `MayBeWalkable` is `len > 0 && ny ≥ (cosine − 1e-3)·len` for the double cross product `n` of the triangle, so a skip never changes a result.
  The soup's tile range must not exceed 2^20 tiles (`ArgumentOutOfRangeException` naming the count).
- **Voxel input.** Voxel `(i, j, k)` is cell `i`, row `j`, cell `k` from `(OriginX, 0, OriginZ)`; tile `tx` shows chunk columns `tx·TileCells − B …`; one voxel is one cell, so choose `TileSize` a whole multiple of `CellSize`.
  Heightfield input maps cell `(i, k)` the same way and copies the spans (`AddSpan` with threshold 0); its frame's cell sizes are ignored.

## 2. Per tile

1. Rasterize (soup: `Voxelizer.ClassifyTriangles` + `Rasterize` with the span merge threshold), run `SpanPipeline` (filters, compaction, erosion, volumes).
2. Partition regions (`RegionPipeline.Partition`).
3. **Cut the border.** Let `borderId = (largest region id) + 1`. Every span with `Region > 0` whose cell is outside the tile's `TileCells²` core cells gets region `borderId`; `chf.RegionCount = borderId`.
   If no span of a region inside the core exists the tile is empty and skipped. The edges between a tile region and the border region are region-to-region edges, which contour simplification keeps
   straight between their end vertices (build3d-regions.md §6), so they lie exactly on the tile boundary lines.
4. Trace contours (`RegionPipeline.Trace`); contours of `borderId` are ignored from here on.
5. Polygonize (§3), choose `minRow` = lowest vertex row, build the detail (§4), write the tile(s) (§5).
   The tile's vertical reach `(maxRow − minRow + 2·(WalkableClimb + 1))·CellHeight` must not exceed `MaxLocalExtent` (`InvalidOperationException` naming the tile).

## 3. Polygons (`Polygonizer`)

Per contour, in contour order; all tests use `long` cross products on cell coordinates (exact below 2^20 cells) or the exact orientation of the stored float32 positions.

**Straight wall vertices.** Before anything else, a contour vertex is removed when both its edges are wall edges (neighbour 0), it is exactly collinear with the previous and next kept vertices (cross 0, same direction),
and its floor row is within one row of the straight line between those neighbours (`|(yᵥ−yₚ)·D − (yₙ−yₚ)·T| ≤ D`, with `D = |n−p|²`, `T = (n−p)·(v−p)`, all in `long`). Removal repeats until none is left,
unless the loop would drop below three vertices (then it is kept as it was). These vertices come from the edge-length split of simplification; each costs a polygon one of its vertex slots and removing it leaves the wall on the same line.

**Ear clipping** (O'Rourke, *Computational Geometry in C*, ch. 1). The loop is a doubly linked ring. A vertex `i` with neighbours `p`, `q` is a *valid ear* when `cross(p, i, q) > 0` and `(p, q)` is a diagonal:
it lies in the cone at `p` and at `q` (ring neighbours decide the cone, so hole bridges, which visit a vertex twice, are handled), and no ring edge that has no endpoint at the position of `p` or `q` touches it
(proper crossing, or an endpoint of either segment on the other). Among all valid ears the one with the shortest squared diagonal is clipped, ties to the lowest vertex index; the triangle `(p, i, q)` is emitted
only if its float32 orientation is positive. A valid ear stays valid when another is clipped, so only the neighbours of the clipped vertex and previously blocked ears are retested.
With no valid ear the first vertex from the current head that has `cross ≥ 0` and a diagonal is clipped (counted in `DegenerateEars`), else the head. The last three vertices form a triangle if positive.

**Welding.** Corner vertices with the same `(x, z)` and floor rows differing by at most 1 are one vertex (the first one created there, numbered in order of first appearance): bridge ends collapse, neighbouring contours share their boundary vertices, and a contour touching itself at a corner does not give a polygon two vertices at one place. Triangles with a repeated vertex are dropped.

**Merging** (Hertel & Mehlhorn 1983). Within one contour, repeat: among all pairs of polygons `A < B` (by index) sharing a directed edge `a→b` of `A` and `b→a` of `B`, with `|A| + |B| − 2 ≤ MaxVertsPerPoly`, choose the pair with the largest squared edge length
(first found in `A` index, then edge index order wins ties) whose merge is valid; merge them and delete `B`. The merged ring is `A` rotated to start at `b` (ending at `a`) followed by `B`'s vertices after `a` up to before `b`.
A merge is valid when the two joint vertices `a` (neighbours `x`, `w`) and `b` (`z`, `y`) do not turn right (`cross ≥ 0`) in cells, no vertex repeats, and in float32 positions `Local(x)`, `Local(z)` the exact orientation at each joint is `≥ 0`.
Exception: a joint that is collinear in cells and whose both incident edges are *wall edges* (contour edges with neighbour 0, or edges created by an earlier drop) is removed from the polygon instead; the turns at its neighbours must then still be `≥ 0` in float32.
(Float32 rounding can tip collinear points to the wrong side, and the format rejects that; wall-collinear vertices exist only because long wall edges are split.)
Merging never crosses contours. The polygon's area and region are the contour's.

**Re-cutting** (after merging, per contour). Merging only joins pairs, so three neighbours that fit into one fewer polygon are looked for explicitly. In passes (at most 16) over polygons `q` in index order,
for each pair of edges `e1 < e2` of `q` (edge order) that have neighbours `p` and `r` (distinct, not yet used in this pass, each sharing exactly one edge with `q`), the ring `q ∪ p ∪ r` is formed (joined as in merging, no convexity tests; its vertices must be distinct).
A ring with more than 2 right turns (in cells) is skipped. Otherwise it is cut by the first diagonal `(i, j)` (`i` ascending, then `j`; ring indices) whose two sides have positive area, at most `MaxVertsPerPoly` vertices, and are convex in cells and in float32 positions (the same test as merging).
The two sides replace `q` and `p`, `r` is deleted, and the three are marked used for the pass. After a pass the edge table is rebuilt; passes stop when none finds anything. Polygons are visited in index order and nothing depends on hashing.

## 4. Height detail (`DetailMeshBuilder`)

Coordinates are the stored tile-local float32 values, widened to double: `TileCoordinates.Local(c)` is `0` for `c = B`, `(float)TileSize` for `c = B + TileCells`, else `(float)((c − B)·cs)`;
`LocalY(row) = (float)((row − minRow)·CellHeight)`. The sample distance in cells is `sd = DetailSampleDistance / cs`; detail is *off* (each polygon is just triangulated) when `sd ≤ 0.9`.

- **Height lookup** `NearestRow(x, z, expected, region)`: the cells `floor(x), floor(z)` and, when `x` (or `z`) is an integer, the cell before it as well; among their spans (all, or only those of `region`
  when `region ≥ 0`) the floor row nearest to `expected` within `max(WalkableClimb, 1) + 1` rows, first (lowest cell, lowest span) on ties; none → no sample.
- **Edge points.** For each polygon edge longer than `sd`: walk from the lexicographically smaller end `(x, z)` (so both polygons sharing the edge generate identical points); `ns = ceil(len / sd)` (edges needing more than 4096 steps get none);
  point `k` (1 ≤ k < ns) is at `p0 + (p1 − p0)·k/ns` with row = `NearestRow(any region, expected = linear interpolation)` or the interpolation if none. Douglas–Peucker over the rows (error in rows; limit `MaxError / CellHeight`; the point of largest deviation, first on ties,
  is kept if the deviation exceeds the limit, then both halves are processed) keeps a subset; kept points are emitted in the polygon's direction. If the polygon vertices plus all edge points exceed 255 vertices no edge points are used.
- **Hull triangulation.** The ring (polygon vertices with their edge points) is ear-clipped with the exact orientation (strictly positive ears, empty of other ring vertices, closed test; relaxed to `≥ 0` then forced if needed; first valid ear from the head) and then every interior edge is made locally Delaunay by Lawson flips
  with the exact in-circle test, flipping only strictly convex quads, in LIFO order starting with all edges in triangle/edge order.
- **Interior samples.** On the grid `x = minX + i·sd`, `z = minZ + j·sd` (i, j ≥ 1, strictly inside the polygon's bounding box, `z` outer loop, `x` inner), keep points whose distance to every polygon edge line exceeds `sd/2`, with row = `NearestRow(region of the polygon, expected = mean vertex row)`.
- **Refinement.** Each sample is located in a triangle (first containing it by exact orientation, boundary included) and its error is `|y − plane height|` in double. While fewer than 255 vertices exist: take the live sample with the largest error (lowest index on ties);
  if its error is not above `MaxError` stop; add it as a vertex (a point on an edge splits the edge and its neighbour; a point on a hull edge or vertex is dropped) and legalize the surrounding edges; only samples inside changed triangles are re-located and re-measured.
- **Output.** Extra vertices in creation order (edge points, then inserted samples); triangles in triangle-id order as three vertex indices (polygon vertices `0…n−1`, then extras) and an edge-flag byte whose bit `k` is set when the triangle's edge `k` has no neighbour (lies on the polygon boundary).

## 5. Tile data

- A region's layer: with `Partition = Layers`, regions are visited in id order; a region may not share a column with a region of the layer it takes; it prefers the lowest-numbered layer of an adjacent region that is free, else the lowest free layer. Other partitions put every polygon on layer 0
  (overlapping floors then share one tile layer, as in a single Detour tile). A tile layer holds only polygons of its regions; links between layers of one tile do not exist (they connect only across tile boundaries). Up to 1024 layers.
- Per layer: vertices used by its polygons in ascending original index; polygons in original order; area = the contour's area with 63 (`SpanSettings.WalkableArea`) replaced by `NavSettings.Area`, other ids unchanged; flags `NavSettings.Flags`; detail ranges copied and rebased.
- `NavTileAssembler.FromLocalPolygons` derives in-tile adjacency (edges shared by exactly two polygons with opposite direction) and marks wall edges whose two vertices lie bitwise on `0` or `(float)TileSize` as portals; `NavTile.Create` validates (convex, counter-clockwise, ranges).
- Tile `OriginY = frameOriginY + minRow·CellHeight` (double). A tile with no polygons is omitted. At most 65536 vertices per layer (`InvalidOperationException`).
- Portals are stitched by the mesh as described in `navmesh-format.md`; neighbouring tiles' boundary vertices need not match.

## 6. Errors and limits

`ArgumentException` (non-finite coordinate, lengths not multiples of three), `ArgumentOutOfRangeException` (coordinate beyond ±2^21 naming the vertex and value, index out of range, setting out of range, more than 2^20 tiles, more than 65535 rows),
`InvalidOperationException` for limits found while building (a tile's vertical reach, vertices per layer, layers, a polygon's detail triangles). Nothing else is thrown for any input; a validator failure of the result is a bug.

## 7. Format changes

None to the format or its version. New public builder API: `NavTileAssembler.FromLocalPolygons` (like `FromPolygons`, but takes the tile-local float32 vertices as they are stored). New `NavSettings` fields: `CellHeight`, `Partition`, `MinRegionArea`, `MergeRegionArea`,
`ContourMaxError`, `MaxEdgeLength`, `DetailSampleDistance`, `DetailMaxError`.

## 8. Quality against Recast 1.6.0

Generated levels (`tools/geom/gen_levels.py`, size 256, 2000 points), same settings (`bench/Bar/OursBuild.cs` prints them), one build each, measured with `tools/bench/quality-3d.sh`; coverage is the sum of polygon areas projected on the ground plane.

| level | coverage ours vs Recast, solo | tiled (64 cells) | polys ours/Recast, solo | tiled | mean path length delta, solo | tiled |
|---|---|---|---|---|---|---|
| city | +0.15 % | −0.09 % | 679 / 361 | 702 / 605 | +0.51 % | +0.39 % |
| overlapping | +0.19 % | −0.01 % | 39 / 256 | 612 / 776 | +0.37 % | −0.11 % |
| ramps | +0.25 % | +0.03 % | 68 / 138 | 282 / 365 | −0.02 % | −0.14 % |
| slivers | +0.08 % | 0.00 % | 5 / 65 | 196 / 247 | 0.00 % | −0.06 % |
| stairs | +0.22 % | −0.07 % | 63 / 147 | 256 / 315 | +0.06 % | −0.06 % |
| terraces | +0.20 % | 0.00 % | 5 / 65 | 196 / 247 | −0.02 % | −0.01 % |

(`stairs` finds 1969 of 1973 paths, −0.2 %, the rest unreachable between snapped points.)

## 9. Hash-matrix scenarios

`build3d/polys` (polygonizer and detail on seeded fields), `build3d/mesh-levels` (serialized meshes of generated levels, three settings), `build3d/mesh-layers-chunk` (layers; voxel chunk).
