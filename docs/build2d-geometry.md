# 2D geometry for the exact navmesh builder

Normative description of `RecursiveAI.OnePath.Mesh.Build2D`: outlines and tile maps → regions → boolean
operations and offsetting → constrained Delaunay triangulation → convex polygons with adjacency. A later port that
follows this text produces the same output, bit for bit.

Sources: Hobby, *Practical segment intersection with finite precision output* (1999) and Guibas & Marimont, *Rounding
arrangements dynamically* (1998) for snap rounding; Chew, *Constrained Delaunay triangulations* (1989) and Sloan,
*A fast algorithm for generating constrained Delaunay triangulations* (1993) for the triangulation; Hertel &
Mehlhorn, *Fast triangulation of the plane with respect to simple polygons* (1983) for the convex merge; Lawson (1977)
for flips; Shewchuk (1997) for the exact predicates, supplied by the math package (`docs/predicates.md` there).

## Coordinates and the snap grid

- Public coordinates are `double` (`Vector2D`) and must be finite with magnitude at most 2^21. Anything else throws
  `ArgumentOutOfRangeException` naming the outline, ring and vertex (or tile-map grid line).
- `Geometry2DOptions.SnapBits` (default 16, range 0 to 16) fixes the grid unit 2^-SnapBits. **Every input
  coordinate is rounded to the nearest multiple of the unit (ties to even, `Math.Round`) before anything else.**
  A snapped coordinate is an integer of at most 37 bits; all later geometry is exact integer geometry.
- Constructed points (crossings, offset corners) are rounded to the same grid. Nothing is ever kept in floating
  point between stages: a `Region` stores integer grid points and reports `double` values `i × 2^-SnapBits`,
  which are exact.
- Signs of orientation and in-circle determinants come from the math package's exact predicates on the integer
  coordinates (exact in `double` up to 2^53). Constructions use 128-bit integers; every grid coordinate stays
  within 2^39 so no product overflows.
- Both operands of a binary operation must use the same `SnapBits`.

## Outlines, fill rule, regions

An `Outline` is a set of rings (outer boundary and holes, any order and orientation, last vertex joined to the
first; at least 3 vertices each). **An outline's area is the even-odd region of its rings.** Overlapping,
self-intersecting, touching and duplicate-point rings are all legal. `Region.FromOutlines` resolves each outline
alone, then takes the **union** of the outlines.

A `Region` is a set of rings that never cross or overlap and touch only at shared vertices, with the interior on
the left of every directed edge: outer boundaries run counter-clockwise, holes clockwise. Collinear degree-2
vertices are dropped. At a vertex where several rings meet, a ring continues along the outgoing edge first met
turning clockwise from the reverse of its incoming edge, which separates the region's wedges into minimal rings.
Rings start at their smallest directed edge in (x, y) order, and rings are ordered by that edge: the ring list
depends only on the set of boundary edges.

`Region.FromTileMap(cells, width, height, cellSize, origin)`: cell (x, y), row-major, spans `origin + (x, y) × cellSize`
to `origin + (x + 1, y + 1) × cellSize`. Grid line i is `origin + i × cellSize` computed once and snapped, so
neighbours share coordinates exactly. The cell size must be at least one grid unit. The boundary between solid and
non-solid cells is traced into rings directly (no noding is needed).

## Arrangement: noding by snap rounding

All boolean operations and offsets reduce to one routine. Input: directed edges, each tagged operand 0 or 1.
Output: non-crossing segments between grid points; each unique segment (vertex indices A < B in (x, y) order)
carries the net number of operand-0 and operand-1 input edges running A→B minus B→A. Zero-net segments are dropped.

1. Edges with equal endpoints are dropped.
2. **Hot pixels:** every endpoint, plus the rounded point of every proper crossing (two edges whose open interiors
   meet in a single interior point). The crossing is computed exactly as a rational and rounded once:
   `floor(x + 1/2)` per coordinate. A pixel is the half-open square `[c − 1/2, c + 1/2)²` of the points that round to
   grid point `c`, so every point belongs to exactly one pixel.
3. **Reroute:** each edge becomes the polyline through its own endpoints and every other hot pixel it meets (closed
   segment against the half-open square; evaluated exactly in doubled coordinates with a symbolic shrink of the
   open sides), ordered by the projection of the pixel centres onto the edge, ties by (x, y).
4. **Repair loop:** until nothing changes — fragments that properly cross get their rounded crossing added as a hot
   pixel and are rerouted again; a vertex lying exactly inside a fragment splits it. This normally makes one
   verifying pass; it guarantees the result has no proper crossings and no vertex inside a segment. After 256 rounds a
   `GeometryException` is thrown (a defect, never bad input).
5. Fragments are merged by endpoints; the vertex set is sorted by (x, y).

Distortion: a segment can move by up to one grid unit near other geometry (every vertex captures the segments passing
through its pixel). With the default grid that is about 1.5e-5 units.

## Labelling

The vertices are triangulated (below) with the arrangement's segments as constraints. Winding numbers (per operand)
are 0 on the unbounded face and change across a constraint running u→v from its right to its left by the segment's
net delta in direction u→v. A face is inside according to the operation:

| Operation | Inside when (w0, w1 windings) |
|---|---|
| even-odd (resolving an outline) | w0 odd |
| union of resolved edge sets | w0 > 0 or w1 > 0 |
| intersection | w0 > 0 and w1 > 0 |
| difference (operand 0 minus 1) | w0 > 0 and w1 ≤ 0 |
| symmetric difference | exactly one of w0 > 0, w1 > 0 |

Constraints separating inside from outside form the boundary, directed with the inside on the left; they are chained
into rings as above.

## Offsetting

`Region.Offset(distance, join, arcTolerance)` with `|distance| ≤ 2^20`: positive grows, negative shrinks. The result
is `region ∪ S` (grow) or `region ∖ S` (shrink), where `S` is the union (windings > 0) of convex pieces swept along
every boundary edge, so thin features vanish and narrow gaps close exactly. Let `r = |distance| × 2^SnapBits` (grid units).

- **Round:** per boundary edge a rectangle of half-width `r` (corners `p ± r·n`, `n` the unit normal of the edge,
  each rounded to the grid, counter-clockwise), and per distinct boundary vertex a regular m-gon *circumscribed*
  about the circle of radius `r`, with its first vertex at half a step from the positive x axis, counter-clockwise,
  vertices rounded to the grid. The m-gon never lies inside the true circle; its vertices lie at most the arc
  tolerance outside it. `m` is the least of 8, 16, 32, 64, 128, 256 with `r / cos(π/m) − r ≤ tol`
  (`tol` = `arcTolerance` in grid units if at least 1, else `max(1, r/128)`), capped at 256. The unit vectors come from
  half-angle recurrences using only `+ − × ÷ sqrt` (`cos(θ/2) = sqrt((1 + cos θ)/2)`, from the 4-gon), and
  vertices are generated by repeated rotation, so they are identical everywhere.
- **Square:** per boundary edge the convex hull (exact, strictly convex, counter-clockwise) of the axis-aligned
  squares of half-side `max(1, round(r))` centred on its two endpoints: the Minkowski sum of the region with an
  axis-aligned square.
- Offset geometry beyond ±2^21 + 2^20 throws `ArgumentOutOfRangeException`.

## Constrained Delaunay triangulation

Input: distinct grid points sorted by (x, y), and noded constraints (segments meeting only at endpoints).

1. A bounding triangle with corners (−2^45, −2^45), (3·2^45, −2^45), (−2^45, 3·2^45) grid units encloses everything.
2. Points are inserted in sorted order into the Delaunay triangulation by Lawson's incremental algorithm: locate by a
   visibility walk from the last triangle (edge order 0, 1, 2; exact orientation), split the containing triangle (or
   the two triangles sharing the edge the point lies on), then flip edges opposite the new point while the opposite
   vertex lies strictly inside the circumcircle (exact in-circle). Cocircular points are not flipped.
3. Constraints are inserted in (A, B) index order by Sloan's method: walk the triangles crossed by the segment, queue
   the crossed edges, and flip each whose quadrilateral is strictly convex (exact test), re-queueing flipped edges that
   still cross, until the segment is an edge; then it is marked constrained.
4. Passes over all triangles in index order flip non-constrained edges whose opposite vertex is strictly inside the
   circumcircle (and whose quadrilateral is convex) until a pass flips nothing.

The result depends only on the sorted points and constraints.

## `Region.Triangulate`

Constraint edges are the region's ring edges; faces with winding > 0 are kept, so the triangles exactly cover the
region and every boundary edge is a triangle edge. Vertices are the ring vertices in (x, y) order. Each triangle is
rotated to start at its smallest vertex index; triangles are sorted by (v0, v1, v2). Neighbour entry `3t + i` is the
triangle across edge `(v_i, v_{i+1})`, or −1 on the boundary.

## `Region.PartitionConvex(maxVerticesPerPolygon)` (3 to 16, default 6)

Hertel–Mehlhorn greedy merge. Candidate diagonals are the interior triangle edges, longest first (squared length in
`double`; ties by smaller then larger vertex index). A diagonal is removed (its two polygons joined) when (a) the
joined polygon has at most the limit's vertices, (b) the polygons share no vertex other than the diagonal's ends,
and (c) the turns at both diagonal ends are left or straight (exact orientation ≥ 0). A single pass in that order is
exhaustive, because a merge only widens corners and enlarges polygons. The polygons are therefore **weakly convex**:
a vertex on a straight edge stays where a neighbouring polygon needs it. Each polygon starts at its smallest vertex
index; polygons are sorted lexicographically by their index lists. The neighbour of polygon edge `(v_i, v_{i+1})` is
the polygon holding the reversed edge, or −1.

## What is promised

- Every output is valid: rings never cross, polygons are convex and counter-clockwise, adjacency is symmetric, areas
  add up exactly.
- Results depend only on the **set** of input edges (not on outline order, ring order, start vertex or direction of a
  ring's even-odd reading) and on `SnapBits` / limits, and are identical on every platform and thread count.
- Operations are exact on the snapped input; the distortion bound above is the only approximation.
- Costs: noding compares edges whose x-ranges overlap (a sweep with an active list) and each fragment against the hot
  pixels in its x-range, so very long near-horizontal segments over very many vertices are quadratic; tile-sized inputs
  are the intended scale.
