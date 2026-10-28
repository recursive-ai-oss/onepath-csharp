# 3D builder: regions and contours

Normative. Continues `build3d-spans.md`: what `RecursiveAI.OnePath.Mesh.Build3D` computes from the compact heightfield up to contours, which are the
hand-off to the polygon stage. Integer arithmetic only (cell and corner coordinates are `int`, products `long`); nothing here uses floating point except
the error threshold compare in §6, which is IEEE `double` multiplication and division of exactly representable integers.

Throughout, *neighbour* and *link* mean a **mutual** link: span `a` has a neighbour `b` in direction `d` only when `a` links to `b` in `d` and `b` links to `a` in the opposite direction (with several floors in one cell a link goes to the lowest qualifying span and can be one-way; boundary tracing needs a symmetric relation to close its loops). A one-way link counts as no link.

Directions are the compact heightfield's: 0 = −x, 1 = +z, 2 = +x, 3 = −z. A span's *neighbour* in direction `d` is the span its link in `d` points to (or none).
Region id 0 means "no region"; ids 1..`RegionCount` are written to `CompactSpan.Region`. A region has exactly one area id and is connected through neighbour links.

## 1. Distance field (`Regions.BuildDistanceField`)

For every span the exact shortest-path distance to a boundary span, as in `build3d-spans.md` §8 (boundary = area 0, a missing link, or a link to an area-0 span;
orthogonal step 2, diagonal step 3), without a cap. Spans of area 0 have distance 0.

## 2. Watershed (`Regions.BuildWatershed`)

Walkable (area ≠ 0) spans are visited by distance from the largest value down; spans of equal distance form a *level*, ordered by span index.
At each level `L`:

1. **Growth.** Candidates are the level's spans. In synchronous rounds, every candidate with at least one neighbour that already has a region *and the same area* joins the smallest such region id;
   all joins of a round are decided from the state at the start of the round. The next round's candidates are the still-unassigned level spans next to a span that joined in the round just finished, in span-index order. Growth ends when a round joins nothing.
2. **Basins.** Each still-unassigned span of the level, in span-index order, becomes a new region (next id) and floods, breadth first, every connected unassigned span of the same area and the same level.

Then the filter of §5 runs.

## 3. Monotone (`Regions.BuildMonotone`)

Rows `z = 0, 1, …`. Within a row a *sweep* is a maximal chain of walkable spans linked in direction 0 (−x) with equal area, found while visiting cells in increasing `x`; a span with no such predecessor starts a new sweep.
For a sweep, look at the spans' neighbours in direction 3 (−z) that have a region and the same area. The sweep *continues* region `R` of the previous row when all those neighbours have region `R` (at least one exists)
and no other sweep of the row touches `R`; otherwise it starts a new region (next id, ids assigned in sweep order). Without filtering, a region has at most one sweep per row, so each of its rows is one run along x.
Then the filter of §5 runs.

## 4. Layers (`Regions.BuildLayers`)

Start from the unfiltered monotone regions. Two regions *overlap* if some column holds a span of each; they are *adjacent* if a link joins them. Repeat until nothing changes: for `a` in increasing id order (alive regions only), for adjacent `b` in increasing id order with the same area
that does not overlap `a`, merge the larger id into the smaller one (overlap and adjacency sets are unioned) and restart for the next `a`. The result is a set of layers in which no column holds two spans of one layer, and any two adjacent same-area layers overlap.
Then only the island removal of §5 runs (minimum area; no merging).

*Shortcut (same result).* When no column holds two walkable spans, no two regions overlap, so every pair of adjacent same-area regions merges and the partition is the connected components of "linked and same area". The
implementation then labels components directly (union over the −x and −z links, ids in order of first span) and skips the sweep and the merge; when a component is smaller than the minimum area it uses the general code
instead. A test compares both on random fields.

## 5. Filtering and numbering (`Regions.FilterRegions`)

1. *Islands:* a region with fewer than `MinRegionArea` spans that has no neighbouring region of any area (no link from any of its spans to a span with a different, non-zero region) is removed: its spans get region 0.
2. *Merging* (not for layers): repeat passes over regions in increasing id order; a region with fewer than `MergeRegionArea` spans merges into the neighbouring region with the **same area** that shares the most links with it (smallest id on ties), if one exists.
   Stop when a pass merges nothing. Afterwards no region smaller than `MergeRegionArea` has a same-area neighbouring region.
3. *Numbering:* surviving regions are numbered 1..k in order of their first span (span index order: column-major by `z`, then `x`, bottom up).

Walkable spans that end with region 0 are exactly those of removed islands. Spans with area 0 never have a region.

## 6. Contours (`Contours.Build`)

**Edges.** For span `s` of region `R ≥ 1` and direction `d`, the *neighbour value* is the region of the neighbour in `d`; 0 if there is no link and the adjacent cell is inside the grid; `Contours.TileBorder` (−1) if it is outside the grid.
The edge is a *boundary edge* when the neighbour value ≠ `R`.

**Tracing.** Cell `(x, z)` covers `[x, x+1] × [z, z+1]`. Edges are oriented with the region on the left (x right, z up): the start corner of the edge in direction 3 is `(x, z)`, direction 2 `(x+1, z)`, direction 1 `(x+1, z+1)`, direction 0 `(x, z+1)`.
Loops are traced from the first unvisited boundary edge in span-index, then direction, order. State `(span, dir)`: if the edge is a boundary edge, emit a vertex at its start corner with the edge's neighbour value, mark it visited, and continue with `(span, (dir+3) mod 4)`;
otherwise move to the neighbour in `dir` and continue with `(neighbour, (dir+1) mod 4)`. A loop ends on returning to its first state. Vertex `Y` = highest floor among the span and the up to three spans reached by the links around the start corner (the two links of the corner's edges and the diagonal one, reached either way).
A loop with positive signed area (counter-clockwise) is an outline of the region; one with negative area is a hole. The sum of the signed areas of all loops of a region equals twice its span count, and the loop lengths sum to its number of boundary edges.

**Simplification** of one loop (outline or hole), on the loop's `n` raw vertices, where a vertex's *neighbour* describes the edge to the next vertex:

1. *Kept vertices.* Every vertex whose neighbour differs from its predecessor's is kept (the *portals*). If there is none (one neighbour value all round), keep the lexicographically smallest and largest `(x, z)` vertices (lowest index on ties).
2. *Refinement.* Each stretch between consecutive kept vertices `a → b` is handled by a work stack, as in Douglas & Peucker (1973): among the raw vertices strictly inside the stretch, find the one farthest from the segment `a b`
   (distance to the *segment*: the perpendicular distance, or the distance to the nearer end when the foot falls outside; computed from integers, compared as `double`; ties go to the lexicographically smaller `(x, z)`, so the two sides of a shared boundary agree).
   If its squared distance exceeds `MaxError²` it is kept and both halves are refined. Otherwise, if `MaxEdgeLength > 0`, the stretch is a wall (neighbour value 0) and the squared chord length exceeds `MaxEdgeLength²`, the raw vertex halfway along the stretch (index `(ia + ib) / 2`) is kept and both halves are refined.
   Portal and border edges are never length-split.
3. Consecutive kept vertices at the same position are merged (the first stays). If fewer than 3 vertices remain, the area is 0 or its sign differs from the raw loop's, the loop is simplified again with `MaxError = 0`; if that fails too the raw loop is used. If the simplified loop has two edges that cross properly (strictly), the raw loop is used. Both cases count in `RawFallbacks`.
   Therefore every raw vertex lies within `MaxError` of the simplified outline (except in fallbacks, where the loop is raw), every portal vertex survives, and a region never disappears.

With `MaxError = 0` both sides of every shared boundary keep exactly the same vertices. With a positive error they do too, except around regions so small that simplification would erase them.

**Holes.** Each hole is simplified like an outline, then assigned to the smallest outline of its region that contains the hole's leftmost vertex (smallest `x`, then `z`, then index; boundary counts as inside); holes without an outline are dropped (`DroppedHoles`).
The holes of one outline are joined in order of their leftmost vertex. For the hole's leftmost vertex `p`, candidate outline vertices `q` are tried by increasing squared distance (lowest index on ties); the first bridge `p q` is accepted when `q` lies inside the interior cone at `p` (in the hole) and `p` inside the cone at `q` (in the outline), and the segment touches no
edge of the outline, the hole or the holes still to be joined other than at edges that end at `p` or `q`'s positions. The merged loop is: outline up to and including `q`, the hole from `p` all round and back to `p`, then `q` again and the rest of the outline.
The two bridge edges (`q → p` and `p → q`) have `Neighbor` = the contour's own region id. If no candidate works the hole is dropped. Cone and crossing tests use exact `long` orientation.
Because every loop is simplified on its own, a simplified hole may cross the simplified outline. After joining, if the merged loop has two edges that cross properly, the outline and its holes are discarded in favour of their raw loops, joined by the same rule (counted in `RawFallbacks`). A region that overlaps itself seen from above (a spiral ramp) can have holes that cancel the outline's area; if the joined loop's signed area is not positive, all of the outline's holes are dropped (`DroppedHoles`) and the outline stays unjoined.

**Output.** `ContourSet.Contours` ordered by region id, then by discovery order; each `Contour` has `Region`, `Area` (of the region), `Vertices` (counter-clockwise, at least 3), and, if requested, the raw outline and raw holes.

## 7. Defaults

`RegionSettings`: watershed, `MinRegionArea` 64, `MergeRegionArea` 400 (spans; 8² and 20², the usual reference values). `ContourSettings`: `MaxError` 1.3, `MaxEdgeLength` 12 cells.

## 8. Determinism

Everything is integer arithmetic over arrays and sorted collections; no result depends on hash order, thread count or platform. The hash-matrix scenarios `build3d/regions-*`, `build3d/contours` and `build3d/regions-contours-terrain` cover §1–§6.
