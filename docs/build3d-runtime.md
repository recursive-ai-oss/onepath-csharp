# The 3D builder at run time: tile cache, obstacles, area volumes

Status: follows the frozen mesh format (`navmesh-format.md`). Written so a port reproduces the same tiles bit for bit.

A level built once can change while the game runs: a barricade appears, a door's doorway closes, a swamp becomes costly. Rebuilding
the whole level is too slow, and rebuilding a tile from the triangles repeats work that did not change. A **tile cache** keeps, for
each tile, the part of the build that depends only on the geometry, and redoes the rest.

## What is cached

Each tile is built on a grid of its cells plus a border of `Border` cells (agent radius in cells + 3) (`build3d-polys.md`). The stages are

1. rasterize the triangles (or voxels) into a solid heightfield;
2. filters (low-hanging obstacles, ledges, low clearance) and compaction into the open-span heightfield (`build3d-spans.md`);
3. erosion by the agent radius and the pipeline's build-time area volumes;
4. **obstacles** (marked grown by the agent radius) and **run-time area volumes**;
5. regions, contours, polygons, detail, tile data (`build3d-regions.md`, `build3d-polys.md`).

The cache keeps the result of step 3 for every tile that was built (a tile with no triangle that could be walkable is never built and
has nothing to cache; nothing can make it walkable later, since obstacles and volumes only remove walkability or change an area id).
A rebuild copies that heightfield and runs steps 4 and 5; it never erodes again. It also keeps the area ids of the span array as last
built and as first built, and the tile data of the first build:

- a rebuild whose areas equal the last build's changes nothing (queueing is generous, so most rebuilds of a tile next to an obstacle find this);
- a rebuild whose areas equal the first build's reuses the first build's tile data (taking the last obstacle off a tile costs no partition).

The links between spans do not depend on areas or regions, so the span graph (mutual links, column of each span) is built once per cached tile and shared by every rebuild
of it; this saves time and changes no result.

Neither shortcut changes a result: a tile is a function of its areas. After a restore the last-built areas are forgotten (the mesh may hold
anything), so the next rebuild of each queued tile runs in full.

Building with a cache (`Mesh3DPipeline.BuildSoupCached`, `BuildChunkCached`, `BuildHeightfieldCached`) produces exactly the mesh the
plain build produces; the pipeline runs the same code on a copy. With `PartitionType.Layers` the cache also keeps each span's layer from the first build (see the layer-first partition in `build3d-regions.md`).

The rebuild's cost is the partition and contour stages of `NavSettings.Partition`. `PartitionType.Layers` (monotone regions on height layers)
with `DetailSampleDistance = 0` is the cheapest setting, and is what a host that adds and removes obstacles often should choose; watershed
with detail sampling gives smoother regions at about twice the rebuild cost.

## Layer-first tiles

With `PartitionType.Layers`, the default contour, polygon and detail steps, and no height sampling in the detail stage (`DetailSampleDistance` at most 0.9 cells; this is the cheap setting a host that adds and
removes obstacles often should choose), a tile is built **one layer at a time**. The general path (every other setting) builds all layers of a tile together and keeps the compact heightfield of the whole tile.

**Splitting.** At the first build the tile's layers are decided as in `build3d-regions.md` §4 (layer partition, then overlapping regions on different layers). A span that is not walkable after erosion goes with the layer of
its first walkable neighbour (direction order −x, +z, +x, −z), else layer 0. Each layer keeps its own spans in compact-heightfield order (column by column), their floor rows, column coordinates, base areas and the links between
spans **of that layer** (a link to a span of another layer is dropped, so a layer's contours see the other layer as a wall and its corner heights ignore it). Every rebuild starts from the layer's *first-build areas*
(the areas after erosion, the pipeline's own area volumes and the pipeline's own `Obstacles`).

**Per layer, for the areas in force:**

1. *Marking.* The cache's obstacles, then its volumes, in the order they were added, on a copy of the first-build areas (rules below). Nothing is marked, and nothing is rebuilt, for a layer whose walkable cells' box (x and z)
   the obstacle's box grown by the agent radius plus one cell (volumes: not grown) does not touch.
2. *Regions.* Connected components of spans of one non-zero area under the −x and −z links, numbered in order of first span; a component smaller than `MinRegionArea` spans with no walkable span of another area linked to it is
   dropped (the island removal of `build3d-regions.md` §5). Spans in the border ring that belong to a region become one border region.
3. *Contours.* Every region with a span inside the tile's own cells is traced and simplified exactly as in `build3d-regions.md` §6, with the border region as the tile border; corner heights use the layer's links.
4. *Polygons, detail, tile.* As in `build3d-polys.md` §3–§5 with this layer's polygons only: the vertex welding is per layer, the tile's vertical origin is this layer's lowest vertex, the layer number is the tile's layer.
   A layer with no polygon has no tile.

For a tile with one layer the result equals the general path's byte for byte (tested). Where a tile has several layers the two differ in welding across layers, corner heights next to another layer's floor, and the vertical
origin of each layer's tile; the walkable area is the same within 1 % (tested).

**Rebuilding.** `Update` marks every layer of a queued tile and compares its areas with those the mesh's tile for that layer was last built from; only a layer that differs is built again, and a layer whose areas equal
the first build's takes the first build's tile data without building. A rebuilt layer whose tile data equals what the mesh holds keeps its tile. With authored off-mesh links, links go to the layer that fits them best,
so a tile in which any layer changed is rebuilt in full, all layers. Pipeline `Obstacles` added or removed after the first build are not seen by rebuilds of a cache (its first-build areas are fixed).

### Repairing a layer

An obstacle only takes walkability away, so the layer a rebuild starts from is the first build's with some spans removed. For such a rebuild the cache does not label, trace and simplify the
whole layer again: it **repairs** what the first build left, and the result is exactly what the from-scratch stages give for the same areas (tested byte for byte against a cache with the repair
turned off, on random obstacles and histories). The first build keeps per layer: the region of every span, each region's size and first span, every traced loop (raw vertices, the edge each
vertex starts, its simplified form), which loop owns which edge, and every region's contours.

Given the list of spans the obstacles set to area 0 (the *removed* spans):

1. *Preconditions.* Otherwise the layer is built from scratch. (a) The change is only removed spans, no area volume. (b) No region shrinks below `MinRegionArea` or vanishes, and no region
   smaller than `MinRegionArea` (which is kept only while a walkable span of another area touches it) has a removed span next to it. (c) Every region that lost spans is still one piece: the
   walkable spans of the region next to a removed span (the *ring*) all reach each other inside the region. Every other span of the region reaches one of them, since a path from it to a removed
   span passes a ring span first, so this proves the region whole. The search first runs through the spans next to the ring, then through the whole region with a budget of a quarter of the
   layer (at least 256 spans). (d) The regions are still numbered in the same order: a region's number is the rank of its first span, and a removed first span moves it later.
2. *New loops.* Every edge of a still-walkable span of a traced region that looks at a removed span starts the trace of a loop (same walk and rule as the full trace). Each loop is rotated to start
   at its smallest edge key (span index × 4 + direction), where a scan of the whole layer would have started it. An old loop with an edge in a new loop is gone; an old loop that loses an edge of
   a removed span is found the same way, because its walk reaches the removed span through an edge that now looks at it.
3. *Regions.* A region with no new and no gone loop keeps the contours of the first build (shared, never changed). Any other region takes its surviving old loops, with the simplified forms the
   first build kept (simplification depends on the loop and on the areas of the regions it borders, which did not change), and the new loops, simplifies the new ones, joins holes to
   outlines and orders loops by their smallest edge key, as the full build does.
4. *Then* polygons, detail and the tile as always.

A repaired rebuild is a function of the first build and of the removed spans, so the result does not depend on the history of edits.

**Orientation tests in the polygon stage.** Polygons are kept convex in the stored float32 positions as well as in cells, which used to cost an exact predicate call per test (about 850 per
rebuilt layer). Rounding a coordinate to float32 moves it by at most 2^-24 of the grid's extent W, so a determinant moves by less than 2^-21·W², and a turn of at least
`((cells across the grid)² >> 19) + 1` cell areas keeps its sign: only turns closer than that call the exact predicate. The tile coordinate map exposes the bound; a test checks it against the
exact predicate on random and nearly collinear triples. Outputs are unchanged.

**Cost.** See "Cost of a rebuild" at the end.

## Obstacles (step 4)

An obstacle is a shape with a height range. Erosion has already run, so the shape is **grown by the agent radius** (`WalkableRadius` cells
times the cell size) instead: a cylinder's radius grows by it, a box's sides and an oriented box's half sizes move out by it. For every
column of the tile's grid (border included) whose **cell centre**
(`origin + (index + 0.5) * cellSize`, in double) lies inside the grown footprint (boundary included), every span with a non-zero area
whose **floor height** (`originY + row * cellHeight`, double) lies in `[MinY, MaxY]` (both included) gets area 0. Order does not matter
(the operation is idempotent per span). The grown footprint is a square-cornered or round offset, not the chamfer distance erosion uses,
so a wall made by an obstacle sits within about a cell of where erosion would put it.

| Shape | Parameters | Footprint test (x, z world; all double) | Height range |
|---|---|---|---|
| Cylinder | base centre (x, y, z), radius r > 0, height h ≥ 0 | (x−cx)² + (z−cz)² ≤ (r+g)² | [y, y+h] |
| Box | min and max corners | min − g ≤ point ≤ max + g on x and z | [min y, max y] |
| Oriented box | centre, half sizes (hx > 0, hy ≥ 0, hz > 0), yaw | with (s, c) = SinCos(yaw) from the deterministic math library: u = dx·c + dz·s, v = dz·c − dx·s; \|u\| ≤ hx+g and \|v\| ≤ hz+g | [y−hy, y+hy] |

g is the growth above. The yaw turns from the +x axis towards the +z axis. Every number is finite and within ±2^21, as is every extent (centre ± radius, centre ± the
rotated half size); anything else is rejected with `ArgumentException` or `ArgumentOutOfRangeException` naming the value.

**Baking.** `Mesh3DPipeline.Obstacles` (set in an object initializer; the list is copied and fixed, because a built cache never reads it again, and `TileCache.AddObstacle` is the only run-time route) holds obstacles applied to every tile of a build, in list order, before the cache's own and after erosion. A mesh
rebuilt from a cache after obstacles A, B, C were added equals, tile for tile and byte for byte, a fresh build of the same geometry with
A, B, C in `Obstacles` (the tests check this), and removing every obstacle gives back the original tiles.

## Area volumes (step 3 and 4)

`ConvexVolume` (`build3d-spans.md`) assigns an area id to eroded, walkable spans. In a rebuild, the pipeline's own volumes
(`Regions.Spans.Volumes`) apply first, then the cache's run-time volumes in the order they were added; the later volume wins where they overlap.
Building with a volume in `Regions.Spans.Volumes` equals adding it to the cache at run time. Because volumes run after erosion, area 0 makes spans
non-walkable without growing by the radius.

## Which tiles rebuild

An operation (add or remove an obstacle or volume) queues every cached tile whose build grid, widened by the agent radius and one cell, overlaps the shape's
bounding box in x and z (height is ignored). Queueing is by tile row, then tile column. A tile already queued keeps its place. Queueing a
tile that turns out unchanged is harmless: it is rebuilt to the same bytes.

## Rebuilding (`Update`)

`Update(maxTiles)` takes tiles from the front of the queue. **One tile is one iteration**; no clock is involved, so a budget does the same
work everywhere. A tile is rebuilt from **the obstacles and volumes in force when it is processed**, never from those at the time it was
queued, so the mesh once the queue is empty depends only on the final set of obstacles and volumes, not on how the work was sliced (tested
for budgets of one tile and for all at once).

Swapping a rebuilt position into the mesh, layer by layer from the lowest:

- a layer whose as-built content id equals the existing tile's keeps the existing tile (its polygon references stay valid and its run-time
  polygon edits stay);
- a changed layer replaces the tile in its slot (new generation; references into it become stale, as for `ReplaceTile`);
- a new layer is added; a layer that no longer exists is removed, lowest first.

Queries and corridors on the mesh see each swap at once, since the mesh object is the same.

## State and snapshots

Run-time state is: the obstacles (with ids), the volumes (with ids), the next ids, the queue, and the mesh's own dynamic state. The cache itself
is not state: it is rebuilt from the geometry, so restoring needs a cache built from the same geometry and settings. Ids start at 1, grow by one,
and are never reused. Layout: `navmesh-snapshot.md`, "Tile cache snapshot".

## Not done here

Tiles rebuild one at a time in queue order; a host may run rebuilds in parallel by tile only if it applies the results in queue order. Off-mesh
links, several agent sizes per world and streaming are separate parts.

## Cost of a rebuild

Measured on the generated `city` level (tile 64 cells, layers, no detail sampling, 16 cylinder obstacles added and removed one at a time, each followed by `UpdateAll`; about 19 layer builds per
cycle, 5000 spans each, 150 removed spans per obstacle and layer). With a warmed-up runtime one cycle takes 1.2 ms on an Apple M2 (the native tile cache it is compared with takes 1.29 ms there;
the layer-by-layer build without repair took 1.95 ms, and 3.85 / 7.7 ms before that). Of about 80 µs per layer build on a loaded x64 desktop, the repair is about 25 µs (the pass over the removed spans'
neighbours, the search for connectivity, joining the new hole), the polygon stage 15, making the tile 6–10, creating and stitching it 10, and comparing the areas of queued layers that turn out unchanged about 10.
A layer is built from scratch when a precondition above fails (about one in twelve on `city`, mostly obstacles that cut a street in two).

## Hash-matrix scenario

`build3d/tilecache`: a rolling terrain built with a cache, then obstacles of all three shapes and a volume added, a sliced update, a snapshot,
more changes, a restore, and finally everything removed; the serialized meshes and snapshots are hashed at each step.
