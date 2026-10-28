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
plain build produces; the pipeline runs the same code on a copy.

The rebuild's cost is the partition and contour stages of `NavSettings.Partition`. `PartitionType.Layers` (monotone regions on height layers)
with `DetailSampleDistance = 0` is the cheapest setting, and is what a host that adds and removes obstacles often should choose; watershed
with detail sampling gives smoother regions at about twice the rebuild cost.

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

**Baking.** `Mesh3DPipeline.Obstacles` holds obstacles applied to every tile of a build, in list order, before the cache's own and after erosion. A mesh
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

## Hash-matrix scenario

`build3d/tilecache`: a rolling terrain built with a cache, then obstacles of all three shapes and a volume added, a sliced update, a snapshot,
more changes, a restore, and finally everything removed; the serialized meshes and snapshots are hashed at each step.
