# Navigation mesh queries

Normative behaviour of `NavMeshQuery` (namespace `RecursiveAI.OnePath.Mesh.Queries`) over a `TiledNavMesh`
(`docs/navmesh-format.md`). Everything here is deterministic: results depend only on the mesh, the inputs and
the options, never on platform, thread count or time. All arithmetic is IEEE `+ − × ÷ sqrt` on double (positions
are world doubles; vertices are float32 tile-local values widened to double and added to the double tile origin).

## Inputs and errors

- Positions are `Vector3D` doubles. A non-finite coordinate or one with |v| ≥ 2^21 throws `ArgumentOutOfRangeException`
  naming the value. Search extents must be finite and not negative (same exception).
- Polygon references that are invalid, stale (the tile was removed or replaced) or rejected by the filter never throw: the
  call returns a status (`InvalidStartPoly`, `InvalidEndPoly`, `RaycastStatus.InvalidStartPoly`, `InvalidCorridor`) or `false`.
- A query object is not thread-safe, may be reused for any number of queries, and allocates nothing in steady state when the
  caller supplies the output buffers (the `Span` forms). The one-call forms that return `PathResult` allocate the result only.

## Filters and costs

A filter decides which polygons may be used and prices travel:

- `QueryFilter` (plain): `IncludeFlags` (default 0xFFFF = anything, including polygons with no flags set; otherwise at least
  one flag bit must match), `ExcludeFlags` (default 0; any match rejects), per-area multipliers `SetAreaCost(area, cost)`
  (default 1; `+∞` forbids the area; costs must be > 0), and an optional `Func<ushort, byte, bool>` predicate.
- `IQueryFilter` (struct generic, for speed): `Passes(flags, area)`, `AreaCost(area)`, `MinAreaCost`.
  `MinAreaCost` must not exceed the true minimum area cost or paths stop being optimal; 0 disables the heuristic.

Off-mesh links are tested with their own flags and area as well as the target polygon's.

## Nearest polygon

`FindNearestPoly(center, halfExtents, filter)` considers every polygon whose axis-aligned vertex box overlaps the box
`center ± halfExtents` (all tiles and layers) and that the filter accepts. For each candidate the *closest point* is:

- if `center` is inside the polygon's outline seen from above (inclusive of the boundary): the point straight above or below it
  on the polygon surface, `OverPolygon = true`;
- otherwise the nearest point of the outline in X/Z (lowest edge index on equal distance), with the height interpolated linearly
  along that edge.

Surface height at (x, z): with detail, the first detail triangle (in index order) containing the point, by barycentric
coordinates with a tolerance of 1e-9; without detail, the first triangle of the fan (v0, vi, vi+1) containing it. A point in no
triangle (rounding at the boundary) takes the plane of the triangle it violates least.

The winner minimises the squared 3D distance from `center` to the closest point; ties go to the lowest
(tile column, tile row, layer, polygon index). This is the multi-layer rule: on overlapping floors the one whose surface is
vertically closest to the query point wins (inside the outline the distance is purely vertical). The result carries the closest
point and `DistanceSquared`; `Poly` is `PolyRef.Invalid` if nothing qualifies.

Each tile builds, once and on first use, a bounding-volume hierarchy over its polygon boxes (recursive median split of the
longest axis of the node box, polygons sorted by centroid with the polygon index as tie-break, leaves of at most 4); it is cached
on the tile and only decides which polygons are examined, never the answer. If the box covers more grid cells than the mesh has
tile slots, all tiles are scanned instead of the covered cells.

## Path search

`BeginPath` / `UpdatePath(maxIterations)` / `FinishPath` (sliced), `FindPath` (all three at once), and the one-call
`FindPath(start, end, PathOptions)` which first finds the nearest polygon to each position.

**Cost model.** A route is a sequence of *crossings*. A crossing leaves a polygon through an *exit* and sits at a fixed
position:

- an in-tile edge shared with a neighbour: the midpoint of the edge (average of the two end vertices, in world coordinates);
- a tile-portal link: the point at fraction `(RangeMin + RangeMax) / 510` along the source edge from its first vertex;
- an off-mesh link: the link's far end point (the route first walks to the near end point inside the polygon, then crosses the link).

Cost of a leg inside polygon *P* from position *a* to position *b* is `|a − b| × AreaCost(area of P)` (3D Euclidean). Crossing
an off-mesh link adds `|start − end| × AreaCost(link area)`. The route starts at the start position in the start polygon and
ends with a leg from the last crossing to the end position inside the end polygon. The search returns the route of least total
cost for this model, so the cost equals Dijkstra's on the crossing graph (tests check it). Exits are numbered per polygon:
in-tile neighbour edges by edge index, then the polygon's links in link order.

**Algorithm.** A\* on crossings (node = leaving polygon + exit; a polygon can be entered through several nodes). Heuristic:
`MinAreaCost × 3D distance to the end position`. Open list: binary heap ordered by (f, insertion sequence), so the pop order is fully
specified; a stale heap entry (node already expanded) is skipped. One *iteration* is one expansion of a node (stale pops are free).
Reaching the end polygon adds a terminal node (cost of the final leg); the search ends when the terminal node is popped (`Success`).

**Partial paths.** If the open list empties, or the node limit (`maxNodes` of the query object, default 65536; new nodes beyond it
are dropped) stops the search from finding the end, the result is `Partial`: the corridor leads to the expanded node closest (3D) to the
end position (ties: the earliest expanded). `NodeLimitReached` tells which. An unfinished sliced search finished early returns
the same kind of corridor with status `InProgress`.

**Corridor.** Polygons along the route, start first. If the route left a polygon and came back, the loop is removed (the first
occurrence is kept), so every polygon appears once. A buffer shorter than the corridor receives its first part and `truncated` is set.
`Cost` is the route cost before loop removal.

**Slicing.** The result of `UpdatePath` calls totalling any number of iterations followed by completion equals the result of one
unlimited call, for every budget: slicing only decides where the same loop pauses. `UpdatePath(int)` uses the plain filter given to
`BeginPath`; `UpdatePath<TFilter>` must be given the same filter value each time. `FindPathAsync` runs slices of
`iterationsPerSlice` iterations, awaiting `Task.Yield()` between them, checks the cancellation token between slices (throws
`OperationCanceledException`) and returns exactly what the synchronous call returns.

## Straight path (string pulling)

`FindStraightPath(start, end, corridor, result)` first moves `start` and `end` to the closest points (nearest rule above) of the first
and last corridor polygons, then applies the simple stupid funnel algorithm (after Lee and Preparata 1984; Hershberger and
Snoeyink) to the portals between consecutive corridor polygons, in the X/Z plane. Portals:

- in-tile neighbours: the shared edge; the edge's first vertex is on the right and its second on the left when walking from the first
  polygon to the second (polygons are counter-clockwise);
- tile portals: the part of the source edge shared with the facing polygon's boundary edges on the opposite side, computed exactly from
  the two edges' extents along the boundary (not from the 1/255 link range); where the facing polygon has several collinear edges on
  that side the hull of their overlaps is used;
- off-mesh links (no shared edge): the funnel is closed at the link's entry point, the entry (flag `OffMeshConnection`) and exit points
  are emitted, and a new funnel starts at the exit. Reverse traversal swaps entry and exit.

Heights of corner points are the interpolated heights along the portal edge. A point equal in position to the previous one is merged
into it (flags are or-ed). Points carry `Start` / `End` / `OffMeshConnection` marks and the corridor polygon entered after the point
(the end point carries the last polygon). Output beyond the buffer sets `truncated`. A corridor whose neighbours do not touch, with
stale references or empty is `InvalidCorridor` and writes nothing. The result is the Euclidean shortest path inside the corridor in
X/Z (tests compare with a visibility-graph oracle).

## Raycast

`Raycast(startPoly, start, end, filter, visited)` follows the segment across the surface in the X/Z plane. In each polygon the exit
edge is found by clipping the segment against the convex outline: the leaving edge with the least parameter *t* (lowest edge index on
ties, e.g. through a vertex). If *t* ≥ 1 the segment ends inside: `Hit = false`, `T = 1`. Otherwise the ray crosses into the neighbour
across the edge if there is one the filter accepts: an in-tile neighbour, or a tile-portal link of that edge whose range
(`RangeMin − 1 .. RangeMax + 1` in 1/255ths of the edge) contains the crossing point. Otherwise it hits a wall there: `Hit = true`,
`T`, the outward unit normal of the edge in the X/Z plane (`Y = 0`), `HitPoly`, `HitEdge`. Off-mesh links are not crossed. Heights are
not tested. The start must be inside the polygon (within 1e-6 of its outline) or the status is `StartOutsidePolygon`. The visited list
(start first) is truncated to the buffer (`Truncated`); `visitedCount` is the number written.

## One-call and tier-1 forms

`NavMeshQuery.FindPath(start, end, PathOptions?)` = nearest polygon at each end (extents default (2, 4, 2)), sliced search run to
completion, funnel; a partial path ends at the closest point of the last corridor polygon to the snapped end. Status values:
`Success`, `Partial`, `NoStartNearby`, `NoEndNearby`, `InvalidCorridor` (cannot occur for results of the search itself). The
extension `TiledNavMesh.FindPath(a, b, options)` creates a query object per call; hold a `NavMeshQuery` when asking repeatedly.

## Debug geometry

`QueryDebug.Collect(query, result)` returns, as plain data, the outlines of the polygons expanded by the last search (each once, in
expansion order), the corridor outlines and the straight path's points.
