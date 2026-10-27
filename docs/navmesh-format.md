# Tiled navigation mesh format (draft, unstable)

Status: **draft**. The format may still change incompatibly until the 3D builder has landed; every file carries
`Version = 1` and header flag bit 0 (`Unstable`) set. A later stable format bumps the version and clears the flag,
so draft files are never mistaken for stable ones. See "What may still change" below.

The format serves two builders (an exact 2D polygon builder and a voxel-based 3D builder), the query layer and
crowds. Nothing here is specific to one builder.

## Concepts

- A **mesh** (`TiledNavMesh`) is a grid of **tiles** plus the parameters that tie them together (`NavMeshParams`).
  One mesh serves one agent class (radius, height, climb, slope are recorded in the parameters). Meshes for several
  agent sizes are several meshes over the same tile grid, each built with its own agent values; they share nothing.
- A **tile** is addressed by `(TileX, TileZ, Layer)`. Layers hold overlapping floors (bridges, multi-storey
  buildings); tiles of different layers at the same column/row are independent and do not connect through
  their interiors. `Layer` is 0 to 1023.
- A tile holds convex **polygons** over shared **vertices**, per-edge adjacency, **areas** and **flags**,
  optional per-polygon **height detail**, and **off-mesh link slots**.
- **Axes:** Y is up. The walkable surface is described in the X/Z plane. A polygon is convex and
  counter-clockwise when X points right and Z points up (positive signed area `Σ(xᵢ·zᵢ₊₁ − xᵢ₊₁·zᵢ)`).
  A builder that works in an X/Y plane maps its second axis to Z.

## Coordinate rule

- World positions cross the public boundary as `double` and must lie strictly within ±2^21 (2 097 152); anything
  else is rejected with `ArgumentOutOfRangeException` naming the value and the bound (a Q31.32 fixed-point
  host converts exactly inside this range).
- `NavMeshParams.TileSize` is a double that must be exactly representable as float32, so the boundary
  coordinate `(float)TileSize` is the same number in double and float32.
- Tile origin: `OriginX + TileX·TileSize`, `OriginZ + TileZ·TileSize` (double products and sums, evaluated exactly in this
  order: product, then sum). Tile Y origin is free (`OriginY` of the tile, finite, within ±2^21).
- Vertices are float32 relative to the tile origin. Bounds: X and Z in `[−TileSize, 2·TileSize]` (builders may
  overlap the neighbours by up to one tile), |Y| ≤ `MaxLocalExtent`.
- **Resolution rule.** `TileSize ≤ 8192·CellSize` and `MaxLocalExtent = 16384·CellSize`. The largest local
  coordinate therefore has float32 spacing at most `CellSize·2^14·2^-23 = CellSize/512`, so every vertex is
  stored with at least a 512th of a cell of resolution. This is the "margin".
- The parameters are checked as a whole (`NavMeshParams.Validate`): finite origins within the bound,
  positive finite cell size, tile size rule above, 3 ≤ `MaxVertsPerPoly` ≤ 16, non-negative climb/radius/height,
  slope in 0..90 degrees.

## Tile contents and rules

All rules are enforced by `NavTile.Create` (hand-built data) and by the deserializer; violations throw
`NavMeshFormatException` naming the polygon, edge or array.

| Array | Meaning |
|---|---|
| `Vertices` | 3 float32 per vertex (x, y, z); at most 65 536 vertices; all finite; nonzero X/Z magnitudes ≥ 1e-30 |
| `PolyVertexCounts` | per polygon, 3 .. `MaxVertsPerPoly`; at most 1 048 575 polygons |
| `PolyIndices` | all polygons' vertex indices concatenated (uint16) |
| `EdgeNeighbors` | per polygon edge (edge *i* runs from vertex *i* to *i+1* cyclically): 0 = wall or portal, *k* > 0 = polygon *k−1* of this tile |
| `EdgePortalSides` | per polygon edge: 255 = none, else the tile side the edge lies on: 0 = +X, 1 = +Z, 2 = −X, 3 = −Z |
| `PolyAreas` | per polygon, 0..63 |
| `PolyFlags` | per polygon, 16 bits, opaque to the format |
| `DetailMeshes` | per polygon `{VertexBase, TriangleBase, VertexCount, TriangleCount}`; all zero = no detail (the 2D builder leaves it so) |
| `DetailVertices` | 3 float32 per extra detail vertex (same bounds as vertices) |
| `DetailTriangles` | 4 bytes per triangle: three indices and an edge-flag byte |
| `OffMeshLinks` | see below |

Polygon rules: distinct vertex indices; no zero-length edge in X/Z; **convex and counter-clockwise**, checked with
the exact orientation predicate: every other vertex lies left of or on every edge, and at least one strictly left
(collinear vertices are allowed, self-overlapping star shapes and zero-area polygons are not).

Adjacency rules: an in-tile neighbour must be another polygon that has the reversed edge (same two vertex indices,
opposite direction) naming the first polygon back. An edge with an in-tile neighbour has no portal side. A portal
edge has neighbour 0, lies exactly (bitwise) on the coordinate `(float)TileSize` (sides 0, 1) or `0` (sides 2, 3) of its axis
(X for sides 0 and 2, Z for 1 and 3), and has nonzero length along the boundary.

Detail: for polygon *p* with *n* vertices and detail mesh *m*, triangle indices *i* < *n* refer to the polygon's own
vertices and *n ≤ i < n + m.VertexCount* to detail vertices `m.VertexBase + (i − n)`. Ranges must lie inside the
arrays. Edge-flag bit *k* of a triangle marks its edge *k* as lying on the polygon boundary (builders choose; the
format only carries it).

### Off-mesh link slots

Per link: start and end (tile-local float32 x, y, z, each |v| ≤ `MaxLocalExtent`; the end may lie in any tile, expressed in the
owning tile's frame), `Radius` (> 0), `Area` (0..63), `PolyFlags`, `Bidirectional`, `UserId`.
The slot only records the link; stitching resolves it (below).

## References

`PolyRef` is a 64-bit value: **salt (24 bits) | tile slot (20 bits) | polygon index (20 bits)**, from the top. Salt 0 is
never issued, so the zero value is "invalid". A mesh has numbered tile **slots** (at most 1 048 575). Every slot has
a generation (salt) starting at 1; it advances (`salt = (salt + 1) & 0xFFFFFF`, skipping 0) whenever the slot's tile is
removed or replaced. A reference is valid iff its slot's current salt equals the reference's salt and the slot holds a tile
with that polygon index. A new tile takes the **lowest free slot**. `TileRef` is the same (salt, slot) pair.
Replacing a tile (same position) keeps the slot and advances the salt: old references are rejected even if the new tile
looks identical. Salts and the slot layout are part of the mesh's serialized state.

## Stitching (links)

Links join polygons across tile boundaries and through off-mesh links. They are **derived**, not serialized: the set of
links is a pure function of the set of tiles and is rebuilt on every add, remove and replace, so any order of edits
that ends in the same tiles in the same slots gives the same links.

**Portals.** For every portal edge A (side *s*, in tile T) and every portal edge B on side *s* ⊕ 2 of every
layer of the tile column `(TileX + dx(s), TileZ + dz(s))` (dx, dz: +1/0, 0/+1, −1/0, 0/−1 for sides 0..3):
let `u` be the coordinate along the shared boundary (Z for sides 0, 2; X for sides 1, 3), and
`[lo, hi]` the intersection of the two edges' `u` intervals. A link A→B exists iff `hi > lo` and the world heights
(`tile.OriginY + y`, double) of A and B, linearly interpolated along each edge at `u = lo` and at `u = hi`, differ by at most
`WalkableClimb` at both. The test is symmetric, so links come in pairs. The link stores the covered part of A's edge as
`RangeMin..RangeMax`, in 1/255ths measured from A's first vertex: `floor(min t·255)` and `ceil(max t·255)` of the parameters
of `lo` and `hi`, clamped to 0..255.

**Off-mesh links.** The start point is attached to the polygon *of the owning tile*, the end point to a polygon of any tile layer in the
tile column under the end point. A point attaches to the polygon with the smallest score among those whose horizontal distance to the
polygon (0 when inside; Euclidean to the nearest edge point otherwise) is ≤ `Radius` and whose vertical
distance outside the polygon's vertex height range is ≤ `WalkableClimb`; score = `distance² + dy²`; ties go to
the lower layer (end point) and then the lower polygon index. A link that cannot attach at either end stays unconnected (no error).
A forward link (kind `OffMesh`) leaves the start polygon; a bidirectional link also gives a reverse link
(kind `OffMeshReverse`) leaving the end polygon.

**Order.** The links of a polygon are ordered: portal links in (side, portal order of the tile, neighbour layer,
neighbour portal order), then forward off-mesh links in link order, then reverse off-mesh links in (owner slot, link index) order.
(Re-stitching happens for the changed column and its four side neighbours; if any tile of the mesh has off-mesh links,
for every tile, because an end point can be anywhere.)

## Byte layout

Little-endian throughout (`BinaryPrimitives`); floats and doubles by their IEEE bit pattern (so `-0.0` and NaN payloads
are preserved, although NaN is rejected in coordinates); no padding except where listed. The same mesh writes the same
bytes on every platform.

### Tile (`ONPT`)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `4F 4E 50 54` |
| 4 | 2 | version (1) |
| 6 | 2 | flags (bit 0 = unstable/draft; other bits must be 0) |
| 8 | 12 | TileX, TileZ, Layer (int32) |
| 20 | 24 | origin X, Y, Z (double). X and Z must equal the grid formula bitwise |
| 44 | 24 | counts (uint32): vertices *nv*, polygons *np*, indices *ni*, detail vertices *ndv*, detail triangles *ndt*, off-mesh links *nom* |
| 68 | 12·nv | vertices (float32 x, y, z) |
| | 16·np | polygon records: u8 vertex count, u8 area, u16 flags, u32 detail vertex base, u32 detail triangle base, u16 detail vertex count, u16 detail triangle count |
| | 2·ni | polygon indices (uint16) |
| | 4·ni | edge neighbours (int32) |
| | ni | edge portal sides (uint8) |
| | 12·ndv | detail vertices |
| | 4·ndt | detail triangles |
| | 36·nom | off-mesh records: 7 float32 (start xyz, end xyz, radius), u8 area, u8 flags (bit 0 bidirectional; other bits 0), u16 poly flags, u32 user id |

The body must be exactly as long as the counts imply; the reader checks this before allocating. Anything else
(trailing bytes included) is an error. A tile is read against the mesh parameters (bounds, `MaxVertsPerPoly`).

### Mesh (`ONPM`)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `4F 4E 50 4D` |
| 4 | 2 | version (1) |
| 6 | 2 | flags (bit 0 = unstable) |
| 8 | 40 | OriginX, OriginY, OriginZ, TileSize, CellSize (double) |
| 48 | 2 | MaxVertsPerPoly (u16) |
| 50 | 2 | reserved, 0 |
| 52 | 16 | WalkableClimb, AgentRadius, AgentHeight, AgentMaxSlopeDegrees (float32) |
| 68 | 4 | slot count *S* (uint32, ≤ 1 048 575, and ≤ remaining bytes / 5) |
| 72 | … | per slot, in slot order: u32 salt (1..0xFFFFFF), u8 present (0/1), and if present u32 byte length followed by a tile |

Reading restores the slots exactly (free slots and generations too) and stitches once. Duplicate (column, row, layer)
positions are an error.

## Behaviour on bad input

Deserialization and `NavTile.Create` never index out of range on hostile input: counts are checked against the remaining
bytes first, then every rule above. All failures are `NavMeshFormatException` (parameters outside their bounds read
from a file are reported the same way). The mesh API throws `ArgumentOutOfRangeException` for world coordinates beyond the
bound, `InvalidOperationException` for adding a tile into an occupied position, and `ArgumentException` for a tile built with
other parameters.

## Debug geometry

`NavMeshDebug.Collect(mesh)` returns plain data: each polygon's world-space outline, area and flags, and every
stitched link as a world-space segment (the covered part of the shared edge for a portal, departure to arrival
for an off-mesh link), in slot / polygon / link order.

## Hand-over from builders

- **2D builder:** `NavTileAssembler.FromPolygons` takes world-space double vertices, per-polygon vertex counts and
  indices (polygons already cut to one tile and sharing vertices by index), optional areas and flags. It converts to tile-local
  float32, snaps coordinates within `CellSize/1024` of a tile boundary onto it, reverses clockwise polygons,
  derives in-tile adjacency from reversed shared edges (an edge shared by other than two polygons stays a wall) and marks wall
  edges lying on a boundary as portals. Detail is left empty.
- **3D builder (not yet built):** fills a `NavTileData` directly from its polygon mesh and detail mesh: polygon
  vertices as tile-local float32, `DetailMeshes` / `DetailVertices` / `DetailTriangles` from the detail stage,
  `EdgePortalSides` from the contour edges that touch a tile border, `Layer` from the layer partitioner,
  `OffMeshLinks` from the link markup. `NavTile.Create` then validates it.

## What 0.3 may still change

The 3D builder's real output may require: a different polygon vertex limit or index width; per-edge flags beyond the portal side
(for example "jumpable" or "crouch" edge markers); per-polygon "walkable height" / clearance; wider area ids; a tile-level bounding volume tree
or a baked per-tile layer-cell map; detail triangle edge-flag semantics; a heightfield or compressed-height
alternative to the detail mesh; and off-mesh link start/end polygon indices baked in by the builder instead of resolved at
stitch time. Any of these bumps `Version` and keeps the `Unstable` flag until the format is declared stable.
