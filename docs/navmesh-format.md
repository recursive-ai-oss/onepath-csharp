# Tiled navigation mesh format (version 2, frozen)

Status: **frozen at version 2**. Header flag bit 0 (the draft marker) is always 0. The reader accepts version 2 only: version 1 (the
draft layout) and every unknown version are rejected with `NavMeshVersionException`, whose message and `Found`/`Expected`
properties name the version found and the one expected. Any later layout change is version 3 with a reader for version 2.
See "Version policy" and "Build fingerprint" below.

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
| 4 | 2 | version (2) |
| 6 | 2 | flags (0; any bit set is an error) |
| 8 | 16 | build fingerprint of the mesh the tile was built for (below) |
| 24 | 12 | TileX, TileZ, Layer (int32) |
| 36 | 24 | origin X, Y, Z (double). X and Z must equal the grid formula bitwise |
| 60 | 24 | counts (uint32): vertices *nv*, polygons *np*, indices *ni*, detail vertices *ndv*, detail triangles *ndt*, off-mesh links *nom* |
| 84 | 12·nv | vertices (float32 x, y, z) |
| | 16·np | polygon records: u8 vertex count, u8 area, u16 flags, u32 detail vertex base, u32 detail triangle base, u16 detail vertex count, u16 detail triangle count |
| | 2·ni | polygon indices (uint16) |
| | 4·ni | edge neighbours (int32) |
| | ni | edge portal sides (uint8) |
| | 12·ndv | detail vertices |
| | 4·ndt | detail triangles |
| | 36·nom | off-mesh records: 7 float32 (start xyz, end xyz, radius), u8 area, u8 flags (bit 0 bidirectional; other bits 0), u16 poly flags, u32 user id |

The body must be exactly as long as the counts imply; the reader checks this before allocating. Anything else
(trailing bytes included) is an error. A tile is read against the mesh parameters (bounds, `MaxVertsPerPoly`), and its fingerprint must equal the parameters' fingerprint, otherwise `NavMeshFingerprintException` (a `NavMeshFormatException`) is thrown.

### Mesh (`ONPM`)

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `4F 4E 50 4D` |
| 4 | 2 | version (2) |
| 6 | 2 | flags (0) |
| 8 | 40 | OriginX, OriginY, OriginZ, TileSize, CellSize (double) |
| 48 | 2 | MaxVertsPerPoly (u16) |
| 50 | 2 | reserved, 0 |
| 52 | 16 | WalkableClimb, AgentRadius, AgentHeight, AgentMaxSlopeDegrees (float32) |
| 68 | 8 | settings digest (u64; below) |
| 76 | 4 | slot count *S* (uint32, ≤ 1 048 575, and ≤ remaining bytes / 5) |
| 80 | … | per slot, in slot order: u32 salt (1..0xFFFFFF), u8 present (0/1), and if present u32 byte length followed by a tile |

The parameter block is offsets 8..75 (68 bytes); snapshots reuse it verbatim. Reading restores the slots exactly (free slots and generations too) and stitches once. Duplicate (column, row, layer)
positions are an error.

### Tier-1 envelopes (`ONNM`, `ONNW`; added in release 1.1.0)

`NavMesh.Save` wraps the frozen mesh bytes above, unchanged, in an envelope that also carries the `NavSettings` the mesh was built with, so
`NavMesh.Load` returns a tier-1 mesh without the caller repeating the settings. The mesh layout and version stay 2. All integers are little-endian.

`ONNM` (one mesh):

| Field | Size | Notes |
|---|---|---|
| magic | 4 | `4F 4E 4E 4D` |
| version | 2 | 1 |
| flags | 2 | 0 |
| AgentRadius, AgentHeight, WalkableClimb, AgentMaxSlopeDegrees, CellSize, TileSize, CellHeight, OriginX, OriginZ | 9 × 8 | double |
| MaxVertsPerPoly | 4 | i32 |
| FloorY | 8 | double |
| Area, Flags | 1 + 2 | u8, u16 |
| SnapBits present, SnapBits | 1 + 4 | u8 (0/1), i32 (0 when absent) |
| Partition, MinRegionArea, MergeRegionArea | 3 × 4 | i32 each; Partition must be a defined value |
| ContourMaxError, MaxEdgeLength, DetailSampleDistance, DetailMaxError | 4 × 8 | double |
| link count *L* | 4 | u32, at most remaining bytes / 64 |
| per link | 64 | Start xyz, End xyz, Radius (7 × double), Bidirectional u8 (0/1), Area u8, Flags u16, UserId u32 |
| mesh length | 4 | u32 |
| mesh | … | the `ONPM` bytes above; nothing may follow |

The reader builds the settings through their validating setters (a value out of range is a `NavMeshFormatException` naming it), reads the
mesh, and requires the parameter block the settings produce (grid, agent class and settings digest) to equal the mesh's, else
`NavMeshFingerprintException`. A caller-supplied `NavSettings` is checked the same way. Bare `ONPM` bytes are also accepted by `NavMesh.Load`
when the caller supplies the settings.

`ONNW` (a `NavWorld`): magic `4F 4E 4E 57`, u16 version 1, u16 flags 0, u32 class count (1 or more, at most remaining bytes / 8), then per
class a u32 length and an `ONNM` envelope; nothing may follow. The classes keep their order.

## Behaviour on bad input

Deserialization and `NavTile.Create` never index out of range on hostile input: counts are checked against the remaining
bytes first, then every rule above. All failures are `NavMeshFormatException` or a subclass (`NavMeshVersionException`, `NavMeshFingerprintException`) (parameters outside their bounds read
from a file are reported the same way). The mesh API throws `ArgumentOutOfRangeException` for world coordinates beyond the
bound, `InvalidOperationException` for adding a tile into an occupied position, and `NavMeshFingerprintException` for adding, replacing,
streaming in or restoring a tile built with other settings.

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
- **3D builder:** fills a `NavTileData` directly from its polygon mesh and detail mesh: polygon
  vertices as tile-local float32, `DetailMeshes` / `DetailVertices` / `DetailTriangles` from the detail stage,
  `EdgePortalSides` from the contour edges that touch a tile border, `Layer` from the layer partitioner,
  `OffMeshLinks` from the link markup. `NavTile.Create` then validates it.

## Freeze decisions (candidates collected through the 3D builder, authored links, agent classes and streaming)

Every item the draft listed as possibly changing, with what the builders, queries, crowds, authoring and streaming showed. Only the fingerprint row
changes the layout; the others are decisions to keep it.

| Candidate | Decision | Reason |
|---|---|---|
| Polygon vertex limit, index width | **Keep**: 3 to 16 vertices (builders use 6), `uint16` vertex indices, 65 536 vertices and 1 048 575 polygons per tile | Builders stay far inside on every test level; a tile that would exceed them fails with a message naming the tile and advising a smaller tile size. |
| Per-edge flags ("jumpable", "crouch") | **Reject** | Jumps and ladders are off-mesh links (authored, below); crouch and similar are polygon areas and flags. Edge flags would double the per-edge data for no query that needs them. |
| Per-polygon walkable height / clearance | **Reject** | One mesh per agent class (`NavWorld`) already bakes clearance into each class's polygons. |
| Wider area ids | **Keep** 6 bits (0 to 63) | Matches the cost table size of queries and filters; 64 areas covers every use found. |
| Stored tile bounding-volume tree | **Reject** | Derived data: queries find polygons by tile bounds and a scan of the tile; storing a tree would make equal geometry unequal on disk. |
| Layer-cell map | **Reject** | Derived from the polygons when needed. |
| Heightfield detail instead of a detail mesh | **Reject** | The detail mesh is what the 3D builder produces and what height queries read; a second representation would need a second conversion. |
| Off-mesh start/end polygon indices baked into the tile | **Reject** | They depend on the neighbouring tile (the end may be in another tile) and would make a tile's bytes depend on its neighbours. Stitching resolves them (rules above); authored links need nothing more than the slot. |
| Detail edge-flag semantics | **Keep** (bit *k* = triangle edge *k* lies on the polygon boundary) | The 3D builder follows it. |
| Wider layer index | **Keep** 0 to 1023 | Builders report the layer count needed; no level needs more than a few. |
| A parameter fingerprint in the tile header | **Add** (version 2) | A tile built for another mesh (other cell size, tile size, agent class, partition, detail) would otherwise be accepted when streamed in whenever its numbers pass validation. The 16-byte fingerprint below is compared on add, replace, stream-in and snapshot restore. |

Nothing else in the draft layout changed: version 2 is the draft layout plus the tile fingerprint, the settings digest in the parameter
block, version 2 and flag bit 0 clear.

## Build fingerprint

A 128-bit value identifying every setting that changes a tile's content. It is **not** stored in the mesh header (the mesh recomputes it
from its parameter block); every tile header carries it.

- **Input bytes:** the 4 ASCII bytes `ONFP`, the format version as u16 (2), then the 68-byte parameter block exactly as the mesh header
  stores it (origin X/Y/Z, tile size, cell size as f64; max vertices per polygon u16; 0 u16; walkable climb, agent radius, agent height,
  agent maximum slope as f32; settings digest u64), all little-endian IEEE bit patterns.
- **Hash:** SHA-256 (FIPS 180-4) of those 74 bytes; the fingerprint is the first 16 bytes of the digest, written to the tile in digest order.
- **Settings digest:** the builder settings that change tile content but are not parameter fields. `NavSettings.ToParams` writes the first
  8 bytes of SHA-256 over: `ONSD` (ASCII), then CellHeight (f64), SnapBits (i32; -1 when automatic), Partition (i32 enum value),
  MinRegionArea (i32), MergeRegionArea (i32), ContourMaxError (f32), MaxEdgeLength (f64), DetailSampleDistance (f64), DetailMaxError (f64),
  Height (f64), Area (u8), Flags (u16), authored link count (u32) and per link start xyz, end xyz, radius (f64 each), bidirectional (u8),
  area (u8), flags (u16), user id (u32); read as a little-endian u64. Hand-built parameters have digest 0.
- **Where it is checked:** `TiledNavMesh.AddTile`/`ReplaceTile`, `NavMeshSerializer.ReadTile` (so `TileStreamer`), and snapshot restore
  (the snapshot's parameter block carries the digest, and library tiles and target meshes must have the same fingerprint). The check
  precedes every other validation of the tile.
- Tiles take the fingerprint from the parameters they are created against (`NavTile.Create`), so builders set nothing. Because it is part of
  a tile's bytes it is part of the tile content id.

## Version policy

The layout above is frozen. A change to any byte of a tile, mesh or snapshot layout, to a rule that decides validity, or to the
fingerprint's encoding is **version 3**, and its reader also reads version 2. A reader rejects any version it does not know, and any set
flag bit, with a typed error. Adding a field is never done in place.

## Authoring: off-mesh links, agent classes, streaming

**Authored links** (`NavSettings.OffMeshLinks`, a list of `OffMeshLinkDefinition`): world-space start and end (double, inside the
coordinate rule), radius, direction, area, flags and a caller id. Every build (2D and 3D) and every rebuild of a tile bakes into the
slots of the tile that holds the link's start:

1. The start's column is `WorldToTile(start.x, start.z)`. If the build produced no tile there, the link is dropped.
2. With several layers in the column, the link goes to the layer whose polygon is nearest the start by the stitching rule
   (horizontal distance to the polygon within the radius, height outside the polygon's vertex range within the climb; score
   horizontal² + vertical²); layers with no polygon in reach score infinity; ties and all-infinity go to the lowest layer.
3. Slot values are `(float)(world − tileOrigin)`, X and Z against the tile's origin, Y against the tile's `OriginY`. The end may
   lie in any column but within `MaxLocalExtent` of the start's tile origin in X and Z; otherwise the build throws
   `ArgumentOutOfRangeException` naming `OffMeshLinks[i]`. Non-finite or out-of-bound numbers, a radius not above 0, an area above 63 are
   rejected the same way. A link whose start or end lies more than `MaxLocalExtent` above or below the tile's `OriginY` is dropped (it could not connect from there anyway, and one odd link must not fail a build or rebuild).
4. Links of one tile are appended after any slots the builder already filled, in definition order. A tile's link order therefore
   equals the order of the definitions whose start it holds.

Resolving to polygons, one-way and two-way behaviour, run-time switches and the corridors' crossing all work as stitching defines.
A link that finds no polygon within its radius at either end stays unconnected.

**Agent classes** (`NavWorld`): one `NavMesh` per `NavSettings`, built from the same input in the order given; class *i* is
byte-identical to a single-class build with `classes[i]`. `ClassFor(radius)` is the class with the smallest `AgentRadius` ≥ radius
(lowest index among equals), or the largest class if none is large enough. `WorldCrowd` holds one crowd per class and steps them
in class order; agents of different classes walk different meshes and do not see each other.

**Streaming** (`TileStreamer`): operations are queued and applied in queue order, `Update(maxOperations)` applying at most that many.
One operation is one tile added or replaced (`ReadTile` against the mesh's parameters, then `AddTile` or `ReplaceTile`), or one tile
column removed (all layers, lowest first). A failing operation leaves the queue before its exception propagates. Adding takes the
lowest free slot and advances slot generations as for any edit, so tile bytes and query answers return after out-and-in but polygon
references and slot numbers do not. Links stitch again after every operation. A crowd whose destination polygon streamed out has no
route (`NoPath`); the host sends it again after the tile returns. The queue is not part of a mesh snapshot.
