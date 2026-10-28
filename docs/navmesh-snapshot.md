# Navigation mesh snapshots

Status: follows the frozen mesh format, version 2 (`navmesh-format.md`); the snapshot header carries the same version as
the mesh format. Its parameter block includes the settings digest, so a snapshot carries the build fingerprint: restoring it into a mesh
with a different fingerprint, or against library tiles built with another one, fails with `NavMeshFingerprintException`.

A deterministic host that rolls back and resimulates needs to put a mesh back exactly as it was. A **snapshot** is the
dynamic state of a `TiledNavMesh`, as bytes. Restoring it gives a mesh in which every query, every stitched link and
every polygon reference behaves as it did when the snapshot was taken, and which snapshots back to the same bytes.

## What is dynamic state

Everything about a mesh that can change after construction:

1. **Slots.** For each slot index 0..*S*−1 (S = `SlotCount`): its generation (salt), and the tile in it, if any. Slot
   indices, free slots and generations are state because polygon references embed them: after a restore a reference
   issued before the snapshot is valid exactly when it was valid at that moment, and a stale one stays stale.
2. **Polygon edits.** Run-time changes to a polygon's area and flags (`SetPolyArea`, `SetPolyFlags`): a door that
   opens, a zone that closes. Stored as differences from the tile as built.
3. **Off-mesh link switches.** Links turned off with `SetOffMeshLinkEnabled`. All links start on.

Stitched links, bounding-volume trees and the like are derived data: they are rebuilt on restore by the same
deterministic stitching that built them (`navmesh-format.md`, "Stitching"), so they are never stored.

Tile contents (geometry, adjacency, detail, off-mesh slots, as-built areas and flags) are state of the *world*, not of
the run: they are stored in full (`Full`) or named by content id (`Referenced`).

Not state: the mesh parameters (stored only to check that a snapshot belongs to the mesh it is restored into), query
objects and their buffers.

Temporary obstacles, run-time area volumes and the rebuild queue of a 3D-built mesh are state of the tile cache; they wrap this snapshot
rather than extending it (see "Tile cache snapshot"), so this format is unchanged by them.

## Tile content id

`TileContentId` = SHA-256 of the tile serialized as in `navmesh-format.md` ("Tile") **with the as-built polygon areas and
flags**, i.e. ignoring run-time edits. SHA-256 is FIPS 180-4, computed by the library itself (the platform's hash is not
available on every target). Two tiles with equal ids have equal geometry, adjacency, detail, off-mesh slots, areas and
flags as built. `NavTile.ContentId` returns it; `TileLibrary` finds tiles by it.

## Ownership of tiles

A mesh keeps its **own copy** of every tile added to it: the immutable arrays (vertices, indices, detail, off-mesh
slots) are shared, and the areas, flags and link switches are copied. Editing a mesh therefore never changes the `NavTile`
object the caller passed in, nor another mesh holding the same object. `GetTile` returns the mesh's copy. Adding a tile
that was taken from an edited mesh carries its current areas and flags and link switches with it; the as-built values
(for the content id and for snapshot differences) stay those of the original build. `ReplaceTile` resets a position's edits
to the new tile's values.

## Modes

| Mode | Tile entry | Restore needs |
|---|---|---|
| `Full` (default) | the tile's as-built bytes | nothing: self-contained |
| `Referenced` | the 32-byte content id | a `TileLibrary` holding every named tile |

A host that keeps its tile data (it loaded or built them) takes `Referenced` snapshots every frame cheaply and keeps
`Full` ones for save files. One snapshot is all one mode; a reader also accepts a mixture. The writer is canonical: the same state in
the same mode always gives the same bytes, on every platform.

## Byte layout

Little-endian throughout; the same conventions as the mesh format.

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `4F 4E 50 53` ("ONPS") |
| 4 | 2 | version (2) |
| 6 | 2 | flags (0) |
| 8 | 68 | the mesh parameters, exactly as in the mesh format (offsets 8..75) |
| 76 | 4 | section count *N* (≤ remaining bytes / 8) |
| 80 | … | *N* sections |

### Sections

Each section: u16 id, u16 reserved (0), u32 body length, body. Sections appear in strictly ascending id order, each at
most once. A reader of this version knows section 1 only and **rejects any other id**: an unknown section is state it
cannot restore, so skipping it would silently lose it. The snapshot must contain section 1. A later version that adds
sections (temporary obstacles, rebuilt-tile queues) raises the version number.

#### Section 1: slots

| Field | Size |
|---|---|
| slot count *S* (≤ 1 048 575 and ≤ remaining bytes / 5) | u32 |
| per slot, in slot order: | |
| generation (1 to 16 777 215) | u32 |
| kind: 0 free, 1 inline tile, 2 referenced tile | u8 |
| (kind 1) tile byte length, then the as-built tile bytes | u32 + bytes |
| (kind 2) tile content id | 32 bytes |
| (kinds 1, 2) polygon edit count *E* (≤ the tile's polygon count) | u32 |
| *E* edits, ascending polygon index: polygon u32, area u8, flags u16 | 7·*E* |
| (kinds 1, 2) disabled off-mesh link count *D* (≤ the tile's link count) | u32 |
| *D* link indices, strictly ascending | u32 each |

An edit is written only for a polygon whose current area or flags differ from the as-built values; a reader rejects an
edit equal to the as-built values (one state, one encoding). An area above 63 is rejected. Trailing bytes after the
last section, and section bodies not consumed exactly, are errors.

## Restore

`NavMeshSnapshot.Restore(bytes, library)` builds a new mesh; `TiledNavMesh.Restore(bytes, library)` replaces the state of
an existing mesh in place, keeping the object (query objects holding it keep working). Both:

1. Parse and validate everything first: header, parameters, section framing, every count against the bytes that remain
   before allocating, every inline tile completely (`NavTile` rules), every referenced tile found in the library with
   equal mesh parameters, edit and link indices against the tile, the uniqueness of (column, row, layer) positions.
2. Build the complete new state on the side: slots and generations placed exactly, edits and switches applied, then
   stitched once in slot order.
3. Only then, for the in-place form, swap it in. The swap cannot fail. Any error before it leaves the mesh untouched.

The in-place form also requires the snapshot's parameters to equal the mesh's, compared bitwise.

Errors are `NavMeshFormatException` with a message naming the slot or field. Other exceptions from hostile bytes are bugs.

## Guarantees (tested)

- snapshot → mutate → restore → snapshot gives identical bytes, in both modes;
- every query (nearest, path with filters and area costs, string pulling, raycast), the stitched links and the validity
  of polygon references, current and forged, equal those of a mesh that applied the same operations without the excursion;
- carrying on with the same operations after a restore keeps the two meshes byte-identical, step by step;
- snapshot bytes are equal on every platform and thread count (hash-matrix scenario `navmesh/snapshot-bytes`).

## Relation to `WriteMesh`

`NavMeshSerializer.WriteMesh` stores a mesh as a file: tiles with their *current* areas and flags, and no link switches.
It is for content, not for rollback. Reading it back makes the current values the as-built ones.

## Tile cache snapshot

`TileCache.Snapshot` (`build3d-runtime.md`) captures the state of a mesh built by the 3D builder with run-time changes: obstacles, area volumes, the
rebuild queue, and a complete mesh snapshot as above. Little-endian; the same state gives the same bytes in every mode.

| Field | Size |
|---|---|
| magic `4F 4E 54 4B` ("ONTK") | 4 |
| version (1) | u16 |
| flags (as the mesh format) | u16 |
| next obstacle id, next volume id (each 1 to 2^31 - 1) | u32, u32 |
| obstacle count *O* (at most 2^20 and the remaining bytes / 13) | u32 |
| per obstacle, ascending id (below the next id): id u32, shape u8 (1 cylinder, 2 box, 3 oriented box), then 5, 6 or 7 float64 as in `build3d-runtime.md` | |
| volume count *V* (at most 2^20 and the remaining bytes / 41) | u32 |
| per volume, ascending id (below the next id): id u32, area u8 (0 to 63), min y f64, max y f64, corner count *K* u32 (3 to 256), 2*K* f64 | |
| queue length *Q* (at most the number of cached tiles) | u32 |
| per queued tile, front first: tile column i32, tile row i32 (each distinct, each a cached tile) | |
| mesh snapshot length, then the mesh snapshot bytes (`Full` or `Referenced`) | u32 + bytes |

The obstacle and volume values go through their constructors again on restore, so every rule of the shape applies to hostile bytes. Restore
parses everything first, then restores the mesh (itself all or nothing), then swaps in the lists; an error leaves the cache untouched.
Trailing bytes are an error. A snapshot is restorable into a cache built from the same geometry and settings; a queued tile the cache has no data for
is rejected.
