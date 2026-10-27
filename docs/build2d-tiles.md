# 2D builder: tiles, plane mapping, rounding

Normative description of `TileSlicer`, `Mesh2DPipeline` and `NavMesh.Build2D`: a walkable `Region` (see
`build2d-geometry.md`) becomes a stitched `TiledNavMesh` (see `navmesh-format.md`). A later port that follows this
text produces the same tiles, bit for bit.

## Stages

1. **Resolve:** outlines or a tile map → `Region` (union of the outlines; true cells of a tile map). Obstacles are
   `Region.Difference`. This is the geometry document's job.
2. **Shrink:** `Region.Offset(-AgentRadius)` with round joins and the default arc tolerance; radius 0 skips it.
   One mesh serves one agent class; several agent sizes are several builds with different settings.
3. **Slice:** cut the region along the tile lines and partition each tile into convex polygons (below).
4. **Tile data:** `Mesh2DPipeline.ToTileData` per tile, through `NavTileAssembler.FromPolygons`.
5. **Assemble:** `NavTile.Create` validates each tile; `TiledNavMesh.AddTile` stitches neighbours.

Each stage is a public static method over plain types; `Mesh2DPipeline.Build` runs them in this order.

## Plane mapping

The geometry plane maps to the mesh as **x → world X, y → world Z**, and world Y is the constant
`NavSettings.Height` for every vertex (tile-local Y is 0, the tile's `OriginY` is the height). A polygon that is
counter-clockwise in the plane is counter-clockwise in X (right) / Z (up), which is the format's winding. There is no
detail mesh, one layer (0), one area id and one flags value for all polygons (`NavSettings.Area`, `NavSettings.Flags`).

## Snap grid and the format

The region's grid unit must be at least `CellSize / 256` (`TileSlicer.MinUnitDivisor`): then two grid-distinct points
stay distinct as tile-local float32 (the format keeps resolution of at least `CellSize / 512` everywhere) and none is
pulled onto a tile edge by the assembler's `CellSize / 1024` snap. A finer region is rejected with an
`ArgumentException` naming the `SnapBits` to use (`TileSlicer.FinestSnapBits(cellSize)`, the largest value up to 16
that satisfies the rule). `NavSettings.SnapBits` null means exactly that value. Features thinner than the unit vanish
in snap rounding, by design.

## Tile lines

Tile line k along an axis is the world coordinate `Origin + k * TileSize`, **computed once in double and rounded to
the grid** (`Math.Round`, ties to even, as for every input coordinate). A line must lie within `CellSize / 1024` of its
true position; otherwise the tiling is rejected (`ArgumentException`: use a tile origin and size on the grid or a finer
`SnapBits`). Tile k spans line k to line k + 1; a point on a line belongs to the tile above it.

## Slicing rule

1. Take the region's bounding box. Lines strictly inside the box (on neither extreme) are cut lines; each is one
   segment across the box, vertical for X lines and horizontal for Z lines. At most 2^16 lines per axis and 2^22
   crossings of X lines with Z lines are accepted.
2. The cut lines join the region's boundary edges in **one snap-rounded arrangement** (`build2d-geometry.md`,
   "Arrangement"): cut lines meet each other at grid points, so those crossings are exact; where a boundary edge
   crosses a line, the crossing is computed exactly as a rational and **rounded once** to the grid, and both the
   boundary and the line pass through the rounded point. Nothing is rounded twice and no step depends on the order of
   the input.
3. The constrained Delaunay triangulation uses boundary and cut segments as constraints, so **no triangle crosses a
   tile line**; the winding is that of the boundary alone (cut segments change nothing). Triangles inside the region
   are kept.
4. Each triangle belongs to the tile whose cell contains its centroid (compared in integers as the sum of the three
   vertices against three times the line). Tiles are processed in ascending (row, column) order.
5. Per tile, the triangles are merged into convex polygons exactly as for an untiled region (Hertel & Mehlhorn,
   greedy longest diagonal first), with at most `MaxVertsPerPoly` vertices; an edge on a tile line has no neighbour,
   so no polygon crosses a line. Unused vertices are dropped and the others keep their (x, y) order.

Consequences, all exact: every polygon is convex and counter-clockwise; polygons are valid on the exact grid, so
convexity and non-overlap never depend on float rounding; **both sides of a tile line are cut at the same vertices**,
because every vertex on a line is in the one arrangement, so portal edges of neighbouring tiles that overlap are
identical and every overlap is linked by stitching. The union of the tile polygons is the region, except that cut
points on the region's boundary move to the rounded crossing: the area changes by less than the boundary length times
one grid unit (and not at all when every crossing is a grid point, as for axis-aligned and 45 degree boundaries on
the integer grid).

## Coordinate rule

Outline and tile-map coordinates are double, finite and within ±2^21 (otherwise `ArgumentOutOfRangeException` naming
the value). The mesh format also bounds each **tile**: a tile whose extent leaves ±2^21 is rejected by the tile
validator with an `ArgumentOutOfRangeException` naming the tile, even if the region itself is inside the bound. Tile
size is at most 8192 cells.

## Presets

`NavSettings.Humanoid` (radius 0.6, height 2, climb 0.9, slope 45 degrees, cell 0.3, tile 64, 6 vertices per polygon),
`Small` (0.3, 1, 0.4, 45, cell 0.15, tile 32) and `Large` (1.5, 4, 1.2, 40, cell 0.5, tile 128). Floor height 0, area
0, flags 1.
