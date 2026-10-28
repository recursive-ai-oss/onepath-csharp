#!/usr/bin/env python3
"""Small tile-map shapes with a known exact answer, for comparing builders' walkable coverage.

    tools/geom/gen_shapes.py <out_dir>

Each shape is a square map of 8x8 tiles (2 world units per tile) written as flat triangles to <name>.obj; the map's
whole grid is written, so every builder sees the same 16x16 bounds. exact.tsv gives, per shape and agent radius, the area of
the exact eroded footprint: the points of the floor that are at least the radius away from every wall (round joins).
The areas come from a fine point sampling (step 1/200 unit, error well under 0.1%) with distances computed exactly
against the tile rectangles.
"""
import math
import os
import sys

TILE = 2.0
N = 8
# 0.5, 0.75 and 1.0 are the agent radii of the rows; the others are those plus one cell (0.125 or 0.25).
RADII = (0.5, 0.625, 0.75, 0.875, 1.0, 1.125, 1.25)

SHAPES = {
    # an L of five tiles: two arms one tile wide meeting at a reflex corner
    "l-room": {(1, 1), (1, 2), (1, 3), (2, 3), (3, 3)},
    # a straight corridor one tile (2 units) wide and five tiles long
    "corridor": {(i, 3) for i in range(1, 6)},
    # a single 2x2 tile on its own
    "single-tile": {(3, 3)},
    # an open room of 3x3 tiles with a square pillar tile in the middle
    "pillar-room": {(i, j) for i in range(1, 4) for j in range(1, 4)} - {(2, 2)},
}


def write_obj(path, floor):
    verts, tris = [], []
    idx = {}
    for i in range(N + 1):
        for j in range(N + 1):
            idx[(i, j)] = len(verts)
            verts.append((i * TILE, 0.0, j * TILE))
    for (i, j) in sorted(floor):
        a, b, c, d = idx[(i, j)], idx[(i + 1, j)], idx[(i + 1, j + 1)], idx[(i, j + 1)]
        tris.append((a, d, c))
        tris.append((a, c, b))
    with open(path, "w", newline="\n") as f:
        for x, y, z in verts:
            f.write("v %.5f %.5f %.5f\n" % (x, y, z))
        for a, b, c in tris:
            f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))


def eroded_area(floor, r, step=0.005):
    walls = [(i, j) for i in range(-1, N + 1) for j in range(-1, N + 1) if (i, j) not in floor]
    area = 0.0
    for (ti, tj) in sorted(floor):
        k = int(TILE / step)
        for a in range(k):
            x = ti * TILE + (a + 0.5) * step
            for b in range(k):
                z = tj * TILE + (b + 0.5) * step
                ok = True
                for (wi, wj) in walls:
                    if abs(wi - ti) > 2 or abs(wj - tj) > 2:
                        continue
                    dx = max(wi * TILE - x, 0.0, x - (wi + 1) * TILE)
                    dz = max(wj * TILE - z, 0.0, z - (wj + 1) * TILE)
                    if dx * dx + dz * dz < r * r:
                        ok = False
                        break
                if ok:
                    area += step * step
    return area


def main(argv):
    if not argv:
        print(__doc__)
        return 2
    out = argv[0]
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "exact.tsv"), "w", newline="\n") as ex:
        ex.write("shape\tradius\tfloor_area\texact_eroded_area\n")
        for name, floor in SHAPES.items():
            write_obj(os.path.join(out, name + ".obj"), floor)
            for r in RADII:
                ex.write("%s\t%g\t%g\t%.4f\n" % (name, r, len(floor) * TILE * TILE, eroded_area(floor, r)))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
