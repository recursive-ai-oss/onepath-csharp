#!/usr/bin/env python3
"""Generate navmesh benchmark levels (OBJ) and matching query-point files, from nothing but a seed.

    tools/geom/gen_levels.py <out_dir> [--size 128] [--seed 1] [--points 2000] [level ...]

Levels: terraces, ramps, stairs, overlapping, slivers, city, tilemap. Each writes <name>.obj and <name>.points
(one "x y z" per line: random points on upward-facing surfaces, to be paired up as path queries).
Output depends only on the arguments (own LCG, no platform randomness); the sine terrain in some levels
can differ in the last printed digit between C libraries, so benchmark rows record the generated file's SHA-256.
All geometry here is our own; nothing is taken from third-party levels.
"""
import math
import os
import sys


class Lcg:
    """64-bit LCG (Knuth MMIX constants); next() returns a float in [0,1)."""

    def __init__(self, seed):
        self.s = (seed * 2862933555777941757 + 3037000493) & 0xFFFFFFFFFFFFFFFF

    def next(self):
        self.s = (self.s * 6364136223846793005 + 1442695040888963407) & 0xFFFFFFFFFFFFFFFF
        return (self.s >> 11) / float(1 << 53)


class Mesh:
    def __init__(self):
        self.v = []
        self.t = []

    def vert(self, x, y, z):
        self.v.append((x, y, z))
        return len(self.v) - 1

    def tri(self, a, b, c):
        self.t.append((a, b, c))

    def quad(self, a, b, c, d):
        self.tri(a, b, c)
        self.tri(a, c, d)

    def box(self, x0, y0, z0, x1, y1, z1):
        """Closed box; all faces wound so normals point outward (top faces up)."""
        p = [self.vert(x, y, z) for y in (y0, y1) for z in (z0, z1) for x in (x0, x1)]
        # index = (yi*4) + (zi*2) + xi
        self.quad(p[4], p[6], p[7], p[5])  # top (+y)
        self.quad(p[0], p[1], p[3], p[2])  # bottom
        self.quad(p[0], p[4], p[5], p[1])  # -z
        self.quad(p[2], p[3], p[7], p[6])  # +z
        self.quad(p[0], p[2], p[6], p[4])  # -x
        self.quad(p[1], p[5], p[7], p[3])  # +x

    def grid(self, x0, z0, size, step, height):
        """Triangulated heightfield over [x0,x0+size]^2 with the given vertex spacing; up-facing."""
        n = int(size / step)
        idx = {}
        for i in range(n + 1):
            for j in range(n + 1):
                x, z = x0 + i * step, z0 + j * step
                idx[(i, j)] = self.vert(x, height(x, z), z)
        for i in range(n):
            for j in range(n):
                a, b, c, d = idx[(i, j)], idx[(i + 1, j)], idx[(i + 1, j + 1)], idx[(i, j + 1)]
                self.tri(a, d, c)
                self.tri(a, c, b)

    def sample_points(self, rng, count, min_up=0.85):
        """Area-weighted random points on triangles whose normal points up."""
        cum, tris = [], []
        total = 0.0
        for (a, b, c) in self.t:
            pa, pb, pc = self.v[a], self.v[b], self.v[c]
            u = (pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2])
            w = (pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2])
            n = (u[1] * w[2] - u[2] * w[1], u[2] * w[0] - u[0] * w[2], u[0] * w[1] - u[1] * w[0])
            ln = math.sqrt(n[0] ** 2 + n[1] ** 2 + n[2] ** 2)
            if ln < 1e-6 or n[1] / ln < min_up or ln < 1.0:  # skip sliver-sized triangles as spawn targets
                continue
            total += ln * 0.5
            cum.append(total)
            tris.append((pa, pb, pc))
        pts = []
        for _ in range(count):
            r = rng.next() * total
            lo, hi = 0, len(cum) - 1
            while lo < hi:
                mid = (lo + hi) // 2
                if cum[mid] < r:
                    lo = mid + 1
                else:
                    hi = mid
            pa, pb, pc = tris[lo]
            s1, s2 = rng.next(), rng.next()
            if s1 + s2 > 1:
                s1, s2 = 1 - s1, 1 - s2
            pts.append(tuple(pa[k] + s1 * (pb[k] - pa[k]) + s2 * (pc[k] - pa[k]) for k in range(3)))
        return pts

    def write(self, base, points):
        with open(base + ".obj", "w", newline="\n") as f:
            for x, y, z in self.v:
                f.write("v %.5f %.5f %.5f\n" % (x, y, z))
            for a, b, c in self.t:
                f.write("f %d %d %d\n" % (a + 1, b + 1, c + 1))
        with open(base + ".points", "w", newline="\n") as f:
            f.write("# x y z\n")
            for x, y, z in points:
                f.write("%.5f %.5f %.5f\n" % (x, y, z))


def smooth_noise(seed):
    """Sum of a few sinusoids with seeded phases: smooth, bounded, deterministic. (Generator-only; not a library function.)"""
    r = Lcg(seed)
    waves = [(0.02 + 0.06 * r.next(), 0.02 + 0.06 * r.next(), r.next() * 6.28, 0.5 + r.next()) for _ in range(5)]
    amp = sum(w[3] for w in waves)
    return lambda x, z: sum(a * math.sin(fx * x + fz * z + ph) for fx, fz, ph, a in waves) / amp


def level_terraces(size, seed):
    m = Mesh()
    f = smooth_noise(seed)
    rise = 0.45  # terrace height, below the default climb so risers are walkable only on the sloped parts

    def h(x, z):
        v = f(x, z) * 6.0
        k = math.floor(v / rise)
        frac = v / rise - k
        t = min(1.0, max(0.0, (frac - 0.7) / 0.3))  # steep riser in the last 30% of each band
        return rise * (k + t * t * (3 - 2 * t))

    m.grid(0, 0, size, 1.0, h)
    return m


def level_ramps(size, seed):
    m = Mesh()
    m.grid(0, 0, size, size / 8, lambda x, z: 0.0)
    angles = [20, 30, 40, 44, 46, 50, 60, 70]  # across the 45 degree walkable-slope threshold
    for i, a in enumerate(angles):
        x0 = (i + 0.15) * size / 8
        run = 8.0
        rise = run * math.tan(math.radians(a))
        w = size / 8 * 0.7
        z0 = size * 0.25
        l0, l1 = m.vert(x0, 0, z0), m.vert(x0 + w, 0, z0)
        h0, h1 = m.vert(x0, rise, z0 + run), m.vert(x0 + w, rise, z0 + run)
        m.quad(l0, h0, h1, l1)
        # a landing on top of each ramp
        m.box(x0, rise - 0.2, z0 + run, x0 + w, rise, z0 + run + 6)
    return m


def level_stairs(size, seed):
    m = Mesh()
    m.grid(0, 0, size, size / 8, lambda x, z: 0.0)
    risers = [0.1, 0.2, 0.3, 0.45, 0.6, 0.9, 1.2, 1.5]  # around the 0.9 default climb
    for i, rh in enumerate(risers):
        x0 = (i + 0.15) * size / 8
        w = size / 8 * 0.7
        for s in range(12):
            z0 = size * 0.2 + s * 0.4
            m.box(x0, 0.0, z0, x0 + w, rh * (s + 1), z0 + 0.4)
    return m


def level_overlapping(size, seed):
    m = Mesh()
    floors = 3
    gap = 4.0  # floor-to-floor, clears a 2.0 agent with room
    for f in range(floors):
        m.box(0, f * gap - 0.3, 0, size, f * gap, size)
        # ramp up to the next floor along alternating edges
        if f < floors - 1:
            run = gap / math.tan(math.radians(25))
            x0, w = (size * 0.1 if f % 2 == 0 else size * 0.6), size * 0.25
            z0 = size * 0.1
            a0, a1 = m.vert(x0, f * gap, z0), m.vert(x0 + w, f * gap, z0)
            b0, b1 = m.vert(x0, (f + 1) * gap, z0 + run), m.vert(x0 + w, (f + 1) * gap, z0 + run)
            m.quad(a0, b0, b1, a1)
    return m


def level_slivers(size, seed):
    m = Mesh()
    r = Lcg(seed)
    n = 64
    # A plane built from long thin triangles with T-junction-free but badly shaped fans, plus tiny and degenerate triangles.
    for i in range(n):
        x0, x1 = i * size / n, (i + 1) * size / n
        a, b = m.vert(x0, 0, 0), m.vert(x1, 0, 0)
        c, d = m.vert(x0, 0, size), m.vert(x1, 0, size)
        m.tri(a, c, d)
        m.tri(a, d, b)
    for _ in range(2000):
        x, z = r.next() * size, r.next() * size
        e = 1e-4 + r.next() * 1e-3
        a, b, c = m.vert(x, 0.001, z), m.vert(x + e, 0.001, z), m.vert(x + 2 * e, 0.001, z)  # collinear: zero area
        m.tri(a, c, b)
        d, e2, f2 = m.vert(x, 0.002, z), m.vert(x + 0.5, 0.002, z + e), m.vert(x + 1.0, 0.002, z)  # needle
        m.tri(d, f2, e2)
    return m


def level_city(size, seed):
    m = Mesh()
    f = smooth_noise(seed + 1)
    m.grid(0, 0, size, 2.0, lambda x, z: f(x, z) * 1.2)
    r = Lcg(seed)
    blocks = int(size / 12)
    for _ in range(blocks * 2):
        x, z = r.next() * (size - 14), r.next() * (size - 14)
        w, d, h = 4 + r.next() * 9, 4 + r.next() * 9, 3 + r.next() * 10
        m.box(x, -0.5, z, x + w, h, z + d)
    return m


TILEMAP_CELL = 2.0     # world units per tile
TILEMAP_SOLID = 0.6    # fraction of tiles that are floor


def tilemap_cells(size, seed):
    """Row-major (z, then x) solidity of the noisy tile map; bench/Shared/TileMapWorld.cs repeats this exactly."""
    n = int(size / TILEMAP_CELL)
    r = Lcg(seed * 1000003 + 77)
    return n, [[r.next() < TILEMAP_SOLID for _ in range(n)] for _ in range(n)]


def level_tilemap(size, seed):
    """A square of size/2 by size/2 tiles (2 units each), about 60% flat floor in noise: the footprint of a tile map
    extruded to flat triangles. Every grid vertex is written so the bounds are the whole square."""
    m = Mesh()
    n, solid = tilemap_cells(size, seed)
    idx = {}
    for i in range(n + 1):
        for j in range(n + 1):
            idx[(i, j)] = m.vert(i * TILEMAP_CELL, 0.0, j * TILEMAP_CELL)
    for j in range(n):
        for i in range(n):
            if solid[j][i]:
                a, b, c, d = idx[(i, j)], idx[(i + 1, j)], idx[(i + 1, j + 1)], idx[(i, j + 1)]
                m.tri(a, d, c)
                m.tri(a, c, b)
    return m


TILEMAP_CLEARANCE = 1.5  # query points of the tile-map "core" set keep this far from every wall


def tilemap_core_points(size, seed, count):
    """Query points on tile-map floor at least TILEMAP_CLEARANCE from any wall or the map edge, so every builder's mesh covers
    them at agent radii up to 1 and both sides of a query row can use the identical pair set."""
    n, floor = tilemap_cells(size, seed)
    m = level_tilemap(size, seed)
    cand = m.sample_points(Lcg(seed * 1000003 + 991), count * 8)
    out = []
    for x, y, z in cand:
        ok = True
        for j in range(max(0, int((z - TILEMAP_CLEARANCE) // TILEMAP_CELL) - 1), min(n, int((z + TILEMAP_CLEARANCE) // TILEMAP_CELL) + 2)):
            for i in range(max(0, int((x - TILEMAP_CLEARANCE) // TILEMAP_CELL) - 1), min(n, int((x + TILEMAP_CLEARANCE) // TILEMAP_CELL) + 2)):
                if floor[j][i]:
                    continue
                dx = max(i * TILEMAP_CELL - x, 0.0, x - (i + 1) * TILEMAP_CELL)
                dz = max(j * TILEMAP_CELL - z, 0.0, z - (j + 1) * TILEMAP_CELL)
                if dx * dx + dz * dz < TILEMAP_CLEARANCE ** 2:
                    ok = False
        if ok and x >= TILEMAP_CLEARANCE and z >= TILEMAP_CLEARANCE and x <= n * TILEMAP_CELL - TILEMAP_CLEARANCE and z <= n * TILEMAP_CELL - TILEMAP_CLEARANCE:
            out.append((x, y, z))
        if len(out) == count:
            break
    return out


LEVELS = {
    "tilemap": level_tilemap,
    "terraces": level_terraces, "ramps": level_ramps, "stairs": level_stairs,
    "overlapping": level_overlapping, "slivers": level_slivers, "city": level_city,
}


def main(argv):
    out = None
    size, seed, npts = 128.0, 1, 2000
    names = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a == "--size":
            size = float(argv[i + 1]); i += 2
        elif a == "--seed":
            seed = int(argv[i + 1]); i += 2
        elif a == "--points":
            npts = int(argv[i + 1]); i += 2
        elif out is None:
            out = a; i += 1
        else:
            names.append(a); i += 1
    if out is None:
        print(__doc__)
        return 2
    os.makedirs(out, exist_ok=True)
    for name in names or list(LEVELS):
        m = LEVELS[name](size, seed)
        pts = m.sample_points(Lcg(seed * 1000003 + len(name)), npts)
        m.write(os.path.join(out, name), pts)
        print("%s: %d verts, %d tris, %d points" % (name, len(m.v), len(m.t), len(pts)))
        if name == "tilemap":
            core = tilemap_core_points(size, seed, npts)
            with open(os.path.join(out, "tilemap-core.points"), "w", newline="\n") as f:
                f.write("# x y z\n")
                for x, y, z in core:
                    f.write("%.5f %.5f %.5f\n" % (x, y, z))
            print("tilemap-core: %d points (clearance %.1f)" % (len(core), TILEMAP_CLEARANCE))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
