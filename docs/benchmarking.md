# Benchmarking: method, row format and how to run

The bar compares our code with a reference on the same machine, in the same session, on the same inputs and
settings. This page is normative: a later port of the bar runner must produce comparable rows from it.

## Method

- **Warm, in-process, best of N.** Every driver loads its input, does untimed warm-up work, then times the measured
  work repeatedly inside one process and reports the best (minimum) time, plus the median. The bar runner repeats the
  whole process `--reps` times and keeps the best per row. Times are monotonic-clock wall times of the measured section
  only (no process start, no file loading), except where a metric says otherwise.
- **Same inputs and settings.** Navmesh builds read the same OBJ with the same settings (below); query sets are the same
  point file; crowds and grid scenarios are the same parameters. Settings for the builders default to a humanoid:
  cell 0.3, cell height 0.2, agent height 2.0, radius 0.6, max climb 0.9, max slope 45 degrees, min region 8, merge region
  20, max edge length 12, max edge error 1.3, 6 vertices per polygon, detail sample distance 6, detail max error 1,
  64-cell tiles for tiled builds.
- **Memory** is the peak resident set of the whole process from `/usr/bin/time -v` (Linux) or `/usr/bin/time -l`
  (macOS), reported as `peak_rss` in MB. The managed runtime's own overhead is therefore part of a .NET row.
- **Machines.** x64 rows run on the Linux benchmark host and arm64 rows on a Mac, one heavy job at a time, with the
  host's benchmark lock held inside the job (`tools/bench/locked-run.sh`). Each run records machine, CPU, core count, 1-minute
  load before, between rows and after, SDK, runtime and reference versions in `env.json`. A run that overlapped another
  heavy job is not publishable.
- **The pass rule** (per row): time-like rows pass at ours ≤ 1.15 × reference; throughput rows at reference ≤ 1.15 × ours;
  grid rows against the fastest reference at 1.0. Navmesh quality: walkable coverage within 1% of Recast's on the
  same settings, with path lengths over the same query set reported. Tolerances live in the suite file, per row.

## Row format

Drivers print one measurement per line on stdout:

```
ROW,<part>,<scenario>,<side>,<metric>,<value>,<unit>
```

`part` is `build`, `query`, `crowd`, `grid` or `tilecache`; `side` names the implementation and version (`recast-1.6.0`,
`dotrecast-2026.3.1`, `rvo2-2.0.2`, `rvo2cs-2.0.1`, `warthog-0.6.2`, `warthog-jps-0.6.2`, `onepath-<version>`); `unit` is
`ms`, `count`, `MB`, `world_units`, `world_units2`, `1/s`, `agent-steps/ms`, `cost` or `text`. The bar runner replaces
`scenario` by the suite row id and writes everything, ours and reference, to `rows.csv` in long format:

```
run_id,suite,row_id,part,scenario,side,metric,value,unit
```

Best-of-N across process repetitions: `ms` metrics keep the minimum; throughput (`…/s`, `…/ms`) and `MB` the maximum;
the rest the last run's value.

### Metrics by part

| part | metrics |
|---|---|
| build | `time_best`, `time_median` (ms, whole build of all tiles from triangles to navmesh data), `polys`, `verts`, `coverage_area` (sum of polygon areas projected on the ground plane, world units²), `tiles`, `input_tris`, `peak_rss`, `settings`; our tile-map rows add `exact_area` and `delta_vs_exact_pct` (the 2D gate, below) |
| query | `points_snapped`, `pairs`; `path_time_best`, `path_per_sec`, `path_mean_length` (string-pulled path, world units), `path_found`, `path_partial`; `raycast_time_best`, `raycast_per_sec`, `raycast_hits`; `nearest_time_best`, `nearest_per_sec` |
| tilecache | `time_best`, `time_median` (ms, one whole obstacle cycle), `cache_build_time` (ms, the level built with its tile cache), `obstacles`, `polys_initial`, `polys` (all obstacles in force), `polys_restored`, `coverage_area_initial`, `coverage_area`, `coverage_area_restored`, `tiles`, `layers`, `input_tris`, `peak_rss`, `settings` |
| crowd | `agents`, `steps`, `time` (ms for the timed steps), `agents_per_ms` (agent-steps per ms), `moving_at_end` or `position_checksum`, `threads` where the library has them |
| grid | per scenario file: `instances`, `reference_exit_code` (only when non-zero: Warthog's `--checkopt` exits with its count of cost mismatches after printing every instance; the rows are kept), `search_time` (sum of per-instance search time, ms), `wall_time` (whole process, ms), `expanded`, `generated`, `cost_sum`, `cost_mismatch_vs_scenario` (instances whose cost differs from the scenario file's optimal cost) |

The query set pairs consecutive points of the level's `.points` file (point 0 with 1, 2 with 3, …); each point is
snapped to the nearest polygon within a (2, 4, 2) box and pairs with an unsnapped end are dropped. Path queries are
`findPath` followed by `findStraightPath`; an incomplete path (goal unreachable) is counted in `path_partial` and
measured to the closest reachable point. Raycasts go from the first to the second point of each pair.

The crowd scenarios: **navmesh crowd** — N agents start at the query points and walk to other query points
(humanoid parameters: radius 0.6, height 2.0, max speed 3.5, max acceleration 8, separation weight 2, all steering flags on),
fixed step 0.05 s, 50 warm-up steps then the timed steps. **Circle** — N agents evenly on a circle (radius
200·√(N/250) units) walk to the antipodal point, step 0.25 s, no obstacles, RVO2's example parameters (neighbour
distance 15, 10 neighbours, time horizons 10 and 5, radius 2, max speed 2).

## Suites and the bar runner

`bench/Bar` runs a **suite** (`bench/Bar/suites/*.tsv`): one tab-separated line per row,
`id part driver metric better tolerance args`. `driver` is `recast`, `rvo2`, `warthog` (A\*, Dijkstra), `jps`
(JPS, JPS+ and variants), `dotrecast` or `rvo2cs`. `args` is the driver's command line, with `{home}`, `{maps}`,
`{geom}` (levels, size 256) and `{geomsmall}` (size 96) expanded, and `env:NAME=VALUE` tokens setting an environment
variable for that run. Our side is whatever `OurScenarios.Run` returns for the row id; until a part registers its
scenarios the report shows "no ours yet" beside the reference's numbers.

```
tools/bench/run-bar.sh <suite|file.tsv> [--out dir] [--reps N] [--only substring] [--machine name] [--light] [--in-process]
```

writes `rows.csv`, `env.json` and `report.md` (per row: ours, reference, ratio, memory, verdict; then every reference
metric; for tile-map rows also the 2D gate table). Suites: `smoke` (tiny; proves every driver on a machine; no timing claims),
`references` (the full reference rows), `quality3d` and `shapes` (coverage and path-length rows, no timing).

Our side of a row runs in a fresh child process under `/usr/bin/time`, exactly like a reference driver, so its `peak_rss` is that
row's own peak and its timing starts from a cold process (the row's own `--reps` give the warm best). `--in-process` runs it inside the
runner instead (debugging only; memory is then the runner's peak so far).

**Light check.** `--light` proves that every row of a suite runs and prints every metric: levels of size 96 instead of 256, one repetition
of every row, 20 crowd steps and at most 200 agents. Label such a run `--machine light-check`; its numbers are never published.

## References

Built unattended from their public releases by `tools/refs/refs.sh` into `$ONEPATH_HOME/refs/<name>-<version>/`
(default `~/onepath`), pinned by URL and SHA-256 in `tools/refs/manifest.tsv`. They are benchmark-only black boxes: nothing
of theirs is stored in this repository, and their sources are never read; each is driven through its documented API or
command line.

| reference | version | driven through | notes |
|---|---|---|---|
| Recast, Detour, DetourCrowd | 1.6.0 | `bench/Harness/Native/recast_harness.cpp`, written from the public API reference | static libraries, CMake Release (`-O3`), no `-march` flags |
| RVO2 | 2.0.2 | `bench/Harness/Native/rvo_harness.cpp`, from the library documentation | parallel with OpenMP where the compiler has it; rows set `OMP_NUM_THREADS=1` for the single-thread comparison and report `threads` |
| DetourTileCache | 1.6.0 (same release) | the `tilecache` command of `recast_harness.cpp`, from the class reference at recastnav.com (`dtTileCache`, `dtTileCacheParams`, `dtTileCacheLayerHeader`, `dtTileCacheCompressor`, `dtTileCacheMeshProcess`, `DetourTileCacheBuilder.h` and `Recast.h` function pages) | layers are stored uncompressed (a pass-through compressor), so the rows measure the rebuild and not LZ compression |
| DotRecast | 2026.3.1 (NuGet) | `bench/Harness/RefHarness` (C#), compiled against the package | same settings as Recast; per-polygon areas set after the build as in the native driver; tiled builds hand the library the bounds of the whole input and the tile's grid position (it offsets the box itself) |
| DotRecast tile cache | 2026.3.1 (NuGet `DotRecast.Detour.TileCache`) | `bench/Harness/RefHarness/DotRecastTileCache.cs` | its own layer builder over the whole tile grid on one thread, the same pass-through storage and the same scenario as the native driver |
| RVO2-CS | 2.0.1 | `bench/Harness/RefHarness`, library sources compiled as shipped by `bench/Harness/Rvo2CsLib` | not on NuGet; its own projects target an old .NET Framework |
| Warthog (core, JPS) | 0.6.2 | the documented `warthog` and `warthog-jps` command lines on Moving AI scenario files; on macOS built with a private LLVM 22.1.5 toolchain (below) | `search_time` sums the per-instance times the program prints; JPS+ preprocessing is not in it, so `wall_time` is reported too; maps with swamp or water (the wc3 set) are given as a converted copy through the documented `--map` option (swamp as free ground, water as blocked, the reading of the published optimal costs), since the programs otherwise stop at the first instance crossing one |

**Warthog on macOS.** Warthog's logger calls `std::chrono::current_zone`, which Apple's C++ library does not have.
`refs.sh` therefore builds a private toolchain under `$ONEPATH_HOME/tools` on macOS and compiles Warthog, unmodified,
with it: the pinned LLVM 22.1.5 macOS release supplies clang, and the same release's runtimes source is built into a
private libc++ with its time-zone database enabled (`-DLIBCXX_ENABLE_TIME_ZONE_DATABASE=ON`), which Warthog links
through an rpath. One line of libc++'s own source needs a change, applied and checked by the script: its time-zone
code names the zoneinfo folder for Linux only, and macOS keeps the same database at the same path
(`/usr/share/zoneinfo`). The official LLVM macOS binaries ship libc++ with the database switched off, and the 21.x
clang does not parse the macOS 27 SDK's `<math.h>`, which is why 22.1.5 is pinned. No Homebrew or system change;
the toolchain adds about 1.5 GB of downloads and a few minutes of build to a first macOS setup. Warthog's search
itself does not use time zones; only its log timestamps do.

Known gaps: DotRecast's and Recast's `raycast_hits` differ on the same input (the
meshes differ slightly, and ray semantics at the end of a path are not documented); raycast rows compare ours with
Recast's.

## Inputs

- **Levels:** `tools/geom/gen_levels.py` writes terraces, ramps, stairs, overlapping floors, slivers, a city and a
  tile map (OBJ plus a point file) from a seed; `tools/bench/prepare.sh` generates the size-96 and size-256 sets under
  `$ONEPATH_HOME/geom` with a `SHA256SUMS` file. Nothing third-party.
- **Maps:** `tools/maps/maps.sh` fetches the Moving AI Lab 2D and voxel sets at pinned SHA-256 into
  `$ONEPATH_HOME/maps/<set>/` (about 665 MB unpacked). Never committed; see `THIRD-PARTY-NOTICES.md`.

## The 2D builder row

`tilemap` is a square of size/2 by size/2 tiles of 2 world units (128 x 128 at size 256), each tile floor with
probability 0.6 from a 64-bit linear congruential generator (`Lcg(seed * 1000003 + 77)`, row-major, the generator's
own `next() < 0.6`), extruded to two flat up-facing triangles per floor tile at height 0. The bounds are always the whole
square. The reference builds it from the triangles (`--cell 0.25 --agent-radius 0.5`, tiled rows with `--tile-size 128`
cells = 32 units, a solo row without); ours is `NavMesh.Build2D` from the tiles, which `bench/Shared/TileMapWorld.cs`
recovers from the OBJ (each triangle belongs to the tile its centroid is in; `OnePath.Micro --check-tilemap` checks
this against its own generator) and times from tiles to mesh (best of `--reps` builds in one process). Ours builds at the nominal agent radius.

**The gate.** The 2D builder is judged against the exact eroded area of the same tile map, not against Recast: the area of the floor
points at least the agent radius away from every wall (solid tiles and everything outside the map), with round joins at convex corners.
`TileMapWorld.ExactErodedArea` computes it in closed form without any builder. For a radius up to half a tile (2 units), wall tiles two or
more tiles away are farther than the radius from every point of a floor tile, so each floor tile only sees its eight neighbours; the lost
area inside a tile of side T is a strip of width r along each side that borders a wall plus a quarter disc of radius r at each corner whose
two adjacent sides border floor and whose diagonal neighbour is a wall, and these overlap only in r x r corner squares where two strips
meet: lost = 2 r S - r^2 C + (pi r^2 / 4) D for S wall sides, C wall-side pairs meeting at a corner and D discs. The only inexact step is
the double rounding of pi, so the result is within 1e-9 relative of the true area; it agrees with the point-sampling in
`tools/geom/gen_shapes.py` (`exact.tsv`, sampled at 1/200 unit) to 0.03% on the shape rows. Our rows print `exact_area` and
`delta_vs_exact_pct`, and the report's "2D gate" table lists, per row, the exact area, ours, ours against exact (pass within 1%, or within 0.02 world units² where the exact area is a sliver of a few cells and 1% is below a vertex's rounding), then
the reference's coverage, its delta against exact and ours against it, as information. Recast's coverage equals the exact area at
radius + one cell (its ledge filter and minimum region area), so it falls below the exact area by construction: on the 256 tile map at
radius 0.5 and cell 0.25, exact 23819, ours 23761 (-0.25%), Recast 16407 (-31%).

Quality rows to read beside the times: `coverage_area` (sum of polygon areas on the ground plane), `polys`, `verts`, and the query row
`query/tilemap-tiled/recast` (`path_mean_length` over the level's own points; each side snaps the points itself, so
the pairs it answers differ slightly).

## The run-time rebuild rows

`tilecache` rows compare temporary-obstacle rebuilds on a tiled level (`--mode tiled --tile-size 64`, cell 0.3): ours `TileCache`
(`Mesh3DPipeline.BuildSoupCached`, `AddObstacle`, `UpdateAll`) against DetourTileCache and DotRecast's tile cache. Obstacle k of
`--obstacles` (16) is a cylinder of radius 2 and height 3 whose base is half a unit under the k-th of every (points / obstacles) query
points. One timed cycle adds every obstacle in turn, each followed by rebuilding the tiles it touches until the cache is up to date, then
removes them the same way; `time_best` is the best cycle of `--reps`, on one cache built before the cycles (`cache_build_time` is the best
of `--reps` builds of the level with its cache, layers stored uncompressed). Quality rows: polygon count and walkable area before, with all
obstacles in force and after they are removed (`polys_restored` and `coverage_area_restored` must equal the initial values).

## Running the full reference rows

On the Linux benchmark host, once per checkout: `tools/refs/refs.sh all` and `tools/bench/prepare.sh`. Then, with the
checkout synced there:

```
tools/bench/locked-run.sh "bar references" 90 tools/bench/run-bar.sh references --reps 3 --out ~/onepath/bar/$(date +%Y%m%d) --machine <name>
```

The lock is taken inside the job and removed when it ends; the command fails at once if the lock is held. On an
Apple-silicon Mac the same commands apply, including the Warthog rows.
