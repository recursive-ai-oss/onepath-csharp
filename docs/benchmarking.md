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

`part` is `build`, `query`, `crowd` or `grid`; `side` names the implementation and version (`recast-1.6.0`,
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
| build | `time_best`, `time_median` (ms, whole build of all tiles from triangles to navmesh data), `polys`, `verts`, `coverage_area` (sum of polygon areas projected on the ground plane, world units²), `tiles`, `input_tris`, `peak_rss`, `settings` |
| query | `points_snapped`, `pairs`; `path_time_best`, `path_per_sec`, `path_mean_length` (string-pulled path, world units), `path_found`, `path_partial`; `raycast_time_best`, `raycast_per_sec`, `raycast_hits`; `nearest_time_best`, `nearest_per_sec` |
| crowd | `agents`, `steps`, `time` (ms for the timed steps), `agents_per_ms` (agent-steps per ms), `moving_at_end` or `position_checksum`, `threads` where the library has them |
| grid | per scenario file: `instances`, `search_time` (sum of per-instance search time, ms), `wall_time` (whole process, ms), `expanded`, `generated`, `cost_sum`, `cost_mismatch_vs_scenario` (instances whose cost differs from the scenario file's optimal cost) |

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
tools/bench/run-bar.sh <suite|file.tsv> [--out dir] [--reps N] [--only substring] [--machine name]
```

writes `rows.csv`, `env.json` and `report.md` (per row: ours, reference, ratio, memory, verdict; then every reference
metric). Suites: `smoke` (tiny; proves every driver on a machine; no timing claims) and `references` (the full reference
rows).

## References

Built unattended from their public releases by `tools/refs/refs.sh` into `$ONEPATH_HOME/refs/<name>-<version>/`
(default `~/onepath`), pinned by URL and SHA-256 in `tools/refs/manifest.tsv`. They are benchmark-only black boxes: nothing
of theirs is stored in this repository, and their sources are never read; each is driven through its documented API or
command line.

| reference | version | driven through | notes |
|---|---|---|---|
| Recast, Detour, DetourCrowd | 1.6.0 | `bench/Harness/Native/recast_harness.cpp`, written from the public API reference | static libraries, CMake Release (`-O3`), no `-march` flags |
| RVO2 | 2.0.2 | `bench/Harness/Native/rvo_harness.cpp`, from the library documentation | parallel with OpenMP where the compiler has it; rows set `OMP_NUM_THREADS=1` for the single-thread comparison and report `threads` |
| DotRecast | 2026.3.1 (NuGet) | `bench/Harness/RefHarness` (C#), compiled against the package | same settings as Recast; per-polygon areas set after the build as in the native driver |
| RVO2-CS | 2.0.1 | `bench/Harness/RefHarness`, library sources compiled as shipped by `bench/Harness/Rvo2CsLib` | not on NuGet; its own projects target an old .NET Framework |
| Warthog (core, JPS) | 0.6.2 | the documented `warthog` and `warthog-jps` command lines on Moving AI scenario files | `search_time` sums the per-instance times the program prints; JPS+ preprocessing is not in it, so `wall_time` is reported too |

Known gaps: Warthog does not build on macOS (Apple's C++ library lacks `std::chrono::current_zone`, which its logger
uses), so grid rows have no arm64 reference. DotRecast's and Recast's `raycast_hits` differ on the same input (the
meshes differ slightly, and ray semantics at the end of a path are not documented); raycast rows compare ours with
Recast's.

## Inputs

- **Levels:** `tools/geom/gen_levels.py` writes terraces, ramps, stairs, overlapping floors, slivers and a city (OBJ
  plus a point file) from a seed; `tools/bench/prepare.sh` generates the size-96 and size-256 sets under
  `$ONEPATH_HOME/geom` with a `SHA256SUMS` file. Nothing third-party.
- **Maps:** `tools/maps/maps.sh` fetches the Moving AI Lab 2D and voxel sets at pinned SHA-256 into
  `$ONEPATH_HOME/maps/<set>/` (about 665 MB unpacked). Never committed; see `THIRD-PARTY-NOTICES.md`.

## Running the full reference rows

On the Linux benchmark host, once per checkout: `tools/refs/refs.sh all` and `tools/bench/prepare.sh`. Then, with the
checkout synced there:

```
tools/bench/locked-run.sh "bar references" 90 tools/bench/run-bar.sh references --reps 3 --out ~/onepath/bar/$(date +%Y%m%d) --machine <name>
```

The lock is taken inside the job and removed when it ends; the command fails at once if the lock is held. On an
Apple-silicon Mac the same commands apply (grid rows will report "reference failed").
