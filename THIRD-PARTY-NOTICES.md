# Third-party notices

No third-party code, maps or data are included in this repository or its package.

## Benchmark inputs and references (used by `bench/` and `tools/`, never redistributed)

- **Moving AI Lab benchmarks** (2D grid maps and scenarios, 3D voxel sets): https://movingai.com/benchmarks/.
  "All data is made available under the Open Data Commons Attribution License" (ODC-By 1.0). Some maps are
  game-derived and distributed for research use with the publishers' permission, so they are fetched at pinned
  SHA-256 by `tools/maps/maps.sh` and never committed. Citation: N. Sturtevant, *Benchmarks for Grid-Based
  Pathfinding*, IEEE Transactions on Computational Intelligence and AI in Games, 4(2):144–148, 2012. Any published
  result computed from these maps cites this paper and the Moving AI Lab.
- **Reference implementations**, fetched from their public releases and run only as comparison baselines, as
  separate processes or bench-only package references; none ships in the library package:
  Recast Navigation 1.6.0 (zlib licence), RVO2 2.0.2 and RVO2-CS 2.0.1 (Apache-2.0), DotRecast 2026.3.1 (zlib licence),
  Warthog core and JPS 0.6.2 (MIT), CMake 3.31.8 (BSD 3-Clause; build tool only).
