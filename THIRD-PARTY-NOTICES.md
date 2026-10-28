# Third-party notices

No third-party code, maps or data are included in this repository or its package.

## Benchmark inputs and references (used by `bench/` and `tools/`, never redistributed)

- **Moving AI Lab benchmarks** (2D grid maps and scenarios, 3D voxel sets): https://movingai.com/benchmarks/.
  "All data is made available under the Open Data Commons Attribution License" (ODC-By 1.0). Some maps are
  game-derived and distributed for research use with the publishers' permission, so they are fetched at pinned
  SHA-256 by `tools/maps/maps.sh` and never committed. Citation: N. Sturtevant, *Benchmarks for Grid-Based
  Pathfinding*, IEEE Transactions on Computational Intelligence and AI in Games, 4(2):144–148, 2012. Any published
  result computed from these maps cites this paper and the Moving AI Lab.
- **Moving AI Lab 3D voxel maps and scenarios** (the Warframe sets, https://movingai.com/benchmarks/voxels.html), used by the
  voxel-grid checks and benchmarks under the same terms ("Open Data Commons Attribution License", ODC-By 1.0). The maps
  come from Warframe, and credit goes to Digital Extremes. Fetched at pinned SHA-256 by `tools/maps/maps.sh`, never
  committed. Citation: C. Brewer and N. Sturtevant, *Benchmarks for Pathfinding in 3D Voxel Space*, Symposium on
  Combinatorial Search (SoCS), 2018.
- **9th DIMACS Implementation Challenge road graphs** (USA road networks derived from US Census TIGER/Line data;
  https://users.diag.uniroma1.it/challenge9/): the challenge page states that, unless otherwise stated, the files
  "contain data/software in the public domain and can be freely downloaded". Fetched by `tools/GraphRoad/fetch.sh`
  (SHA-256 checked against `tools/GraphRoad/manifest.tsv` once pinned), never committed. Citation: C. Demetrescu,
  A. V. Goldberg and D. S. Johnson (eds.), *The Shortest Path Problem: Ninth DIMACS Implementation Challenge*, AMS, 2009.
- **Reference implementations**, fetched from their public releases and run only as comparison baselines, as
  separate processes or bench-only package references; none ships in the library package:
  Recast Navigation 1.6.0 (zlib licence), RVO2 2.0.2 and RVO2-CS 2.0.1 (Apache-2.0), DotRecast 2026.3.1 (zlib licence),
  Warthog core and JPS 0.6.2 (MIT), CMake 3.31.8 (BSD 3-Clause; build tool only).
