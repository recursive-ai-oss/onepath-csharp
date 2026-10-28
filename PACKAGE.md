# RecursiveAI.OnePath

Pathfinding and navigation for .NET, written from first principles: grids, graphs, navigation meshes and crowds in
one package. Results are bit-identical on every platform, so it is safe inside a lockstep or rollback game simulation.

**Preview.** The API can still change between previews.

- **Grids and graphs:** square (4- and 8-connected), hex and 3D voxel grids; A\*, JPS, JPS+, Theta\*, Lazy Theta\*, HPA\*,
  LPA\* and D\* Lite replanning, flow fields, area costs, agent sizes; weighted and point graphs with bidirectional
  search and ALT landmarks.
- **Navigation meshes:** an exact 2D builder (outlines, tile maps) and a Recast-class 3D builder (triangle soup, glTF,
  voxel chunks) into one tiled format, with path, string-pulling, raycast, nearest-point and random-point queries, run-time
  tile rebuild and obstacles, off-mesh links, several agent sizes and streaming.
- **Crowds:** path corridors and navmesh-aware ORCA avoidance, fixed-step, with byte-exact snapshot and restore.
- **Determinism:** only IEEE-exact `+ - * /` and `sqrt`, our own transcendental functions
  ([`RecursiveAI.OneMath`](https://www.nuget.org/packages/RecursiveAI.OneMath), the only dependency), fixed reduction
  orders and index-ordered parallel work. Time slicing counts iterations, never clock time.
- **Platforms:** `net10.0` on Windows, Linux and macOS (x64 and arm64), NativeAOT and trimming (an app that only uses
  grids trims the navmesh builders away), browser WASM.

```csharp
using RecursiveAI.OneMath;
using RecursiveAI.OnePath;

var grid = Grid.FromText("""
    ........
    .#####..
    .#...#..
    .#.#.#..
    ...#....
    """);
GridPath path = grid.FindPath(new Cell(0, 0), new Cell(4, 3));

NavMesh level = NavMesh.Build(triangles, NavSettings.Humanoid);
PathResult route = level.FindPath(new Vector3D(3, 0, 3), new Vector3D(36, 0, 36));

Crowd crowd = level.CreateCrowd();
Agent walker = crowd.Add(new Vector3D(3, 0, 3), Agent.Humanoid);
walker.SetDestination(new Vector3D(36, 0, 36));
crowd.Step(0.05);   // fixed time step; the crowd has no clock of its own
```

More examples, the documentation index ([docs](https://github.com/recursive-ai-oss/onepath-csharp/blob/main/docs/README.md)), build instructions and the speed table against the native references:
<https://github.com/recursive-ai-oss/onepath-csharp/blob/main/README.md>. Release notes:
<https://github.com/recursive-ai-oss/onepath-csharp/blob/main/CHANGELOG.md>. Third-party notices:
<https://github.com/recursive-ai-oss/onepath-csharp/blob/main/THIRD-PARTY-NOTICES.md>.

Licensed under either of MIT or Apache-2.0 at your option. © Recursive AI, LLC.
