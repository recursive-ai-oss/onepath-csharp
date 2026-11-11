# Changelog of RecursiveAI.OnePath.Scope

The viewer module is released on its own, with its own version and tag (`scope-v<version>`), separately from the library package in the
repository's top-level `CHANGELOG.md`. Each release has a `## [<version>]` heading; the text under it becomes the release notes.

## [Unreleased]

## [1.0.0]

The first stable release, on `RecursiveAI.OneScope.Sdk` 1.0 and `RecursiveAI.OnePath` 1.2.0 (the package names 1.2.0 as its floor).

### Added

- **Crowd replays** (`.oprp`, the library's `CrowdReplay` bytes) open in 2D or 3D: the navigation mesh, agents coloured by state, paths, targets, velocities, agent ids and off-mesh links as overlays; agents and mesh polygons as entities with inspector rows; tracks for joins, leaves, orders and series for agent count, agents moving and mean speed. Seeking lands on the library's own per-step hashes at any keyframe interval.
- **Live attach** to a crowd that a program records with `CrowdRecorder`: the recorder's snapshot and records feed the viewer's probe, and the viewer re-simulates locally.
- **Fork and poke** on a crowd: `set_target` (the drag poke), `add_agent`, `remove_agent`, `stop_agent`, `teleport_agent`; diff names the first differing agent and field.
- **Navigation-mesh build inputs** (`.opbi`) with a `stage` axis of seven positions (voxels, filtered spans, compact spans, regions, contours, polygons, detail), each stage's hash and counts in the inspector, and the finished mesh as an overlay.
- **Grid runs** (`.opgr`): A\*, JPS, JPS+ searches and flow fields stepped by queue pops through an `iteration` axis, with the open and closed sets, path, start and goal, flow costs and directions as overlays; pokes `move_goal` (the drag poke), `move_start`, `set_wall`, `set_area`; the finding `flow.goal-blocked`.

### Changed

- Built against `RecursiveAI.OneScope.Sdk` 1.0.0 instead of its 0.3 previews; draw styles use the full constructor.
