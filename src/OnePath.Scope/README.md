# RecursiveAI.OnePath.Scope

The OnePath module for the OneScope viewer. It opens OnePath **crowd replays** and **navigation-mesh build inputs** in 2D or 3D, lets the viewer play, seek,
fork, poke and diff them, and attaches to a crowd that a running program records. It is built against `RecursiveAI.OneScope.Sdk`
(preview), is not part of `OnePath.slnx`, and is not built by CI until the SDK is on nuget.org.

Build and test from the repository root with the SDK package in a local folder:

```
ONESCOPE_SDK_FEED=<folder holding RecursiveAI.OneScope.Sdk.*.nupkg> dotnet test tests/OnePath.Scope.Tests
```

## Host side

```csharp
var crowd = mesh.CreateCrowd();
var rec = new CrowdRecorder(crowd, 1.0 / 30.0, keepSteps: true);   // or keepSteps: false for live only
var agent = rec.Add(start);            // use the recorder for everything that changes the crowd
rec.SetDestination(agent, goal);
rec.Step();                            // same as crowd.Step(dt), recorded with the crowd's hash
File.WriteAllBytes("run.oprp", rec.ToReplay().ToBytes());
```

For a live attach, pass `rec.WriteSnapshot` to the OneScope probe as its snapshot source and set `rec.Record` to the probe's record call, with
`ScopeInfo.ModuleId`, `ScopeInfo.LibraryVersion` and `ScopeInfo.FormatVersion` in the probe's handshake. A mesh build input is written with
`BuildInput.Write(vertices, indices, settings, writer)`. The byte layouts are in `docs/replay.md`.

## What the viewer shows

Crowd sources (`onepath`, 2D when the mesh is flat, 3D otherwise; metres):

| Overlay | Draws |
|---|---|
| Navigation mesh (on) | every polygon filled, its outline on top |
| Agents (on) | a circle (2D) or cylinder (3D) of the agent's radius per agent, coloured by state (moving, arrived, idle, stuck, no path, off-mesh link), picks the agent |
| Paths (on) | the agent's remaining path: its position then each corner of the corridor |
| Targets (on) | a point at the destination of every agent that has a path |
| Velocities (off) | an arrow from each agent along its velocity |
| Agent ids (off) | the agent's id above it |
| Off-mesh links (off) | a line per off-mesh link |

Entities: the crowd, its agents (inspector: state, position, velocity, destination, remaining distance, settings), the mesh and its polygons
(inspector: tile slot, index, area, flags, vertex count, centre, links, agents standing on it; the first 4096 polygons are listed). Tracks: agents
joining and leaving, orders, and series for the agent count, agents moving and mean speed. Pokes on a fork: `set_target` (the drag poke),
`add_agent`, `remove_agent`, `stop_agent`, `teleport_agent`. Diff names the first differing agent and field (`present`, `position`, `velocity`,
`destination`, `state`, `remaining distance`), and falls back to the state hash for anything else.

Build sources have one empty time step and a `stage` axis of seven positions: voxels, filtered spans, compact spans, regions, contours, polygons, detail.
Overlays: input triangles (on), the stage output at the axis position (on), and the finished mesh from `NavMesh.Build` (off). The inspector shows each
stage's hash, counts and a summary. A stage's hash is FNV-1a over what the library's stage function returned, so it equals calling that function directly.
