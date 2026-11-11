# Recording and replaying a crowd

A crowd run can be recorded and played back bit for bit: the host calls `CrowdRecorder` instead of the crowd's own methods, and
whatever it records can be saved as a **replay** (`CrowdReplay`), played by `CrowdPlayer`, or streamed to a viewer as it happens
(**live records**). The OneScope viewer's `RecursiveAI.OnePath.Scope` module reads exactly these bytes. All integers are
little-endian and doubles go by bit pattern, so the same run gives the same bytes on every platform.

## What is recorded

Everything a host does to a crowd between steps, in the order it did it, and the crowd's `StateHash()` after each step:

| Kind | Byte | Fields after the kind byte | Applies |
|---|---|---|---|
| Add | 1 | position (3 × f64), agent settings | `Crowd.TryAdd`; a failed add (full, no walkable polygon) does the same on replay |
| Remove | 2 | agent id (i64) | `Crowd.Remove` |
| SetDestination | 3 | agent id (i64), destination (3 × f64) | `Agent.SetDestination` |
| Stop | 4 | agent id (i64) | `Agent.Stop` |
| Teleport | 5 | agent id (i64), position (3 × f64) | `Agent.Teleport` |

A position is finite with every axis below 2^21 in magnitude. An agent id names an agent in the crowd at that moment; an operation
naming another is a `CrowdReplayException`. Agent settings are the bytes the crowd state uses for them (`docs/crowds.md`, "State bytes",
the row "agent options"), including the optional path-follower settings. A step's **operations** are these records
concatenated, at most 16 MiB; a step with none is empty. A step is: apply its operations in order, then `Crowd.Step(dt)`.

Not recorded, so a recording must not depend on them: edits to the mesh (the recorder's `Step` throws if the mesh changed), the
crowd's query filter (a delegate; the recorded crowd runs with none), and `MaxThreads` (results do not depend on it). Changes made
to the crowd without going through the recorder make the recording differ from the run; the replay's own hashes show where.

## Setup block

Everything needed to rebuild the crowd at its first recorded step:

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `OPRS` |
| 4 | 1 | version, 1 |
| 5 | 1 | kind, 1 (crowd) |
| 6 | 2 | reserved, 0 |
| 8 | 8 | time step `dt` (f64), above 0 and at most 3600 |
| 16 | 8 | start step (i64), the crowd's `StepCount` at the first recorded step; 0 or more |
| 24 | 4 | `MaxAgents` (i32) |
| 28 | 4 | `QueryNodes` (i32) |
| 32 | 1 | `QueryNodeModel` |
| 33 | 4 | `MaxWallsPerAgent` (i32) |
| 37 | 8 | `WallRefreshDistance` (f64) |
| 45 | 4 | `MaxWallsAvoided` (i32) |
| 49 | 4 | mesh length *m* (i32) |
| 53 | *m* | the mesh: `NavMeshSnapshot.Take(mesh, SnapshotMode.Full)` |
| 53 + *m* | 4 | crowd state length *s* (i32) |
| 57 + *m* | *s* | the crowd: `Crowd.SaveState()` |

The setup ends exactly there. The crowd options are validated as `CrowdOptions.Validate` does, and the restored crowd's step count
must equal the start step. `CrowdRecorder.WriteSnapshot` writes the same block for the crowd as it is now, which is how a live
viewer starts mid-run.

## Replay file

| Offset | Size | Field |
|---|---|---|
| 0 | 4 | magic `OPRP` |
| 4 | 1 | version, 1 |
| 5 | 1 | kind, 1 (crowd) |
| 6 | 2 | reserved, 0 |
| 8 | 4 | setup length (i32) |
| 12 | | the setup block |
| | 4 | step count *n* (i32), at most 2^26 and no more than the remaining bytes allow (each step takes at least 12) |
| | | *n* steps: operations length (i32), the operations, the crowd's `StateHash()` after the step (u64) |

The file ends after the last step. Reading it checks every length against the bytes present before allocating, so a damaged file
is a `CrowdReplayException` that names what was wrong and where. Reading then writing gives the same bytes. `CrowdReplay.FindMismatch`
plays the whole replay and returns the first step whose hash differs from the recorded one.

## Live records

`CrowdRecorder.Record` is called after each recorded change with one record, as the viewer's probe wants it:

| First byte | Rest | Meaning |
|---|---|---|
| 1 | one operation (as in the table above) | the host did this between steps |
| 2 | the crowd's hash after the step (u64) | the host stepped |

The operations since the last step record are the next step's operations; so a replay step is the concatenation of the kind-1 records
before a kind-2 record. A viewer that attaches takes `WriteSnapshot` (the setup block) at a moment when the crowd is exactly as it is
after the last record it was sent.

## Navigation-mesh build input

The viewer can step a mesh build along its stages. The host writes the input geometry and settings (`BuildInput.Write` in the Scope
package); the viewer runs the library's public stage functions on the whole input as one tile, in the order `Mesh3DPipeline` runs them
per tile: rasterize (`Voxelizer`), the three span filters (`SpanFilters`), compaction and erosion, region partition (`Regions`), contour trace
(`Contours`), polygonize (`Polygonizer`) and detail (`DetailMeshBuilder`), with the settings `Mesh3DPipeline` derives. File layout, version 1:
magic `OPBI`, version 1 and three zero bytes; ten f64: agent radius, agent height, walkable climb, max slope (degrees), cell size, cell
height, contour max error, max edge length, detail sample distance, detail max error; `MinRegionArea`, `MergeRegionArea` and
`MaxVertsPerPoly` (i32 each); partition (i32); vertex count (i32, at most 4,000,000) and triangle count (i32, at most 8,000,000); the
vertices (3 × f64 each, finite and below 2^21 in magnitude); the triangle indices (3 × i32 each, below the vertex count). The file ends
there. A grid of more than 4,000,000 columns is refused.
