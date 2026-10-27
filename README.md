# onepath

Pathfinding and navigation for .NET, written from first principles: grids, graphs, navmeshes and
crowds in one package, `RecursiveAI.OnePath`. Results are bit-identical on every platform.

**Status:** repository skeleton. The package builds and ships a version type only; the algorithms
arrive part by part.

Licensed under either of MIT ([LICENSE-MIT](LICENSE-MIT)) or Apache-2.0
([LICENSE-APACHE](LICENSE-APACHE)) at your option. © Recursive AI, LLC.

## Layout

| Path | What |
|---|---|
| `src/OnePath/` | the package (`RecursiveAI.OnePath`, target `net10.0`) |
| `tests/OnePath.Tests/` | unit tests (xUnit v3) |
| `tests/AotSmoke/` | NativeAOT + full-trim smoke app |
| `tests/WasmSmoke/` | browser-wasm smoke app, run headless under Node |
| `bench/Bar/` | the bar runner: time, memory and quality rows as CSV |
| `bench/Micro/` | BenchmarkDotNet micro-benchmarks |
| `fuzz/OnePath.Fuzz/` | SharpFuzz / libFuzzer targets |
| `ci/` | the build, test and smoke scripts CI runs; run them locally the same way |

## Building

Requires the .NET 10 SDK (`global.json` pins 10.0.100 and rolls forward to later 10.0 feature bands).

```sh
ci/build.sh            # Release and Checked builds of the core projects
ci/test.sh             # unit tests, Release and Checked
ci/aot-smoke.sh        # NativeAOT + trimmed publish of the smoke app, then runs it
ci/wasm-smoke.sh       # browser-wasm build, run headless under Node (installs the wasm-tools workload if missing)
```

Each script takes configurations as arguments (`ci/test.sh Debug`). `dotnet build OnePath.slnx -c Release`
builds everything, including the WASM smoke app, which needs the `wasm-tools` workload.

- **Configurations:** `Debug`, `Release`, and `Checked` (optimized, defines `CHECKED`; the internal
  unchecked-access helpers become bounds-checked, and tests and fuzzing run in it).
- **Output location:** `bin/` and `obj/` never appear in the tree. They go to `$ONEPATH_ARTIFACTS`
  if set, else `<temp>/onepath-artifacts/<checkout folder name>`.
- **Package ID:** `$(PackagePrefix).OnePath`, with `PackagePrefix` defined once in `Directory.Build.props`.
- **Warnings are errors**, analyzers run at `latest-recommended`, and `BannedSymbols.txt` forbids in
  `src/` the calls that could make results machine-dependent (platform maths functions, fused
  multiply-add, `Vector<T>`, clocks, `Random`, processor count).
- **Math package:** `RecursiveAI.OneMath` will be referenced by `src/OnePath`. Until it is published,
  set `ONEMATH_FEED` to a folder containing its package and restore picks it up.
- `ONEPATH_DOTNET_ROOT`: scripts put this user-local SDK folder first on `PATH`.
