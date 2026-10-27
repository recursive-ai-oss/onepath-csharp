# Hash matrix

Results must be bit-identical on every platform, runtime mode and thread count. The hash matrix
checks that: a runner executes registered *scenarios* (deterministic functions of the library),
hashes the bytes each one writes, and prints a manifest. Equal manifests across *columns* mean
equal behaviour. The manifest format is the same as the one used by the family's math library, so one
comparison script serves both.

## Manifest format

Text, `\n` line ends, UTF-8:

```
<scenario name> <sha256 hex, lower case>      one line per scenario, ordinal name order
manifest <sha256 hex>                          SHA-256 of all the lines above, each with its '\n'
```

A scenario's hash is the SHA-256 of the bytes it wrote through `ScenarioContext`: integers
little-endian, floats and doubles by bit pattern (so NaN payloads and signed zeros count). SHA-256
is implemented inside the runner, so it behaves the same on every runtime including browser-wasm.

## Columns

`tools/hash-matrix.sh run <outdir> [column ...]` builds and runs each column and writes
`<outdir>/<tag>-<column>.manifest`; the tag is `$HASHMATRIX_TAG` or `<os>-<arch>`. With no columns
it runs every column that applies to the machine.

| Column | What differs |
|---|---|
| `jit` | framework-dependent, default JIT, 1 thread |
| `jit-threads` | same, `--threads 0` (all cores) |
| `jit-scalar` | `DOTNET_EnableHWIntrinsic=0`: no SIMD instructions |
| `jit-noavx2` | `DOTNET_EnableAVX2=0` (x64 only; skipped elsewhere) |
| `jit-fulljit` | `DOTNET_TieredCompilation=0`: no tier-0 code |
| `aot` | NativeAOT for the current OS and architecture |
| `wasm` | browser-wasm, interpreted, run headless under Node |
| `wasm-aot` | browser-wasm, AOT-compiled (`RunAOTCompilation`), under Node |

The `osx-arm64` and other-OS columns are the same script run on that machine; copy the manifests
together and compare. `tools/hash-matrix.sh compare <manifest ...>` exits 0 when all files are
identical and otherwise prints the differing lines; `all <outdir> [column ...]` runs, then compares.
Paths may be relative to where you call the script.

Requirements: the .NET 10 SDK (`ONEPATH_DOTNET_ROOT` selects a user-local one); the `wasm-tools`
workload for the wasm columns (installed automatically when missing); Node (from `PATH`, otherwise
the copy the workload ships); a C toolchain for NativeAOT.

`--threads N` (0 = all cores, default 1) reaches every scenario as `ctx.Threads`. A scenario may use
it to split work, but must split by index and combine in index order: the bytes must not depend on it.

## Adding a scenario

Create one file under `tools/HashMatrix/Scenarios/<area>/` with a module initializer; no shared file
changes. Module initializers need no reflection, so this works under NativeAOT, trimming and wasm.

```csharp
using System.Runtime.CompilerServices;

namespace RecursiveAI.OnePath.HashMatrix;

internal static class AStarScenarios
{
    [ModuleInitializer]
    internal static void Init() =>
        Scenarios.Register("grids/astar/octile-random", ctx =>
        {
            // build inputs from a fixed seed, run the part, write every result:
            ctx.Write(cost);        // int, long, float, double, or ctx.Write(ReadOnlySpan<byte>)
        });
}
```

Names are lower-case, slash-separated and unique. Seed inputs from the library's own generator, never
from time or `System.Random`. Write results, not timings. Write both sides of a SIMD/scalar pair so
a mismatch shows.

`--filter PREFIX` runs a subset and `--list` prints names. A changed manifest line is either an
intended change (say so in the commit) or a bug.

## Cost

With the placeholder scenarios a full local run of every column takes about a minute. Once real
scenarios exist, the full matrix is a heavy job: run it on the build worker. A focused
`--filter` run of a single column is light.

## Placeholder scenarios

`meta/bits` (bit patterns), `meta/float-chunks` (an IEEE-exact float computation split over 64
chunks, run on `ctx.Threads` workers) and `meta/simd-vs-scalar` (Vector128 against its scalar twin).
They stay as runner self-checks.
