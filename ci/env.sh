# Sourced by the other ci scripts. Finds the repo root and the .NET SDK, and sets sane defaults.
#   ONEPATH_DOTNET_ROOT  optional: folder of a user-local .NET SDK to put first on PATH
#   ONEPATH_ARTIFACTS    optional: where bin/obj go (see Directory.Build.props)
#   ONEMATH_FEED         optional: folder holding the RecursiveAI.OneMath package (once it is referenced)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_ROOT

if [ -n "${ONEPATH_DOTNET_ROOT:-}" ]; then
  export DOTNET_ROOT="$ONEPATH_DOTNET_ROOT"
  export PATH="$ONEPATH_DOTNET_ROOT:$PATH"
fi
export DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1 DOTNET_SKIP_FIRST_TIME_EXPERIENCE=1 MSBUILDDISABLENODEREUSE=1

# Projects built and tested everywhere. The two smoke apps have their own scripts: they need
# NativeAOT tooling and the wasm-tools workload, which not every machine has.
CORE_PROJECTS=(
  src/OnePath/OnePath.csproj
  tests/OnePath.Tests/OnePath.Tests.csproj
  fuzz/OnePath.Fuzz/OnePath.Fuzz.csproj
  bench/Bar/Bar.csproj
  bench/Micro/Micro.csproj
)

cd "$REPO_ROOT"
