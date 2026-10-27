#!/usr/bin/env bash
# Build the bar runner and run a suite, recording the results under <out>.
#   tools/bench/run-bar.sh <suite.tsv> [--out dir] [--reps N] [--only substring] [--machine name]
# The suite path may be a name in bench/Bar/suites (e.g. "smoke" or "references"). Results: rows.csv, env.json, report.md.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
H="${ONEPATH_HOME:-$HOME/onepath}"

# Prefer a user-local SDK: the worker's ~/dotnet10, a Mac's ~/onepath/dotnet, else whatever is on PATH.
for d in "$HOME/dotnet10" "$H/dotnet"; do
  if [ -x "$d/dotnet" ]; then export DOTNET_ROOT="$d" PATH="$d:$PATH"; break; fi
done
export DOTNET_CLI_TELEMETRY_OPTOUT=1 DOTNET_NOLOGO=1 DOTNET_SKIP_FIRST_TIME_EXPERIENCE=1 MSBUILDDISABLENODEREUSE=1
export ONEPATH_ARTIFACTS="${ONEPATH_ARTIFACTS:-$H/artifacts}"

suite="$1"; shift
[ -f "$suite" ] || suite="$REPO/bench/Bar/suites/$suite.tsv"
dotnet build "$REPO/bench/Bar/Bar.csproj" -c Release -v quiet -clp:NoSummary
exec dotnet "$ONEPATH_ARTIFACTS/bin/Bar/release/OnePath.Bar.dll" run --suite "$suite" "$@"
