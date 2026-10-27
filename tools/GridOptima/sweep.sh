#!/usr/bin/env bash
# Checks A* costs against the published optimal lengths of Moving AI Lab scenario sets.
#   tools/GridOptima/sweep.sh [--lock] [set ...]      (default sets: all nine 2D sets)
# Environment: MAPS (folder with <set>/*.map and *.map.scen, or <set>-map.zip / <set>-scen.zip; default ~/onepath/maps),
#              THREADS (default 6).
# The street and maze sets need well over 30 minutes of CPU: on a shared host pass --lock, which holds the host's
# benchmark lock file (~/WORKER-BENCH.lock) for the whole run and releases it on exit.
source "$(dirname "${BASH_SOURCE[0]}")/../../ci/env.sh"
if [ "${1:-}" = "--lock" ]; then
  shift
  set -o noclobber
  echo "onepath grid optima sweep $(date -Is) threads ${THREADS:-6}" > "$HOME/WORKER-BENCH.lock" || { echo "benchmark lock is held" >&2; exit 3; }
  trap 'rm -f "$HOME/WORKER-BENCH.lock"' EXIT
  set +o noclobber
fi
sets=("$@")
[ ${#sets[@]} -gt 0 ] || sets=(dao da2 bg512 sc1 wc3maps512 street maze random room)
dotnet build tools/GridOptima/GridOptima.csproj -c Release -v quiet -clp:NoSummary
out="$(dotnet msbuild tools/GridOptima/GridOptima.csproj -p:Configuration=Release -getProperty:OutDir | tr -d '\r')"
status=0
for s in "${sets[@]}"; do
  /usr/bin/time -f "$s wall %es cpu %Us rss %MKB" dotnet "${out}GridOptima.dll" --maps "${MAPS:-$HOME/onepath/maps}" --set "$s" --threads "${THREADS:-6}" || status=1
done
exit $status
