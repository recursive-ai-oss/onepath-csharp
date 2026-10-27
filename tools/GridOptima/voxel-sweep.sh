#!/usr/bin/env bash
# Checks voxel A* costs against the published optimal lengths of the Moving AI Lab voxel (Warframe) scenario sets.
#   tools/GridOptima/voxel-sweep.sh [--lock] [map ...]      (default: every map of the set)
# Environment: MAPS (folder holding warframe/*.3dmap and *.3dmap.3dscen; fetch with tools/maps/maps.sh warframe; default ~/onepath/maps),
#              THREADS (default 2; each thread holds search buffers of 20 bytes per voxel of the map it works on: 10 GB for the largest map),
#              PER_MAP (scenarios per map; default all 10001), CONN (6, 18 or 26; default 26), RULE (cut, squeeze or all; default cut).
# A full sweep is about 440000 searches of around a second each: far over 30 minutes of CPU. With PER_MAP=10 it is a light check.
# On a shared host pass --lock for a full sweep: it holds the host's benchmark lock file (~/WORKER-BENCH.lock) for the run.
source "$(dirname "${BASH_SOURCE[0]}")/../../ci/env.sh"
if [ "${1:-}" = "--lock" ]; then
  shift
  set -o noclobber
  echo "onepath voxel optima sweep $(date -Is) threads ${THREADS:-2}" > "$HOME/WORKER-BENCH.lock" || { echo "benchmark lock is held" >&2; exit 3; }
  trap 'rm -f "$HOME/WORKER-BENCH.lock"' EXIT
  set +o noclobber
fi
dotnet build tools/GridOptima/GridOptima.csproj -c Release -v quiet -clp:NoSummary
out="$(dotnet msbuild tools/GridOptima/GridOptima.csproj -p:Configuration=Release -getProperty:OutDir | tr -d '\r')"
only=""
if [ $# -gt 0 ]; then only="$(IFS=,; echo "$*")"; fi
/usr/bin/time -f "voxel wall %es cpu %Us rss %MKB" dotnet "${out}GridOptima.dll" --voxel --maps "${MAPS:-$HOME/onepath/maps}" --set warframe \
  --threads "${THREADS:-2}" --conn "${CONN:-26}" --rule "${RULE:-cut}" ${PER_MAP:+--per-map "$PER_MAP"} ${only:+--only "$only"}
