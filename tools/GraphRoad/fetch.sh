#!/usr/bin/env bash
# Fetches 9th DIMACS Challenge road graphs (fetch-only; never committed) into ${GRAPHS:-$HOME/onepath/maps/graphs}.
#   tools/GraphRoad/fetch.sh [name ...]        default: USA-road-d.NY (264,346 nodes, 733,846 arcs, travel distance)
#
# Terms (https://users.diag.uniroma1.it/challenge9/download.shtml): "Unless otherwise stated, the files available at this
# page contain data/software in the public domain and can be freely downloaded." The road graphs are derived from US Census
# TIGER/Line data. Verdict: OK, fetch-only; cite the challenge ("The Shortest Path Problem: Ninth DIMACS Implementation
# Challenge", DeMetrescu, Goldberg, Johnson, eds., AMS 2009) wherever results are published.
#
# Each file's SHA-256 is checked against manifest.tsv (name<TAB>sha256) when it has an entry, and printed otherwise.
set -euo pipefail
dir="${GRAPHS:-$HOME/onepath/maps/graphs}"
manifest="$(dirname "${BASH_SOURCE[0]}")/manifest.tsv"
mkdir -p "$dir"
names=("$@")
[ ${#names[@]} -gt 0 ] || names=(USA-road-d.NY)
for n in "${names[@]}"; do
  f="$dir/$n.gr.gz"
  if [ ! -s "$f" ]; then
    curl -sSfL -m 600 -o "$f" "http://users.diag.uniroma1.it/challenge9/data/USA-road-d/$n.gr.gz"
  fi
  sum="$(sha256sum "$f" | cut -d' ' -f1)"
  want="$(grep -P "^$n\t" "$manifest" 2>/dev/null | cut -f2 || true)"
  if [ -n "$want" ] && [ "$want" != "$sum" ]; then
    echo "SHA-256 mismatch for $n: $sum (manifest $want)" >&2
    exit 1
  fi
  echo "$f $sum"
done
