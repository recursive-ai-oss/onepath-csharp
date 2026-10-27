#!/usr/bin/env bash
# Generate the benchmark inputs under $ONEPATH_HOME (default ~/onepath): our own test levels at two sizes
# (geom/96 for smoke runs, geom/256 for the reference rows), and the Moving AI maps (tools/maps/maps.sh).
#   tools/bench/prepare.sh [--no-maps]
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
H="${ONEPATH_HOME:-$HOME/onepath}"
PY="${PYTHON:-python3}"

"$PY" "$REPO/tools/geom/gen_levels.py" "$H/geom/96" --size 96 --points 400 >/dev/null
"$PY" "$REPO/tools/geom/gen_levels.py" "$H/geom/256" --size 256 --points 4000 >/dev/null
"$PY" "$REPO/tools/geom/gen_shapes.py" "$H/shapes" >/dev/null
( cd "$H/geom" && { command -v sha256sum >/dev/null && sha256sum */*.obj */*.points || shasum -a 256 */*.obj */*.points; } > SHA256SUMS )
echo "levels written under $H/geom (hashes in SHA256SUMS)"
[ "${1:-}" = "--no-maps" ] || "$REPO/tools/maps/maps.sh" all
