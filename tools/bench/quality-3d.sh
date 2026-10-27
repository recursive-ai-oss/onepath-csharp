#!/usr/bin/env bash
# Walkable coverage and mean path length of our 3D builder against Recast on the generated levels, one repetition each (no timing).
#   tools/bench/quality-3d.sh [--out dir]
# Needs the levels (tools/geom/gen_levels.py into ~/onepath/geom/256) and the Recast harness (tools/refs) on this machine.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${HOME}/onepath/quality3d"
if [ "${1:-}" = "--out" ]; then OUT="$2"; fi
nice -n 10 bash "$HERE/run-bar.sh" quality3d --reps 1 --out "$OUT" --machine "$(hostname)"
python3 - "$OUT/rows.csv" <<'PY'
import csv, sys, collections
rows = list(csv.reader(open(sys.argv[1])))
hdr, rows = rows[0], rows[1:]
i = {h: n for n, h in enumerate(hdr)}
d = collections.defaultdict(dict)
for r in rows:
    d[(r[i['scenario']], r[i['metric']])][r[i['side']].split('-')[0]] = r[i['value']]
print('| row | metric | ours | recast | delta % |')
print('|---|---|---|---|---|')
for (sc, m), v in sorted(d.items()):
    if m in ('coverage_area', 'path_mean_length', 'polys', 'path_found', 'points_snapped') and 'onepath' in v and 'recast' in v:
        a, b = float(v['onepath']), float(v['recast'])
        print(f'| {sc} | {m} | {a:.4g} | {b:.4g} | {100*(a-b)/b if b else 0:+.2f} |')
PY
