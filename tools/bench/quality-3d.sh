#!/usr/bin/env bash
# Walkable coverage and mean path length of our 3D builder against Recast on the generated levels, one repetition each (no timing).
#   tools/bench/quality-3d.sh [--out dir]
# Needs the levels (tools/geom/gen_levels.py into ~/onepath/geom/256) and the Recast harness (tools/refs) on this machine.
# Paths are measured with both search node models (polygon, the default, and crossing); the reference's own model is polygon nodes
# with edge-midpoint costs, which crossing reproduces exactly. A pair whose goal is unreachable ends at the closest point of the
# polygon nearest the goal (path_partial counts them); both sides handle it the same way.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT="${HOME}/onepath/quality3d"
if [ "${1:-}" = "--out" ]; then OUT="$2"; fi
for model in polygon crossing; do
  ONEPATH_NODE_MODEL=$model nice -n 10 bash "$HERE/run-bar.sh" quality3d --reps 1 --out "$OUT/$model" --machine "$(hostname)"
done
python3 - "$OUT/polygon/rows.csv" "$OUT/crossing/rows.csv" <<'PY'
import csv, sys, collections
def load(path):
    rows = list(csv.reader(open(path)))
    hdr, rows = rows[0], rows[1:]
    i = {h: n for n, h in enumerate(hdr)}
    d = collections.defaultdict(dict)
    for r in rows:
        d[(r[i['scenario']].replace('quality/', '').replace('/recast', ''), r[i['metric']])][r[i['side']].split('-')[0]] = r[i['value']]
    return d
poly, cross = load(sys.argv[1]), load(sys.argv[2])
print('| row | metric | ours | recast | delta % |')
print('|---|---|---|---|---|')
for (sc, m), v in sorted(poly.items()):
    if sc.startswith('build/') and m in ('coverage_area', 'polys') and 'onepath' in v and 'recast' in v:
        a, b = float(v['onepath']), float(v['recast'])
        print(f'| {sc} | {m} | {a:.4g} | {b:.4g} | {100*(a-b)/b if b else 0:+.2f} |')
print()
print('| row | path mean polygon / crossing / recast | delta % polygon | delta % crossing | found ours / recast | partial ours / recast |')
print('|---|---|---|---|---|---|')
for (sc, m), v in sorted(poly.items()):
    if sc.startswith('query/') and m == 'path_mean_length' and 'onepath' in v and 'recast' in v:
        c = cross[(sc, m)]
        rc = float(v['recast'])
        a, b = float(v['onepath']), float(c['onepath'])
        f = lambda k: f"{float(poly[(sc, k)]['onepath']):.0f} / {float(poly[(sc, k)]['recast']):.0f}"
        print(f'| {sc} | {a:.2f} / {b:.2f} / {rc:.2f} | {100*(a-rc)/rc:+.2f} | {100*(b-rc)/rc:+.2f} | {f("path_found")} | {f("path_partial")} |')
PY
