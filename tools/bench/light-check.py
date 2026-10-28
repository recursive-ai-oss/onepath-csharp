#!/usr/bin/env python3
"""Light-check report: does every row of a suite run on both sides and print every metric?

  tools/bench/light-check.py <suite.tsv> <run dir> [<suite.tsv> <run dir> ...]

Reads the suite file and the run's rows.csv. A row is present when ours and the reference each printed its compared metric with a
finite value; the report lists every row that is not, every NaN/empty value, every 'error' metric and, for 2D tile-map rows, whether the
exact-area gate columns are there. Prints markdown; exits 1 when a problem was found. It judges completeness, never speed.
"""
import csv
import math
import sys


def load_suite(path):
    rows = []
    for line in open(path, encoding="utf-8"):
        line = line.rstrip("\n")
        if not line or line.startswith("#"):
            continue
        f = line.split("\t")
        rows.append({"id": f[0], "part": f[1], "driver": f[2], "metric": f[3], "args": f[6] if len(f) > 6 else ""})
    return rows


def finite(v):
    try:
        return math.isfinite(float(v))
    except ValueError:
        return False


# Reference variants we have no counterpart of yet (JPS with improved pruning): their rows are reference-only, and counted apart.
REFERENCE_ONLY = ("--alg jpsP ", "--alg jpsP+ ")


def check(suite_path, run_dir):
    suite = load_suite(suite_path)
    by = {}
    for r in csv.DictReader(open(run_dir + "/rows.csv", encoding="utf-8")):
        by.setdefault(r["row_id"], []).append(r)
    problems, ok, gate, ref_only = [], 0, 0, 0
    for row in suite:
        got = by.get(row["id"], [])
        ours = [r for r in got if r["side"].startswith("onepath")]
        ref = [r for r in got if not r["side"].startswith("onepath")]
        bad = []
        only = any(k in row["args"] + " " for k in REFERENCE_ONLY)
        if only and not ours:
            ref_only += 1
            ours = [{"metric": row["metric"], "side": "-", "value": "0", "unit": ""}]
            skip_ours = True
        else:
            skip_ours = False
        if not ours:
            bad.append("no ours rows")
        if not ref:
            bad.append("no reference rows")
        for side, rs in ((("reference", ref),) if skip_ours else (("ours", ours), ("reference", ref))):
            have = {r["metric"] for r in rs}
            if rs and row["metric"] not in have:
                bad.append(f"{side} lacks {row['metric']}")
            for r in rs:
                if r["metric"] == "error":
                    bad.append(f"{side} error: {r['value']}")
                elif r["unit"] != "text" and not finite(r["value"]):
                    bad.append(f"{side} {r['metric']} = {r['value']!r}")
            if rs and "peak_rss" not in have:
                bad.append(f"{side} lacks peak_rss")
            elif rs and "peak_rss" in have and next(float(r["value"]) for r in rs if r["metric"] == "peak_rss") <= 0:
                bad.append(f"{side} peak_rss is 0")
        if ("tilemap" in row["args"] or "/shapes/" in row["args"]) and row["part"] == "build":
            have = {r["metric"] for r in ours}
            if {"exact_area", "delta_vs_exact_pct"} <= have:
                gate += 1
            else:
                bad.append("2D gate columns missing")
        if bad:
            problems.append((row["id"], "; ".join(bad)))
        else:
            ok += 1
    return len(suite), ok, gate, ref_only, problems


def main(argv):
    if len(argv) < 2 or len(argv) % 2:
        print(__doc__)
        return 2
    failed = False
    print("| suite | rows | complete | of which reference-only | with 2D gate columns | problems |\n|---|---:|---:|---:|---:|---:|")
    detail = []
    for i in range(0, len(argv), 2):
        n, ok, gate, ref_only, problems = check(argv[i], argv[i + 1])
        name = argv[i].rsplit("/", 1)[-1]
        print(f"| {name} | {n} | {ok} | {ref_only} | {gate} | {len(problems)} |")
        failed |= bool(problems)
        detail += [(name, rid, msg) for rid, msg in problems]
    if detail:
        print("\n| suite | row | problem |\n|---|---|---|")
        for name, rid, msg in detail:
            print(f"| {name} | {rid} | {msg} |")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
