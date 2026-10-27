#!/usr/bin/env bash
# Runs one fuzz target under libFuzzer for a bounded time, in the Checked configuration.
#   fuzz/run.sh <target> [seconds=600] [workdir]
# Targets: see `OnePath.Fuzz` with no arguments. Seeds come from testdata/fuzz/<target>/. The workdir keeps the
# corpus and any crash artifacts (crash-*, one per failure); it defaults to a temporary folder.
source "$(dirname "${BASH_SOURCE[0]}")/../ci/env.sh"
target="${1:?usage: fuzz/run.sh <target> [seconds] [workdir]}"
seconds="${2:-600}"
work="${3:-$(mktemp -d)}"
TOOLS="$(bash "$REPO_ROOT/fuzz/get-tools.sh" | tail -n 1)"

mkdir -p "$work/corpus" "$work/artifacts"
rm -rf "$work/app"
dotnet publish fuzz/OnePath.Fuzz/OnePath.Fuzz.csproj -c Checked -o "$work/app" -v quiet -clp:NoSummary
# Instrument the library (not the fuzz harness) so libFuzzer sees its coverage.
"$TOOLS/sharpfuzz" "$work/app/RecursiveAI.OnePath.dll"
cp -n "$REPO_ROOT/testdata/fuzz/$target/"* "$work/corpus/" 2>/dev/null || true

"$TOOLS/libfuzzer-dotnet" --target_path="$work/app/OnePath.Fuzz" --target_arg="$target" \
  -max_total_time="$seconds" -rss_limit_mb=2048 -artifact_prefix="$work/artifacts/" \
  "$work/corpus" 2>&1 | tee "$work/$target.log" | tail -n 25
status=${PIPESTATUS[0]}
echo "fuzz target=$target exit=$status crashes=$(ls "$work/artifacts" | wc -l) work=$work"
exit "$status"
