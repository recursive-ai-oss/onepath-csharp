#!/usr/bin/env bash
# Coverage-guided fuzz run of one target with AFL through SharpFuzz, for machines without clang
# (libFuzzer needs it). Build tools are fetched user-locally under ~/onepath/fuzz.
#
#   tools/fuzz/run-afl.sh <target> [seconds]     default 1000 seconds, one core
#
# Output: ~/onepath/fuzz/<target>/out (AFL's queue, crashes, fuzzer_stats). Crashing inputs are
# replayed with `dotnet OnePath.Fuzz.dll <target> <file>` to see the exception.
set -euo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/../../ci/env.sh"

TARGET="${1:?usage: run-afl.sh <target> [seconds]}"
SECONDS_TO_RUN="${2:-1000}"
BASE="$HOME/onepath/fuzz"
AFL_URL="https://lcamtuf.coredump.cx/afl/releases/afl-latest.tgz"
AFL_SHA256="7167f5f6c1dc54bc3d7f09b4025a5baab54aaa248e755a23c80dfff7493bbfd5"   # afl-2.53b
mkdir -p "$BASE"

if [ ! -x "$BASE/afl/afl-fuzz" ]; then
  curl -fsSL -o "$BASE/afl.tgz" "$AFL_URL"
  echo "$AFL_SHA256  $BASE/afl.tgz" | sha256sum -c -
  rm -rf "$BASE/afl-src" "$BASE/afl"
  mkdir -p "$BASE/afl-src"
  tar xzf "$BASE/afl.tgz" -C "$BASE/afl-src" --strip-components=1
  make -C "$BASE/afl-src" afl-fuzz >/dev/null
  mkdir -p "$BASE/afl"
  cp "$BASE/afl-src/afl-fuzz" "$BASE/afl/"
fi

if [ ! -x "$BASE/tools/sharpfuzz" ]; then
  dotnet tool install SharpFuzz.CommandLine --tool-path "$BASE/tools" >/dev/null
fi

BUILD="$BASE/$TARGET/build"
rm -rf "$BUILD"
dotnet build fuzz/OnePath.Fuzz/OnePath.Fuzz.csproj -c Checked -v quiet -clp:NoSummary -o "$BUILD"
# Instrument the library only; the fuzz driver and the shared math package stay as they are.
"$BASE/tools/sharpfuzz" "$BUILD/RecursiveAI.OnePath.dll"

IN="$BASE/$TARGET/in"
OUT="$BASE/$TARGET/out"
rm -rf "$IN" "$OUT"
mkdir -p "$IN"
for i in 1 2 3 4 5 6; do head -c $((100 + i * 150)) /dev/urandom > "$IN/seed$i"; done
head -c 400 /dev/zero > "$IN/zeros"

# The target is the dotnet host, so AFL cannot find instrumentation in it; SharpFuzz instruments the managed library.
export AFL_SKIP_BIN_CHECK=1 AFL_SKIP_CPUFREQ=1 AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_NO_UI=1 AFL_QUIET=1
timeout --signal=INT "$SECONDS_TO_RUN" nice "$BASE/afl/afl-fuzz" -i "$IN" -o "$OUT" -m none -t 20000 -- \
  dotnet "$BUILD/OnePath.Fuzz.dll" --afl "$TARGET" || true

echo "---- fuzzer_stats"
grep -E 'execs_done|execs_per_sec|paths_total|bitmap_cvg|unique_crashes|unique_hangs|run_time|cycles_done' "$OUT/fuzzer_stats" || true
echo "crashes: $(ls "$OUT/crashes" 2>/dev/null | grep -vc README || true)"
echo "hangs: $(ls "$OUT/hangs" 2>/dev/null | grep -vc README || true)"
