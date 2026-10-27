#!/usr/bin/env bash
# Coverage-guided fuzzing of one target with AFL++ and SharpFuzz (no native libFuzzer host needed).
#   fuzz/run-afl.sh <target> <seconds> <workdir>
# Needs on PATH: dotnet (10), afl-fuzz, sharpfuzz (the SharpFuzz.CommandLine tool). One process, Checked build.
# Findings land in <workdir>/out/default/crashes and hangs; replay one with
#   dotnet <build>/OnePath.Fuzz.dll <target> <file>
set -euo pipefail
target="${1:?target name (see Targets in fuzz/OnePath.Fuzz/Program.cs)}"
seconds="${2:?duration in seconds}"
work="${3:?work directory}"
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [ -z "${ONEMATH_FEED:-}" ] && [ -d "$HOME/onepath/feed" ]; then export ONEMATH_FEED="$HOME/onepath/feed"; fi

mkdir -p "$work/seeds" "$work/out"
build="$work/build"
ONEPATH_ARTIFACTS="$work/artifacts" dotnet build "$root/fuzz/OnePath.Fuzz/OnePath.Fuzz.csproj" -c Checked -o "$build" -v quiet -clp:NoSummary

# Instrument the library and the math package it calls.
sharpfuzz "$build/RecursiveAI.OnePath.dll"
sharpfuzz "$build/RecursiveAI.OneMath.dll"

# A few fixed seeds: runs of structured bytes.
i=0
for pattern in '\x00' '\x41' '\x55' '\x7f' '\xc3' '\x10\x80\x01\x44'; do
  printf "$pattern%.0s" $(seq 1 120) > "$work/seeds/seed$i.bin"
  i=$((i + 1))
done

export AFL_SKIP_BIN_CHECK=1 AFL_NO_UI=1 AFL_I_DONT_CARE_ABOUT_MISSING_CRASHES=1 AFL_NO_AFFINITY=1 AFL_SKIP_CPUFREQ=1
afl-fuzz -V "$seconds" -i "$work/seeds" -o "$work/out" -t 20000 -m none -- dotnet "$build/OnePath.Fuzz.dll" "$target" --afl
echo "crashes: $(ls "$work/out/default/crashes" 2>/dev/null | grep -vc README || true)"
echo "hangs:   $(ls "$work/out/default/hangs" 2>/dev/null | wc -l)"
grep -E 'execs_done|corpus_count|edges_found|total_edges|bitmap_cvg|saved_crashes|saved_hangs' "$work/out/default/fuzzer_stats" || true
