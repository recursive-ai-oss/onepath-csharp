#!/usr/bin/env bash
# Hash matrix: run the scenarios under every column this machine supports and compare manifests.
#
#   tools/hash-matrix.sh run <outdir> [column ...]   build and run columns, writing <outdir>/<tag>-<column>.manifest
#   tools/hash-matrix.sh compare <manifest ...>      exit 0 if every manifest is identical, else show the differences
#   tools/hash-matrix.sh all <outdir>                run, then compare what was produced
#
# Columns (default: all that apply here):
#   jit          framework-dependent, default JIT, 1 thread
#   jit-threads  same, all cores (--threads 0)
#   jit-scalar   DOTNET_EnableHWIntrinsic=0 (no SIMD instructions)
#   jit-noavx2   DOTNET_EnableAVX2=0 (x64 only)
#   jit-fulljit  DOTNET_TieredCompilation=0 (no tier-0 code)
#   aot          NativeAOT for this machine's runtime
#   wasm         browser-wasm, interpreted, under Node
#   wasm-aot     browser-wasm, AOT-compiled (needs the wasm-tools workload), under Node
# The tag in file names is $HASHMATRIX_TAG, default <os>-<arch> (for example linux-x86_64).
CALLER_PWD="$PWD"
source "$(dirname "${BASH_SOURCE[0]}")/../ci/env.sh"

PROJECT=tools/HashMatrix/HashMatrix.csproj
TAG="${HASHMATRIX_TAG:-$(uname -s | tr 'A-Z' 'a-z')-$(uname -m)}"

find_node() {
  local n
  n="$(command -v node || true)"
  if [ -z "$n" ]; then
    n="$(ls "$(dirname "$(command -v dotnet)")"/packs/Microsoft.NET.Runtime.Emscripten.*.Node.*/*/tools/bin/node 2>/dev/null | head -n 1 || true)"
  fi
  echo "$n"
}

ensure_wasm_tools() {
  if ! dotnet workload list | grep -q '^wasm-tools'; then
    dotnet workload install wasm-tools
  fi
}

# publish <name> <extra dotnet publish args...>; prints the folder holding the build
publish() {
  local name="$1"; shift
  local art="$OUT/build/$name"
  dotnet publish "$PROJECT" -c Release -p:ArtifactsPath="$art" -v quiet -clp:NoSummary "$@" >&2
  echo "$art"
}

run_jit() { # <column> <threads> [ENV=VALUE ...]
  local col="$1" threads="$2"; shift 2
  if [ -z "${JIT_DIR:-}" ]; then
    JIT_DIR="$OUT/build/jit/pub"
    publish jit -o "$JIT_DIR" >/dev/null
  fi
  env "$@" dotnet "$JIT_DIR/HashMatrix.dll" --threads "$threads" --out "$OUT/$TAG-$col.manifest" >/dev/null
}

run_column() {
  local col="$1"
  echo "== $col" >&2
  case "$col" in
    jit)         run_jit jit 1 A=1 ;;
    jit-threads) run_jit jit-threads 0 A=1 ;;
    jit-scalar)  run_jit jit-scalar 1 DOTNET_EnableHWIntrinsic=0 ;;
    jit-noavx2)
      case "$(uname -m)" in
        x86_64|amd64) run_jit jit-noavx2 1 DOTNET_EnableAVX2=0 ;;
        *) echo "   skipped (x64 only)" >&2; return 0 ;;
      esac ;;
    jit-fulljit) run_jit jit-fulljit 1 DOTNET_TieredCompilation=0 ;;
    aot)
      local d; d="$OUT/build/aot/pub"
      publish aot --use-current-runtime -p:PublishAot=true -o "$d" >/dev/null
      local exe="$d/HashMatrix"; [ -x "$exe" ] || exe="$d/HashMatrix.exe"
      "$exe" --threads 1 --out "$OUT/$TAG-aot.manifest" >/dev/null ;;
    wasm|wasm-aot)
      local node; node="$(find_node)"
      [ -n "$node" ] || { echo "   no node found" >&2; return 1; }
      ensure_wasm_tools
      local extra=(); [ "$col" = wasm-aot ] && extra=(-p:RunAOTCompilation=true)
      local art; art="$(publish "$col" -r browser-wasm "${extra[@]}")"
      # Node writes the manifest to stdout; the module's file system is not the host's.
      (cd "$art/bin/HashMatrix/release_browser-wasm/AppBundle" && "$node" main.mjs --threads 1) > "$OUT/$TAG-$col.manifest" ;;
    *) echo "unknown column: $col" >&2; return 2 ;;
  esac
}

abspath() { case "$1" in /*) echo "$1" ;; *) echo "$CALLER_PWD/$1" ;; esac; }

cmd_run() {
  OUT="$(abspath "$1")"; mkdir -p "$OUT"; shift
  local cols=("$@")
  [ ${#cols[@]} -gt 0 ] || cols=(jit jit-threads jit-scalar jit-noavx2 jit-fulljit aot wasm wasm-aot)
  for c in "${cols[@]}"; do run_column "$c"; done
  for f in "$OUT"/*.manifest; do echo "$(basename "$f") $(grep '^manifest ' "$f" | cut -d' ' -f2)"; done
}

cmd_compare() {
  local args=() a
  for a in "$@"; do args+=("$(abspath "$a")"); done
  set -- "${args[@]}"
  [ $# -ge 2 ] || { echo "compare needs at least two manifests" >&2; return 2; }
  local ref="$1" bad=0 f
  for f in "$@"; do
    if [ "$(grep -c '^manifest ' "$f")" != 1 ]; then echo "MALFORMED $f"; bad=1; fi
  done
  for f in "$@"; do
    if diff -q <(tr -d '\r' < "$ref") <(tr -d '\r' < "$f") >/dev/null; then
      echo "equal    $(basename "$f")  $(grep '^manifest ' "$f" | cut -d' ' -f2)"
    else
      echo "DIFFERS  $(basename "$f")"; diff <(tr -d '\r' < "$ref") <(tr -d '\r' < "$f") | sed 's/^/    /' | head -20
      bad=1
    fi
  done
  return $bad
}

case "${1:-}" in
  run) shift; cmd_run "$@" ;;
  compare) shift; cmd_compare "$@" ;;
  all) shift; cmd_run "$@" && cmd_compare "$(abspath "$1")"/*.manifest ;;
  *) sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
