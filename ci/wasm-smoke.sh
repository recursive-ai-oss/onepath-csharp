#!/usr/bin/env bash
# Build the browser-wasm smoke app and run it headless under Node (no browser needed).
# Installs the wasm-tools workload into the active SDK if it is missing.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
if ! dotnet workload list | grep -q '^wasm-tools'; then
  dotnet workload install wasm-tools
fi
dotnet publish tests/WasmSmoke/WasmSmoke.csproj -c Release -v quiet -clp:NoSummary -p:WasmEmitSymbolMap=false
bundle="$(dotnet msbuild tests/WasmSmoke/WasmSmoke.csproj -p:Configuration=Release -getProperty:OutDir | tr -d "\r")AppBundle"
# Use Node from PATH, else the one the wasm-tools workload ships.
node_bin="$(command -v node || true)"
if [ -z "$node_bin" ]; then
  node_bin="$(ls "$(dirname "$(command -v dotnet)")"/packs/Microsoft.NET.Runtime.Emscripten.*.Node.*/*/tools/bin/node 2>/dev/null | head -n 1 || true)"
fi
"$node_bin" --version
(cd "$bundle" && "$node_bin" main.mjs)
