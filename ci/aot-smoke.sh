#!/usr/bin/env bash
# Publish the smoke app with NativeAOT and full trimming for this machine's runtime, fail on any
# warning (warnings are errors in the build), then run it.
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
out="$(mktemp -d)"
trap 'rm -rf "$out"' EXIT
dotnet publish tests/AotSmoke/AotSmoke.csproj -c Release --use-current-runtime -o "$out" -v quiet -clp:NoSummary
exe="$out/AotSmoke"
[ -x "$exe" ] || exe="$out/AotSmoke.exe"
"$exe"
