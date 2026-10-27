#!/usr/bin/env bash
# Installs the fuzzing tools into $FUZZ_TOOLS (default ~/onepath/fuzz-tools): the SharpFuzz instrumenter
# (a dotnet tool) and the libFuzzer driver for .NET (a prebuilt release binary, pinned by hash).
source "$(dirname "${BASH_SOURCE[0]}")/../ci/env.sh"
TOOLS="${FUZZ_TOOLS:-$HOME/onepath/fuzz-tools}"
mkdir -p "$TOOLS"
SHARPFUZZ_VERSION=2.3.0
DRIVER_TAG=v2025.05.02.0904
DRIVER_SHA256=c2c2a90d94c409a4af339a0d4f244e0442c5a5d249be0f1252ba07871f285958

if [ ! -x "$TOOLS/sharpfuzz" ]; then
  dotnet tool install SharpFuzz.CommandLine --version "$SHARPFUZZ_VERSION" --tool-path "$TOOLS"
fi
if [ ! -x "$TOOLS/libfuzzer-dotnet" ]; then
  curl -sSfL "https://github.com/Metalnem/libfuzzer-dotnet/releases/download/$DRIVER_TAG/libfuzzer-dotnet-ubuntu" -o "$TOOLS/libfuzzer-dotnet"
  echo "$DRIVER_SHA256  $TOOLS/libfuzzer-dotnet" | sha256sum -c -
  chmod +x "$TOOLS/libfuzzer-dotnet"
fi
echo "$TOOLS"
