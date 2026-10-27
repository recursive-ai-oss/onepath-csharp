#!/usr/bin/env bash
# Build the core projects. Usage: ci/build.sh [Release|Checked|Debug ...]   (default: Release Checked)
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
configs=("$@")
[ ${#configs[@]} -gt 0 ] || configs=(Release Checked)
dotnet --version
for c in "${configs[@]}"; do
  for p in "${CORE_PROJECTS[@]}"; do
    dotnet build "$p" -c "$c" -v quiet -clp:NoSummary
  done
done
