#!/usr/bin/env bash
# Run the unit tests. Usage: ci/test.sh [Release|Checked|Debug ...]   (default: Release Checked)
source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
configs=("$@")
[ ${#configs[@]} -gt 0 ] || configs=(Release Checked)
for c in "${configs[@]}"; do
  dotnet test tests/OnePath.Tests/OnePath.Tests.csproj -c "$c" -v quiet -clp:NoSummary
done
