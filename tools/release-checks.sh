#!/usr/bin/env bash
# Checks the release workflow runs on a version tag, so they can also be run by hand.
# Two tag forms: v<version> releases the library packages, scope-v<version> releases the viewer module alone.
#
#   tools/release-checks.sh tag-form  <tag>      the tag looks like v1.2.3, v1.2.3-preview.1, scope-v1.2.3 or scope-v1.2.3-preview.1
#   tools/release-checks.sh changelog <tag>      the changelog has a "## [<version>]" heading
#                                                (CHANGELOG.md for v tags, src/OnePath.Scope/CHANGELOG.md for scope-v tags)
#   tools/release-checks.sh version   <tag>      the tag says the same as the version the package builds as
#                                                (src/OnePath for v tags, src/OnePath.Scope for scope-v tags)
#   tools/release-checks.sh notes     <tag>      prints the changelog section of the version (fails when empty)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."

cmd="${1:-}"; tag="${2:-}"
[[ -n "$cmd" && -n "$tag" ]] || { echo "usage: $0 tag-form|changelog|version|notes <tag>" >&2; exit 2; }
# The tag form picks the version, the changelog and the project the checks look at.
if [[ "$tag" == scope-v* ]]; then
  prefix="scope-v"; changelog_file="src/OnePath.Scope/CHANGELOG.md"; project="src/OnePath.Scope"
else
  prefix="v"; changelog_file="CHANGELOG.md"; project="src/OnePath"
fi
version="${tag#"$prefix"}"

case "$cmd" in
  tag-form)
    [[ "$tag" =~ ^(scope-)?v[0-9]+\.[0-9]+\.[0-9]+(-[0-9A-Za-z.-]+)?$ ]] \
      || { echo "::error::$tag is not a version tag such as v1.2.3, v1.2.3-preview.1 or scope-v1.2.3"; exit 1; }
    ;;
  changelog)
    awk -v heading="## [$version]" 'index($0, heading) == 1 { found = 1 } END { exit !found }' "$changelog_file" \
      || { echo "::error::$changelog_file has no \"## [$version]\" section"; exit 1; }
    ;;
  version)
    # The version is whatever the project builds; the tag has to say the same.
    built="$(dotnet msbuild "$project" -getProperty:PackageVersion | tr -d '[:space:]')"
    if [[ "$prefix$built" != "$tag" ]]; then
      echo "::error::tag $tag does not match the package version $built"
      exit 1
    fi
    ;;
  notes)
    # From the version's heading up to the next version heading or the link definitions at the end of the file.
    out="$(awk -v heading="## [$version]" '
      index($0, heading) == 1 { on = 1; next }
      on && (index($0, "## [") == 1 || $0 ~ /^\[[^]]+\]: /) { exit }
      on { print }
    ' "$changelog_file")"
    if [[ -z "${out//[[:space:]]/}" ]]; then echo "::error::the changelog section for $version is empty"; exit 1; fi
    printf '%s\n' "$out"
    ;;
  *) echo "unknown check: $cmd" >&2; exit 2 ;;
esac
