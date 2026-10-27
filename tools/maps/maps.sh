#!/usr/bin/env bash
# Fetch the Moving AI Lab 2D grid and 3D voxel benchmark sets at pinned SHA-256 and unpack them.
#
#   tools/maps/maps.sh [all|<set> ...]     default: all
#
# Zips land in $ONEPATH_HOME/maps/_zips (or $ONEPATH_MAPS when set), each set unpacks into
# $ONEPATH_HOME/maps/<set>/ (maps and scenarios side by side, as the scenario files expect).
# The data is never committed: it is released under the Open Data Commons Attribution License, but its game-derived
# maps carry research-use permissions only (see THIRD-PARTY-NOTICES.md). Idempotent.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
H="${ONEPATH_HOME:-$HOME/onepath}"
OUT="${ONEPATH_MAPS:-$H/maps}"
MANIFEST="$HERE/manifest.tsv"
sha256_of() { if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi; }

want=("$@"); [ ${#want[@]} -eq 0 ] && want=(all)
mkdir -p "$OUT/_zips"
while IFS=$'\t' read -r set file url sha; do
  case "$set" in \#*|'') continue ;; esac
  if [ "${want[0]}" != all ]; then
    keep=0; for w in "${want[@]}"; do [ "$w" = "$set" ] && keep=1; done
    [ $keep -eq 1 ] || continue
  fi
  zip="$OUT/_zips/$file"
  if [ ! -f "$zip" ] || [ "$(sha256_of "$zip")" != "$sha" ]; then
    echo "fetching $file"
    curl -fsSL --retry 3 -o "$zip.part" "$url"; mv "$zip.part" "$zip"
  fi
  got="$(sha256_of "$zip")"
  [ "$got" = "$sha" ] || { echo "SHA-256 mismatch for $file: expected $sha, got $got" >&2; exit 1; }
  if [ ! -f "$OUT/$set/.unpacked-$file" ]; then
    mkdir -p "$OUT/$set"
    unzip -q -o "$zip" -d "$OUT/$set"
    touch "$OUT/$set/.unpacked-$file"
  fi
  echo "ok $file $sha"
done < "$MANIFEST"
echo "total size: $(du -sk "$OUT" | cut -f1) KiB under $OUT"
