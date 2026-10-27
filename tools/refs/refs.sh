#!/usr/bin/env bash
# Fetch, verify and build the benchmark references, unattended, into a user-local folder.
#
#   tools/refs/refs.sh bootstrap   user-local CMake (and, on macOS, a .NET SDK) under $ONEPATH_HOME/tools
#   tools/refs/refs.sh fetch       download every reference release and check its SHA-256 against manifest.tsv
#   tools/refs/refs.sh build       build and install each reference (idempotent: a finished build is skipped)
#   tools/refs/refs.sh harness     build the native black-box drivers in bench/Harness/Native against the installs
#   tools/refs/refs.sh all         bootstrap, fetch, build, harness
#
# ONEPATH_HOME (default ~/onepath) holds everything: tools/, refs/<name>-<version>/{src,build,install}, bin/.
# The references are benchmark-only black boxes: this script only runs their documented build, and nothing here
# opens, lists or searches their source trees.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
H="${ONEPATH_HOME:-$HOME/onepath}"
JOBS="${REFS_JOBS:-8}"
MANIFEST="$HERE/manifest.tsv"

case "$(uname -s)-$(uname -m)" in
  Linux-x86_64) PLAT=linux-x64 ;;
  Darwin-arm64) PLAT=osx-arm64 ;;
  *) echo "unsupported platform $(uname -s)-$(uname -m)" >&2; exit 2 ;;
esac

sha256_of() { if command -v sha256sum >/dev/null; then sha256sum "$1" | cut -d' ' -f1; else shasum -a 256 "$1" | cut -d' ' -f1; fi; }

# row <id> <field>: field 2 = version, 3 = url, 4 = sha256
row() { awk -F'\t' -v id="$1" -v f="$2" '$1==id {print $f}' "$MANIFEST"; }

fetch_one() { # <id> -> prints the archive path
  local id="$1" ver url sha file
  ver="$(row "$id" 2)"; url="$(row "$id" 3)"; sha="$(row "$id" 4)"
  [ -n "$ver" ] || { echo "no manifest row for $id" >&2; return 1; }
  mkdir -p "$H/refs/_dl"
  file="$H/refs/_dl/$id-$ver.tar.gz"
  if [ ! -f "$file" ] || [ "$(sha256_of "$file")" != "$sha" ]; then
    echo "fetching $id $ver" >&2
    curl -fsSL --retry 3 -o "$file.part" "$url"
    mv "$file.part" "$file"
  fi
  local got; got="$(sha256_of "$file")"
  if [ "$got" != "$sha" ]; then echo "SHA-256 mismatch for $id $ver: expected $sha, got $got" >&2; return 1; fi
  echo "$file"
}

unpack() { # <id> -> prints the src dir
  local id="$1" ver dir file
  ver="$(row "$id" 2)"; dir="$H/refs/$id-$ver"
  if [ ! -f "$dir/.unpacked" ]; then
    file="$(fetch_one "$id")"
    rm -rf "$dir/src"; mkdir -p "$dir/src"
    tar -xzf "$file" -C "$dir/src" --strip-components=1
    touch "$dir/.unpacked"
  fi
  echo "$dir"
}

cmake_bin() {
  if [ -x "$H/tools/cmake/bin/cmake" ]; then echo "$H/tools/cmake/bin/cmake"
  elif [ -x "$H/tools/cmake/CMake.app/Contents/bin/cmake" ]; then echo "$H/tools/cmake/CMake.app/Contents/bin/cmake"
  elif command -v cmake >/dev/null; then command -v cmake
  else echo "cmake missing: run tools/refs/refs.sh bootstrap" >&2; return 1; fi
}

bootstrap() {
  if [ ! -d "$H/tools/cmake" ]; then
    local id="cmake-$PLAT" file
    file="$(fetch_one "$id")"
    mkdir -p "$H/tools/cmake"
    tar -xzf "$file" -C "$H/tools/cmake" --strip-components=1
  fi
  echo "cmake: $(cmake_bin)"
  if [ "$PLAT" = osx-arm64 ] && [ ! -x "$H/dotnet/dotnet" ]; then
    # .NET 10 SDK, user-local; SHA-512 from the release's published checksum file.
    local v=10.0.400 sha=e440e9a58d4ff7741c8342ac3e086fa9ee2dadc25e01c0449a88317a74cfbd63625b8092c3b2a131ae14b16ab3401e9cc470e578e4c65a72a0b5786bd2308cde
    local f="$H/refs/_dl/dotnet-sdk-$v-osx-arm64.tar.gz"
    mkdir -p "$H/refs/_dl" "$H/dotnet"
    curl -fsSL --retry 3 -o "$f" "https://builds.dotnet.microsoft.com/dotnet/Sdk/$v/dotnet-sdk-$v-osx-arm64.tar.gz"
    [ "$(shasum -a 512 "$f" | cut -d' ' -f1)" = "$sha" ] || { echo "dotnet SDK SHA-512 mismatch" >&2; exit 1; }
    tar -xzf "$f" -C "$H/dotnet"
  fi
}

fetch_all() { for id in recast rvo2 rvo2cs warthog-core warthog-jps; do fetch_one "$id" >/dev/null; echo "ok $id $(row "$id" 2) $(row "$id" 4)"; done; }

cmake_build() { # <dir> [extra cmake args...]  (configure, build, install into <dir>/install)
  local dir="$1"; shift
  local cm; cm="$(cmake_bin)"
  "$cm" -S "$dir/src" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$dir/install" "$@"
  "$cm" --build "$dir/build" -j "$JOBS"
  "$cm" --install "$dir/build"
}

build_recast() {
  local dir; dir="$(unpack recast)"
  [ -f "$dir/.built" ] && { echo "recast: built"; return; }
  # Documented CMake options: skip the demo, the tests and the examples; the libraries are what we drive.
  cmake_build "$dir" -DRECASTNAVIGATION_DEMO=OFF -DRECASTNAVIGATION_TESTS=OFF -DRECASTNAVIGATION_EXAMPLES=OFF
  touch "$dir/.built"
}

build_rvo2() {
  local dir; dir="$(unpack rvo2)"
  [ -f "$dir/.built" ] && { echo "rvo2: built"; return; }
  cmake_build "$dir"
  touch "$dir/.built"
}

build_warthog() {
  local core jps cm; cm="$(cmake_bin)"
  core="$(unpack warthog-core)"; jps="$(unpack warthog-jps)"
  if [ ! -f "$core/.built" ]; then
    "$cm" -S "$core/src" -B "$core/build" -DCMAKE_BUILD_TYPE=Release
    "$cm" --build "$core/build" -j "$JOBS"
    touch "$core/.built"
  fi
  if [ ! -f "$jps/.built" ]; then
    # warthog-jps picks up warthog-core from extern/warthog-core when it is present (documented submodule layout),
    # so no network access is needed during the build.
    rm -rf "$jps/src/extern/warthog-core"; mkdir -p "$jps/src/extern"
    cp -R "$core/src" "$jps/src/extern/warthog-core"
    "$cm" -S "$jps/src" -B "$jps/build" -DCMAKE_BUILD_TYPE=Release
    "$cm" --build "$jps/build" -j "$JOBS"
    touch "$jps/.built"
  fi
}

dotnet_bin() {
  if [ -n "${ONEPATH_DOTNET:-}" ]; then echo "$ONEPATH_DOTNET"
  elif [ -x "$H/dotnet/dotnet" ]; then echo "$H/dotnet/dotnet"
  elif [ -x "$HOME/dotnet10/dotnet" ]; then echo "$HOME/dotnet10/dotnet"
  else echo dotnet; fi
}

build_rvo2cs() {
  local dir; dir="$(unpack rvo2cs)"
  [ -f "$dir/.built" ] && { echo "rvo2cs: built"; return; }
  local dn; dn="$(dotnet_bin)"
  # The release's own projects target an old .NET Framework, so our Rvo2CsLib project compiles its library
  # sources (everything but the examples) as shipped into Rvo2Cs.dll.
  "$dn" build "$REPO/bench/Harness/Rvo2CsLib/Rvo2CsLib.csproj" -c Release -o "$dir/out" -p:Rvo2CsSrc="$dir/src" -v quiet -nologo
  touch "$dir/.built"
}

build_harness() {
  local cm rc rv; cm="$(cmake_bin)"
  rc="$(unpack recast)/install"; rv="$(unpack rvo2)/install"
  mkdir -p "$H/bin"
  "$cm" -S "$REPO/bench/Harness/Native" -B "$H/harness-build" -DCMAKE_BUILD_TYPE=Release \
    -DRECAST_PREFIX="$rc" -DRVO2_PREFIX="$rv" -DCMAKE_INSTALL_PREFIX="$H"
  "$cm" --build "$H/harness-build" -j "$JOBS"
  "$cm" --install "$H/harness-build"
  # The .NET drivers: DotRecast from NuGet, RVO2-CS from the build above.
  local dn rvocs; dn="$(dotnet_bin)"; rvocs="$(unpack rvo2cs)/out/Rvo2Cs.dll"
  "$dn" build "$REPO/bench/Harness/RefHarness/RefHarness.csproj" -c Release -o "$H/bin/refharness" -p:Rvo2CsDll="$rvocs" -v quiet -nologo
  echo "harness binaries in $H/bin"
}

# Warthog needs C++20 time-zone support in the standard library (std::chrono::current_zone), which Apple's libc++
# does not have yet; there the grid references are skipped with a warning and their bar rows report "reference failed".
build_warthog_optional() {
  if [ "$PLAT" != osx-arm64 ]; then build_warthog; return; fi
  if ! ( build_warthog ); then
    echo "WARNING: warthog did not build on this platform ($PLAT); grid reference rows will be unavailable here" >&2
  fi
}

case "${1:-}" in
  bootstrap) bootstrap ;;
  fetch) fetch_all ;;
  build) bootstrap; fetch_all; build_recast; build_rvo2; build_warthog_optional; build_rvo2cs ;;
  harness) build_harness ;;
  all) bootstrap; fetch_all; build_recast; build_rvo2; build_warthog_optional; build_rvo2cs; build_harness ;;
  *) sed -n '2,12p' "$0"; exit 2 ;;
esac
