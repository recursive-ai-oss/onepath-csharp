#!/usr/bin/env bash
# Copies RecursiveAI.OneMath packages into ~/onepath/feed, the folder ci/env.sh uses as ONEMATH_FEED when
# it is not set. For machines that cannot see the folder the packages were built into.
#   tools/install-feed.sh <folder containing RecursiveAI.OneMath.*.nupkg>
set -euo pipefail
src="${1:?usage: tools/install-feed.sh <folder with RecursiveAI.OneMath.*.nupkg>}"
dest="$HOME/onepath/feed"
mkdir -p "$dest"
cp -v "$src"/RecursiveAI.OneMath.*.nupkg "$dest"/
