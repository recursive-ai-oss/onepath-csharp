#!/usr/bin/env bash
# Run a command while holding the benchmark-host lock (~/WORKER-BENCH.lock), taken here, inside the job.
#   tools/bench/locked-run.sh "<what>" <expected-minutes> <command...>
# Fails at once when the lock is held (LOCK_MAX_WAIT_SECONDS=N retries every 30 s for up to N seconds instead).
# The lock file is removed when the command ends, however it ends.
set -uo pipefail
what="$1"; minutes="$2"; shift 2

stamp() { date +"%Y-%m-%dT%H:%M:%S%z"; }
if date -d "+1 minute" +%s >/dev/null 2>&1; then end="$(date -d "+${minutes} minutes" +"%Y-%m-%dT%H:%M:%S%z")"; else end="$(date -v+"${minutes}"M +"%Y-%m-%dT%H:%M:%S%z")"; fi

lock="$HOME/WORKER-BENCH.lock"
waited=0
until ( set -o noclobber; echo "onepath $what $(stamp) end $end" > "$lock" ) 2>/dev/null; do
  if [ "$waited" -ge "${LOCK_MAX_WAIT_SECONDS:-0}" ]; then
    echo "bench lock held: $(cat "$lock" 2>/dev/null)" >&2
    exit 75
  fi
  sleep 30; waited=$((waited + 30))
done
trap 'rm -f "$lock"' EXIT
"$@"
