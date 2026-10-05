#!/bin/bash
# Pure Eclipse Zenoh transport micro-benchmark (no ROS): one publisher, 1 or 3 subscriber processes,
# network vs shared memory, subscriber copying the payload vs reading it in place.
# Build first: scripts/build.sh
# Usage: run_zenoh_pure.sh [mb=12] [hz=10]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MB=${1:-12}; HZ=${2:-10}; B=${B:-$ROOT/build/bench/zenoh_pure}
SECS=10; WARMUP=5; PORT=7461
[ -x "$B/zpub" ] || { echo "build first: scripts/build.sh"; exit 1; }

run() {  # mode copy nsub
  local mode=$1 copy=$2 nsub=$3
  echo "=== $mode, ${nsub} subscriber process(es), copy=$copy, ${MB} MB @ ${HZ} Hz"
  $B/zpub $mode $MB $HZ $SECS $WARMUP $PORT > /tmp/zpub.out 2>&1 &
  local pp=$!
  sleep 1.5
  for i in $(seq $nsub); do
    $B/zsub $mode $copy $((SECS + WARMUP)) $PORT > /tmp/zsub_$i.out 2>&1 &
  done
  wait $pp; sleep 4
  grep -h "^PUB" /tmp/zpub.out
  for i in $(seq $nsub); do grep -h "^SUB" /tmp/zsub_$i.out; done
  pkill -f "$B/zsub" 2>/dev/null; sleep 1
}

for nsub in 1 3; do
  run net 1 $nsub
  run shm 1 $nsub
  run shm 0 $nsub
  PORT=$((PORT + 1))
done
