#!/bin/bash
# Stage-by-stage latency of the ROS pipeline (needs the instrumented drivers, see patches/README.md).
# Three scenarios: idle (no cross-process readers), and loaded (recorder + 2 readers) with SHM off and on.
# Prerequisites: scripts/build.sh, the Zenoh router, data/ filled. Takes ~5 min.
# Usage: run_latency_trace.sh [scenario ...]    scenarios: idle load_net load_shm
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA=${BENCH_DATA_DIR:-$ROOT/data}
PCAP=${PCAP:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245-000.pcap}
META=${META:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245.json}
export PYLON_CAMEMU=${PYLON_CAMEMU:-2} BENCH_TRACE=1
source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash"; source "$ROOT/scripts/zenoh_env.sh"
pkill -f "[c]omponent_container" 2>/dev/null

run() {  # name record shm probes
  local name=$1 rec=$2 mode=$3 probes=$4
  zenoh_shm $mode
  echo "################ $name  (recorder=$rec, SHM $mode, cross-process readers=$probes)"
  ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=$rec pcap:=$PCAP metadata:=$META \
      bag_dir:=$ROOT/bags > /tmp/lt_$name.log 2>&1 &
  local LP=$!; sleep 15
  if [ "$probes" = 2 ]; then
    python3 "$ROOT/benchmarks/pipeline/probe_ros.py" 30 cloud > /dev/null 2>&1 &
    local P1=$!
    python3 "$ROOT/benchmarks/pipeline/probe_ros.py" 30 cloud > /dev/null 2>&1 &
    wait $P1 $!
  else
    sleep 30
  fi
  kill -INT $LP 2>/dev/null; sleep 6; pkill -9 -f "[c]omponent_container"; sleep 2
  rm -rf "$ROOT"/bags/benchmark_*; rmdir "$ROOT/bags" 2>/dev/null
  python3 "$ROOT/benchmarks/pipeline/latency_trace.py" /tmp/lt_$name.log | grep -v "hop "
  sleep 30
}

for s in ${@:-idle load_net load_shm}; do
  case $s in
    idle) run idle false net 0 ;;
    load_net) run load_net true net 2 ;;
    load_shm) run load_shm true shm 2 ;;
    *) echo "unknown scenario: $s"; exit 1 ;;
  esac
done
