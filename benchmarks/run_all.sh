#!/bin/bash
# Run every benchmark and keep the raw output in results/<timestamp>/ (ignored by git). About 35-40 minutes, and
# the CPU is busy the whole time: run it on an otherwise idle machine and watch the temperature it logs.
# Prerequisites: scripts/build.sh, the Zenoh router (docker compose up -d zenoh), data/ filled.
# Usage: benchmarks/run_all.sh [output_dir]       (inside the ros2 container)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT=${1:-$ROOT/results/$(date +%Y%m%d_%H%M%S)}
mkdir -p "$OUT"
step() { local name=$1; shift; echo "[$(date +%H:%M:%S)] $name"; "$@" 2>&1 | grep --line-buffered -v "WARN" > "$OUT/$name.txt"; }

# packets: ~30 KB at 1150 Hz stands in for the LiDAR packet stream (a real packet is 33 KB)
step micro_zenoh_cloud     "$ROOT/benchmarks/micro/run_zenoh_pure.sh" 12 10
step micro_zenoh_packets   "$ROOT/benchmarks/micro/run_zenoh_pure.sh" 0.029296875 1150
step micro_ros_cloud_1sub  "$ROOT/benchmarks/micro/run_ros_rmw.sh" 12 10 1
step micro_ros_cloud_3sub  "$ROOT/benchmarks/micro/run_ros_rmw.sh" 12 10 3
step micro_ros_packets     "$ROOT/benchmarks/micro/run_ros_rmw.sh" 0.029296875 1150 1
step pipeline_with_recorder "$ROOT/benchmarks/pipeline/run_pipeline_compare.sh"
REC=false step pipeline_ros_no_recorder "$ROOT/benchmarks/pipeline/run_pipeline_compare.sh" ros_net ros_shm
step queue_depth           "$ROOT/benchmarks/pipeline/run_flicker.sh"
step latency_trace         "$ROOT/benchmarks/pipeline/run_latency_trace.sh"
echo "[$(date +%H:%M:%S)] done, results in $OUT"
