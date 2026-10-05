#!/bin/bash
# Build everything inside the ros2 container:
#   1. ROS packages (ouster_ros, pylon driver, sensor_benchmark, transport_bench) with colcon
#   2. the two pure-Zenoh benchmark programs with CMake (into build/bench/)
# Usage: scripts/build.sh [--skip-sources]     JOBS=<n> limits parallelism (default 6)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
JOBS=${JOBS:-6}
cd "$ROOT"

[ "${1:-}" = "--skip-sources" ] || "$ROOT/scripts/setup_sources.sh"

set +u; source /opt/ros/jazzy/setup.bash; set -u

echo "== colcon build (ROS packages)"
colcon build --symlink-install --parallel-workers "$JOBS" \
  --base-paths src benchmarks/micro/ros_rmw \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_PCAP=ON

echo "== cmake build (pure Zenoh micro-benchmark)"
cmake -S benchmarks/micro/zenoh_pure -B build/bench/zenoh_pure -DCMAKE_BUILD_TYPE=Release
cmake --build build/bench/zenoh_pure -j "$JOBS"

echo "== cmake build (pure Zenoh sensor pipeline: Ouster SDK + Pylon SDK)"
cmake -S benchmarks/pipeline/zenoh_sensors -B build/bench/zenoh_sensors -DCMAKE_BUILD_TYPE=Release
cmake --build build/bench/zenoh_sensors -j "$JOBS"

echo "== done. In new shells: source install/setup.bash"
