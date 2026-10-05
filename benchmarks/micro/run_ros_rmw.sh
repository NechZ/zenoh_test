#!/bin/bash
# ROS 2 (rmw_zenoh) transport micro-benchmark with the same payload as run_zenoh_pure.sh.
# One publisher process, N subscriber processes, SHM off then on. Needs the rmw_zenoh router
# (docker compose up -d zenoh) and a built workspace (scripts/build.sh).
# Usage: run_ros_rmw.sh [mb=12] [hz=10] [nsub=1]
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MB=${1:-12}; HZ=${2:-10}; NSUB=${3:-1}
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
source "$ROOT/scripts/zenoh_env.sh"
for mode in net shm; do
  zenoh_shm $mode
  echo "=== ROS rmw_zenoh, SHM $([ $mode = shm ] && echo on || echo off), $NSUB subscriber process(es), ${MB} MB @ ${HZ} Hz"
  for i in $(seq $NSUB); do ros2 run transport_bench sub --ros-args -p secs:=15.0 > /tmp/rsub_$i.txt 2>&1 & done
  sleep 4
  ros2 run transport_bench pub --ros-args -p mb:=$(printf "%.7f" $MB) -p hz:=$(printf "%.1f" $HZ) -p secs:=10.0 > /tmp/rpub.txt 2>&1 &
  PP=$!; wait $PP; sleep 6
  grep -h "PUB" /tmp/rpub.txt
  grep -h "can_loan" /tmp/rpub.txt | sed 's/.*\] //'
  for i in $(seq $NSUB); do grep -h -o "SUB n=.*" /tmp/rsub_$i.txt; done
  pkill -f "lib/transport_bench/sub"; sleep 2
done
