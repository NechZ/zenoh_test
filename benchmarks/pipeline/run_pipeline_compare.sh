#!/bin/bash
# End-to-end comparison: the ROS 2 pipeline (ouster_ros + pylon driver + rosbag2 recorder over rmw_zenoh)
# vs a pure Eclipse Zenoh app built on the same Ouster and Pylon SDKs. Run inside the ros2 container.
#
# Prerequisites: scripts/build.sh, the Zenoh router (docker compose up -d zenoh), data/ filled (data/README.md).
#
# Usage: run_pipeline_compare.sh [scenario ...]
#   scenarios: ros_net ros_shm pure_net pure_shm     (default: all four)
# Environment:
#   REC=false   run the ROS scenarios without the rosbag2 recorder (to separate the recorder's cost)
#   WINDOW=25   measurement window in seconds
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
B=${B:-$ROOT/build/bench/zenoh_sensors}
DATA=${BENCH_DATA_DIR:-$ROOT/data}
PCAP=${PCAP:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245-000.pcap}
META=${META:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245.json}
HERE="$ROOT/benchmarks/pipeline"
WINDOW=${WINDOW:-25}
export PYLON_CAMEMU=${PYLON_CAMEMU:-2}

for f in "$B/zenoh_sensors" "$PCAP" "$META" "$ROOT/install/setup.bash"; do
  [ -e "$f" ] || { echo "missing: $f  (run scripts/build.sh and see data/README.md)"; exit 1; }
done
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/setup.bash"
source "$ROOT/scripts/zenoh_env.sh"

ticks() { awk '{ sub(/^.*\) /,""); print $12+$13 }' /proc/$1/stat; }
rss_mb() { awk '/VmRSS/{printf "%.0f", $2/1024}' /proc/$1/status; }
temp() { for z in /sys/class/thermal/thermal_zone*; do [ "$(cat $z/type 2>/dev/null)" = x86_pkg_temp ] && echo $(( $(cat $z/temp)/1000 )); done; }
HZ=$(getconf CLK_TCK)

sample_start() { S_T0=$(date +%s.%N); for p in "$@"; do eval "T0_$p=$(ticks $p)"; done; }
sample_end() {  # prints "pid N: X.XX cores of CPU, RSS N MB" per pid
  local e=$(date +%s.%N)
  for p in "$@"; do
    local t0; eval "t0=\$T0_$p"
    awk -v a=$t0 -v b=$(ticks $p) -v hz=$HZ -v s=$S_T0 -v e=$e -v r=$(rss_mb $p) -v p=$p \
      'BEGIN{printf "  pid %s: %.2f cores of CPU, RSS %d MB\n", p, (b-a)/hz/(e-s), r}'
  done
}

run_ros() {  # $1 = net|shm
  echo "=== ROS 2 rmw_zenoh, SHM $([ $1 = shm ] && echo on || echo off), recorder ${REC:-true}: ouster_ros + pylon + consumer node in one container"
  zenoh_shm $1
  local T=$(temp)
  ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=${REC:-true} pcap:=$PCAP metadata:=$META \
      bag_dir:=$ROOT/bags > /tmp/cmp_ros.log 2>&1 &
  local LP=$!; sleep 15
  local PID=$(pgrep -f "[l]ib/rclcpp_components/component_container" | head -1)
  python3 $HERE/probe_ros.py $WINDOW all > /tmp/cmp_p1.txt 2>&1 &
  local P1=$!
  python3 $HERE/probe_ros.py $WINDOW cloud > /tmp/cmp_p2.txt 2>&1 &
  local P2=$!
  sample_start $PID; sleep $WINDOW; echo "pipeline process (all nodes + recorder if enabled):"; sample_end $PID
  wait $P1 $P2; sleep 2   # (job specs like %2 do not work in a non-interactive shell)
  grep -h "^CONSUMER" /tmp/cmp_p1.txt
  # background jobs of a non-interactive shell ignore SIGINT, so stop the container directly
  kill -INT $LP 2>/dev/null; sleep 8; pkill -9 -f "[c]omponent_container"; sleep 2
  rm -rf "$ROOT"/bags/benchmark_*; rmdir "$ROOT/bags" 2>/dev/null
  echo "  [pkg temp ${T}C -> $(temp)C]"
}

run_pure() {  # $1 = net|shm
  local m=$([ $1 = shm ] && echo 1 || echo 0)
  echo "=== Pure Zenoh, SHM $([ $m = 1 ] && echo on || echo off): Ouster SDK + Pylon SDK in one process, recorder + 2 consumers separate"
  local T=$(temp) REC_DIR=/tmp/cmp_rec; rm -rf $REC_DIR
  $B/zenoh_sensors --pcap $PCAP --meta $META --shm $m --cams 2 --fps 10 --secs 40 --warmup 8 > /tmp/cmp_sensors.txt 2>&1 &
  local SP=$!; sleep 2
  $B/zconsumer --shm $m --secs 44 --check 1 > /tmp/cmp_c1.txt 2>&1 &
  $B/zconsumer --shm $m --secs 44 --check 1 > /tmp/cmp_c2.txt 2>&1 &
  $B/zconsumer --shm $m --secs 44 --check 0 --record $REC_DIR > /tmp/cmp_rec.txt 2>&1 &
  local RP=$!
  sleep 14
  sample_start $SP $RP; sleep $WINDOW
  echo "pipeline process (Ouster + Pylon + publishing):"; sample_end $SP
  echo "recorder process (zstd file compression):"; sample_end $RP
  wait $SP; sleep 5
  grep -h "^PIPELINE\|^STAGE\|\[cam" /tmp/cmp_sensors.txt
  grep -h "^CONSUMER" /tmp/cmp_c1.txt | grep -v "cloud2"
  grep -h "recorder wrote" /tmp/cmp_rec.txt | sed 's/^/  /'
  pkill -f "zconsumer"; sleep 1
  rm -rf $REC_DIR
  echo "  [pkg temp ${T}C -> $(temp)C]"
}

pkill -f "[c]omponent_container" 2>/dev/null
for s in ${@:-ros_net ros_shm pure_net pure_shm}; do
  case $s in
    ros_net) run_ros net ;; ros_shm) run_ros shm ;;
    pure_net) run_pure net ;; pure_shm) run_pure shm ;;
    *) echo "unknown scenario: $s"; exit 1 ;;
  esac
  sleep 30  # cool-down between scenarios
done
