#!/bin/bash
# Packet-queue-depth experiment on the ROS pipeline, under load (recorder + 2 cross-process cloud readers).
# Counts scans that are missing columns (= dropped LiDAR packets) for a sweep of packet queue depths
# (0 = leave the stack default, which is NOT 10), with Zenoh shared memory off (net) and on (shm).
# Prerequisites: scripts/build.sh, the Zenoh router, data/ filled. Takes ~20 min.
# Usage: run_flicker.sh [runs]     runs = space separated "depth:mode" (mode net|shm; depth a number, 0 = stack
#                                   default, or 'sensor' = the driver's original best-effort depth-5 QoS)
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DATA=${BENCH_DATA_DIR:-$ROOT/data}
PCAP=${PCAP:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245-000.pcap}
META=${META:-$DATA/OS-1-128_v3.0.1_2048x10_20230216_143245.json}
RUNS=${1:-"sensor:net 0:net 10:net 32:net 64:net 128:net 256:net 512:net sensor:shm 0:shm 10:shm 32:shm 512:shm"}   # depth:mode; 0 = stack default, sensor = driver default
export PYLON_CAMEMU=${PYLON_CAMEMU:-2}
source /opt/ros/jazzy/setup.bash; source "$ROOT/install/setup.bash"; source "$ROOT/scripts/zenoh_env.sh"
pkill -f "[c]omponent_container" 2>/dev/null
temp() { for z in /sys/class/thermal/thermal_zone*; do [ "$(cat $z/type 2>/dev/null)" = x86_pkg_temp ] && echo $(( $(cat $z/temp)/1000 )); done; }

for r in $RUNS; do
  depth=${r%%:*}; mode=${r##*:}
  zenoh_shm $mode
  if [ "$depth" = sensor ]; then QOS="system_default_qos:=false packet_qos_depth:=0"      # driver default: best-effort, depth 5
  else QOS="system_default_qos:=true packet_qos_depth:=$depth"; fi
  T=$(temp)
  ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=true $QOS \
      pcap:=$PCAP metadata:=$META bag_dir:=$ROOT/bags > /tmp/fl_launch.log 2>&1 &
  LP=$!; sleep 15
  python3 "$ROOT/benchmarks/pipeline/probe_ros.py" 30 cloud > /tmp/fl_p1.txt 2>&1 &
  P1=$!
  python3 "$ROOT/benchmarks/pipeline/probe_ros.py" 30 cloud > /tmp/fl_p2.txt 2>&1 &
  P2=$!
  wait $P1 $P2
  printf "depth=%-4s shm=%-3s %s\n" "$depth" "$mode" "$(grep -h '^CONSUMER' /tmp/fl_p1.txt | sed 's/^CONSUMER *//')"
  kill -INT $LP 2>/dev/null; sleep 6; pkill -9 -f "[c]omponent_container"; sleep 2
  rm -rf "$ROOT"/bags/benchmark_*; rmdir "$ROOT/bags" 2>/dev/null
  echo "    [pkg temp ${T}C -> $(temp)C]"
  sleep 30
done
