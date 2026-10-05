# Raw measurements behind the report

Output of `benchmarks/run_all.sh` plus repeats, 2026-10-05, one laptop. Each file is the unedited script output.

| File | Report table |
|---|---|
| `micro_zenoh_cloud.txt`, `micro_zenoh_packets.txt` | §4.2 (plain Zenoh, 12 MB @ 10 Hz and ~30 KB @ 1,150 Hz) |
| `micro_ros_cloud_1sub.txt`, `micro_ros_cloud_3sub.txt`, `micro_ros_packets.txt`, `micro_ros_{1,3}sub_r{1,2}.txt` | §4.2 (ROS; the `_r1`/`_r2` files are repeats) |
| `pipeline_with_recorder.txt`, `pipeline_ros_no_recorder.txt`, `pure_net_repeat{1,2}.txt` | §4.3 |
| `queue_depth.txt`, `queue_depth_sweep.txt`, `qos_default_vs_sensor.txt` | §4.1 (`depth=sensor` is the driver's best-effort depth-5 QoS, `depth=0` the stack default) |
| `latency_trace.txt` | §4.4 |

`cols_present<99.5%: X/N` counts scans missing columns. Times in `latency_trace.txt` are milliseconds. A
`Segmentation fault` line from `probe_ros.py` at the end of a queue-depth run is the Python reader crashing during
interpreter shutdown after it printed its result; it does not affect the numbers.
