# Benchmarks

All commands run **inside the `ros2` container** from the repo root, after `scripts/build.sh` (see the
[top-level README](../README.md)). Every benchmark that uses ROS needs the router: `docker compose up -d zenoh`.
Results and the reasoning behind them: [../docs/transport-ipc-report.md](../docs/transport-ipc-report.md); the raw
output of the reference run is in [../docs/measurements/](../docs/measurements/).

| # | Question | Command | Time |
|---|---|---|---|
| 1 | What does plain Zenoh cost for a big message stream, with and without shared memory? | `benchmarks/micro/run_zenoh_pure.sh` | ~2 min |
| 2 | Same for ROS 2 (`rmw_zenoh`) | `benchmarks/micro/run_ros_rmw.sh` | ~1 min |
| 3 | Whole sensor pipeline: ROS drivers vs a pure-Zenoh app on the same SDKs | `benchmarks/pipeline/run_pipeline_compare.sh` | ~6 min |
| 4 | Are LiDAR packets dropped, and which queue depth fixes it? | `benchmarks/pipeline/run_flicker.sh` | ~20 min |
| 5 | Where do the milliseconds go (stage-by-stage latency of the ROS pipeline)? | `benchmarks/pipeline/run_latency_trace.sh` | ~4 min |
| all | Everything, output kept in `results/<timestamp>/` | `benchmarks/run_all.sh` | ~35-40 min |

`run_all.sh` keeps every script's raw output as text files in `results/<timestamp>/` (ignored by git). It keeps the
CPU busy the whole time: run it on an otherwise idle machine and watch the temperature the scripts log.

## 1. Pure Zenoh transport micro-benchmark

`micro/zenoh_pure/` is a publisher and subscriber written against zenoh-cpp (the same Zenoh version `rmw_zenoh`
uses). The publisher sends a payload of the given size at a fixed rate; the first 8 bytes are a timestamp.

```bash
benchmarks/micro/run_zenoh_pure.sh                          # 12 MB @ 10 Hz (a LiDAR cloud)
benchmarks/micro/run_zenoh_pure.sh 0.029296875 1150         # ~30 KB @ 1150 Hz (a LiDAR packet stream)
```

For 1 and 3 subscriber processes it runs: `net` (shared memory off), `shm` with the subscriber **copying** the whole
payload (like a ROS reader), and `shm` reading the timestamp **in place** (what zero-copy costs). Output: publisher
CPU (% of one core), subscriber CPU, latency p50/p99/max, message count, and per subscriber how many payloads arrived
as SHM (so you can see SHM really was used).

## 2. ROS 2 transport micro-benchmark

`micro/ros_rmw/` (package `transport_bench`) does the same with a `sensor_msgs/PointCloud2` of the same size,
reliable QoS depth 10, over `rmw_zenoh`.

```bash
benchmarks/micro/run_ros_rmw.sh [mb=12] [hz=10] [subscribers=1]
benchmarks/micro/run_ros_rmw.sh 0.029296875 1150 1          # packet-sized messages
```

It runs SHM off, then on (via `scripts/zenoh_env.sh`), and prints whether the RMW can loan messages
(`can_loan PointCloud2=0 Float64(fixed-size)=0` on `rmw_zenoh` 0.2.10: no zero-copy loans).

## 3. Full pipeline comparison

`pipeline/zenoh_sensors/` is a ROS-free program built on the **Ouster SDK** (pcap replay, `ScanBatcher`, XYZ lookup
table, destagger, both returns, 48 B/point organized cloud) and the **Pylon SDK** (two emulated 1080p cameras at
10 Hz). It publishes everything over Zenoh from one process, mirroring the ROS container. `zconsumer` measures and
can record to disk (2 GiB raw files, closed files zstd-compressed, like rosbag2).

```bash
benchmarks/pipeline/run_pipeline_compare.sh                              # all four scenarios
benchmarks/pipeline/run_pipeline_compare.sh pure_shm ros_shm             # a subset
REC=false benchmarks/pipeline/run_pipeline_compare.sh ros_net ros_shm    # ROS without the recorder
```

Scenarios: `ros_net`, `ros_shm`, `pure_net`, `pure_shm` (SHM = Zenoh shared memory). The load is identical: two
cameras, the LiDAR with two returns, a recorder, and two cross-process cloud readers. Each scenario prints:

- **pipeline process CPU** and RSS (from `/proc`) over a 25 s window; the recorder's CPU separately for the pure
  version (in ROS it lives inside the container, which is why `REC=false` exists);
- per stream (`cloud`, `cam0`, `cam1`): rate, **stamp gaps** (healthy = all in 50-150 ms), **latency** p50/p99,
  and for the cloud **`cols_present<99.5%`**: how many scans were missing columns (healthy = `0/N`);
- the pure app's counters: scans, packets, `ring_drops`, `shm_alloc_fail` (all should be 0) and
  `STAGE cloud build+publish`, the time from dequeue to all returns built and published.

Reference result (one laptop; ranges where repeats were made; full tables and caveats in the report):

| | ROS net | ROS SHM | Pure net | Pure SHM |
|---|---|---|---|---|
| Pipeline CPU without recorder (cores) | 0.58 | 0.42 | 0.58-0.68 | 0.19-0.21 |
| Total with recorder (cores) | 1.25 | 1.08 | 1.2-1.35 | 0.71 |
| Scan complete → cloud published (ms) | 29.5 | 20.9 | 27-31 | 3.6 |

The ROS camera stamp is taken before the blocking grab (~12 ms before the frame), so the probe's ROS camera latency
(~14-18 ms) is not comparable with the pure app's (stamped after the grab); the stage trace (5) gives the comparable
number. The ROS cloud stamp is correct since `patches/ouster-ros-stamp-fix.patch`, which `scripts/setup_sources.sh`
applies by default.

## 4. Packet queue depth under load

LiDAR *packets* travel from the pcap node to the cloud node over a topic at ~1,150 messages/s; a dropped packet
removes 16 scan columns, so the cloud flickers. This only shows under load, so the script runs the full pipeline with
the recorder and two cross-process readers for each setting and counts scans with missing columns.

```bash
benchmarks/pipeline/run_flicker.sh                                   # sweep: sensor 0 10 32 64 128 256 512, SHM off and on
benchmarks/pipeline/run_flicker.sh "10:net 512:net 512:shm"          # your own list of depth:mode
```

`depth` is a number (`0` = the stack default, which is **not** 10), or `sensor` = the driver's original best-effort
depth-5 QoS (`system_default_qos:=false`); `mode` is `net` or `shm`. Look at `cols_present<99.5%: X/N`: `0/N` is
healthy. By hand: `ros2 launch sensor_benchmark benchmark_drivers.launch.py packet_qos_depth:=10` plus two
`benchmarks/pipeline/probe_ros.py 30 cloud` readers started together after ~15 s.

## 5. Latency breakdown (stage trace)

Needs the instrumented drivers, which `scripts/setup_sources.sh` applies (`patches/latency-trace-*.patch`; the trace
is off unless `BENCH_TRACE=1`).

```bash
benchmarks/pipeline/run_latency_trace.sh                    # idle, loaded SHM off, loaded SHM on
benchmarks/pipeline/run_latency_trace.sh idle               # one scenario
```

By hand: `BENCH_TRACE=1 ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=false 2>&1 | tee /tmp/trace.log`,
Ctrl-C after ~40 s, then `benchmarks/pipeline/latency_trace.py /tmp/trace.log`. The script prints median / p95 / max
per stage:

- **Cloud**: `stamp_offset` (message stamp vs the real first packet; about −0.8 ms), `assembly` (the sensor sweep,
  ~110 ms), `queue`, `process` (building the clouds), `publish`, `consume`, and the totals `since_stamp`,
  `since_first_pkt`, `since_complete` (what a consumer waits after the scan is done).
- **Camera** (per camera): `wait` (stamp before the blocking grab → frame retrieved), `copy`, `rest`, `consume`.
- Ignore the `hop` row (pcap node → cloud node): the cloud node's first-packet time sits one packet (0.8 ms) before
  the pcap node's, so the join matches the previous scan; `run_latency_trace.sh` hides it.
- `consume` is negative under load because the in-process consumer receives the message during `publish()`.

Try other Ouster timestamp modes with `timestamp_mode:=TIME_FROM_INTERNAL_OSC` or `TIME_FROM_PTP_1588`. In a replay the
stamps come from the recorded sensor clock, so `stamp_offset` is a huge number there; only a live sensor with
synchronised clocks gives a meaningful absolute value.

## How the numbers are measured

- **Completeness**: share of scan columns that contain at least one finite point. A lost LiDAR packet removes 16
  columns.
- **Stamp gaps**: time between consecutive header stamps. A 9 Hz LiDAR should give ~110 ms every time.
- **Latency (probes)**: time the consumer's callback starts minus the message's stamp (same host clock). For the pure
  app the stamp is the first packet of the scan (~110 ms of the latency is the sensor sweep) or the grab time of the
  image.
- **CPU / RSS**: `utime+stime` and `VmRSS` of the process, sampled from `/proc` over the window.

## Fairness notes and limits

- The pure-Zenoh programs move opaque bytes and implement only what the benchmark needs (no IMU, TF, parameters,
  lifecycle or diagnostics). The ROS side does all of that. A real pure-Zenoh product also needs a wire format, which
  is not measured here.
- Zenoh turns SHM **on** by default; `rmw_zenoh` (Jazzy) turns it **off**. The `net` scenarios switch it off
  explicitly on both ends, the `shm` ones on explicitly.
- The pcap replay runs at about 91% of real time (scans every ~109.5 ms, ~9.1 Hz) for the ROS node and the pure app
  alike.
- ROS latencies come from a single-threaded Python probe, the pure side from a C++ consumer.
- Runs are mostly single per scenario on a laptop whose temperature rises during the run (the scripts print it).
  Treat small differences as noise and repeat runs that decide a conclusion.
- `/tmp/*.txt` files hold the raw output of the last run of each script.

## Adding a benchmark

Put a program under `benchmarks/<kind>/`, build it from `scripts/build.sh`, add a `run_*.sh` that derives the repo
root from its own path (as the existing ones do), add it to `run_all.sh`, and list it in the table at the top of this
file. See [../AGENTS.md](../AGENTS.md) for the testing method.
