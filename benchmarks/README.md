# Benchmarks

All commands run **inside the `ros2` container** from the repo root, after `scripts/build.sh` (see the
[top-level README](../README.md)). Every benchmark that uses ROS needs the router: `docker compose up -d zenoh`.

| # | Question | Command | Time |
|---|---|---|---|
| 1 | What does plain Zenoh cost for a big message stream, with and without shared memory? | `benchmarks/micro/run_zenoh_pure.sh` | ~3 min |
| 2 | Same for ROS 2 (`rmw_zenoh`) | `benchmarks/micro/run_ros_rmw.sh` | ~2 min |
| 3 | Whole sensor pipeline: ROS drivers vs a pure-Zenoh app on the same SDKs | `benchmarks/pipeline/run_pipeline_compare.sh` | ~12 min |
| 4 | Does the LiDAR flicker (dropped packets) happen, and does queue depth fix it? | see [below](#4-reproduce-the-flicker-and-the-queue-depth-fix) | ~2 min per run |
| 5 | Where do the milliseconds go (stage-by-stage latency of the ROS pipeline)? | see [below](#5-latency-breakdown-stage-trace) | ~2 min per run |

Findings and the reasoning behind them: [../docs/transport-ipc-report.md](../docs/transport-ipc-report.md).

## 1. Pure Zenoh transport micro-benchmark

`micro/zenoh_pure/` is a publisher and subscriber written against zenoh-cpp (the same Zenoh version `rmw_zenoh`
uses). The publisher sends a payload of the given size at a fixed rate; the first 8 bytes are a timestamp.

```bash
benchmarks/micro/run_zenoh_pure.sh              # 12 MB @ 10 Hz (a LiDAR cloud)
benchmarks/micro/run_zenoh_pure.sh 0.0234375 1280   # ~24 KB @ 1280 Hz (a LiDAR packet stream)
```

For 1 and 3 subscriber processes it runs: `net` (shared memory off), `shm` with the subscriber **copying** the whole
payload (like a ROS reader), and `shm` reading the timestamp **in place** (what zero-copy costs).

Output: publisher CPU (% of one core), subscriber CPU, latency p50/p99/max, message count, and for each
subscriber how many payloads arrived as SHM (so you can see SHM really was used).

## 2. ROS 2 transport micro-benchmark

`micro/ros_rmw/` (package `transport_bench`) does the same with a `sensor_msgs/PointCloud2` of the same size,
reliable QoS depth 10, over `rmw_zenoh`.

```bash
benchmarks/micro/run_ros_rmw.sh [mb=12] [hz=10] [subscribers=1]
```

It runs SHM off, then on (via `scripts/zenoh_env.sh`), and prints whether the RMW can loan messages
(`can_loan PointCloud2=0 Float64(fixed-size)=0` on `rmw_zenoh` 0.2.10: no zero-copy loans).

## 3. Full pipeline comparison

`pipeline/zenoh_sensors/` is a ROS-free program built on the **Ouster SDK** (pcap replay, `ScanBatcher`, XYZ lookup
table, destagger, both returns, 48 B/point organized cloud) and the **Pylon SDK** (two emulated 1080p cameras at
10 Hz). It publishes everything over Zenoh from one process, mirroring the ROS container. `zconsumer` measures and
can record to disk (2 GiB raw files, closed files zstd-compressed, like rosbag2).

```bash
benchmarks/pipeline/run_pipeline_compare.sh                      # all four scenarios
benchmarks/pipeline/run_pipeline_compare.sh pure_shm ros_shm     # a subset
REC=false benchmarks/pipeline/run_pipeline_compare.sh ros_net ros_shm   # ROS without the recorder
```

Scenarios: `ros_net`, `ros_shm`, `pure_net`, `pure_shm` (SHM = Zenoh shared memory). The load is identical: two
cameras, the LiDAR with two returns, a recorder, and two cross-process cloud readers. Each scenario prints:

- **pipeline process CPU** and RSS (from `/proc`) over a 25 s window; the recorder's CPU separately for the pure
  version (in ROS it lives inside the container, which is why `REC=false` exists);
- per stream (`cloud`, `cam0`, `cam1`): rate, **stamp gaps** (healthy = all in 50-150 ms), **latency** p50/p99,
  and for the cloud **`cols_present<99.5%`**: how many scans were missing columns (healthy = `0/N`);
- the pure app's counters: scans, packets, `ring_drops`, `shm_alloc_fail` (all should be 0).

Reference result on one laptop (single runs, see the report for caveats):

| | ROS net | ROS SHM | Pure net | Pure SHM |
|---|---|---|---|---|
| Pipeline CPU without recorder (cores) | 0.60 | 0.44 | 0.62 | 0.21 |
| Total with recorder (cores) | 1.30 | 1.06 | 1.35 | 0.77 |
| Scan complete → cloud published (ms) | 31 | 22 | 30.5 | 3.6 |

**Do not compare the printed ROS `latency` columns with the pure-Zenoh ones.** The ROS driver stamps clouds about
one scan period (99 ms) early and cameras ~12 ms early (taken before the blocking grab), so the probe reports
~250 ms (cloud) and ~15 ms (camera) for ROS against ~113 ms and ~0.6 ms for the pure app. Use benchmark 5 for a
real stage breakdown. The pure app prints the stage time as `STAGE cloud build+publish`.

## 4. Reproduce the flicker and the queue-depth fix

The cloud flickered because LiDAR *packets* were dropped between the pcap node and the cloud node (queue depth
5-10 at ~1,150 packets/s). It only shows under load, so run the recorder and two readers:

```bash
# terminal 1: depth 10 = the old behaviour; use 512 (the default) for the fix
ros2 launch sensor_benchmark benchmark_drivers.launch.py packet_qos_depth:=10
# terminals 2 and 3, started together after ~15 s (two cross-process readers)
python3 benchmarks/pipeline/probe_ros.py 30 all
python3 benchmarks/pipeline/probe_ros.py 30 cloud
```

Look at the `cloud` line: with depth 10 expect a handful of `cols_present<99.5%` scans out of ~270 and some
stamp gaps under 50 ms or over 150 ms; with 512 expect `0/N` and all gaps in 50-150 ms. Add shared memory with
`source scripts/zenoh_env.sh shm` in **each** terminal before starting (it reduces but does not remove the drops
at depth 10). Stop the launch with Ctrl-C so the recorder writes its metadata; bags go to `bags/` (ignored by git,
about 2.4 GB per minute compressed, delete them afterwards).

## 5. Latency breakdown (stage trace)

Needs the instrumented drivers, which `scripts/setup_sources.sh` applies (`patches/latency-trace-*.patch`; the
trace is off unless `BENCH_TRACE=1`). Start the ROS pipeline with tracing and keep its output:

```bash
BENCH_TRACE=1 ros2 launch sensor_benchmark benchmark_drivers.launch.py record:=false 2>&1 | tee /tmp/trace.log
# after ~40 s, stop with Ctrl-C and analyse:
benchmarks/pipeline/latency_trace.py /tmp/trace.log
```

For the loaded case use `record:=true` and start two `probe_ros.py 30 cloud` readers (see 4), optionally after
`source scripts/zenoh_env.sh shm`. The script prints median / p95 / max per stage:

- **Cloud**: `stamp_offset` (message stamp vs the real first packet; expect ~99 ms early), `assembly` (the sensor
  sweep, ~110 ms), `queue`, `process` (building the clouds), `publish`, `consume`, and the totals
  `since_stamp` (what the probes report), `since_first_pkt`, `since_complete` (what a consumer waits after the scan
  is done).
- **Camera** (per camera): `wait` (stamp before the blocking grab → frame retrieved), `copy`, `rest`, `consume`.
- The `hop` row (pcap node → cloud node) is **not reliable**: the cloud node's first-packet time sits one packet
  (0.8 ms) before the pcap node's, so the join matches the previous scan. Ignore it.
- `consume` is negative under load because the in-process consumer receives the message during `publish()`.

For the pure-Zenoh side the matching number is the `STAGE` line printed by `run_pipeline_compare.sh pure_net
pure_shm` (dequeue → both returns built and published).

## How the numbers are measured

- **Completeness**: share of scan columns that contain at least one finite point. A lost LiDAR packet removes
  16 columns.
- **Stamp gaps**: time between consecutive header stamps. A 9 Hz LiDAR should give ~110 ms every time.
- **Latency (probes)**: time the consumer's callback starts minus the message's stamp (same host clock). For the
  pure app the stamp is the first packet of the scan (~110 ms of the latency is the sensor sweep) or the grab time
  of the image. ROS stamps differ (see the warning in 3), so ROS and pure-Zenoh probe latencies are not comparable.
- **CPU / RSS**: `utime+stime` and `VmRSS` of the process, sampled from `/proc` over the window.

## Fairness notes and limits

- The pure-Zenoh programs move opaque bytes and implement only what the benchmark needs (no IMU, TF, parameters,
  lifecycle or diagnostics). The ROS side does all of that. A real pure-Zenoh product also needs a wire format,
  which is not measured here.
- Zenoh turns SHM **on** by default; `rmw_zenoh` (Jazzy) turns it **off**. The `net` scenarios switch it off
  explicitly on both ends, the `shm` ones on explicitly.
- ROS latencies come from a single-threaded Python probe, the pure side from a C++ consumer.
- One run per scenario on a laptop whose temperature rises during the run (the scripts print it). Treat small
  differences as noise and repeat runs.
- `/tmp/*.txt` files hold the raw output of the last run.

## Adding a benchmark

Put a program under `benchmarks/<kind>/`, build it from `scripts/build.sh`, add a `run_*.sh` that derives the repo
root from its own path (as the existing ones do), and list it in the table at the top of this file.
