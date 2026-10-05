# AGENTS.md: hand-off for people and agents working on this repo

Read this first. It says what the project is, how it is laid out, what is known about the stack, how tests are run and
how results are reported, and which mistakes were already made so they are not repeated. Numbers live in
`docs/transport-ipc-report.md`; this file holds the knowledge and the method.

## 1. What this project is

A benchmark suite and write-up on moving high-bandwidth sensor data (Ouster LiDAR cloud, Basler camera images) for an
autonomous racing car, where **milliseconds on the critical path matter**. It compares ROS 2 (`rmw_zenoh`, composable
nodes, intra-process comms, shared memory) with plain Eclipse Zenoh written directly on the Ouster and Pylon SDKs.
No hardware is needed: the LiDAR is a pcap replay, the cameras are Basler's emulator.

Deliverables: `docs/transport-ipc-report.md` (the result, for the team wiki), `benchmarks/` (how to reproduce),
`patches/` (our changes to the two driver repos), `scripts/` and `docker/` (reproducible environment).

## 2. Layout and commands

```
docker/Dockerfile, docker-compose.yml   environment: ros2 (work container), zenoh (router), foxglove (optional viewer)
scripts/setup_sources.sh                clone the driver repos at pinned commits and apply patches/ (idempotent)
scripts/build.sh                        colcon build + the two pure-Zenoh programs (build/bench/...)
scripts/zenoh_env.sh                    `source scripts/zenoh_env.sh shm|net` turns Zenoh shared memory on/off
src/sensor_benchmark/                   launch file (benchmark_drivers.launch.py) + in-process consumer
src/ouster-ros, src/pylon-ros-camera/   clones of the team GitLab repos (gitignored; changes live in patches/)
patches/                                base patches, latency-trace patches, stamp fix (applied in that order)
benchmarks/micro/                       transport micro-benchmarks: pure Zenoh (zenoh_pure/), ROS (ros_rmw/)
benchmarks/pipeline/                    zenoh_sensors (ROS-free app), comparison, queue-depth, latency trace, probes
benchmarks/run_all.sh                   everything, results in results/<timestamp>/ (gitignored), ~35-40 min
docs/measurements/                      raw output of the reference run behind every number in the report
```

Typical session (inside the ros2 container, repo mounted at `/workspaces/zenoh_test`):

```bash
docker compose build ros2 && docker compose up -d zenoh ros2 && docker compose exec ros2 bash
scripts/build.sh && source install/setup.bash
benchmarks/run_all.sh            # or one script from benchmarks/README.md
```

Not in git and needed: pylon installers (`third_party/pylon/`), the pcap + matching metadata (`data/`, see
`data/README.md`), ssh access to the team GitLab for `scripts/setup_sources.sh`.

## 3. Environment facts and traps

- **Everything runs as root in the container**, so files it creates in the bind-mounted repo (`build/`, `install/`,
  `results/`, bags) are root-owned. Edit repo files from the host; delete root-owned files through the container.
- **Zenoh SHM defaults differ.** `rmw_zenoh` (Jazzy 0.2.10) ships with SHM **off**
  (`DEFAULT_RMW_ZENOH_SESSION_CONFIG.json5`, "disabled by default until fully tested"). Plain Zenoh has SHM **on**
  by default. Always set it explicitly on every process of a run (`scripts/zenoh_env.sh`, or
  `transport/shared_memory/enabled=false` in the pure programs) and confirm it really engaged: the pure consumer
  prints `(arrived as SHM: N)`, and the ROS container's RSS jumps by ~1 GB with SHM on (pool pages).
- **The router must be running** for every ROS benchmark (`docker compose up -d zenoh`). Pure-Zenoh programs connect
  to each other directly on `tcp/127.0.0.1:<port>` and need no router.
- **Pylon emulator**: `PYLON_CAMEMU=2` gives two cameras (serials `0815-0000`, `0815-0001`); both pylon enumeration
  and open must not run concurrently from two threads (the pure app serialises it with a mutex; it raced once).
- **The pcap replay runs at ~91% of real time** (a scan every ~109.5 ms, ~9.1 Hz, ~1,150 packets/s, packet size
  33,024 B) because the per-packet `sleep` overshoots. Both stacks copy this pacing, so comparisons stay fair, but
  absolute rates are not 10 Hz. Looping the pcap needs the SDK patch in `patches/ouster-sdk.patch`.
- **The laptop throttles.** Runs push the CPU to 70-83 °C. Scripts log the package temperature and cool down between
  scenarios; an invalid run (throttling) must be discarded and repeated, not averaged in.
- **Shell traps in the scripts** (all already hit once):
  - job specs like `wait %2` do not work in a non-interactive shell; use PIDs (`wait $P1 $P2`);
  - background processes in a non-interactive shell ignore SIGINT, so `kill -INT` does not stop `ros2 launch`; the
    scripts send SIGINT and then `pkill -9 -f "[c]omponent_container"`;
  - `pkill -f <pattern>` kills your own shell when the pattern appears in its command line; use the `[c]omponent`
    bracket trick;
  - piping through `grep -v` without `--line-buffered` hides progress until the end;
  - Write-tool/IDE edits fail in root-owned directories; `chown -R 1000:1000` fixes it.
- **Docker**: the build context excludes the 4.4 GB pcap and everything except `docker/` and `third_party/pylon/`
  (`.dockerignore`); the pylon `.deb` is bind-mounted during the build so it is not copied into a layer.
  `git config --system safe.directory '*'` is set in the image because the clones are owned by another uid.

## 4. What is known about the stack

Tags: **measured** = seen in our runs (numbers in the report); **code** = read in the source, not tested.

**Queues and drops.** Data is lost at the smallest queue, not at the largest message. The pcap node → cloud node
LiDAR *packet* topic (~1,150 msg/s) overflows when the receiver stalls: each lost packet removes 16 scan columns
(measured: scans with missing columns, bursty frame stamps). The original config (best-effort sensor-data QoS,
depth 5) loses packets constantly; an explicit depth of 10 still loses most scans under load; the stack default
(`packet_qos_depth:=0`) is borderline; 32 and above was clean in every run. Our launch uses 512 (~0.4 s). The
`packet_qos_depth` parameter (our patch) applies to the packet topic only. The live `os_driver` node has no packet
topic, so none of this applies to the car.

**Shared memory.** Works with ordinary variable-size messages (PointCloud2, Image); it does not require fixed-size
types. It is not end-to-end zero-copy in ROS (the publisher serialises into the SHM buffer, readers deserialise).
Loaned messages are not available in `rmw_zenoh` 0.2.10: `can_loan_messages()` is false even for `Float64`
(measured). SHM makes publisher CPU independent of the number of readers; it gives nothing for ~30 KB messages.
Pure Zenoh with a reader that only looks at the shared buffer is the true zero-copy floor.

**Zenoh congestion control.** The pure app publishes with `Z_CONGESTION_CONTROL_BLOCK` (comparable to reliable
QoS). In one run without SHM a slow reader (the recorder, writing ~145 MB/s through zstd) blocked `put()` for ~5 s,
the ring buffer overflowed and 1,269 packets were dropped; three repeats were clean. For a sensor path prefer DROP
or make sure readers keep up.

**Ouster driver internals** (`src/ouster-ros/ouster-ros/src`):
- `LidarPacketHandler` batches packets into `LidarScan`s (`ScanBatcher`), a 10-slot ring buffer hands scans to a
  worker thread which runs the processors (`PointCloudProcessor`: XYZ lookup table, PCL cloud, convert to the ROS
  message, per return). Both `os_cloud` (replay) and `os_driver` (live) use this same handler.
- **Stamp bug, fixed in `patches/ouster-ros-stamp-fix.patch`**: with `TIME_FROM_ROS_TIME`, `lidar_handler_ros_time`
  derived the next cloud's stamp from the packet that completes a scan. With this SDK a scan completes on its *last*
  packet, so every cloud was stamped one scan period early (99.3 ms = 2032 columns × 48.8 µs at 2048x10; in general
  about (1 − packet_columns/W) × scan period). Verified in replay: +99.25 ms → −0.84 ms. The sensor-time and PTP
  modes take stamps from the sensor columns (**code**), so the car (PTP) should not be affected; **verify once on the
  car** with `latency_trace.py` (offset should be a few ms, not ~100).
- Cloud build is 13-15 ms for two returns and `publish()` to cross-process readers costs 7 ms (SHM) to 15 ms (no
  SHM) on the same thread. The pure app builds the same output in ~3 ms. Suspected cause (**not profiled**): a PCL
  cloud plus a staging `PCLPointCloud2` and a conversion instead of writing the output once. See future work.
- The `os_driver` publish lambda is not instrumented by the trace patch; only `os_cloud` is.

**Pylon driver** (`src/pylon-ros-camera`): the image stamp is taken *before* the blocking `grab()`, so it precedes the
frame by the grab wait (~12 ms with the emulator at 10 fps). From frame available to an in-process consumer it costs
~0.5 ms. Real cameras (chunk timestamps, triggered vs free-running) change what the stamp means.

**Consumers**: a reader in the same container gets the message through intra-process comms *during* `publish()`
(before it returns), so in-process numbers hide the serialisation cost that cross-process readers (recorder, viewer,
perception in another process) pay.

## 5. Methodology: testing

Principles that made the results trustworthy (and each came from a mistake):

1. **Measure completeness, not just rate.** A stream can run at the right Hz and still be broken. The metric is the
   share of scan columns with finite points per scan, plus gaps between header stamps. Healthy = 100% columns and
   steady ~110 ms gaps. Never accept "it ran at 9 Hz".
2. **Test under the load you will have**: recorder on, two cross-process readers. Idle runs hid the drops.
3. **One variable at a time, ABBA order, repeats.** Interleave A/B/B/A to cancel thermal drift; repeat anything that
   decides a conclusion; report the range where repeats disagree (the ROS publisher CPU without SHM varied 15-24%).
4. **Verify the setting took effect**, not that you passed it (SHM actually used; parameter reached the node,
   e.g. `ros2 param get`; both cameras delivered frames; `ring_drops`, `shm_alloc_fail` are zero).
5. **Compare like with like.** Same payload size and layout, same QoS semantics, same stamp definition, same load,
   same consumers. Check what each stamp *means* before comparing latencies (the early ROS stamps made the ROS
   pipeline look 130 ms slower than it was).
6. **Trace stages instead of guessing.** Put timestamps at every hand-off (`BENCH_TRACE=1`), join by header stamp,
   report median/p95/max per stage (`benchmarks/pipeline/latency_trace.py`). Do not attribute a total to a cause
   without a stage that explains it; a constant, jitter-free offset usually means a definition problem, not load.
7. **Rule out measurement artifacts** before believing a number: a suspicious "hop" of exactly one scan period was
   my own join matching the previous scan; negative "consume" times are the intra-process delivery order.
8. **Sanity-check a tool before trusting it**: smoke-test every new script on a short run and read the raw output;
   the first full run of a script has hidden a bug three times (wrong shell semantics, a grep that dropped the
   interesting line, an unvalidated flag).
9. **Keep the machine honest**: log temperature, cool down between scenarios, discard throttled runs, clean up
   processes and recordings after each run (bags are ~2.4 GB/min compressed).

Adding a benchmark: a program or script under `benchmarks/<kind>/`, built from `scripts/build.sh`, a `run_*.sh` that
derives the repo root from its own path (no hardcoded paths), a line in `benchmarks/README.md` and in
`benchmarks/run_all.sh`, output as `key=value`-ish lines a person can read.

## 6. Methodology: reporting

- **Report final results, not the investigation.** The report states what is true now, with the numbers from the
  final runs. No "we first thought", no "earlier measurement", no running log. If a result changes, fix the sentence
  and the table; do not append a correction. History belongs in git and in this file's pitfall list.
- **Tag every claim** `[measured]` (ours) or `[background]` (general knowledge not re-tested). Say plainly what was
  not tested.
- **Numbers**: take them from the final result files (`results/<run>/`, kept in `docs/measurements/`); give ranges when repeats disagree; say
  single run when it is single; round to the precision the noise allows.
- **Recommendations come with the evidence that supports them and the ms they are worth**, ordered by gain per effort.
  Separate "done and verified" from "hypothesis".
- **Caveats are part of the result**: replay at 91% real time, emulated cameras, laptop thermals, in-process vs
  cross-process consumers, what was not measured.
- Write for the reader who will act on it (a teammate designing the car's pipeline): lead with the answer, keep the
  structure short (summary, mental model, building blocks, results, recommendations, verdict, checklist, future work,
  limits), explain jargon once.
- Before publishing: grep for stale numbers and old paths, check every cross-reference, re-run the numbers you
  quote if the code changed since they were measured.

## 7. Mistakes already made (so they are not repeated)

| What went wrong | Why | Lesson |
|---|---|---|
| Claimed SHM was active, then "SHM off made no difference" | Jazzy `rmw_zenoh` has SHM off by default; the test was a no-op | verify the setting took effect |
| A "network" baseline that used SHM | plain Zenoh has SHM on by default | set both explicitly, read the `shm=` counter |
| Said ROS loses ~130 ms inside the driver, ~14 ms on cameras | stamps were early (99 ms cloud) and taken before the grab (12 ms camera); the pure app stamped differently | align stamp definitions, then trace stages |
| "Default queue depth is 10" | the stack default (depth 0) is not 10; explicit 10 is far worse | sweep the parameter, label the default as default |
| Judged ROS vs Zenoh by total CPU | the recorder (zstd) dominates CPU but is off the latency path | pick the metric that matches the goal (ms on the critical path) |
| A "hop" of 111 ms | the join matched the previous scan | cross-check a surprising stage against raw timestamps |
| Camera 1 delivered nothing in one pure-Zenoh run | concurrent pylon enumeration | serialise device init |
| `wait %2` hung a 14 minute run; `pkill -f` killed the shell | non-interactive shell semantics | see section 3 |

## 8. Future work

- **Cloud build cost** (13-15 ms in `ouster_ros` vs ~3 ms in the pure app for the same output): profile
  `PointCloudProcessor::process` per step (XYZ, PCL compose, staging copy, move); try composing straight into the
  output message; build the two returns on two threads; build only the returns that are used. Verify the new output
  point for point against the original.
- Instrument the live `os_driver` publish path and make `latency_trace.py` work without a consumer node, then
  trace on the car; check the PTP stamp offset (expected a few ms).
- Fix the replay pacing (absolute schedule instead of per-packet sleep) so the benchmark runs at a true 10 Hz.
- Measure end-to-end latency to the real consumer (perception), cross-process, on target hardware with a live
  sensor and real cameras; measure the effect of a multi-threaded executor on the camera delivery tail under load.
- Foxglove with a live client against the 100 MB send-buffer bridge; recorder off the critical thread/process, and
  recording without zstd on the car.
- Sensor modes with a shorter sweep (higher-rate modes) or publishing partial scans: the ~110 ms sweep is the
  largest fixed item at 10 Hz. [background, untested here]
- A wire format and version policy for a pure-Zenoh hot link (not measured here).
- Optional: make the two driver repos submodules or commit the patches to branches in the team GitLab.
