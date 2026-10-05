# Moving high-bandwidth sensor data: ROS 2, Zenoh, IPC and shared memory

*A learning document for the team. ROS 2 Jazzy · `rmw_zenoh_cpp` 0.2.10 · Ouster OS-1-128 (2048x10, dual return, 12 MB cloud per return, ~9 Hz) and 2x Basler 1080p @ 10 Hz.*

**[measured]** = seen in our runs (raw output in [measurements/](measurements/), scripts in `benchmarks/`). **[background]** = general ROS 2 / Zenoh knowledge we did not re-test.

---

## 1. Summary

1. **Data is lost at the smallest queue on the path, not at the biggest message.** The LiDAR *packets* (33 KB, ~1,150 per second) between the pcap node and the cloud node were the fragile hop. With the driver's original queue (best-effort, depth 5) every scan was incomplete under load; with an explicit depth of 10 about 75% were; from depth 32 up none were. We use 512. [measured]
2. **Shared memory (SHM) works with ordinary variable-size messages** and makes publisher CPU independent of the number of readers (≈4% instead of 15-24% of a core). It is *off by default* in Jazzy's `rmw_zenoh`, it is not end-to-end zero-copy, and loaned messages are not available. [measured]
3. **Check what your timestamps mean.** `ouster_ros` stamped clouds one scan period (99 ms) too early in `TIME_FROM_ROS_TIME` mode (fixed by our patch; the PTP and sensor-time modes should not be affected, by code reading). The camera driver stamps images ~12 ms before the frame arrives. [measured]
4. **Where the milliseconds go** (per 9 Hz scan, ROS): ~110 ms sensor sweep (fixed), 14-15 ms building the clouds, and 6-14 ms in `publish()` to cross-process readers. After a scan completes the cloud is available to other processes after **21 ms with SHM and 30 ms without**. Pure Zenoh needs **3.6 ms** with SHM. [measured]
5. **Without SHM, pure Eclipse Zenoh costs the same as ROS** (27-31 ms on that stage); the gain comes from SHM plus building the cloud straight into the shared buffer. A rewrite also gives up the ROS ecosystem.
6. **For a latency-critical car:** fix the stamps, enable SHM, and cut the cloud-build cost inside ROS first; consider pure Zenoh only for a single hot link afterwards (§6).

---

## 2. A mental model: where can data get lost or slowed?

```
 sensor ──UDP──▶ driver ──▶ processing ──▶ publish ──▶ readers
                  (packets)   (scan → cloud)   │
                                               ├─▶ same process   (IPC: pointer hand-over)
                                               ├─▶ other process  (Zenoh: serialize + copy, or SHM)
                                               └─▶ viewer/recorder (always another process)
```

Every arrow has a **queue** and every queue has a **depth**. When a consumer stalls for a moment the queue fills and the oldest data is dropped. The big 12 MB message is not the fragile part: it arrives once per frame. The small, fast messages (packets) are.

> **Rule of thumb:** `needed depth ≈ message rate × longest stall you want to survive`.
> Packets: 1,150/s × 0.1 s ≈ 115, so depth 128 or more. Our worst observed stall of the processing thread was 43 ms (≈50 packets); we use 512 (≈0.45 s).

---

## 3. The building blocks and what each one really does

| Piece | What it does | Works when | Does **not** help when |
|---|---|---|---|
| **Composable nodes** | Run several nodes inside one process | Nodes are loaded into the same `component_container` | Nodes are launched as separate processes |
| **Intra-process comms (IPC)** | Hands a message to a node in the same process without serializing it [background] | Same container and `use_intra_process_comms: True`; best with one `unique_ptr` reader | The reader is in another process: **recorders and viewers are always other processes**. Queues are still bounded |
| **Zenoh network transport** | Delivers between processes/hosts | Router (`rmw_zenohd`) running, same RMW on every side | Large messages are copied for every reader |
| **Zenoh SHM** | Puts large messages in shared memory instead of copying them | Same host, SHM enabled on **both** ends, shared IPC namespace if containerized (`ipc: host`), pool big enough | Across hosts (falls back to the network); WebSocket viewers such as Foxglove [background]; ~30 KB messages (no gain) |
| **Loaned messages** | Publisher writes straight into the transport buffer: true zero-copy | RMW and message type both support it | **`rmw_zenoh` 0.2.10 does not.** `can_loan_messages()` is `false` for `PointCloud2` *and* for the fixed-size `Float64` [measured] |

Two common misconceptions:

- **"SHM needs fixed-size messages."** No: a 12 MB variable-size `PointCloud2` went through SHM. Fixed-size types matter for *loaned* zero-copy, a separate feature that is not available here. [measured]
- **"SHM is zero-copy."** Not end to end in ROS: the publisher serializes into the shared buffer and each reader deserializes out of it. Plain Zenoh can read the buffer in place (§4.2). [measured]

---

## 4. Results

### 4.1 Packet queue depth [measured]

The pcap node publishes LiDAR packets to the cloud node (the live `os_driver` node has no such topic). A dropped packet removes 16 scan columns, so slices of the cloud pop in and out and frame stamps bunch up. Incomplete scans per ~30 s run (~270 scans), recorder + 2 cross-process readers:

| Packet queue | SHM off | SHM on |
|---|---|---|
| Driver default: best-effort, depth 5 | 270, 270 of 270 (worst scan keeps 74% of its columns) | 270 of 270 (85%) |
| Explicit depth 10 | 221-224 of ~271 (3 runs) | 194-204 of ~270 (3 runs) |
| Stack default (reliable, depth 0) | 0 in 4 runs | 0 in 3 runs |
| Depth 32 / 64 / 128 / 256 | 0 / 0 / 0 / 0 | 0 (depth 32) |
| Depth 512 (our setting) | 0 in 3 runs | 0 in 3 runs |

The stack's default depth is *not* 10. It was clean in these 7 runs, but earlier runs of the same setting under load showed 2-9% incomplete scans, so treat it as borderline. SHM lowers how often a small queue overflows (publishing gets cheaper) but does not remove the failure mode: **queue depth is the fix, SHM is headroom.** A multi-threaded container and a faster CPU did not help (CPU was never saturated); putting the packet hop on the network transport made it worse.

### 4.2 Transport micro-benchmarks: ROS vs plain Zenoh [measured]

One publisher process, 1 or 3 reader processes on one host, 12 MB messages at 10 Hz.

| | Publisher CPU (1 / 3 readers) | Reader CPU | Median latency (1 / 3 readers) |
|---|---|---|---|
| **ROS**, SHM off | 15-16% / 17-24% | 10-13% / 12-18% | 19-23 ms / 20-32 ms |
| **ROS**, SHM on | 3.7-3.9% / 3.7-4.2% | 7-9% / 8-13% | 10-11 ms / 11-20 ms |
| **Zenoh**, SHM off | 5.9% / 15.7% | 4-5% | 5.6 ms / 6-13 ms |
| **Zenoh**, SHM on, reader copies the data | 1.2% / 1.2% | 2.1-2.5% | 3.2 ms / 3.1-3.7 ms |
| **Zenoh**, SHM on, reader reads in place | 1.1% / 1.2% | 0.3% | 0.4 ms / 0.4 ms |

(ROS: three runs per cell; Zenoh: one run, repeats in the same session agreed closely.) With SHM the publisher cost stays flat as readers are added, which matters for fan-out (recorder + viewer + perception). Tail latency improves less than the median: with 3 ROS readers and SHM two of three runs had a p99 of 170-200 ms, and a reader missed 1 of 100 messages in two runs.

**Small messages** (30 KB at 1,150 Hz, a LiDAR packet stream, 1 reader): nothing was lost in any configuration, medians were 0.1-0.3 ms. ROS: publisher 7.8% / reader 7.7% without SHM, 11% / 6.6% with SHM (SHM does not help and costs slightly more). Plain Zenoh: publisher 4.5-5.6% and reader ~5% in every mode.

### 4.3 The full pipeline [measured]

A ROS-free app (`benchmarks/pipeline/zenoh_sensors`) built on the **Ouster SDK** and **Pylon SDK** mirrors the ROS pipeline, and both ran under the same load: 2 cameras 1080p @ 10 Hz, Ouster 2048x10 with two returns, a recorder, two cross-process cloud readers. 25-40 s windows.

| | ROS, SHM off | ROS, SHM on | Pure Zenoh, SHM off | Pure Zenoh, SHM on |
|---|---|---|---|---|
| Pipeline CPU, no recorder (cores) | 0.58 | 0.42 | 0.58-0.68 | **0.19-0.21** |
| Recorder CPU (cores) | ~0.67 (1) | ~0.66 (1) | 0.65-0.73 | 0.52-0.56 |
| **Total with recorder (cores)** | 1.25 | 1.08 | 1.2-1.35 | **0.71** |
| Pipeline memory (RSS) | 290-375 MB | 1.3 GB | 194 MB | 684 MB |
| **Scan complete → cloud published** (§4.4) | 29.5 ms | 20.9 ms | 27-31 ms | **3.6 ms** |
| Cloud latency, cross-process reader, from scan start (2) | 160 ms | 146 ms | 119-128 ms | **112 ms** |
| Camera: frame available → reader (3) | ~4-6 ms | ~2.5-4 ms | 2-4 ms | **0.6 ms** |
| Scans with missing columns; rates | 0; 9.0 Hz, 10 Hz | 0; same | 0; same | 0; same |

(1) Derived: with-recorder minus no-recorder run. (2) ROS from a single-threaded Python probe, pure Zenoh from a C++ reader; the recorder adds 3-4 ms to the ROS number. (3) ROS: the probe's 14-18 ms minus the ~12 ms by which the camera stamp precedes the frame (§4.4); pure Zenoh stamps after the grab.

- **Without SHM the two stacks cost the same** (0.58 vs 0.58-0.68 cores; 29.5 vs 27-31 ms). The ROS layer is not expensive by itself.
- **With SHM the pure version needs half the pipeline CPU and a sixth of the critical-stage time**, because it fills the shared buffer directly and readers use it in place.
- **The recorder costs ~0.6-0.7 cores in both stacks** (zstd compression of ~145 MB/s raw). That caps what any rewrite can save in total CPU, but it is not on the latency path.
- **Blocking publish can stall a sensor.** In one of four pure-Zenoh runs without SHM, a slow reader blocked `put()` (congestion control `BLOCK`) for ~5 s, the ring buffer overflowed and 1,269 packets were dropped. For sensor data use a dropping congestion mode or make sure readers keep up.

### 4.4 Where the milliseconds go: stage trace [measured]

Optional timestamps at every stage of the ROS pipeline (`patches/latency-trace-*.patch`, `BENCH_TRACE=1`, analysed by `benchmarks/pipeline/latency_trace.py`), ~455 scans and ~500 frames per camera per scenario. The consumer is the in-process node.

**Cloud (two returns):**

| Stage | No cross-process readers | Recorder + 2 readers, SHM off | Recorder + 2 readers, SHM on |
|---|---|---|---|
| Scan assembly (sensor sweep) | 109.3 ms | 110.6 ms | 109.8 ms |
| Queue wait for the processing thread | 0.09 ms | 0.09 ms | 0.11 ms |
| Build the clouds | **14.0 ms** | **15.3 ms** | **14.7 ms** |
| `publish()` calls | 0.02 ms | **14.2 ms** | **6.3 ms** |
| **Scan complete → in-process consumer** | 14.1 ms | 19.7 ms | 18.8 ms |
| Stamp vs the real first packet | −0.8 ms | −0.8 ms | −0.8 ms |

- **`publish()` runs on the same thread that builds the next cloud.** With cross-process readers it costs 14 ms, or 6 ms with SHM (serialization and sending). That is on the critical path.
- **In-process consumers get the message before `publish()` returns**, so their numbers hide this cost; cross-process readers receive it after `build + publish`: 29.5 ms without SHM, 20.9 ms with.
- **Cloud stamp.** In `TIME_FROM_ROS_TIME` mode `ouster_ros` derived the next cloud's stamp from the packet that completes a scan, assuming it is the first packet of the next scan. With this SDK a scan completes on its *last* packet, so every cloud was stamped one scan period early: 99.3 ms at 2048x10 (2032 columns × 48.8 µs, jitter 0.02 ms). `patches/ouster-ros-stamp-fix.patch` fixes it: the stamp now equals the real scan start (the −0.8 ms is one packet period). Anything using the stamp (TF lookups, de-skew, fusion) would otherwise place the cloud ~99 ms in the past. The PTP and sensor-time modes take the stamp from the sensor's column timestamps (by code reading, not tested), so a PTP setup should be unaffected; check once on the car that the offset is a few ms.

**Camera (emulated, 1080p @ 10 Hz):**

| Stage | Median |
|---|---|
| Stamp (taken *before* the blocking grab) → frame retrieved | **11.9 ms** |
| Copy pixels into the message | 0.2 ms |
| Rest until published | 0.2 ms (0.5 ms under load) |
| **Frame available → in-process consumer** | **0.45 ms** (0.6 ms under load, p95 9-12 ms under load) |

The driver stamps the image before the blocking grab, so the stamp precedes the frame by the grab wait; once the frame is available the ROS path costs about 0.5 ms. Under load the p95 rises to 9-12 ms (cause not identified). Which stamp is right depends on the camera (free-running or triggered, hardware chunk timestamps on or off).

**Same stage, same load, ROS vs plain Zenoh** (scan complete → all returns built and published):

| ROS, SHM off | ROS, SHM on | Pure Zenoh, SHM off | Pure Zenoh, SHM on |
|---|---|---|---|
| 15.3 + 14.2 = 29.5 ms | 14.7 + 6.3 = 20.9 ms | 27-31 ms | **3.6 ms** |

Without SHM both spend the time serializing 12 MB twice for three readers. With SHM the pure version fills the shared buffer directly, while `ouster_ros` builds a PCL cloud and converts it into the ROS message (14-15 ms for the build alone; the reason is a hypothesis, see §8).

---

## 5. Recommended setup for a high-bandwidth sensor

**Layout**
- Driver, processing and in-process consumers in **one composable container** with IPC on.
- Recorders and viewers are separate readers: expect them to add serialization work on the publish path. This is the main reason to enable SHM.

**Queues**
- Give **small, fast** hops (LiDAR packets, IMU) a **deep** queue (rate × worst stall); reliable QoS is fine on a same-process hop.
- Keep **big-message** topics (clouds, images) shallow: a deep queue of 12 MB messages is just memory.
- For sensor data on a Zenoh link prefer a dropping congestion mode over blocking.

**Turn on Zenoh SHM** (off by default in Jazzy). On every process, via `ZENOH_CONFIG_OVERRIDE` or a session config file (`scripts/zenoh_env.sh` does this):

```
transport/shared_memory/enabled=true
transport/shared_memory/mode="init"
transport/shared_memory/transport_optimization/enabled=true
transport/shared_memory/transport_optimization/pool_size=536870912      # 512 MB
transport/shared_memory/transport_optimization/message_size_threshold=512
```

- **Size the pool** for messages in flight: roughly message size × (queue depth + readers holding one). The 48 MB default holds about four 12 MB clouds. [background sizing rule]
- Containers need `ipc: host` and `ulimits: memlock: -1`. The `error setting scheduling priority` watchdog warnings are harmless.
- SHM raises resident memory (shared pool pages count): ROS container 290 MB → 1.3 GB, pure Zenoh 194 → 684 MB. [measured]

**Recorder:** rosbag2 mcap, ~2 GiB splits, `compression_mode: file`, `compression_format: zstd`. Raw cloud + cameras are ~145 MB/s; zstd costs ~0.6-0.7 cores. Stop with Ctrl-C so metadata is written. Keep it off the critical thread, and consider recording raw on the car and compressing offboard.

**Viewers:** Foxglove's default `send_buffer_limit` (10 MB) is smaller than one 12 MB cloud, so raise it above your largest message; the compose file does this (`docker compose --profile viewer up -d foxglove`, 100 MB). *Not verified with a live client.*

**Live Ouster lidar:** use the **`os_driver`** node. It receives the UDP packets and builds the cloud in one node, so there is **no packet topic that can drop**. Also: enlarge the kernel UDP receive buffer (`net.core.rmem_max`) [background]; use `min_scan_valid_columns_ratio` to skip mostly empty scans; verify the cloud header stamp against a trace (§4.4); and use the metadata file that matches the sensor and mode, since a wrong one distorts or splits the cloud.

---

## 6. Verdict: should we write this in pure Eclipse Zenoh?

For a **latency-critical pipeline such as a racing car** judge by the stages in §4.4, not by total CPU: the recorder dominates CPU but is not on the latency path.

| Item (per 9 Hz scan, ROS) | ms | Fixable? |
|---|---|---|
| Sensor sweep (one full revolution) | ~110 | Only by sensor mode or publishing partial scans |
| Cloud stamped one scan too early (ROS-time mode) | 99, a timestamp error | **Fixed** by `patches/ouster-ros-stamp-fix.patch`; verify on the car |
| Building the clouds in `ouster_ros` | 14-15 | Yes: the pure app does the same work in ~3 ms |
| `publish()` to cross-process readers | 6 (SHM) to 14 (no SHM) | Largely, with SHM and fewer or lighter readers |
| Camera stamp before the blocking grab | ~12, a stamp artifact | Depends on the camera setup |
| ROS camera path after the frame is available | ~0.5 | Nothing to gain |

**What this means**
1. **The transport (ROS vs Zenoh) is not where most milliseconds are.** Without SHM the two stacks cost the same on the critical stage. The big items are the sweep, the stamp, the cloud build and `publish()`.
2. **Do these first, all inside ROS:** (a) the stamp (99 ms of error matters more to fusion than any latency below): fixed in our patch set, verify on the car; (b) enable SHM (`publish()` 14 → 6 ms); (c) cut the cloud-build cost (§8).
3. **Pure Eclipse Zenoh is the floor, not the starting point.** It reaches 3.6 ms for build + publish and 0.4 ms for a zero-copy read. That is worth a bridge on a single hot link (cloud → perception) if, after step 2, the remaining ~10-15 ms on the critical path still matter. A full rewrite also costs the ROS ecosystem (the Ouster and Pylon ROS drivers, rosbag2, Foxglove, TF, launch and parameters) plus a wire format of our own, **not measured here**.
4. **Keep the recorder off the critical thread.** `publish()` serializes on the thread that builds the next cloud, so every cross-process reader costs time there.
5. **Shorten the sweep if the budget demands it.** At 10 Hz the sensor needs ~110 ms per revolution before any software runs; higher-rate sensor modes or partial scans cut that. [background, not tested here]

---

## 7. Checklist for the next high-bandwidth sensor

- [ ] Find the **smallest, fastest queue** on the path and size it for the worst stall.
- [ ] Keep driver and processing **in one process**; avoid an extra hop through a topic.
- [ ] Decide who reads the data: **recorders and viewers are always other processes**.
- [ ] Enable **SHM** on both ends, size the pool, share `/dev/shm` in containers, and verify it really engages (off by default in Jazzy `rmw_zenoh`, on by default in plain Zenoh).
- [ ] Measure **completeness** (not just rate): share of valid scan columns, gaps between header stamps, under the real load.
- [ ] Check what each **header stamp** means (when is it taken, relative to scan start or exposure?) and compare it to a stage trace.
- [ ] Trace the **stages** (arrival, assembly, build, publish, delivery) before optimising: `benchmarks/pipeline/latency_trace.py`.
- [ ] Watch **disk rate** (~145 MB/s raw here) and the recorder's CPU.
- [ ] Repeat runs, interleave A/B, log temperature: laptops throttle.

---

## 8. Future work

- **Cloud build cost** (14-15 ms in `ouster_ros` vs ~3 ms in the pure app for the same output): time the steps inside `PointCloudProcessor::process` (XYZ, PCL compose, staging copy, move); try composing straight into the output message; build the two returns on two threads; build only the returns that are used; verify the new output point for point against the original.
- Trace the **live `os_driver`** (its publish path is not instrumented yet) and confirm the PTP stamp offset on the car; measure end-to-end latency to the real consumer on target hardware with a live sensor and real cameras.
- Make the **pcap replay run at a true 10 Hz** (it runs at ~91% of real time).
- Try a **multi-threaded executor** and see if the camera delivery tail under load (p95 9-12 ms) disappears.
- **Foxglove** with a live client against the 100 MB send-buffer bridge.
- **Sensor modes** with a shorter sweep, or partial-scan publishing.
- A **wire format and versioning policy** for a pure-Zenoh hot link.

---

## 9. Test setup and limits

- One host. A `component_container` with the Ouster pcap replay and cloud node, 2 emulated Basler cameras (1080p @ 10 Hz), a consumer node and the rosbag2 recorder; `rmw_zenohd` router in a second container. SHM was enabled explicitly where marked "SHM on".
- Metrics: share of scan columns containing finite points; gaps between header stamps (healthy = 100% columns, steady ~110 ms); latency; process CPU and RSS from `/proc`; stage times from the trace.
- Run-to-run variation is real (ROS publisher CPU without SHM varied 15-24% across runs of one configuration): ranges are given where repeats were made, otherwise treat a number as indicative. The laptop reached 70-83 °C during runs.
- **The pcap replay runs at ~91% of real time** (a scan every ~109.5 ms, ~1,150 packets/s): both stacks use the same pacing, so comparisons are fair, but the load is ~9% below a real 10 Hz sensor.
- ROS cross-process latency comes from a single-threaded Python probe, pure Zenoh from a C++ reader; the stage trace uses an in-process C++ consumer.
- Emulated cameras and a replayed pcap do not reproduce a live sensor's UDP socket, PTP clocks or real camera timing.
- Reproduce everything: `benchmarks/run_all.sh` (see `benchmarks/README.md`); `AGENTS.md` describes the method.
