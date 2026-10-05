# Moving high-bandwidth sensor data: ROS 2, Zenoh, IPC and shared memory

*A learning document for the team. Date: 2026-10-05 · ROS 2 Jazzy · `rmw_zenoh_cpp` 0.2.10 · Reference sensors: Ouster OS-1-128 (12 MB cloud @ ~9-10 Hz) and 2x Basler 1080p @ 10 Hz.*

How to read it: **[measured]** = we saw it in our own tests (details and scripts in `benchmarks/`). **[background]** = general ROS 2 / Zenoh knowledge we did not re-test.

---

## 1. The short version

1. **Data is lost at the smallest queue on the path, not at the biggest message.** Our flicker was LiDAR *packets* (33 KB, ~1,150/s) overflowing a queue with ~8 ms of room. [measured]
2. **Size queues for the stall you must survive**, not for the average rate. A deeper packet queue (512) fixed it completely. [measured]
3. **Shared memory (SHM) works with normal variable-size messages** (clouds, images). It is *off by default* in ROS 2 Jazzy's `rmw_zenoh`, and turning it on cuts publisher cost and latency a lot. [measured]
4. **True zero-copy needs "loaned messages", which `rmw_zenoh` 0.2.10 does not support**, even for fixed-size types. [measured]
5. **Check your timestamps before chasing milliseconds.** In our stage trace (§4.5) about **99 ms of the cloud's "latency" was a header stamp one scan period too early** (fixed in our patch set), and ~12 ms of the camera's was a stamp taken before a blocking grab. The real time after a scan completes is ~14-20 ms in-process, dominated by building the cloud (13-15 ms) and, with cross-process readers, `publish()` (7-16 ms). [measured]
6. **Pure Eclipse Zenoh is not magic.** On the critical stage (scan complete → cloud published) it costs the same as ROS without SHM (30.5 vs 31 ms) and ~6x less with SHM (3.6 vs 22 ms), because it builds the cloud straight into the shared buffer. **For a latency-critical car: fix the stamps, enable SHM, and cut the cloud build cost first; only then consider pure Zenoh for the hot link.** (§6)

---

## 2. A mental model: where can data get lost or slowed?

```
 sensor ──UDP──▶ driver ──▶ processing ──▶ publish ──▶ readers
                  (packets)   (scan → cloud)   │
                                               ├─▶ same process   (IPC: pointer hand-over)
                                               ├─▶ other process  (Zenoh: serialize + copy, or SHM)
                                               └─▶ viewer/recorder (always another process)
```

Every arrow has a **queue** and every queue has a **depth**. When a consumer stalls for a moment, the queue fills and the oldest data is dropped. The big 12 MB message is not the fragile part: it arrives once per frame. The small, fast messages (packets) are.

> **Rule of thumb:** `needed depth ≈ message rate × longest stall you want to survive`.
> Packets: 1,150/s × 0.4 s ≈ 460, so depth 512. The default was 5-10, about 8 ms.

---

## 3. The building blocks and what each one really does

| Piece | What it does | Works when | Does **not** help when |
|---|---|---|---|
| **Composable nodes** | Run several nodes inside one process | Nodes are loaded into the same `component_container` | Nodes are launched as separate processes |
| **Intra-process comms (IPC)** | Hands a message to a node in the same process without serializing it [background] | Same container and `use_intra_process_comms: True`. Best with one `unique_ptr` reader | The reader is in another process. **Recorders and viewers are always other processes.** Queues are still bounded |
| **Zenoh network transport** | Delivers between processes/hosts | Router (`rmw_zenohd`) running, same RMW on every side | Large messages are copied for every reader |
| **Zenoh SHM** | Puts large messages in shared memory instead of copying them | Same host, SHM enabled on **both** ends, shared IPC namespace if containerized (`ipc: host`), pool big enough | Across hosts (falls back to the network); WebSocket viewers such as Foxglove [background] |
| **Loaned messages** | Publisher writes straight into the transport buffer: true zero-copy | RMW and message type both support it | **`rmw_zenoh` 0.2.10 does not.** `can_loan_messages()` returned `false` for `PointCloud2` *and* for fixed-size `Float64` [measured] |

Two myth-busters from our tests:

- **"SHM needs fixed-size messages."** No. A 12 MB variable-size `PointCloud2` went through SHM fine. [measured] Fixed-size types matter for *loaned* zero-copy, which is a separate feature that is not available here.
- **"SHM is zero-copy."** Not end to end. In ROS the publisher serializes into the shared buffer and each subscriber deserializes out of it, so there is still one copy at each end. [background, consistent with our CPU numbers]

---

## 4. What we found

### 4.1 The flicker: a tiny queue, not a big message [measured]

The pcap → cloud hop sends ~1,150 packets/s. With the default queue (depth 5-10) any short stall dropped packets; each lost packet removed 16 scan columns, so slices of the cloud popped in and out and frame timestamps bunched up.

Incomplete scans per run (≈313 scans each; recorder on, 2 extra cloud readers):

| Packet queue depth | SHM off | SHM on |
|---|---|---|
| 10 (default) | 20, 27 | 6, 7 |
| **512** | **0, 0** | **0** |

SHM *reduces* how often the small queue overflows (publishing gets cheaper), but only the deeper queue removes the failure mode: **queue depth is the fix, SHM is headroom.**

Things that did **not** help: a multi-threaded container, a faster CPU (it was never saturated: busiest thread ~17% of a core), and moving the packet hop onto the network transport (worse: every frame incomplete).

### 4.2 What SHM buys you (ROS, 12 MB cloud @ 10 Hz) [measured]

| | Publisher CPU (1 / 3 readers) | Median latency (1 / 3 readers) |
|---|---|---|
| SHM off | 18% / 35% | 18 ms / 31-42 ms |
| SHM on | 4% / 4% | 5.6 ms / 13-14 ms |

With SHM the publisher cost **stays flat as you add readers** (recorder + viewer + perception). Without it, cost grows with every reader.

### 4.3 What pure Eclipse Zenoh buys you (no ROS, same Zenoh version) [measured]

Same 12 MB payload, same measurements:

| | Publisher CPU (1 / 3 readers) | Reader CPU | Median latency (1 / 3 readers) |
|---|---|---|---|
| Pure Zenoh, no SHM | 5% / 21% | 4-5% | 5 ms / 9-19 ms |
| Pure Zenoh, SHM, reader copies the data | 1.2% / 1.2% | 1.4-2.4% | 2 ms / 3.2-3.4 ms |
| Pure Zenoh, SHM, reader reads in place | 1.1% / 1.2% | 0.2-0.3% | 0.4 ms / 0.4 ms |

Pure Zenoh is cheaper mostly because it does not build a ROS message or CDR-serialize it, and because a reader can look at the shared buffer *without copying*. Note a trap: **plain Zenoh turns SHM on by default**, whereas ROS's `rmw_zenoh` config turns it off. Check this before comparing anything.

For small high-rate messages (24 KB @ 1,280 Hz) the two stacks were close and **nothing was lost** in either (ROS: 10% publisher / 8% reader CPU, pure Zenoh: 7-8% / 5%). SHM does not help at that size.

### 4.4 The full pipeline, side by side [measured]

We wrote a ROS-free app (`benchmarks/pipeline/zenoh_sensors`) on the **Ouster SDK** and **Pylon SDK** directly, mirroring the ROS pipeline, and ran both under the same load: 2 cameras 1080p@10 Hz, Ouster 2048x10 with two returns, a recorder, two cloud readers.

| | ROS net | ROS SHM | Pure net | Pure SHM |
|---|---|---|---|---|
| Pipeline CPU, no recorder (cores) | 0.60 | 0.44 | 0.62 | **0.21** |
| Recorder CPU (cores) | ~0.7 | ~0.6 | 0.73 | 0.56 |
| **Total with recorder (cores)** | 1.30 | 1.06 | 1.35 | **0.77** |
| Scan complete → cloud published (build + publish, see §4.5) | 31 ms | 22 ms | 30.5 ms | **3.6 ms** |
| Camera: frame available → consumer (cross-process) | ~2-6 ms (1) | ~2-4 ms (1) | 4-5 ms | **0.6 ms** |
| Incomplete scans / rates | 0 / 9 Hz, 10 Hz | 0 / same | 0 / same | 0 / same |

(1) approximate: the probe's latency (14-18 ms) minus the 11.9 ms the ROS camera stamp precedes the frame (§4.5). The raw probe numbers, 257/244 ms cloud and 14-18 ms camera for ROS, **should not be compared with the pure-Zenoh numbers**: the ROS stamps are early (§4.5).

How to read it:

- **Without SHM the two are equal** (0.60 vs 0.62 cores). The ROS layer is not expensive by itself.
- **With SHM the pure version needs half the pipeline CPU**, because it builds the cloud directly in the shared buffer and readers use it in place.
- **The recorder costs ~0.6-0.7 cores in both stacks** (zstd compression). That caps what any rewrite can save: total 1.06 → 0.77 cores.
- **Our first reading of the latency gap was wrong.** We attributed ~130 ms to the ROS driver. The stage trace in §4.5 shows ~99 ms of it is a **mis-stamp** (not delay) and the real driver cost after a scan completes is ~14-20 ms.

### 4.5 Where the milliseconds go: a stage trace [measured]

We added optional timestamps at every stage of the ROS pipeline (`patches/latency-trace-*.patch`, enabled with `BENCH_TRACE=1`, analysed by `benchmarks/pipeline/latency_trace.py`) and measured ~400 scans and ~440 frames per camera. The consumer is the in-process node, so no cross-process transport is included except where stated.

**Cloud (2048x10, two returns):**

| Stage | No cross-process readers | Recorder + 2 readers, SHM off | Recorder + 2 readers, SHM on |
|---|---|---|---|
| Scan assembly (first → last packet; the sensor sweep) | 111.7 ms | 110.3 ms | 109.7 ms |
| Queue wait for the processing thread | 0.09 ms | 0.09 ms | 0.09 ms |
| Build the clouds (both returns) | **13.4 ms** | **15.4 ms** | **15.0 ms** |
| `publish()` calls | 0.02 ms | **15.8 ms** | **7.2 ms** |
| **Scan complete → in-process consumer** | 13.6 ms | 19.8 ms | 19.4 ms |
| Header stamp vs the real first packet | **99.3 ms early** | 99.3 ms early | 99.3 ms early |

- **The cloud stamp is one scan period too early.** The offset is almost jitter-free (95th percentile 99.27 ms, max 99.28 ms) and equals 2032 columns × 48.8 µs, the column index of the *last* packet of a 2048x10 scan. `ouster_ros` extrapolates the next cloud's stamp from the packet that completes the current scan and assumes it is the *first* packet of the next one. With this SDK's `ScanBatcher` a scan completes on its *last* packet, so every cloud is stamped about one scan early. Anything that uses the stamp (TF lookups, de-skewing, fusion) places the cloud ~99 ms in the past. We saw this with `timestamp_mode: TIME_FROM_ROS_TIME`; from reading the code the sensor-time and PTP modes take stamps from the sensor instead, but we did not test them against a real sensor, and we do not know which mode the car uses.
- **Fix, verified in replay.** `patches/ouster-ros-stamp-fix.patch` stops deriving the next stamp from the completing packet: it clears it so the next packet, which belongs to the next scan, sets it (`extrapolate_frame_ts` works from any packet via its column index). With it the stamp offset drops from +99.25 ms to **−0.84 ms**, exactly one packet period (the trace's "first packet" time is that of the previous scan's last packet), and the arrival-minus-stamp difference is constant from scan to scan (366 of 368 scans within ±2 ms). The other stages are unchanged.
- **Sensor-time modes cannot be checked for an absolute offset in a replay**: the recorded sensor clock is not the wall clock. Their stamps advance exactly 100.00 ms per scan (the sensor's true 10 Hz), which is consistent. On the car, run the trace and look at `stamp_offset`: a few ms is fine, ~99 ms means this problem.
- **Real latency after the scan completes is 14-20 ms**, most of it building the clouds. Queueing is negligible.
- **`publish()` runs on the same thread that builds the next cloud.** With cross-process readers (recorder, viewers, perception) it costs 16 ms, or 7 ms with SHM: that is serialization and sending. It is on the critical path.
- **In-process consumers get the message before `publish()` returns** (the intra-process copy comes first), so their numbers hide this cost; cross-process readers get it after.

**Camera (emulated, 1080p @ 10 Hz):**

| Stage | Median |
|---|---|
| Stamp (taken *before* the blocking grab) → frame retrieved | **11.9 ms** |
| Copy pixels into the message | 0.2 ms |
| Rest until published | 0.2 ms |
| Published → in-process consumer | 0.04 ms |
| **Frame available → consumer** | **0.45 ms** |

The driver stamps the image *before* the blocking grab, so the stamp precedes the frame by the grab wait. The "14-18 ms camera latency" of the first measurements was this offset, not delay: once the frame is available, the ROS path costs about 0.5 ms. Which stamp is right depends on the camera (free-running or triggered, hardware chunk timestamps on or off); check it for your setup.

**Same stage, same load, ROS vs pure Zenoh** (scan complete → all returns built and published):

| ROS net | ROS SHM | Pure net | Pure SHM |
|---|---|---|---|
| 15.4 + 15.8 = 31 ms | 15.0 + 7.2 = 22 ms | 30.5 ms | **3.6 ms** |

Without SHM the two are equal, because both spend their time serializing 12 MB twice for three readers. With SHM the pure version needs 3.6 ms: it fills the shared buffer directly, while `ouster_ros` builds a PCL cloud and then converts and copies it into the ROS message (13-15 ms for the build alone).

---

## 5. Recommended setup for a high-bandwidth sensor

**Layout**
- Driver, processing and in-process consumers in **one composable container** with IPC on.
- Recorders and viewers are separate readers. Expect them to add serialization work on the publish path. This is the main reason to enable SHM.

**Queues**
- Give **small, fast** hops (LiDAR packets, IMU) a **deep** queue (rate × worst stall). Reliable QoS is fine on a same-process hop.
- Keep **big-message** topics (clouds, images) shallow. A deep queue of 12 MB messages is just memory.

**Turn on Zenoh SHM** (off by default in Jazzy). On every process, via `ZENOH_CONFIG_OVERRIDE` or a session config file:

```
transport/shared_memory/enabled=true
transport/shared_memory/mode="init"
transport/shared_memory/transport_optimization/enabled=true
transport/shared_memory/transport_optimization/pool_size=268435456      # 256 MB
transport/shared_memory/transport_optimization/message_size_threshold=512
```

- **Size the pool** for messages in flight: roughly message size × (queue depth + readers holding one). The 48 MB default holds about four 12 MB clouds. [background sizing rule]
- Containers need `ipc: host` and `ulimits: memlock: -1`. The `error setting scheduling priority` watchdog warnings are harmless.
- SHM raises resident memory (shared pool pages count): ROS container 289 → 1,340 MB, pure Zenoh 182 → 684 MB. [measured]

**Recorder:** rosbag2 mcap, ~2 GiB splits, `compression_mode: file`, `compression_format: zstd`. Raw cloud + cameras ran ~125-150 MB/s; zstd cut finished files ~3.7x. Stop with Ctrl-C so metadata is written.

**Viewers:** Foxglove's default `send_buffer_limit` (10 MB) is smaller than one 12 MB cloud, so raise it above your largest message. The repo's compose file does this (`docker compose --profile viewer up -d foxglove`, 100 MB). *Not yet verified with a live client.*

**Live Ouster lidar:** use the **`os_driver`** node. It receives the UDP packets and builds the cloud in one node, so there is **no packet topic that can drop** (the problem in §4.1 comes from our pcap *replay*, which sends packets over a topic). Also: enlarge the kernel UDP receive buffer (`net.core.rmem_max`) [background]; use `min_scan_valid_columns_ratio` to skip scans that are mostly empty; check the **cloud header stamp** against a trace (with `TIME_FROM_ROS_TIME` we measured it ~99 ms early, see §4.5); and use the metadata file that matches the sensor and mode, since a wrong one distorts or splits the cloud.

---

## 6. Verdict: should we write this in pure Eclipse Zenoh?

The answer depends on what you optimise. We first judged by total CPU, where the recorder (zstd, ~0.6-0.7 cores in both stacks) dominates, and concluded "stay on ROS". For a **latency-critical pipeline such as a racing car**, CPU is the wrong yardstick and the picture is different. Recording is not on the latency path (cloud latency was 257 ms with the recorder off and 259 ms with it on), so judge by the stages in §4.5.

**Where the milliseconds are** (per 10 Hz scan, ROS, in-process consumer):

| Item | ms | Fixable? |
|---|---|---|
| Scan assembly (one full revolution) | ~110 | Only by sensor mode or publishing partial scans (item 5 below) |
| Cloud stamped one scan too early | 99 (a timestamp error, not delay) | **Fixed** by `patches/ouster-ros-stamp-fix.patch` (verified in replay); verify on the car |
| Building the clouds in `ouster_ros` | 13-15 | Yes (pure Zenoh: ~3 ms for the same work) |
| `publish()` to cross-process readers | 7 (SHM) to 16 (no SHM) | Largely, with SHM and fewer or lighter readers |
| Camera stamp before the blocking grab | ~12 (a stamp artifact) | Depends on camera setup |
| ROS camera path after the frame is available | ~0.5 | Nothing to gain |

**What this means**
1. **The transport (ROS vs Zenoh) is not where most milliseconds are.** Without SHM the two stacks cost the same on the critical stage. The big items are the sweep, the stamp, the cloud build and `publish()`.
2. **Do these first, all inside ROS:** (a) the stamp (99 ms of error matters more to fusion than any latency below): our patch set fixes it, verify it on the car; (b) enable SHM (publish 15.8 → 7.2 ms); (c) cut the cloud build cost, for example by building straight into the output message instead of through PCL, or by publishing only the returns you use. The last point is a hypothesis: the pure app does the same work in ~3 ms, but we did not try it inside `ouster_ros`.
3. **Pure Eclipse Zenoh is the floor, not the starting point.** It reaches 3.6 ms for build + publish and 0.4 ms for a zero-copy read. That is worth a bridge on a single hot link (cloud → perception) if, after step 2, the remaining ~10 ms on the critical path still matter. A full rewrite also costs the ROS ecosystem: the Ouster and Pylon ROS drivers, rosbag2, Foxglove, TF, launch and parameters, plus a wire format of our own (**not measured here**).
4. **Keep the recorder off the critical thread.** `publish()` serializes on the thread that builds the next cloud, so every cross-process reader costs time there. Record from a separate process or core, and consider skipping zstd on the car (compress offboard).
5. **Shorten the sweep if the budget demands it.** At 10 Hz the sensor needs ~110 ms per revolution before any software runs. Higher-rate sensor modes or publishing partial scans cut that. [background, not tested here]

**Caveats:** replayed pcap and emulated cameras on a laptop; a live sensor (UDP socket, PTP clocks, real camera timing) changes the absolute numbers. We did not measure end-to-end latency to the real consumer (perception), and the cross-process ROS numbers come from a Python probe.

---

## 7. Checklist for the next high-bandwidth sensor

- [ ] Find the **smallest, fastest queue** on the path and size it for the worst stall.
- [ ] Keep driver and processing **in one process**; avoid an extra hop through a topic.
- [ ] Decide who reads the data: **recorders and viewers are always other processes**.
- [ ] Enable **SHM** on both ends, size the pool, share `/dev/shm` in containers.
- [ ] Verify SHM is **really on** (it is off by default in Jazzy `rmw_zenoh`, on by default in plain Zenoh).
- [ ] Measure **completeness** (not just rate): share of valid scan columns, gaps between header stamps.
- [ ] Check what each **header stamp** really means (when is it taken, relative to scan start / exposure?) and compare it to a trace. We found a 99 ms cloud stamp error and a 12 ms camera stamp offset.
- [ ] Trace the **stages** (arrival, assembly, build, publish, delivery) before optimising: `benchmarks/pipeline/latency_trace.py`.
- [ ] Watch **disk rate**: ~125-150 MB/s raw for our setup. Use compression and splitting.
- [ ] Re-test after each change **under realistic load** (recorder + viewer), not idle.

---

## 8. Test setup and limits

- One host. A `component_container` with Ouster pcap replay + cloud node, 2 emulated Basler cameras (1080p @ 10 Hz), a consumer node and the rosbag2 recorder; `rmw_zenohd` router in a second container. SHM was enabled explicitly only for tests marked "SHM on". **All earlier baseline runs used plain network transport.**
- Metrics: share of scan columns containing finite points; gaps between header stamps (healthy = 100% columns, steady ~110 ms); end-to-end latency; process CPU from `/proc`.
- **The pcap replay runs at ~91% of real time**: scans arrive every ~109.5 ms instead of 100 ms (~9.1 Hz, ~1,150 packets/s), because the per-packet `sleep` in the replay overshoots. Both stacks use the same pacing, so comparisons are fair, but the load is ~9% lower than a real 10 Hz sensor.
- Most results are **single runs** on a laptop (58-83°C during runs, similar across scenarios). Treat numbers as indicative.
- Cross-process ROS latency was measured with a Python probe and pure Zenoh with a C++ consumer, so absolute cross-process numbers are rough. The stage trace (§4.5) uses an in-process C++ consumer.
- The ROS cloud and camera stamps are early (§4.5), so the probe's raw ROS latencies are not comparable with the pure-Zenoh ones.
- **Not done:** Foxglove with a live client; the stamp behaviour in the sensor-time and PTP modes; building the cloud inside `ouster_ros` without the PCL conversion; a wire-format design for a pure-Zenoh build; a fused pcap + cloud node for replay benchmarks.
- Reproduce: `benchmarks/README.md` (micro-benchmarks, the full pipeline comparison, and the flicker repro).
