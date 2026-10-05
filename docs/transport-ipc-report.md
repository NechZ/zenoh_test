# Moving high-bandwidth sensor data: ROS 2, Zenoh, IPC and shared memory

*A learning document for the team. Date: 2026-10-05 · ROS 2 Jazzy · `rmw_zenoh_cpp` 0.2.10 · Reference sensors: Ouster OS-1-128 (12 MB cloud @ ~9-10 Hz) and 2x Basler 1080p @ 10 Hz.*

How to read it: **[measured]** = we saw it in our own tests (details and scripts in `benchmarks/`). **[background]** = general ROS 2 / Zenoh knowledge we did not re-test.

---

## 1. The short version

1. **Data is lost at the smallest queue on the path, not at the biggest message.** Our flicker was LiDAR *packets* (33 KB, ~1,150/s) overflowing a queue with ~8 ms of room. [measured]
2. **Size queues for the stall you must survive**, not for the average rate. A deeper packet queue (512) fixed it completely. [measured]
3. **Shared memory (SHM) works with normal variable-size messages** (clouds, images). It is *off by default* in ROS 2 Jazzy's `rmw_zenoh`, and turning it on cuts publisher cost and latency a lot. [measured]
4. **True zero-copy needs "loaned messages", which `rmw_zenoh` 0.2.10 does not support**, even for fixed-size types. [measured]
5. **Pure Eclipse Zenoh (no ROS) is not magic.** Without SHM it cost about the same as ROS. With SHM it was about 2x cheaper on the pipeline and much faster, but you give up the ROS ecosystem. **Our verdict: stay on ROS, enable SHM, size the queues. Consider pure Zenoh only for one hot link.** (§6)

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
| Cloud latency, median | 257 ms | 244 ms | 127 ms | **113 ms** |
| Camera latency, median | 16-18 ms | 14-16 ms | 4-5 ms | **0.6 ms** |
| Incomplete scans / rates | 0 / 9 Hz, 10 Hz | 0 / same | 0 / same | 0 / same |

How to read it:

- **Without SHM the two are equal** (0.60 vs 0.62 cores). The ROS layer is not expensive by itself.
- **With SHM the pure version needs half the pipeline CPU**, because it builds the cloud directly in the shared buffer and readers use it in place.
- **The recorder costs ~0.6-0.7 cores in both stacks** (zstd compression). That caps what any rewrite can save: total 1.06 → 0.77 cores.
- **The ~130 ms cloud latency gap sits inside the ROS driver, not in the transport**: an in-process consumer, with no transport at all, saw similar numbers earlier. A 9 Hz scan takes ~111 ms to build, so pure Zenoh delivers within a few ms of scan completion while `ouster_ros` adds ~130 ms. We did **not** profile why (likely its scan ring buffer, PCL conversion and message copy, but that is unconfirmed).

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

**Live Ouster lidar:** use the **`os_driver`** node. It receives the UDP packets and builds the cloud in one node, so there is **no packet topic that can drop** (the problem in §4.1 comes from our pcap *replay*, which sends packets over a topic). Also: enlarge the kernel UDP receive buffer (`net.core.rmem_max`) [background]; use `min_scan_valid_columns_ratio` to skip scans that are mostly empty; and use the metadata file that matches the sensor and mode, since a wrong one distorts or splits the cloud.

---

## 6. Verdict: should we write this in pure Eclipse Zenoh?

**What pure Zenoh gives you**
- About **half the pipeline CPU** with SHM (0.21 vs 0.44 cores) and ~0.3 cores less in total; far lower latency (camera 0.6 ms vs ~15 ms; cloud ~110 vs ~245 ms).
- **True zero-copy reads** from shared memory, which ROS cannot do today.
- Direct control over queues, drop policy and memory.

**What you give up**
- The **ROS ecosystem**: the Ouster and Pylon ROS drivers, rosbag2, Foxglove integration, TF, launch files and parameters. Our pure app does only what the benchmark needs (no IMU, TF, parameters, lifecycle or diagnostics).
- A **wire format**: Zenoh moves bytes, so you design (and version) your own message format. Cheap for flat sensor buffers, a real cost for everything else, and **not measured here**.
- **Tooling and hiring familiarity.**

**Verdict**
1. **Stay on ROS 2.** Without SHM the stacks cost the same. With SHM and deep packet queues ROS had no drops and kept 9-10 Hz everywhere.
2. **The first and cheapest wins are not a rewrite:** enable SHM, size the queues, use `os_driver` for the live sensor.
3. **Pure Zenoh makes sense for a single hot link** (for example cloud → perception) behind a thin bridge, *if* measured CPU or latency there is a real problem. Compared with the effort, the total saving is modest because the recorder dominates.
4. **If latency matters,** look at the driver first: ~130 ms of the cloud latency is inside `ouster_ros`, not the transport. (Unprofiled.)

---

## 7. Checklist for the next high-bandwidth sensor

- [ ] Find the **smallest, fastest queue** on the path and size it for the worst stall.
- [ ] Keep driver and processing **in one process**; avoid an extra hop through a topic.
- [ ] Decide who reads the data: **recorders and viewers are always other processes**.
- [ ] Enable **SHM** on both ends, size the pool, share `/dev/shm` in containers.
- [ ] Verify SHM is **really on** (it is off by default in Jazzy `rmw_zenoh`, on by default in plain Zenoh).
- [ ] Measure **completeness** (not just rate): share of valid scan columns, gaps between header stamps.
- [ ] Watch **disk rate**: ~125-150 MB/s raw for our setup. Use compression and splitting.
- [ ] Re-test after each change **under realistic load** (recorder + viewer), not idle.

---

## 8. Test setup and limits

- One host. A `component_container` with Ouster pcap replay + cloud node, 2 emulated Basler cameras (1080p @ 10 Hz), a consumer node and the rosbag2 recorder; `rmw_zenohd` router in a second container. SHM was enabled explicitly only for tests marked "SHM on". **All earlier baseline runs used plain network transport.**
- Metrics: share of scan columns containing finite points; gaps between header stamps (healthy = 100% columns, steady ~110 ms); end-to-end latency; process CPU from `/proc`.
- Most results are **single runs** on a laptop (58-83°C during runs, similar across scenarios). Treat numbers as indicative.
- ROS latency was measured with a Python probe, pure Zenoh with a C++ consumer. The in-process C++ consumer's earlier numbers matched, so probe overhead does not explain the gaps.
- **Not done:** Foxglove with a live client; profiling inside `ouster_ros`; a wire-format design for a pure-Zenoh build; a fused pcap + cloud node for replay benchmarks.
- Reproduce: `benchmarks/README.md` (micro-benchmarks, the full pipeline comparison, and the flicker repro).
