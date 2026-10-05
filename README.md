# Sensor transport benchmarks: ROS 2 + Zenoh vs pure Eclipse Zenoh

Reproducible benchmarks for moving high-bandwidth sensor data (Ouster LiDAR clouds, Basler camera images)
through ROS 2 (`rmw_zenoh`, composable nodes, intra-process comms, shared memory) and through plain
Eclipse Zenoh built directly on the Ouster and Pylon SDKs. No hardware needed: the LiDAR is replayed from a
pcap and the cameras are Basler's built-in emulator.

**What we learned** is written up in [docs/transport-ipc-report.md](docs/transport-ipc-report.md) (start there).
**How to run each benchmark** is in [benchmarks/README.md](benchmarks/README.md).

## Quick start

You need Docker with the compose plugin, about 15 GB of disk, and three things that are not in git:

| What | Where | Details |
|---|---|---|
| Basler pylon SDK installers | `third_party/pylon/` | [third_party/pylon/README.md](third_party/pylon/README.md) |
| Ouster pcap + matching metadata JSON | `data/` | [data/README.md](data/README.md) |
| Access to the team GitLab (ouster-ros, pylon-ros-camera) | your ssh key, used on the host | `scripts/setup_sources.sh` |

```bash
# 1. on the host: fetch the two driver repos at the pinned commits and apply our patches
scripts/setup_sources.sh

# 2. build the image and start the Zenoh router + the work container
docker compose build ros2
docker compose up -d zenoh ros2
docker compose exec ros2 bash

# 3. inside the container: build everything (ROS packages + the pure-Zenoh programs), ~10 min first time
scripts/build.sh
source install/setup.bash

# 4. run a benchmark, for example the end-to-end comparison
benchmarks/pipeline/run_pipeline_compare.sh
```

Optional viewer: `docker compose --profile viewer up -d foxglove` starts a Foxglove bridge on port 8765 with a
send buffer large enough for a 12 MB point cloud.

## What is in the repo

```
docker/Dockerfile        build environment (ROS 2 Jazzy, rmw_zenoh, Ouster deps, pylon SDK, Foxglove bridge)
docker-compose.yml       services: ros2 (work container), zenoh (router), foxglove (optional viewer)
scripts/                 setup_sources.sh (clone + patch drivers), build.sh, zenoh_env.sh (SHM on/off)
src/
  sensor_benchmark/      ROS 2 launch file + in-process consumer used by the ROS benchmarks
  ouster-ros/            (cloned by setup_sources.sh) Ouster ROS driver + SDK
  pylon-ros-camera/      (cloned by setup_sources.sh) Basler ROS driver
patches/                 our local changes to the two drivers, see patches/README.md
benchmarks/
  micro/                 transport micro-benchmarks: ROS (rmw_zenoh) vs pure Zenoh, one big message stream
  pipeline/              full sensor pipeline: ROS drivers vs a pure-Zenoh app on the Ouster/Pylon SDKs
data/                    pcap + metadata (not in git)
third_party/pylon/       pylon installers (not in git)
docs/                    the write-up
```

## Things that commonly go wrong

- **`docker compose build` fails with "put pylon_*.deb ... into third_party/pylon/"**: the installers are missing, see
  [third_party/pylon/README.md](third_party/pylon/README.md).
- **No cameras found**: the emulator needs `PYLON_CAMEMU=2`. The image and compose file set it; if you start
  processes some other way, export it.
- **Benchmarks hang or print nothing**: the router must be running (`docker compose up -d zenoh`).
- **SHM seems to do nothing**: `rmw_zenoh` has SHM *off* by default. Use `source scripts/zenoh_env.sh shm` in
  *every* shell that starts a ROS process (the benchmark scripts do this for you).
- **`error setting scheduling priority ... OS(1)` warnings** from the Zenoh watchdog are harmless in containers.
- **Files created in the container are owned by root** (the repo is bind-mounted). Fix with `sudo chown -R $USER .`
  on the host if needed.
- **Numbers jump around between runs**: the benchmarks use the whole CPU for tens of seconds. Let the machine
  cool down (the scripts log the package temperature) and repeat a run before trusting a small difference.
