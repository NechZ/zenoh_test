# Local patches to the driver repos

`src/ouster-ros` and `src/pylon-ros-camera` are clones of the team's GitLab repos, pinned in
`scripts/setup_sources.sh`. Our changes on top of the pinned commits live here as patches so a fresh checkout
reproduces exactly the tested state. `scripts/setup_sources.sh` applies them (and skips ones already applied).

| Patch | Applies to | What it does |
|---|---|---|
| `ouster-ros.patch` | `ouster-ros` @ `7948853` | `packet_qos_depth` parameter on `os_cloud` and `os_pcap`: queue depth of the LiDAR packet topic only (0 = unchanged). This is the fix for the point-cloud flicker, see the report. Also: `num_returns_override` parameter and bounds guards for the publisher lists; `os_pcap` auto-start triggers the lifecycle transitions directly |
| `ouster-sdk.patch` | submodule `ouster-ros/ouster-sdk` @ `0483198` | `ScanBatcher`: do not drop packets of a repeated frame id. **Needed to loop a pcap** (the replay restarts with the same frame ids); marked `HACK` in the code. Also touches one test metadata file that nothing here uses |
| `pylon-ros-camera.patch` | `pylon-ros-camera` @ `39581c0` | emulated cameras (`BaslerCamEmu`) are treated as GigE cameras; `device_user_id` also matches a camera's **serial number** (emulated cameras have no user id); new `startup_image_width` / `startup_image_height` parameters apply an ROI at startup (the emulator defaults to 1024x1040, we use 1920x1080) |

The patches contain everything that was uncommitted in the two clones when this repo was reorganised, including
changes made before the benchmarks. Do not assume each hunk is required by the benchmarks; the first group above
is the part this project depends on.

## Changing a patch

```bash
git -C src/ouster-ros diff -- . ':!ouster-ros/ouster-sdk' > patches/ouster-ros.patch
git -C src/ouster-ros/ouster-ros/ouster-sdk diff > patches/ouster-sdk.patch
git -C src/pylon-ros-camera diff > patches/pylon-ros-camera.patch
```

Cleaner long-term options: commit these changes on a branch in the GitLab repos and pin that commit instead of
patching, or turn the two clones into git submodules.
