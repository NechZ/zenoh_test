# Test data (not in git)

The benchmarks replay an Ouster recording instead of using a live sensor. Put these two files here:

| File | Size | Notes |
|---|---|---|
| `OS-1-128_v3.0.1_2048x10_20230216_143245-000.pcap` | ~4.4 GB | OS-1-128, firmware 3.0.1, 2048x10 mode, dual-return profile (`RNG19_RFL8_SIG16_NIR16_DUAL`) |
| `OS-1-128_v3.0.1_2048x10_20230216_143245.json` | ~10 KB | the metadata of **that** recording |

Source: the recording was downloaded from Ouster Studio: <https://studio.ouster.com/share/VVX59BLMFXPLVPI5>
(a share link, so it may expire or need an Ouster account; the metadata JSON comes with the same recording).

The metadata must match the pcap. A JSON from another sensor or mode (different `column_window`,
`pixel_shift_by_row` or beam angles) produces a distorted or half-empty point cloud.

Any other Ouster pcap works if you pass its metadata and change the paths:

```bash
# ROS benchmarks
ros2 launch sensor_benchmark benchmark_drivers.launch.py pcap:=/path/x.pcap metadata:=/path/x.json
# benchmarks/pipeline/run_pipeline_compare.sh
PCAP=/path/x.pcap META=/path/x.json benchmarks/pipeline/run_pipeline_compare.sh
```

The benchmark numbers in the docs were measured with the file above; other data (different resolution,
single return) changes cloud size and CPU cost.
