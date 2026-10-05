// Wire format shared by zenoh_sensors (publisher) and zconsumer (subscriber).
// Every payload = WireHeader followed by raw data (cloud: CloudPoint[w*h], image: Mono8 pixels).
#pragma once
#include <sys/resource.h>

#include <chrono>
#include <cstdint>

struct WireHeader {
  int64_t stamp_ns;      // system_clock ns: first packet of the scan (cloud) / grab time (image)
  uint32_t width;
  uint32_t height;
  uint32_t point_step;   // bytes per point (cloud) or 1 (Mono8 image)
  uint32_t seq;
  uint64_t reserved;
};
static_assert(sizeof(WireHeader) == 32, "WireHeader must be 32 bytes");

// Same size as the ouster_ros "original" point (48 B) so payloads match the ROS PointCloud2.
struct CloudPoint {
  float x, y, z, intensity;
  uint32_t t;
  uint16_t reflectivity, ring, ambient, pad0;
  uint32_t range;
  uint8_t pad[16];
};
static_assert(sizeof(CloudPoint) == 48, "CloudPoint must be 48 bytes");

inline int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

inline double cpu_seconds() {
  rusage r;
  getrusage(RUSAGE_SELF, &r);
  return r.ru_utime.tv_sec + r.ru_utime.tv_usec * 1e-6 + r.ru_stime.tv_sec + r.ru_stime.tv_usec * 1e-6;
}
