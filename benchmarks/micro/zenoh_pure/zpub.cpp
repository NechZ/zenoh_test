// Pure Eclipse Zenoh publisher benchmark (no ROS).
// Usage: zpub <net|shm> <mb> <hz> <secs> <warmup_secs> [port]
// Publishes <mb> MB payloads at <hz>; first 8 bytes carry a system_clock timestamp (ns).
#include <sys/resource.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include "zenoh.hxx"

using namespace zenoh;
using Clock = std::chrono::steady_clock;

static double cpu_seconds() {
  rusage r;
  getrusage(RUSAGE_SELF, &r);
  return r.ru_utime.tv_sec + r.ru_utime.tv_usec * 1e-6 + r.ru_stime.tv_sec + r.ru_stime.tv_usec * 1e-6;
}

static int64_t now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch()).count();
}

int main(int argc, char** argv) {
  if (argc < 6) {
    std::fprintf(stderr, "usage: zpub <net|shm> <mb> <hz> <secs> <warmup_secs> [port]\n");
    return 1;
  }
  const bool use_shm = std::string(argv[1]) == "shm";
  const double mb = std::atof(argv[2]);
  const double hz = std::atof(argv[3]);
  const double secs = std::atof(argv[4]);
  const double warmup = std::atof(argv[5]);
  const std::string port = argc > 6 ? argv[6] : "7461";
  const size_t size = std::max<size_t>(64, static_cast<size_t>(mb * 1048576.0));

  Config config = Config::create_default();
  config.insert_json5("mode", "\"peer\"");
  config.insert_json5("scouting/multicast/enabled", "false");
  config.insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:" + port + "\"]");
  // Zenoh enables SHM transport optimization by default, so "net" must switch it off explicitly.
  config.insert_json5("transport/shared_memory/enabled", use_shm ? "true" : "false");
  if (use_shm) config.insert_json5("transport/shared_memory/mode", "\"init\"");
  auto session = Session::open(std::move(config));

  Session::PublisherOptions opts = Session::PublisherOptions::create_default();
  opts.congestion_control = Z_CONGESTION_CONTROL_BLOCK;  // comparable to ROS reliable QoS
  auto pub = session.declare_publisher(KeyExpr("bench/cloud"), std::move(opts));

  std::vector<uint8_t> tmpl(size, 7);  // stands in for the sensor data
  std::optional<PosixShmProvider> provider;
  if (use_shm) provider.emplace(256ull * 1024 * 1024);  // 256 MB pool, like the ROS test

  std::this_thread::sleep_for(std::chrono::duration<double>(warmup));  // let subscribers connect

  const auto t0 = Clock::now();
  const double c0 = cpu_seconds();
  size_t sent = 0, alloc_fail = 0;
  const auto period = std::chrono::duration<double>(1.0 / hz);
  auto next = t0;
  while (Clock::now() - t0 < std::chrono::duration<double>(secs)) {
    next += std::chrono::duration_cast<Clock::duration>(period);
    if (use_shm) {
      auto res = provider->alloc_gc_defrag_blocking(size);
      auto* buf = std::get_if<ZShmMut>(&res);
      if (!buf) { alloc_fail++; std::this_thread::sleep_until(next); continue; }
      std::memcpy(buf->data(), tmpl.data(), size);
      int64_t t = now_ns();
      std::memcpy(buf->data(), &t, sizeof(t));
      pub.put(Bytes(std::move(*buf)));
    } else {
      std::vector<uint8_t> v(tmpl);  // same one 12 MB fill as the ROS publisher's message copy
      int64_t t = now_ns();
      std::memcpy(v.data(), &t, sizeof(t));
      pub.put(Bytes(std::move(v)));
    }
    sent++;
    std::this_thread::sleep_until(next);
  }
  const double el = std::chrono::duration<double>(Clock::now() - t0).count();
  std::printf("PUB[%s] sent=%zu msg=%.2fMB alloc_fail=%zu cpu=%.1f%% of one core\n", argv[1], sent,
              size / 1048576.0, alloc_fail, 100 * (cpu_seconds() - c0) / el);
  std::this_thread::sleep_for(std::chrono::seconds(2));  // let subscribers drain
  return 0;
}
