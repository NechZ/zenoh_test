// Pure Eclipse Zenoh subscriber benchmark (no ROS).
// Usage: zsub <net|shm> <copy 0|1> <secs> [port]
// copy=1 copies the whole payload out (like a ROS subscriber deserializing a message);
// copy=0 only reads the 8-byte timestamp (what true zero-copy access would cost).
#include <sys/resource.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
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
  if (argc < 4) {
    std::fprintf(stderr, "usage: zsub <net|shm> <copy 0|1> <secs> [port]\n");
    return 1;
  }
  const bool use_shm = std::string(argv[1]) == "shm";
  const bool copy = std::atoi(argv[2]) != 0;
  const double secs = std::atof(argv[3]);
  const std::string port = argc > 4 ? argv[4] : "7461";

  Config config = Config::create_default();
  config.insert_json5("mode", "\"peer\"");
  config.insert_json5("scouting/multicast/enabled", "false");
  config.insert_json5("connect/endpoints", "[\"tcp/127.0.0.1:" + port + "\"]");
  // Zenoh enables SHM transport optimization by default, so "net" must switch it off explicitly.
  config.insert_json5("transport/shared_memory/enabled", use_shm ? "true" : "false");
  if (use_shm) config.insert_json5("transport/shared_memory/mode", "\"init\"");
  auto session = Session::open(std::move(config));

  std::mutex mu;
  std::vector<double> lat;
  size_t shm_hits = 0;
  auto sub = session.declare_subscriber(
      KeyExpr("bench/cloud"),
      [&](const Sample& s) {
        const Bytes& p = s.get_payload();
        int64_t stamp = 0;
        bool is_shm = p.as_shm().has_value();
        if (copy) {
          std::vector<uint8_t> v = p.as_vector();  // full payload copy
          std::memcpy(&stamp, v.data(), sizeof(stamp));
        } else {
          auto r = p.reader();
          r.read(reinterpret_cast<uint8_t*>(&stamp), sizeof(stamp));
        }
        double ms = (now_ns() - stamp) / 1e6;
        std::lock_guard<std::mutex> g(mu);
        lat.push_back(ms);
        if (is_shm) shm_hits++;
      },
      closures::none);

  const auto t0 = Clock::now();
  const double c0 = cpu_seconds();
  std::this_thread::sleep_for(std::chrono::duration<double>(secs));
  const double el = std::chrono::duration<double>(Clock::now() - t0).count();

  std::lock_guard<std::mutex> g(mu);
  if (lat.empty()) {
    std::printf("SUB[%s copy=%d] received 0\n", argv[1], (int)copy);
    return 0;
  }
  std::sort(lat.begin(), lat.end());
  std::printf("SUB[%s copy=%d] n=%zu (arrived as SHM: %zu) latency ms: p50=%.1f p99=%.1f max=%.1f | cpu=%.1f%% of one core\n",
              argv[1], (int)copy, lat.size(), shm_hits, lat[lat.size() / 2],
              lat[static_cast<size_t>(lat.size() * 0.99)], lat.back(), 100 * (cpu_seconds() - c0) / el);
  return 0;
}
