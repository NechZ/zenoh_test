// Consumer / recorder for zenoh_sensors.
//
// Measures the same things as the ROS probe: frame gaps (stamp to stamp), share of scan columns that
// contain points, latency (callback entry - stamp), per-stream rate. Optionally records everything to disk
// like the rosbag2 recorder: raw files split at 2 GiB, each closed file compressed with zstd.
//
// Usage: zconsumer [--shm 0|1] [--secs S] [--port P] [--check 1] [--record DIR]
#include <sys/stat.h>
#include <zstd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "wire.h"
#include "zenoh.hxx"

using Clock = std::chrono::steady_clock;

namespace {

struct Stream {
  std::vector<int64_t> stamps;
  std::vector<double> latency_ms;
  std::vector<float> cols_present;  // clouds only
  uint64_t bytes = 0;
};

// ---- recorder: raw 2 GiB files, closed files compressed with zstd (like rosbag2 file compression) ----
class Recorder {
 public:
  explicit Recorder(std::string dir) : dir_(std::move(dir)) {
    mkdir(dir_.c_str(), 0755);
    writer_ = std::thread([this] { run(); });
  }
  void push(std::vector<uint8_t>&& msg) {
    std::lock_guard<std::mutex> g(m_);
    if (queued_bytes_ > kMaxQueue) { dropped_++; return; }
    queued_bytes_ += msg.size();
    q_.push_back(std::move(msg));
    cv_.notify_one();
  }
  void finish() {
    done_ = true;
    cv_.notify_all();
    if (writer_.joinable()) writer_.join();
    for (auto& t : compressors_) t.join();
  }
  uint64_t dropped() const { return dropped_; }
  uint64_t written() const { return written_; }

 private:
  static constexpr size_t kMaxQueue = 512ull << 20;
  static constexpr uint64_t kSplit = 2ull << 30;

  void run() {
    FILE* f = nullptr;
    uint64_t in_file = 0;
    int idx = 0;
    std::string path;
    for (;;) {
      std::vector<uint8_t> msg;
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [this] { return !q_.empty() || done_; });
        if (q_.empty()) break;
        msg = std::move(q_.front());
        q_.pop_front();
        queued_bytes_ -= msg.size();
      }
      if (!f) {
        path = dir_ + "/part_" + std::to_string(idx++) + ".raw";
        f = std::fopen(path.c_str(), "wb");
        in_file = 0;
      }
      uint32_t len = static_cast<uint32_t>(msg.size());
      std::fwrite(&len, sizeof(len), 1, f);
      std::fwrite(msg.data(), 1, msg.size(), f);
      in_file += msg.size();
      written_ += msg.size();
      if (in_file >= kSplit) {  // close + compress in the background
        std::fclose(f);
        f = nullptr;
        compressors_.emplace_back([path] { compress(path); });
      }
    }
    if (f) std::fclose(f);  // last file stays raw, same as rosbag2 after a hard stop
  }

  static void compress(const std::string& path) {
    FILE* in = std::fopen(path.c_str(), "rb");
    FILE* out = std::fopen((path + ".zst").c_str(), "wb");
    if (!in || !out) return;
    ZSTD_CCtx* cctx = ZSTD_createCCtx();
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, 3);
    std::vector<char> ibuf(ZSTD_CStreamInSize()), obuf(ZSTD_CStreamOutSize());
    size_t n;
    while ((n = std::fread(ibuf.data(), 1, ibuf.size(), in)) > 0) {
      ZSTD_inBuffer ib{ibuf.data(), n, 0};
      while (ib.pos < ib.size) {
        ZSTD_outBuffer ob{obuf.data(), obuf.size(), 0};
        ZSTD_compressStream2(cctx, &ob, &ib, ZSTD_e_continue);
        std::fwrite(obuf.data(), 1, ob.pos, out);
      }
    }
    ZSTD_inBuffer ib{nullptr, 0, 0};
    size_t rem;
    do {
      ZSTD_outBuffer ob{obuf.data(), obuf.size(), 0};
      rem = ZSTD_compressStream2(cctx, &ob, &ib, ZSTD_e_end);
      std::fwrite(obuf.data(), 1, ob.pos, out);
    } while (rem);
    ZSTD_freeCCtx(cctx);
    std::fclose(in);
    std::fclose(out);
    std::remove(path.c_str());
  }

  std::string dir_;
  std::mutex m_;
  std::condition_variable cv_;
  std::deque<std::vector<uint8_t>> q_;
  size_t queued_bytes_ = 0;
  std::atomic<uint64_t> dropped_{0}, written_{0};
  bool done_ = false;
  std::thread writer_;
  std::vector<std::thread> compressors_;
};

double pct(std::vector<double> v, double p) {
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, static_cast<size_t>(v.size() * p))];
}

}  // namespace

int main(int argc, char** argv) {
  std::string port = "7461", record_dir;
  bool shm = true, check = true;
  double secs = 30;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i], v = argv[i + 1];
    if (k == "--shm") shm = v != "0";
    else if (k == "--secs") secs = std::atof(v.c_str());
    else if (k == "--port") port = v;
    else if (k == "--check") check = v != "0";
    else if (k == "--record") record_dir = v;
  }

  zenoh::Config config = zenoh::Config::create_default();
  config.insert_json5("mode", "\"peer\"");
  config.insert_json5("scouting/multicast/enabled", "false");
  config.insert_json5("connect/endpoints", "[\"tcp/127.0.0.1:" + port + "\"]");
  config.insert_json5("transport/shared_memory/enabled", shm ? "true" : "false");
  if (shm) config.insert_json5("transport/shared_memory/mode", "\"init\"");
  auto session = zenoh::Session::open(std::move(config));

  std::unique_ptr<Recorder> rec;
  if (!record_dir.empty()) rec = std::make_unique<Recorder>(record_dir);

  std::mutex mu;
  std::map<std::string, Stream> streams;
  auto sub = session.declare_subscriber(
      zenoh::KeyExpr("bench/**"),
      [&](const zenoh::Sample& s) {
        const int64_t arrival = now_ns();
        std::string key(s.get_keyexpr().as_string_view());
        const zenoh::Bytes& p = s.get_payload();

        // contiguous view of the payload: zero-copy if it arrived as SHM, otherwise one copy
        const uint8_t* ptr;
        size_t len;
        std::vector<uint8_t> tmp;
        if (auto shm_buf = p.as_shm()) {
          ptr = shm_buf->get().data();
          len = shm_buf->get().len();
        } else {
          tmp = p.as_vector();
          ptr = tmp.data();
          len = tmp.size();
        }
        if (len < sizeof(WireHeader)) return;
        WireHeader hdr;
        std::memcpy(&hdr, ptr, sizeof(hdr));

        float cols_present = -1;
        if (check && hdr.point_step == sizeof(CloudPoint) && key == "bench/cloud") {
          const auto* pts = reinterpret_cast<const CloudPoint*>(ptr + sizeof(WireHeader));
          std::vector<uint8_t> col(hdr.width, 0);
          for (uint32_t r = 0; r < hdr.height; ++r)
            for (uint32_t c = 0; c < hdr.width; ++c) {
              const CloudPoint& q = pts[static_cast<size_t>(r) * hdr.width + c];
              if (std::isfinite(q.x) && (q.x != 0 || q.y != 0)) col[c] = 1;
            }
          size_t n = 0;
          for (auto b : col) n += b;
          cols_present = static_cast<float>(n) / hdr.width;
        }
        if (rec && (key == "bench/cloud" || key.rfind("bench/cam", 0) == 0))
          rec->push(std::vector<uint8_t>(ptr, ptr + len));  // copy: SHM buffer is released after return

        std::lock_guard<std::mutex> g(mu);
        Stream& st = streams[key];
        st.stamps.push_back(hdr.stamp_ns);
        st.latency_ms.push_back((arrival - hdr.stamp_ns) / 1e6);
        if (cols_present >= 0) st.cols_present.push_back(cols_present);
        st.bytes += len;
      },
      zenoh::closures::none);

  const double c0 = cpu_seconds();
  const auto t0 = Clock::now();
  std::this_thread::sleep_for(std::chrono::duration<double>(secs));
  const double el = std::chrono::duration<double>(Clock::now() - t0).count();
  const double cpu = cpu_seconds() - c0;
  if (rec) rec->finish();

  std::lock_guard<std::mutex> g(mu);
  for (auto& [key, st] : streams) {
    int lt50 = 0, mid = 0, ge150 = 0;
    for (size_t i = 1; i < st.stamps.size(); ++i) {
      double d = (st.stamps[i] - st.stamps[i - 1]) / 1e6;
      (d < 50 ? lt50 : d < 150 ? mid : ge150)++;
    }
    const double span = st.stamps.size() > 1 ? (st.stamps.back() - st.stamps.front()) / 1e9 : 1.0;  // rate over the streaming span
    std::printf("CONSUMER %-12s n=%zu (%.1f Hz) | stamp gaps: <50ms=%d, 50-150ms=%d, >=150ms=%d | latency ms p50=%.1f p99=%.1f",
                key.c_str(), st.stamps.size(), (st.stamps.size() - 1) / span, lt50, mid, ge150, pct(st.latency_ms, 0.5),
                pct(st.latency_ms, 0.99));
    if (!st.cols_present.empty()) {
      int bad = 0;
      float mn = 1;
      for (float c : st.cols_present) { bad += c < 0.995f; mn = std::min(mn, c); }
      std::printf(" | cols_present<99.5%%: %d/%zu (min %.3f)", bad, st.cols_present.size(), mn);
    }
    std::printf("\n");
  }
  std::printf("CONSUMER process CPU=%.0f%% of one core", 100 * cpu / el);
  if (rec) std::printf(" | recorder wrote %.2f GB, dropped %llu msgs", rec->written() / 1e9, (unsigned long long)rec->dropped());
  std::printf("\n");
  return 0;
}
