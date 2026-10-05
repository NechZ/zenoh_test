// Ouster pcap replay + Basler (Pylon) cameras published directly over Eclipse Zenoh. No ROS.
//
// Mirrors what the ROS composable container does, in one process:
//   Ouster SDK:  PcapReader (paced like os_pcap) -> ScanBatcher -> ring of scans -> worker thread
//                -> XYZ LUT + destagger + field copy -> 48 B/point organized cloud (each return)
//   Pylon SDK:   one grab thread per camera -> Mono8 1080p frame
//   Zenoh:       cloud / images are built straight into a SHM buffer (shm=1) or a heap vector (shm=0)
//
// Usage: zenoh_sensors --pcap F --meta F [--shm 0|1] [--cams N] [--fps 10] [--secs S] [--warmup W] [--port P]
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

#include <ouster/lidar_scan.h>
#include <ouster/os_pcap.h>
#include <ouster/types.h>
#include <ouster/xyzlut.h>

#include "impl/cartesian.h"  // ouster_ros' cartesianT with min/max range handling

#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>

#include "wire.h"
#include "zenoh.hxx"

using Clock = std::chrono::steady_clock;
namespace core = ouster::sdk::core;

namespace {

std::atomic<bool> g_stop{false};
std::atomic<uint64_t> g_packets{0}, g_scans{0}, g_ring_drops{0}, g_clouds{0}, g_shm_fail{0};
std::vector<std::atomic<uint64_t>> g_frames(8);

std::string read_text(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("cannot read " + path);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

// One Zenoh publisher that can build its payload directly in SHM or in a heap vector.
class Out {
 public:
  Out(zenoh::Session& s, const std::string& key, zenoh::PosixShmProvider* prov)
      : pub_(s.declare_publisher(zenoh::KeyExpr(key), [] {
          auto o = zenoh::Session::PublisherOptions::create_default();
          o.congestion_control = Z_CONGESTION_CONTROL_BLOCK;  // comparable to ROS reliable QoS
          return o;
        }())),
        prov_(prov) {}

  // fill(uint8_t* dst) writes `size` bytes. Returns false if the SHM pool is exhausted.
  template <class Fill>
  bool publish(size_t size, Fill&& fill) {
    if (prov_) {
      auto res = prov_->alloc_gc_defrag_blocking(size);
      auto* buf = std::get_if<zenoh::ZShmMut>(&res);
      if (!buf) {
        g_shm_fail++;
        return false;
      }
      fill(buf->data());
      pub_.put(zenoh::Bytes(std::move(*buf)));
    } else {
      std::vector<uint8_t> v(size);
      fill(v.data());
      pub_.put(zenoh::Bytes(std::move(v)));
    }
    return true;
  }

 private:
  zenoh::Publisher pub_;
  zenoh::PosixShmProvider* prov_;
};

// ---------------------------------------------------------------------------------------------
// Ouster: pcap -> scans -> clouds
// ---------------------------------------------------------------------------------------------
class OusterPipeline {
 public:
  OusterPipeline(const std::string& pcap, const std::string& meta, std::vector<std::unique_ptr<Out>> outs)
      : info_(read_text(meta)),
        pf_(core::get_format(info_)),
        pcap_path_(pcap),
        outs_(std::move(outs)),
        w_(info_.format.columns_per_frame),
        h_(info_.format.pixels_per_column),
        pixel_shift_(info_.format.pixel_shift_by_row),
        ring_size_(10) {
    auto lut = core::make_xyz_lut(w_, h_, core::RANGE_UNIT, info_.beam_to_lidar_transform,
                                  core::mat4d::Identity(), info_.beam_azimuth_angles,
                                  info_.beam_altitude_angles);
    dir_ = lut.direction.cast<float>();
    off_ = lut.offset.cast<float>();
    points_ = core::PointCloudXYZf(dir_.rows(), off_.cols());
    num_returns_ = info_.num_returns();
    for (int i = 0; i < ring_size_; ++i)
      scans_.push_back(std::make_unique<core::LidarScan>(w_, h_, info_.format.udp_profile_lidar));
    batcher_ = std::make_unique<core::ScanBatcher>(info_);
    std::printf("[ouster] %zux%zu, %d return(s), packet %zu B\n", w_, h_, num_returns_,
                static_cast<size_t>(pf_.lidar_packet_size));
  }

  void start() {
    worker_ = std::thread([this] { worker(); });
    reader_ = std::thread([this] { reader(); });
  }
  void join() {
    if (reader_.joinable()) reader_.join();
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
  }

 private:
  // Same packet pacing as ouster_ros' os_pcap node (sleep for the recorded inter-packet gap).
  void reader() {
    ouster::sdk::pcap::PcapReader pcap(pcap_path_);
    core::LidarPacket packet(pf_.lidar_packet_size);
    packet.format = std::make_shared<core::PacketFormat>(pf_);
    while (!g_stop) {
      size_t payload = pcap.next_packet();
      auto pi = pcap.current_info();
      while (payload && !g_stop) {
        auto t_start = std::chrono::high_resolution_clock::now();
        if (pi.dst_port == info_.config.udp_port_lidar) {
          std::memcpy(packet.buf.data(), pcap.current_data(), pf_.lidar_packet_size);
          g_packets++;
          on_packet(packet);
        }
        auto prev_ts = pi.timestamp;
        payload = pcap.next_packet();
        pi = pcap.current_info();
        auto dt = (pi.timestamp - prev_ts) - (std::chrono::high_resolution_clock::now() - t_start);
        if (dt.count() > 0) std::this_thread::sleep_for(dt);
      }
      pcap.reset();  // loop the file like `loop: true`
    }
  }

  void on_packet(const core::LidarPacket& packet) {
    {
      std::lock_guard<std::mutex> g(m_);
      if (count_ == ring_size_) {  // ouster_ros: "lidar_scans full, DROPPING PACKET"
        g_ring_drops++;
        return;
      }
    }
    // ouster_ros stamps a cloud with the (estimated) arrival time of the scan's first column, so
    // the stamp here is the first-packet time of the scan, not its completion time.
    if (!started_[write_]) {
      start_ns_[write_] = now_ns();
      started_[write_] = true;
    }
    if (!(*batcher_)(packet, *scans_[write_])) return;
    stamps_[write_] = start_ns_[write_];
    started_[write_] = false;
    write_ = (write_ + 1) % ring_size_;
    g_scans++;
    {
      std::lock_guard<std::mutex> g(m_);
      count_++;
    }
    cv_.notify_one();
  }

  void worker() {
    while (true) {
      {
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait_for(lk, std::chrono::seconds(1), [this] { return count_ > 0 || g_stop; });
        if (count_ == 0) {
          if (g_stop) return;
          continue;
        }
      }
      process(*scans_[read_], stamps_[read_]);
      read_ = (read_ + 1) % ring_size_;
      std::lock_guard<std::mutex> g(m_);
      count_--;
    }
  }

  void process(const core::LidarScan& scan, int64_t stamp) {
    const auto ts = scan.timestamp();
    uint64_t scan_ts = 0;
    for (int i = 0; i < ts.size(); ++i)
      if (ts.data()[i] != 0) { scan_ts = ts.data()[i]; break; }

    const uint8_t* refl = scan.field<uint8_t>(core::ChanField::REFLECTIVITY).data();
    const uint16_t* sig = scan.field<uint16_t>(core::ChanField::SIGNAL).data();
    const uint16_t* nir = scan.field<uint16_t>(core::ChanField::NEAR_IR).data();

    for (int r = 0; r < num_returns_ && r < static_cast<int>(outs_.size()); ++r) {
      auto range_img = scan.field<uint32_t>(r == 0 ? core::ChanField::RANGE : core::ChanField::RANGE2);
      const uint32_t* range = range_img.data();
      ouster::cartesianT(points_, range_img, dir_, off_, 0u, 1000000u, std::numeric_limits<float>::quiet_NaN());
      const size_t bytes = sizeof(WireHeader) + w_ * h_ * sizeof(CloudPoint);
      bool ok = outs_[r]->publish(bytes, [&](uint8_t* dst) {
        WireHeader hdr{stamp, static_cast<uint32_t>(w_), static_cast<uint32_t>(h_), sizeof(CloudPoint),
                       static_cast<uint32_t>(g_scans.load()), 0};
        std::memcpy(dst, &hdr, sizeof(hdr));
        auto* pt = reinterpret_cast<CloudPoint*>(dst + sizeof(WireHeader));
        for (size_t u = 0; u < h_; ++u) {
          for (size_t v = 0; v < w_; ++v) {  // organized + destaggered, like ouster_ros
            const size_t vs = (v + w_ - pixel_shift_[u]) % w_;
            const size_t src = u * w_ + vs;
            CloudPoint& p = pt[u * w_ + v];
            p.x = points_(src, 0);
            p.y = points_(src, 1);
            p.z = points_(src, 2);
            p.intensity = sig[src];
            p.t = ts.data()[vs] > scan_ts ? static_cast<uint32_t>(ts.data()[vs] - scan_ts) : 0;
            p.reflectivity = refl[src];
            p.ring = static_cast<uint16_t>(u);
            p.ambient = nir[src];
            p.range = range[src];
          }
        }
      });
      if (ok && r == 0) g_clouds++;
    }
  }

  core::SensorInfo info_;
  core::PacketFormat pf_;
  std::string pcap_path_;
  std::vector<std::unique_ptr<Out>> outs_;
  size_t w_, h_;
  std::vector<int> pixel_shift_;
  int num_returns_ = 1;
  core::ArrayX3R<float> dir_, off_;
  core::PointCloudXYZf points_;

  std::vector<std::unique_ptr<core::LidarScan>> scans_;
  std::unique_ptr<core::ScanBatcher> batcher_;
  int64_t stamps_[64]{}, start_ns_[64]{};
  bool started_[64]{};
  int ring_size_, write_ = 0, read_ = 0, count_ = 0;
  std::mutex m_;
  std::condition_variable cv_;
  std::thread reader_, worker_;
};

// ---------------------------------------------------------------------------------------------
// Pylon: one grab thread per emulated camera
// ---------------------------------------------------------------------------------------------
void camera_thread(int idx, double fps, Out* out) {
  try {
    Pylon::CTlFactory& f = Pylon::CTlFactory::GetInstance();
    Pylon::DeviceInfoList_t devs;
    if (f.EnumerateDevices(devs) <= static_cast<size_t>(idx)) {
      std::fprintf(stderr, "[cam%d] device not found (set PYLON_CAMEMU=N)\n", idx);
      return;
    }
    Pylon::CBaslerUniversalInstantCamera cam(f.CreateDevice(devs[idx]));
    cam.Open();
    cam.Width.SetValue(1920);
    cam.Height.SetValue(1080);
    cam.AcquisitionFrameRateEnable.TrySetValue(true);
    cam.AcquisitionFrameRate.TrySetValue(fps);
    cam.StartGrabbing(Pylon::GrabStrategy_OneByOne);
    Pylon::CGrabResultPtr res;
    uint32_t seq = 0;
    while (!g_stop && cam.IsGrabbing()) {
      if (!cam.RetrieveResult(2000, res, Pylon::TimeoutHandling_Return) || !res->GrabSucceeded()) continue;
      const int64_t stamp = now_ns();
      const uint32_t w = res->GetWidth(), h = res->GetHeight();
      const size_t px = static_cast<size_t>(w) * h;
      out->publish(sizeof(WireHeader) + px, [&](uint8_t* dst) {
        WireHeader hdr{stamp, w, h, 1, seq++, 0};
        std::memcpy(dst, &hdr, sizeof(hdr));
        std::memcpy(dst + sizeof(WireHeader), res->GetBuffer(), px);
      });
      g_frames[idx]++;
    }
    cam.StopGrabbing();
    cam.Close();
  } catch (const GenICam::GenericException& e) {
    std::fprintf(stderr, "[cam%d] %s\n", idx, e.GetDescription());
  }
}

}  // namespace

int main(int argc, char** argv) {
  std::string pcap, meta, port = "7461";
  bool shm = true;
  int cams = 2;
  double fps = 10, secs = 30, warmup = 5;
  for (int i = 1; i + 1 < argc; i += 2) {
    std::string k = argv[i], v = argv[i + 1];
    if (k == "--pcap") pcap = v;
    else if (k == "--meta") meta = v;
    else if (k == "--shm") shm = v != "0";
    else if (k == "--cams") cams = std::atoi(v.c_str());
    else if (k == "--fps") fps = std::atof(v.c_str());
    else if (k == "--secs") secs = std::atof(v.c_str());
    else if (k == "--warmup") warmup = std::atof(v.c_str());
    else if (k == "--port") port = v;
  }
  if (pcap.empty() || meta.empty()) {
    std::fprintf(stderr, "usage: zenoh_sensors --pcap F --meta F [--shm 0|1] [--cams N] [--fps 10] [--secs S] [--warmup W] [--port P]\n");
    return 1;
  }

  zenoh::Config config = zenoh::Config::create_default();
  config.insert_json5("mode", "\"peer\"");
  config.insert_json5("scouting/multicast/enabled", "false");
  config.insert_json5("listen/endpoints", "[\"tcp/127.0.0.1:" + port + "\"]");
  config.insert_json5("transport/shared_memory/enabled", shm ? "true" : "false");
  if (shm) config.insert_json5("transport/shared_memory/mode", "\"init\"");
  auto session = zenoh::Session::open(std::move(config));

  std::optional<zenoh::PosixShmProvider> provider;
  if (shm) provider.emplace(512ull * 1024 * 1024);
  zenoh::PosixShmProvider* prov = shm ? &*provider : nullptr;

  Pylon::PylonInitialize();
  std::vector<std::unique_ptr<Out>> cloud_outs;
  cloud_outs.push_back(std::make_unique<Out>(session, "bench/cloud", prov));
  cloud_outs.push_back(std::make_unique<Out>(session, "bench/cloud2", prov));
  std::vector<std::unique_ptr<Out>> cam_outs;
  for (int i = 0; i < cams; ++i) cam_outs.push_back(std::make_unique<Out>(session, "bench/cam" + std::to_string(i), prov));

  std::printf("[zenoh_sensors] shm=%d cams=%d fps=%.1f, waiting %.0fs for consumers\n", shm, cams, fps, warmup);
  std::this_thread::sleep_for(std::chrono::duration<double>(warmup));

  const double c0 = cpu_seconds();
  const auto t0 = Clock::now();
  OusterPipeline ouster(pcap, meta, std::move(cloud_outs));
  std::vector<std::thread> cam_threads;
  for (int i = 0; i < cams; ++i) cam_threads.emplace_back(camera_thread, i, fps, cam_outs[i].get());
  ouster.start();

  std::this_thread::sleep_for(std::chrono::duration<double>(secs));
  g_stop = true;
  const double el = std::chrono::duration<double>(Clock::now() - t0).count();
  const double cpu = cpu_seconds() - c0;
  ouster.join();
  for (auto& t : cam_threads) t.join();
  Pylon::PylonTerminate();

  std::printf("PIPELINE[shm=%d] %.1fs: cloud scans=%llu (%.1f Hz) packets=%llu ring_drops=%llu shm_alloc_fail=%llu",
              shm, el, (unsigned long long)g_clouds.load(), g_clouds.load() / el,
              (unsigned long long)g_packets.load(), (unsigned long long)g_ring_drops.load(),
              (unsigned long long)g_shm_fail.load());
  for (int i = 0; i < cams; ++i) std::printf(" cam%d=%llu (%.1f Hz)", i, (unsigned long long)g_frames[i].load(), g_frames[i].load() / el);
  std::printf(" | process CPU=%.0f%% of one core\n", 100 * cpu / el);
  std::this_thread::sleep_for(std::chrono::seconds(2));  // let consumers drain
  return 0;
}
