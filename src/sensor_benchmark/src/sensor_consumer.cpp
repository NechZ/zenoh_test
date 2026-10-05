#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace sensor_benchmark {

// BENCH_TRACE=1: print when each message reaches this (in-process) consumer, keyed by its header stamp.
// benchmarks/pipeline/latency_trace.py joins these lines with the driver-side stage times.
static bool trace_enabled() {
  static const bool e = std::getenv("BENCH_TRACE") != nullptr;
  return e;
}
static int64_t trace_now_ns() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}
static void trace_arrival(const char * what, const builtin_interfaces::msg::Time & stamp) {
  if (trace_enabled()) {
    std::fprintf(stderr, "TRACE consumer what=%s stamp=%lld arrive=%lld\n", what,
      static_cast<long long>(rclcpp::Time(stamp).nanoseconds()), static_cast<long long>(trace_now_ns()));
  }
}

class SensorConsumer : public rclcpp::Node
{
public:
  explicit SensorConsumer(const rclcpp::NodeOptions & options)
  : Node("sensor_consumer", options)
  {
    cam1_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera1/image", rclcpp::SensorDataQoS(),
      std::bind(&SensorConsumer::cam1_callback, this, std::placeholders::_1));

    cam2_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "camera2/image", rclcpp::SensorDataQoS(),
      std::bind(&SensorConsumer::cam2_callback, this, std::placeholders::_1));

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "ouster/points", rclcpp::SensorDataQoS(),
      std::bind(&SensorConsumer::lidar_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "SensorConsumer started.");
  }

private:
  void cam1_callback(sensor_msgs::msg::Image::UniquePtr msg)
  {
    trace_arrival("cam1", msg->header.stamp);
    auto latency = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Cam1] Received Image: %dx%d (%.2f MB) | Latency: %.3f ms", 
      msg->width, msg->height, msg->data.size() / 1048576.0, latency);
  }

  void cam2_callback(sensor_msgs::msg::Image::UniquePtr msg)
  {
    trace_arrival("cam2", msg->header.stamp);
    auto latency = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[Cam2] Received Image: %dx%d (%.2f MB) | Latency: %.3f ms", 
      msg->width, msg->height, msg->data.size() / 1048576.0, latency);
  }

  void lidar_callback(sensor_msgs::msg::PointCloud2::UniquePtr msg)
  {
    trace_arrival("cloud", msg->header.stamp);
    auto latency = (this->now() - msg->header.stamp).seconds() * 1000.0;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[LiDAR] Received PointCloud2: %dx%d (%.2f MB) | Latency: %.3f ms", 
      msg->width, msg->height, msg->data.size() / 1048576.0, latency);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam2_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
};

} // namespace sensor_benchmark

RCLCPP_COMPONENTS_REGISTER_NODE(sensor_benchmark::SensorConsumer)
