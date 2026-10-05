#include <algorithm>
#include <vector>
#include <sys/resource.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
static double cpu(){rusage r;getrusage(RUSAGE_SELF,&r);return r.ru_utime.tv_sec+r.ru_utime.tv_usec*1e-6+r.ru_stime.tv_sec+r.ru_stime.tv_usec*1e-6;}
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  auto n=rclcpp::Node::make_shared("shm_sub"); n->declare_parameter("secs",16.0);
  double secs=n->get_parameter("secs").as_double();
  std::vector<double> lat; auto t0=std::chrono::steady_clock::now(); double c0=cpu();
  auto sub=n->create_subscription<sensor_msgs::msg::PointCloud2>("bench_cloud",rclcpp::QoS(10).reliable(),
    [&](sensor_msgs::msg::PointCloud2::UniquePtr m){ lat.push_back((n->now()-m->header.stamp).seconds()*1000.0);});
  auto stop=n->create_wall_timer(std::chrono::duration<double>(secs),[&](){ rclcpp::shutdown(); });  // exit even if the stream stopped
  rclcpp::spin(n);
  double el=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  std::sort(lat.begin(),lat.end());
  if(lat.empty()){printf("SUB received 0\n");return 0;}
  printf("SUB n=%zu latency ms: p50=%.1f p99=%.1f max=%.1f | cpu=%.1f%% of one core\n",lat.size(),lat[lat.size()/2],lat[size_t(lat.size()*0.99)],lat.back(),100*(cpu()-c0)/el);
}
