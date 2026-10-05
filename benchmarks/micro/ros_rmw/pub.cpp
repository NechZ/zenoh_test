#include <chrono>
#include <sys/resource.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float64.hpp"
using namespace std::chrono_literals;
static double cpu(){rusage r;getrusage(RUSAGE_SELF,&r);return r.ru_utime.tv_sec+r.ru_utime.tv_usec*1e-6+r.ru_stime.tv_sec+r.ru_stime.tv_usec*1e-6;}
int main(int argc,char**argv){
  rclcpp::init(argc,argv);
  auto n=rclcpp::Node::make_shared("shm_pub");
  n->declare_parameter("mb",12.0); n->declare_parameter("hz",10.0); n->declare_parameter("secs",12.0);
  double mb=n->get_parameter("mb").as_double(),hz=n->get_parameter("hz").as_double(),secs=n->get_parameter("secs").as_double();
  auto qos=rclcpp::QoS(10).reliable();
  auto pub=n->create_publisher<sensor_msgs::msg::PointCloud2>("bench_cloud",qos);
  auto fpub=n->create_publisher<std_msgs::msg::Float64>("bench_fixed",qos);
  RCLCPP_INFO(n->get_logger(),"can_loan PointCloud2=%d  Float64(fixed-size)=%d",(int)pub->can_loan_messages(),(int)fpub->can_loan_messages());
  sensor_msgs::msg::PointCloud2 tmpl; tmpl.height=128; tmpl.width=static_cast<uint32_t>(mb*1048576/(128*48)); tmpl.point_step=48; tmpl.row_step=tmpl.width*48;
  tmpl.data.assign(static_cast<size_t>(tmpl.row_step)*128,7); tmpl.header.frame_id="x";
  auto t0=std::chrono::steady_clock::now(); double c0=cpu(); size_t sent=0;
  auto tm=n->create_wall_timer(std::chrono::duration<double>(1.0/hz),[&](){
    auto m=std::make_unique<sensor_msgs::msg::PointCloud2>(tmpl); m->header.stamp=n->now(); pub->publish(std::move(m)); sent++;
    if(std::chrono::steady_clock::now()-t0>std::chrono::duration<double>(secs)) rclcpp::shutdown();});
  rclcpp::spin(n);
  double el=std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
  printf("PUB sent=%zu msg=%.1fMB cpu=%.1f%% of one core\n",sent,tmpl.data.size()/1048576.0,100*(cpu()-c0)/el);
}
