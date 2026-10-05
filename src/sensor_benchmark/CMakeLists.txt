cmake_minimum_required(VERSION 3.8)
project(sensor_benchmark)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# In-process consumer of the cloud and both camera images (shows what an IPC subscriber sees).
add_library(sensor_consumer_component SHARED src/sensor_consumer.cpp)
ament_target_dependencies(sensor_consumer_component rclcpp rclcpp_components sensor_msgs)
rclcpp_components_register_node(sensor_consumer_component
  PLUGIN "sensor_benchmark::SensorConsumer" EXECUTABLE sensor_consumer)

install(TARGETS sensor_consumer_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
