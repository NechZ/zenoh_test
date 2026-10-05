cmake_minimum_required(VERSION 3.16)
project(zenoh_sensors CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
option(CMAKE_POSITION_INDEPENDENT_CODE "Build position independent code." ON)

# ---- Ouster SDK (built from the vendored source, same way ouster_ros does) ----
set(OUSTER_ROS_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../src/ouster-ros/ouster-ros CACHE PATH "ouster_ros source dir (see scripts/setup_sources.sh)")
get_filename_component(OUSTER_ROS_DIR "${OUSTER_ROS_DIR}" ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH ${OUSTER_ROS_DIR}/ouster-sdk/cmake)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_PCAP ON CACHE BOOL "" FORCE)
set(BUILD_VIZ OFF CACHE BOOL "" FORCE)
set(BUILD_OSF OFF CACHE BOOL "" FORCE)
set(BUILD_MAPPING OFF CACHE BOOL "" FORCE)
find_package(OusterSDK REQUIRED)
add_definitions(-DEIGEN_MPL2_ONLY)

# ---- Zenoh (zenoh-c / zenoh-cpp from ros-jazzy-zenoh-cpp-vendor) ----
set(ZENOH_ROOT /opt/ros/jazzy/opt/zenoh_cpp_vendor CACHE PATH "zenoh-c/zenoh-cpp install prefix")

# ---- Pylon ----
set(PYLON_ROOT /opt/pylon CACHE PATH "Basler pylon install prefix")
file(GLOB PYLON_GENAPI ${PYLON_ROOT}/lib/libGenApi_gcc_*_Basler_pylon*.so)
file(GLOB PYLON_GCBASE ${PYLON_ROOT}/lib/libGCBase_gcc_*_Basler_pylon*.so)

add_executable(zenoh_sensors zenoh_sensors.cpp)
target_compile_definitions(zenoh_sensors PRIVATE ZENOHCXX_ZENOHC)
target_include_directories(zenoh_sensors PRIVATE
  ${ZENOH_ROOT}/include ${PYLON_ROOT}/include ${OUSTER_ROS_DIR}/src ${OUSTER_ROS_DIR}/ouster-sdk/ouster_pcap/include)
target_link_directories(zenoh_sensors PRIVATE ${ZENOH_ROOT}/lib ${PYLON_ROOT}/lib)
target_link_libraries(zenoh_sensors PRIVATE ouster_client ouster_pcap zenohc
  pylonbase pylonutility ${PYLON_GENAPI} ${PYLON_GCBASE} pthread)
set_target_properties(zenoh_sensors PROPERTIES BUILD_RPATH "${ZENOH_ROOT}/lib;${PYLON_ROOT}/lib")

add_executable(zconsumer zconsumer.cpp)
target_compile_definitions(zconsumer PRIVATE ZENOHCXX_ZENOHC)
target_include_directories(zconsumer PRIVATE ${ZENOH_ROOT}/include)
target_link_directories(zconsumer PRIVATE ${ZENOH_ROOT}/lib)
target_link_libraries(zconsumer PRIVATE zenohc zstd pthread)
set_target_properties(zconsumer PROPERTIES BUILD_RPATH "${ZENOH_ROOT}/lib")
