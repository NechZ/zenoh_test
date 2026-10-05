# Build + benchmark environment: ROS 2 Jazzy, rmw_zenoh, Ouster SDK deps, Basler pylon SDK, Foxglove bridge.
# Build context is the repo root (see docker-compose.yml). The pylon installers are NOT in git:
# put them in third_party/pylon/ (see third_party/pylon/README.md).
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# ---- toolchain -------------------------------------------------------------------------------
RUN apt-get update && apt-get install -y \
    git \
    openssh-client \
    build-essential \
    cmake \
    gdb \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS packages: transport, recording, viewer, and what ouster_ros / pylon_ros2_camera need --
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-zenoh-cpp \
    ros-jazzy-zenoh-cpp-vendor \
    ros-jazzy-rosbag2 \
    ros-jazzy-rosbag2-storage-mcap \
    ros-jazzy-rosbag2-compression-zstd \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-image-geometry \
    ros-jazzy-camera-info-manager \
    ros-jazzy-diagnostic-updater \
    ros-jazzy-pcl-conversions \
    ros-jazzy-pcl-ros \
    ros-jazzy-tf2-eigen \
    && rm -rf /var/lib/apt/lists/*

# ---- system libraries: Ouster SDK (pcap replay, JSON, HTTP) and the pure-Zenoh benchmarks ------
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libjsoncpp-dev \
    libcurl4-openssl-dev \
    libpng-dev \
    libpcap-dev \
    libtins-dev \
    libzip-dev \
    libpcl-dev \
    libopencv-dev \
    libboost-dev \
    libspdlog-dev \
    libzstd-dev \
    desktop-file-utils \
    && rm -rf /var/lib/apt/lists/*

# ---- Basler pylon SDK + blaze supplementary package (bind-mounted, so the 1.1 GB .deb is not
#      copied into an image layer). Provides /opt/pylon and the camera emulator. -----------------
RUN --mount=type=bind,source=third_party/pylon,target=/tmp/pylon \
    ls /tmp/pylon/pylon_*.deb /tmp/pylon/pylon-supplementary-package-for-blaze*.deb >/dev/null \
      || { echo "ERROR: put pylon_*.deb and pylon-supplementary-package-for-blaze*.deb into third_party/pylon/ (see its README)"; exit 1; } ; \
    apt-get update \
    && apt-get install -y /tmp/pylon/pylon_*.deb /tmp/pylon/pylon-supplementary-package-for-blaze*.deb \
    && rm -rf /var/lib/apt/lists/*

# ---- environment -------------------------------------------------------------------------------
ENV RMW_IMPLEMENTATION=rmw_zenoh_cpp
ENV PYLON_ROOT=/opt/pylon
# two emulated Basler cameras (serials 0815-0000 / 0815-0001) so no hardware is needed
ENV PYLON_CAMEMU=2

# the repo is bind-mounted from the host with a different owner; let git work in it (and in the driver clones)
RUN git config --system --add safe.directory '*'

WORKDIR /workspaces/zenoh_test

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
    && echo "[ -f /workspaces/zenoh_test/install/setup.bash ] && source /workspaces/zenoh_test/install/setup.bash" >> /root/.bashrc
