"""ROS 2 sensor pipeline used by the benchmarks.

One composable container holds the Ouster pcap replay + cloud nodes, two (emulated) Basler
cameras and a small consumer node, all with intra-process comms. A rosbag2 recorder (mcap +
zstd) is loaded into the same container unless `record:=false`.

Needs PYLON_CAMEMU=2 for the emulated cameras (set by docker-compose.yml).

Arguments:
  record     true|false   record cloud + both camera topics to a bag      (default true)
  pcap       path         Ouster pcap to replay                           (default: <data_dir>/OS-1-128_...pcap)
  metadata   path         Ouster metadata JSON matching that pcap
  bag_dir    path         where bags are written                          (default /workspaces/zenoh_test/bags)
  packet_qos_depth  int   queue depth of the lidar packet topic             (default 512; 10 reproduces the flicker)
  system_default_qos true|false  reliable packet topics with the stack's default depth (true, default) or the Ouster
                                 driver's original best-effort sensor-data QoS, depth 5 (false)
  timestamp_mode   str    ouster timestamp mode: TIME_FROM_ROS_TIME (default), TIME_FROM_INTERNAL_OSC, TIME_FROM_PTP_1588

Environment: BENCH_DATA_DIR overrides the default data directory (/workspaces/zenoh_test/data).
"""
import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

DATA_DIR = os.environ.get('BENCH_DATA_DIR', '/workspaces/zenoh_test/data')
DEFAULT_PCAP = os.path.join(DATA_DIR, 'OS-1-128_v3.0.1_2048x10_20230216_143245-000.pcap')
DEFAULT_METADATA = os.path.join(DATA_DIR, 'OS-1-128_v3.0.1_2048x10_20230216_143245.json')

# Lidar packets arrive at ~1.1-1.3 kHz. A deep queue lets the cloud node survive short stalls without
# dropping packets (a dropped packet = 16 missing scan columns). 512 ~ 0.4 s. See docs/.


def generate_launch_description():
    pcap = LaunchConfiguration('pcap')
    metadata = LaunchConfiguration('metadata')
    packet_depth = ParameterValue(LaunchConfiguration('packet_qos_depth'), value_type=int)
    system_qos = ParameterValue(LaunchConfiguration('system_default_qos'), value_type=bool)

    ouster_pcap_node = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterPcap',
        name='os_pcap',
        namespace='/ouster',
        parameters=[{
            'metadata': metadata,
            'pcap_file': pcap,
            'loop': True,
            'use_system_default_qos': system_qos,  # True: reliable packet topics
            'packet_qos_depth': packet_depth,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    ouster_cloud_node = ComposableNode(
        package='ouster_ros',
        plugin='ouster_ros::OusterCloud',
        name='os_cloud',
        namespace='/ouster',
        parameters=[{
            'timestamp_mode': LaunchConfiguration('timestamp_mode'),
            'sensor_frame': 'os_sensor',
            'lidar_frame': 'os_lidar',
            'imu_frame': 'os_imu',
            'ptp_utc_tai_offset': -37.0,
            'use_system_default_qos': system_qos,
            'packet_qos_depth': packet_depth,
            'metadata': metadata,
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    def make_camera(name, serial):
        # Emulated cameras (PYLON_CAMEMU=2) have serials 0815-0000 / 0815-0001.
        return ComposableNode(
            package='pylon_ros2_camera_component',
            plugin='pylon_ros2_camera::PylonROS2CameraNode',
            name=name,
            namespace='/',
            parameters=[{
                'device_user_id': serial,  # patched driver also matches serial numbers
                'frame_rate': 10.0,
                'startup_image_width': 1920,   # added by patches/pylon-ros-camera.patch
                'startup_image_height': 1080,
            }],
            extra_arguments=[{'use_intra_process_comms': True}],
        )

    consumer_node = ComposableNode(
        package='sensor_benchmark',
        plugin='sensor_benchmark::SensorConsumer',
        name='sensor_consumer',
        namespace='/',
        remappings=[
            ('ouster/points', '/ouster/points'),
            ('camera1/image', '/my_camera/image_raw'),
            ('camera2/image', '/my_camera_2/image_raw'),
        ],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    container = ComposableNodeContainer(
        name='benchmark_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ouster_pcap_node,
            ouster_cloud_node,
            make_camera('my_camera', '0815-0000'),
            make_camera('my_camera_2', '0815-0001'),
            consumer_node,
        ],
        output='screen',
    )

    # rosbag2 recorder (mcap, 2 GiB splits, each closed file zstd-compressed) in the same container.
    bag_name = datetime.now().strftime('benchmark_%Y%m%d_%H%M%S')
    recorder_node = ComposableNode(
        package='rosbag2_transport',
        plugin='rosbag2_transport::Recorder',
        name='recorder',
        parameters=[{
            'storage.uri': [LaunchConfiguration('bag_dir'), '/' + bag_name],
            'storage.storage_id': 'mcap',
            'storage.max_bagfile_size': 2 * 1024 ** 3,
            'record.compression_mode': 'file',
            'record.compression_format': 'zstd',
            'record.topics': [
                '/ouster/points',
                '/my_camera/image_raw',
                '/my_camera_2/image_raw',
            ],
        }],
    )
    load_recorder = LoadComposableNodes(
        target_container='/benchmark_container',
        composable_node_descriptions=[recorder_node],
        condition=IfCondition(LaunchConfiguration('record')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('record', default_value='true',
                              description='Record cloud + camera topics to a rosbag'),
        DeclareLaunchArgument('pcap', default_value=DEFAULT_PCAP, description='Ouster pcap to replay'),
        DeclareLaunchArgument('metadata', default_value=DEFAULT_METADATA,
                              description='Ouster metadata JSON that matches the pcap'),
        DeclareLaunchArgument('system_default_qos', default_value='true',
                              description='true: reliable packet topics; false: best-effort sensor-data QoS, depth 5 (the driver default)'),
        DeclareLaunchArgument('timestamp_mode', default_value='TIME_FROM_ROS_TIME',
                              description='Ouster timestamp mode (TIME_FROM_ROS_TIME, TIME_FROM_INTERNAL_OSC, TIME_FROM_PTP_1588)'),
        DeclareLaunchArgument('packet_qos_depth', default_value='512',
                              description='Queue depth of the lidar packet topic (10 = default, drops packets)'),
        DeclareLaunchArgument('bag_dir', default_value='/workspaces/zenoh_test/bags',
                              description='Directory the rosbag is written to'),
        container,
        load_recorder,
    ])
