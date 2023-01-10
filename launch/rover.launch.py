import math
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Package Directories
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_ros_gz_rover = get_package_share_directory("ros_gz_rover")

    # Parse robot description from xacro
    robot_description_file = os.path.join(
        pkg_ros_gz_rover, "models", "rover.urdf.xacro"
    )
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "-s -r rover_playpen.sdf"}.items(),
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_ros_gz_rover, "rviz", "rover.rviz")],
    )

    # Spawn
    #
    # spawn the model published on the topic 'robot_description'
    # into the current world with the name 'rover'. Adjust the pose
    # as required.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world",
            "",
            "-param",
            "",
            "-name",
            "rover",
            "-topic",
            "robot_description",
            "-x",
            "0.0",
            "-y",
            "0.0",
            "-z",
            "0.25",
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            f"{math.pi/2}",
        ],
        output="screen",
    )

    # Gz - ROS Bridge
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/rover_playpen/model/rover/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/world/rover_playpen/model/rover/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/rover/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        ],
        remappings=[
            ("/world/rover_playpen/model/rover/joint_state", "joint_states"),
            (
                "/world/rover_playpen/model/rover/link/base_link/sensor/imu_sensor/imu",
                "sensors/imu",
            ),
            ("/lidar", "sensors/laser_scan"),
            ("/lidar/points", "sensors/laser_scan/points"),
            ("/model/rover/odometry", "odom"),
            ("/model/rover/pose", "pose"),
        ],
        output="screen",
    )

    # static transform from lidar link to the lidar sensor (identity)
    static_tf_lidar_to_gpu_lidar = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_lidar_to_gpu_lidar",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "lidar_link",
            "--child-frame-id",
            "rover/base_link/gpu_lidar",
        ],
    )

    # static transform from map to odom (identity - assuming perfect odometry)
    #
    # NOTE: the Ardupilot plugin must use the correct transform from the
    #       Gazebo ENU world frame to the ArduPilot NED world frame
    #
    #   <gazeboXYZToNED>0 0 0 ${PI} 0 ${PI/2}</gazeboXYZToNED>
    #
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_map_to_odom",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            "0.0",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # static transform from base_link to base_link_frd (body-frame aerospace convention)
    static_tf_base_link_to_base_link_frd = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_link_to_base_link_frd",
        arguments=[
            "--x",
            "0.0",
            "--y",
            "0.0",
            "--z",
            "0.0",
            "--roll",
            f"{math.pi}",
            "--pitch",
            "0.0",
            "--yaw",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "base_link_frd",
        ],
    )

    tf_broadcaster = Node(
        package="ros_gz_rover",
        executable="tf_broadcaster",
        arguments=[],
    )

    # laser scan processing
    laser_scan_transform = Node(
        package="ros_gz_rover",
        executable="laser_scan_transform_flu_to_frd",
        arguments=[],
    )

    # Nodes and Launches
    return LaunchDescription(
        [
            gazebo,
            spawn,
            bridge,
            robot_state_publisher,
            rviz,
            static_tf_lidar_to_gpu_lidar,
            static_tf_map_to_odom,
            static_tf_base_link_to_base_link_frd,
            tf_broadcaster,
            laser_scan_transform,
        ]
    )
