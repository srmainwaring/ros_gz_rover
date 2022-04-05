import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_ros_ign_rover = get_package_share_directory('ros_ign_rover')

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_ign_rover,
        'models', 'rover.urdf.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file)
    robot_description = {"robot_description": robot_description_config.toxml()}

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description],
    )

    # Ignition gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-s -r empty_physics.sdf'}.items(),
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(pkg_ros_ign_rover, 'rviz', 'rover.rviz')
        ],
    )

    # Spawn
    # 
    # spawn the model published on the topic 'robot_description'
    # into the current world with the name 'rover'. Adjust the pose
    # as required.
    spawn = Node(package='ros_ign_gazebo', executable='create',
        arguments=[
            '-world', '',
            '-param', '',
            '-name', 'rover',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    # Ign - ROS Bridge
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/world/empty/model/rover/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/world/empty/model/rover/link/base_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/empty/model/rover/joint_state', 'joint_states'),
            ('/world/empty/model/rover/link/base_link/sensor/imu_sensor/imu', 'sensors/imu'),
            # ('/lidar', 'sensors/laser_scan'),
            # ('/lidar/points', 'sensors/laser_scan/points'),
        ],
        output='screen'
    )

    # tf2 static broadcaster
    static_tf2_broadcaster = Node(
        package='ros_ign_rover',
        executable='tf2_broadcaster',
        arguments=[
        ],
    )

    return LaunchDescription(
        [
            # Nodes and Launches
            gazebo,
            spawn,
            bridge,
            robot_state_publisher,
            rviz,
            static_tf2_broadcaster,
        ]
    )
