import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    pkg_mavros = get_package_share_directory('mavros')
    pkg_ros_ign_rover = get_package_share_directory('ros_ign_rover')

    # Config
    config = os.path.join(pkg_ros_ign_rover, 'config', 'config.yaml')
    # pluginlists = os.path.join(pkg_ros_ign_rover, 'config', 'pluginlists.yaml')

    # Resolved:
    #   Parameter yaml files have a different structure for ROS2.
    #   See the migration guide:
    #   https://docs.ros.org/en/galactic/How-To-Guides/Parameters-YAML-files-migration-guide.html?highlight=parameters
    # 
    # Issue: using yaml in composite nodes
    #   https://github.com/ros2/rclcpp/issues/715#issuecomment-497392626
    #   https://answers.ros.org/question/346409/ros2-component_container-yaml-parsing/
    # 
    # Workaround
    # 
    # with open(config, 'r') as f:
    #     params = yaml.safe_load(f)
    # print(params)

    # mavros
    #
    # obstacle
    #   /mavros/obstacle/send to /sensors/laser_scan.
    #
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        arguments=[
            '--ros-args',
            '-p', 'fcu_url:=udp://127.0.0.1:14551@14555'
        ],
        parameters=[config],
        remappings=[
            ('/mavros/obstacle/send', '/sensors/laser_scan'),
        ]
    )

    # Nodes and Launches
    return LaunchDescription(
        [
            mavros,
        ]
    )
