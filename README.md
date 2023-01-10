# ros-gz-rover

This package contains an example demonstrating how to use the ROS2 - Gazebo
bridge for a skid-steer rover controlled by ArduPilot. It assumes an advanced
level of familiarity with the packages referred to below, most of the
dependencies will need to be built from source. 

![ros_gz_rover_playpen_5](https://user-images.githubusercontent.com/24916364/163236091-29607f17-39ec-45e9-9625-54550738672e.jpg)


## Quick start

The following instructions are for macOS (Monterey 12.5). They assume that
ROS2 Humble (or Rolling) is available on the system (including rviz2), and that
Gazebo Garden has been installed from source. An ArduPilot development
environment must be installed including MAVProxy. The ArduPilot/ardupilot_gazebo
plugins must be installed and the ArduPilot/SITL_Models repo available. Each
terminal running a command will need to correctly source the appropriate
environments and ensure the Gazebo environment variables specifying Gazebo
resource and system plugins are exported. 

Create workspace

```bash
# create workspace
$ mkdir -p ~/ros2-gz/src && cd ~/ros2-gz/src

# clone ros-gz (ros2 rolling branch)
$ git clone https://github.com/gazebosim/ros_gz.git -b ros2

# clone tf_transformations (ros2 humble may not include this)
https://github.com/DLu/tf_transformations.git


# clone dependencies for mavros (ROS2)
git clone https://github.com/ros/angles.git -b ros2
git clone https://github.com/ros/diagnostics.git -b humble
git clone https://github.com/ros/diagnostics.git -b humble
git clone https://github.com/ros-geographic-info/geographic_info.git -b ros2
git clone https://github.com/mavlink/mavlink-gbp-release.git
git clone https://github.com/srmainwaring/mavros.git -b srmainwaring/ros2-macos

# clone this repo
git clone https://github.com/srmainwaring/ros_gz_rover.git
```

Build

```bash
$ cd ~/ros2-gz
$ colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DCMAKE_CXX_STANDARD=17
```

Launch the ROS2 nodes, the Gazebo server, and RViz

```bash
$ cd ~/ros2-gz
$ source ./install/setup.zsh
$ ros2 launch ros_gz_rover rover.launch.py
```

Launch the mavros nodes including plugins from mavros_extras

```bash
$ cd ~/ros2-gz
$ source ./install/setup.zsh
$ ros2 launch ros_gz_rover mavros.launch.py
```

Launch SITL and provide output to additional GCS running on another machine

```bash
$ sim_vehicle.py -N -v Rover -f rover-skid --model JSON --mavproxy-args="--out=udp:192.168.1.83:14551"
```

Launch the Gazebo client

```bash
$ gz sim -v4 -g
```

