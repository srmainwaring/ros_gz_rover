# Running with ArduPilot

Instructions for building mavros for ROS2 on macOS and running with ArduPilot

## Quick start

### Additional dependencies

- [angles](https://github.com/ros/angles)
- [diagnostics](https://github.com/ros/diagnostics)
- [eigen_stl_containers](https://github.com/ros/eigen_stl_containers)
- [geographic_info](https://github.com/ros-geographic-info/geographic_info)
- [mavlink](https://github.com/mavlink/mavlink)
- [mavros](https://github.com/mavlink/mavros)

### Build

```bash
colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_MACOSX_RPATH=FALSE -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib -DCMAKE_CXX_STANDARD=17
```

### Run

```bash
$ source ~/Code/ros2/ros2-mavros/install/setup.zsh
$ ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://127.0.0.1:14551@14555
```

### Topics

```bash
/mavros/actuator_control
/mavros/altitude
/mavros/battery
/mavros/estimator_status
/mavros/extended_state
/mavros/geofence/fences
/mavros/global_position/compass_hdg
/mavros/global_position/global
/mavros/global_position/gp_lp_offset
/mavros/global_position/gp_origin
/mavros/global_position/local
/mavros/global_position/raw/fix
/mavros/global_position/raw/gps_vel
/mavros/global_position/raw/satellites
/mavros/global_position/rel_alt
/mavros/global_position/set_gp_origin
/mavros/home_position/home
/mavros/home_position/set
/mavros/imu/data
/mavros/imu/data_raw
/mavros/imu/diff_pressure
/mavros/imu/mag
/mavros/imu/static_pressure
/mavros/imu/temperature_baro
/mavros/imu/temperature_imu
/mavros/local_position/accel
/mavros/local_position/odom
/mavros/local_position/pose
/mavros/local_position/pose_cov
/mavros/local_position/velocity_body
/mavros/local_position/velocity_body_cov
/mavros/local_position/velocity_local
/mavros/manual_control/control
/mavros/manual_control/send
/mavros/mission/reached
/mavros/mission/waypoints
/mavros/nav_controller_output/output
/mavros/param/event
/mavros/rallypoint/rallypoints
/mavros/rc/in
/mavros/rc/out
/mavros/rc/override
/mavros/setpoint_accel/accel
/mavros/setpoint_attitude/cmd_vel
/mavros/setpoint_attitude/thrust
/mavros/setpoint_position/global
/mavros/setpoint_position/global_to_local
/mavros/setpoint_position/local
/mavros/setpoint_raw/attitude
/mavros/setpoint_raw/global
/mavros/setpoint_raw/local
/mavros/setpoint_raw/target_attitude
/mavros/setpoint_raw/target_global
/mavros/setpoint_raw/target_local
/mavros/setpoint_trajectory/desired
/mavros/setpoint_trajectory/local
/mavros/setpoint_velocity/cmd_vel
/mavros/setpoint_velocity/cmd_vel_unstamped
/mavros/state
/mavros/statustext/recv
/mavros/statustext/send
/mavros/target_actuator_control
/mavros/time_reference
/mavros/timesync_status
/mavros/wind_estimation
```


## Details

### Topics

#### `rc_io`

RC inputs (manual mode)

```bash
# MAV> rc 1 1400
% ros2 topic echo /mavros/rc/in
header:
  stamp:
    sec: 1649765956
    nanosec: 377929042
  frame_id: ''
rssi: 255
channels:
- 1400
- 1500
- 1500
- 1500
- 1800
- 1000
- 1000
- 1800
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
---
```

Servo outputs (manual mode)

```bash
# MAV> status servo*
% ros2 topic echo /mavros/rc/out
header:
  stamp:
    sec: 1649766021
    nanosec: 289517775
  frame_id: ''
channels:
- 1426
- 0
- 1574
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
---
```

#### `waypoint`

WP reached

```bash
# MAV> mode auto
$ ros2 topic echo /mavros/mission/reached 
header:
  stamp:
    sec: 1649765772
    nanosec: 199085870
  frame_id: ''
wp_seq: 1
---
```

## Links

Creating launch files and remapping topics

  - https://docs.ros.org/en/foxy/Tutorials/Launch/Creating-Launch-Files.html

Loading YAML parameters in ROS2 launch files

  - https://roboticsbackend.com/ros2-yaml-params/



