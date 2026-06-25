# Differential Mobile Robot for SLAM and Dynamic Target Following

This repository contains the simulation and hardware implementation of a differential-drive mobile robot developed using ROS 2.

The robot can perform **SLAM (Simultaneous Localization and Mapping)** in unknown environments or, given a pre-built map, autonomously **follow a person or a dynamic object** that is not part of the static environment representation.

## Features

* Differential-drive kinematics.
* Wheel odometry computation.
* IMU integration for local localization (**WIP**).
* Real-time SLAM.
* Frontier-based autonomous exploration (**WIP**).
* Global localization using AMCL.
* Dynamic object/person following.
* Shared software architecture for both simulation and real-world deployment.
* Teleoperation via keyboard or joystick.
* Configurable robot parameters, including wheel radius and wheel separation. It is recommended to keep the default values, which match the physical robot.
* An additional node can inject measurement errors into the wheel radius and wheel separation parameters, producing a more realistic odometry model that better resembles real-world robot behavior.

## Known Issues

### Work-in-Progress Features

* A local localization algorithm based on an Extended Kalman Filter (EKF) has been implemented to fuse IMU and wheel encoder data. However, its output is currently not used due to issues encountered during hardware deployment.
* The frontier exploration algorithm is functional, but in some situations it may eventually select an unreachable frontier cell in the occupancy grid, causing the exploration process to fail.

### Other Issues

* The `object_detector` node may experience startup issues. Several attempts may be required before it starts working correctly.
* Since the robot continuously updates the global costmap while moving, certain obstacle trajectories (for example, a moving person being followed) may cause the navigation stack to fail and trigger a recovery behavior. This issue is difficult to describe precisely, so a demonstration video is provided.

## How to Use

The entire system can be launched using a single launch file:

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py
```

### Main Launch Arguments

#### `use_slam`

Enables or disables SLAM.

Possible values:

* `true`
* `false`

#### `world_name`

Selects the simulation environment.

Available worlds:

* `small_house`
* `small_warehouse`

#### `map_name`

Use this parameter only when `use_slam` is set to `false`.

In this case, a pre-built map must be provided so that the robot can localize itself using AMCL.

Available maps:

* `small_house`
* `small_warehouse`

Make sure the selected map matches the simulated environment.

### Keyboard Teleoperation

To control the robot using the keyboard:

```bash
ros2 run key_teleop key_teleop
```

### Dynamic Object Following

To run the object detector and enable the robot to follow an unknown obstacle:

```bash
ros2 run leg_detector obj_detector.py
```

**Note:** Several attempts may be required before the node starts correctly.

### Operate real robot hardware

In case you want to operate a real robot hardware:

```bash
ros2 launch bumperbot_bringup real_robot.launch.py
```

Regarding the launch arguments you need to provide only **use_slam** and if you set it to false provide **map_name** as well.

## Fast launch commads:

### Small_house with slam

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=true
```
### Small_house with amcl

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_house use_slam:=false map_name:=small_house
```

### Small_warehouse with slam

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_warehouse use_slam:=true
```

### Small_warehouse with amcl

```bash
ros2 launch bumperbot_bringup simulated_robot.launch.py world_name:=small_warehouse use_slam:=false map_name:=small_warehouse
```



