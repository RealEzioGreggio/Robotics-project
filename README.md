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
* Teleoperation via keyboard or joystick (All the inputs for the robot are managed by a mux node)
* Configurable robot parameters, including wheel radius and wheel separation. It is recommended to keep the default values, which match the physical robot.
* An additional node can inject measurement errors into the wheel radius and wheel separation parameters, producing a more realistic odometry model that better resembles real-world robot behavior.


## Package Description

* **bumperbot_bringup**: Launches all the required launch files from the other packages, providing a single entry point to start the entire system.

* **bumperbot_controller**: Starts the `twist_mux` node, which manages velocity commands coming from the joystick, keyboard, or navigation stack. It also launches the differential-drive controller that solves the robot's inverse kinematics.

* **bumperbot_description**: The core package for the robot description and simulation. It contains the robot model, simulation worlds, sensor configurations (including ROS–Gazebo bridges), and the `ros2_control` configuration.

* **bumperbot_firmware**: Launches the custom hardware interface used by `ros2_control` to communicate with the real robot. It also contains the complete firmware project for the STM32 microcontroller.

* **bumperbot_localization**: Launches both the local localization algorithm (EKF) and the global localization algorithm (AMCL).

* **bumperbot_utils**: Contains additional utility nodes for the robot. At the moment, it includes a safety stop node and a frontier detector for autonomous exploration (**WIP**).

* **mapping**: Launches the SLAM algorithm and contains maps of different simulated and real-world environments.

* **navigation**: Launches the autonomous navigation stack, enabling the robot to reach a target position while avoiding obstacles.

* **leg_detector**: Contains the object detector algorithm


## Known Issues

### Work-in-Progress Features

* A local localization algorithm based on an Extended Kalman Filter (EKF) has been implemented to fuse IMU and wheel encoder data. However, its output is currently not used due to issues encountered during hardware deployment.
* The frontier exploration algorithm is functional, but in some situations it may eventually select an unreachable frontier cell in the occupancy grid, causing the exploration process to fail.

### Other Issues

* The `object_detector` node may experience startup issues. Several attempts may be required before it starts working correctly.
* Since the robot continuously updates the global costmap while moving, certain obstacle trajectories (for example, a moving person being followed) may cause the navigation stack to fail and trigger a recovery behavior.

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

**Note:** Remember to input the correct initial guess for the amcl algorithm at startup.

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

### Safety stop

If you want to avoid the robot to crash into an obstacles:

```bash
ros2 run bumperbot_utils safety_stop.py 
```

**Note:** You can add on rviz by the topic /zones a marker array that shows the actual zones. If an obstacles is in the yellow area the robot will slow down and if an obstacle enter the red area the robot will stop receiving commands velocity so you have to manually remove the obstacle or move the robot.

### Local localization

this will create base_footprint_ekf so you can confront it with base_footprint_noisy:

```bash
ros2 launch bumperbot_localization local_localization.launch.py 
```

**Note:** Start this before moving the robot around the map.

### Frontier detector

Automatically detects frontiers for autonomous SLAM.

```bash
ros2 run bumperbot_utils frontier_detector.py
```

## Troubleshooting

### Container startup issues

The container is configured to use the host computer's physical GPU. Depending on your hardware and software configuration, this may cause startup issues.

If this happens, try launching the container without GPU support:

```bash
xhost +local:root

docker run -it --rm \
    --net=host \
    --ipc=host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.Xauthority:/root/.Xauthority \
    -v ./ros_ws/:/root/ros_workspace \
    --device=/dev/input \
    --name lab1 \
    ros:livelab1 bash
```

### Gazebo rendering issues

If you experience rendering problems in Gazebo, especially with the laser scanner rays, open:

```text
src/bumperbot_description/urdf/bumperbot_gazebo.xacro
```

and change the rendering engine configuration to:

```xml
<render_engine>ogre2</render_engine>
```

(around line 57 of the file).
 



