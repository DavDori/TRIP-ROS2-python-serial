# TRIP-ROS2-interface

This repository contains a ROS2 node designed to interface with an Arduino board as part of a larger project. The node facilitates communication between the ROS 2 framework and the Arduino microcontroller, enabling seamless integration of sensor data, control commands, or any other relevant information between the two systems.

## Requirements
- ROS2 Humble

## Installation

Clone this repository into your ROS2 workspace:

```bash
cd /path/to/your/ros2_ws/src
git clone https://github.com/DavDori/TRIP-ROS2-python-serial.git
```

 ## Build the ROS2 workspace

```bash
cd /path/to/your/ros2_ws
colcon build
```
and source
```bash
source install/setup.bash
```

## How to use it

Depending on the application, you can choose between controlling the motors directly, or take advantage of the unicycle model.

### Direct Motor Control

You can control the velocity set-point for each motor via a ROS message of type `sensor_msgs/JointState` that can be published on the
`/joints_cmd` topic. The command to launch the node is

```
ros2 run trip_serial trip_base
```

### Unicycle Model

To control the robot with forward speed and angular velocity use 

```
ros2 run trip_serial trip_unicycle
```

In this case, you have to publish a message of type `geometry_msgs/Twist` on the `/cmd_vel` topic to set the deisred velocities.

## Authors and acknowledgment
Thanks to the the ROS and python community

## License
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

This project is licensed under the Apache-2.0 License.

