# Offboard Control

## Download

Place the following in your ROS2 src folder:

```
git clone https://github.com/PX4/px4_msgs
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
```

## Build

From the root of your ROS2 workspace:

```
colcon build --packages-select px4_msgs
colcon build --packages-select microxrcedds_agent
```

## Start

Make sure PX4 SITL is running on the host:

```
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

This should start the microdds client in SITL.

Now launch the offboard nodes:

```
source install/setup.bash
ros2 launch dexi_py offboard.launch.xml
```

## Command Line
```
ros2 topic pub /offboard/command std_msgs/String "data: 'takeoff'" -1
ros2 topic pub /offboard/command std_msgs/String "data: 'land'" -1
```
