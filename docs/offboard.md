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
