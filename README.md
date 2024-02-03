# Update repos

- sudo apt update
- sudo add-apt-repository universe
- sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
- sudo apt update

# Create the dexi package (first time setup)

ros2 pkg create dexi --dependencies rclcpp --build-type ament_cmake

# ROSBRIDGE

- sudo apt install ros-humble-rosbridge-server
- ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# Camera

### Build

- cd /root/ros2_ws/src
- git clone https://github.com/Kapernikov/cv_camera
- cd /root/ros2_ws
- rosdep install --from-paths src -y --ignore-src
- colcon build

### Run

- source install/setup.bash
- ros2 run cv_camera cv_camera_node

# Web video server

### Build

- cd /root/ros2_ws/src
- git clone https://github.com/RobotWebTools/web_video_server/
- cd web_video_server
- git checkout ros2
- cd ~/ros2_ws
- rosdep install --from-paths src -y --ignore-src
- colcon build

### Run

- source install/setup.bash
- ros2 run web_video_server web_video_server

# Micro DDS Client

### Build

- cd ~/ros2_ws/src
- git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
- cd ~/ros2_ws
- colcon build

### Run

- source install/setup.bash
- MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600

# PX4 ROS Messages and ROS Com Example

### Build

- cd ~/ros2_ws/src
- git clone https://github.com/PX4/px4_msgs
- git clone https://github.com/PX4/px4_ros_com
- cd ~/ros2_ws
- colcon build

# LED

# GPIO

# Docker Dev

### New Container

Mapping port 9090 for rosbridge websocket:

- docker run -it -p 9090:9090 -v ${PWD}:/root/ros2_ws/src osrf/ros:humble-desktop

### Existing Container

- docker start 12b3e9d32467
- docker exec -it 12b3e9d32467 /bin/bash

# Calling DroneBlocks Services

- ros2 service call /droneblocks/run dexi_msgs/srv/Run "{code: 'mission code goes in here'}"
- ros2 service call /droneblocks/stop std_srvs/srv/Trigger
- ros2 service call /droneblocks/load dexi_msgs/srv/Load
# nginx for DroneBlocks

- docker run -it --rm -d -p 7777:80 --name droneblocks -v ${PWD}/droneblocks/www:/usr/share/nginx/html nginx

# Thanks

A special thanks to the following projects for inspiration:

- https://github.com/CopterExpress/clover
- https://github.com/StarlingUAS/clover_ros2_pkgs
