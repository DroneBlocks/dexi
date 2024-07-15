#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/pi/dexi_ws/install/setup.bash

/opt/ros/humble/bin/ros2 launch dexi dexi.launch.xml
