#!/bin/bash

source /opt/ros/humble/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

/opt/ros/humble/bin/ros2 launch dexi dexi.launch.xml cam0:=true cam1:=false apriltags:=true gpio:=true
