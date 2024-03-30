# Setup ROS2

```bash
# Setup apt repos
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update

# Install ROS2 humble
sudo apt install ros-humble-ros-base ros-dev-tools ros-humble-rosbridge-server -y
sudo rosdep init

# Source the base workspace by default
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# Create the workspace for development

Create a workspace, clone the DEXI repo into it, and run a colcon build:

```bash
mkdir -p ~/dexi_ws/src

cd ~/dexi_ws/src

git clone -b develop https://github.com/DroneBlocks/dexi.git

git submodule update --init --remote --recursive

cd ..

rosdep install --from-paths src -y --ignore-src

colcon build --symlink-install
```

# Setup the service

Run the install script to set the service to start on boot:

```bash
bash ~/dexi_ws/src/dexi/dexi/scripts/install.bash
```

# Camera

Make sure to enable legacy camera support on Pi 4 (Ubuntu 22.04 LTS) in /boot/firmware/config.txt:

```
start_x=1
```

and make sure to comment out the following line:

```
#camera_auto_detect=1
```

# MAVROS and SITL

Run the following from your host machine in the docker directory. It will spin up PX4-SITL and a ROS Humble container for DEXI development. You will need to make sure PX4 SITL is configured to send packets to the DEXI container IP

```
docker-compose up
```

Alternatively, if you have a DEXI container already running you can find its IP address and then start SITL. You'll need to make sure to specify the IP of your DEXI container. You can find this by running:

```
docker inspect bridge
```

Find your DEXI IP and then run:

```
docker run --rm -it jonasvautherin/px4-gazebo-headless:1.14.0 172.17.0.2 <- This is your DEXI container IP
```

In your DEXI container make sure MAVROS is installed:

```
sudo apt install ros-humble-mavros
```

and make sure to update your px4.launch file in:

```
/opt/ros/humble/shared/mavros/launch/px4.launch
```

with:

```
<arg name="fcu_url" default="udp://:14540@:14540" />
```

Install the geographic lib dependencies:

```
/opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

Finally, test the node:

```
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch
```

# Docker Dev

### New Container

Mapping port 9090 for rosbridge websocket:

- docker run -it -p 9090:9090 --name dexi -v ${PWD}:/root/ros2_ws/src osrf/ros:humble-desktop

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
