# Camera

Make sure to enable legacy camera support on Pi 4 (Ubuntu 22.04 LTS) in /boot/firmware/config.txt:

```
start_x=1
```

and make sure to comment out the following line:

```
#camera_auto_detect=1
```

## For ARK CM4:

Edit /boot/firmware/config.txt 

```
dtoverlay=imx219,cam0
dtoverlay=imx219,cam1
```

```
sudo apt install ros-humble-camera-ros
```

Reboot.

### Run

- source install/setup.bash
- ros2 run camera_ros camera_node --ros-args -p width:=640 -p height:=480

# Web video server

### Build

- cd ~/dexi_ws/src
- rosdep install --from-paths src -y --rosdistro humble --ignore-src
- colcon build --packages-select web_video_server

### Run

- source install/setup.bash
- ros2 run web_video_server web_video_server

# Image Rectification

```
sudo apt install ros-humble-image-proc
```
