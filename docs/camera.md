# Camera

Make sure to enable legacy camera support on Pi 4 (Ubuntu 22.04 LTS) in /boot/firmware/config.txt:

```
start_x=1
```

and make sure to comment out the following line:

```
#camera_auto_detect=1
```

### Build

Dependencies

```
sudo apt-get install build-essential
sudo apt-get install libboost-all-dev
```

- cd ~/ros2_ws/src
- git clone https://github.com/Kapernikov/cv_camera
- cd ~/ros2_ws
- rosdep install --from-paths src -y --rosdistro humble --ignore-src
- colcon build --packages-select cv_camera

### Run

- source install/setup.bash
- ros2 run cv_camera cv_camera_node

# Web video server

### Build

- cd ~/ros2_ws/src
- git clone https://github.com/RobotWebTools/web_video_server/
- cd web_video_server
- git checkout ros2
- cd ~/ros2_ws
- rosdep install --from-paths src -y --rosdistro humble --ignore-src
- colcon build --packages-select web_video_server

### Run

- source install/setup.bash
- ros2 run web_video_server web_video_server

# Image Rectification

```
sudo apt install ros-humble-image-proc
```
