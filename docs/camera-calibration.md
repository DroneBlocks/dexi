# Calibrating Pi Cam v2.1

Followed the process here: https://medium.com/starschema-blog/offline-camera-calibration-in-ros-2-45e81df12555

Camera node should be run on Pi and Calibration GUI on a laptop/desktop. Make sure both have the same ROS_DOMAIN_ID set:

export ROS_DOMAIN_ID=1

## Checkerboard
Checkerboard can be [found here](./assets/calib.io_checker_200x150_8x10_15.pdf). Be sure when printing to print at 100% so your printer does not scale the image. Each checker should be 15x15mm as specified in the [pattern generator](https://calib.io/pages/camera-calibration-pattern-generator).

![Sample checkerboard](./assets/pattern_generator.png)

The size of this board is 7x9 (interior corners) with a square size of 0.015 (in meters).

## Raspberry Pi Side

For Pi cam v2 and v2.1 make sure to have your pi /boot/firmware/config.txt properly set:

```
#camera_auto_detect=1
start_x=1
```

Then run:

```
ros2 run cv_camera cv_camera_node --ros-args -r __ns:=/camera
```

This is from: https://github.com/Kapernikov/cv_camera

The namespace of /camera is important so that on the laptop/desktop it finds the /camera/image_raw topic and we can do the calibration.

## Laptop side

```
sudo apt install ros-humble-camera-calibration
```

Now run:

```
ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.015 --ros-args -r image:=/image_raw
```










