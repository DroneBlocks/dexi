# April Tags

April tag detection is disabled by default in the DEXI launch configuration. To enable april tag detection, you need to modify the launch configuration in the DEXI service.

> **Note:** The following steps require root access. You'll need to use `sudo` or switch to the root user (`sudo su`) to modify the service files.

## Enabling April Tags

### 1. Edit the start.bash script

The DEXI service uses `start.bash` to launch the system. To enable april tags, modify this script:

```bash
# Edit the start.bash script
sudo nano ~/dexi_ws/src/dexi/scripts/start.bash
```

Add the `apriltags` argument to the launch command:

```bash
/opt/ros/humble/bin/ros2 launch dexi dexi.launch.xml apriltags:=true
```

### 2. Restart the DEXI service

After modifying the script, restart the DEXI service to apply the changes:

```bash
sudo systemctl restart dexi.service
```

> **Note:** Alternatively, you can reboot the Raspberry Pi to apply the changes:
> ```bash
> sudo reboot
> ```

## Verification

Once april tags are enabled, you can verify they are working by:

1. Checking the `/detections` topic for april tag detections:
```bash
ros2 topic echo /detections
```

2. Or using the throttled detections topic for a slower rate:
```bash
ros2 topic echo /throttled/detections
```

## Video Tutorial

For a complete tutorial on working with April Tags in DEXI, including detection with Node-RED, check out our video course:

[DEXI - April Tag Fiducial Marker Detection with ROS2](https://learn.droneblocks.io/courses/2654886/lectures/57725121) 