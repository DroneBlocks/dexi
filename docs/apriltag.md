# April Tags

cd ~/dexi_ws

rosdep update

sudo apt update

git clone https://github.com/christianrauch/apriltag_ros ~/dexi_ws/src/apriltag_ros

git clone https://github.com/christianrauch/apriltag_msgs ~/dexi_ws/src/apriltag_msgs

rosdep install --from-paths src -y --rosdistro humble --ignore-src

colcon build --packages-select apriltag_msgs

colcon build --packages-select apriltag_ros

## Running the node standalone with default camera setup

ros2 run apriltag_ros apriltag_node --ros-args -r image_rect:=/camera/image_raw

## Running the node with DEXI launch file

- Make sure to change camera.launch.xml and set apriltags variable to true

- Look for tag detections ```ros2 topic echo /detections```