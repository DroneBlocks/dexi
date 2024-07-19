# April Tags

cd ~/dexi_ws

rosdep update

sudo apt update

git clone https://github.com/christianrauch/apriltag_ros ~/dexi_ws/src/apriltag_ros

git clone https://github.com/christianrauch/apriltag_msgs ~/dexi_ws/src/apriltag_msgs

rosdep install --from-paths src -y --rosdistro humble --ignore-src

colcon build --packages-select apriltag_msgs



colcon build --packages-select apriltag_ros