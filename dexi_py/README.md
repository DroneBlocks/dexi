ros2 service call /set_led_pixel led_msgs/srv/SetLEDPixel "{index: 0, r: 0, g: 255, b: 0}"

ros2 service call /set_led_color led_msgs/srv/SetLEDColor "{color: 'white'}"