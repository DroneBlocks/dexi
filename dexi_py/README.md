ros2 service call /set_led_pixel dexi_interfaces/srv/SetLEDPixel "{index: 0, r: 0, g: 255, b: 0}"

ros2 service call /set_led_color dexi_interfaces/srv/SetLEDColor "{color: 'white'}"