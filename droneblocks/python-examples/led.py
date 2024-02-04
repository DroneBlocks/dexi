import rclpy
from led_msgs.srv import SetLEDs
from led_msgs.msg import LEDState

rclpy.init()

node = rclpy.create_node('temp')

set_leds = node.create_client(SetLEDs, 'led/set_leds')

#set_leds([LEDState(index=int(0), r=51, g=51, b=255)])

